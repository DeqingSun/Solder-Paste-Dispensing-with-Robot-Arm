import serial
import struct
import time
import threading
import warnings

from .message import Message
from .enums.EIOMode import EIOMode
from .enums.PTPMode import PTPMode
from .enums.CommunicationProtocolIDs import CommunicationProtocolIDs
from .enums.ControlValues import ControlValues


class Dobot:

    def __init__(self, port, verbose=False, skipInit=False):
        threading.Thread.__init__(self)

        self._on = True
        self.verbose = verbose
        self.lock = threading.Lock()
        self.ser = serial.Serial(port,
                                 baudrate=115200,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS)
        is_open = self.ser.isOpen()
        if self.verbose:
            print('pydobot: %s open' % self.ser.name if is_open else 'failed to open serial port')
            
        if not skipInit:
            self._set_queued_cmd_start_exec()
            self._set_queued_cmd_clear()
            self.set_ptp_joint_params(200, 200, 200, 200, 200, 200, 200, 200)
            self.set_ptp_coordinate_params(velocity=200, acceleration=200)
            self.set_ptp_jump_params(10, 200)
            self.set_ptp_common_params(velocity=100, acceleration=100)
            self.set_end_effector_parameters(65)
            self._get_pose()

    """
        Gets the current command index
    """
    def _get_queued_cmd_current_index(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_QUEUED_CMD_CURRENT_INDEX
        response = self._send_command(msg)
        idx = struct.unpack_from('L', response.params, 0)[0]
        return idx

    """
        Gets the real-time pose of the Dobot
    """
    def _get_pose(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_POSE
        response = self._send_command(msg)
        self.x = struct.unpack_from('f', response.params, 0)[0]
        self.y = struct.unpack_from('f', response.params, 4)[0]
        self.z = struct.unpack_from('f', response.params, 8)[0]
        self.r = struct.unpack_from('f', response.params, 12)[0]
        self.j1 = struct.unpack_from('f', response.params, 16)[0]
        self.j2 = struct.unpack_from('f', response.params, 20)[0]
        self.j3 = struct.unpack_from('f', response.params, 24)[0]
        self.j4 = struct.unpack_from('f', response.params, 28)[0]

        if self.verbose:
            print("pydobot: x:%03.1f y:%03.1f z:%03.1f r:%03.1f j1:%03.1f j2:%03.1f j3:%03.1f j4:%03.1f" %
                  (self.x, self.y, self.z, self.r, self.j1, self.j2, self.j3, self.j4))
        return response

    def _read_message(self):
        time.sleep(0.015)
        b = self.ser.read_all()
        if len(b) > 0:
            msg = Message(b)
            if self.verbose:
                print('pydobot: <<', msg)
            return msg
        return

    def _send_command(self, msg, wait=False):
        self.lock.acquire()
        self._send_message(msg)
        for i in range(30): #some commands are slower, give them more time
            response = self._read_message()
            if response != None:
                break        
        self.lock.release()

        if not wait:
            return response

        expected_idx = struct.unpack_from('L', response.params, 0)[0]
        if self.verbose:
            print('pydobot: waiting for command', expected_idx)

        while True:
            try:
                current_idx = self._get_queued_cmd_current_index()
            except:
                pass    #at end of home command, there is a huge delay, which means dobot will not respond for 1~2 seconds 

            if current_idx != expected_idx:
                time.sleep(0.1)
                continue

            if self.verbose:
                print('pydobot: command %d executed' % current_idx)
            break

        return response

    def _send_message(self, msg):
        time.sleep(0.1)
        if self.verbose:
            print('pydobot: >>', msg)
        self.ser.write(msg.bytes())
        #print(' '.join(format(x, '02x') for x in msg.bytes()))

    """
        Get Device Version
    """
    def _get_device_version(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_DEVICE_VERSION
        msg.ctrl = ControlValues.ZERO
        response = self._send_command(msg,wait=False)
        self.majorVersion = struct.unpack_from('B', response.params, 0)[0]
        self.minorVersion = struct.unpack_from('B', response.params, 1)[0]
        self.revision = struct.unpack_from('B', response.params, 2)[0]
        if self.verbose:
            print("pydobot device version: %d.%d.%d" %
                  (self.majorVersion, self.minorVersion, self.revision))
        return response

    """
        Get HOME parameters
    """
    def _get_home_parameters(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_HOME_PARAMS
        msg.ctrl = ControlValues.ZERO
        response = self._send_command(msg,wait=False)
        self.homeX = struct.unpack_from('f', response.params, 0)[0]
        self.homeY = struct.unpack_from('f', response.params, 4)[0]
        self.homeZ = struct.unpack_from('f', response.params, 8)[0]
        self.homeR = struct.unpack_from('f', response.params, 12)[0]
        if self.verbose:
            print("pydobot home params: %03.1f, %03.1f, %03.1f, %03.1f" % (self.homeX, self.homeY, self.homeZ, self.homeR))
        return response

    """
        Set HOME parameters
    """
    def _set_home_parameters(self, x, y, z, r):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_HOME_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.extend(bytearray(struct.pack('f', r)))
        return self._send_command(msg)  #this is slower than other commands

    """
        Executes Set Home Command
    """
    def _set_home_cmd(self, wait=False):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_HOME_CMD
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray()
        return self._send_command(msg, wait)

    """
        Executes the CP Command
    """
    def _set_cp_cmd(self, x, y, z):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_CP_CMD
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray(bytes([0x01]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.append(0x00)
        return self._send_command(msg)

    """
        Set end effector parameters
    """
    def _set_end_effector_parameters(self, xBias):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', xBias)))
        return self._send_command(msg)

    """
        Get end effector parameters
    """
    def _get_end_effector_parameters(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_PARAMS
        msg.ctrl = ControlValues.ZERO
        response = self._send_command(msg,wait=False)
        self.endEffectorXBias = struct.unpack_from('f', response.params, 0)[0]
        #self.endEffectorYBias = struct.unpack_from('f', response.params, 4)[0] #no response?
        #self.endEffectorZBias = struct.unpack_from('f', response.params, 8)[0]
        if self.verbose:
            #print("pydobot: endEffector bias X %03.1f, Y %03.1f, Z %03.1f." % (self.endEffectorXBias, self.endEffectorYBias, self.endEffectorZBias))
            print("pydobot: endEffector bias X %03.1f." % (self.endEffectorXBias))
        return response

    """
        Sets the status of the gripper
    """
    def _set_end_effector_gripper(self, enable=False):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_GRIPPER
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        return self._send_command(msg)

    """
        Sets the status of the suction cup
    """
    def _set_end_effector_suction_cup(self, enable=False):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_SUCTION_CUP
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        return self._send_command(msg)

    """
        Get jog joint parameters
    """
    def _get_jog_joint_parameters(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_JOG_JOINT_PARAMS
        msg.ctrl = ControlValues.ZERO
        response = self._send_command(msg,wait=False)
        self.jogJointVelocity = [0]*4
        self.jogJointAcceleration = [0]*4
        self.jogJointVelocity[0] = struct.unpack_from('f', response.params, 0)[0]
        self.jogJointVelocity[1] = struct.unpack_from('f', response.params, 4)[0]
        self.jogJointVelocity[2] = struct.unpack_from('f', response.params, 8)[0]
        self.jogJointVelocity[3] = struct.unpack_from('f', response.params, 12)[0]
        self.jogJointAcceleration[0] = struct.unpack_from('f', response.params, 16)[0]
        self.jogJointAcceleration[1] = struct.unpack_from('f', response.params, 20)[0]
        self.jogJointAcceleration[2] = struct.unpack_from('f', response.params, 24)[0]
        self.jogJointAcceleration[3] = struct.unpack_from('f', response.params, 28)[0]
        if self.verbose:
            print("pydobot: jog joint velocity %03.1f, %03.1f, %03.1f, %03.1f. jog joint acceleration %03.1f, %03.1f, %03.1f, %03.1f" %
                  (self.jogJointVelocity[0], self.jogJointVelocity[1], self.jogJointVelocity[2], self.jogJointVelocity[3],
                   self.jogJointAcceleration[0], self.jogJointAcceleration[1], self.jogJointAcceleration[2], self.jogJointAcceleration[3]))
        return response

    """
        Get jog coordinate parameters
    """
    def _get_jog_coordinate_parameters(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_JOG_COORDINATE_PARAMS
        msg.ctrl = ControlValues.ZERO
        response = self._send_command(msg,wait=False)
        self.jogCoordinateVelocity = [0]*4
        self.jogCoordinateAcceleration = [0]*4
        self.jogCoordinateVelocity[0] = struct.unpack_from('f', response.params, 0)[0]
        self.jogCoordinateVelocity[1] = struct.unpack_from('f', response.params, 4)[0]
        self.jogCoordinateVelocity[2] = struct.unpack_from('f', response.params, 8)[0]
        self.jogCoordinateVelocity[3] = struct.unpack_from('f', response.params, 12)[0]
        self.jogCoordinateAcceleration[0] = struct.unpack_from('f', response.params, 16)[0]
        self.jogCoordinateAcceleration[1] = struct.unpack_from('f', response.params, 20)[0]
        self.jogCoordinateAcceleration[2] = struct.unpack_from('f', response.params, 24)[0]
        self.jogCoordinateAcceleration[3] = struct.unpack_from('f', response.params, 28)[0]
        if self.verbose:
            print("pydobot: jog coordinate velocity %03.1f, %03.1f, %03.1f, %03.1f. jog coordinate acceleration %03.1f, %03.1f, %03.1f, %03.1f" %
                  (self.jogCoordinateVelocity[0], self.jogCoordinateVelocity[1], self.jogCoordinateVelocity[2], self.jogCoordinateVelocity[3],
                   self.jogCoordinateAcceleration[0], self.jogCoordinateAcceleration[1], self.jogCoordinateAcceleration[2], self.jogCoordinateAcceleration[3]))
        return response

    """
        Get jog common parameters
    """
    def _get_jog_common_parameters(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_JOG_COMMON_PARAMS
        msg.ctrl = ControlValues.ZERO
        response = self._send_command(msg,wait=False)
        self.jogVelocityRatio = struct.unpack_from('f', response.params, 0)[0]
        self.jogAccelerationRatio = struct.unpack_from('f', response.params, 4)[0]
        if self.verbose:
            print("pydobot: jog velocityRatio:%03.3f jog accelerationRatio:%03.3f" % (self.jogVelocityRatio, self.jogAccelerationRatio))
        return response
    
    """
        Set jog common parameters
    """
    def _set_jog_common_parameters(self, velocityRatio, accelerationRatio):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_JOG_COMMON_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocityRatio)))
        msg.params.extend(bytearray(struct.pack('f', accelerationRatio)))
        return self._send_command(msg)
    
    """
        Sets jog command
    """
    def _set_jog_command(self, isJoint, cmd, isQueued = True):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_JOG_COMMAND
        if isQueued:
            msg.ctrl = ControlValues.THREE
        else:
            msg.ctrl = ControlValues.TWO
        msg.params = bytearray([])
        msg.params.extend(bytearray([isJoint,cmd]))
        return self._send_command(msg)


    """
        Gets the velocity ratio and the acceleration ratio in PTP mode
    """
    def _get_ptp_joint_params(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JOINT_PARAMS
        msg.ctrl = ControlValues.ZERO
        response = self._send_command(msg,wait=False)
        self.ptpJointVelX = struct.unpack_from('f', response.params, 0)[0]
        self.ptpJointVelY = struct.unpack_from('f', response.params, 4)[0]
        self.ptpJointVelZ = struct.unpack_from('f', response.params, 8)[0]
        self.ptpJointVelR = struct.unpack_from('f', response.params, 12)[0]
        self.ptpJointAccX = struct.unpack_from('f', response.params, 16)[0]
        self.ptpJointAccY = struct.unpack_from('f', response.params, 20)[0]
        self.ptpJointAccZ = struct.unpack_from('f', response.params, 24)[0]
        self.ptpJointAccR = struct.unpack_from('f', response.params, 28)[0]
        if self.verbose:
            print("pydobot: PTP joint vel: %03.1f,%03.1f,%03.1f,%03.1f acc:%03.1f,%03.1f,%03.1f,%03.1f" %
                  (self.ptpJointVelX, self.ptpJointVelY, self.ptpJointVelZ, self.ptpJointVelR,
                   self.ptpJointAccX, self.ptpJointAccY, self.ptpJointAccZ, self.ptpJointAccR))
        return response

    """
        Sets the velocity ratio and the acceleration ratio in PTP mode
    """
    def _set_ptp_joint_params(self, v_x, v_y, v_z, v_r, a_x, a_y, a_z, a_r):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JOINT_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', v_x)))
        msg.params.extend(bytearray(struct.pack('f', v_y)))
        msg.params.extend(bytearray(struct.pack('f', v_z)))
        msg.params.extend(bytearray(struct.pack('f', v_r)))
        msg.params.extend(bytearray(struct.pack('f', a_x)))
        msg.params.extend(bytearray(struct.pack('f', a_y)))
        msg.params.extend(bytearray(struct.pack('f', a_z)))
        msg.params.extend(bytearray(struct.pack('f', a_r)))
        return self._send_command(msg)

    """
        Gets the velocity and acceleration of the Cartesian coordinate axes in PTP mode
    """
    def _get_ptp_coordinate_params(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COORDINATE_PARAMS
        msg.ctrl = ControlValues.ZERO
        response = self._send_command(msg,wait=False)
        self.ptpCoordinateXYZVel = struct.unpack_from('f', response.params, 0)[0]
        self.ptpCoordinateRVel = struct.unpack_from('f', response.params, 4)[0]
        self.ptpCoordinateXYZAcc = struct.unpack_from('f', response.params, 8)[0]
        self.ptpCoordinateRAcc = struct.unpack_from('f', response.params, 12)[0]
        if self.verbose:
            print("pydobot: PTP Coordinate vel: %03.1f,%03.1f acc:%03.1f,%03.1f" %
                  (self.ptpCoordinateXYZVel, self.ptpCoordinateRVel, self.ptpCoordinateXYZAcc, self.ptpCoordinateRAcc))
        return response

    """
        Sets the velocity and acceleration of the Cartesian coordinate axes in PTP mode
    """
    def _set_ptp_coordinate_params(self, velocity, acceleration):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COORDINATE_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)

    """
       Gets the lifting height and the maximum lifting height in JUMP mode
    """
    def _get_ptp_jump_params(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JUMP_PARAMS
        msg.ctrl = ControlValues.ZERO
        response = self._send_command(msg,wait=False)
        self.ptpJump = struct.unpack_from('f', response.params, 0)[0]
        self.ptpLimit = struct.unpack_from('f', response.params, 4)[0]
        if self.verbose:
            print("pydobot: PTP Jump: %03.1f Limit:%03.1f" % (self.ptpJump, self.ptpLimit))
        return response

    """
       Sets the lifting height and the maximum lifting height in JUMP mode
    """
    def _set_ptp_jump_params(self, jump, limit):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JUMP_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', jump)))
        msg.params.extend(bytearray(struct.pack('f', limit)))
        return self._send_command(msg)

    """
        Gets the velocity ratio, acceleration ratio in PTP mode
    """
    def _get_ptp_common_params(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COMMON_PARAMS
        msg.ctrl = ControlValues.ZERO
        response = self._send_command(msg,wait=False)
        self.ptpCommonVelocityRatio = struct.unpack_from('f', response.params, 0)[0]
        self.ptpCommonAccelerationRatio = struct.unpack_from('f', response.params, 4)[0]
        if self.verbose:
            print("pydobot: PTP common velocity ratio: %03.1f acceleration ratio:%03.1f" % (self.ptpCommonVelocityRatio, self.ptpCommonAccelerationRatio))
        return response

    """
        Sets the velocity ratio, acceleration ratio in PTP mode
    """
    def _set_ptp_common_params(self, velocity, acceleration):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COMMON_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)

    """
        Executes PTP command
    """
    def _set_ptp_cmd(self, x, y, z, r, mode, wait):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_PTP_CMD
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray([mode]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.extend(bytearray(struct.pack('f', r)))
        return self._send_command(msg, wait)

    """
        Gets arc parameters
    """
    def _get_arc_params(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_ARC_PARAMS
        msg.ctrl = ControlValues.ZERO
        response = self._send_command(msg,wait=False)
        self.arcXYZVel = struct.unpack_from('f', response.params, 0)[0]
        self.arcRVel = struct.unpack_from('f', response.params, 4)[0]
        self.arcXYZAcc = struct.unpack_from('f', response.params, 8)[0]
        self.arcRAcc = struct.unpack_from('f', response.params, 12)[0]
        if self.verbose:
            print("pydobot: Arc params vel: %03.1f,%03.1f acc:%03.1f,%03.1f" %
                  (self.arcXYZVel, self.arcRVel, self.arcXYZAcc, self.arcRAcc))
        return response

    """
        Sets the velocity and acceleration of arc parameters
    """
    def _set_arc_params(self, velocity, acceleration):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_ARC_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)

    """
        Sets arc command
    """
    def _set_arc_command(self, x, y, z, r, x1, y1, z1, r1, wait = False):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_ARC_COMMAND
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.extend(bytearray(struct.pack('f', r)))
        msg.params.extend(bytearray(struct.pack('f', x1)))
        msg.params.extend(bytearray(struct.pack('f', y1)))
        msg.params.extend(bytearray(struct.pack('f', z1)))
        msg.params.extend(bytearray(struct.pack('f', r1)))
        return self._send_command(msg, wait)

    """
        Gets the velocity ratio, acceleration ratio in ARC mode
    """
    def _get_arc_common_params(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_ARC_COMMON_PARAMS_UNDOCUMENTED
        msg.ctrl = ControlValues.ZERO
        response = self._send_command(msg,wait=False)
        self.arcCommonVelocityRatio = struct.unpack_from('f', response.params, 0)[0]
        self.arcCommonAccelerationRatio = struct.unpack_from('f', response.params, 4)[0]
        if self.verbose:
            print("pydobot: ARC common velocity ratio: %03.1f acceleration ratio:%03.1f" % (self.arcCommonVelocityRatio, self.arcCommonAccelerationRatio))
        return response

    """
        Sets the velocity ratio, acceleration ratio in ARC mode
    """
    def _set_arc_common_params(self, velocity, acceleration):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_ARC_COMMON_PARAMS_UNDOCUMENTED
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)

    
    """
        Gets EIO Multiplexing
    """    
    def _get_io_multiplexing(self,addr):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IOMULTIPLEXING
        msg.ctrl = ControlValues.ZERO
        msg.params = bytearray([addr,0])  #different from manual, add ano
        response = self._send_command(msg,wait=False)
        return response

    """
        Sets EIO Multiplexing
    """    
    def _set_io_multiplexing(self,addr,multiplexing):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IOMULTIPLEXING
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([addr,multiplexing])  
        response = self._send_command(msg,wait=False)
        return response    
    
    """
        Gets EIO Output Level
    """    
    def _get_io_do(self,addr):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IODO
        msg.ctrl = ControlValues.ZERO
        msg.params = bytearray([addr,0])  #different from manual, add ano
        response = self._send_command(msg,wait=False)
        return response
    
    """
        Sets EIO Output Level
    """    
    def _set_io_do(self,addr,do):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_IODO
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([addr,do]) 
        response = self._send_command(msg,wait=False)
        return response       
    
    """
        Set External Motor movement
    """
    def _set_emotor_s(self, index, isEnabled, speed, distance, wait = False):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_EMOTOR_S
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray([index]))
        msg.params.extend(bytearray(struct.pack('B', isEnabled)))
        msg.params.extend(bytearray(struct.pack('i', speed)))
        msg.params.extend(bytearray(struct.pack('i', distance)))
        return self._send_command(msg, wait)

    """
        Clears command queue
    """
    def _set_queued_cmd_clear(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_CLEAR
        msg.ctrl = ControlValues.ONE
        return self._send_command(msg)

    """
        Start command
    """
    def _set_queued_cmd_start_exec(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_START_EXEC
        msg.ctrl = ControlValues.ONE
        return self._send_command(msg)

    """
        Stop command
    """
    def _set_queued_cmd_stop_exec(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_STOP_EXEC
        msg.ctrl = ControlValues.ONE
        return self._send_command(msg)

    def close(self):
        self._on = False
        self.lock.acquire()
        self.ser.close()
        if self.verbose:
            print('pydobot: %s closed' % self.ser.name)
        self.lock.release()

    def go(self, x, y, z, r=0.):
        warnings.warn('go() is deprecated, use move_to() instead')
        self.move_to(x, y, z, r)

    def move_to(self, x, y, z, r, wait=False):
        return self._set_ptp_cmd(x, y, z, r, PTPMode.MOVL_XYZ, wait)
        
    def move_inc_to(self, x, y, z, r, wait=False):
        return self._set_ptp_cmd(x, y, z, r, PTPMode.MOVJ_XYZ_INC, wait)

    def jump_to(self, x, y, z, r, wait=False):
        return self._set_ptp_cmd(x, y, z, r, PTPMode.JUMP_XYZ, wait)
    
    def arc_via_to(self, x, y, z, r, x1, y1, z1, r1, wait = False):
        return self._set_arc_command(x, y, z, r, x1, y1, z1, r1, wait)

    def suck(self, enable):
        self._set_end_effector_suction_cup(enable)

    def grip(self, enable):
        self._set_end_effector_gripper(enable)

    def speed(self, velocity=100., acceleration=100.):
        self.set_ptp_common_params(velocity, acceleration)
        self.set_ptp_coordinate_params(velocity, acceleration)

    def pose(self):
        response = self._get_pose()
        x = struct.unpack_from('f', response.params, 0)[0]
        y = struct.unpack_from('f', response.params, 4)[0]
        z = struct.unpack_from('f', response.params, 8)[0]
        r = struct.unpack_from('f', response.params, 12)[0]
        j1 = struct.unpack_from('f', response.params, 16)[0]
        j2 = struct.unpack_from('f', response.params, 20)[0]
        j3 = struct.unpack_from('f', response.params, 24)[0]
        j4 = struct.unpack_from('f', response.params, 28)[0]
        return x, y, z, r, j1, j2, j3, j4

    def home(self, x=None, y=None, z=None, r=None):
        if (x!=None and y!=None and z!=None and r!=None):
            self._get_home_parameters()
            if (abs(x-self.homeX)>0.01 or abs(y-self.homeY)>0.01 or abs(z-self.homeZ)>0.01 or abs(r-self.homeR)>0.01):  #float number never equal....
                self._set_home_parameters(x, y, z, r);
                self._get_home_parameters()
        self._set_home_cmd(True)
    
    def set_ptp_joint_params(self, v_x, v_y, v_z, v_r, a_x, a_y, a_z, a_r):
        self._get_ptp_joint_params()
        if (abs(v_x-self.ptpJointVelX)>0.01 or abs(v_y-self.ptpJointVelY)>0.01 or
            abs(v_z-self.ptpJointVelZ)>0.01 or abs(v_r-self.ptpJointVelR)>0.01 or
            abs(a_x-self.ptpJointAccX)>0.01 or abs(a_y-self.ptpJointAccY)>0.01 or
            abs(a_z-self.ptpJointAccZ)>0.01 or abs(a_r-self.ptpJointAccR)>0.01):  #float number never equal....
            self._set_ptp_joint_params(v_x, v_y, v_z, v_r, a_x, a_y, a_z, a_r)
            self._get_ptp_joint_params()
            
    def set_end_effector_parameters(self, xBias):
        self._get_end_effector_parameters()
        if (abs(xBias-self.endEffectorXBias)>0.01):  #float number never equal....
            self.set_end_effector_parameters(xBias) 
            self._get_end_effector_parameters()
        
    def set_ptp_coordinate_params(self, velocity, acceleration):
        self._get_ptp_coordinate_params()
        if (abs(velocity-self.ptpCoordinateXYZVel)>0.01 or abs(acceleration-self.ptpCoordinateXYZAcc)>0.01):  #float number never equal....
            self._set_ptp_coordinate_params(velocity, acceleration)
            self._get_ptp_coordinate_params()

    def set_ptp_jump_params(self, jump, limit):
        self._get_ptp_jump_params()
        if (abs(jump-self.ptpJump)>0.01 or abs(limit-self.ptpLimit)>0.01):  #float number never equal....
            self._set_ptp_jump_params(jump, limit)
            self._get_ptp_jump_params()

    def set_ptp_common_params(self, velocity, acceleration):
        self._get_ptp_common_params()
        if (abs(velocity-self.ptpCommonVelocityRatio)>0.01 or abs(acceleration-self.ptpCommonAccelerationRatio)>0.01):  #float number never equal....
            self._set_ptp_common_params(velocity, acceleration)
            self._get_ptp_common_params()

    def set_arc_params(self, velocity, acceleration):
        self._get_arc_params()
        if (abs(velocity-self.arcXYZVel)>0.01 or abs(acceleration-self.arcXYZAcc)>0.01):  #float number never equal....
            self._set_arc_params(velocity, acceleration)
            self._get_arc_params()
            
    def set_arc_speed(self, velocity, acceleration):
        #undocumented, reverse engineered from Teaching & Playback in Dobot Studio
        self._set_ptp_common_params(velocity, acceleration)
        self._set_arc_common_params(velocity, acceleration)
        
    def get_io_multiplexing(self,addr):
        msg = Message()
        response = self._get_io_multiplexing(addr)
        multiplexing = struct.unpack_from('B', response.params, 1)[0]
        return multiplexing
    
    def set_io_multiplexing(self,addr,multiplexing):
        return self._set_io_multiplexing(addr,multiplexing)

    def get_io_do(self,addr):
        msg = Message()
        response = self._get_io_do(addr)
        do = struct.unpack_from('B', response.params, 1)[0]
        return do
    
    def set_io_do(self,addr,do):
        return self._set_io_do(addr,do)    
