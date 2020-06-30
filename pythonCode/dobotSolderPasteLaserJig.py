#!/usr/bin/env python
# coding: utf-8


import http.server as SimpleHTTPServer
import socketserver as SocketServer
from urllib.parse import urlparse
from urllib.parse import parse_qs
import logging
import sys
import threading

from serial.tools import list_ports
import os,sys,inspect
from pydobot import Dobot
import time
import psutil
from glob import glob
from pydobot.enums.EIOMode import EIOMode

from LaserJig import LaserJig
from PCBMap import PCBMap

PORT = 8000

webRespDict = {}

class GetHandler(
                 SimpleHTTPServer.SimpleHTTPRequestHandler
                 ):
    
    def do_GET(self):
        parsedPath = urlparse(self.path)
        if parsedPath.path in webRespDict:
            if ('img' in parsedPath.path):
                respContent = webRespDict[parsedPath.path]
                self.protocol_version='HTTP/1.1'
                self.send_response(200, 'OK')
                self.send_header('Content-type', 'image/jpeg')
                self.end_headers()
                query_components = parse_qs(parsedPath.query)
                functionResp = respContent(query_components)
                self.wfile.write(functionResp)
            else:
                respContent = webRespDict[parsedPath.path]
                self.protocol_version='HTTP/1.1'
                self.send_response(200, 'OK')
                self.send_header('Content-type', 'text/plain')
                self.end_headers()
                if isinstance(respContent, str):
                    self.wfile.write(bytes(respContent, 'UTF-8'))
                elif callable(respContent):
                    query_components = parse_qs(parsedPath.query)
                    functionResp = respContent(query_components)
                    if (type(functionResp) == str):
                        self.wfile.write(bytes(functionResp, 'UTF-8'))
                    else:
                        self.wfile.write(bytes("call "+respContent.__name__, 'UTF-8'))
                else:
                    self.wfile.write(bytes("unknown ", 'UTF-8'))
        else:
            #logging.error(self.headers)
            SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)

Handler = GetHandler
SocketServer.TCPServer.allow_reuse_address = True   #debugging, avoid port in use shortly after restart script
httpd = SocketServer.TCPServer(("", PORT), Handler)


def jogMachine(parameters):
    print("jogMachine")
    print(parameters)
    isJoint = int(parameters["isJoint"][0])
    cmd = int(parameters["cmd"][0])
    device._set_jog_command(isJoint,cmd);
    
def moveInc(parameters):
    print("moveInc")
    print(parameters)
    x = y = z = r = 0
    try:
        x = float(parameters["x"][0])
        y = float(parameters["y"][0])
        z = float(parameters["z"][0])
        r = float(parameters["r"][0])
    except:
        pass
    device.move_inc_to(x,y,z,r,wait=True)
    
def moveEMotor(parameters):
    print("moveEMotor")
    print(parameters)
    index = int(parameters["index"][0])
    isEnabled = int(parameters["isEnabled"][0])
    speed = int(parameters["speed"][0])
    distance = int(parameters["distance"][0])
    device._set_emotor_s(index, isEnabled, speed, distance);

def getPose(parameters):
    device._get_pose()
    return ("pose: x:%03.1f y:%03.1f z:%03.1f r:%03.1f j1:%03.1f j2:%03.1f j3:%03.1f j4:%03.1f" %
            (device.x, device.y, device.z, device.r, device.j1, device.j2, device.j3, device.j4))

def home(parameters):
    print("home")
    print(parameters)
    x = y = z = r = None
    try:
        x = float(parameters["x"][0])
        y = float(parameters["y"][0])
        z = float(parameters["z"][0])
        r = float(parameters["r"][0])
    except:
        pass
    device.home(x,y,z,r)

def accessIOPorts(parameters):
    index = int(parameters["index"][0])
    mode = int(parameters["mode"][0])
    if ("level" in parameters):
        level = int(parameters["level"][0])
    if (mode == 1):
        device.set_io_multiplexing(index,EIOMode.IO_FUNC_DO)
        device.set_io_do(index,level)
        return("IO %d OUTPUT LEVEL %d" % (index,level))
    elif (mode == 0):
        device.set_io_multiplexing(index,EIOMode.IO_FUNC_DUMMY)
        device.set_io_do(index,0)
        return("IO %d INPUT" % (index))

available_ports = []
for port in list_ports.comports():
    if (port.pid == 0x7523 and port.vid == 0x1a86):
        available_ports.append(port.device)

if len(available_ports)==0:
    print("No Dobot found")
    exit()

device = Dobot(port=available_ports[0], verbose=False, skipInit=False)
device._get_device_version()
print(("Dobot device version: %d.%d.%d" % (device.majorVersion, device.minorVersion, device.revision)))
webRespDict["/version"]=("Dobot device version: %d.%d.%d" % (device.majorVersion, device.minorVersion, device.revision))

webRespDict["/jog"]=jogMachine
webRespDict["/emotor"]=moveEMotor
webRespDict["/pose"]=getPose
webRespDict["/home"]=home
webRespDict["/moveinc"]=moveInc
webRespDict["/io"]=accessIOPorts

laserJig=LaserJig.LaserJig(device)

def laserCali(parameters):
    mode = int(parameters["mode"][0])
    if mode == 0:
        try:
            laserJig.calibrated = False
            os.remove("jig_coordinate.txt")
            os.remove("jig_Z_laser.txt")
        except:
            pass
    elif mode == 1:
        #laser do a rough calibration
        laserJig.loadJigCoordinatefromFile()
        if not laserJig.calibrated:
            laserJig.doXYCalibration()
            laserJig.doZCalibrationLaser()  # make sure laser is blocked when this is called
        else:
            laserJig.loadZCalibrationLaserfromFile()
    elif mode == 2:
        mappedX, mappedY = laserJig.getJigMapping(9,9);
        x, y, z, r, j1, j2, j3, j4=device.pose()
        device.move_to(mappedX, mappedY,laserJig.jigZLaser-5,r)     
    elif mode == 3:
        #touch pad sensor do a fine calibration
        laserJig.measureZWithTouchpad(limit = 3) 
    elif mode == 4:
        if laserJig.jigZTouchpad is not None:
            mappedX, mappedY = laserJig.getJigMapping(9,9);
            x, y, z, r, j1, j2, j3, j4=device.pose()
            device.move_to(mappedX, mappedY,laserJig.jigZTouchpad,r) 
        else:
            print("jigZTouchpad is none")
webRespDict["/lasercali"]=laserCali

dobotAsyncJob = False 
def pasteDispenseAsync():
    global dobotAsyncJob
    pcbMap = PCBMap.PCBMap()
    pcbMap.loadJigDataFromFile()
    pcbMap.loadPasteCSV("tiny85Digispark0805.csv",rotation = 180)
    
    padsCount = pcbMap.getPastePadsCount()
    #print(padsCount)
    padsCount = min(padsCount,9999)

    z = laserJig.jigZTouchpad-0.2

    for i in range(padsCount):
        result = pcbMap.getPadMapping(i)
        x,y,step = result
        device.jump_to(x,y,z+5,0,wait=True)
        #do paste
        tractBack = 20
        print("Step:",step)

        device._set_jog_command(0,0);   #or next command will trigger bug, axis will move.
        device._set_emotor_s(0, 1, -1000, step+tractBack,wait=True)
        device._set_jog_command(0,0);
        
        if not dobotAsyncJob:
            break;
        time.sleep(0.7)
        if not dobotAsyncJob:
            break;
        
        device.move_to(x,y,z,0,wait=True)
        device._set_jog_command(0,0)
        time.sleep(0.5)
        device._set_emotor_s(0, 1, 1000, tractBack,wait=True)
        if not dobotAsyncJob:
            break;
        #time.sleep(0.5)
    device.move_inc_to(0,0,30,0,wait=True)
    device._set_jog_command(0,0);

    dobotAsyncJob = False;
    
def startPasteDispense(parameters):
    global dobotAsyncJob
    if (dobotAsyncJob):
        return "Already dispensing"
    dobotAsyncJob = True;    
    threading.Thread(target=pasteDispenseAsync, name='LoopThread').start()
    return "Start dispensing"

def stopPasteDispense(parameters):
    global dobotAsyncJob
    if (dobotAsyncJob):
        dobotAsyncJob = False
        return "Stop dispensing"
    return "Not dispensing"

webRespDict["/pasteDispense"]=startPasteDispense
webRespDict["/stopPasteDispense"]=stopPasteDispense

httpd.serve_forever()

device.close()
