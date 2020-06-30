import math,time, struct
from datetime import datetime
import serial

class LaserJig:
    def __init__(self,_dobot):
        self.dobot = _dobot
        
        available_ports = []
        for port in serial.tools.list_ports.comports():
            if (port.pid == 0x8036 and port.vid == 0x2341):
                available_ports.append(port.device)
        
        if len(available_ports)==0:
            print("No Arduino found")
            exit()
        self.ser = serial.Serial(None,
                         baudrate=115200,
                         parity=serial.PARITY_NONE,
                         stopbits=serial.STOPBITS_ONE,
                         bytesize=serial.EIGHTBITS,timeout = 0.01)
        self.ser.port = available_ports[0]
        
        self.calibrated = False
        self.sensorXIdle = None
        self.sensorYIdle = None
        self.jigZTouchpad = None
        
        #print("LaserJig init1")
        
        
    def dobotArcMovePrepare(self,x,y,z,r,radius,startAngle,travelAngle,wait = False):
        #self.dobot.move_to(x,y,z,r, wait=_wait)  #may not be needed in future
        endAngle = startAngle + travelAngle
        midAngle = startAngle + travelAngle*0.5
        startX = x+radius*math.cos(startAngle)
        startY = y+radius*math.sin(startAngle)
        self.midX = x+radius*math.cos(midAngle)
        self.midY = y+radius*math.sin(midAngle)        
        self.endX = x+radius*math.cos(endAngle)
        self.endY = y+radius*math.sin(endAngle)
        self.z = z
        self.r = r
        self.dobot.move_to(startX,startY,z,r, wait)

    def dobotArcMove(self,wait = False):
        response = self.dobot.arc_via_to(self.midX,self.midY,self.z,self.r,self.endX,self.endY,self.z,self.r, wait)
        expected_idx = struct.unpack_from('L', response.params, 0)[0]
        return expected_idx
    
    def dobotArcGetSensorData(self,x,y,z,r,radius,startAngle,travelAngle = math.pi*2*359/360):
        self.ser.open() #open port only when we need it. This helps solve the failure to read in second try.
        self.dobotArcMovePrepare(x,y,z,r,radius,startAngle,travelAngle, wait = True)
        self.ser.reset_input_buffer()
        self.ser.write(b'L\n');
        expected_idx = self.dobotArcMove()
        current_idx = 0
        self.ser.read_all()
        recvBuffer = ""
        while True:
            try:
                current_idx = self.dobot._get_queued_cmd_current_index()
            except:
                pass    #at end of home command, there is a huge delay, which means dobot will not respond for 1~2 seconds 
            if current_idx == expected_idx:
                break
            bytesData = self.ser.read_all()
            recvBuffer = recvBuffer + bytesData.decode("ascii", errors="ignore")
        firstNewLineIndex = recvBuffer.index('\r\n')
        lastNewLineIndex = recvBuffer.rindex('\r\n')
        dataLines = recvBuffer[firstNewLineIndex+2:lastNewLineIndex].split('\r\n')
        dataX = []
        dataY = []
        for line in dataLines:
            valuesStr = line.split(',')
            if len(valuesStr)==2:
                dataX.append(int(valuesStr[0]))
                dataY.append(int(valuesStr[1]))
        self.ser.write(b'S\n');
        self.ser.close()
        return [dataX,dataY]
        
        
    def measureLinePositionInterval(self, x, y, z, r, lineAngle,radius,sensorIndex):
        #do 2 circles.
        sensorDatas = self.dobotArcGetSensorData(x,y,z,r,radius,startAngle = lineAngle + math.pi/2 )
        dataArr1 = sensorDatas[sensorIndex]
        peaks1 = self.findPeaks(dataArr1) 
        sensorDatas = self.dobotArcGetSensorData(x,y,z,r,radius,startAngle = lineAngle + math.pi/2 + math.pi )
        dataArr2 = sensorDatas[sensorIndex]
        peaks2 = self.findPeaks(dataArr2)
        
        if (len(peaks1)==2 and len(peaks2)==2):
            interval1=peaks1[1]-peaks1[0]
            interval2=peaks2[1]-peaks2[0]
            #print("measureLinePositionInterval",interval1,interval2)
            return [interval1,interval2]
        else:
            print("not all 2 while test the line.!!!!",peaks1,peaks2)
            if (len(peaks1)!=2):
                print("dataArr1",len(dataArr1),min(dataArr1),max(dataArr1))
            if (len(peaks2)!=2):
                print("dataArr2",len(dataArr2),min(dataArr2),max(dataArr2))
            return None
        
    def estimateLinePosition(self, x, y, z, r, lineAngle,radius,sensorIndex): #x,y is center of estimation(test circle)
        #use 2 circles to calculate line position
        result = self.measureLinePositionInterval(x, y, z, r, lineAngle,radius,sensorIndex)
        if (result == None):
            print("something wrong in measureLinePositionInterval, maybe not trigger 2 times each?")
            return None
        diff = result[1]-result[0];
        if (abs(diff)<3):
            pass
        else:
            halfRatioAngle = math.pi*result[0]/(result[0]+result[1])
            step = radius*math.cos(halfRatioAngle)
            xInc = step*math.cos(lineAngle + math.pi/2 + math.pi)
            yInc = step*math.sin(lineAngle + math.pi/2 + math.pi)
            x += xInc
            y += yInc
        return [x,y]
    
    def getToLineCenter(self, x, y, z, r, lineAngle,radius,skipRough = False): #x,y is center of estimation(test circle)
        if not skipRough:
            #do one rough and one fine (2*2 circle)
            self.dobot.set_arc_speed(50,50); #rough
            result1 = self.estimateLinePosition(x,y, z, r,lineAngle,radius,sensorIndex=0)
            if (result1 == None):
                print("something wrong in rough estimateLinePosition")
                return None
            self.dobot.set_ptp_common_params(velocity=100, acceleration=50)  #restore move speed
            #self.dobot.move_to(result1[0], result1[1], z, r, True)
            #time.sleep(0.5)
        else:
            result1 = [x,y]
        self.dobot.set_arc_speed(10,50); #fine
        result2 = self.estimateLinePosition(result1[0], result1[1], z, r,lineAngle,radius,sensorIndex=0)
        if (result2 == None):
            print("something wrong in fine estimateLinePosition")
            return None
        self.dobot.set_ptp_common_params(velocity=100, acceleration=50)  #restore move speed
        #self.dobot.move_to(result2[0], result2[1], z, r, True)
        return [result2[0], result2[1]]

    def getCrossCenterStartAngle(self, x, y, z, r , radius, startAngle):
        while True:
            sensorDatas = self.dobotArcGetSensorData(x,y,z,r,radius,startAngle-math.pi/8,math.pi/4 )
            if ( (max(sensorDatas[0])<(self.sensorXIdle+100)) and (max(sensorDatas[1])<(self.sensorYIdle+100)) ):
                #seems no beam is blocked around start angle
                break
            else:
                startAngle+=math.pi/8
        return startAngle
    
    def getCrossCenterArc3Interval(self, x, y, z, r , radius, startAngle):
        startBeam = 0
        sensorDatas = self.dobotArcGetSensorData(x,y,z,r,radius,startAngle)
        peakX = self.findPeaks(sensorDatas[0])
        peakY = self.findPeaks(sensorDatas[1])
        if (len(peakX)!=2 or len(peakY)!=2):
            print("In getCrossCenterAndRoughRotation, not getting 2 blocks from each beam, try a better start point")
            #print(sensorDatas,peakX,peakY) #for debug
            return None
        if (peakX[0]<peakY[0] and peakY[0]<peakX[1] and peakX[1]<peakY[1]):
            #print("X triggered first, start angle within 2, 4 Quadrant or jig space")
            startBeam = 0
        elif (peakY[0]<peakX[0] and peakX[0]<peakY[1] and peakY[1]<peakX[1]):
            #print("Y triggered first, start angle within 1, 3 Quadrant or jig space")
            startBeam = 1
        else:
            print("In getCrossCenterAndRoughRotation, beams not triggered alternatively, try a better start point")
            print(peakX,peakY)
            return None        
        #print(peakX,peakY)
        allPeaks = sorted(peakX+peakY)
        interval = [allPeaks[1]-allPeaks[0],allPeaks[2]-allPeaks[1],allPeaks[3]-allPeaks[2]]
        return [startBeam, interval]
    
    def wrapRadianPMPi(self, angle):
        while (angle >= math.pi):
            angle -= math.pi*2;
        while (angle < -math.pi):
            angle += math.pi*2;
        return angle
    
    def meanAngle(self, angles):
        x = 0
        y = 0
        for angle in angles:
            x += math.cos(angle)
            y += math.sin(angle)
        return math.atan2(y, x)
    
    def closest(self, lst, K): 
        return lst[min(range(len(lst)), key = lambda i: abs(lst[i]-K))]
    
    def closestAngle(self, lst, K): 
        return lst[min(range(len(lst)), key = lambda i: abs(self.wrapRadianPMPi(lst[i]-K)))]
    
    def interpolatePoseDataWithNoneElements(self,inputData):
        elementWithValue = []
        for i in range(len(inputData)):
                if inputData[i]!=None:
                    elementWithValue.append(i)
        slopeBegin = (inputData[elementWithValue[1]]-inputData[elementWithValue[0]])/(elementWithValue[1]-elementWithValue[0])
        for i in range(elementWithValue[0]):
            inputData[i] = inputData[elementWithValue[0]]-slopeBegin*(elementWithValue[0]-i)
        #there is value in the end, no need to interpolate tail
        for j in range(len(elementWithValue)-1):
            slope = (inputData[elementWithValue[j+1]]-inputData[elementWithValue[j]])/(elementWithValue[j+1]-elementWithValue[j])
            for i in range(elementWithValue[j]+1,elementWithValue[j+1]):
                inputData[i] = inputData[elementWithValue[j]]-slope*(elementWithValue[j]-i)
        return inputData
    
    def getCrossCenterSlowSearch(self, x, y, z, r , radius, startAngle,travelAngle = math.pi*2*359/360, arcSpeed = 5):
        self.ser.open() #open port only when we need it. This helps solve the failure to read in second try.
        self.dobot.set_ptp_common_params(velocity=100, acceleration=50) 
        self.dobotArcMovePrepare(x,y,z,r,radius,startAngle,travelAngle, wait = True)
        self.ser.reset_input_buffer()
        self.ser.write(b'L\n');
        self.dobot.set_arc_speed(arcSpeed,100);
        expected_idx = self.dobotArcMove()
        current_idx = 0
        self.ser.read_all()
        recvBuffer = ""
        sensorDataX = []
        sensorDataY = []
        poseAngle = []
        
        cX, cY, cZ, cR, j1, j2, j3, j4=self.dobot.pose()
        pX = 0
        pY = 0
        while True:
            cX, cY, cZ, cR, j1, j2, j3, j4=self.dobot.pose()
            dist = abs(cX-pX)+abs(cY-pY)
            pX=cX
            pY=cY
            #print(cX, cY, dist)
            decodedData = self.ser.read_all().decode("ascii", errors="ignore")
            dataLines = (recvBuffer+decodedData).split('\r\n')
            recvBuffer = dataLines.pop()
            for line in dataLines:
                valuesStr = line.split(',')
                if len(valuesStr)==2:
                    try:
                        t1 = int(valuesStr[0])
                        t2 = int(valuesStr[1])
                        sensorDataX.append(t1)
                        sensorDataY.append(t2)
                        poseAngle.append(None)
                    except:
                        print("some non-critial error in parsing:"+line)
            poseAngle[-1] = math.atan2(cY-y,cX-x)
            
            if (dist)<0.1:
                break
        #print("DOBOT STOP MOVING")
        while True:
             try:
                 current_idx = self.dobot._get_queued_cmd_current_index()
             except:
                 pass    #at end of home command, there is a huge delay, which means dobot will not respond for 1~2 seconds 
             if current_idx == expected_idx:
                 break
        #print("Command completed")
        
        #1st data may be corrupted
        sensorDataX.pop(0)  
        sensorDataY.pop(0)
        poseAngle.pop(0)

        self.ser.write(b'S\n');
        self.ser.close()
        
        #interpolate poseAngle, because pose is slow
        
        roundVal = 0
        prevAngle = 0
        if (travelAngle>0):
            for i in range(len(poseAngle)):
                if poseAngle[i]!=None:
                    #print(poseAngle[i],poseAngle[i]+roundVal)
                    if ((poseAngle[i]+roundVal)<prevAngle):
                        roundVal += math.pi*2
                    poseAngle[i] += roundVal
                    prevAngle = poseAngle[i]
        else:
            for i in range(len(poseAngle)):
                if poseAngle[i]!=None:
                    #print(poseAngle[i],poseAngle[i]+roundVal)
                    if ((poseAngle[i]-roundVal)>prevAngle):
                        roundVal -= math.pi*2
                    poseAngle[i] -= roundVal
                    prevAngle = poseAngle[i]
                #print(poseAngle[i])
        
        #print(poseAngle)
        poseAngle = self.interpolatePoseDataWithNoneElements(poseAngle)
        peakXY = [self.findPeaks(sensorDataX),self.findPeaks(sensorDataY)]
        if (len(peakXY[0])!=2 or len(peakXY[1])!=2):
            print("In getCrossCenterSlowSearch, not getting 2 blocks from each beam, try a better start point")
            #print(sensorDatas,peakXY[0],peakXY[1]) #for debug
            return None
        if (peakXY[0][0]<peakXY[1][0] and peakXY[1][0]<peakXY[0][1] and peakXY[0][1]<peakXY[1][1]):
            #print("X triggered first, start angle within 2, 4 Quadrant or jig space")
            startBeam = 0
        elif (peakXY[1][0]<peakXY[0][0] and peakXY[0][0]<peakXY[1][1] and peakXY[1][1]<peakXY[0][1]):
            #print("Y triggered first, start angle within 1, 3 Quadrant or jig space")
            startBeam = 1
        else:
            print("In getCrossCenterSlowSearch, beams not triggered alternatively, try a better start point")
            print(peakXY[0],peakXY[1])
            return None        
        #print(peakXY[0],peakXY[1])
        peakXYLoc = [[],[]]
 
        for j in range(2):
            for i in peakXY[j]:
                angle = poseAngle[i]
                locX = x + radius*math.cos(angle)
                locY = y + radius*math.sin(angle)
                peakXYLoc[j].append( [locX,locY] )
                #self.dobot.move_to(locX, locY, z , r, True)
                #print(i,angle)
                #time.sleep(1)
        
        sumX = 0
        sumY = 0
        
        for j in range(2):
            for i in peakXYLoc[j]:
                sumX += i[0]
                sumY += i[1]
        
        centerX = sumX/4
        centerY = sumY/4
        
        #limit range
        xDir = [peakXYLoc[0][0][0]-peakXYLoc[0][1][0],peakXYLoc[0][0][1]-peakXYLoc[0][1][1]]
        yDir = [peakXYLoc[1][0][0]-peakXYLoc[1][1][0],peakXYLoc[1][0][1]-peakXYLoc[1][1][1]]
        #print(xDir,yDir)    
        angleX = math.atan2(xDir[1],xDir[0])
        angleY = math.atan2(yDir[1],yDir[0])
        
        if abs(self.wrapRadianPMPi(angleY-math.pi*0.5-angleX))<(math.pi/10):
            angle = self.meanAngle([angleY-math.pi*0.5,angleX])
        elif abs(self.wrapRadianPMPi(angleY+math.pi*0.5-angleX))<(math.pi/10):
            angle = self.meanAngle([angleY+math.pi*0.5,angleX])
        else:
            print("the calulated XY beam directions are not perpendicular:"+str(angleX)+","+str(angleY))
            return None
            
        if (angle>math.pi*.4 or angle<math.pi*-0.9):
            angle += math.pi #use outward direction
        elif (angle>math.pi*-0.5 and angle<math.pi*0.1):
            angle = angle
        else:
            print("Please put opening of jig outward")
            return None

        #using sensorData to calculate again, we are using ratio of intervals between peaks, to calculate x,y(center of test circle) in jig coordinates,
        #then we use angle to calulate jig zero in robot coordinates
        allPeaks = sorted(peakXY[0]+peakXY[1])
        #get movementSpeed
        speedTP1 = int(0.25*len(poseAngle))
        speedTP2 = int(0.75*len(poseAngle))
        arcSpeed = ((poseAngle[speedTP2]-poseAngle[speedTP1])/(speedTP2-speedTP1))
        fullCircleTime = int(round(math.pi * 2 / arcSpeed))
        interval = [allPeaks[1]-allPeaks[0],allPeaks[2]-allPeaks[1],allPeaks[3]-allPeaks[2]]
        interval.append(fullCircleTime-sum(interval))
        
        xPosDirectionIndex = poseAngle.index(self.closestAngle( poseAngle, angle))
        xPosDirectionPeakIndex = allPeaks.index(self.closest( allPeaks, xPosDirectionIndex))    #find which interval is 1st quadrant in jig 
        #print(xPosDirectionPeakIndex)
        
        #print(fullCircleTime)
        #print(allPeaks)
        #print(interval)
        for i in range(xPosDirectionPeakIndex):
            interval.append(interval.pop(0))
        #print(interval)

        yOffsetInJigCor = radius*math.cos(math.pi*(interval[3]+interval[2])/fullCircleTime)
        xOffsetInJigCor = radius*math.cos(math.pi*(interval[1]+interval[2])/fullCircleTime)
        offsetInJigCorLen = math.sqrt(yOffsetInJigCor*yOffsetInJigCor+xOffsetInJigCor*xOffsetInJigCor)
        offsetInJigCorAngle = math.atan2(yOffsetInJigCor,xOffsetInJigCor)
        
        offsetInRobotCorAngle = offsetInJigCorAngle + angle
        
        jigCenterX = x-offsetInJigCorLen*math.cos(offsetInRobotCorAngle)
        jigCenterY = y-offsetInJigCorLen*math.sin(offsetInRobotCorAngle)

        #print("x,y",x,y)
        #print("xOffsetInJigCor,yOffsetInJigCor",xOffsetInJigCor,yOffsetInJigCor)
        #print("offsetInJigCorLen,offsetInJigCorAngle",offsetInJigCorLen,offsetInJigCorAngle)
        #print("offsetInRobotCorAngle",offsetInRobotCorAngle)
        #print("use cor",centerX, centerY, angle)
        #print("jigCenter",jigCenterX,jigCenterY)
        #self.dobot.move_to(jigCenterX, jigCenterY, z , r, True)
        #time.sleep(5)

        #self.dobot.move_to(centerX, centerY, z , r, True)
        #time.sleep(5)
        #self.dobot.move_to(centerX+radius*2*math.cos(angle), centerY+radius*2*math.sin(angle), z , r, True)
        #time.sleep(5)
        #print(poseAngle)
        
        #return [centerX, centerY, angle]
        return [jigCenterX, jigCenterY, angle]

    def getCrossCenterAndRoughRotation(self, x, y, z, r , radius):
        self.dobot.set_ptp_common_params(velocity=100, acceleration=50)  #restore move speed
        self.dobot.move_to(x, y, z + 15, r, True)   #leave laser to get sensor average values
        self.getSensorAvgValues(0.3)
        self.dobot.move_to(x, y, z , r, True)
        
        
        
        
        
        self.dobot.set_arc_speed(50,100); 
        #test if there is laser around this start angle
        startAngle = self.getCrossCenterStartAngle( x, y, z, r , radius, math.pi*5/4)    #initial search angle
        #now we are sure we can start at start angle, and we should get 2 blocks on each beam!
        result = self.getCrossCenterSlowSearch(x, y, z, r , radius, startAngle, math.pi*2*359/360 , arcSpeed = 20) #rough
        if (result == None):
             return None
        centerXRough, centerYRough, angleRough = result
        #print("DEBUG.result",result)
        
        resultFine1 = self.getCrossCenterSlowSearch(centerXRough, centerYRough, z, r , radius, angleRough-math.pi*0.25, math.pi*2*359/360, arcSpeed = 10)
        if (resultFine1 == None):
             return None
        #print("DEBUG.resultFine1",resultFine1)
        resultFine2 = self.getCrossCenterSlowSearch(centerXRough, centerYRough, z, r , radius, angleRough+math.pi*0.75, -math.pi*2*359/360, arcSpeed = 10)
        if (resultFine2 == None):
             return None
        #print("DEBUG.resultFine2",resultFine2)
            
        centerFineX = (resultFine1[0])  #seems the reveresed is worse in accuracy
        centerFineY = (resultFine1[1])
        
        angleFine = self.meanAngle([resultFine1[2],resultFine2[2]]) #seems the reveresed is helpful in angle calculation
        
        # self.dobot.move_to(centerFineX, centerFineY, z , r, True)
        # time.sleep(5)
        # self.dobot.move_to(centerFineX+radius*2*math.cos(angleRough), centerFineY+radius*2*math.sin(angleRough), z , r, True)
        # time.sleep(5)
        # self.dobot.move_to(centerFineX+radius*2*math.cos(resultFine1[2]), centerFineY+radius*2*math.sin(resultFine1[2]), z , r, True)
        # time.sleep(5)
        # self.dobot.move_to(centerFineX+radius*2*math.cos(angleFine), centerFineY+radius*2*math.sin(angleFine), z , r, True)
        # time.sleep(5)
        
        return [centerFineX, centerFineY, angleFine]
        
    def saveJigCoordinateToFile(self):
        f = open("jig_coordinate.txt", "w")
        content = "%03.3f,%03.3f,%03.3f" % (self.jigX, self.jigY, self.jigAngle)
        f.write(content)
        f.close()
        
    def saveJigZLaserToFile(self):
        f = open("jig_Z_laser.txt", "w")
        content = "%03.3f" % (self.jigZLaser)
        f.write(content)
        f.close()
    
    def loadZCalibrationLaserfromFile(self):
        try:
            f = open("jig_Z_laser.txt", "r")
            lines = f.read().split('\n')
            f.close()
            data = lines[0].strip().split(',')
            self.jigZLaser = float(data[0])
            print("loaded jigZLaser: %03.3f" % (self.jigZLaser))
        except Exception as e:
            print("something wrong in loadZCalibrationLaserfromFile")
            print(e)
    
    def loadJigCoordinatefromFile(self):
        try:
            f = open("jig_coordinate.txt", "r")
            lines = f.read().split('\n')
            f.close()
            data = lines[0].strip().split(',')
            self.jigX = float(data[0])
            self.jigY = float(data[1])
            self.jigAngle = float(data[2])
            self.calibrated = True
            print("loaded jigX: %03.3f, jigY: %03.3f, jigAngle: %03.3f" % (self.jigX, self.jigY, self.jigAngle))
        except Exception as e:
            print("something wrong in loadJigCoordinatefromFile")
            print(e)

    def getJigPositonRotation(self):
        x, y, z, r, j1, j2, j3, j4=self.dobot.pose()
        testCenterRadius = 12
        testLineRadius = 10
        result = self.getCrossCenterAndRoughRotation(x,y,z,r,testCenterRadius);
        if (result == None):
             return None
        centerFineX, centerFineY, angleFine = result
        self.dobot.set_ptp_coordinate_params(velocity=200, acceleration=200)
        self.dobot.set_ptp_jump_params(10, 200)
        self.dobot.set_ptp_common_params(velocity=100, acceleration=50)
        self.dobot.move_to(centerFineX, centerFineY, z , r, True)
        time.sleep(0.5)
        lineTestDistance = 80
        lineTestX = centerFineX+lineTestDistance*math.cos(angleFine)
        lineTestY = centerFineY+lineTestDistance*math.sin(angleFine)
        #print(result)
        self.dobot.jump_to(lineTestX, lineTestY, z , r, True)
        time.sleep(0.5)
        result = self.getToLineCenter(lineTestX, lineTestY, z, r,angleFine,testLineRadius,True)
        if (result == None):
             return None
        lineCenterX, lineCenterY = result
        angleEvenFine = math.atan2(lineCenterY-centerFineY,lineCenterX-centerFineX)
        self.calibrated = True
        self.jigX = centerFineX
        self.jigY = centerFineY
        self.jigAngle = angleEvenFine
        self.saveJigCoordinateToFile()
        
        #print(angleFine,angleEvenFine)
        self.dobot.move_to(lineCenterX, lineCenterY, z , r, True)
        time.sleep(0.5)
        
        #test
        #self.dobot.move_to(centerFineX+50*math.cos(angleEvenFine), centerFineY+50*math.sin(angleEvenFine), z , r, True)
        #time.sleep(2)
        
        #self.dobot.move_to(x,y,z,r)
        return [self.jigX,self.jigY,self.jigAngle]
    
    def measureZWithLaser(self):
        x, y, z, r, j1, j2, j3, j4=self.dobot.pose()
        if not self.calibrated:
            print("Calibarte Jig before measureZ")
            return None
        self.dobot.set_ptp_coordinate_params(velocity=200, acceleration=200)
        self.dobot.set_ptp_jump_params(10, 200)
        self.dobot.set_ptp_common_params(velocity=100, acceleration=50)
        if abs(x - self.jigX)>0.1 or abs(y - self.jigY)>0.1:
            self.dobot.jump_to(self.jigX,self.jigY, z , r, True)
        self.dobot.set_ptp_common_params(velocity=10, acceleration=50)
        measureMoveUpDistance = 10
        self.dobot.move_to(self.jigX,self.jigY, z+measureMoveUpDistance , r, True)

        if self.sensorXIdle is None:
            self.getSensorAvgValues(0.3)
        #measure pose and sensor at the same time
        self.ser.open() #open port only when we need it. This helps solve the failure to read in second try.
        self.ser.reset_input_buffer()
        self.ser.write(b'L\n');
        response = self.dobot.move_to(self.jigX,self.jigY, z , r, False)
        expected_idx = struct.unpack_from('L', response.params, 0)[0]    
        current_idx = 0
        self.ser.read_all()
        recvBuffer = ""
        sensorDataX = []    #we do X beam only
        nozzleZ = []
        
        cX, cY, cZ, cR, j1, j2, j3, j4=self.dobot.pose()
        pZ = 9999999 # a number far away
        while True:
            cX, cY, cZ, cR, j1, j2, j3, j4=self.dobot.pose()
            dist = abs(cZ-pZ)
            pZ=cZ
            #print(cX, cY, dist)
            decodedData = self.ser.read_all().decode("ascii", errors="ignore")
            dataLines = (recvBuffer+decodedData).split('\r\n')
            recvBuffer = dataLines.pop()
            for line in dataLines:
                valuesStr = line.split(',')
                if len(valuesStr)==2:
                    try:
                        t1 = int(valuesStr[0])
                        t2 = int(valuesStr[1])
                        sensorDataX.append(t1)
                        nozzleZ.append(None)
                    except:
                        print("some non-critial error in parsing:"+line)
            nozzleZ[-1] = cZ
            
            if (dist)<0.1:
                break
        #print("DOBOT STOP MOVING")
        while True:
             try:
                 current_idx = self.dobot._get_queued_cmd_current_index()
             except:
                 pass    #at end of home command, there is a huge delay, which means dobot will not respond for 1~2 seconds 
             if current_idx == expected_idx:
                 break
        #print("Command completed")
        #1st data may be corrupted
        sensorDataX.pop(0)  
        nozzleZ.pop(0)
        self.ser.write(b'S\n');
        self.ser.close()
        
        #interpolate nozzleZ, because pose is slow
        nozzleZ = self.interpolatePoseDataWithNoneElements(nozzleZ)
        #threshold = self.sensorXIdle + int((max(sensorDataX)-self.sensorXIdle)/10)  #use 10% of max inc, this may need change in future
        threshold = self.sensorXIdle+10 #for test
        if (threshold<(self.sensorXIdle+10)):
            print("In measureZWithLaser, seems laser is not triggered. Adjust Z lower and try again",threshold,self.sensorXIdle)
            return None
        
        thresholdZPos = z+measureMoveUpDistance
        for i in range(len(sensorDataX)):
            if sensorDataX[i]>threshold:
                break
            thresholdZPos = nozzleZ[i]
            
        searchBack = 0.6
        searchStep = 0.1
        searchZ = thresholdZPos+searchBack
        searchFinish = False
        self.dobot.move_to(self.jigX,self.jigY, searchZ , r, True)
        
        
        self.ser.open() #open port only when we need it. This helps solve the failure to read in second try.
        self.ser.reset_input_buffer()
        self.ser.write(b'L\n');
        
        while (searchZ>z) and (not searchFinish):
            self.dobot.move_to(self.jigX,self.jigY, searchZ , r, True)
            self.ser.read_all()
            time.sleep(0.05)
            decodedData = self.ser.read_all().decode("ascii", errors="ignore")
            dataLines = (decodedData).split('\r\n')
            dataLines.pop()
            dataLines.pop(0)
            senserXForAvg = []
            for line in dataLines:
                valuesStr = line.split(',')
                if len(valuesStr)==2:
                    try:
                        t1 = int(valuesStr[0])
                        t2 = int(valuesStr[1])
                        senserXForAvg.append(t1)
                    except:
                        print("some non-critial error in parsing:"+line)
            senserXAvg = sum(senserXForAvg)/len(senserXForAvg)
            if (senserXAvg>threshold):
                searchFinish = True
            else:
                searchZ-=searchStep

        self.ser.write(b'S\n');
        self.ser.close()
        self.dobot.set_ptp_common_params(velocity=100, acceleration=50)
        if (searchFinish):
            self.jigZLaser = searchZ
            self.saveJigZLaserToFile()
            return searchZ
        else:
            print("In measureZWithLaser, laser not triggered")
            return None

    def measureZWithTouchpad(self, limit = 1):
        x, y, z, r, j1, j2, j3, j4=self.dobot.pose()
        if not self.calibrated:
            print("Calibarte Jig before measureZ")
            return None    
 
        searchZLimit = z-limit
        searchStep = 0.1
        searchZ = z
        searchFinish = False
        self.dobot.move_to(x, y, searchZ , r, True)
        
        
        self.ser.open() #open port only when we need it. This helps solve the failure to read in second try.
        self.ser.reset_input_buffer()
        self.ser.write(b'T\n');
        
        while (searchZLimit<searchZ) and (not searchFinish):
            self.dobot.move_to(x, y, searchZ , r, True)
            self.ser.read_all()
            time.sleep(0.05)
            decodedData = self.ser.read_all().decode("ascii", errors="ignore")
            dataLines = (decodedData).split('\r\n')
            dataLines.pop()
            dataLines.pop(0)
            sensordata = int(dataLines[-1])
            if (sensordata == 0):
                searchFinish = True
            else:
                searchZ-=searchStep

        self.ser.write(b'S\n');
        self.ser.close()
        self.dobot.set_ptp_common_params(velocity=100, acceleration=50)
        self.dobot.move_to(x, y, z , r, True) # go back
        if (searchFinish):
            self.jigZTouchpad = searchZ
            return searchZ
        else:
            print("In measureZWithTouchpad, Pad not touched")
            return None


        
        
    def doXYCalibration(self):
        x, y, z, r, j1, j2, j3, j4=self.dobot.pose()
        result = self.getJigPositonRotation();
        if (result == None):
            self.dobot.move_to(x,y,z,r)
        else:
            self.dobot.move_to(self.jigX,self.jigY,z,r)
            
    def doZCalibrationLaser(self):
        pass
        self.loadJigCoordinatefromFile()
        result = self.measureZWithLaser()
        if (result == None):
             return None
        print("Z ",result)
        
    def getJigMapping(self, inputX, inputY):
        if not self.calibrated:
            return None, None
        
        #add jig offset from laser cross and board origin
        inputX=inputX + 3.5
        inputY=inputY + 3.5
        
        angle = self.jigAngle
        x1=math.cos(angle)*inputX-math.sin(angle)*inputY;
        y1=math.cos(angle)*inputY+math.sin(angle)*inputX;
        return (x1+self.jigX),(y1+self.jigY)
        
        
    def getSensorAvgValues(self, sampleTime):
        self.ser.open() #open port only when we need it. This helps solve the failure to read in second try.
        self.ser.reset_input_buffer()
        self.ser.write(b'L\n');
        self.ser.read_all()
        recvBuffer = ""
        timeout = time.time() + sampleTime
        while time.time() <= timeout:
            bytesData = self.ser.read_all()
            recvBuffer = recvBuffer + bytesData.decode("ascii", errors="ignore")
        firstNewLineIndex = recvBuffer.index('\r\n')
        lastNewLineIndex = recvBuffer.rindex('\r\n')
        dataLines = recvBuffer[firstNewLineIndex+2:lastNewLineIndex].split('\r\n')
        dataX = []
        dataY = []
        for line in dataLines:
            valuesStr = line.split(',')
            if len(valuesStr)==2:
                dataX.append(int(valuesStr[0]))
                dataY.append(int(valuesStr[1]))
        self.ser.write(b'S\n');
        self.ser.close()
        avgX = int(sum(dataX)/len(dataX))
        avgY = int(sum(dataY)/len(dataY))
        self.sensorXIdle = avgX
        self.sensorYIdle = avgY
        return [avgX,avgY]
        
    def findPeaks(self,arr):
        sortedArr = sorted(arr)
        lowestValues = sortedArr[:int(len(arr)/4)]
        highestValue = sortedArr[-1]
        lowestValue = int(sum(lowestValues)/len(lowestValues))
        #threshold = int((highestValue+lowestValue)/2)
        #threshold = max(threshold,lowestValue+100)
        threshold = lowestValue+100 #some quick scan may get 2 peaks with different max
        threshold_low = threshold-10
        
        peaks = []
        
        valueAbove = False;
        risingEdge = 0;
        fallingEdge = 0
        for i in range(len(arr)):
          if not valueAbove:
            if arr[i]>threshold:
              risingEdge = i
              valueAbove = True
          else:
            if arr[i]<threshold_low:
              fallingEdge = i
              valueAbove = False
              peaks.append( int((risingEdge+fallingEdge)/2) )
        
        if (valueAbove):  #in case the end is high
              fallingEdge = len(arr)-1
              valueAbove = False
              peaks.append( int((risingEdge+fallingEdge)/2) )
        
        return peaks


if __name__ == '__main__':
    pass
    
