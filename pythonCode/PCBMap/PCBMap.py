import math
import os
from collections import namedtuple

PastePad = namedtuple("PastePad", "x y area")

class PCBMap:
    def __init__(self):
        self.scriptPath = os.path.dirname(os.path.abspath(__file__))
        self.pastePads = []
        
        self.stepperMapping = 10
        
        self.boardWidth = 0
        self.boardHeight = 0

    def filterCells(self,cells):
        if ("0805" in cells[1]):
            return True
        if ("SO08" in cells[1]):
            return True
        if ("SOD" in cells[1]):
            return True
        return False
    
    def loadPasteCSV(self,filename,rotation = 0):
        if (os.path.exists(filename)):
            pass
        elif (os.path.exists(self.scriptPath+'/'+filename)):
            filename = self.scriptPath+'/'+filename
        else:
            print("PasteCSV "+filename+" not found")
            return None
        
        self.pastePads = []
        
        f = open(filename, "r")
        csvLines = f.readlines()
        f.close()
        headerLine = csvLines.pop(0).strip()
        
        ll_x = 0;
        ll_y = 0;
        
        for line in csvLines:
            line = line.strip();
            if (len(line)>0):
                cells = line.split(',')
                if (cells[0]=="CORNER LL"):
                    ll_x = float(cells[4])
                    ll_y = float(cells[5])
                if (cells[0]=="CORNER UR"):
                    ur_x = float(cells[4])
                    ur_y = float(cells[5])
                    self.boardWidth = ur_x - ll_x
                    self.boardHeight =  ur_y - ll_y
        
        for line in csvLines:
            line = line.strip();
            if (len(line)>0):
                cells = line.split(',')
                
                if cells[2]=="SMD":
                    if not self.filterCells(cells):
                        continue
                    padX = float(cells[4])
                    padY = float(cells[5])
                    padAngle = float(cells[6])
                    padSizeX = float(cells[7])
                    padSizeY = float(cells[8])
                    padArea = padSizeX * padSizeY
                    if (rotation == 180):
                        padX = self.boardWidth - padX
                        padY = self.boardHeight - padY
                    
                    pastePad = PastePad(x=padX, y=padY, area=padArea)    
                    self.pastePads.append(pastePad)
                else:
                    pass
        
        return self.pastePads    
        
        
        
    def loadJigDataFromFile(self): 
        try:
            f = open("jig_coordinate.txt", "r")
            lines = f.read().split('\n')
            f.close()
            data = lines[0].strip().split(',')
            self.jigX = float(data[0])
            self.jigY = float(data[1])
            self.jigAngle = float(data[2])
            print("loaded jigX: %03.3f, jigY: %03.3f, jigAngle: %03.3f" % (self.jigX, self.jigY, self.jigAngle))
        except Exception as e:
            print("something wrong in loadJigCoordinatefromFile")
            print(e)
        
    def setPastePads(self, pastePads):
        self.pastePads = pastePads
        
    def getPastePadsCount(self):
        return len(self.pastePads)
    
    def getPadMapping(self,index):
        if (index>=len(self.pastePads)):
            return None
        
        pad = self.pastePads[index]
        
        steps = int(self.stepperMapping*pad.area)
        
        x=pad.x + 3.5
        y=pad.y + 3.5
        
        angle = self.jigAngle
        x1=math.cos(angle)*x-math.sin(angle)*y;
        y1=math.cos(angle)*y+math.sin(angle)*x;
        return (x1+self.jigX,y1+self.jigY,steps)
    
    # def addPcbRefPoint(self,x,y):
    #     self.pcbRefPoints.append((x,y));
    #     
    # def addDobotRefPoint(self,x,y):
    #     self.dobotRefPoints.append((x,y));
    #     
    # def calculateTransform(self):
    #     #for better algorithm
    #     #nghiaho.com/?page_id=671
    #     #https://math.stackexchange.com/questions/77462/finding-transformation-matrix-between-two-2d-coordinate-frames-pixel-plane-to
    #     if (len(self.pcbRefPoints)==len(self.dobotRefPoints) and len(self.pcbRefPoints)==2):
    #         #pcbCenterX = pcbCenterY = 0;
    #         #for p in self.pcbRefPoints: 
    #         #    pcbCenterX += p[0]
    #         #    pcbCenterY += p[1]
    #         #pcbCenterX = pcbCenterX/len(self.pcbRefPoints)
    #         #pcbCenterY = pcbCenterY/len(self.pcbRefPoints)
    #         
    #         
    #         #self.offset = (pcbCenterX,pcbCenterY)
    #         
    #         self.offset = (self.dobotRefPoints[0][0]-self.pcbRefPoints[0][0],self.dobotRefPoints[0][1]-self.pcbRefPoints[0][1]);
    #         anglePCB = math.atan2(self.pcbRefPoints[1][1]-self.pcbRefPoints[0][1],self.pcbRefPoints[1][0]-self.pcbRefPoints[0][0])
    #         angleDobot = math.atan2(self.dobotRefPoints[1][1]-self.dobotRefPoints[0][1],self.dobotRefPoints[1][0]-self.dobotRefPoints[0][0])
    #         self.angleDiff = angleDobot-anglePCB
    #         #print("OK",self.offset,anglePCB,self.angleDiff)
    
    def calculateMappedPoint(self,point):
        x=point[0]
        y=point[1]
        angle = self.angleDiff
        x1=math.cos(angle)*x-math.sin(angle)*y;
        y1=math.cos(angle)*y+math.sin(angle)*x;
        return (x1+self.offset[0],y1+self.offset[1])
    
    def calculateMappedPoint(self,point):
        x=point[0]
        y=point[1]
        angle = self.angleDiff
        x1=math.cos(angle)*x-math.sin(angle)*y;
        y1=math.cos(angle)*y+math.sin(angle)*x;
        return (x1+self.offset[0],y1+self.offset[1])
    

