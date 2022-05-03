import cv2
import numpy as np
import matplotlib.pyplot as plt
import csv
import serial
import time

with open('tra.csv','r') as csvfile:
        reader = csv.reader(csvfile)
        x = []
        y = []
        for row in reader:
            x.append(float(row[0]))
            y.append(float(row[1]))

########################################
##         Plot Scatter Graph         ##
########################################
            
plt.scatter(x, y)
plt.xlabel('Aruco Marker Width')
plt.ylabel('Distance of Aruco Marker')

########################################
##        linear  regression          ##
########################################

z = np.polyfit(x, y, 1)
p = np.poly1d(z)
#plt.plot(x,p(x),"r--")

########################################
##       polynomial regression        ##
########################################

z2 = np.polyfit(x, y,3)
p2 = np.poly1d(z2)
plt.plot(x,p2(x),"g")

########################################
##      Generate Final Equation       ##
########################################

#plt.title("y=%.10fx^3 %.10fx^2 %.10fx (%.10f)"%(z2[0],z2[1],z2[2],z2[3])) #Polynomial Equation
#plt.title("y=%.10fx (%.10f)"%(z[0],z[1]))  #Linear Equation
#eqL = "%.10f"%(z2[0])
#qeL2 =  "%.10f"%(z2[1])

eq = "%.10f"%(z2[0])
eq2 = "%.10f"%(z2[1])
eq3 = "%.10f"%(z2[2])
eq4 = "%.10f"%(z2[3])
#solution = (float(eq))*((mk.area)**2) +(float(eq2))*(mk.area) +(float(eq3))
#print (solution)
 
########################################
##            Show Results            ##
########################################

plt.show()

########################################
##           Intialization            ##
########################################

font = cv2.FONT_HERSHEY_SIMPLEX
capture = cv2.VideoCapture(0, cv2.CAP_DSHOW);
parameters = cv2.aruco.DetectorParameters_create()
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

arduino = serial.Serial('COM7', 9600)
time.sleep(0.1)
print("Connection to arduino...")
########################################
##             Main Loop              ##
########################################
#capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
#capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
midScreenX =320
midScreenY =240
midScreenWindow = 100
stepSize = 4
upperServo=20
LowerServo=70
servoTiltPosition =70
servoPanPosition =90
sendcommand=False

while True:
    NoFrame, Frame = capture.read()
    #cv2.resizeWindow('frame', 800,800)

    if not NoFrame:
        break
    if cv2.waitKey(1) == ord('q'):
        break
    
########################################
##            BGR TO GRAY             ##
########################################   
        
    FrameGray = cv2.cvtColor(Frame, cv2.COLOR_BGR2GRAY)
    
########################################
##       Detect Markers & Draw        ##
########################################   
    
    MarkerCorners, MarkerIds, RejectedCandidates = cv2.aruco.detectMarkers(FrameGray, dictionary, parameters=parameters)
    cv2.aruco.drawDetectedMarkers(Frame, MarkerCorners,MarkerIds,(255, 128, 10))
    #print(MarkerCorners)
  #  cv2.putText(Frame, "NO MARKER HERE......"+ str( MarkerCorners), (0,64), font, 1, (255, 128, 0),2,cv2.LINE_AA)
    #cv2.rectangle(Frame, (x, y), (x + w, y + h), (36,255,12), 2)
    #cv2.putText(Frame, "w={},h={}".format(w,h), (x,y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (36,255,12), 2)
    
########################################
##    Find the marker ID & Output     ##
########################################

    if np.all(MarkerIds != None):    
        DetectedID =''
        for i in range(0, MarkerIds.size):
            DetectedID = str(MarkerIds[i])
        #cv2.putText(Frame, "MARKER ID IS: " + DetectedID, (0,64), font, 1, (255, 128, 0),2,cv2.LINE_AA)
    else:
        cv2.putText(Frame, "NO MARKER HERE......", (0,64), font, 1, (255, 128, 0),2,cv2.LINE_AA)

########################################
##     Find Marker Area  & Output     ##
########################################   

    if len(MarkerCorners) !=0:

        x,y,w,h = cv2.boundingRect(MarkerCorners[i])
        cv2.putText(Frame, "w={},h={}".format(w,h), (x,y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (36,255,12), 2)
        #cv2.rectangle(Frame,(x,y),(x+w,y+h),(0,255,0),2)
        
        
        area = cv2.contourArea(MarkerCorners[i])
        FinalAreaValue = str(area)
       
        
        #Now for Arduino 
        xx = int(x+(w/2))
        yy = int(y+(h/2))
        # print ('X',xx)
        #print ('Y',yy)
        """
        if xx<300:
            LowerServo += stepSize
            sendcommand=True
        if yy>250:
            upperServo -= stepSize
            sendcommand=True
        if xx>300:
            LowerServo -= stepSize
            sendcommand=True
        if yy>250:
            upperServo -= stepSize
            sendcommand=True  
        if xx<300:
            LowerServo += stepSize
            sendcommand=True
        if yy<250:
            upperServo += stepSize
            sendcommand=True 
        if xx>300:
            LowerServo -= stepSize
            sendcommand=True
        if yy<250:
            upperServo += stepSize
            sendcommand=True 
        if sendcommand:
            data = "X{0:f}Y{1:f}Z".format(upperServo, LowerServo)
            arduino.write(data.encode())
            print(data.encode())
            sendcommand = False
        """
        
        if yy<(midScreenY-midScreenWindow):
            if(servoTiltPosition<=100):
                servoTiltPosition+=stepSize
                sendcommand=True
                
        elif yy>(midScreenY+midScreenWindow):
            if(servoTiltPosition>=20):
                servoTiltPosition-=stepSize
                sendcommand=True
                
        if xx>(midScreenX+midScreenWindow):
            if(servoPanPosition>=20):
                servoPanPosition-=stepSize
                sendcommand=True       
                
        elif xx<(midScreenX-midScreenWindow):
            if(servoPanPosition<=160):
                servoPanPosition+=stepSize
                sendcommand=True
       
        rawdata =[]
        count =0        
        if sendcommand:
            data = "X{0:f}Y{1:f}Z".format(servoPanPosition, servoTiltPosition)
            arduino.write(data.encode())
            sendcommand = False

       
       # print ("x",x)
        #print ("y",y)
        #print ("w",w)
        #print ("h",h)
        
       # print ("yy",yy)
       # print ("xx",xx)
        
        
       # print ("screen Y",midScreenY)
       # print ("screen x",midScreenX)
        # cv2.putText(Frame, "Area: " + FinalAreaValue, (5,150), font, 1, (255, 128, 0),2,cv2.LINE_AA)
        #solution =(float(eq))*((w)**3) + (float(eq2))*((w)**2) + (float(eq3))*(w) +(float(eq4))
        #solution = - 1.263516*w +162.36698
        #solution = (2*(10**-7)*((area)**2))-(0.0114*area)+159.16
        #solution = ((5*(10)**-8)*(area**2)) -(0.0044*area) +97.79
        #solution = 0.0000*(w**5) - 0.000007*(w**4) -0.000219*(w**3) + 0.133927*(w**2) - 10.929491*w + 352.86469 
        #solution = 0.018483*(w**2) - 3.901566*w +234.758499
        #solution = -1.26232*w +162.33915
        #solution = -0.000058*(w**3)+0.028719*(w**2)-4.37055*w +231.600374
        
        #FINAL
        #solution = 0.0090*(w**2) - 2.5912*w +217.2665  #for 9.7 height
        solution = -0.0001*(w**3) +0.0389*(w**2) -5.2054*w + 282.5307
        #solution = 0.0000*(w**4) -0.0005*(w**3)+0.0973*(w**2)-8.3917*w+341.3565
        #solution = -0.8668*w +152.8c
        #solution = 0.0310*(w**2) - 5.2794*w +287.7776  #for 18.5 height
        #solution = 0.0368*(w**2) - 5.7748*w +296.1462  #for 20 height
       
        #Final Using W H
        #solution = 194.0266 -2.5563*w + 0.8221*h #for 9.7
        #solution = 172.8077 -12.1396*w +11.1384*h #for 18.5
        #solution = 162.4934 +11.4256*w -12.7706*h #for 20
        
        #Final sum
        #solution = 156.4897 - 3.0175 *w + 2.1124*h
        
        #solution = 148.1122 -2.657*w + 1.839*h
       
        #print (np.log(solution))
        print ("Tilt Angle:",servoTiltPosition)
        print ("Pan Angle: ",servoPanPosition)
        print ("Distance:  ",solution)
########################################
##         Output Final Frame         ##
#######################################
    #parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    x=  0
    y=  0
    cv2.imshow("frame", Frame)
    
########################################
##             Close Window           ##
########################################  
    
capture.release()
cv2.destroyAllWindows() 