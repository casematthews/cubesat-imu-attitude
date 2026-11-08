import serial
import time
arduinoData = serial.Serial('com3',115200)
time.sleep(1)
while (1==1):
    while (arduinoData.inWaiting()==0):
        pass
    dataPacket = arduinoData.readline()
    
    dataPacket = str(dataPacket, 'utf-8')
    splitPacket = dataPacket.split(',')
    Acal = float(splitPacket[0])
    Gcal = float(splitPacket[1])
    Mcal = float(splitPacket[2])
    Scal = float(splitPacket[3])
    Pitch = float(splitPacket[4])
    Roll = float(splitPacket[5])
    Yaw = float(splitPacket[6])
    print('Acal=',Acal,'Gcal=',Gcal,'Mcal=',Mcal,'Scal=',Scal,'Pitch=',Pitch,'Roll=',Roll,'Yaw=',Yaw)
    
    
