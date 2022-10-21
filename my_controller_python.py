from controller import Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())
max_speed = 2
LFSensor=[]
ds_names=["lineFollowSensor0","lineFollowSensor1","lineFollowSensor2","lineFollowSensor3","lineFollowSensor4"]
ds_val=[0]*len(ds_names)
for name in ds_names:
    LFSensor.append(robot.getDevice(name))
ds=[]   
for i in range(len(ds_names)):
    ds.append(robot.getDevice(ds_names[i]))
    ds[i].enable(timestep)
   
wheels=[]
wheel_names=["right_wheel","left_wheel"]

for name in wheel_names:
    wheels.append(robot.getDevice(name))
    wheels[-1].setPosition(float('inf'))
    wheels[-1].setVelocity(0.0)


previousError=error=0
Kp=0.005
Ki=0
Kd=0.15
I=0
PIDvalue=0
iniMotorSpeed = iniMotorPower=5

def calculatePID():
    global previousError,I,Kp,Kd,Ki,PIDvalue
    P = error
    I = I + error
    D = error-previousError
    PIDvalue = (Kp*P) + (Ki*I) + (Kd*D)
    previousError = error
    print("PID",PIDvalue)
    return PIDvalue

def motorPIDcontrol(PIDValue1):
    
    leftMotorSpeed = 5 + 2*PIDValue1*100
    rightMotorSpeed= 5 - 2*PIDValue1*100
    if rightMotorSpeed>10:
        rightMotorSpeed=9
    if leftMotorSpeed<-10:
        leftMotorSpeed=-9
    if rightMotorSpeed<-10:
        rightMotorSpeed=-9
    if leftMotorSpeed>10:
        leftMotorSpeed=9
        
  
    print("leftMotorSpeed",leftMotorSpeed)
    print("rightMotorSpeed",rightMotorSpeed)

    wheels[0].setVelocity(rightMotorSpeed)
    wheels[1].setVelocity(leftMotorSpeed)


while robot.step(timestep)!=-1:
    LFSensor[0]=ds[0].getValue()
    LFSensor[1]=ds[1].getValue()
    LFSensor[2]=ds[2].getValue() 
    LFSensor[3]=ds[3].getValue()
    LFSensor[4]=ds[4].getValue()
    print(LFSensor[0])
    print(LFSensor[1])
    print(LFSensor[2])
    print(LFSensor[3])
    print(LFSensor[4])
    print("--------------------")


 
    if((LFSensor[0]<=750 ) and (LFSensor[1]<=750 )and(LFSensor[2]<=750)and(LFSensor[3]<=750 )and(LFSensor[4]>=800 )):
        error = 4
        

    elif((LFSensor[0]<=750 )and(LFSensor[1]<=750)and(LFSensor[2]<=750)and(LFSensor[3]>=800  )and(LFSensor[4]>=800  )):
        error = 3
        
    elif((LFSensor[0]<=750  )and(LFSensor[1]<=750 )and(LFSensor[2]<=750  )and(LFSensor[3]>=800   )and(LFSensor[4]<=750 )):
        error = 2
        print("we are in this error 2")
    elif((LFSensor[0]<=750  )and(LFSensor[1]<=750 )and(LFSensor[2]>=800 )and(LFSensor[3]>=800  )and(LFSensor[4]<=750 )):
        error = 1
        print("we are in this error 1")
    elif((LFSensor[0]<=750 )and(LFSensor[1]<=750)and(LFSensor[2]>=800  )and(LFSensor[3]<=750 )and(LFSensor[4]<= 750 )):
        error = 0
        print("we are in this error 0")
        
    elif((LFSensor[0]<=750)and(LFSensor[1]>=800  )and(LFSensor[2]>=800  )and(LFSensor[3]<=750)and(LFSensor[4]<=750 )):
        error =- 1
        print("we are in this error -1")
    elif((LFSensor[0]<=750 )and(LFSensor[1]>=800  )and(LFSensor[2]<=750 )and(LFSensor[3]<=750)and(LFSensor[4]<=750 )): 
        error = -2
        print("we are in this error -2")
    elif((LFSensor[0]>=800 )and(LFSensor[1]>=800  )and(LFSensor[2]<=750 )and(LFSensor[3]<=750)and(LFSensor[4]<=750 )):
        error = -3
        
    elif((LFSensor[0]>=800 ) and(LFSensor[1]<=750 )and(LFSensor[2]<=750)and(LFSensor[3]<=750 )and(LFSensor[4]<=750)):
        error = -4
    PIDvalue1=calculatePID()
    motorPIDcontrol(PIDvalue1)