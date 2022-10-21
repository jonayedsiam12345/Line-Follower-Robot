from controller import Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())
max_speed = 2
LFS=[]
ds_names=["ir_sensor_0","ir_sensor_1","ir_sensor_2","ir_sensor_3","ir_sensor_4"]
ds_val=[0]*len(ds_names)
for name in ds_names:
    LFS.append(robot.getDevice(name))
ds=[]   
for i in range(len(ds_names)):
    ds.append(robot.getDevice(ds_names[i]))
    ds[i].enable(timestep)
   
wheels=[]
wheel_names=["wheel_right","wheel_left"]

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
    LFS[0]=ds[0].getValue()
    LFS[1]=ds[1].getValue()
    LFS[2]=ds[2].getValue() 
    LFS[3]=ds[3].getValue()
    LFS[4]=ds[4].getValue()
    print(LFS[0])
    print(LFS[1])
    print(LFS[2])
    print(LFS[3])
    print(LFS[4])
    print("--------------------")


    if((LFS[0]== 1 )and(LFS[1]== 1 )and(LFS[2]== 1 )and(LFS[3]== 1 )and(LFS[4]== 1 )):
       
        pass
       
    elif((LFS[0]== 0 )and(LFS[1]== 0 )and(LFS[2]== 0 )and(LFS[3]== 0 )and(LFS[4]== 0 )): 
        wheels[0].setVelocity(3.5)
        wheels[1].setVelocity(0.5)
        
    elif((LFS[0]<=750 ) and (LFS[1]<=750 )and(LFS[2]<=750)and(LFS[3]<=750 )and(LFS[4]>=800 )):
        error = 4
        

    elif((LFS[0]<=750 )and(LFS[1]<=750)and(LFS[2]<=750)and(LFS[3]>=800  )and(LFS[4]>=800  )):
        error = 3
        
    elif((LFS[0]<=750  )and(LFS[1]<=750 )and(LFS[2]<=750  )and(LFS[3]>=800   )and(LFS[4]<=750 )):
        error = 2
       
    elif((LFS[0]<=750  )and(LFS[1]<=750 )and(LFS[2]>=800 )and(LFS[3]>=800  )and(LFS[4]<=750 )):
        error = 1
       
    elif((LFS[0]<=750 )and(LFS[1]<=750)and(LFS[2]>=800  )and(LFS[3]<=750 )and(LFS[4]<= 750 )):
        error = 0
        
        
    elif((LFS[0]<=750)and(LFS[1]>=800  )and(LFS[2]>=800  )and(LFS[3]<=750)and(LFS[4]<=750 )):
        error =- 1
       
    elif((LFS[0]<=750 )and(LFS[1]>=800  )and(LFS[2]<=750 )and(LFS[3]<=750)and(LFS[4]<=750 )): 
        error = -2
       
    elif((LFS[0]>=800 )and(LFS[1]>=800  )and(LFS[2]<=750 )and(LFS[3]<=750)and(LFS[4]<=750 )):
        error = -3
        
    elif((LFS[0]>=800 ) and(LFS[1]<=750 )and(LFS[2]<=750)and(LFS[3]<=750 )and(LFS[4]<=750)):
        error = -4
    PIDvalue1=calculatePID()
    motorPIDcontrol(PIDvalue1)