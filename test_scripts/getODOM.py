import matplotlib.pyplot as plt
import datetime
import traceback
import math

class RobotOdometry():
    def __init__(self,radius=1.27,lengthBetweenTwoWheels=10,encoderResolution=720,sensordist=[5,5,5,5,5]):
        self.lengthBetweenTwoWheels=lengthBetweenTwoWheels #cm
        self.radius = radius #cm
        self.encoderResolution = encoderResolution
        self.distPerCount= (2*math.pi*self.radius)/self.encoderResolution #cm
        self.prevLtick=0 
        self.prevRtick=0
        self.sensordist = sensordist
        
        self.x=0
        self.y=0
        self.th=0

        # front sensor
        self.sensor0_x=self.sensordist[0]*math.cos(0)+self.x
        self.sensor0_y=self.sensordist[0]*math.sin(0)+self.y

        # extreme right sensor
        self.sensor1_x=self.sensordist[1]*math.cos(math.pi/2.0)+self.x
        self.sensor1_y=self.sensordist[1]*math.sin(math.pi/2.0)+self.y

        # extreme left sensor
        self.sensor2_x=self.sensordist[2]*math.cos(-1*math.pi/2.0)+self.x
        self.sensor2_y=self.sensordist[2]*math.sin(-1*math.pi/2.0)+self.y
        
        self.sensor3_x=self.sensordist[3]*math.cos(math.pi/4.0)+self.x
        self.sensor3_y=self.sensordist[3]*math.sin(math.pi/4.0)+self.y
        
        self.sensor4_x=self.sensordist[4]*math.cos(-1*math.pi/4.0)+self.x
        self.sensor4_y=self.sensordist[4]*math.sin(-1*math.pi/4.0)+self.y
        
        self.currTime=datetime.datetime.now()
        self.plt=plt
        self.plt.show(block=True)

    def getTimeDelta(self):
        newtime = datetime.datetime.now()
        timeDelta = (newtime-self.currTime).microseconds/1000000.0
        print("deltatime:",timeDelta)
        self.currTime = newtime
        return float(timeDelta)

    def getODOM(self,lm,rm):
        self.deltaL = lm-self.prevLtick 
        self.deltaR = rm-self.prevRtick 

        # self.dt = self.getTimeDelta();
        self.dt = 1;
        self.omega_left = (self.deltaL * self.distPerCount) / self.dt
        self.omega_right = (self.deltaR * self.distPerCount) / self.dt

        # self.v_left = self.omega_left * self.radius
        # self.v_right = self.omega_right * self.radius
        self.v_left = self.omega_left 
        self.v_right = self.omega_right 

        self.vx = ((self.v_right + self.v_left) / 2)
        self.vy = 0
        self.vth = ((self.v_right - self.v_left)/self.lengthBetweenTwoWheels)

        self.delta_x = (self.vx * math.cos(self.th)) * self.dt;
        self.delta_y = (self.vx * math.sin(self.th)) * self.dt;
        self.delta_th = self.vth * self.dt;

        self.x += self.delta_x;
        self.y += self.delta_y;
        self.th += self.delta_th;


        # sensor positions
        self.sensor0_x = self.sensordist[0]*math.cos(self.th)+self.x
        self.sensor0_y = self.sensordist[0]*math.sin(self.th)+self.y

        self.sensor1_x = self.sensordist[1]*math.cos(self.th+math.pi/2.0)+self.x
        self.sensor1_y = self.sensordist[1]*math.sin(self.th+math.pi/2.0)+self.y

        self.sensor2_x = self.sensordist[2]*math.cos(self.th-math.pi/2.0)+self.x
        self.sensor2_y = self.sensordist[2]*math.sin(self.th-math.pi/2.0)+self.y

        self.sensor3_x = self.sensordist[3]*math.cos(self.th+math.pi/4.0)+self.x
        self.sensor3_y = self.sensordist[3]*math.sin(self.th+math.pi/4.0)+self.y

        self.sensor4_x = self.sensordist[4]*math.cos(self.th-math.pi/4.0)+self.x
        self.sensor4_y = self.sensordist[4]*math.sin(self.th-math.pi/4.0)+self.y

        return (self.x,self.y,self.th)

    def printPos(self):
        print("I am at ({},{}) facing {}".format(self.x,self.y,(self.th*180/math.pi)))
        plt.plot(self.x,self.y,'r.')
        plt.plot(self.sensor0_x,self.sensor0_y,'g.')
        plt.plot(self.sensor1_x,self.sensor1_y,'b.')
        plt.plot(self.sensor2_x,self.sensor2_y,'b.')
        plt.plot(self.sensor3_x,self.sensor3_y,'y.')
        plt.plot(self.sensor4_x,self.sensor4_y,'y.')
        plt.draw()
        # plt.pause(0.5)

if __name__ == '__main__':
    try:
        robot = RobotOdometry()
        # while True:
        #     lm = input("left motor ticks >> ")
        #     rm = input("Right motor ticks >> ")
        #     robot.getODOM(lm,rm)
        #     robot.printPos()
        #     print(lm,rm)
        # for j in range(2):
        for i in range(714):
            lm=1
            rm=1
            robot.getODOM(lm,rm)
            robot.printPos()
            print(lm,rm)
        robot.plt.show() 
        for i in range(714):
            lm=0
            rm=1
            robot.getODOM(lm,rm)
            robot.printPos()
            print(lm,rm)
        robot.plt.show()    
        # for i in range(714):
        #     lm=0
        #     rm=-1
        #     robot.getODOM(lm,rm)
        #     robot.printPos()
        #     print(lm,rm)
        # robot.plt.show()    
    except Exception as e:
        traceback.print_exc()
        quit()