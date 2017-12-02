import matplotlib.pyplot as plt
import datetime
import traceback
import math
import random


class Sensor():
    '''
        Models the behaviour of the sensor and calculates the x,y 
        co-ordinates of the obstacle w.r.t the centroid of the 
        robot it is attached to.
    '''

    def __init__(self, distFromCenter=0,
                 angleFromHeading=0,
                 minVal=0,
                 maxVal=10.0,
                 origin=[0, 0]):

        self.distFromCenter = distFromCenter
        self.angleFromHeading = angleFromHeading
        self.minVal = minVal
        self.maxVal = maxVal

        # defining the initial position of the sensor w.r.t origin frame
        self.sensor_x = self.distFromCenter * \
            math.cos(self.angleFromHeading)+origin[0]
        self.sensor_y = self.distFromCenter * \
            math.sin(self.angleFromHeading)+origin[1]

    def getReading(self, val, x, y, theta):
        # returns the (x,y) co-ordinates of the point which the sensor 
        # detected w.r.t (x,y,theta) frame provided

        if (val >= self.minVal) and (val < self.maxVal):
            self.sensor_x = (self.distFromCenter+val) * \
                math.cos(theta+self.angleFromHeading)+x
            self.sensor_y = (self.distFromCenter+val) * \
                math.sin(theta+self.angleFromHeading)+y
            return (self.sensor_x, self.sensor_y)
        else:
            return (None, None)


class RobotOdometry():
    '''
        This class describes the odometry of the robot
        It returns the point cloud of the sensors connected to it
    '''

    def __init__(self, radius=1.27,
                 lengthBetweenTwoWheels=10,
                 encoderResolution=720,
                 mapSize=(1000, 1000),
                 # distances of each sensor from the centroid of the robot
                 # sensor distance in order front, Left, Right, Left_45, Right_45 in cm
                 sensordist=[1, 1, 1, 1, 1],
                 # angular offset of each sensor from the heading 
                 sensorAngle=[0,                    # front sensor
                              math.pi/2,            # Left sensor
                              -1*math.pi/2,         # Right sensor
                              math.pi/4,            # Left_45 sensor 
                              -1*math.pi/4]):       # Right_45 sensor

        self.lengthBetweenTwoWheels = lengthBetweenTwoWheels  # cm
        self.radius = radius  # cm
        self.encoderResolution = encoderResolution
        self.distPerCount = (2*math.pi*self.radius) / \
            self.encoderResolution  # cm
        self.prevLtick = 0
        self.prevRtick = 0
        self.sensordist = sensordist
        self.dt = 1 

        self.x = 0
        self.y = 0
        self.th = 0

        # sensor instances in order front, Left, Right, Left_45, Right_45
        self.sensors = [Sensor(sensordist[i], sensorAngle[i], 0.3, 40.0, [
                               0, 0]) for i in range(len(sensordist))]

        self.plt = plt
        self.plt.show(block=True)
        # self.plt.axis([-1*mapSize[0]/2, mapSize[0] /
        #                2, -1*mapSize[0]/2, mapSize[0]/2])

    def getODOM(self, lm, rm, sensorData=[0, 0, 0, 0, 0]):
        
        # calculate the rotation of the wheel in terms of ticks
        deltaL = lm-self.prevLtick
        deltaR = rm-self.prevRtick

        # calculate the velocity of each wheel
        omega_left = (deltaL * self.distPerCount) / self.dt
        omega_right = (deltaR * self.distPerCount) / self.dt

        v_left = omega_left
        v_right = omega_right

        # calculate the velocity along heading
        vx = ((v_right + v_left) / 2)

        # calculate the velocity perpendicular to heading
        # along wheel axle
        vy = 0

        # calculate the heading in radians
        vth = ((v_right - v_left)/self.lengthBetweenTwoWheels)

        # calculate the change in x position and y position
        delta_x = (vx * math.cos(self.th)) * self.dt
        delta_y = (vx * math.sin(self.th)) * self.dt
        delta_th = vth * self.dt

        # update position of Robot (centroid)
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # update sensor with readings
        # measurement = (distance of sensor from centroid)+(sensor reading)
        sensorData = map(lambda x, y: x+y, self.sensordist, sensorData)

        # sensor reading is a tuple of (x,y)
        self.sensor_readings = [
            sensor.getReading(sensorData[idx], self.x, self.y, self.th)
            for idx, sensor in enumerate(self.sensors)]

        return {"frame":(self.x, self.y, self.th),"sensorValues":self.sensor_readings}

    def printPos(self):
        # print("I am at ({},{}) facing {}".format(self.x,self.y,(self.th*180/math.pi)))
        self.markers = ['g.', 'b.', 'b.', 'y.', 'y.']
        plt.plot(self.x, self.y, 'r.')

        for idx, reading in enumerate(self.sensor_readings):
            plt.plot(reading[0], reading[1], self.markers[idx])

        plt.draw()
        plt.pause(0.01)

if __name__ == '__main__':
    try:
        robot = RobotOdometry()
        while True:
            lm = input("left motor ticks >> ")
            rm = input("Right motor ticks >> ")
            robot.getODOM(lm,rm, sensorData=[random.uniform(0, 1),
                                              random.uniform(0, 1),
                                              random.uniform(0, 1),
                                              random.uniform(0, 1),
                                              random.uniform(0, 1)])
                
            robot.printPos()
            print(lm,rm)
        # for j in range(2):
        # for i in range(71):
        #     lm = 5
        #     rm = 5
        #     robot.getODOM(lm, rm, sensorData=[random.uniform(0, 1),
        #                                       random.uniform(0, 1),
        #                                       random.uniform(0, 1),
        #                                       random.uniform(0, 1),
        #                                       random.uniform(0, 1)])
        #     robot.printPos()
        #     # print(lm,rm)
        # # robot.plt.show()
        # for i in range(71):
        #     lm = 0
        #     rm = 10
        #     # robot.getODOM(lm,rm)
        #     robot.getODOM(lm, rm, sensorData=[random.uniform(0, 1),
        #                                       random.uniform(0, 1),
        #                                       random.uniform(0, 1),
        #                                       random.uniform(0, 1),
        #                                       random.uniform(0, 1)])
        #     robot.printPos()
            # print(lm,rm)
        # for i in range(714):
        #     lm = 1
        #     rm = 1
        #     robot.getODOM(lm, rm, sensorData=[random.uniform(0, 1),
        #                                       random.uniform(0, 1),
        #                                       random.uniform(0, 1),
        #                                       random.uniform(0, 1),
        #                                       random.uniform(0, 1)])
        #     robot.printPos()
        #     # print(lm,rm)
        # for i in range(714):
        #     lm = 1
        #     rm = 0
        #     # robot.getODOM(lm,rm)
        #     robot.getODOM(lm, rm, sensorData=[random.uniform(0, 1),
        #                                       random.uniform(0, 1),
        #                                       random.uniform(0, 1),
        #                                       random.uniform(0, 1),
        #                                       random.uniform(0, 1)])
        #     robot.printPos()
            # print(lm,rm)
        # robot.plt.show()
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
