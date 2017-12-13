import math
import random
import traceback


class Sensor():
    '''
    Models the behaviour of the sensor and calculates the x,y
    co-ordinates of the obstacle w.r.t the centroid of the
    robot it is attached to.
    '''

    def __init__(self,
                 distFromCenter=0,
                 angleFromHeading=0,
                 minVal=0,
                 maxVal=200.0,
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

    def get_reading(self, value, x, y, theta):
        # returns the (x,y) co-ordinates of the point which the sensor
        # detected w.r.t (x,y,theta) frame provided

        if (value >= self.minVal) and (value < self.maxVal):
            self.sensor_x = (self.distFromCenter + value) * math.cos(theta + self.angleFromHeading) + x
            self.sensor_y = (self.distFromCenter + value) * math.sin(theta + self.angleFromHeading) + y

            return (self.sensor_x, self.sensor_y)
        else:
            return (0, 0)


class RobotOdometry(object):
    '''
    This class describes the odometry of the robot
    It returns the point cloud of the sensors connected to it
    '''

    def __init__(self,
                 radius=1.27,
                 length_between_wheels=16.14,
                 encoder_resolution=720,
                 sensor_params = [
                                  {
                                    "id": "LeftSensor",
                                    "sensordist": 1,
                                    "sensorAngle": math.pi/2,
                                    "minVal": 0.3,
                                    "maxVal": 200,
                                    "marker":'b.'
                                  },
                                  {
                                    "id": "Left45Sensor",
                                    "sensordist": 1,
                                    "sensorAngle": math.pi/4,
                                    "minVal": 0.3,
                                    "maxVal": 200,
                                    "marker":'y.'
                                  },
                                  {
                                    "id": "frontSensor",
                                    "sensordist": 1,
                                    "sensorAngle": 0,
                                    "minVal": 0.3,
                                    "maxVal": 200,
                                    "marker":'g.'
                                  },
                                  {
                                    "id": "Right45Sensor",
                                    "sensordist": 1,
                                    "sensorAngle": -1*math.pi/4,
                                    "minVal": 0.3,
                                    "maxVal": 200,
                                    "marker":'y.'
                                  },
                                  {
                                    "id": "RightSensor",
                                    "sensordist": 1,
                                    "sensorAngle": -1*math.pi/2,
                                    "minVal": 0.3,
                                    "maxVal": 200,
                                    "marker":'b.'
                                  },
                                ]):
        self.length_between_wheels = length_between_wheels  # cm
        self.dist_per_tick = (2 * math.pi * radius) / encoder_resolution  # cm

        self.prev_left_tick = 0
        self.prev_right_tick = 0
        self.sensor_params = sensor_params
        self.sensordist = [param["sensordist"] for param in self.sensor_params]
        self.dt = 1

        self.x = 500
        self.y = 500
        self.heading = 0

        self.sensors = [Sensor(param["sensordist"],
                               param["sensorAngle"],
                               param['minVal'],
                               param['maxVal'],
                               [0, 0])
                               for param in sensor_params]


    def getODOM(self, left_ticks, right_ticks, sensor_data):
        if self.prev_right_tick == 0 and self.prev_left_tick == 0:
            self.prev_left_tick, self.prev_right_tick = left_ticks, right_ticks
            return

        dt = 1
        # calculate the rotation of the wheel in terms of ticks
        deltaL = left_ticks - self.prev_left_tick
        deltaR = right_ticks - self.prev_right_tick

        self.prev_left_tick = left_ticks
        self.prev_right_tick = right_ticks

        # calculate the velocity of each wheel
        omega_left = (deltaL * self.dist_per_tick) / dt
        omega_right = (deltaR * self.dist_per_tick) / dt

        v_left = omega_left
        v_right = omega_right

        # calculate the velocity along heading
        vx = ((v_right + v_left) / 2)

        # calculate the heading in radians
        vth = ((v_left - v_right ) / self.length_between_wheels)

        # calculate the change in x position and y position
        delta_x = (vx * math.cos(self.heading)) * dt
        delta_y = (vx * math.sin(self.heading)) * dt
        delta_th = vth * dt

        # update position of Robot (centroid)
        self.x += delta_x
        self.y += delta_y
        self.heading += delta_th

        # update sensor with readings
        # measurement = (distance of sensor from centroid)+(sensor reading)
        sensor_data = list(map(lambda x, y: x+y, self.sensordist, sensor_data))

        # sensor reading is a tuple of (x,y)
        sensor_readings = [sensor.get_reading(sensor_data[idx], self.x, self.y, self.heading) for idx, sensor in enumerate(self.sensors)]

        return {"frame": (self.x, self.y, self.heading),"sensorValues": sensor_readings}
