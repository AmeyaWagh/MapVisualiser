#!/usr/bin/env python

import json
import math
import os
os.sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages/")
os.sys.path.append("/home/aleks/space/.pyenv/versions/2.7.14/lib/python2.7/site-packages")
import signal
import sys
import traceback

import rospy
import zmq
from std_msgs.msg import String
from tornado import web, websocket
from zmq.eventloop import ioloop, zmqstream

from odometry import RobotOdometry


ioloop.install()

WEB_PATH = os.path.join(os.path.dirname(__file__), "static")
ROBOT_ADDRESS = "tcp://black-pearl:7654"


class NoCacheStaticFileHandler(web.StaticFileHandler):
    '''
    This handler disables caching of any files in the given directory.
    '''
    def set_extra_headers(self, path):
        '''
        Disable caching on files served from this path.
        '''
        self.set_header('Cache-Control',
                        'no-store, no-cache, must-revalidate, max-age=0')


class IMUServer(websocket.WebSocketHandler):
    '''
    This websocket will be proxying the IMU data to the web client.
    '''
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        print("IMUServer web socket connection created")

    def _handle_imu_data(self, imu_data):
        self.write_message(imu_data.data)

    def open(self):
        print("IMU Rospy connection opened")
        rospy.init_node("imu_data", anonymous=True)
        rospy.Subscriber("imu_data", String, self._handle_imu_data)

    def on_message(self, msg):
        print(f"Unexpected messge from client: {msg}")

    def on_close(self):
        print("ROSPY connection has been closed, so sad.")
        try:
            rospy.signal_shutdown("IMUData connection closed down")
        except Exception:
            print("Failed to ROSPY connection: {}".format("imu_data"))
            traceback.print_exc()


class SensorServer(websocket.WebSocketHandler):
    '''
    This websocket will be calculating all sensor model predictions based on
    odometry and will serve it up to the web client.
    '''
    def __init__(self, *args, **kwargs):
        self._loop = kwargs.pop("loop")
        super().__init__(*args, **kwargs)
        print("Sensor server is open for business")
        self._ctx = zmq.Context()
        self._sub = None
        self._stream = None
        self._rob_odom = RobotOdometry()

    def _handle_sensor_data(self, msg_array):
        msg = json.loads(msg_array[0])
        odometry = msg["odometry"]
        left, right = odometry["left"], odometry["right"]

        if left == 0 or right == 0:
            print(f"Still getting bad readings, skipping: {odometry}")
            return

        ultrasonic = msg["ultrasonic"]
        predictions = self._rob_odom.getODOM(left, right, ultrasonic)
        self.write_message(json.dumps(predictions))

    def open(self):
        print("Connection on server has been openend")
        try:
            self._sub = self._ctx.socket(zmq.SUB)
            self._sub.setsockopt(zmq.SUBSCRIBE, b"")
            self._sub.connect(ROBOT_ADDRESS)
            self._stream = zmqstream.ZMQStream(self._sub, io_loop=self._loop)
            self._stream.on_recv(self._handle_sensor_data)
        except Exception:
            print("Failed to establish ZMQ connection: {}".format(ROBOT_ADDRESS))
            traceback.print_exc()

    def on_message(self, message):
        print("Unexpected message from web client: {}".format(message))

    def on_close(self):
        print("Connection has been closed, so sad.")
        try:
            if self._sub:
                self._sub.close()
                self._sub = None
                self._stream = None
        except Exception:
            print("Failed to close ZMQ connection: {}".format(ROBOT_ADDRESS))
            traceback.print_exc()


def make_app(loop):
    settings = {
        "static_path": WEB_PATH,
        "template_path": WEB_PATH,
        "debug": True
    }
    return web.Application([
        (r"/ws/sensors", SensorServer, {"loop": loop}),
        (r"/ws/imu_data", IMUServer),
        (r"/(.*)", NoCacheStaticFileHandler, {"path": WEB_PATH, "default_filename": "index.html"}),
        ],
        **settings)


def main():
    loop = ioloop.IOLoop.instance()
    try:
        app = make_app(loop)
        def shutdown_ros(*args):
            print("Shutting down rospy")
            rospy.signal_shutdown("Closing ROSPY")
            sys.exit()

        signal.signal(signal.SIGINT, shutdown_ros)
        app.listen(8888)
        loop.start()
    except (SystemExit, KeyboardInterrupt):
        print("Exiting due to interrupt...")
    except Exception:
        traceback.print_exc()


if __name__ == "__main__":
    main()
