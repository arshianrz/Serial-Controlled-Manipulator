#!/usr/bin/env python
import serial
import time
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String


def lsb(value):
    return value & 0xff


def msb(value):
    return value >> 8


class SerialConnection:
    def __init__(self, port, baudrate):
        self._connection = serial.Serial()
        self._connection.port = port
        self._connection.baudrate = baudrate
        # self._connection.timeout = 5
        self.open()
        self._is_open = False
        self.sub = rospy.Subscriber('/joy', Joy, self.callback)
        self.position_pitch = 512
        self.position_yaw = 512
        self.moving_speed(1, 180)
        self.moving_speed(2, 100)

    def open(self):
        self._connection.open()
        self._is_open = True

    def close(self):
        self._connection.close()
        self._is_open = False

    def _write(self, values):
        print(values)
        for b in values:
            self._connection.write(chr(b))

    def send(self, id, reg, val ):
        packets = [0xFF, 0xFF, id, 0x03 + len(val), 0x03, reg] + val
        checksum = 0
        for i in range(2, len(packets)):
            packet = packets[i]
            checksum = checksum + packet
        checksum = (~checksum) & 0xFF
        packets.append(checksum)
        self._write(packets)
        time.sleep(0.00002)

    def moving_speed(self, id, speed):
        self.send(id, 0x20, [lsb(speed), msb(speed)])

    def goal_position(self, id, position):
        self.send(id, 0x1E, [lsb(position), msb(position)])


    def callback(self, data):
        if data.buttons[1] == 1 :
            self.position_yaw = int((data.axes[0] + 1) * 512)
            if self.position_yaw < 90 :
                self.position_yaw = 90
            if self.position_yaw > 925 :
                self.position_yaw = 925
            rospy.logwarn("Yaw = %s",self.position_yaw)
            self.goal_position(1,self.position_yaw) 
            self.position_pitch = int((-data.axes[1] + 1) * 512)
            if self.position_pitch < 180:
                self.position_pitch = 180
            if self.position_pitch > 800:
                self.position_pitch = 800
            rospy.logwarn("Pitch = %s",self.position_pitch)
            self.goal_position(2,self.position_pitch)


if __name__ == '__main__':
    rospy.init_node("arm")

    serial = SerialConnection('/dev/ttyUSB0', 19230)
    rospy.spin()
