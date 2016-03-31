#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
gyro: control Ocean Server OS5000
publisher: auv_common/GyroData
"""

import rospy
from auv_common.msg import GyroData
import auv_common.auvfunc as ac
import serial, time

class Gyro(object):
    def __init__(self):
        #変数の設定。ほぼ固定だが、別ロボットへの移植を考慮してset paramで変えられるようにしている
        self._port_name = rospy.get_param('~port_name', '/dev/OceanServer')
        self._OUTPUT_RATE = rospy.get_param('~OUTPUT_RATE', "\x1B\x52\x35\r\n")  # <ESC>R5<crcf>  5Hz (max: 40Hz)
        self._OUTPUT_DATA = rospy.get_param('~OUTPUT_DATA', "\x1B\x58\x31\x35\r\n") # <ESC>X15<crcf> output heading, pitch, roll, temp
        self._OUTPUT_FORMAT = rospy.get_param('~OUTPUT_FORMAT', "\x1B\x2A\x32\r\n") # <ESC>*2<crcf> format 2 '$OHPR,heading,pitch,roll,temperature*checksum'
        #self._MOUNT = rospy.get_param('~MOUNT', "\x1B\x45\x36\r\n") #変更毎にキャリブレーションが必要とマニュアルにあるので、手動変更だけにする
        self._roll_offset = rospy.get_param('~roll_offset', 0.0)
        self._pitch_offset = rospy.get_param('~pitch_offset', 0.0)
        self._heading_offset = rospy.get_param('~heading_offset', 0.0)
        self._update_rate = rospy.get_param('~rate', 5) #更新レート
        #変数の設定ここまで。set param使わないと引数２番めのデフォルト値が渡る
        self._rate = rospy.Rate(self._update_rate)
        self._port = serial.Serial(self._port_name, 19200, timeout=0.1) # default 19200, N, 8, 1 更新レートより短いtimeout設定しとかないとダメだと思う
        self._pub = rospy.Publisher('gyro', GyroData, queue_size=self._update_rate)
        self._publish_data = GyroData()
        # write settings
        self._port.flushInput()
        self._port.flushOutput()
        time.sleep(1) #ここの待ち時間が短すぎると設定が書き込まれず、何も出てこない
        self._port.write(self._OUTPUT_RATE)
        time.sleep(1) #ここの待ち時間が短すぎると設定が書き込まれず、何も出てこない
        self._port.write(self._OUTPUT_DATA)
        time.sleep(1) #ここの待ち時間が短すぎると設定が書き込まれず、何も出てこない
        #self._port.write(self._MOUNT)
        #time.sleep(0.2)
        self._port.write(self._OUTPUT_FORMAT)
        time.sleep(1) #ここの待ち時間が短すぎると設定が書き込まれず、何も出てこない
        #設定時のメッセージ関連を消しておく
        self._port.flushInput()
        self._port.flushOutput()

    def publish(self):
        # read data. data is uninterruptedly
        line = self._port.readline()
        # line format likes '$OHPR,heading,roll,ptich,temperature*num'
        if line.startswith('$OHPR'):
            self._publish_data.header.stamp = rospy.Time.now()
            line = line.replace('*', ',')
            line = line.split(',')
            self._publish_data.roll = ac.normalize_angle(float(line[3]) + self._roll_offset)
            self._publish_data.pitch = ac.normalize_angle(float(line[2]) + self._pitch_offset)
            self._publish_data.heading = ac.normalize_angle(float(line[1]) + self._heading_offset)
            self._publish_data.temperature = float(line[4])
            self._pub.publish(self._publish_data)
        else:
            print('Gyro OS5000: Read error: ' + line)

    def run(self):
        while not rospy.is_shutdown():
            self.publish()
            self._rate.sleep()

if __name__ == '__main__':
    rospy.init_node('gyro')
    gyro = Gyro()
    gyro.run()
