#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
current: control JEF Advantec electromagnetic current meter
publisher: auv_common/CurrentData, WaterData
subscriber: auv_common/DepthData
'''

import rospy
from auv_common.msg import CurrentData, WaterData, DepthData
import serial
import numpy as np

class Current(object):
    def __init__(self):
        #変数の設定。ほぼ固定だが、別ロボットへの移植を考慮してset paramで変えられるようにしている
        self._port_name = rospy.get_param('~port_name', '/dev/Current')
        self._water_salinity = rospy.get_param('~water_salinity', 34.0)
        self._depth = rospy.get_param('~depth', 0.0)
        self._rate = rospy.get_param('~rate', 1) #更新レート
        #変数の設定ここまで。set param使わないと引数２番めのデフォルト値が渡る
        self._port = serial.Serial(self._port_name, 38400, timeout=0.5)
        self._current_pub = rospy.Publisher('current', CurrentData, queue_size=self._update_rate)
        self._water_pub = rospy.Publisher('water', WaterData, queue_size=self._update_rate)
        self._current_pub_data = CurrentData()
        self._water_pub_data = WaterData()
        self._depth_sub = rospy.Subscriber('depth', DepthData, self.depth_handler)

    def depth_handler(self, depthdata):
        self._depth = depthdata.depth

    def publish(self):
        # request data
        self._port.write("\x11\x11pval,\r")
        # read request data
        line = self._port.readline()
        # line format likes '\x11\x11pval,data1,data2,....'
        if line.startswith("\x11\x11paval,"):
            #print(line)
            line = line.split(',')
            #流速系データを投げる
            time_now = rospy.Time.now()
            water_temperature = float(line[1])
            self._current_pub_data.header.stamp = time_now
            self._current_pub_data.water_temperature = water_temperature
            self._current_pub_data.compass_a = float(line[2])
            self._current_pub_data.compass_b = float(line[3])
            self._current_pub_data.x_velocity = float(line[4])
            self._current_pub_data.y_velocity = float(line[5])
            self._current_pub_data.current_velocity = float(line[6])
            self._current_pub_data.currentmeter_direction = float(line[7]) + 90.0 #流速計の仕様でy-の方向が出るのでxに合わせる
            self._current_pub_data.current_direction = float(line[8])
            self._current_pub_data.north_velocity = float(line[9])
            self._current_pub_data.east_velocity = float(line[10])
            self._current_pub_data.voltage = float(line[11])
            self._current_pub.publish(self._current_pub_data)
            #水データを投げる
            self._water_pub_data.header.stamp = time_now
            self._water_pub_data.speed = self.calc_speed(water_temperature)
            self._water_pub_data.temperature = water_temperature
            self._water_pub_data.salinity = self._water_salinity
            self._water_pub_data.reliability = False
            self._water_pub_data.location = [0.0, 0.0, 0.0] #STは動かない
            self._water_pub.publish(self._water_pub_data)
        else:
            print 'Current: Read error'

    def calc_speed(self, water_temperature):
        speed = 1449.2 + 4.6 * water_temperature - 5.5 * 0.01 * np.pow(water_temperature,2) + 2.9 * 0.0001 * np.pow(water_temperature,3) + (1.34 - 0.01 * water_temperature) * (self._water_salinity - 35.0) + 1.58 * 0.01 * self._depth)
        return speed

    def run(self)
        while not rospy.is_shutdown():
            self.publish()
            self._rate.sleep()

if __name__ == '__main__':
    rospy.init_node('current')
    current = Current()
    current.run()
