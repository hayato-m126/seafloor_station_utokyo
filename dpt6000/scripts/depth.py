#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
depth: control Mensor depth sensor
publisher: auv_common/DepthData
'''

import rospy
from auv_common.msg import DepthData
import serial

class Depth(object):
    def __init__(self):
        #変数の設定。ほぼ固定だが、別ロボットへの移植を考慮してset paramで変えられるようにしている
        self._port_name = rospy.get_param('~port_name', '/dev/Depth')
        self._ATOMOSPERIC_PRESSURE = rospy.get_param('~ATOMOSPERIC_PRESSURE', 0.101325)
        self._WATER_DENSITY = rospy.get_param('~WATER_DENSITY', 1030.0)
        self._update_rate = rospy.get_param('~rate', 5) #更新レート
        #変数の設定ここまで。set param使わないと引数２番めのデフォルト値が渡る
        self._port = serial.Serial(self._port_name, 9600, timeout=0.1)
        self._pub = rospy.Publisher('depth', DepthData, queue_size=self._update_rate)
        self._publish_data = DepthData()
        self._rate = rospy.Rate(self._update_rate)

    def publish(self):
        # request data
        self._port.write("#1?\r")
        # read request data
        line = self._port.readline()
        line = line.lstrip("1    ") #1    mpaという形式なので、1から空白4個までを削除
        mpa = float(line)
        self._publish_data.header.stamp = rospy.Time.now()
        self._publish_data.pressure = mpa
        #print(str(mpa))
        # calc depth
        self._publish_data.depth = (mpa - self._ATOMOSPERIC_PRESSURE) * 1000000.0 / (self._WATER_DENSITY * 9.80665)
        self._pub.publish(self._publish_data)

    def run(self):
        while not rospy.is_shutdown():
            self.publish()
            self._rate.sleep()

if __name__ == '__main__':
    rospy.init_node('depth')
    depth = Depth()
    depth.run()
