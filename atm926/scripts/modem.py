#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
modem: control Benthos modem
subscriber: auv_common/ModemData
"""

import rospy
from auv_common.msg import ModemData
import serial

class Modem(object):
    def __init__(self):
        #変数の設定。ほぼ固定だが、別ロボットへの移植を考慮してset paramで変えられるようにしている
        self._port_name = rospy.get_param('~port_name', '/dev/Modem')
        self._SOUND_RATE = rospy.get_param('~SOUND_RATE', 5) #ATS4の値
        self._SEND_POWER = rospy.get_param('~GAIN', 1) #ATS6の値
        self._MY_ID = rospy.get_param('~MY_ID', 1) #ATS18の値
        self._PAIR_ID = rospy.get_param('~PAIR_id', 0) #ATS14の値
        self._update_rate = rospy.get_param('~rate', 5) #更新レート
        #変数の設定ここまで。set param使わないと引数２番めのデフォルト値が渡る
        self._rate = rospy.Rate(self._update_rate)
        self._port = serial.Serial(self._port_name, 9600, timeout=0.1)
        self._sub = rospy.Subscriber('modem', ModemData, self.modem_handler)
        self._port.flushInput()
        self._port.flushOutput()
        self.setup()

    def setup(self):
        # move to command mode
        self._port.write("+++\r\n")
        self._port.write("ATS4="+str(self._SOUND_RATE)+"\r\n")
        self._port.write("ATS6="+str(self._SEND_POWER)+"\r\n")
        self._port.write("ATS18="+str(self._MY_ID)+"\r\n")
        self._port.write("ATS14="+str(self._PAIR_ID)+"\r\n")
        self._port.write("cfg store"+"\r\n")
        self._port.write("ATO"+"\r\n")
        #設定時のメッセージ関連を消しておく
        self._port.flushInput()
        self._port.flushOutput()

    def read_data(self):
        # read data
        line = self._port.readline()
        print("readline:" + line)

    def run(self):
        while not rospy.is_shutdown():
            self.read_data() #モデムでデータが送られて来たらなんかする場合はこの中を実装する
            self._rate.sleep()

    def modem_handler(self, request):
        self._port.write(request.content)

if __name__ == '__main__':
    rospy.init_node('modem')
    modem = Modem()
    rospy.spin()
    #modem.run()
