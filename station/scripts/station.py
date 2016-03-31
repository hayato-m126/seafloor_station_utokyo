#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
station: High level program
publisher: auv_common/ModemData, LedData, AlocRequestm StationData
subscriber: auv_common/AlocData
'''

import rospy
from auv_common.msg import DepthData, GyroData, CurrentData, ModemData, LedData, AlocRequest, AlocData, StationData
import auv_common.auvfunc as ac
import time, sys
import numpy as np

class Station(object):
    def __init__(self):
        #変数の設定。ほぼ固定だが、別ロボットへの移植を考慮してset paramで変えられるようにしている
        self._COMMAND_TIMEOUT = rospy.get_param('~COMMAND_TIMEOUT', 60.0)
        self._update_rate = rospy.get_param('~rate', 5) #更新レート
        #変数の設定ここまで。set param使わないと引数２番めのデフォルト値が渡る
        self._modem_pub = rospy.Publisher('modem', ModemData, queue_size=self._update_rate)
        self._modem_pub_data = ModemData()
        self._led_pub = rospy.Publisher('led', LedData, queue_size=self._update_rate)
        self._led_pub_data = LedData()
        self._aloc_pub = rospy.Publisher('alocrequest', AlocRequest, queue_size=self._update_rate)
        self._aloc_pub_data = AlocRequest()
        self._station_pub = rospy.Publisher('station', StationData, queue_size=self._update_rate)
        self._station_pub_data = StationData()
        self._aloc_sub = rospy.Subscriber('aloc', AlocData, self.aloc_handler)
        self._gyro_sub = rospy.Subscriber('gyro', GyroData, self.gyro_handler)
        self._rate = rospy.Rate(self._update_rate)
        self._reset_variables()

    def transmit_data_by_aloc(self, content, target, is_active):
        """
        alocにデータ送信を要求する
        contentは1byteのデータ8個の配列
        targetは送信先のalocの番号
        is_activeはTrueが能動的に送信、Falseは$Xでデータ詰めるだけ
        """
        self._aloc_pub_data.header.stamp = rospy.Time.now()
        self._aloc_pub_data.transmit_data = content
        self._aloc_pub_data.target = target
        self._aloc_pub_data.is_active = is_active
        self._aloc_pub.publish(self._aloc_pub_data)

    def transmit_data_by_modem(self, content):
        self._modem_pub_data.header.stamp = rospy.Time.now()
        self._modem_pub_data.content = content
        self._modem_pub.publish(self._modem_pub_data)

    def set_led(self, on_off):
        self._led_pub_data.header.stamp = rospy.Time.now()
        self._led_pub_data.on_flag = on_off
        if on_off:
            self._led_pub_data.node[0].r = 255
            self._led_pub_data.node[0].g = 0
            self._led_pub_data.node[0].b = 0
            self._led_pub_data.node[1].r = 0
            self._led_pub_data.node[1].g = 255
            self._led_pub_data.node[1].b = 0
            self._led_pub_data.node[2].r = 0
            self._led_pub_data.node[2].g = 0
            self._led_pub_data.node[2].b = 255
            self._led_pub_data.node[3].r = 0
            self._led_pub_data.node[3].g = 0
            self._led_pub_data.node[3].b = 0
        self._led_pub.publish(self._led_pub_data)

    def gyro_handler(self, gyrodata):
        self._roll = gyrodata.roll
        self._pitch = gyrodata.pitch
        self._heading = gyrodata.heading

    def aloc_handler(self, alocdata):
        """
        alocからのデータを受け取って、それに応じてデータを詰め込む
        0:Me to Vehicle Direction 100倍整数の上位bit
        1:Me to Vehicle Direction 100倍整数の下位bit
        2:上位 16方位roll,下位 16方位pitch
        【相手がSSBL船上局】
        3:上位 16方位STのheading, 下位 電圧未使用
        4:300刻みのauvのmissionタイム(10→3000ミッションタイム)
        5:AUVのモード
        6:0.1m刻みのAUVの高度(10→高度1m)
        7:256段階のAUVのheading
        【相手がAUV】
        3:256段階の自分のheading
        4:30倍の整数の深度の上位bit
        5:30倍の整数の深度の下位bit
        6:コマンド
        7:コマンド（文字化け対策のため同じもの）
        """
        self._my_depth = alocdata.my_depth
        self._receive_data = list(alocdata.receive_data)
        if alocdata.status > 3: #エラーとかでも吐かれる。データを詰める条件を確認する
            #受け取ったデータの処理
            if alocdata.address_source_data != ac.ADDRESS_SURFACE_UNIT:
                #いちいちデコードするのは無駄なので、publish(ログ)するときにデコードする
                self._source_mission_time = self._receive_data[4]
                self._source_mode = self._receive_data[5]
                self._source_altitude = self._receive_data[6]
                self._source_heading = self._receive_data[7]
            elif self._receive_data[0] == 255 and self._receive_data[1] == 255 and self._receive_data[2] == 255 and self._receive_data[3] == 255 and self._receive_data[6] == self._receive_data[7] and self._receive_data[6] > 0:
                #判定の順番考えた方が高速になるかも↑
                self._time_catch_bypass_command = rospy.Time.now()
                self._bypass_command = self._receive_data[6]
            #送信データの返却
            self._transmit_data[0], self._transmit_data[1] = ac.round_angle_to_16bit(alocdata.sound_horizontal_angle)
            send_roll = ac.round_angle_to_4bit(self._roll)
            send_pitch = ac.round_angle_to_4bit(self._pitch)
            self._transmit_data[2] = send_roll << 4 | send_roll #論理和
            if alocdata.address_source_data == ac.ADDRESS_SURFACE_UNIT: #船上局
                send_heading = ac.round_angle_to_4bit(self._heading)
                send_voltage = 0 #電圧測ってないので固定値入れておく
                self._transmit_data[3] = send_heading << 4 | send_voltage #論理和
                self._transmit_data[4] = self._source_mission_time / 300
                self._transmit_data[5] = self._source_mode
                self._transmit_data[6] = ac.round_altitude_to_8bit(self._source_altitude)
                self._transmit_data[7] = ac.round_angle_to_8bit(self._source_direction)
            else: #相手がAUV
                self._transmit_data[3] = ac.round_angle_to_8bit(self._heading)
                #ここで深度詰めているから12bit分別のデータ詰められる、無駄がある
                self._transmit_data[4], self._transmit_data[5] = ac.round_depth_to_16bit(self._my_depth)
                #ここで深度詰めているから12bit分別のデータ詰められる、無駄がある
                if rospy.Time.now() < self._time_catch_bypass_command + rospy.Duration(self._COMMAND_TIMEOUT):
                    self._transmit_data[6] = self._bypass_command
                    self._transmit_data[7] = self._bypass_command
                else:
                    self._transmit_data[6] = 0
                    self._transmit_data[7] = 0
            self.transmit_data_by_aloc(self._transmit_data, 1, False)
            self.st_publish()

    def _reset_variables(self):
        self._time_catch_bypass_command = rospy.Time()
        self._transmit_data = [0, 0, 0, 0, 0, 0, 0, 0]
        self._receive_data = [0, 0, 0, 0, 0, 0, 0, 0]
        self._source_mission_time = 0
        self._source_mode = 0
        self._source_altitude = 0
        self._source_heading = 0
        self._bypass_command = 0
        self._roll = 0.0
        self._pitch = 0.0
        self._heading = 0.0
        self._my_depth = 0.0

    def st_publish(self):
        """
        STデータのパブリッシュ
        """
        self._station_pub_data.header.stamp = rospy.Time.now()
        self._station_pub_data.time_catch_bypass_command = self._time_catch_bypass_command
        self._station_pub_data.transmit_data = self._transmit_data
        self._station_pub_data.receive_data = self._receive_data
        self._station_pub_data.source_mission_time = self._source_mission_time * 300 #人間が見やすいようにデコードする
        self._station_pub_data.source_mode = self._source_mode
        self._station_pub_data.source_altitude = self._source_altitude * 0.1 #人間が見やすいようにデコードする
        self._station_pub_data.source_heading = self._source_heading * 360.0 /256.0 #人間が見やすいようにデコードする
        self._station_pub_data.bypass_command = self._bypass_command
        self._station_pub.publish(self._station_pub_data)

    def run(self):
        while not rospy.is_shutdown():
            #ここに定期処理を書く
            #すぐにデータ送信を呼びに行っても接続が確立してないので、受け取らないのでまず待つ
            #time.sleep(5)
            #station.transmit_data_by_aloc(np.random.randint(0,255,8), 6, True)
            #station.transmit_data_by_aloc([0, 1, 2, 3, 4, 5, 6, 7], 6, True)
            self._rate.sleep()

if __name__ == '__main__':
    rospy.init_node('station')
    station = Station()
    station.run()
