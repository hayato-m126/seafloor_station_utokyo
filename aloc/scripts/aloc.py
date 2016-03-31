#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
aloc: control SGK ALOC
publisher: auv_common/AlocData, SerialDebug
subscriber: auv_common/GyroData, DepthData, WaterData, AlocRequest
"""

import rospy
from auv_common.msg import GyroData, DepthData, AlocData, WaterData, SerialDebug, AlocRequest
import auv_common.auvfunc as ac
import serial, sys, time
import numpy as np

class Aloc(object):
    """
    標準的なALOCのクラス
    Stationで使うことを想定、カスタマイズはこれを継承すること
    """
    def __init__(self):
        #変数の設定。ほぼ固定だが、別ロボットへの移植を考慮してset paramで変えられるようにしている
        self._port_name = rospy.get_param('~port_name', '/dev/ALOC')
        self._BASELINE = rospy.get_param('~BASELINE', 0.45)  #デフォルトはA型用
        self._GAIN = rospy.get_param('~GAIN', 1) #デフォルトは水槽仕様
        self._TRANSMITION_LEVEL = rospy.get_param('~TRANSMITION_LEVEL', 1) #デフォルトは水槽仕様
        self._CORRELATION_THRESHOLD = rospy.get_param('~CORRELATION_THRESHOLD', 50)
        self._CORRELATION_PEAK_RATIO = rospy.get_param('~CORRELATION_PEAK_RATIO', 5)
        self._water_sound_speed = rospy.get_param('~water_sound_speed', 1500.0) #初期音速
        self._my_depth = rospy.get_param('~my_depth', 0.0) #初期深度
        self._roll = rospy.get_param('~roll', 0.0)
        self._pitch = rospy.get_param('~pitch', 0.0)
        self._heading = rospy.get_param('~heading', 0.0)
        self._depth_offset = rospy.get_param('~depth_offset', -0.65) #デフォルト値はA型
        self._update_rate = rospy.get_param('~rate', 5) #更新レート
        #変数の設定ここまで。set param使わないと引数２番めのデフォルト値が渡る
        self._rate = rospy.Rate(self._update_rate)
        #timeoutを長くとりすぎるとSerialException device reports readiness to read but returned no dataと出る。よくわからない
        self._port = serial.Serial(self._port_name, 115200, timeout=0.1) #更新間隔のタイムアウト
        self._pub = rospy.Publisher('aloc', AlocData, queue_size=self._update_rate)
        self._publish_data = AlocData()
        self._debug_pub = rospy.Publisher('alocdebug', SerialDebug, queue_size=self._update_rate)
        self._debug_publish_data = SerialDebug()
        self._debug_publish_data.publisher = "aloc"
        self._gyro_sub = rospy.Subscriber('gyro', GyroData, self.gyro_handler)
        self._depth_sub = rospy.Subscriber('depth', DepthData, self.depth_handler)
        self._water_sub = rospy.Subscriber('water', WaterData, self.water_handler)
        self._aloc_sub = rospy.Subscriber('alocrequest', AlocRequest, self.aloc_handler)

        #内部変数
        #max_delay = baseline*余裕分(1.2)/音速 1.2の根拠は仕様書
        self._MAX_DELAY = int((self._BASELINE * 1.2 * 1000000.0) / 1500.0)   #最大遅延時間[micro sec] [100-2000]
        if self._MAX_DELAY < 100:
            self._MAX_DELAY = 100
        if self._MAX_DELAY > 2000:
            self._MAX_DELAY = 2000
        #内部変数

        # write settings
        self._port.flushInput()
        self._port.flushOutput()
        self.serial_write("$d,0"+"\r") #改行コードをわけないと何故かダメだった
        # 返って来るconfigデータの中身を確認する
        line = self.readline() #$okがかえってくる
        line = self.readline() #ここで$config
        if line.startswith("$config"):
            line = line.split(',')
            self._address_me = int(line[2]) # 通信仕様書P19参照
        else:
            print("ALOC: read config error")
            sys.exit(-1)
        self.serial_write("$G,"+str(self._GAIN)+"\r")
        line = self.readline() #ここで@ G:GAINがかえってくる
        line = self.readline() #ここで$configがかえってくる
        self.serial_write("$p,"+str(self._TRANSMITION_LEVEL)+"\r")
        line = self.readline() #ここで@ tx_powerがかえってくる
        line = self.readline() #ここで$configがかえってくる
        self.serial_write("$H,"+str(self._CORRELATION_THRESHOLD)+"\r")
        line = self.readline() #ここで@ th:CORRELATION_THRESHOLDがかえってくる
        line = self.readline() #ここで$configがかえってくる
        self.serial_write("$R,"+str(self._CORRELATION_PEAK_RATIO)+"\r")
        line = self.readline() #ここで@ Xfr th:CORRELATION_PEAK_RATIOがかえってくる
        line = self.readline() #ここで$configがかえってくる
        self.serial_write("$L,"+str(self._MAX_DELAY)+"\r")
        line = self.readline() #ここで@ MaxDly:MAX_DELAY uSがかえってくる
        line = self.readline() #ここで$configがかえってくる
        self._reset_variables()

    def gyro_handler(self, gyrodata):
        """
        gyroのデータが更新されたら受け取り内部変数に反映する
        """
        self._roll = gyrodata.roll
        self._pitch = gyrodata.pitch
        self._heading = gyrodata.heading

    def depth_handler(self, depthdata):
        """
        depthのデータが更新されたら受け取り内部変数に反映する
        """
        self._my_depth = depthdata.depth + self._depth_offset

    def water_handler(self, waterdata):
        """
        waterのデータが更新されたら受け取り内部変数に反映する
        """
        self._water_sound_speed = waterdata.speed

    def aloc_handler(self, alocrequest):
        """
        alocで送信するデータを受け通り、データをセットする
        is_activeがTrueなら能動的に送信、Falseならデータセットだけ
        """
        self._transmit_data = list(alocrequest.transmit_data)
        if alocrequest.is_active:
            self.send_data(list(alocrequest.transmit_data), alocrequest.target)
        else:
            self.set_reply_data()

    def receive_data(self):
        """
        シリアル通信を監視する処理、データがあったら処理する
        """
        line = self.readline()
        if line.startswith('$rx'):
            line = line.split(',')
            self._address_target = int(line[2])
            self._receive_level = [int(line[3]), int(line[4])]
            #続いて$SBLが来るので読み込む
            time.sleep(0.5)
            line = self.readline()
            if line.startswith('$SBL'):
                print("get sbl") #for debug
                self._time_success_position = rospy.Time.now()
                line = line.split(',')
                #取得する場所は仕様書参考。仕様書が間違ってる罠が仕掛けられてる可能性は十分にあるので、ちゃんとデータも見る
                self._address_target = int(line[2])
                self._travel_time = float(line[3])
                self._hydrophone_time_difference = [float(line[4]), float(line[5]), float(line[6])]
                self._hydrophone_receive_level = [int(line[8]), int(line[9]), int(line[10]), int(line[11])]
                self._hydrophone_level_threshold = int(line[13])
                self._slant_range = self._travel_time * self._water_sound_speed / 2.0
                self._status = 4 #測位のみ成功
                time.sleep(0.5)
                #続いてdataが来るか
                line = self.readline()
                if line.startswith('$data'):
                    print("get data") #for debug
                    self._time_success_data = rospy.Time.now()
                    line = line.split(',')
                    #取得する場所は仕様書参考。仕様書が間違ってる罠が仕掛けられてる可能性は十分にあるので、ちゃんとデータも見る
                    self._address_source_data = int(line[2])
                    self._address_target_data = int(line[3])
                    self._receive_data_status = int(line[4])
                    self._source_depth = int(line[6])
                    #int("FF", 16)とやるとstringを16進と解釈してくれる
                    self._receive_data = [int(line[7],16), int(line[8],16), int(line[9],16), int(line[10],16), int(line[11],16), int(line[12],16), int(line[13],16), int(line[14],16)]
                    if self._slant_range > 0:
                        self._sound_horizontal_angle_source = ac.normalize_angle(((self._receive_data[0] << 8) | self._receive_data[1]) / 100.0)
                    self.check_valid_data()
                    #この後、3d_positionを実行してからpublishされるのでここではやらない
            else:
                self._status = 3 #通信エラー
                self.publish() #何かデータが来てた状態をログ
        elif line.startswith('$NO'):
            self._status = 2
            self.publish() #NO_REPLYだったことをログ
        else:
            #ずっとset_reply_dataするのを防ぐ
            self._status = 0

    def check_valid_data(self):
        """
        連番のチェックを行い、有効なデータかどうかを調べる
        """
        valid = False
        #相手側の処理でデータが詰めるのが追いついてないと連番が来るので正しいデータが詰まってるか調べる
        #ここnumpyでうまいことやればfor回さずに済むかも
        for i in range(0,7):
            #255と1はつながっている
            if self._receive_data[i] == 255 and self._receive_data[i+1] == 1:
                continue
            if self._receive_data[i] != self._receive_data[i+1]-1:
                valid = True
                break
        if valid:
            self._time_success_data = rospy.Time.now()
            self._status = 5 #通信成功
        else:
            self._status = 6 #データがおかしかった

    def send_data(self, content, target):
        """
        能動的にデータ送信をする。
        """
        self._time_last_call = rospy.Time.now()
        self.serial_write("$C,%d,1,%x,%x,%x,%x,%x,%x,%x,%x,%x\r\n" % (target, ac.round_depth_to_12bit(self._my_depth), content[0], content[1], content[2], content[3], content[4], content[5], content[6], content[7])) # 1は過去バージョンとの互換性のために入れてる固定値
        self._status = 1 #送信中
        self.publish()
        #self.readline() #別スレッドから呼ばれる事があるので、ここで読んではいけない。リソースの取り合いになる
        self._transmit_data = [0, 0, 0, 0, 0, 0, 0, 0]

    def set_reply_data(self):
        """
        呼ばれたときに返信するデータを詰める関数。
        発信トリガーはハード任せ
        handlerでtrasmitにデータは突っ込まれている
        """
        self.serial_write("$X,1,%x,%x,%x,%x,%x,%x,%x,%x,%x\r\n" % (ac.round_depth_to_12bit(self._my_depth), self._transmit_data[0], self._transmit_data[1], self._transmit_data[2], self._transmit_data[3], self._transmit_data[4], self._transmit_data[5], self._transmit_data[6], self._transmit_data[7]))
        self._status = -1
        self.publish() #詰めたデータ確認したい
        self.readline() #確認メッセージを消す用
        self._transmit_data = [0, 0, 0, 0, 0, 0, 0, 0]

    def correct_rotation(self):
        """
        姿勢を考慮して補正をする
        """
        roll = self._roll * np.pi / 180.0
        pitch = self._pitch * np.pi / 180.0
        heading = self._heading * np.pi / 180.0
        cosYZ = np.cos(roll)
        sinYZ = np.sin(roll)
        cosZX = np.cos(pitch)
        sinZX = np.sin(pitch)
        cosXY = np.cos(heading)
        sinXY = np.sin(heading)
        # roll補正
        self._y = self._y * cosYZ - self._z * sinYZ
        self._z = self._y * sinYZ + self._z * cosYZ
        # pitch補正
        self._z = self._z * cosZX - self._x * sinZX
        self._x = self._z * sinZX + self._x * cosZX
        # heading補正
        self._x = self._x * cosXY - self._y * sinXY
        self._y = self._x * sinXY + self._y * cosXY

    def calc_3d_position(self):
        time_diff_sec = np.array(self._hydrophone_time_difference) * 0.000001 #numpyの配列にしてMATLAB的に配列を一気に処理する。micro secをsecに直す
        time_square = np.power(time_diff_sec[0], 2) + np.power((time_diff_sec[2] - time_diff_sec[1]), 2)
        relative_depth = self._source_depth - self._my_depth
        if time_diff_sec[0] == 0.0 and (time_diff_sec[2] == time_diff_sec[1]):
            self._sound_horizontal_angle = -999.0
        else:
            self._sound_horizontal_angle = ac.normalize_angle(np.arctan2((time_diff_sec[2] - time_diff_sec[1]), time_diff_sec[0]) * 180.0 / np.pi + 45.0)
        if self._status == 5: #データ通信が成功している
            if self._slant_range != 0.0 and relative_depth != 0.0 and (self._slant_range > relative_depth):
                self._sound_vertical_angle = ac.normalize_angle(np.arctan2(np.abs(relative_depth), np.sqrt(np.power(self._slant_range, 2) - np.power(relative_depth, 2))) * 180.0 / np.pi)
                self._rotation_flag = True #データ通信に深度差がわかって、仰角がきちんと計算出来ているので、補正した方よい
            else:
                if self._water_sound_speed * np.sqrt(time_square) > self._BASELINE:
                    self._sound_vertical_angle = -999.0
                else:
                    self._sound_vertical_angle = np.arccos(self._water_sound_speed * np.sqrt(time_square) / self._BASELINE) * 180.0 / np.pi
        else:
            if self._water_sound_speed * np.sqrt(time_square) > self._BASELINE:
                self._sound_vertical_angle = -999.0
            else:
                self._sound_vertical_angle = np.arccos(self._water_sound_speed * np.sqrt(time_square) / self._BASELINE) * 180.0 / np.pi
        true_dist = np.sqrt(np.abs(np.power(self._slant_range, 2) - np.power(relative_depth, 2))) #ルートの中身がマイナスになる可能性あり
        self._target_position = [true_dist * np.cos(self._sound_horizontal_angle * np.pi / 180.0), true_dist * np.sin(self._sound_horizontal_angle * np.pi / 180.0)]
        if self._travel_time == 0 or np.abs(self._slant_range) < np.abs(relative_depth):
            self._target_position = [9999.0, 9999.0]

    def _reset_variables(self):
        """
        初期化に使う
        """
        time_zero = rospy.Time() #0の時刻データ
        self._status = 0
        self._hydrophone_level_threshold = 0
        self._time_success_position = time_zero
        self._time_success_data = time_zero
        self._time_last_call = time_zero
        #self._address_me 初期化時に本体から取得する
        self._address_target = -1
        self._address_target_data = -1
        self._address_source_data = -1
        self._receive_data_status = 2
        self._source_depth = 0
        self._receive_level = [0,0]
        self._travel_time = 0.0
        self._sound_vertical_angle = 998.0
        self._sound_horizontal_angle = 998.0
        self._sound_vertical_angle_corrected = 998.0
        self._sound_horizontal_angle_corrected = 998.0
        self._sound_horizontal_angle_source = 998.0
        self._slant_range = 0.0
        self._hydrophone_time_difference = [0.0, 0.0, 0.0] #0をセット
        self._hydrophone_receive_level = [0,0,0,0]
        #self._water_sound_speed paramで与える
        self._target_position = [0.0, 0.0]
        #self._my_depth paramで与える
        self._transmit_data = [0, 0, 0, 0, 0, 0, 0, 0]
        self._receive_data = [0, 0, 0, 0, 0, 0, 0, 0]
        self._rotation_flag = False

    def publish(self):
        """
        alocが内部に持ってるデータをpublishする。log機能相当
        """
        self._publish_data.header.stamp = rospy.Time.now()
        self._publish_data.status = self._status
        self._publish_data.hydrophone_level_threshold = self._hydrophone_level_threshold
        self._publish_data.time_success_position = self._time_success_position
        self._publish_data.time_success_data = self._time_success_data
        self._publish_data.time_last_call = self._time_last_call
        self._publish_data.address_me = self._address_me
        self._publish_data.address_target = self._address_target
        self._publish_data.address_target_data = self._address_target_data
        self._publish_data.address_source_data = self._address_source_data
        self._publish_data.receive_data_status = self._receive_data_status
        self._publish_data.source_depth = self._source_depth
        self._publish_data.receive_level = self._receive_level
        self._publish_data.travel_time = self._travel_time
        self._publish_data.sound_vertical_angle = self._sound_vertical_angle
        self._publish_data.sound_horizontal_angle = self._sound_horizontal_angle
        self._publish_data.sound_vertical_angle_corrected = self._sound_vertical_angle_corrected
        self._publish_data.sound_horizontal_angle_corrected = self._sound_horizontal_angle_corrected
        self._publish_data.sound_horizontal_angle_source = self._sound_horizontal_angle_source
        self._publish_data.slant_range = self._slant_range
        self._publish_data.hydrophone_time_difference = self._hydrophone_time_difference
        self._publish_data.hydrophone_receive_level = self._hydrophone_receive_level
        self._publish_data.water_sound_speed = self._water_sound_speed
        self._publish_data.target_position = self._target_position
        self._publish_data.my_depth = self._my_depth
        self._publish_data.transmit_data = self._transmit_data
        self._publish_data.receive_data = self._receive_data
        self._publish_data.rotation_flag = self._rotation_flag
        self._pub.publish(self._publish_data)

    def readline(self):
        """
        alocとのシリアル通信の読み取りを行い、通信内容をログする
        """
        self._debug_publish_data.header.stamp = rospy.Time.now()
        try:
            line = self._port.readline()
            self._debug_publish_data.readline = line
        except serial.SerialException:
            pass
        self._debug_pub.publish(self._debug_publish_data)
        return line

    def serial_write(self, content):
        """
        alocとのシリアル通信の書込を行い、通信内容をログする
        """
        self._port.write(content)
        self._debug_publish_data.header.stamp = rospy.Time.now()
        self._debug_publish_data.readline = content
        self._debug_pub.publish(self._debug_publish_data)

    def run(self):
        """
        Station用のAlocThreadと同様の機能
        """
        while not rospy.is_shutdown():
            self._slant_range = 0.0 #毎回書き換えてSBLの到達の判定に使う
            self.receive_data() #データ来てないときはstatus=3への変更だけが走る
            if self._status > 3:
                self.calc_3d_position()
                self.publish()
                time.sleep(0.1) #計算した後でhighにデータ投げて詰めるデータ返してもらうための待ち
            self._rate.sleep()

class MultiAloc(Aloc):
    """
    マルチAUV用の継承クラス
    元の処理の変更や、処理の追加等を実装する
    """
    #Alocにない変数を追加する場合
    Aloc._multiauv_unique_variable = "add_multiauv_unique_variable"
    def multiauv_unique_function(self):
        """
        マルチ専用関数
        """
        pass #ここを消して処理を書く
    def set_reply_data(self):
        """
        ST固有の送信データ梱包処理をオーバーライド
        """
        pass #ここを消して処理を書く
    def run(self):
        """
        STの定期処理をオーバーライド
        """
        pass #ここを消して処理を書く

if __name__ == '__main__':
    rospy.init_node('aloc')
    aloc = Aloc()
    #aloc = MultiAloc() #マルチAUV用のalocを動かすとき、aloc=Aloc()の行は消す
    aloc.run()
