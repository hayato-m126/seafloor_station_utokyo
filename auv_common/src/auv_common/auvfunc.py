#!/usr/bin/env python
# -*- coding: utf-8 -*-

#共通で読み込むべきアドレス表
ADDRESS_STATION = 4
ADDRESS_TRIDOG = 5
ADDRESS_TRITON = 6
ADDRESS_TRITON2 = 7
ADDRESS_SURFACE_UNIT = 11

def normalize_angle(angle):
    if angle > 180.0:
        angle -= 360.0
    return angle

def round_angle_to_4bit(angle):
    send_angle = int((angle + 16.0) / 2.0)
    if send_angle > 15:
        send_angle = 15 #16deg以上ついたら全部16degとみなす
    elif send_angle < 0:
        send_angle = 0 #-16deg以下なら-16degとみなす
    return send_angle

def round_angle_to_8bit(angle):
    if angle < 0:
        angle += 360.0
    send_angle = int(angle * 256.0/360.0)
    return send_angle

def round_angle_to_16bit(angle):
    if angle < 0:
        angle += 360.0 #一旦0-360にスケールを戻す
    int_angle = int(angle * 100.0) #36000までなら4byte分使わないから符号は気にしなくて大丈夫
    upper = (int_angle & 0xFF00) >> 8
    lower = int_angle & 0x00FF
    return [upper, lower]

def round_altitude_to_8bit(altitude):
    send_altitude = int(altitude * 10.0) #altitudeにマイナスはない
    if send_altitude > 255:
        send_altitude = 255
    return send_altitude

def round_depth_to_12bit(depth):
    if depth < 0.0: #空中に飛び出てる
        depth = 0.0
    elif depth > 4095.0:#桁あふれするので丸める。これより深い所にはそもそも行けないので問題ない
        depth = 4095.0
    return int(depth)

def round_depth_to_16bit(depth):
    int_depth = int(depth * 30.0)
    upper = (int_depth & 0xFF00) >> 8
    lower = int_depth & 0x00FF
    return [upper, lower]
