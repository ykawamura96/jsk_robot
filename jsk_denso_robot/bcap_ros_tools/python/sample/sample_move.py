# -*- coding:utf-8 -*-
#b-capを使用してRC8の内のプログラム(pro1.pcs)を操作する。
#フォルダ構成はこのプログラム(03_Task.py)と同じ
#ディレクトリにbCAPClientのpythonファイル3つが存在すること
#bcapclient.py , orinexception.py , variant.py
#b-cap Lib URL 
# https://github.com/DENSORobot/orin_bcap

from bcap_client import bcapclient
import random
import time     

#接続先RC8 の　IPアドレス、ポート、タイムアウトの設定
host = "192.168.11.4"  #Default Robot Controller IP is 192.168.0.1
port = 5007
timeout = 2000

#TCP通信の接続処理
m_bcapclient = bcapclient.BCAPClient(host,port,timeout)
print("Open Connection")

#b_cap Service を開始する
m_bcapclient.service_start("")
print("Send SERVICE_START packet")


Name = ""
Provider="CaoProv.DENSO.VRC"
Machine = ("localhost")
Option = ("Server = " + host)   #Actual Robot 
#Option = ("WPJ=*")             #Simulation 

#RC8との接続処理 (RC8(VRC)プロバイダ)
hCtrl = m_bcapclient.controller_connect(Name,Provider,Machine,Option)
print("Connect RC8")

HRobot = m_bcapclient.controller_getrobot(hCtrl,"Arm","")
print("AddRobot")

#
Command = "TakeArm"
Param = [0,1]
m_bcapclient.robot_execute(HRobot,Command,Param)
print("TakeArm")


x   = 250.0
y   = -44.50
z   = 77.65
rx  = 180.0
ry  = 0.0
rz  = 180.0
fig = 261
start_pose = '@0 P('+ str(x)  + ',' + str(y)  +',' + str(z)  + ',' \
                    + str(rx) + ',' + str(ry) +',' + str(rz) + ',' \
                    + str(fig) + ')' 
print(start_pose)
Comp=1  #ロボットの動作補間方法：PTP :1, CP:2
m_bcapclient.robot_move(HRobot,Comp,start_pose,"")
print("Arrive Start Position")

for ii in range(0,10):
    curRobotPos = m_bcapclient.robot_execute(HRobot,"CurPos")
    print('Current Robot Position :' + str(curRobotPos) )
    offSet_Z = 50.0
    poseData = '@0 P('+ str(curRobotPos[0]) + ',' + str(curRobotPos[1]) +',' + str(curRobotPos[2] + offSet_Z)  + ',' \
                      + str(curRobotPos[3]) + ',' + str(curRobotPos[4]) +',' + str(curRobotPos[5])             + ',' \
                      + str(curRobotPos[6]) + ')' 
    isMotionComp = m_bcapclient.robot_execute(HRobot,"MotionComplete",[-1,1])
    if isMotionComp == 0 :                                      #0:Moving 1:Stop
        m_bcapclient.robot_execute(HRobot,"MotionSkip",[-1,3])  #StopMove Command
    m_bcapclient.robot_move(HRobot,Comp,poseData,"Next")        #Move Command
    time.sleep(0.1)                                             #100msec Delay

print("Complete Move Motion")

Command = "GiveArm"
Param = None
m_bcapclient.robot_execute(HRobot,Command,Param)
print("GiveArm")

#切断処理
if(HRobot != 0):
    m_bcapclient.robot_release(HRobot)
    print("Release Robot")
#End If
if(hCtrl != 0):
    m_bcapclient.controller_disconnect(hCtrl)
    print("Release Controller")
#End If
m_bcapclient.service_stop()
print("B-CAP service Stop")

