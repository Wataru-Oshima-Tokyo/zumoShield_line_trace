#!/usr/bin/env python3
# --*-- coding: utf-8 -*-

import cv2 as cv
from sensor_msgs.msg import Image
import rospy
import cv_bridge
import numpy as np
from geometry_msgs.msg import Twist

"""
#M : 与える操作量
#M1 : 一つ前に与えた操作量
#e : 偏差(目的値と現在値の差)
#e1 : 前回の偏差
#e2 : 前々回の偏差
#Kp : 比例制御（P制御)の比例定数
#Ki : 積分制御（I制御)の比例定数
#Kd : 微分制御（D制御)の比例定数
"""



class Follower:
	def __init__(self):
		print("__init__")
		self.bridge = cv_bridge.CvBridge()
		self.M = 0.00
		self.M1 =  0.00
		self.e = 0.00
		self.e1 = 0.00
		self.e2 = 0.00
		self.Kp = 0.1
		self.Ki = 0.1
		self.Kd = 0.1
		self.cx = 0
		self.cy = 0
		self.count = 100
		#cv.namedWindow('BGR Image', 1)  #'BGR Image'という名前の画像表示のウィンドウを作成
		#cv.namedWindow('MASK', 1)   #'MASK'という名前の画像表示のウィンドウを作成
		#cv.namedWindow('MASKED', 1) #'MASK'という名前の画像表示のウィンドウを作成
		self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)   #Image型で画像トピックを購読し，コールバック関数を呼ぶ
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
		self.twist = Twist()    #Twistインスタンス生成

	def image_callback(self, msg):
		#print("I will write down codes below")
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
		hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)  #色空間の変換(BGR→HSV)
		lower_black = np.array([17, 123, 121])       #黄色の閾値（下限）
		upper_black = np.array([37, 143, 201])    #黄色の閾値（上限）
		mask = cv.inRange(hsv, lower_black, upper_black)  #閾値によるHSV画像の2値化（マスク画像生成）
		masked = cv.bitwise_and(image, image, mask = mask)  #mask画像において，1である部分だけが残る（フィルタに通している）



		h, w = image.shape[:2]
		RESIZE = (w//3, h//3)
		search_top = (h//4)*3
		search_bot = search_top + 20    #目の前の線にだけに興味がある→20行分くらいに絞る
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0
		cg = cv.moments(mask)    #maskにおける1の部分の重心
		if cg['m00'] > 0:    #重心が存在する
			self.cx = int(cg['m10']/cg['m00']) #重心のx座標
			self.cy = int(cg['m01']/cg['m00']) #重心のy座標
			cv.circle(image, (self.cx, self.cy), 20, (0, 0, 255), -1) #赤丸を画像に描画

			err = self.cx - w//2 #黄色の先の重心座標(x)と画像の中心(x)との差
			self.twist.linear.x = 0.25
			self.M = -float(err)/200 #誤差にあわせて回転速度を変化させる（-1/1000がP制御でいうところの比例ゲインにあたる）
			self.twist.angular.z = self.M
			rospy.loginfo("Linear: " + str(self.twist.linear.x) + " Angular " + str(self.twist.angular.z))
			self.cmd_vel_pub.publish(self.twist)
			self.count = 100
			#self.PIDcontrol(err)
		
		else:
			self.count -=10
			if(self.count <0):
				if(self.count <-400):
					rospy.signal_shutdown("shut down!")
				self.twist.linear.x = -0.2
				self.twist.angular.z = 1.5
				self.cmd_vel_pub.publish(self.twist)
		
                    #大きすぎるため，サイズ調整
		#print("大きすぎるため，サイズ調整")
		#self.cmd_vel_pub.publish(self.twist)
		display_mask = cv.resize(mask, RESIZE)
		display_masked = cv.resize(masked, RESIZE)
		display_image = cv.resize(image, RESIZE)

		#表示
		#print("表示")
		cv.imshow('BGR Image', display_image)   #'BGR Image'ウィンドウにimageを表示
#                 cv.imshow('MASK', display_mask)         #'MASK'ウィンドウにimageを表示
#                 cv.imshow('MASKED', display_masked)     #'MASKED'ウィンドウにimageを表示
		#cv.setMouseCallback("HSV", self.mouseEvent)
		cv.waitKey(3)   #3秒待つ


	def PIDcontrol(self, goal):
		self.twist.linear.x = 0.2
		self.M1 = self.M
		self.e2 = self.e1
		self.e1 = self.e
		self.e = goal - self.M #偏差（e） = 目的値（goal） - 前回の操作量
		self.M =  self.M1 + self.Kp * (self.e-self.e1) + self.Ki * self.e + self.Kd * ((self.e-self.e1) - (self.e1-self.e2))	
		rospy.loginfo("Linear: " + str(self.twist.linear.x) + " Angular " + str(self.twist.angular.z))
		self.twist.angular.z = self.M*0.001
		self.cmd_vel_pub.publish(self.twist)

if __name__=="__main__":
	print("Start")
	rospy.init_node('follower')
	follower = Follower()
	rospy.spin()

