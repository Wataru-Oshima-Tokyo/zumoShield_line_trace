#!/usr/bin/env python
# --*-- coding: utf-8 -*-

import cv2 as cv
from sensor_msgs.msg import Image
import rospy
import cv_bridge
import numpy as np


class Follower:
	def __init__(self):
		print("__init__")
		self.bridge = cv_bridge.CvBridge()
# 		cv.namedWindow('BGR Image', 1)  #'BGR Image'という名前の画像表示のウィンドウを作成
# 		cv.namedWindow('MASK', 1)   #'MASK'という名前の画像表示のウィンドウを作成
# 		cv.namedWindow('MASKED', 1) #'MASK'という名前の画像表示のウィンドウを作成
		self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)   #Image型で画像トピックを購読し，コールバック関数を呼ぶ
		
	def image_callback(self, msg):
		#print("I will write down codes below")
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
		self.hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)  #色空間の変換(BGR→HSV)
		cv.namedWindow('hsv')
		cv.setMouseCallback('hsv', self.mouseEvent)
		# now click into the hsv img , and look at values:
		cv.imshow("hsv",self.hsv)
		cv.waitKey(3)
		
#Unnecessary but it will be  used in the future--------------

	def mouseEvent(self,event,x,y,flags,param):
		if event == cv.EVENT_LBUTTONUP:
			pixel= self.hsv[y,x]
			#you might want to adjust the ranges(+-10, etc):
			upper=  np.array([pixel[0] + 10, pixel[1] + 10, pixel[2] + 40])
			lower=  np.array([pixel[0] -10, pixel[1] -10, pixel[2] -40])
			rospy.loginfo("lower")
			for i in lower:
				rospy.loginfo(str(i))
			rospy.loginfo("upper")
			for i in upper:
				rospy.loginfo(str(i))
			print(pixel, lower, upper)
			image_mask= cv.inRange(image_hsv,lower,upper)
			# cv.imshow("mask",image_mask)
      

	def setColorRange(self,index, x, y):
		print("\tPOS: ", x, y)
		ret, img = cap.read()
		bgr = img[y:y + 1, x:x + 1]
		bgr00 = bgr[0][0]
		print("\tRGB: ", bgr00)
		hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV_FULL)
		hsv00 = hsv[0][0]
		print("\tHSV: ", hsv00)
		#################################################################
		#  HSV 空間における、指定色から最小値と最大値                   #
		#################################################################
		MIN_DH, MIN_DS, MIN_DV = [15,  60, 60]
		MAX_DH, MAX_DS, MAX_DV = [15, 150, 60]
		#################################################################
		global CIRCLE_PARAMS
		CIRCLE_PARAMS[index]["MIN"][0] = max([0, hsv00[0] - MIN_DH])
		CIRCLE_PARAMS[index]["MIN"][1] = max([0, hsv00[1] - MIN_DS])
		CIRCLE_PARAMS[index]["MIN"][2] = max([0, hsv00[2] - MIN_DV])
		print("\tMIN: ", np.array(CIRCLE_PARAMS[index]["MIN"]))

		CIRCLE_PARAMS[index]["MAX"][0] = min([255, hsv00[0] + MAX_DH])
		CIRCLE_PARAMS[index]["MAX"][1] = min([255, hsv00[1] + MAX_DS])
		CIRCLE_PARAMS[index]["MAX"][2] = min([255, hsv00[2] + MAX_DV])
		print("\tMAX: ", np.array(CIRCLE_PARAMS[index]["MAX"]))


	def setOpenCVParams(self):
		#################################################################
		#  カメラの高さ = 500 mm  [台からカメラレンズの下側まで]        #
		#################################################################
		HOUGH_1      = 30  # 手法依存の 1 番目のパラメータ．: CV_HOUGH_GRADIENT の場合は，
			# Canny() エッジ検出器に渡される2つの閾値の内，大きい方の閾値を表す
		HOUGH_2      = 10  # 円の中心を検出する際の投票数の閾値を表す。これが小さくなるほど，
                       # より多くの誤検出が起こる可能性がある。
                       # より多くの投票を獲得した円が，最初に出力される。
		ARM_SIZE_MIN = 12   # アームの円形のマークの最小半径  (5)
		ARM_SIZE_MAX = 25   # アームの円形のマークの最大半径  (20)
		OBJ_SIZE_MIN = 30  # 円形のピッキング・オブジェクトの最小半径  (20)
		OBJ_SIZE_MAX = 42  # 円形のピッキング・オブジェクトの最大半径  (50)
		#################################################################
		global CIRCLE_PARAMS
		CIRCLE_PARAMS = []

		CIRCLE_PARAMS.append({  # Arm
		"HOUGH": [HOUGH_1, HOUGH_2, ARM_SIZE_MIN, ARM_SIZE_MAX],
		"MIN": [0, 0, 0],
		"MAX": [0, 0, 0]})
		CIRCLE_PARAMS.append({  # Obj
		"HOUGH": [HOUGH_1, HOUGH_2, OBJ_SIZE_MIN, OBJ_SIZE_MAX],
		"MIN": [0, 0, 0],
		"MAX": [0, 0, 0]})

#--------------------------------------------------

if __name__=="__main__":
	print("Start")
	rospy.init_node('follower')
	follower = Follower()
#	follower.setOpenCVParams()
	rospy.spin()


