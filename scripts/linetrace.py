#!/usr/bin/env python3


import cv2 as cv
from sensor_msgs.msg import Image
import rospy
import cv_bridge
import numpy as np
from geometry_msgs.msg import Twist

class Follower:
	def __init__(self):
		print("__init__")
		self.bridge = cv_bridge.CvBridge()
#		cv.namedWindow('BGR Image', 1)  #'BGR Image'という名前の画像表示のウィンドウを作成
#		cv.namedWindow('MASK', 1)   #'MASK'という名前の画像表示のウィンドウを作成
#		cv.namedWindow('MASKED', 1) #'MASK'という名前の画像表示のウィンドウを作成
		self.image_sub = rospy.Subscriber('camera/color/image_raw', Image, self.image_callback)   #Image型で画像トピックを購読し，コールバック関数を呼ぶ
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
		self.twist = Twist()    #Twistインスタンス生成
	def image_callback(self, msg):
		#print("I will write down codes below")
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
		hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)  #色空間の変換(BGR→HSV)
		lower_black = np.array([0, 0, 0])       #黒色の閾値（下限）
		upper_black = np.array([179, 128, 100])    #黒色の閾値（上限）
		mask = cv.inRange(hsv, lower_black, upper_black)  #閾値によるHSV画像の2値化（マスク画像生成）
		masked = cv.bitwise_and(image, image, mask = mask)  #mask画像において，1である部分だけが残る（フィルタに通している）



		h, w = image.shape[:2]
		RESIZE = (w//3, h//3)
		search_top = (h//4)*3
		search_bot = search_top + 20    #目の前の線にだけに興味がある→20行分くらいに絞る
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0

		M = cv.moments(mask)    #maskにおける1の部分の重心
		if M['m00'] > 0:    #重心が存在する
			cx = int(M['m10']/M['m00']) #重心のx座標
			cy = int(M['m01']/M['m00']) #重心のy座標
			cv.circle(image, (cx, cy), 20, (0, 0, 255), -1) #赤丸を画像に描画

		err = cx - w//2 #黄色の先の重心座標(x)と画像の中心(x)との差
		self.twist.linear.x = 0.2
		#self.twist.angular.z = -float(err)/100 #画像が大きいためか，-1/100では絶対値がまだ十分に大きく，ロボットが暴れてしまう
		self.twist.angular.z = -float(err)/1000 #誤差にあわせて回転速度を変化させる（-1/1000がP制御でいうところの比例ゲインにあたる）
		self.cmd_vel_pub.publish(self.twist)

		#大きすぎるため，サイズ調整
		#print("大きすぎるため，サイズ調整")
#		display_mask = cv.resize(mask, RESIZE)
#		display_masked = cv.resize(masked, RESIZE)
#		display_image = cv.resize(image, RESIZE)

		#表示
		#print("表示")
#		cv.imshow('BGR Image', display_image)   #'BGR Image'ウィンドウにimageを表示
#		cv.imshow('MASK', display_mask)         #'MASK'ウィンドウにimageを表示
#		cv.imshow('MASKED', display_masked)     #'MASKED'ウィンドウにimageを表示
		#cv.setMouseCallback("HSV", self.mouseEvent)
		cv.waitKey(3)   #3秒待つ


#Unnecessary but it will be  used in the future--------------

	def mouseEvent(self,event,x,y,flags,param):
		if event == cv.EVENT_LBUTTONUP:
			print("Clicked")
			setColorRange(0, x, y)

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

