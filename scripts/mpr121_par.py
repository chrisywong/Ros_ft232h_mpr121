#!/usr/bin/env python3
import rospy
from touch_pkg.msg import mpr121Msg
from touch_pkg.msg import mpr121MsgArray
import board
import busio
import time
import adafruit_mpr121
import copy
import numpy as np
import sys

from joblib import Parallel, delayed, parallel_backend

global flag_leftonly, flag_rightonly
flag_leftonly = False
flag_rightonly = False

try: #use command line arguments to
	if sys.argv[1] == '--leftonly':
		flag_leftonly = True
		print('[mpr121.py] Only touch sensors on LEFT arm are activated')
	elif sys.argv[1] == '--rightonly':
		flag_rightonly = True
		print('[mpr121.py] Only touch sensors on RIGHT arm are activated')
	else:
		print('[mpr121.py] Argument not understood')
except:
	print('[mpr121.py] No arguments given, reading touch sensors on both arms')


def readmpr(cap, j, msg):
	global flag_leftonly, flag_rightonly
	tM=mpr121Msg()
	# print("Reading %i", j)
	for i in range(12):
		if flag_leftonly == True and (j == 0 or j == 1): #if read LEFT only, then RIGHT sensors are not read (since this step takes time)
			tM.val_raw = ref_value[j,i]
		elif flag_rightonly == True and (j == 2 or j == 3): #if read RIGHT only, then LEFT sensors are not read (since this step takes time)
			tM.val_raw = ref_value[j,i]
		else:
			tM.val_raw = cap[j][i].raw_value #this is the step that takes a while

		delta=ref_value[j,i]-tM.val_raw
		if (delta>10):
			tM.touchval=True
		else :
			tM.touchval=False
		tM.channel_id=i
		tM.sensor_id=j+1
		msg.touchSens.append(copy.copy(tM))
	return

def touch():
	global msg
	global cap
	global ref_value
	print("[mpr121.py] Node initializing")
	pub = rospy.Publisher('mpr121_topic', mpr121MsgArray, queue_size=12)
	rospy.init_node('talk', anonymous=True)
	rate = rospy.Rate(100) # 10hz
	touch_value=0
	ref_value=np.zeros((4,12))
	msg=mpr121MsgArray()
	tM=mpr121Msg()
	i2c=busio.I2C(board.SCL,board.SDA)
	cap1=adafruit_mpr121.MPR121(i2c,0x5A)
	cap2=adafruit_mpr121.MPR121(i2c,0x5B)
	cap3=adafruit_mpr121.MPR121(i2c,0x5C)
	cap4=adafruit_mpr121.MPR121(i2c,0x5D)
	cap=[cap1,cap2,cap3,cap4]
	for j in range(4):
		k=j+1
		print("[mpr121.py] setting reference values for capteur ", k)
		for i in range(12):
			ref_value[j,i]=cap[j][i].raw_value
	print("[mpr121.py] reference values set, tactile sensors now running")
	t_last = time.time()

	delayed_func = [delayed(readmpr)(cap, j, msg) for j in range(4)]
	with Parallel(n_jobs=1) as parallel:
		while not rospy.is_shutdown():
			msg.touchSens.clear()
			# for j in range(4):
			# 	for i in range(12):
			# 		tM.val_raw=cap[j][i].raw_value
			# 		delta=ref_value[j,i]-tM.val_raw
			# 		if (delta>10):
			# 			tM.touchval=True
			# 		else :
			# 			tM.touchval=False
			# 		tM.channel_id=i
			# 		tM.sensor_id=j+1
			# 		msg.touchSens.append(copy.copy(tM))
			# rospy.loginfo(msg.touchSens)

			parallel(delayed_func)
			# with parallel_backend('threading', n_jobs=1):Parallel()(delayed(readmpr)(cap, j, msg) for j in range(4))

			# Parallel(n_jobs=2)(delayed(readmpr)(cap, j, msg) for j in range(4))

			rospy.loginfo('[mpr121.py] Sampling time: %f', time.time() - t_last)

			touchsensmsg = ''
			for sensor in msg.touchSens:
				if sensor.touchval == True:
					touchsensmsg = touchsensmsg + str(sensor.sensor_id)+"-"+str(sensor.channel_id)+" "
			rospy.loginfo('[mpr121.py] Touched sensors: %s', touchsensmsg)

			t_last = time.time()
			pub.publish(msg)
			rate.sleep()
	print("[mpr121.py] Touch sensors shutting down")


if __name__ == '__main__':
    try:
        touch()
    except rospy.ROSInterruptException:
        pass
