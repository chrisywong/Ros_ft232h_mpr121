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
from micropython import const

def touch():
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

	# Playing with MPR121 registers to increase sampling rate (according to AN3890), but doesn't seem to do anything
	# text = bytearray(2)
	# cap1._read_register_bytes(const(0x5C), text , 1)
	# print(hex(text[0]), ' ', bin(text[0]))
	# cap1._read_register_bytes(const(0x5D), text , 1)
	# cap1._write_register_byte(const(0x5C),0x10)
	# cap1._write_register_byte(const(0x5D),0x20)

	use_sensors = [1, 1, 1, 1] #define which sensors are being used
	#1-2: right arm
	#3-4: left arm

	for j in range(4):
		k = j + 1
		if use_sensors[j] == 1:
			print("[mpr121.py] Setting reference values for sensor ID", k)
			for i in range(12):
				ref_value[j,i] = cap[j][i].raw_value
		else:
			print("[mpr121.py] Sensor ID", k, "IS NOT BEING USED")
			for i in range(12):
				ref_value[j,i] = -1
	print("[mpr121.py] Reference values set, tactile sensors now running")

	t_last = time.time()
	while not rospy.is_shutdown():
		msg.touchSens.clear()
		for j in range(4):
			for i in range(12):
				if use_sensors[j] == 1:
					tM.val_raw = cap[j][i].raw_value
				else:
					tM.val_raw = -1

				delta = ref_value[j,i] - tM.val_raw

				if (delta>10):
					tM.touchval = True
				else:
					tM.touchval = False

				tM.channel_id = i
				tM.sensor_id = j + 1
				msg.touchSens.append(copy.copy(tM))
		rospy.loginfo(msg.touchSens)
		rospy.loginfo('[mpr121.py] Sampling time: %f', time.time() - t_last)
		t_last = time.time()
		pub.publish(msg)
		rate.sleep()

	print("[mpr121.py] Touch sensors shutting down")

if __name__ == '__main__':
    try:
        touch()
    except rospy.ROSInterruptException:
        pass
