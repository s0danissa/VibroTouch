#!/usr/bin/env python2.7
import rospy
from std_msgs.msg import Float32
import numpy as np
import time
from std_msgs.msg import Float32MultiArray

arr = Float32MultiArray()
arr.data = []

velocityGlobal = np.float32()
positionGlobal = np.float32()

velocityGlobal = 0
positionGlobal = 0

def talker():
    
    while not rospy.is_shutdown():
        velocityGlobal = 20
        #for i in xrange(60,10000000):
        #    if (i%20) == 0 and (i%30) == 0:
        #        velocityGlobal = 80
        #        time.sleep(20)
	#
        #    elif (i%20) == 0 and ((i-2)%30) == 0:
        #        velocityGlobal = 60
        #        time.sleep(20)

        #    elif (i%20) == 0 and ((i-40)%30) == 0:
        #        velocityGlobal = 40
        #        time.sleep(2)


#            if (i%2) == 0:
#                arr.data.insert(0, velocityGlobal)
#                arr.data.insert(1, 0)
#                pub.publish(arr)
#                time.sleep(1)
#                arr.data[:]=[] #clear array
#            else: 
	arr.data.insert(0, velocityGlobal)
        arr.data.insert(1, 10)
        pub.publish(arr)
        arr.data[:]=[]
        time.sleep(1)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('listener1', anonymous=True)
    pub = rospy.Publisher('setData', Float32MultiArray, queue_size = 100)
    rate = rospy.Rate(8000) #10hz

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
    
