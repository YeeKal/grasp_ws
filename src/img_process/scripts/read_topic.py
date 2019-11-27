import math
import csv
import numpy as np
import threading
import multiprocessing as mp
from time import sleep

#ros
import rospy
from std_msgs.msg import (
    Float64,UInt16,String,Header
)
from geometry_msgs.msg import( Pose2D)



def poseCB(data):
    writer.writerow([data.data.x,data.data.y,data.data.theta])

file=open("pose.csv",'w')
writer=csv.writer(file)
topic="haha"
rospy.init_node('read_topic')
pose_sub=rospy.Subscriber(topic,Pose2D,poseCB)
rospy.spin()