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


class ReadPose():
    def __init__(self,topic="counter_pose2d",file_name="pose.csv"):
        self.num=0
        self.topic=topic
        self.file=open(file_name,'w')
        self.writer=csv.writer(self.file)
        self.pose_sub=rospy.Subscriber(self.topic,Pose2D,self.poseCB)
        pass

    def poseCB(self,data):
        self.num =self.num+1
        self.writer.writerow([self.num,data.x,data.y,data.theta])
        if self.num%20==0:
            print("%d saved"%self.num)


if __name__ == '__main__':
    rospy.init_node('read_topic')
    reader=ReadPose("counter_pose2d","pose2.csv")
    rospy.spin()

    reader.file.close()
    print("%d saved. All completed."%reader.num)