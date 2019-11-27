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

Qs=[1e-6,1e-6,1e-6]
Rs=[0.01,0.01,0.01]
Ps=[1e-4,1e-4,1e-4]
Ks=[1e-3,1e-3,1e-3]

xhat=0      # a posteri estimate of x 滤波估计值  
P=0.01*np.ones((3,2))       # a posteri error estimate滤波估计协方差矩阵  
xhatminus=np.zeros(sz) # a priori estimate of x 估计值  
Pminus=np.zeros(sz)    # a priori error estimate估计协方差矩阵  
K=Ks[i]*np.ones(2)         # gain or blending factor卡尔曼增益 

plt.figure(figsize=(18, 9))


class ReadPose():
    def __init__(self,topic="counter_pose2d",file_name="pose.csv"):
        self.num=0
        self.topic=topic
        self.file=open(file_name,'w')
        self.writer=csv.writer(self.file)
        self.pose_sub=rospy.Subscriber(self.topic,Pose2D,self.poseCB)
        self.pose=[0,0,0]
        

        # kalman coeff
        

    def poseCB(self,data):
        self.num =self.num+1
        self.writer.writerow([self.num,data.x,data.y,data.theta])
        if self.num%20==0:
            print("%d saved"%self.num)
        self.pose[0]=data.x
        self.pose[1]=data.y
        self.pose[2]=data.z
        self.filterOnce()
    def filterOnce(self):




if __name__ == '__main__':
    rospy.init_node('read_topic')
    reader=ReadPose("counter_pose2d","pose2.csv")
    rospy.spin()

    reader.file.close()
    print("%d saved. All completed."%reader.num)



    z = data[:,i+1] # observations (normal about x, sigma=0.1)观测值  
        
        Q = Qs[i] # process variance  
        R = Rs[i] # estimate of measurement variance, change to see effect  

        
        # 分配数组空间  
        xhat=np.zeros(sz)      # a posteri estimate of x 滤波估计值  
        P=np.zeros(sz)         # a posteri error estimate滤波估计协方差矩阵  
        xhatminus=np.zeros(sz) # a priori estimate of x 估计值  
        Pminus=np.zeros(sz)    # a priori error estimate估计协方差矩阵  
        K=Ks[i]*np.ones(sz)         # gain or blending factor卡尔曼增益  
        
        
        # intial guesses  
        xhat[0] = z[0] 
        P[0] = Ps[i]  
        
        for k in range(1,num):  
            # 预测  
            xhatminus[k] = xhat[k-1]  #X(k|k-1) = AX(k-1|k-1) + BU(k) + W(k),A=1,BU(k) = 0  
            Pminus[k] = P[k-1]+Q      #P(k|k-1) = AP(k-1|k-1)A' + Q(k) ,A=1  
        
            # 更新  
            K[k] = Pminus[k]/( Pminus[k]+R ) #Kg(k)=P(k|k-1)H'/[HP(k|k-1)H' + R],H=1  
            xhat[k] = xhatminus[k]+K[k]*(z[k]-xhatminus[k]) #X(k|k) = X(k|k-1) + Kg(k)[Z(k) - HX(k|k-1)], H=1  
            P[k] = (1-K[k])*Pminus[k] #P(k|k) = (1 - Kg(k)H)P(k|k-1), H=1  
    
