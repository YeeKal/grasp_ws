# -*- coding=utf-8 -*- 
#!/usr/bin/env python
import csv
import matplotlib.pyplot as plt
import numpy as np
import sys

filename="pose.csv"
if len(sys.argv)>1:
    filename=sys.argv[1]



# display chinese
# plt.rcParams['font.sans-serif']=['SimHei']
# plt.rcParams['axes.unicode_minus'] = False

try:
    file=open(filename)
except FileNotFoundError:
    print("Not valid file")
else:
    data=np.loadtxt(file,delimiter = ",")
    #print(data)
    data[:,0]=data[:,0]/20
    filtered_data=[]

    #dynamic
    # Qs=[1e-3,1e-3,1e-3]
    # Rs=[0.01,0.01,0.01]
    # Ps=[1e-2,1e-2,1e-2]
    # Ks=[1e-3,1e-3,1e-3]
    Qs=[1e-3,1e-3,1e-3]
    Rs=[0.05,0.05,0.05]
    Ps=[1e-4,1e-4,1e-4]
    Ks=[1e-3,1e-3,1e-3]
    # static
    # Qs=[1e-6,1e-6,1e-6]
    # Rs=[0.01,0.01,0.01]
    # Ps=[1e-4,1e-4,1e-4]
    # Ks=[1e-3,1e-3,1e-3]
    plt.figure(figsize=(18, 9))
    num = np.size(data,0)
    sz = (num,)
    for i in range(0,3):
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
            # print(P[k])
            # print(K[k])
        filtered_data.append(xhat)
    plt.subplot(3,1,1)
    plt.plot(data[:,0],data[:,1],color="blue",label="original",linewidth=1)# 85-125
    plt.plot(data[:,0],filtered_data[0],color="red",label="filtered",linewidth=1)# 85-125
    plt.xlabel("t/s")
    plt.ylabel("x/m")
    plt.legend(loc="upper right")
    plt.text(13,0.7065,"Standard Deviation"+   \
        "\nOriginal: "+str(format(np.std(data[:,1]),'.6f'))+  \
        "\nFltered:  "+str(format(np.std(filtered_data[0]),'.6f')),   \
        bbox = dict(facecolor = "r", alpha = 0.2))
    # plt.text(13,0.7081,"Standard Deviation")
    # plt.text(13,0.7071,"Original: "+str(format(np.std(data[:,1]),'.6f')))
    # plt.text(13,0.7061,"Fltered: "+str(format(np.std(filtered_data[0]),'.6f')))


 

    plt.subplot(3,1,2)
    plt.plot(data[:,0],data[:,2],color="green",label="original",linewidth=1)# 85-125
    plt.plot(data[:,0],filtered_data[1],color="red",label="filtered",linewidth=1)# 85-125
    plt.xlabel("t/s")
    plt.ylabel("y/m")
    plt.legend(loc="upper right")
    plt.text(13,-0.635,"Standard Deviation"+   \
        "\nOriginal: "+str(format(np.std(data[:,2]),'.6f'))+  \
        "\nFltered:  "+str(format(np.std(filtered_data[1]),'.6f')),   \
        bbox = dict(facecolor = "r", alpha = 0.2))


    plt.subplot(3,1,3)
    plt.plot(data[:,0],data[:,3],color="orange",label="original",linewidth=1)# 85-125
    plt.plot(data[:,0],filtered_data[2],color="red",label="filtered",linewidth=1)# 85-125
    plt.xlabel("t/s")
    plt.ylabel(r"$\theta$/rad")
    plt.legend(loc="upper right")
    plt.text(13,-0.2125,"Standard Deviation"+   \
        "\nOriginal: "+str(format(np.std(data[:,3]),'.6f'))+  \
        "\nFltered:  "+str(format(np.std(filtered_data[2]),'.6f')),   \
        bbox = dict(facecolor = "r", alpha = 0.2))

    plt.subplots_adjust(wspace =0, hspace =0.3)
    plt.show()
