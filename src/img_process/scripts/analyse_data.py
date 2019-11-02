#!/usr/bin/env python
import csv
import matplotlib.pyplot as plt
import numpy as np

try:
    file=open("../../../imgs/hsv.csv")
except FileNotFoundError:
    print('文件不存在')
else:
    data=np.loadtxt(file,delimiter = ",",skiprows = 1)
    print(data[0])
    plt.figure(figsize=(12, 6))
    plt.subplot(1,2,1)
    plt.plot(data[:,0],data[:,1],color="blue",label="Hue")# 85-125
    plt.axvspan(85,125,facecolor='#2a5caa', alpha=0.3)
    plt.subplot(1,2,2)
    plt.plot(data[:,0],data[:,2],color="red",label="Saturation")
    plt.plot(data[:,0],data[:,3],color="green",label="Value")
    plt.axvspan(20,256,facecolor='#5c7a29', alpha=0.3)#454926

    plt.legend()

    plt.show()
    

