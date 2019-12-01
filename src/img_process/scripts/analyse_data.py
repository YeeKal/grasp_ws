#!/usr/bin/env python
import csv
import matplotlib.pyplot as plt
import numpy as np

# try:
#     file=open("../../../imgs/box_small/hsv.csv")
# except FileNotFoundError:
#     print('文件不存在')
# else:
#     data=np.loadtxt(file,delimiter = ",",skiprows = 1)
#     print(data[0])
#     plt.figure(figsize=(12, 6))
#     plt.subplot(1,2,1)
#     plt.plot(data[:,0],data[0：180,1],color="blue",label="Hue")# 85-125
#     plt.axvspan(100,123,facecolor='#2a5caa', alpha=0.3)
#     plt.subplot(1,2,2)
#     plt.plot(data[:,0],data[:,2],color="red",label="Saturation")
#     plt.plot(data[:,0],data[:,3],color="green",label="Value")
#     plt.axvspan(150,255+1,facecolor='#ff7256', alpha=0.3)#454926
#     plt.axvspan(85,220,facecolor='#5c7a29', alpha=0.3)#454926


#     plt.legend()

#     plt.show()
    

# box_big: [85,125] [20,255],[20,255]
# box_small: [100,123] [150,255] [85,220]


#  to get pictures

try:
    file=open("../../../imgs/box_small/hsv.csv")
except FileNotFoundError:
    print('文件不存在')
else:
    data=np.loadtxt(file,delimiter = ",",skiprows = 1)
    print(data[0])
    plt.figure()
    plt.plot(data[0:180,0],data[0:180,1],color="blue",label="H:[100,130]")# 85-125
    plt.axvspan(100,130,facecolor='#4c96ca', alpha=0.3)#2a5caa
    plt.legend()
    plt.subplots_adjust(top = 1.0, bottom = 0.05, right = 1, left = 0.09, hspace = 0, wspace = 0)

    plt.figure()
    plt.plot(data[:,0],data[:,2],color="red",label="S:[130,255]")
    plt.axvspan(130,255+1,facecolor='#f08080', alpha=0.3)#ff7256
    plt.legend()
    plt.subplots_adjust(top = 1.0, bottom = 0.05, right = 1, left = 0.09, hspace = 0, wspace = 0)


    plt.figure()
    plt.plot(data[0:252,0],data[0:252,3],color="green",label="V:[75,230]")
    plt.axvspan(75,230,facecolor='#66cdaa', alpha=0.3)#5c7a29
    plt.legend()

    # plt.gca().xaxis.set_major_locator(plt.NullLocator())
    # plt.gca().yaxis.set_major_locator(plt.NullLocator())
    plt.subplots_adjust(top = 1.0, bottom = 0.05, right = 1, left = 0.09, hspace = 0, wspace = 0)
    # plt.margins(0,0)


    plt.show()