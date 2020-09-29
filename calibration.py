# Acceleration and angular acceleration visualization program
# September 21, 2020 
# Yuto Nakayachi 

import sys 
import csv 
import math 
import numpy as np 
import matplotlib.pyplot as plt
import pandas as pd 


# Variable to get real value 
MPU9250A_2g  =     0.000061035156 # 0.000061035156 g/LSB
MPU9250A_4g  =     0.000122070312 # 0.000122070312 g/LSB
MPU9250A_8g  =     0.000244140625 # 0.000244140625 g/LSB
MPU9250A_16g =     0.000488281250 # 0.000488281250 g/LSB

MPU9250G_250dps  = 0.007633587786 # 0.007633587786 dps/LSB
MPU9250G_500dps  = 0.015267175572 # 0.015267175572 dps/LSB
MPU9250G_1000dps = 0.030487804878 # 0.030487804878 dps/LSB
MPU9250G_2000dps = 0.060975609756 # 0.060975609756 dps/LSB

MPU9250M_4800uT  = 0.6            # 0.6 uT/LSB

MPU9250T_85degC  = 0.002995177763 # 0.002995177763 degC/LSB
Magnetometer_Sensitivity_Scale_Factor = 0.15



yl=MPU9250A_4g * 35000
yr=MPU9250G_500dps * 35000


args = sys.argv 
title = args[1] 
theta = float(args[2])

data = pd.read_csv(title,encoding="UTF-8")
xdata = data["ms"] 
print(xdata.head())
num_data = len(xdata)
data=data.drop(data.columns[[0,1]],axis=1)
print(data.head())

data1 = data.drop(data.columns[[6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]],axis=1) #CH1
data2 = data.drop(data.columns[[0,1,2,3,4,5,12,13,14,15,16,17,18,19,20,21,22,23]],axis=1) #CH2
data3 = data.drop(data.columns[[0,1,2,3,4,5,6,7,8,9,10,11,18,19,20,21,22,23]],axis=1) #CH3
data4 = data.drop(data.columns[[0,1,2,3,4,5, 6,7,8,9,10,11, 12,13,14,15,16,17]],axis=1) #CH4 
print(data1.head())
print(data2.head()) 
print(data3.head())  
print(data4.head()) 


"""###############################################
　キャリブレーションの計算
"""
N = len(data1["ACC_X1"]) 
sin = math.sin(theta) 
cos = math.cos(theta) 
for i in range(N):
    # CH1
    if abs(data1["ACC_X1"][i] + sin)<abs(data1["ACC_X1"][i] - sin):
        x = data1["ACC_X1"][i] + sin
    else:
        x = data1["ACC_X1"][i] - sin 
    
    if abs(data1["ACC_Z1"][i] + cos ) < abs(data1["ACC_Z1"][i] - cos ):
        z = data1["ACC_Z1"][i] + cos 
    else:
        z = data1["ACC_Z1"][i] - cos 

    data1["ACC_X1"][i] = x * cos - z * sin 
    data1["ACC_Z1"][i] = x * sin + z * cos 

    # CH2 

    if abs(data2["ACC_X2"][i] + sin)<abs(data2["ACC_X2"][i] - sin):
        x = data2["ACC_X2"][i] + sin
    else:
        x = data2["ACC_X2"][i] - sin 
    
    if abs(data2["ACC_Z2"][i] + cos ) < abs(data2["ACC_Z2"][i] - cos ):
        z = data2["ACC_Z2"][i] + cos 
    else:
        z = data2["ACC_Z2"][i] - cos 

    data2["ACC_X2"][i] = x * cos - z * sin 
    data2["ACC_Z2"][i] = x * sin + z * cos 

    # CH3 

    if abs(data3["ACC_X3"][i] + sin)<abs(data3["ACC_X3"][i] - sin):
        x = data3["ACC_X3"][i] + sin
    else:
        x = data3["ACC_X3"][i] - sin 
    
    if abs(data3["ACC_Z3"][i] + cos ) < abs(data3["ACC_Z3"][i] - cos ):
        z = data3["ACC_Z3"][i] + cos 
    else:
        z = data3["ACC_Z3"][i] - cos 

    data3["ACC_X3"][i] = x * cos - z * sin 
    data3["ACC_Z3"][i] = x * sin + z * cos 

    # CH4 

    if abs(data4["ACC_X4"][i] + sin)<abs(data4["ACC_X4"][i] - sin):
        x = data4["ACC_X4"][i] + sin
    else:
        x = data4["ACC_X4"][i] - sin 
    
    if abs(data4["ACC_Z4"][i] + cos ) < abs(data4["ACC_Z4"][i] - cos ):
        z = data4["ACC_Z4"][i] + cos 
    else:
        z = data4["ACC_Z4"][i] - cos 

    data4["ACC_X4"][i] = x * cos - z * sin 
    data4["ACC_Z4"][i] = x * sin + z * cos 


for i in range(N):
    x = data1["GYRO_X1"][i] 
    z = data1["GYRO_Z1"][i] 

    data1["GYRO_X1"][i] = x * cos - z * sin 
    data1["GYRO_Z1"][i] = x * sin + z * cos 

    x = data2["GYRO_X2"][i] 
    z = data2["GYRO_Z2"][i] 

    data2["GYRO_X2"][i] = x * cos - z * sin 
    data2["GYRO_Z2"][i] = x * sin + z * cos 

    x = data3["GYRO_X3"][i] 
    z = data3["GYRO_Z3"][i] 

    data3["GYRO_X3"][i] = x * cos - z * sin 
    data3["GYRO_Z3"][i] = x * sin + z * cos 

    x = data4["GYRO_X4"][i] 
    z = data4["GYRO_Z4"][i] 

    data4["GYRO_X4"][i] = x * cos - z * sin 
    data4["GYRO_Z4"][i] = x * sin + z * cos 



"""##############################################
"""

def writeCSV():
    global buf_f 
    global title
    print("Start Create CSV File") 
    head = ["sample_cc","ms","ACC_X1","ACC_Y1","ACC_Z1","GYRO_X1","GYRO_Y1","GYRO_Z1","ACC_X2","ACC_Y2","ACC_Z2","GYRO_X2","GYRO_Y2","GYRO_Z2","ACC_X3","ACC_Y3","ACC_Z3","GYRO_X3","GYRO_Y3","GYRO_Z3","ACC_X4","ACC_Y4","ACC_Z4","GYRO_X4","GYRO_Y4","GYRO_Z4"]
    #dt_now = datetime.datetime.now() 
    title_float = "calibration_"+title 
    FILE_float = open(title_float,"w",newline="")    
    wf = csv.writer(FILE_float) 
    wf.writerow(head) 
    n = len(buf_f)
    for i in range(n):
        wf.writerow(buf_f[i]) 
    FILE_float.close() 
    print()
    print(title_float+" "+"created")
    print()
    print("Done Create CSV File") 

buf_f = [[] for i in range(num_data)]

for i in range(num_data):
    buf_f[i].append(i) 
    buf_f[i].append(xdata[i])

    buf_f[i].append(data1["ACC_X1"][i]) 
    buf_f[i].append(data1["ACC_Y1"][i])
    buf_f[i].append(data1["ACC_Z1"][i])
    buf_f[i].append(data1["GYRO_X1"][i])
    buf_f[i].append(data1["GYRO_Y1"][i])
    buf_f[i].append(data1["GYRO_Z1"][i])

    buf_f[i].append(data2["ACC_X2"][i]) 
    buf_f[i].append(data2["ACC_Y2"][i])
    buf_f[i].append(data2["ACC_Z2"][i])
    buf_f[i].append(data2["GYRO_X2"][i])
    buf_f[i].append(data2["GYRO_Y2"][i])
    buf_f[i].append(data2["GYRO_Z2"][i])

    buf_f[i].append(data3["ACC_X3"][i]) 
    buf_f[i].append(data3["ACC_Y3"][i])
    buf_f[i].append(data3["ACC_Z3"][i])
    buf_f[i].append(data3["GYRO_X3"][i])
    buf_f[i].append(data3["GYRO_Y3"][i])
    buf_f[i].append(data3["GYRO_Z3"][i])

    buf_f[i].append(data4["ACC_X4"][i]) 
    buf_f[i].append(data4["ACC_Y4"][i])
    buf_f[i].append(data4["ACC_Z4"][i])
    buf_f[i].append(data4["GYRO_X4"][i])
    buf_f[i].append(data4["GYRO_Y4"][i])
    buf_f[i].append(data4["GYRO_Z4"][i])

writeCSV()

print("theta:",theta)
print("sin:",math.sin(theta)) 
print("cos:",math.cos(theta)) 
print("sin^2+cos^2=",math.sin(theta)**2 + math.cos(theta)**2)

accx = 0
accy = 0
accz = 0
gyrox = 0
gyroy = 0 
gyroz = 0

for i in range(N):
    accx += data1["ACC_X1"][i]**2 
    #accx += abs(data1["ACC_X1"][i])
accx /= N 
accx = math.sqrt(accx) 

for i in range(N):
    accy += data1["ACC_Y1"][i]**2 
    #accy += abs(data1["ACC_Y1"][i])
accy /= N 
accy = math.sqrt(accy)

for i in range(N):
    accz += data1["ACC_Z1"][i]**2 
    #accz += abs(data1["ACC_Z1"][i])
accz /= N 
accz = math.sqrt(accz) 

print()
print("ACC")

print("EV ACC X",accx)
print("EV ACC Y",accy)
print("EV ACC Z",accz)

for i in range(N):
    gyrox += data1["GYRO_X1"][i]**2 
gyrox /= N 
gyrox = math.sqrt(gyrox) 

for i in range(N):
    gyroy += data1["GYRO_Y1"][i]**2 
gyroy /= N 
gyroy = math.sqrt(gyroy)

for i in range(N):
    gyroz += data1["GYRO_Z1"][i]**2 
gyroz /= N 
gyroz = math.sqrt(gyroz) 

print()
print("GYRO")

print("EV GYRO X",gyrox)
print("EV GYRO Y",gyroy)
print("EV GYRO Z",gyroz)

#CH1 
fig1,(axeL1,axeR1) = plt.subplots(ncols=2) 

axeL1.plot(xdata,data1["ACC_X1"],label="ACC_X") 
axeL1.plot(xdata,data1["ACC_Y1"],label="ACC_Y")
axeL1.plot(xdata,data1["ACC_Z1"],label="ACC_Z")
axeR1.plot(xdata,data1["GYRO_X1"],label="GYRO_X")
axeR1.plot(xdata,data1["GYRO_Y1"],label="GYRO_Y")
axeR1.plot(xdata,data1["GYRO_Z1"],label="GYRO_Z")
axeL1.set_title("CH1 ACC")
axeR1.set_title("CH1 GYRO") 
axeL1.set_xlabel("ms")
axeR1.set_xlabel("ms")  
axeL1.set_ylabel("Value") 
axeR1.set_ylabel("Value") 
axeL1.set_ylim(-yl,yl)
axeR1.set_ylim(-yr,yr) 
axeL1.legend() 
axeR1.legend()

#CH2
fig2,(axeL2,axeR2) = plt.subplots(ncols=2) 

axeL2.plot(xdata,data2["ACC_X2"],label="ACC_X") 
axeL2.plot(xdata,data2["ACC_Y2"],label="ACC_Y")
axeL2.plot(xdata,data2["ACC_Z2"],label="ACC_Z")
axeR2.plot(xdata,data2["GYRO_X2"],label="GYRO_X")
axeR2.plot(xdata,data2["GYRO_Y2"],label="GYRO_Y")
axeR2.plot(xdata,data2["GYRO_Z2"],label="GYRO_Z")
axeL2.set_title("CH2 ACC")
axeR2.set_title("CH2 GYRO") 
axeL2.set_xlabel("ms")
axeR2.set_xlabel("ms")  
axeL2.set_ylabel("Value") 
axeR2.set_ylabel("Value") 
axeL2.set_ylim(-yl,yl)
axeR2.set_ylim(-yr,yr) 
axeL2.legend() 
axeR2.legend()

#CH3 
fig3,(axeL3,axeR3) = plt.subplots(ncols=2) 

axeL3.plot(xdata,data3["ACC_X3"],label="ACC_X") 
axeL3.plot(xdata,data3["ACC_Y3"],label="ACC_Y")
axeL3.plot(xdata,data3["ACC_Z3"],label="ACC_Z")
axeR3.plot(xdata,data3["GYRO_X3"],label="GYRO_X")
axeR3.plot(xdata,data3["GYRO_Y3"],label="GYRO_Y")
axeR3.plot(xdata,data3["GYRO_Z3"],label="GYRO_Z")
axeL3.set_title("CH3 ACC")
axeR3.set_title("CH3 GYRO") 
axeL3.set_xlabel("ms")
axeR3.set_xlabel("ms")  
axeL3.set_ylabel("Value") 
axeR3.set_ylabel("Value") 
axeL3.set_ylim(-yl,yl)
axeR3.set_ylim(-yr,yr) 
axeL3.legend() 
axeR3.legend()

#CH4
fig4,(axeL4,axeR4) = plt.subplots(ncols=2) 

axeL4.plot(xdata,data4["ACC_X4"],label="ACC_X") 
axeL4.plot(xdata,data4["ACC_Y4"],label="ACC_Y")
axeL4.plot(xdata,data4["ACC_Z4"],label="ACC_Z")
axeR4.plot(xdata,data4["GYRO_X4"],label="GYRO_X")
axeR4.plot(xdata,data4["GYRO_Y4"],label="GYRO_Y")
axeR4.plot(xdata,data4["GYRO_Z4"],label="GYRO_Z")
axeL4.set_title("CH4 ACC")
axeR4.set_title("CH4 GYRO") 
axeL4.set_xlabel("ms")
axeR4.set_xlabel("ms")  
axeL4.set_ylabel("Value") 
axeR4.set_ylabel("Value") 
axeL4.set_ylim(-yl,yl)
axeR4.set_ylim(-yr,yr) 
axeL4.legend() 
axeR4.legend()

plt.show()