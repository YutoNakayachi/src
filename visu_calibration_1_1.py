# acc calibration 
# ver 1.1
# Oct 02, 2020 
# 回転行列をかけてキャリブレーションを行った結果を図にするプログラム

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

# 回転行列
#rotmat = np.array([[ 0.53915493,0.60957543,-0.58114521], [ 0.70248984, -0.70611601, -0.08892804],[-0.46456429, -0.36030262 ,-0.80892648]])
#rotmat = np.array([[ 0.46519735 , 0.46413067 ,-0.75377327] , [ 0.47835842 ,-0.84829011, -0.22710593] , [-0.74482524 ,-0.25492472 ,-0.61664313]])
rotmat = np.array([[-0.83786996 , 0.46555038 ,-0.28502067],
 [-0.34834086, -0.85801171 ,-0.37745801],
 [-0.42027679, -0.21697638  ,0.88107245]])

args = sys.argv 
title = args[1] 

data = pd.read_csv(title,encoding="UTF-8")
xdata = data["ms"] 
print(xdata.head())
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


# キャリブレーションの計算を行う

N = len(data1["ACC_X1"]) 

# acc calibration 
for i in range(N):
    x = data1["ACC_X1"][i] 
    y = data1["ACC_Y1"][i]
    z = data1["ACC_Z1"][i]
    a_s = np.array([x,y,z])
    a_s = a_s.T 
    w = np.dot(rotmat,a_s) 

    data1["ACC_X1"][i] = w[0] 
    data1["ACC_Y1"][i] = w[1] 
    data1["ACC_Z1"][i] = w[2] - 1 # 重力加速度分を引く


# gyro calibration 
for i in range(N):
    x = data1["GYRO_X1"][i] 
    y = data1["GYRO_Y1"][i]
    z = data1["GYRO_Z1"][i]
    a_s = np.array([x,y,z])
    a_s = a_s.T 
    w = np.dot(rotmat,a_s) 

    data1["GYRO_X1"][i] = w[0] 
    data1["GYRO_Y1"][i] = w[1] 
    data1["GYRO_Z1"][i] = w[2]


accx = 0
accy = 0
accz = 0
gyrox = 0
gyroy = 0 
gyroz = 0

for i in range(N):
    accx += data1["ACC_X1"][i]**2 
    
accx /= N 
accx = math.sqrt(accx) 

for i in range(N):
    accy += data1["ACC_Y1"][i]**2 
  
accy /= N 
accy = math.sqrt(accy)

for i in range(N):
    accz += data1["ACC_Z1"][i]**2 
  
accz /= N 
accz = math.sqrt(accz) 

print("\nACC")

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

print("\nGYRO")

print("EV GYRO X",gyrox)
print("EV GYRO Y",gyroy)
print("EV GYRO Z",gyroz)

#CH1 
fig1,(axeL1,axeR1) = plt.subplots(ncols=2) 
# ms -> s 
xdata = xdata / 1000

axeL1.plot(xdata,data1["ACC_X1"],label="ACC_X") 
axeL1.plot(xdata,data1["ACC_Y1"],label="ACC_Y")
axeL1.plot(xdata,data1["ACC_Z1"],label="ACC_Z")
axeR1.plot(xdata,data1["GYRO_X1"],label="GYRO_X")
axeR1.plot(xdata,data1["GYRO_Y1"],label="GYRO_Y")
axeR1.plot(xdata,data1["GYRO_Z1"],label="GYRO_Z")
axeL1.set_title("CH1 ACC")
axeR1.set_title("CH1 GYRO") 
axeL1.set_xlabel("Time [s]")
axeR1.set_xlabel("Time [s]")  
axeL1.set_ylabel("Acceleration [G]") 
axeR1.set_ylabel("Anguler Velocity [degree per sec]") 
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
axeL2.set_xlabel("Time [s]")
axeR2.set_xlabel("Time [s]")  
axeL2.set_ylabel("Acceleration [G]") 
axeR2.set_ylabel("Anguler Velocity [degree per sec]") 
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
axeL3.set_xlabel("Time [s]")
axeR3.set_xlabel("Time [s]")  
axeL3.set_ylabel("Acceleration [G]") 
axeR3.set_ylabel("Anguler Velocity [degree per sec]") 
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
axeL4.set_xlabel("Time [s]")
axeR4.set_xlabel("Time [s]")  
axeL4.set_ylabel("Acceleration [G]") 
axeR4.set_ylabel("Anguler Velocity [degree per sec]") 
axeL4.set_ylim(-yl,yl)
axeR4.set_ylim(-yr,yr) 
axeL4.legend() 
axeR4.legend()

plt.show()