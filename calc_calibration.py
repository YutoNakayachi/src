#from https://github.com/mfurukawa/imu_sensor/tree/master/src/Python 
# September 03, 2020
# Yuto Nakayachi 


from __future__ import unicode_literals ,print_function
import serial 
from time import sleep 
import numpy as np 
import matplotlib.pyplot as plt
import io 
import csv 
import time 
import datetime 
import struct 
import math 


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

# number of axis 
numVariable = 24 # 4ch * 6acc 

# Maximum time for measure
minuteLength = 25 

# sampling rate 
smplHz = 500 

# Variable to count number of sampling 
smpl_cnt = 0

# Variable to count number of fail
fail_cnt_byte = 0
fail_cnt_head = 0

# Array to store data
buf = [[0 for i in range(numVariable + 2)] for j in range(smplHz*60*minuteLength)] 

# Array to store real value 
buf_f = [[0 for i in range(numVariable + 2)] for j in range(smplHz*60*minuteLength)]

# define serial port 
ser = serial.Serial("COM3",921600,timeout=1) 


# Check serial connection 
if ser.is_open:
    print("Start Serial Connection") 
else:
    print("PORT ERROR") 
    ser.close() 
    exit() 


       
# Function to Measure 
def readByte():
    global ser 
    global smpl_cnt 
    global buf 
    global buf_f  
    global xl 
    global yl 
    global fail_cnt_byte 
    global fail_cnt_head 

    ser.write(b"r") 
    time.sleep(0.01)
    ser.write(b"s") 
    time.sleep(0.01) 

    state = 0
    store = []
    while(1):
        res = ser.read()

        if state == 0 and res == b'\r':
            res=ser.read() 
            if res == b'\n':
                state = 1 
                store = []
            else:
                #fail_cnt += 1
                #print("End byte set error")
                fail_cnt_byte += 1 
                #time.sleep(2)
        
        elif state == 1:
            store.append(res) 
                
            if len(store)==50:

                # check header 
                if store[0]==b'*':
                    #print(store)
                    del store[0]
                    """
                    if smpl_cnt<5:
                        print(len(store)) 
                        print(store)
                    """
                else:
                    #print("header error") 
                    #time.sleep(2)
                    fail_cnt_head += 1 
                    state = 0
                    store = []
                    continue 

        
                #add time stamp
                if smpl_cnt==0:
                    #start_time = int.from_bytes(res[-1],"big")
                    start_time = struct.unpack("b",store[-1])[0]
                    #start_time = store[-1]
                    #print(start_time)
                    tmp_time = 0
                else:
                    #now_time = int.from_bytes(res[-1],"big") 
                    add_time = struct.unpack("b",store[-1])[0]
                    #add_time = store[-1]
                    tmp_time += add_time
                buf[smpl_cnt][1] = tmp_time
                buf_f[smpl_cnt][1] = tmp_time
                buf[smpl_cnt][0] = smpl_cnt
                buf_f[smpl_cnt][0] = smpl_cnt

                # store data 
                for i in range(0,48,2):
                    res2 = store[i:i+2] 
                    tup = struct.unpack('>h', b''.join(res2)) 
                    val = tup[0]
                    num = i%12 
                    ch = i//12 
                    buf[smpl_cnt][6*ch + num//2 + 2]=val 
                    if (num//2)>=3:
                        buf_f[smpl_cnt][6*ch + num//2 + 2]=val * MPU9250G_500dps
                    else:
                        buf_f[smpl_cnt][6*ch + num//2 + 2]=val * MPU9250A_4g

                """           
                print (
                    "{:+.3f}".format(buf_f[smpl_cnt][2]) ,      "{:+.3f}".format(buf_f[smpl_cnt][3]) ,      "{:+.3f}".format(buf_f[smpl_cnt][4]) , 
                    "{:+.3f}".format(buf_f[smpl_cnt][5]) ,  "{:+.3f}".format(buf_f[smpl_cnt][6]) ,  "{:+.3f}".format(buf_f[smpl_cnt][7]) ,'\t',
                    
                    "{:+.3f}".format(buf_f[smpl_cnt][8]) ,      "{:+.3f}".format(buf_f[smpl_cnt][9]) ,      "{:+.3f}".format(buf_f[smpl_cnt][10]) , 
                    "{:+.3f}".format(buf_f[smpl_cnt][11]) ,  "{:+.3f}".format(buf_f[smpl_cnt][12]) ,  "{:+.3f}".format(buf_f[smpl_cnt][13]) , '\t',
                    
                    "{:+.3f}".format(buf_f[smpl_cnt][14]) ,      "{:+.3f}".format(buf_f[smpl_cnt][15]) ,      "{:+.3f}".format(buf_f[smpl_cnt][16]) , 
                    "{:+.3f}".format(buf_f[smpl_cnt][17]) ,  "{:+.3f}".format(buf_f[smpl_cnt][18]) ,  "{:+.3f}".format(buf_f[smpl_cnt][19]) , '\t',
                    
                    "{:+.3f}".format(buf_f[smpl_cnt][20]) ,      "{:+.3f}".format(buf_f[smpl_cnt][21]) ,      "{:+.3f}".format(buf_f[smpl_cnt][22]) , 
                    "{:+.3f}".format(buf_f[smpl_cnt][23]) ,  "{:+.3f}".format(buf_f[smpl_cnt][24]) ,  "{:+.3f}".format(buf_f[smpl_cnt][25]) ) 
                """
                
                smpl_cnt += 1
                store = []
                state = 0

            if smpl_cnt>=5000:
                break 

# Start
print("Start Calibration")
print("ready? --> press s key") 
while(1):
    ready_s = input() 
    if ready_s == "s":
        break 
    if ready_s == "r":
        print("over") 
        ser.close()
        exit() 

# Measure the start time 
p_time = time.time() 

# Function to measure 
readByte()

# Measure the end time 
e_time = time.time() 

# The time it took 
print("time: ",e_time - p_time) 


# close serial port 
ser.close() 


# Start Calibration 
cnt = 0 
acc_calib = [[0 for j in range(3)] for i in range(4)]

for cnt in range(smpl_cnt):
    for i in range(0,48,2):
        num = i%12 
        ch = i//12 
        val = buf_f[cnt][6*ch + num//2 + 2]
        if (num//2)<3:
            acc_calib[ch][num//2] += val 

for ch in range(4):
    for num in range(3):
        acc_calib[ch][num] = acc_calib[ch][num] / smpl_cnt 
        print("CH:"+str(ch+1)+", "+str(acc_calib[ch][num])) 


print(buf_f[0])
print("CH1:",acc_calib[0]) 
print("CH2:",acc_calib[1]) 
print("CH3:",acc_calib[2]) 
print("CH4:",acc_calib[3])


RSS = 0 
for i in range(3):
    RSS += acc_calib[0][i]**2 
RSS = math.sqrt(RSS) 
theta = math.atan(acc_calib[0][0]/acc_calib[0][2])  # atan(x/z)
#fai = math.acos(acc_calib[0][2]/(RSS)) 

print("RSS:",RSS)
print("theta:",theta)
print("theta(degree):",math.degrees(theta))
#print("fai:",math.degrees(fai))

print("number of byte fail: ",fail_cnt_byte)
print("number of header fail: ",fail_cnt_head)



print("END")