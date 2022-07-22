#!/usr/bin/env python3
import struct
from struct import pack
from tabnanny import check
import time
import numpy as np
import serial
from ctypes import *
import rospy
from std_msgs.msg import Float32MultiArray

rx = []
COMPort = '/dev/ttyACM0'   ##use sudo dmesg | grep tty to look for the correct port name

ser = serial.Serial()
ser.port = COMPort
ser.baudrate = 750000  ## can change to other value
ser.timeout = 30  ## can change to other value
try:
    ser.open()
    ser.flushInput()
except serial.SerialException as var:
    print("An exception occurred when try to open serial port")
    print("Exception details-> ", var)
else:
    print("Serial port opened")


## send the any input data to retrieve the sensor data through serial communication protocol
packet = bytearray()
packet.append(0xAA)
packet.append(0xCC)
packet.append(0x16)
packet.append(0x4D)
packet.append(0x31)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x00)
packet.append(0x94)
packet.append(0x55)

def main():
    pub = rospy.Publisher('sensor_data', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(5)
    sensor_reading = Float32MultiArray()
    while not rospy.is_shutdown():
        ser.write(packet)
        rx = ser.read(52)
        left_1=[]
        left_2=[]
        left_3=[]
        left_4=[]
        left_5=[]
        left_6=[]
        right_1=[]
        right_2=[]
        right_3=[]
        right_4=[]
        right_5=[]
        right_6=[]
        zero = '0'
        checksum = 0
        for i in range(2,50,1):   ##verify data
            if checksum > 255:
                checksum = checksum - 256
            checksum = checksum + rx[i]

        if checksum == rx[50]:
            print('checksum, data =',rx[50],checksum)
            for i in range(5,1,-1):    
                s1_c1 = hex(rx[i])    ## convert data from hex to float
                if len(s1_c1)!=4:
                    s1_c1 += zero
                left_1.append(s1_c1[2:])
            S1_CH_1 = struct.unpack('!f', bytes.fromhex("".join(left_1)))[0]

            for i in range(9,5,-1):
                s1_c2 = hex(rx[i])
                if len(s1_c2)!=4:
                    s1_c2 += zero
                left_2.append(s1_c2[2:])
            S1_CH_2 = struct.unpack('!f', bytes.fromhex("".join(left_2)))[0]

            for i in range(13,9,-1):
                s1_c3 = hex(rx[i])
                if len(s1_c3)!=4:
                    s1_c3 += zero
                left_3.append(s1_c3[2:])
            S1_CH_3 = struct.unpack('!f', bytes.fromhex("".join(left_3)))[0]

            for i in range(17,13,-1):
                s1_c4 = hex(rx[i])
                if len(s1_c4)!=4:
                    s1_c4 += zero
                left_4.append(s1_c4[2:])
            S1_CH_4 = struct.unpack('!f', bytes.fromhex("".join(left_4)))[0]

            for i in range(21,17,-1):
                s1_c5 = hex(rx[i])
                if len(s1_c5)!=4:
                    s1_c5 += zero
                left_5.append(s1_c5[2:])
            S1_CH_5 = struct.unpack('!f', bytes.fromhex("".join(left_5)))[0]

            for i in range(25,21,-1):
                s1_c6 = hex(rx[i])
                if len(s1_c6)!=4:
                    s1_c6 += zero
                left_6.append(s1_c6[2:])
            S1_CH_6 = struct.unpack('!f', bytes.fromhex("".join(left_6)))[0]
            
            for i in range(29,25,-1):
                s2_c1 = hex(rx[i])
                if len(s2_c1)!=4:
                    s2_c1 += zero
                right_1.append(s2_c1[2:])
            S2_CH_1 = struct.unpack('!f', bytes.fromhex("".join(right_1)))[0]

            for i in range(33,29,-1):
                s2_c2 = hex(rx[i])
                if len(s2_c2)!=4:
                    s2_c2 += zero
                right_2.append(s2_c2[2:])
            S2_CH_2 = struct.unpack('!f', bytes.fromhex("".join(right_2)))[0]

            for i in range(37,33,-1):
                s2_c3 = hex(rx[i])
                if len(s2_c3)!=4:
                    s2_c3 += zero
                right_3.append(s2_c3[2:])
            S2_CH_3 = struct.unpack('!f', bytes.fromhex("".join(right_3)))[0]
            
            for i in range(41,37,-1):
                s2_c4 = hex(rx[i])
                if len(s2_c4)!=4:
                    s2_c4 += zero
                right_4.append(s2_c4[2:])
            S2_CH_4 = struct.unpack('!f', bytes.fromhex("".join(right_4)))[0]

            for i in range(45,41,-1):
                s2_c5 = hex(rx[i])
                if len(s2_c5)!=4:
                    s2_c5 += zero
                right_5.append(s2_c5[2:])
            S2_CH_5 = struct.unpack('!f', bytes.fromhex("".join(right_5)))[0]
            
            for i in range(49,45,-1):
                s2_c6 = hex(rx[i])
                if len(s2_c6)!=4:
                    s2_c6 += zero
                right_6.append(s2_c6[2:])
            S2_CH_6 = struct.unpack('!f', bytes.fromhex("".join(right_6)))[0]
            S2_CH_6 = (S2_CH_6 - 1.63)*2    ###because this channel will give half of the voltage supply, so need to calibrate accordingly

            sensor_reading.data = np.array([S1_CH_1,S1_CH_2,S1_CH_3,S1_CH_4,S1_CH_5,S1_CH_6,S2_CH_1,S2_CH_2,S2_CH_3,S2_CH_4,S2_CH_5,S2_CH_6])
            pub.publish(sensor_reading)
    
            
            left_1.clear()
            left_2.clear()
            left_3.clear()
            left_4.clear()
            left_5.clear()
            left_6.clear()
            right_1.clear()
            right_2.clear()
            right_3.clear()
            right_4.clear()
            right_5.clear()
            right_6.clear()
        #    except:
        #        continue
            rate.sleep()
        else:
            pass

if __name__ == '__main__':
  rospy.init_node('touch_sensor')
  main()
