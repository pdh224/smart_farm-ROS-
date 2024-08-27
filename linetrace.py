#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from jetbotmini import Camera
from jetbotmini import bgr8_to_jpeg
import numpy as np
import torch
import torchvision
import torch.nn.functional as F
import cv2
import traitlets
import ipywidgets.widgets as widgets
import numpy as np
from jetbotmini import Robot
from jetbotmini import bgr8_to_jpeg
from IPython.display import display
import time
import rospy
from jetbotmini_msgs.msg import *
import threading
import pyzbar.pyzbar as pyzbar
from PIL import Image, ImageDraw, ImageFont
from queue import Queue
from std_msgs.msg import String, Float32
from jetbotmini_msgs.srv import Motor, MotorResponse
from jetbotmini import Robot
import smbus



#---------------------------------------------------------------

# 초기화

cX =""
I=""
capture_width = 500
capture_height = 500
qr_data = set()
flag = 2
frame_queue = Queue(maxsize=1)
btn_data=''
state="standby"
right=0
left=0
# rospy.init_node('qr_code_publisher_node', anonymous=True)
pub1= rospy.Publisher('qr_code_topic',Tree, queue_size=10)
pub2= rospy.Publisher('jetbot_mini/state',State, queue_size=10)

bus = smbus.SMBus(1)
ADDRESS = 0x1B

#----------------------------------------------------------------

# 카메라 설정

def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=480,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

#-------------------------------------------------------------

# 퍼블리셔

def publisher():
    global state
    pub = rospy.Publisher('jetbot_mini/states', String, queue_size=10)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        states = f"{read_battery_level()}"
        states += f",{state}"
        pub.publish(states)
        print("{}".format(states))
        rate.sleep()

def read_battery_level():
    AD_value = bus.read_i2c_block_data(ADDRESS,0x00,2)
    voltage = ((AD_value[0] << 8) + AD_value[1]) * 13.3 / 1023.0
    return voltage
#     if voltage >= 12:
#         return 'Battery_High'
#     elif voltage  >= 11.1:
#         return 'Battery_Medium'
#     elif voltage  >= 10.05:
#         return 'Battery_Low'
#     elif voltage  <= 9.9:
#         return 'Battery_Empty'
#     elif voltage  <= 10.95:
#         return 'Battery_Low'
#     elif voltage  <= 11.85:
#         return 'Battery_Medium'


# QR코드 감지

def decodeDisplay():
    global frame, flag, qr_data, state,I
    global frame_queue
    while True:        
         if not frame_queue.empty():
            frame = frame_queue.get()
            frame_bgr = cv2.convertScaleAbs(frame)
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            #gray_i=255-gray
            barcodes = pyzbar.decode(gray)
            barcodeData = "none"
            for barcode in barcodes:
                encoding = 'UTF-8'
                barcodeData = barcode.data.decode(encoding)
                barcodeType = barcode.type
                item = barcodeData
                item_t = tuple(item.split(', '))

                if item_t[0] == "tomato":
                    if item_t not in qr_data:
                        qr_data.add(item_t)
                        print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
                        id1=item_t[1]
                        print(id1)
                        pub1.publish("{}".format(id1))
                        print("publish {}".format(I))
                        flag = 1
                        time.sleep(1)

                elif item_t[0]=="end":
                    print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
                    flag = 2
                    pub1.publish("autoplay mode is done")
                    print("complete")
                    time.sleep(1)
                    
                
                    
                
                                                                              
#--------------------------------------------------------------

#서브스크라이버

def linecap(data=None):
    global flag,state,I
    global frame_queue,btn_data,qr_data
    global cX
    btn_data=data.data
    if data is not None:      
        if btn_data=="start" :
            qr_data = set()
            print("start")
            flag=0
            state="running"
                                        
        elif btn_data=="stop" :
            print("stop")
            flag=2
#             state="standby"                                              

def change(data=None):
    global state, flag,ch_data
    ch_data=data.state
    if ch_data == "standby":
        state="standby"
        flag=2
        
        pub1.publish("autoplay mode is done")
        print(ch_data)
        print(state)
        print(flag)
#         print("data:({})".format(data))
    
  
            
            
            
def ros_spin():
    rospy.Subscriber('/jetbot_mini/auto', std_msgs.msg.String, linecap)
#     time.sleep(1)
#     rospy.spin()
    

def ros_spin2():
    rospy.Subscriber('/state',State,change)
    
    
    
    
#-----------------------------------------------------------------

# 메인 코드


def video_main():
    global flag,state
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 프레임 큐 업데이트
        if not frame_queue.full():
            frame_queue.put(frame)

        lower_bound = np.array([0, 0, 0])
        upper_bound = np.array([255, 255, 80])
        
        mask = cv2.inRange(frame, lower_bound, upper_bound)
        M = cv2.moments(mask)
        
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        
        height, width, _ = frame.shape
        center_offset = width // 2 - cX
        fcX = width // 2
        fcY = height // 2
        
        cv2.circle(frame, (cX, cY), 5, (0, 255, 0), -1)
        cv2.circle(frame, (fcX, fcY), 5, (0, 0, 255), -1)
        cv2.imshow('Camera', frame)
#         rospy.Service('motor', Motor, handle_motor_request)
#         motor_server() 
        if cX:
                if state=="running" and flag==1:
                    robot.set_motors(0,0)
                    time.sleep(7)
                    flag = 0
                    
                elif flag==0:
                    center_x = (150 - cX) / 150
                    robot.set_motors(
                         float(0.4 - 0.3 * center_x),
                         float(0.4 + 0.3 * center_x)
                    )
                elif flag==2:
                        robot.set_motors(0,0)                        
                        state="standby"

                
        # 'q' 키를 눌러서 루프 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows() 




if __name__ == '__main__':
    robot = Robot()
    print("Hello World!")
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    rospy.init_node('line_node', anonymous=True)

    # 스레드 생성 및 시작
    
    thread_QR = threading.Thread(target=decodeDisplay)
    thread_QR.daemon = True
    thread_QR.start()
    
    thread_BT = threading.Thread(target=publisher)
    thread_BT.daemon = True
    thread_BT.start()

    thread_ros = threading.Thread(target=ros_spin)
    thread_ros.daemon = True
    thread_ros.start()
        
    thread_MO =threading.Thread(target=video_main)
    thread_MO.daemon = True
    thread_MO.start()
    
    ros_spin()
    ros_spin2()
#     motor_server()
    rospy.spin()
    # 주 스레드 루프
   
    
    


    
    
    
    