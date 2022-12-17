#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import configargparse

import cv2 as cv

from gestures.tello_gesture_controller import TelloGestureController
# from utils import CvFpluosCalc
from utils.cvfpscalc import CvFpsCalc
#from djitellopy import Tello
from gestures import *

import threading

from cv_bridge import CvBridge
import rospy
import rospkg
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Empty
from geometry_msgs.msg import Twist

def get_args():
    print('## Reading configuration ##')
    parser = configargparse.ArgParser(default_config_files=['config.txt'])

    parser.add('-c', '--my-config', required=False, is_config_file=True, help='config file path')
    parser.add("--device", type=int)
    parser.add("--width", help='cap width', type=int)
    parser.add("--height", help='cap height', type=int)
    parser.add("--is_keyboard", help='To use Keyboard control by default', type=bool)
    parser.add('--use_static_image_mode', action='store_true', help='True if running on photos')
    parser.add("--min_detection_confidence",
               help='min_detection_confidence',
               type=float)
    parser.add("--min_tracking_confidence",
               help='min_tracking_confidence',
               type=float)
    parser.add("--buffer_len",
               help='Length of gesture buffer',
               type=int)

    args = parser.parse_args()

    return args


class tello_ros:
    def __init__(self):
        self.processing = False
        self.sub_image = rospy.Subscriber('/image_raw', Image, self.image_callback)
        self.image_raw = None
        self.sub_battery = rospy.Subscriber('/battery', Float32, self.battery_callback)
        self.battery = -1

        self.pub_takeoff = rospy.Publisher('/takeoff',Empty)
        self.pub_land = rospy.Publisher('/land',Empty)
        self.empty_msg = Empty()

        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.speed = Twist()
        # self.detect_process()
        self.speed.linear.x = 0.0
        self.speed.linear.y = 0.0
        self.speed.linear.z = 0.0
        self.speed.angular.x = 0.0
        self.speed.angular.y = 0.0
        self.speed.angular.z = 0.0
        
        self.bridge = CvBridge()
        
    def image_callback(self,data):
        self.image_raw = self.bridge.imgmsg_to_cv2(data)
        #self.image_raw = cv2.cvtColor(self.image_raw, cv2.COLOR_BGR2RGB)

        #print(type(frame),frame.size,frame.shape,frame.dtype)

        #cv.imshow("camera", frame)
        #cv.waitKey(1)

        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        #self.image_raw = np.uint8(frame)
        #print(type(self.image_raw),self.image_raw.size,self.image_raw.shape,self.image_raw.dtype)
        
    def battery_callback(self,data):
        self.battery = data.data

    def stop(self):
        self.speed.linear.x = 0.0
        self.speed.linear.y = 0.0
        self.speed.linear.z = 0.0
        self.speed.angular.x = 0.0
        self.speed.angular.y = 0.0
        self.speed.angular.z = 0.0
        self.pub_vel.publish(self.speed)

def main():
    rospy.init_node('img_dete', anonymous=True)
    # init global vars
    global gesture_buffer
    global gesture_id
    # global battery_status

    # Argument parsing
    args = get_args()
    KEYBOARD_CONTROL = args.is_keyboard
    WRITE_CONTROL = False
    in_flight = False

    # Camera preparation
    tello = tello_ros()

    #cap = tello.get_frame_read()

    # Init Tello Controllers
    # 手势控制类，继承了tello
    gesture_controller = TelloGestureController(tello)
    # 键盘控制类，继承了tello
    keyboard_controller = TelloKeyboardController(tello)
    # 用cv做的检测类
    gesture_detector = GestureRecognition(args.use_static_image_mode, args.min_detection_confidence,
                                          args.min_tracking_confidence)
    # 用队列写的buffer，专门存储结果，还有统计防止误判的功能
    gesture_buffer = GestureBuffer(buffer_len=args.buffer_len)

    def tello_control(key, keyboard_controller, gesture_controller):
        global gesture_buffer

        if KEYBOARD_CONTROL:
            keyboard_controller.control(key)
            # print(key)
        else:
            gesture_controller.gesture_control(gesture_buffer)
            #print(2)

    # def tello_battery(tello):
    #     global battery_status
    #     try:
    #         battery_status = tello.get_battery()[:-2]
    #     except:
    #         battery_status = -1

    # FPS Measurement 专门写了个类，还挺认真
    cv_fps_calc = CvFpsCalc(buffer_len=10)

    mode = 0
    number = -1
    # battery_status = -1

    # tello.move_down(20)

    while True:
        fps = cv_fps_calc.get()

        # Process Key (ESC: end)
        # 由键盘选择当前模式
        key = cv.waitKey(1) & 0xff
        if key == 27:  # ESC
            break
        # 起飞，改写成pub take off
        elif key == 32:  # Space
            if not in_flight:
                # Take-off drone
                # tello.takeoff()
                tello.pub_takeoff.publish(tello.empty_msg)
                in_flight = True
        # 降落，同理
            elif in_flight:
                # Land tello
                # tello.land()
                tello.pub_land.publish(tello.empty_msg)

                in_flight = False

        elif key == ord('k'):
            mode = 0
            KEYBOARD_CONTROL = True
            WRITE_CONTROL = False
            tello.stop()
            # tello.send_rc_control(0, 0, 0, 0)  # Stop moving
        elif key == ord('g'):
            KEYBOARD_CONTROL = False
        elif key == ord('n'):
            mode = 1
            WRITE_CONTROL = True
            KEYBOARD_CONTROL = True

        if WRITE_CONTROL:
            number = -1
            if 48 <= key <= 57:  # 0 ~ 9
                number = key - 48

        # Camera capture
        # 图像的获取，这里需要修改，参考比赛的回调函数
        # 重点是图像的格式
        try:
            image = tello.image_raw
            # 默认它是黑箱吧
            debug_image, gesture_id = gesture_detector.recognize(image, number, mode)
            # print(type(image),image.size,image.shape,image.dtype)
        except:
            continue

        gesture_buffer.add_gesture(gesture_id)

        # Start control 
        # 控制部分，把tello_control相关的改成ros
        threading.Thread(target=tello_control, args=(key, keyboard_controller, gesture_controller,)).start()
        
        # 完全可以删掉，直接用sub的回调函数
        #threading.Thread(target=tello_battery, args=(tello,)).start()

        debug_image = gesture_detector.draw_info(debug_image, fps, mode, number)

        # Battery status and image rendering
        cv.putText(debug_image, "Battery: {}".format(tello.battery), (5, 720 - 5),
                   cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv.imshow('Tello Gesture Recognition', debug_image)

    # 直接改成降落就行
    tello.pub_land.publish(tello.empty_msg)
    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
