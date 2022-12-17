#from djitellopy import Tello

import rospy
import rospkg
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Empty


class TelloGestureController:
    def __init__(self, tello):
        self.tello = tello
        self._is_landing = False

        # RC control velocities
        self.forw_back_velocity = 0
        self.up_down_velocity = 0
        self.left_right_velocity = 0
        self.yaw_velocity = 0

    def gesture_control(self, gesture_buffer):
        gesture_id = gesture_buffer.get_gesture()
        print("GESTURE", gesture_id)

        if not self._is_landing:
            if gesture_id == 0:  # Forward
                self.forw_back_velocity = 0.3
            elif gesture_id == 1:  # STOP
                self.forw_back_velocity = self.up_down_velocity = \
                    self.left_right_velocity = self.yaw_velocity = 0
            if gesture_id == 5:  # Back
                self.forw_back_velocity = -0.3

            elif gesture_id == 2:  # UP
                self.up_down_velocity = 0.25
            elif gesture_id == 4:  # DOWN
                self.up_down_velocity = -0.25

            elif gesture_id == 3:  # LAND
                self._is_landing = True
                self.forw_back_velocity = self.up_down_velocity = \
                    self.left_right_velocity = self.yaw_velocity = 0
                # self.tello.land()
                self.pub_land.publish(self.empty_msg)

            elif gesture_id == 6:  # LEFT
                self.left_right_velocity = 0.2
            elif gesture_id == 7:  # RIGHT
                self.left_right_velocity = -0.2

            elif gesture_id == -1:
                self.forw_back_velocity = self.up_down_velocity = \
                    self.left_right_velocity = self.yaw_velocity = 0

            self.tello.speed.linear.x = self.forw_back_velocity
            self.tello.speed.linear.y = self.left_right_velocity
            self.tello.speed.linear.z = self.up_down_velocity
            #self.speed.angular.x = 0.0
            #self.speed.angular.y = 0.0
            self.tello.speed.angular.z = self.yaw_velocity
            self.tello.pub_vel.publish(self.tello.speed)
            # self.tello.send_rc_control(self.left_right_velocity, self.forw_back_velocity,
            #                            self.up_down_velocity, self.yaw_velocity)
