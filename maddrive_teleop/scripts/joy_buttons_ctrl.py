#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, UInt8
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from time import sleep
import numpy as np


# 0 - Blue
# 1 - Green
# 2 - Red
# 3 - Yellow
# 4 - LU
# 5 - RU
# 6 - LB
# 7 - RB
# 10 - Left stick
# 11 - Right stick
pedal = 0
button_names = [
    "Blue",
    "Green",
    "Red",
    "Yellow",
    "Left upper",
    "Right upper",
    "Left bottom",
    "Right bottom",
    "_",
    "_",
    "Left stick",
    "Right stick",
]

axes_names = [
    "Lstick horz",
    "Lstick vert",
    "Rstick horz",
    "Rstick vert",
    "Btns horz",
    "Btns vert",
]



def show_clicked(msg):
    print("Buttons:")
    for i in range(len(msg.buttons)):
        if msg.buttons[i] != 0:
            print("\t" + button_names[i])

    print("Axes:")
    for i in range(len(msg.axes)):
        print("\t%s: %.2f" % (axes_names[i], msg.axes[i]))
    print("Flag:", pedal_state.flag)


def joy_cb(msg):

    if debug_enabled:
        show_clicked(msg)

    angular_pos.set_relative(msg.axes[0])
    linear_vel.set_relative(msg.axes[3])
    pedal_state.set_state(msg)




class TwoDirectionVelocity:
    def __init__(self, min_value, max_value, zero_point=0):
        assert min_value < zero_point < max_value

        self._low_ratio = zero_point - min_value
        self._high_ratio = max_value - zero_point

        self._zero_point = zero_point
        self._velocity = 0

    def set_relative(self, ratio):
        ratio = np.clip(ratio, -1, 1)

        if ratio < 0:
            self._velocity = self._zero_point + ratio * self._low_ratio
        else:
            self._velocity = self._zero_point + ratio * self._high_ratio

    def get_velocity(self):
        return self._velocity

class PressButton:
    def __init__(self):
        self.zero_point = 0
        self.state = 100
        self.flag = 0
    def set_state(self, msg):   
        if msg.buttons[1] == 0 and msg.buttons[0] == 0 and msg.buttons[2] == 0 :
            self.flag = 0
        if self.flag == 0:
            if msg.buttons[0] == 1:
                if self.state < 100:
                    self.state+=2.5
                    self.flag = 1
            elif msg.buttons[1] == 1:
                if self.state > 0:
                    self.state-=2.5
                    self.flag = 1
            elif msg.buttons[2] == 1:
                self.state = 100
                self.flag = 1            
        
        

    def get_state(self):
        return self.state



if __name__ == "__main__":
    rospy.init_node("control_link")

    debug_enabled = rospy.get_param("~debug", False)
    if debug_enabled:
        rospy.loginfo("Debug enabled")

    command_state = Twist()
   

    rospy.Subscriber("joy", Joy, joy_cb, queue_size=5)
    cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)

    forward_speed_limit_mps = rospy.get_param("~speed/frwd_limit", 1)
    backward_speed_limit_mps = rospy.get_param("~speed/bkwrd_limit", -1)
    steer_limit_deg = rospy.get_param("~steer/limit", 25)
    steer_limit_rad = np.deg2rad(steer_limit_deg)

    linear_vel = TwoDirectionVelocity(
        min_value=backward_speed_limit_mps, max_value=forward_speed_limit_mps
    )
    angular_pos = TwoDirectionVelocity(
        min_value=-steer_limit_rad, max_value=steer_limit_rad
    )
    pedal_state = PressButton()

    rospy.loginfo("Ready, go!")
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        # command_state.linear.x = linear_vel.get_velocity()
        command_state.angular.z = angular_pos.get_velocity()

        command_state.linear.x = pedal_state.get_state()

        cmd_pub.publish(command_state)
        rate.sleep()
