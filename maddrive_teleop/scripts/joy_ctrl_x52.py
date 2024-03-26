#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import numpy as np
import time


# BUTTONS_NUM={"J_triger":0,
#             "J_safe_fire":1,
#             "J_A":2,
#             "J_B":3,
#             "J_C":4,
#             "J_pinky":5,
#             "D_D":6,
#             "D_E":7,
#             "J_T1":8,
#             "J_T2":9,
#             "J_T3":10,
#             "J_T4":11,
#             "J_T5":12,
#             "J_T6":13,
#             "J_triger_2":14,
#             "J_UP":15,
#             "J_RIGHT":16,
#             "J_DOWN":17,
#             "J_LEFT":18,
#             "D_backward":19,
#             "D_right":20,
#             "D_forward":21,
#             "D_left":22,
#             "J_Mode_green":23,
#             "J_Mode_yellow":24,
#             "J_Mode_red":25,
#             "D_function":26,
#             "D_start_stop":27,
#             "D_reset":28,
#             "D_info":29,
#             "D_mouse":30,
#             "D_scroll":31,
#             "D_scroll_forward":32,
#             "D_scroll_backward":33,
#             }


# Axes_name:
# "J_roll":0
# "J_pitch":1
# "D_hand":2
# "D_Iscroll":3
# "D_Escroll":4
# "J_yaw":5
# "D_slider":6
# "J_horz":7
# "J_vert":8
# "D_joy_ver":9
# "D_jor_hor":10


class Controller():
    def __init__(self) -> None:
        self.axis = self.TwoDirectionVelocity
        self.buttons_status = self.ButtonsStatus
        self.command_state = Twist()
        self.buttons_msg = Int32MultiArray()
        self.zero_published = False
        self.rate = 5
        self.reverse = False

        rospy.init_node("control_link")
        rospy.Subscriber("joy", Joy, self.joy_cb, queue_size=5)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        self.btn_pub = rospy.Publisher("button_pressed", Int32MultiArray, queue_size=5)
       
        self.debug_enabled = rospy.get_param("~debug", True)
        forward_speed_limit_mps = rospy.get_param("~speed/frwd_limit", 1)
        backward_speed_limit_mps = rospy.get_param("~speed/bkwrd_limit", -1)
        steer_limit_deg = rospy.get_param("~steer/limit", 25)
        steer_limit_rad = np.deg2rad(steer_limit_deg)

        self.linear_vel = self.axis(
            min_value=backward_speed_limit_mps, max_value=forward_speed_limit_mps
        )
        self.angular_pos = self.axis(
            min_value=-steer_limit_rad, max_value=steer_limit_rad, zero_point=-0.1
        )
        self.buttons = self.buttons_status()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if self.debug_enabled:
            rospy.loginfo("Debug enabled")
        rospy.loginfo("Wating for move_base server.....") 
        rospy.loginfo("Ready, go!")



    def joy_cb(self, msg):

        if self.debug_enabled:
            self.show_clicked(msg)

        ## last message timestamp
        # self.linear_vel.last_stamp=msg.header.stamp.secs
        self.linear_vel.last_stamp=time.time()

        self.angular_pos.set_relative_angular(msg.axes[5]) # j_yaw
        self.angular_pos.set_relative_angular(msg.axes[10]) #"point"

        self.linear_vel.set_relative_linear(msg.axes[2], self.reverse)
        self.buttons.set_buttons(msg.buttons)


    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
    
    def button_handler(self,button_arr):
        if button_arr[27] == 1:
            self.reverse = True
            # self.client.cancel_goal()
            # self.client.cancel_all_goals() # Try both
            # rospy.loginfo("Goal cancelled")
            rospy.loginfo("Revrese Mode")
        elif button_arr[28] == 1:
            rospy.loginfo("Forward Mode")

            self.reverse = False


    def update(self):
       
          
        self.command_state.linear.x = self.linear_vel.get_velocity()
        self.command_state.angular.z = self.angular_pos.get_velocity()
        self.buttons_msg.data = self.buttons.get_buttons()
        self.button_handler(self.buttons_msg.data)

        self.cmd_pub.publish(self.command_state)
        self.btn_pub.publish(self.buttons_msg)


    def show_clicked(self,msg):
        button_names = ["J_triger","J_safe_fire","J_A","J_B",
                    "J_C","J_pinky","D_D","D_E","J_T1","J_T2",
                    "J_T3","J_T4","J_T5","J_T6","J_triger_2",
                    "J_UP","J_RIGHT","J_DOWN","J_LEFT","D_backward",
                    "D_right","D_forward","D_left","J_Mode_green", "J_Mode_yellow",
                    "J_Mode_red","D_function","D_start_stop","D_reset",
                    "D_info","D_mouse","D_scroll","D_scroll_forward","D_scroll_backward"
        ]

        axes_names = ["J_roll","J_pitch",
                "D_hand","D_Iscroll",
                "D_Escroll","J_yaw","D_slider","J_horz","J_vert","D_joy_ver","D_jor_hor"
        ]

        print("Buttons:")
        for i in range(len(msg.buttons)):
            if msg.buttons[i] != 0:
                print("\t" + button_names[i])

        print("Axes:")
        for i in range(len(msg.axes)):
            print("\t%s: %.2f" % (axes_names[i], msg.axes[i]))

        # print(f'timestamp diif is {int(time.time()) - self.linear_vel.last_stamp}')
        
    # def GoalHandler

    class TwoDirectionVelocity:
        def __init__(self, min_value, max_value, zero_point=0):
            assert min_value < zero_point < max_value

            self._low_ratio = zero_point - min_value
            self._high_ratio = max_value - zero_point

            self._zero_point = zero_point
            self._velocity = 0
            self.last_stamp = 0

        def set_relative_linear(self, ratio, is_reverse):
            ratio = np.clip(ratio, -1, 1)
            ratio = (ratio + 1) / 2 
            if is_reverse:  #if ratio <0
                print('reversed')
                self._velocity = self._zero_point + (-1)*ratio * self._low_ratio
            else:
                print('forwarded')

                self._velocity = self._zero_point + ratio * self._high_ratio

        def set_relative_angular(self, ratio):
            ratio = np.clip(ratio, -1, 1)

            if ratio <0:
                self._velocity = (ratio - self._zero_point) * self._low_ratio
            else:
                self._velocity = (ratio + self._zero_point) * self._high_ratio

        def get_velocity(self):
            return self._velocity
        
    class ButtonsStatus:
        def __init__(self) -> None:
            self._pressed_buttons=np.zeros(35)

        def set_buttons(self,msg):
            self._pressed_buttons = msg
            
        def get_buttons(self):
            return self._pressed_buttons
        

if __name__ == "__main__":
    try:
        controller=Controller()
        controller.spin()
    except rospy.ROSInterruptException:
        print('Exception catched!')
