#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy

# F710
# BUTTONS_NUM={   "A":0,    "BACK":6,   AXES_NUM={"LSX":0,  "LSY":1,
#                 "B":1,    "START":7,            "LT":2,   "RT":5, 
#                 "X":2,    "LIVE":8,             "RSX":3,  "RSY":4,
#                 "Y":3,    "LS":9,               "DPADX":6, "DPADY":7
#                 "LB":4,   "RS":10             }
#                 "RB":5,}

# XBOX
# BUTTONS_NUM={   "A":0,    "BACK":10,   AXES_NUM={"LSX":0,  "LSY":1,
#                 "B":1,    "START":11,            "LT":5,   "RT":4, 
#                 "X":3,    "LIVE":12,             "RSX":2,  "RSY":3,
#                 "Y":4,    "LS":13,               "DPADX":6, "DPADY":7
#                 "LB":6,   "RS":14             }
#                 "RB":7,}


class Remapper:
    def __init__(self):
        rospy.init_node('xbox_remapper', anonymous=True)
        rospy.Subscriber("joy_source", Joy, self.callback)
        self.pub = rospy.Publisher("joy_dest", Joy , queue_size = 1)
        rospy.loginfo("Starting joystick remapping")

    def callback(self, inmsg: Joy):
        outmsg = Joy()
        outmsg = inmsg
        new_buttons_order = [0,1,3,4,6,7,10,11,12,13,14]
        new_axes_order = [0,1,5,2,3,4,6,7]
        outmsg.buttons = [inmsg.buttons[i] for i in new_buttons_order]
        outmsg.axes = [inmsg.axes[i] for i in new_axes_order]
        self.pub.publish(outmsg)
       

if __name__ == '__main__':
    Remapper()
    rospy.spin()
