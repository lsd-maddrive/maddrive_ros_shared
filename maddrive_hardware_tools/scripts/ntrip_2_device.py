#!/usr/bin/env python

import rospy
import os

def main():
    rospy.init_node('ntrip_2_device', anonymous=False)

    # Получение параметров
    login = rospy.get_param('~login', 'msc9862')
    password = rospy.get_param('~password', '16ihqidp')
    ntrip_server = rospy.get_param('~ntrip_server', 'ntrip.eftgroup.ru')
    port = rospy.get_param('~port', '7047')
    mountpoint = rospy.get_param('~mountpoint', 'SPBE3_2')
    device = rospy.get_param('~device', 'ttyACM0')
    baudrate = rospy.get_param('~baudrate', '115200')

    # Формирование команды
    ntrip_command = f"str2str -in ntrip://{login}:{password}@{ntrip_server}:{port}/{mountpoint} -out serial://{device}:{baudrate}"

    rospy.loginfo(f"Starting NTRIP trasport node to device {device}")
    
    try:
        # Запуск команды
        process = os.popen(ntrip_command)
        
        while not rospy.is_shutdown():
            output = process.read()
            if output:
                rospy.loginfo(output.strip())
                
        process.close()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt received, terminating process")
        process.close()

if __name__ == '__main__':
    main()
