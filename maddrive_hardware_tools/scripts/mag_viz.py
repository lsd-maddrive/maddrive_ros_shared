#!/usr/bin/env python

import rospy
from sensor_msgs.msg import MagneticField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
import tf

def callback(msg):
    # Создание сообщения Marker
    marker = Marker()
    marker.header.frame_id = "base_link"  # Укажите нужный фрейм
    marker.header.stamp = rospy.Time.now()
    marker.ns = "magnetic_field"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    # Установка позиции начала стрелки
    marker.pose.position = Point(0, 0, 0)  # Начало стрелки в точке (0, 0, 0)

    # Установка направления стрелки на основе магнитного поля
    # Конец стрелки будет в точке, определяемой магнитным полем
    end_point = Point(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z)

    # Установка масштаба стрелки
    length = (end_point.x**2 + end_point.y**2 + end_point.z**2) ** 0.5
    marker.scale.x = length * 0.1  # Длина стрелки
    marker.scale.y = 0.2  # Толщина "оперения" стрелки
    marker.scale.z = 0.0  # Не используется для стрелки

    # Установка цвета стрелки
    marker.color.r = 1.0  # Красный
    marker.color.g = 0.0  # Зеленый
    marker.color.b = 0.0  # Синий
    marker.color.a = 1.0  # Прозрачность

    # Установка ориентации стрелки
    if length > 0:
        # Вычисление кватерниона для ориентации стрелки
        # Используем tf для преобразования вектора в кватернион
        direction = (end_point.x, end_point.y, end_point.z)
        quaternion = tf.transformations.quaternion_from_matrix(tf.transformations.quaternion_matrix(direction))
        marker.pose.orientation = Quaternion(*quaternion)

    # Публикация сообщения Marker
    publisher.publish(marker)

if __name__ == '__main__':
    rospy.init_node('magnetic_field_to_marker', anonymous=True)

    # Публикация сообщений Marker
    publisher = rospy.Publisher('/magnetic_field_marker', Marker, queue_size=10)

    # Подписка на сообщения MagneticField
    subscriber = rospy.Subscriber('/mavros/imu/mag', MagneticField, callback)

    rospy.spin()