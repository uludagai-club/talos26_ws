#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2

def cloud_callback(msg):
    # Gazebo'nun matematik hatasını KÖKÜNDEN çözüyoruz
    # Unorganized (Tek Sıra) nokta bulutu için kusursuz matematik:
    
    msg.height = 1
    # Toplam veri byte'ını, tek bir noktanın byte boyutuna bölersek KESİN genişliği (nokta sayısını) buluruz.
    msg.width = len(msg.data) // msg.point_step 
    msg.row_step = len(msg.data)
    
    # Düzeltilmiş bulutu yeni bir topic üzerinden yayınla
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('gazebo_cloud_fixer')
    rospy.loginfo("Gazebo PointCloud Filtresi V2 Devrede! Kusursuz matematik uygulaniyor...")
    
    pub = rospy.Publisher('/scan_cloud_fixed', PointCloud2, queue_size=10)
    rospy.Subscriber('/cart/center_laser/scan', PointCloud2, cloud_callback)
    rospy.spin()