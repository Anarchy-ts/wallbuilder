#!/usr/bin/python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
import struct
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
import tf_transformations

class LaneMidWorld(Node):
    def __init__(self):
        super().__init__('midpt_ipm')
        self.create_subscription(CameraInfo, "/zed/zed_node/rgb/camera_info", self.infocam, 10)
        self.create_subscription(Odometry, "/vidhyut/odom", self.odom_info, 10)
        self.midpcl = self.create_publisher(PointCloud2, '/flane/midworldcoord', 2)
        self.lanegoal = self.create_publisher(Point, "/flane/lanegoal",10)

    def infocam(self, msg):
        self.K  = CameraInfo()
        self.K = np.linalg.inv(np.array(msg.k).reshape([3, 3]))
        self.create_subscription(Point, '/flane/midpts', self.get_world_coordinates, 10)

    def odom_info(self,msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z
        self.odom_ang_x = msg.pose.pose.orientation.x
        self.odom_ang_y = msg.pose.pose.orientation.y
        self.odom_ang_z = msg.pose.pose.orientation.z
        self.odom_w= msg.pose.pose.orientation.w
        quaternion = [self.odom_ang_x, self.odom_ang_y, self.odom_ang_z, self.odom_w]
        # q = tf_transformations.quaternion_from_euler(r, p, y)
        self.r, self.p, self.y = tf_transformations.euler_from_quaternion(quaternion)
        self.roll = np.rad2deg(self.r)
        self.pitch = np.rad2deg(self.p)
        self.yaw = np.rad2deg(self.y)
        # print(f"Odom in Euler : {self.roll},{self.pitch},{self.yaw}")
        self.bot_vec = np.array([self.odom_x,self.odom_y,0.0])

    def get_world_coordinates(self,msg):
        point_holder = []
        rgb = struct.unpack('I', struct.pack('BBBB', 0, 255, 255, 100))[0]
        roll = np.deg2rad(0)
        pitch = np.deg2rad(0) 
        yaw = np.deg2rad(0)
        h = 0.905
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        rotation_ground_to_cam = np.array([[cr*cy+sp*sr+sy, cr*sp*sy-cy*sr, -cp*sy],
                                            [cp*sr, cp*cr, sp],
                                            [cr*sy-cy*sp*sr, -cr*cy*sp -sr*sy, cp*cy]])
                
        rotation_cam_to_ground = rotation_ground_to_cam.T # inv of rotation mat is same as its transpose

        n = np.array([0, 1, 0])
        ground_normal_to_cam = (rotation_cam_to_ground.T).dot(n)

        if len(self.K) == 1:
            return np.zeros(3)
        uv_hom = np.array([msg.x, msg.y, 1])
        Kinv_uv = self.K.dot(uv_hom)
        denom = ground_normal_to_cam.dot(Kinv_uv)
        vector = h*Kinv_uv/denom
        X = vector[2]
        Y = -vector[0]
        bot_point_vec = np.array([X*np.cos(self.y)-Y*np.sin(self.y),X*np.sin(self.y)+Y*np.cos(self.y),0.0])
        point_vec = self.bot_vec + bot_point_vec
        point_holder += [[point_vec[0],point_vec[1],0.0,rgb]]
        # print(point_holder)

        pointgoal = Point()
        pointgoal.x = point_vec[0]
        pointgoal.y = point_vec[1]
        pointgoal.z = 0.0
        
        self.lanegoal.publish(pointgoal)
        
        pc2 = self.generate_point_cloud(point_holder)
        self.midpcl.publish(pc2)

    def generate_point_cloud(self, points):
        header = Header()
        header.frame_id = "odom"
        pointfieldx = PointField()
        pointfieldx.name = 'x'
        pointfieldx.offset = 0
        pointfieldx.datatype = 7
        pointfieldx.count = 1
        pointfieldy = PointField()
        pointfieldy.name = 'y'
        pointfieldy.offset = 4
        pointfieldy.datatype = 7
        pointfieldy.count = 1
        pointfieldz = PointField()
        pointfieldz.name = 'z'
        pointfieldz.offset = 8
        pointfieldz.datatype = 7
        pointfieldz.count = 1
        pointfieldrgb = PointField()
        pointfieldrgb.name = 'rgba'
        pointfieldrgb.offset = 12
        pointfieldrgb.datatype = 6
        pointfieldrgb.count = 1

        # fields = [PointField(('x', 0, PointField.FLOAT32, 1)),
        #           PointField(('y', 4, PointField.FLOAT32, 1)),
        #           PointField(('z', 8, PointField.FLOAT32, 1)),
        #           PointField(('rgba', 12, PointField.UINT32, 1))]
        
        fields = [pointfieldx, pointfieldy, pointfieldz, pointfieldrgb]

        pc2 = point_cloud2.create_cloud(header, fields, points)
        pc2.header.stamp = self.get_clock().now().to_msg()
        return pc2

def main(args=None):
    rclpy.init(args=args)
    lanemid = LaneMidWorld()
    rclpy.spin(lanemid)
    lanemid.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

