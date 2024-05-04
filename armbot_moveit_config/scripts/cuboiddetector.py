#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
import struct
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
import tf_transformations

class CuboidDetector(Node):
    def __init__(self):
        super().__init__('cuboid_detector')
        self.create_subscription(Image,'/zed/zed_node/rgb/image_rect_color',self.image_callback,10)
        
        self.midpcl = self.create_publisher(PointCloud2, '/block_pcl', 2)
        self.blockpub = self.create_publisher(Point, "/block_pos",10)
        self.centre_coords = []
        self.point_holder = []
        self.cv_bridge = CvBridge()

    def infocam(self, msg):
        self.K  = CameraInfo()
        self.K = np.linalg.inv(np.array(msg.k).reshape([3, 3]))
        
        self.get_world_coordinates(self.centre_coords[0])
            

    def get_world_coordinates(self,msg):
        
        rgb = struct.unpack('I', struct.pack('BBBB', 0, 255, 255, 100))[0]
        roll = np.deg2rad(0)
        pitch = np.deg2rad(-90) 
        yaw = np.deg2rad(0)
        h = 0.90
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        rotation_ground_to_cam = np.array([[cr*cy+sp*sr+sy, cr*sp*sy-cy*sr, -cp*sy],
                                            [cp*sr, cp*cr, sp],
                                            [cr*sy-cy*sp*sr, -cr*cy*sp -sr*sy, cp*cy]])
                
        rotation_cam_to_ground = rotation_ground_to_cam.T # inv of rotation mat is same as its transpose

        n = np.array([0, 1, 0])
        ground_normal_to_cam = (rotation_cam_to_ground.T).dot(n)
        self.point_holder = []
        if len(self.K) == 1:
            return np.zeros(3)
        uv_hom = np.array([msg[0], msg[1], 1])
        Kinv_uv = self.K.dot(uv_hom)
        denom = ground_normal_to_cam.dot(Kinv_uv)
        vector = h*Kinv_uv/denom
        print(vector)
        X = vector[2]
        Y = -vector[0]
        Z = -vector[1]
        bot_point_vec = np.array([X,Y,Z])
        # point_vec = self.bot_vec + bot_point_vec
        self.point_holder += [[bot_point_vec[0],bot_point_vec[1],bot_point_vec[2],rgb]]
        # print(point_holder)

        pointgoal = Point()
        pointgoal.z = bot_point_vec[0]-h
        pointgoal.x = bot_point_vec[1]+0.7
        pointgoal.y = bot_point_vec[2]
        
        self.blockpub.publish(pointgoal)
        
        pc2 = self.generate_point_cloud(self.point_holder)
        self.midpcl.publish(pc2)
    
    def generate_point_cloud(self, points):
        header = Header()
        header.frame_id = "zed_camera"
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


    def image_callback(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        blurred_image = cv2.GaussianBlur(img, (5, 5), 0)
        
        edges = cv2.Canny(blurred_image, 100, 200)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        filled_edges = np.zeros_like(edges)
        
        # Draw filled contours
        cv2.fillPoly(filled_edges, contours, (255, 255, 255))

        # Find the indices of white pixels
        white_pixel_indices = np.argwhere(filled_edges == 255)
        # Run DBSCAN on white pixel indices
        db = DBSCAN(eps=1, min_samples=5).fit(white_pixel_indices)
        labels = db.labels_
        # no_clusters = len(np.unique(labels))
        # no_noise = np.sum(labels == -1)

        # print('Estimated no. of clusters: %d' % no_clusters)
        # print('Estimated no. of noise points: %d' % no_noise)
        self.centre_coords = []
        clustered_image = np.zeros_like(img)
        for label in np.unique(labels):
            if label == -1 :
                continue 
            cluster_indices = white_pixel_indices[labels == label]
            centroid = np.mean(cluster_indices, axis=0).astype(int)
            for point in cluster_indices:
                clustered_image[point[0], point[1]] = 255
            cv2.putText(clustered_image, str(label), (centroid[1], centroid[0]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            # print(f"Cluster {label} centroid coordinates: {centroid[1]},{centroid[0]}")
            self.centre_coords.append((centroid[1],centroid[0]))
        
        if len(self.centre_coords)>0:
        # print(self.centre_coords[0])
            self.create_subscription(CameraInfo, "/zed/zed_node/rgb/camera_info", self.infocam, 10)
        else:
            print("Re-iterate")
        # print(len(self.centre_coords))
        cv2.imshow("Objects", clustered_image)
        cv2.waitKey(1)  


def main(args=None):
    rclpy.init(args=args)
    cuboid_detector = CuboidDetector()
    rclpy.spin(cuboid_detector)
    cuboid_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
