#!/usr/bin/env python

# ros
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from isaac_cv_ros.msg import IsaacSensorRaw

# Image conversion
import io
import cv2
from cv_bridge import CvBridge, CvBridgeError

import PIL.Image
# Python
import sys
import math
import numpy as np
from struct import pack, unpack
import time


# DEBUGGING
import pdb
import matplotlib.pyplot as plt


class SensorModel:

    def __init__(self):
        '''  Initialize ros node and read params '''

        # Read in params
        model_type_in = rospy.get_param('~model_type', 'ground_truth')
        camera_params_ns = rospy.get_param('~camera_params_ns', rospy.get_namespace()+"isaac_ros_client/camera_params")
        self.publish_color_images = rospy.get_param('~publish_color_images', False)
        self.publish_gray_images = rospy.get_param('~publish_gray_images', False)
        self.maximum_distance = rospy.get_param('~maximum_distance', 0)  # Set to 0 to keep all points
        self.flatten_distance = rospy.get_param('~flatten_distance', 0)  # Set to 0 to ignore
        # If true publish a transform stamped message based on the teleport pose
        self.publish_transform_stamped = rospy.get_param('~publish_transform_stamped', False) 
        
        # Setup sensor type
        model_types = {'ground_truth': 'ground_truth', 'kinect': 'kinect',
                       'gaussian_depth_noise': 'gaussian_depth_noise'}      # Dictionary of implemented models
        selected = model_types.get(model_type_in, 'NotFound')
        if selected == 'NotFound':
            warning = "Unknown sensor model '" + model_type_in + "'. Implemented models are: " + \
                      "".join(["'" + m + "', " for m in model_types])
            rospy.logfatal(warning[:-2])
        else:
            self.model = selected

        # Model dependent params
        if self.model == 'gaussian_depth_noise':
            # coefficients for polynomial, f(z) = k0 + k1z + k2z^2 + k3z^3
            self.coefficients = np.array([0.0]*8)
            for i in range(4):
                self.coefficients[i] = rospy.get_param('~k_mu_%i' % i, 0.0)
                self.coefficients[4 + i] = rospy.get_param('~k_sigma_%i' % i, 0.0)

        # Initialize camera params from params or wait for isaac_ros_client to publish them
        if not rospy.has_param(camera_params_ns+'width'):
            rospy.loginfo("Waiting for isaac camera params at '%s' ...", camera_params_ns)
            while not rospy.has_param(camera_params_ns+'/width'):
                rospy.sleep(0.1)
        
        

        self.camera_params = [rospy.get_param(camera_params_ns+'/width'), rospy.get_param(camera_params_ns+'/height'),
                              rospy.get_param(camera_params_ns+'/focal_length')]

        # Initialize node
        self.pub = rospy.Publisher("~isaac_sensor_out", PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber("isaac_sensor_raw", IsaacSensorRaw, self.callback, queue_size=10)
        self.cv_bridge = CvBridge()
        if self.publish_color_images:
            self.color_img_pub = rospy.Publisher("~isaac_color_image_out", Image, queue_size=10)
        if self.publish_gray_images:
            self.gray_img_pub = rospy.Publisher("~isaac_gray_image_out", Image, queue_size=10)

        if self.publish_transform_stamped:
            self.transform_pub = rospy.Publisher("~transform", TransformStamped, queue_size=10)
            self.pose_pub = rospy.Publisher("~pose", PoseStamped, queue_size=10)
        rospy.loginfo("Sensor model setup cleanly.")

    def callback(self, sensor_raw_data):
        ''' Produce simulated sensor outputs from raw binary data '''
        # Time the callback
        # now = rospy.Time.now()
        # Read out images
        if sensor_raw_data.color_data is not None:
            img_color = self.cv_bridge.imgmsg_to_cv2(sensor_raw_data.color_data)
        else:
            rospy.logerr("Color image not populated!!")
        if sensor_raw_data.depth_data is not None:
            img_depth = self.cv_bridge.imgmsg_to_cv2(sensor_raw_data.depth_data)
        else:
            rospy.logerr("Depth image not populated!!")
        
        mask_depth = img_depth.reshape(-1)

        # Build 3D point cloud from depth
        if self.flatten_distance > 0:
            img_depth = np.clip(img_depth, 0, self.flatten_distance)
        (x, y, z) = self.depth_to_3d(img_depth)

        # Pack RGB image (for ros representation)
        rgb = self.rgb_to_float(img_color)

        # Remove invalid points
        if self.maximum_distance > 0:
            mask = mask_depth <= self.maximum_distance
            x = x[mask]
            y = y[mask]
            z = z[mask]
            rgb = rgb[mask]

        # Sensor model processing, put other processing functions here
        if self.model == 'kinect':
            x, y, z, rgb = self.process_kinect(x, y, z, rgb)
        elif self.model == 'gaussian_depth_noise':
            z = self.process_gaussian_depth_noise(z)

        # rospy.loginfo("Populating point cloud message with timestamp: %f", ros_data.header.stamp.to_nsec())

        # TODO: Try to transform the pointcloud to /world manually with the pose where the image was taken from
        #       There should be some tf utility for transforming a whole pointcloud

        # TODO: Visualize the pose that we took the image from to check that the shit is actually reasonable

        # TODO: Publish the teleported poses with another frame name and change the pointcloud frame to that one

        # Publish transform
        if self.publish_transform_stamped:
            self.publish_transform(sensor_raw_data)


        # Publish pointcloud
        data = np.transpose(np.vstack((x, y, z, rgb)))
        msg = PointCloud2()
        msg.header.stamp = sensor_raw_data.header.stamp
        msg.header.frame_id = 'camera' # Needs to be in Gazebo frame, so NOT camera_sim
        msg.width = data.shape[0]
        msg.height = 1
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = np.dtype(np.float32).itemsize*4 # 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        #msg.data = np.float32(data).tostring()
        msg.data = data.astype(np.float32).tobytes()
        self.pub.publish(msg)

        # If requested, also publish the image
        if self.publish_color_images:
            self.color_img_pub.publish(sensor_raw_data.color_data)
        if self.publish_gray_images:
            # Utilize cv_bridge to transform it to grayscale
            try:
                # Convert the BGR image to grayscale
                grayscale_image = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)

                # Convert the grayscale image back to a ROS Image message
                grayscale_msg = self.cv_bridge.cv2_to_imgmsg(grayscale_image, "mono8")

                # Publish the grayscale image
                grayscale_msg.header.stamp = sensor_raw_data.header.stamp
                grayscale_msg.header.frame_id = 'camera' # Needs to be in Gazebo frame, so NOT camera_sim
                self.gray_img_pub.publish(grayscale_msg)

            except CvBridgeError as e:
                rospy.logerr("CVBridge Error: %s", e)
        
        # Time the callback
        # duration = rospy.Time.now() - now
        # rospy.loginfo("Callback time: %f seconds", duration.to_sec())
            
    def publish_transform(self, sensor_raw_msg):
        # Create a new TransformStamped message
        transform_msg = TransformStamped()

        # Set the header
        transform_msg.header = sensor_raw_msg.header

        # Set the child and parent frames
        transform_msg.child_frame_id = 'camera_pointcloud'
        transform_msg.header.frame_id = 'world'

        # Set the transform translation and rotation
        transform_msg.transform.translation = sensor_raw_msg.pose.position
        transform_msg.transform.rotation = sensor_raw_msg.pose.orientation

        self.transform_pub.publish(transform_msg)


        # Also publish a pose topic to visualize in rviz
        pose_msg = PoseStamped()
        pose_msg.header = sensor_raw_msg.header
        pose_msg.pose = sensor_raw_msg.pose
        self.pose_pub.publish(pose_msg)


    def depth_to_3d(self, img_depth):
        ''' Create point cloud from depth image and camera params. Returns a single array for x, y and z coords '''
        # read camera params and create image mesh
        height = self.camera_params[1]
        width = self.camera_params[0]
        center_x = width/2
        center_y = height/2
        f = self.camera_params[2]
        cols, rows = np.meshgrid(np.linspace(0, width - 1, num=width), np.linspace(0, height - 1, num=height))


        # Process depth image from ray length to camera axis depth
        distance = ((rows - center_y) ** 2 + (cols - center_x) ** 2) ** 0.5
        # points_z = img_depth / (1 + (distance / f) ** 2) ** 0.5
        
        # TODO: (michbaum) Testing if depth images are already in camera axis depth -> They indeed are
        points_z = img_depth

        # Create x and y position
        points_x = points_z * (cols - center_x) / f
        points_y = points_z * (rows - center_y) / f

        return points_x.reshape(-1), points_y.reshape(-1), points_z.reshape(-1)

    @staticmethod
    def rgb_to_float(img_color):
        ''' Stack uint8 rgb image into a single float array (efficiently) for ros compatibility '''
        r = np.ravel(img_color[:, :, 0]).astype(int)
        g = np.ravel(img_color[:, :, 1]).astype(int)
        b = np.ravel(img_color[:, :, 2]).astype(int)
        color = np.left_shift(r, 16) + np.left_shift(g, 8) + b
        packed = pack('%di' % len(color), *color)
        unpacked = unpack('%df' % len(color), packed)
        return np.array(unpacked)

    @staticmethod
    def process_kinect(x, y, z, rgb):
        # Simulate a kinect sensor model according to this paper: https://ieeexplore.ieee.org/abstract/document/6375037
        # The effect of the target plane orientation theta is neglected (its constant until theta ~ 60/70 deg).
        mask = (z > 0.5) & (z < 3.0)
        x = x[mask]
        y = y[mask]
        z = z[mask]
        rgb = rgb[mask]
        sigma_l = 0.0014 * z
        sigma_z = 0.0012 + 0.0019 * (z - 0.4) ** 2
        mu = np.zeros(np.shape(z))
        dx = np.random.normal(mu, sigma_l)
        dy = np.random.normal(mu, sigma_l)
        dz = np.random.normal(mu, sigma_z)
        return x+dx, y+dy, z+dz, rgb

    def process_gaussian_depth_noise(self, z_in):
        # Add a depth dependent guassian error term to the perceived depth. Mean and stddev can be specified as up to
        # deg3 polynomials.
        mu = np.ones(np.shape(z_in)) * self.coefficients[0]
        sigma = np.ones(np.shape(z_in)) * self.coefficients[4]
        for i in range(1, 4):
            if self.coefficients[i] != 0:
                mu = mu + np.power(z_in, i) * self.coefficients[i]
            if self.coefficients[4 + i] != 0:
                sigma = sigma + np.power(z_in, i) * self.coefficients[4 + i]
        return z_in + np.random.normal(mu, sigma)


if __name__ == '__main__':
    rospy.init_node('sensor_model', anonymous=True)
    sm = SensorModel()
    rospy.spin()

