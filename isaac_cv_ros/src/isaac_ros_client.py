#!/usr/bin/env python

# Unrealcv
# from unrealcv import Client
# client = Client(('172.20.0.1',9000))


# ros
import rospy
import threading
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from isaac_cv_ros.msg import IsaacSensorRaw
from isaac_ros_messages.srv import IsaacPose
from isaac_ros_messages.srv import IsaacPoseRequest
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from std_srvs.srv import SetBool
import tf
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Python
import sys
import math
import numpy as np
import time

# Profiling
import cProfile

# DEBUGGING
import pdb
import matplotlib.pyplot as plt


class IsaacRosClient:

    def __init__(self):
        '''  Initialize ros node, isaac_sim client and params '''
        self.should_terminate = False

        # Read in params
        self.collision_on = rospy.get_param('~collision_on', True)  # Check for collision
        # self.publish_tf = rospy.get_param('~publish_tf', False)  # If true publish the camera transformation in tf
        self.publish_color_images = rospy.get_param('~publish_color_images', False)
        self.publish_gray_images = rospy.get_param('~publish_gray_images', False)
        self.queue_size = rospy.get_param('~queue_size', 1)  # How many requests are kept
        self.lock = threading.Lock()
        self.service_proxy = rospy.ServiceProxy('teleport', IsaacPose, persistent=True)
        self.slowdown = 0.0 # Delay between teleport and image/depth saving for rendering purposes

        # Initialize a tf listener to transform the physical sim frame to the visual sim frame
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # Wait for the relevant transforms to be available
        self.tfBuffer.can_transform("sim_world", "camera_sim", rospy.Time(0), rospy.Duration(10.0))
    
        
        self.previous_odom_msg = None  # Previously processed Odom message

        # TODO: At the moment we can't support collisions, would need mor Isaac Sim sided coding most likely
        # self.collision_tolerance = rospy.get_param('~collision_tol', 10)  # Distance threshold in UE units


        # Initialize relative coordinate system (so camera starts at [0, 0, 0] position and [0, 0, yaw]).

        # This location is hardcoded for the warehouse sim - change accordingly
        location = np.array([5.0, 5.0, 2.0]) # x, y, z
        rotation = np.array([1 / math.sqrt(2), 0, 0, 1 / math.sqrt(2)]) # x, y, z, w Quaternion
        self.coord_origin = location
        self.coord_rotation = rotation

        # Set the camera to the origin
        origin = Pose()
        origin.position.x = location[0]
        origin.position.y = location[1]
        origin.position.z = location[2]
        origin.orientation.x = rotation[0]
        origin.orientation.y = rotation[1]
        origin.orientation.z = rotation[2]
        origin.orientation.w = rotation[3]

        teleport_msg = IsaacPoseRequest()
        teleport_msg.names = ["/World/Camera"]
        teleport_msg.poses = [origin]


        # DEBUGGING
        # Convert the position and orientation components to strings
        position_str = "Position: ({}, {}, {})".format(origin.position.x, origin.position.y, origin.position.z)
        orientation_str = "Orientation: ({}, {}, {}, {})".format(origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w)
        # Format the complete Pose information as a string
        pose_info = "\n".join([position_str, orientation_str])
        # Print the Pose information using rospy.loginfo
        rospy.loginfo("Initializing the camera at:\n%s", pose_info)


        # Call the service
        self.service_proxy(teleport_msg)


        # We populate the image message centrally as soon as a new odometry message is received
        self.combined_img_msg = IsaacSensorRaw()
        self.combined_img_msg.color_data = None
        self.combined_img_msg.depth_data = None
        self.odom_ready = False

        self.skip_frames_color = 0
        self.skip_frames_depth = 0


        # Setup subscribers
        self.cam_info_sub = rospy.Subscriber("camera_info", CameraInfo, self.cam_info_callback, queue_size=self.queue_size)
        # TODO: (michbaum) Check if the buffer size is an issue
        self.odom_sub = rospy.Subscriber("odometry", Odometry, self.odom_callback, queue_size=self.queue_size)

        # Finish setup
        self.pub = rospy.Publisher("~isaac_sensor_raw", IsaacSensorRaw, queue_size=10)
        rospy.Service('~terminate_with_reset', SetBool, self.terminate_with_reset_srv)
        if self.collision_on:
            self.collision_pub = rospy.Publisher("~collision", String, queue_size=10)

        if self.publish_color_images or self.publish_gray_images:
            self.cv_bridge = CvBridge()
        if self.publish_color_images:
            self.color_img_pub = rospy.Publisher("~isaac_color_image_out", Image, queue_size=10)
        if self.publish_gray_images:
            self.gray_img_pub = rospy.Publisher("~isaac_gray_image_out", Image, queue_size=10)
        
        rospy.loginfo("isaac_ros_client is ready.")

    def cam_info_callback(self, cam_info_data):
        # Setup camera parameters from isaac sim config

        height = cam_info_data.height
        width = cam_info_data.width
        f = cam_info_data.K[0]

        rospy.loginfo("Camera parameters set as %f %f %f" % (height, width, f))
        rospy.set_param('~camera_params', {'width': float(width), 'height': float(height), 'focal_length': float(f)})

        # Unsubscribe from the camera info topic and instead subscribe to the rgb and depth topics
        self.cam_info_sub.unregister()
        
        self.rgb_sub = rospy.Subscriber("rgb", Image, self.rgb_callback, queue_size=self.queue_size)
        self.depth_sub = rospy.Subscriber("depth", Image, self.depth_callback, queue_size=self.queue_size)
        
    def rgb_callback(self, rgb_data):
        # We only collect an image if we teleported to the next position
        if not self.odom_ready:
            return
        
        # rospy.loginfo("In color callback with timestamp: %f", rgb_data.header.stamp.to_nsec())
        
        # Make sure to not capture stale data
        # Doesn't work until the ROS time is synced
        # if rgb_data.header.stamp.to_nsec() < self.image_message.header.stamp.to_nsec() + self.slowdown * 1e9:
        #     return

        # Since the image data is currently not published with ros /clock timestamps but with isaac sim time
        # we solve the staleness by throwing away some frames
        if self.skip_frames_color < 1:
            self.skip_frames_color += 1
            return
        
        # Check if rgb image is already written
        if self.combined_img_msg.color_data is None:
            
            # Publish the images if necessary
            if self.publish_color_images:
                rgb_data.header.stamp = self.combined_img_msg.header.stamp
                self.color_img_pub.publish(rgb_data)
            if self.publish_gray_images:
            # Utilize cv_bridge to transform it to grayscale
                try:
                    img_color = self.cv_bridge.imgmsg_to_cv2(rgb_data)
                    # Convert the BGR image to grayscale
                    grayscale_image = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)

                    # Convert the grayscale image back to a ROS Image message
                    grayscale_msg = self.cv_bridge.cv2_to_imgmsg(grayscale_image, "mono8")

                    # Publish the grayscale image
                    grayscale_msg.header.stamp = self.combined_img_msg.header.stamp
                    grayscale_msg.header.frame_id = 'camera' # Needs to be in Gazebo frame, so NOT camera_sim
                    self.gray_img_pub.publish(grayscale_msg)

                except CvBridgeError as e:
                    rospy.logerr("CVBridge Error: %s", e)

            # Acquire lock for the message
            with self.lock:
                self.combined_img_msg.color_data = rgb_data

                # rospy.loginfo("Populate sensor message color with timestamp: %f", rgb_data.header.stamp.to_nsec())

                # Check if depth has also been written, then we can publish
                if self.combined_img_msg.depth_data is not None:
                    self.pub.publish(self.combined_img_msg)

                    # Reset the message
                    self.combined_img_msg = IsaacSensorRaw()
                    self.combined_img_msg.color_data = None
                    self.combined_img_msg.depth_data = None
                    self.skip_frames_color = 0
                    self.skip_frames_depth = 0
                    self.odom_ready = False


    def depth_callback(self, depth_data):
        # We only collect an image if we teleported to the next position
        if not self.odom_ready:
            return
        
        # rospy.loginfo("In depth callback with timestamp: %f", depth_data.header.stamp.to_nsec())

        # Make sure to not capture stale data
        # Doesn't work until the ROS time is synced
        # if depth_data.header.stamp.to_nsec() < self.image_message.header.stamp.to_nsec() + self.slowdown * 1e9:
        #     return

        # Since the image data is currently not published with ros /clock timestamps but with isaac sim time
        # we solve the staleness by throwing away some frames
        if self.skip_frames_depth < 1:
            self.skip_frames_depth += 1
            return

        # Check if rgb image is already written
        if self.combined_img_msg.depth_data is None:
            # Acquire lock for the message
            with self.lock:
                self.combined_img_msg.depth_data = depth_data

                # rospy.loginfo("Populate sensor message depth with timestamp: %f", depth_data.header.stamp.to_nsec())


                # Check if rgb has also been written, then we can publish
                if self.combined_img_msg.color_data is not None:
                    self.pub.publish(self.combined_img_msg)

                    # Reset the message
                    self.combined_img_msg = IsaacSensorRaw()
                    self.combined_img_msg.color_data = None
                    self.combined_img_msg.depth_data = None
                    self.skip_frames_color = 0
                    self.skip_frames_depth = 0
                    self.odom_ready = False
                    

    def odom_callback(self, ros_data):
        ''' Produce images for given odometry '''
        if self.should_terminate:
            return

        # We should only do something if the last message has been successfully sent
        if self.odom_ready:
            return

        # Generate images - but only after the last set of images has been sent
        if self.previous_odom_msg is not None:
            # This is not the initialization step

            with self.lock:
                self.publish_images(self.previous_odom_msg)
            
            # rospy.loginfo("Releasing lock in odom callback.")

        self.previous_odom_msg = ros_data
                

    def publish_images(self, odom_msg):
        ''' Produce and publish images'''
        # Populate the image time stamp
        self.combined_img_msg.header.stamp = odom_msg.header.stamp

        # rospy.loginfo("Populate sensor message with timestamp: %f", odom_msg.header.stamp.to_nsec())
        
        # Add the pose into the sensor message for publication to be able to debug better
        self.combined_img_msg.pose = odom_msg.pose.pose

        # Set the camera in the simulation
        teleport_msg = IsaacPoseRequest()
        teleport_msg.names = ["/World/Camera"]
        # Transform the physics odometry pose to the isaac frame
        isaac_pose = self.pose_to_isaac(odom_msg)
        if isaac_pose is None:
            rospy.loginfo("Couldn't teleport, won't publish images.")
            return
        
        teleport_msg.poses = [isaac_pose]

        # # DEBUGGING
        # # Convert the position and orientation components to strings
        # position_str = "Position: ({}, {}, {})".format(isaac_pose.position.x, isaac_pose.position.y, isaac_pose.position.z)
        # orientation_str = "Orientation: ({}, {}, {}, {})".format(isaac_pose.orientation.x, isaac_pose.orientation.y, isaac_pose.orientation.z, isaac_pose.orientation.w)
        # # Format the complete Pose information as a string
        # pose_info = "\n".join([position_str, orientation_str])
        # # Print the Pose information using rospy.loginfo
        # rospy.loginfo("Trying to move the camera to:\n%s in /isaac_sim frame", pose_info)

        # Call the service
        self.service_proxy(teleport_msg)
        # TODO: Possibel bug of sending stale renders in isaac sim
        # self.service_proxy(teleport_msg)
        # self.service_proxy(teleport_msg)

        # rospy.loginfo("TELEPORTED")

        # TODO: (michbaum) Need a new solution for collision
        # Probably solvable by publishing the camera tf https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_ros_tf.html

        self.odom_ready = True

    def pose_to_isaac(self, pose):
        # Transform the pose from /camera to /camera_sim frame
        try:
            # Create a TransformStamped object to store the transformed pose
            # transformStamped = self.tfBuffer.lookup_transform("sim_world", "camera_sim", pose.header.stamp)
            transformStamped = self.tfBuffer.lookup_transform("sim_world", "camera_sim_frame", pose.header.stamp)

            transformed_pose = Pose()
            transformed_pose.orientation = transformStamped.transform.rotation
            transformed_pose.position.x = transformStamped.transform.translation.x
            transformed_pose.position.y = transformStamped.transform.translation.y
            transformed_pose.position.z = transformStamped.transform.translation.z

            return transformed_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed!")
            return None
        

    def terminate_with_reset_srv(self, _):
        # Reset to initial conditions after experiment
        rospy.loginfo("Terminate_with_reset service requested, initiating reset.")
        self.should_terminate = True    # This stops the client from taking other requests (odom topic)

        goal = Pose()
        goal.position.x = self.coord_origin[0]
        goal.position.y = self.coord_origin[1]
        goal.position.z = self.coord_origin[2]
        goal.orientation.x = self.coord_rotation[0]
        goal.orientation.y = self.coord_rotation[1]
        goal.orientation.z = self.coord_rotation[2]
        goal.orientation.w = self.coord_rotation[3]
        teleport_msg = IsaacPoseRequest()
        teleport_msg.names = ["/World/Camera"]
        teleport_msg.poses = [goal]

        time.sleep(2.0)     # Make sure regular execution is finished -> isaac client is ready

        # Call the service
        self.service_proxy(teleport_msg)
        
        return [True, ""]

    def on_collision(self):
        # Collision handling for all modes here
        rospy.logwarn("MAV collision detected!")
        self.collision_pub.publish(String("MAV collision detected!"))

def main():
    rospy.init_node('isaac_ros_client', anonymous=False)  # Currently only 1 Client at a time is supported by isaaccv
    rospy.wait_for_service("teleport")
    ic = IsaacRosClient()
    rospy.spin()

if __name__ == '__main__':
    # TODO: (michbaum) Does this work?
    try:
        main()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    except rospy.ROSInterruptException:
        pass