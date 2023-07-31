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
from geometry_msgs.msg import Pose, TransformStamped
from std_srvs.srv import SetBool
import tf
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

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
        self.publish_tf = rospy.get_param('~publish_tf', False)  # If true publish the camera transformation in tf
        self.queue_size = rospy.get_param('~queue_size', 1)  # How many requests are kept
        self.lock = threading.Lock()
        self.service_proxy = rospy.ServiceProxy('teleport', IsaacPose, persistent=True)

        # Initialize a tf listener to transform the physical sim frame to the visual sim frame
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # Wait for the relevant transforms to be available
        self.tfBuffer.can_transform("sim_world", "camera_sim", rospy.Time(0), rospy.Duration(10.0))
    
        
        self.previous_odom_msg = None  # Previously processed Odom message

        # TODO: At the moment we can't support collisions, would need mor Isaac Sim sided coding most likely
        self.collision_tolerance = rospy.get_param('~collision_tol', 10)  # Distance threshold in UE units


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
        # TODO: (michbaum) Maybe needs a mutex
        self.image_message = IsaacSensorRaw()
        self.image_message.color_data = None
        self.image_message.depth_data = None
        self.odom_ready = False


        # Setup subscribers
        self.cam_info_sub = rospy.Subscriber("camera_info", CameraInfo, self.cam_info_callback, queue_size=self.queue_size)
        # TODO: (michbaum) Check if the buffer size is an issue
        self.odom_sub = rospy.Subscriber("odometry", Odometry, self.odom_callback, queue_size=self.queue_size)

        # Finish setup
        self.pub = rospy.Publisher("~isaac_sensor_raw", IsaacSensorRaw, queue_size=10)
        rospy.Service('~terminate_with_reset', SetBool, self.terminate_with_reset_srv)
        if self.collision_on:
            self.collision_pub = rospy.Publisher("~collision", String, queue_size=10)
        # rospy.loginfo("isaac_ros_client is ready in %s mode." % self.mode)

    def cam_info_callback(self, cam_info_data):
        # Setup camera parameters from isaac sim config

        height = cam_info_data.height
        width = cam_info_data.width
        f = cam_info_data.K[0]

        rospy.loginfo("Camera parameters set as %f %f %f" % (height, width, f))
        rospy.set_param('~camera_params', {'width': float(width), 'height': float(height), 'focal_length': float(f)})

        # Unsubscribe from the camera info topic and instead subscribe to the rgb and depth topics
        self.cam_info_sub.unregister()
        # TODO: (michbaum) Check if the buffer size needs to be bigger
        self.rgb_sub = rospy.Subscriber("rgb", Image, self.rgb_callback, queue_size=self.queue_size)
        self.depth_sub = rospy.Subscriber("depth", Image, self.depth_callback, queue_size=self.queue_size)
        
    def rgb_callback(self, rgb_data):
        # We only collect an image if we teleported to the next position
        # rospy.loginfo("In rgb callback.")
        if not self.odom_ready:
            return
        
        # rospy.loginfo("In rgb callback after odom_ready.")
        
        # Check if rgb image is already written
        if self.image_message.color_data is None:
            # Acquire lock for the message
            # rospy.loginfo("Trying to acquire lock in rgb_callback.")
            with self.lock:
                self.image_message.color_data = rgb_data
                # rospy.loginfo("In rgb callback after acquiring lock.")

                # Check if depth has also been written, then we can publish
                if self.image_message.depth_data is not None:
                    # rospy.loginfo("In rgb callback publishing.")
                    self.pub.publish(self.image_message)

                    # Reset the message
                    self.odom_ready = False
                    self.image_message = IsaacSensorRaw()
                    self.image_message.color_data = None
                    self.image_message.depth_data = None


    def depth_callback(self, depth_data):
        # We only collect an image if we teleported to the next position
        # rospy.loginfo("In depth callback.")
        if not self.odom_ready:
            return
        
        # rospy.loginfo("In depth callback after odom_ready.")

        # Check if rgb image is already written
        if self.image_message.depth_data is None:
            # Acquire lock for the message
            # rospy.loginfo("Trying to acquire lock in depth_callback.")
            with self.lock:
                self.image_message.depth_data = depth_data
                # rospy.loginfo("In depth callback after acquiring lock.")

                # Check if rgb has also been written, then we can publish
                if self.image_message.color_data is not None:
                    # rospy.loginfo("In depth callback publishing.")
                    self.pub.publish(self.image_message)

                    # Reset the message
                    self.odom_ready = False
                    self.image_message = IsaacSensorRaw()
                    self.image_message.color_data = None
                    self.image_message.depth_data = None
                    

    def odom_callback(self, ros_data):
        ''' Produce images for given odometry '''
        # Profiling
        # pr = cProfile.Profile()
        # pr.enable()
        if self.should_terminate:
            return
        # Slowdown to give more rendering time to the unreal engine (should not be necessary in normal mode)
        # time.sleep(self.slowdown)

        if self.publish_tf:
            self.publish_tf_data(ros_data)

        # rospy.loginfo("In odom callback before odom_ready.")

        # We should only do something if the last message has been successfully sent
        if self.odom_ready:
            return
        
        # rospy.loginfo("In odom callback past odom_ready.")

        # Generate images - but only after the last set of images has been sent
        if self.previous_odom_msg is not None:
            # This is not the initialization step
            # TODO: (michbaum) They actually stagger the image generation and the pose setting, such that unreal has enough time to
            # render the images. If we try to decouple pose setting and image retrieval, this could be a potential problem.

            with self.lock:
                self.publish_images(self.previous_odom_msg)
            
            # rospy.loginfo("Releasing lock in odom callback.")


        self.previous_odom_msg = ros_data
                

    def publish_images(self, odom_msg):
        ''' Produce and publish images'''
        self.image_message.header.stamp = odom_msg.header.stamp
        
        # Set the camera in the simulation
        teleport_msg = IsaacPoseRequest()
        teleport_msg.names = ["/World/Camera"]
        # Transform the physics odometry pose to the isaac frame
        isaac_pose = self.pose_to_isaac(odom_msg)
        if isaac_pose is None:
            rospy.loginfo("Couldn't teleport, won't publish images.")
            return
        
        teleport_msg.poses = [isaac_pose]

        # DEBUGGING
        # Convert the position and orientation components to strings
        position_str = "Position: ({}, {}, {})".format(isaac_pose.position.x, isaac_pose.position.y, isaac_pose.position.z)
        orientation_str = "Orientation: ({}, {}, {}, {})".format(isaac_pose.orientation.x, isaac_pose.orientation.y, isaac_pose.orientation.z, isaac_pose.orientation.w)
        # Format the complete Pose information as a string
        pose_info = "\n".join([position_str, orientation_str])
        # Print the Pose information using rospy.loginfo
        rospy.loginfo("Trying to move the camera to:\n%s in /isaac_sim frame", pose_info)

        # Call the service
        self.service_proxy(teleport_msg)

        # rospy.loginfo("Teleported")

        # TODO: (michbaum) Need a new solution for collision
        # # Set camera in unrealcv
        # if self.collision_on:
        #     # Apparently only camera0 can be moveto-ed, but the localization is the same as the fusion camera
        #     client.request('vset /camera/{0:d}/moveto {1:f} {2:f} {3:f}'.format(self.camera_id, *position))
        #     # Trying to make sure it uses the fusion camera location
        #     # client.request('vset /camera/{0:d}/moveto {1:f} {2:f} {3:f}'.format(1, *position))

        #     # Check collision
        #     # position_eff = client.request('vget /camera/%d/location' % self.camera_id)
        #     # Trying to make sure it uses the fusion camera location
        #     position_eff = client.request('vget /camera/1/location')
        #     position_eff = np.array([float(x) for x in str(position_eff).split(' ')])
        #     # TODO: (michbaum) Check if this still works in Isaac as expected
        #     if np.linalg.norm(position - position_eff) >= self.collision_tolerance:
        #         self.on_collision()
        # else:
        #     # I think we can only really affect camera0, but camera1 will follow
        #     client.request("vset /camera/{0:d}/location {1:f} {2:f} {3:f}".format(self.camera_id, *position))
        #     # Trying to make sure it uses the fusion camera location
        #     # client.request("vset /camera/{0:d}/location {1:f} {2:f} {3:f}".format(1, *position))

        # # I think we can only really affect camera0, but camera1 will follow
        # client.request("vset /camera/{0:d}/rotation {1:f} {2:f} {3:f}".format(self.camera_id, *orientation))
        # # Trying to make sure it uses the fusion camera rotation
        # # client.request("vset /camera/{0:d}/rotation {1:f} {2:f} {3:f}".format(1, *orientation))

        self.odom_ready = True

    def pose_to_isaac(self, pose):
        # Transform the pose from /camera to /camera_sim frame
        try:
            # Create a TransformStamped object to store the transformed pose
            # transformStamped = self.tfBuffer.lookup_transform("camera_sim", "sim_world", pose.header.stamp)
            transformStamped = self.tfBuffer.lookup_transform("sim_world", "camera_sim", pose.header.stamp)

            # transformStamped = self.tfBuffer.lookup_transform("camera_sim", "world", rospy.Time(0))
            # transformStamped = self.tfBuffer.lookup_transform("world", "camera_sim", rospy.Time(0))
            # transformStamped = self.tfBuffer.lookup_transform("firefly/base_link", "camera_sim", rospy.Time(0))
            # transformStamped = self.tfBuffer.lookup_transform("camera_sim", "firefly/base_link", rospy.Time(0))
            # transformStamped = self.tfBuffer.lookup_transform("sim_world", "world", rospy.Time(0))
            # transformStamped = self.tfBuffer.lookup_transform("world", "sim_world", rospy.Time(0))

            # Use do_transform_pose to transform the pose
            # transformed_pose = do_transform_pose(pose, transformStamped)

            transformed_pose = Pose()
            transformed_pose.orientation = transformStamped.transform.rotation
            transformed_pose.position.x = transformStamped.transform.translation.x
            transformed_pose.position.y = transformStamped.transform.translation.y
            transformed_pose.position.z = transformStamped.transform.translation.z

            # return transformed_pose
            return transformed_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed!")
            return None

    def publish_tf_data(self, odom_msg):
        pos = odom_msg.pose.pose.position
        rot = odom_msg.pose.pose.orientation
        self.tf_br.sendTransform((pos.x, pos.y, pos.z), (rot.x, rot.y, rot.z, rot.w), odom_msg.header.stamp,
                                 "camera_link", "world")

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