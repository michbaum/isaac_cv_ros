#!/usr/bin/env python

# Unrealcv
from unrealcv import Client
client = Client(('172.20.0.1',9000))


# ros
import rospy
import threading
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from isaac_cv_ros.msg import IsaacSensorRaw
from isaac_ros_messages.srv import IsaacPose
from isaac_ros_messages.srv import IsaacPoseRequest
from geometry_msgs.msg import Pose
from std_srvs.srv import SetBool
import tf

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
        # self.mode = rospy.get_param('~mode', "standard")  # Client mode (test, standard, fast, fast2)
        self.collision_on = rospy.get_param('~collision_on', True)  # Check for collision
        self.publish_tf = rospy.get_param('~publish_tf', False)  # If true publish the camera transformation in tf
        # self.slowdown = rospy.get_param('~slowdown', 0.0)  # Artificially slow down rate for UE to finish rendering
        # self.camera_id = rospy.get_param('~camera_id', 0)  # CameraID for unrealcv compatibility (usually use 0)
        self.queue_size = rospy.get_param('~queue_size', 1)  # How many requests are kept
        self.lock = threading.lock()
        self.service_proxy = rospy.ServiceProxy('teleport', IsaacPose, persistent=True)

        # Select client mode
        # mode_types = {'standard': 'standard', 'fast': 'fast', 'test': 'test', 'generate': 'generate'}
        # selected = mode_types.get(self.mode, 'NotFound')
        # if selected == 'NotFound':
        #     warning = "Unknown client mode '" + self.mode + "'. Implemented modes are: " + \
        #               "".join(["'" + m + "', " for m in mode_types])
        #     rospy.logfatal(warning[:-2])

        # Setup unrealcv client
        # client.connect()
        # if not client.isconnected():
        #     rospy.logfatal("No unreal game running to connect to. Please start a game before launching the node.")

        # status = client.request('vget /unrealcv/status')
        # if status is None:
        #     rospy.logfatal("Error addressing the unrealcv client. Try restarting the game.")

        # TODO: (michbaum) Setup all the camera subscribers and get the camera_info parameters
        self.cam_info_sub = rospy.Subscriber("camera_info", CameraInfo, self.cam_info_callback, queue_size=self.queue_size)        
        # TODO: (michbaum) Check if the buffer size is an issue
        self.odom_sub = rospy.Subscriber("odometry", Odometry, self.odom_callback, queue_size=self.queue_size)
        
        self.previous_odom_msg = None  # Previously processed Odom message
        self.collision_tolerance = rospy.get_param('~collision_tol', 10)  # Distance threshold in UE units


        # Initialize relative coordinate system (so camera starts at [0, 0, 0] position and [0, 0, yaw]).

        # This location is hardcoded for the warehouse sim - change accordingly
        location = np.array([5.0, 5.0, 2.0]) # x, y, z
        rotation = np.array([1 / math.sqrt(2), 0, 0, 1 / math.sqrt(2)]) # x, y, z, w Quaternion
        self.coord_origin = location
        self.coord_rotation = rotation

        # Set the camera to the origin
        origin = Pose()
        origin.position = location
        origin.orientation = rotation
        teleport_msg = IsaacPoseRequest()
        teleport_msg.names = ["/World/Camera"]
        teleport_msg.poses = [origin]

        # Call the service
        self.service_proxy(teleport_msg)


        # We populate the image message centrally as soon as a new odometry message is received
        # TODO: (michbaum) Maybe needs a mutex
        self.image_message = IsaacSensorRaw()
        self.image_message.color_data = None
        self.image_message.depth_data = None
        self.odom_ready = False

        # Finish setup
        self.pub = rospy.Publisher("~ue_sensor_raw", IsaacSensorRaw, queue_size=10)
        rospy.Service('~terminate_with_reset', SetBool, self.terminate_with_reset_srv)
        if self.collision_on:
            self.collision_pub = rospy.Publisher("~collision", String, queue_size=10)
        rospy.loginfo("isaac_ros_client is ready in %s mode." % self.mode)

    def cam_info_callback(self, cam_info_data):
        # Setup camera parameters from isaac sim config

        height = cam_info_data.height
        width = cam_info_data.width
        f = cam_info_data.K[0]

        rospy.loginfo("Camera parameters set as %f %f %f" % (height, width, f))
        rospy.set_param('~camera_params', {'width': float(width), 'height': float(height), 'focal_length': float(f)})

        # Unsubscribe from the camera info topic and instead subscribe to the rgb and depth topics
        self.cam_info_sub.unregister()
        self.rgb_sub = rospy.Subscriber("rgb", Image, self.rgb_callback, queue_size=self.queue_size,
                                         buff_size=(2 ** 24) * self.queue_size)
        
        self.depth_sub = rospy.Subscriber("depth", Image, self.depth_callback, queue_size=self.queue_size,
                                         buff_size=(2 ** 24) * self.queue_size)
        
    def rgb_callback(self, rgb_data):
        # We only collect an image if we teleported to the next position
        if not self.odom_ready:
            return
        
        # Check if rgb image is already written
        if self.image_message.rgb_data is not None:
            # Acquire lock for the message
            with self.lock:
                self.image_message.rgb_data = rgb_data.data

                # Check if depth has also been written, then we can publish
                if self.image_message.depth_data is not None:
                    self.pub.publish(self.image_message)

                    # Reset the message
                    self.odom_ready = False
                    self.image_message = IsaacSensorRaw()
                    self.image_message.rgb_data = None
                    self.image_message.depth_data = None


    def depth_callback(self, depth_data):
        # We only collect an image if we teleported to the next position
        if not self.odom_ready:
            return
        
        # Check if rgb image is already written
        if self.image_message.depth_data is not None:
            # Acquire lock for the message
            with self.lock:
                self.image_message.depth_data = depth_data.data

                # Check if rgb has also been written, then we can publish
                if self.image_message.rgb_data is not None:
                    self.pub.publish(self.image_message)

                    # Reset the message
                    self.odom_ready = False
                    self.image_message = IsaacSensorRaw()
                    self.image_message.rgb_data = None
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

        # We should only do something if the last message has been successfully sent
        if self.odom_ready:
            return

        # Get pose in unreal coords
        # TODO: (michbaum) Check if this is still necessary
        # position, orientation = self.transform_to_unreal(ros_data.pose.pose)

        # Generate images - but only after the last set of images has been sent
        if self.previous_odom_msg is not None:
            # This is not the initialization step
            # TODO: (michbaum) They actually stagger the image generation and the pose setting, such that unreal has enough time to
            # render the images. If we try to decouple pose setting and image retrieval, this could be a potential problem.

            # TODO: (michbaum) Need a lock
            with self.lock:
                self.publish_images(self.previous_odom_msg)


        self.previous_odom_msg = ros_data
                

    def publish_images(self, odom_msg):
        ''' Produce and publish images'''
        self.image_message.header.stamp = odom_msg.header.stamp
        
        # Set the camera in the simulation
        teleport_msg = IsaacPoseRequest()
        teleport_msg.names = ["/World/Camera"]
        teleport_msg.poses = [odom_msg]

        # Call the service
        self.service_proxy(teleport_msg)

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
        goal.position = self.coord_origin
        goal.orientation = self.coord_rotation
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