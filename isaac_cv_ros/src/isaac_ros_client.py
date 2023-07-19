#!/usr/bin/env python

# Unrealcv
from unrealcv import Client
client = Client(('172.20.0.1',9000))


# ros
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
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
        
        # TODO: (michbaum) In the camera info callback, unsubscribe from the camera info and subscribe to the actual image streams
        #                  Also set all the camera parameters there
        
        # self.rgb_sub = rospy.Subscriber("rgb", Odometry, self.odom_callback, queue_size=self.queue_size,
        #                                  buff_size=(2 ** 24) * self.queue_size)
        
        # self.depth_sub = rospy.Subscriber("depth", Odometry, self.odom_callback, queue_size=self.queue_size,
        #                                  buff_size=(2 ** 24) * self.queue_size)
        
        self.odom_sub = rospy.Subscriber("odometry", Odometry, self.odom_callback, queue_size=self.queue_size,
                                         buff_size=(2 ** 24) * self.queue_size)
        
        self.previous_odom_msg = None  # Previously processed Odom message
        self.collision_tolerance = rospy.get_param('~collision_tol', 10)  # Distance threshold in UE units


        # Initialize relative coordinate system (so camera starts at [0, 0, 0] position and [0, 0, yaw]).
        # location = client.request('vget /camera/%i/location' % self.camera_id)
        # Trying to make sure it uses the fusion camera location
        location = client.request('vget /camera/1/location')
        print(location)
        #print(location)
        self.coord_origin = np.array([float(x) for x in str(location).split(' ')])
        # rot = client.request('vget /camera/%i/rotation' % self.camera_id)
        # Trying to make sure it uses the fusion camera rotation
        rot = client.request('vget /camera/1/rotation')
        self.coord_yaw = float(str(rot).split(' ')[1])
        # Apparently only cam0 can be moved
        client.request("vset /camera/{0:d}/rotation 0 {1:f} 0".format(self.camera_id, self.coord_yaw))
        # Trying to make sure it changes the fusion camera rotation
        # client.request("vset /camera/{0:d}/rotation 0 {1:f} 0".format(1, self.coord_yaw))



        # Finish setup
        # TODO: (michbaum) Change sensor message type
        self.pub = rospy.Publisher("~ue_sensor_raw", UeSensorRaw, queue_size=10)
        rospy.Service('~terminate_with_reset', SetBool, self.terminate_with_reset_srv)
        if self.collision_on:
            self.collision_pub = rospy.Publisher("~collision", String, queue_size=10)
        rospy.loginfo("unreal_ros_client is ready in %s mode." % self.mode)

    def cam_info_callback(self, cam_info_data):
        # Setup camera parameters from unrealcv config

        # TODO: (michbaum) Currently, we need to adapt the isaac sim camera
        #                  to match our pinhole camera model.

        loc_width = status.find('Width:')
        loc_height = status.find('Height:')
        loc_fov = status.find('FOV:')
        loc_end = status.find('EnableInput:')
        width = int(status[loc_width + 7:loc_height])
        height = int(status[loc_height + 8:loc_fov])
        fov = float(status[loc_fov + 5:loc_end])
        f = width / 2 / np.tan(fov * math.pi / 180 / 2)
        rospy.set_param('~camera_params', {'width': float(width), 'height': float(height), 'focal_length': float(f)})

    def odom_callback(self, ros_data):
        ''' Produce images for given odometry '''
        # Profiling
        # pr = cProfile.Profile()
        # pr.enable()
        if self.should_terminate:
            return
        # Slowdown to give more rendering time to the unreal engine (should not be necessary in normal mode)
        time.sleep(self.slowdown)

        # Get pose in unreal coords
        position, orientation = self.transform_to_unreal(ros_data.pose.pose)

        # Generate images
        if self.previous_odom_msg is not None:
            # This is not the initialization step
            # TODO: (michbaum) They actually stagger the image generation and the pose setting, such that unreal has enough time to
            # render the images. If we try to decouple pose setting and image retrieval, this could be a potential problem.
            self.publish_images(self.previous_odom_msg.header.stamp)

        if self.publish_tf:
            self.publish_tf_data(ros_data)

        self.previous_odom_msg = ros_data

        # Set camera in unrealcv
        if self.collision_on:
            # Apparently only camera0 can be moveto-ed, but the localization is the same as the fusion camera
            client.request('vset /camera/{0:d}/moveto {1:f} {2:f} {3:f}'.format(self.camera_id, *position))
            # Trying to make sure it uses the fusion camera location
            # client.request('vset /camera/{0:d}/moveto {1:f} {2:f} {3:f}'.format(1, *position))

            # Check collision
            # position_eff = client.request('vget /camera/%d/location' % self.camera_id)
            # Trying to make sure it uses the fusion camera location
            position_eff = client.request('vget /camera/1/location')
            position_eff = np.array([float(x) for x in str(position_eff).split(' ')])
            if np.linalg.norm(position - position_eff) >= self.collision_tolerance:
                self.on_collision()
        else:
            # I think we can only really affect camera0, but camera1 will follow
            client.request("vset /camera/{0:d}/location {1:f} {2:f} {3:f}".format(self.camera_id, *position))
            # Trying to make sure it uses the fusion camera location
            # client.request("vset /camera/{0:d}/location {1:f} {2:f} {3:f}".format(1, *position))

        # I think we can only really affect camera0, but camera1 will follow
        client.request("vset /camera/{0:d}/rotation {1:f} {2:f} {3:f}".format(self.camera_id, *orientation))
        # Trying to make sure it uses the fusion camera rotation
        # client.request("vset /camera/{0:d}/rotation {1:f} {2:f} {3:f}".format(1, *orientation))

        # Profiling
        # pr.disable()
        # pr.print_stats(sort='time')
                

    def publish_images(self, header_stamp):
        ''' Produce and publish images for test and standard mode'''
        # Retrieve images
        # res_color = client.request('vget /camera/%d/lit png' % self.camera_id)
        # res_depth = client.request('vget /camera/%d/depth npy' % self.camera_id)
        # res_color_orig = client.request('vget /camera/%d/lit png' % self.camera_id)
        # res_depth_orig = client.request('vget /camera/%d/depth npy' % self.camera_id)
        # quick fix for New Maze environment, camera 1 is a fusion camera
        now = rospy.Time.now()
        res_color = client.request('vget /camera/1/lit png') # Takes ~0.25s
        rospy.loginfo("Time to get color image: %f", (rospy.Time.now() - now).to_sec())
        now = rospy.Time.now()
        res_depth = client.request('vget /camera/1/depth npy') # Takes ~0.3s
        rospy.loginfo("Time to get depth image: %f", (rospy.Time.now() - now).to_sec())
        #res_depth = np.fromstring(res_depth[80:], dtype=np.float32, count=-1, sep='')
        # Remove this header: "\x93NUMPY\x01\x00F\x00{'descr': '<f4', 'fortran_order': False, 'shape': (480, 640), }      \n"
        now = rospy.Time.now()
        res_depth = np.fromstring(res_depth[80:], dtype=np.float32, count=-1, sep='')
        # res_depth 
        # Publish data
        # plt.imshow(res_depth.reshape(480,640))
        # plt.show()
        # pdb.set_trace()

        
        msg = UeSensorRaw()
        msg.header.stamp = header_stamp
        msg.color_data = res_color
        msg.depth_data = res_depth
        

        self.pub.publish(msg)
        rospy.loginfo("Time to publish image/depth: %f s", (rospy.Time.now() - now).to_sec())

    def publish_tf_data(self, odom_msg):
        pos = odom_msg.pose.pose.position
        rot = odom_msg.pose.pose.orientation
        self.tf_br.sendTransform((pos.x, pos.y, pos.z), (rot.x, rot.y, rot.z, rot.w), odom_msg.header.stamp,
                                 "camera_link", "world")

    # TODO: (michbaum) I think we don't need this anymore
    def transform_to_unreal(self, pose):
        '''
        Transform from ros to default unreal coordinates.
        Input:      ros pose in global frame
        Output:     position ([x, y, z] array, in unreal coordinates)
                    orientation ([pitch, yaw, roll] array in unreal coordinates)
        '''
        # Read out vectors
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        # Transformation of position to relative UE coordsys
        yaw = self.coord_yaw / 180 * math.pi
        position = np.array([math.cos(yaw) * x + math.sin(yaw) * y, math.sin(yaw) * x - math.cos(yaw) * y, z])
        position = position * 100 + self.coord_origin  # default units are cm

        # Transformation of orientation to relative UE coordsys
        q_rot = tf.transformations.quaternion_from_euler(0, 0, -yaw)
        orientation = tf.transformations.quaternion_multiply(q_rot, orientation)
        (r, p, y) = tf.transformations.euler_from_quaternion(orientation)
        orientation = np.array([-p, -y, r]) * 180 / math.pi  # default units are deg
        return position, orientation

    @staticmethod
    def transform_from_unreal(position, orientation):
        '''
        Transform from unreal coordinates to odom coordinates (to camera_link)
        Input:      position [x, y, z,], orientation [pitch, yaw, roll], in unrealcv coordinates)
        Output:     position ([x, y, z] array)
                    orientation ([x, y, z, w] quaternion)
        '''

        # Transform from unreal coordinate units
        position = position / 100  # default is cm
        orientation = orientation / 180 * math.pi  # default is deg

        # Transform from pitch, yaw, roll (taking into account rotation directions
        (x, y, z, w) = tf.transformations.quaternion_from_euler(orientation[2], -orientation[0], -orientation[1])
        orientation = np.array([x, y, z, w])

        # Invert y axis
        position[1] = -position[1]
        return position, orientation

    def terminate_with_reset_srv(self, _):
        # Reset to initial conditions after experiment
        rospy.loginfo("Terminate_with_reset service requested, initiating reset.")
        self.should_terminate = True    # This stops the client from taking other requests (odom topic)
        goal = np.concatenate((self.coord_origin, np.array([0, self.coord_yaw, 0])), axis=0)
        time.sleep(2.0)     # Make sure regular execution is finished -> unrealcv client is ready
        
        # TODO: (michbaum) Exchange with service call
        # Apparently only cam0 can be moved
        client.request("vset /camera/{0:d}/pose {1:f} {2:f} {3:f} {4:f} {5:f} {6:f}".format(self.camera_id, *goal))
        # Trying to make sure that we use the fusion camera
        # client.request("vset /camera/{0:d}/pose {1:f} {2:f} {3:f} {4:f} {5:f} {6:f}".format(1, *goal))
        return [True, ""]

    def on_collision(self):
        # Collision handling for all modes here
        rospy.logwarn("MAV collision detected!")
        self.collision_pub.publish(String("MAV collision detected!"))

def main():
    rospy.init_node('isaac_ros_client', anonymous=False)  # Currently only 1 Client at a time is supported by isaaccv
    ic = IsaacRosClient()
    rospy.spin()

if __name__ == '__main__':
    # rospy.init_node('unreal_ros_client', anonymous=False)  # Currently only 1 Client at a time is supported by unrealcv
    # uc = UnrealRosClient()
    # rospy.spin()
    # cProfile.run('main()')
    main()
