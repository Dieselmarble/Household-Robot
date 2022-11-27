#!/usr/bin/env python3

from __future__ import print_function
from cmath import pi
from math import copysign, dist, sqrt, pow
import rospy
import ros_numpy as rn
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from rocon_std_msgs.msg import StringArray
from sensor_msgs.msg import PointCloud2
from robot_vision_pkg.msg import Robot3D_NEW
from std_srvs.srv import Trigger, TriggerRequest
from move_base_seq import MoveBaseSeq as base_controller
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Vector3
from move_arm_seq import move_arm_log as arm_controller
import math
import threading
from semantic_map_server.srv import SemanticMapQuery, SemanticMapQueryResponse


Gripping_length = rospy.get_param('robot_arm/GRIPPING_LENGTH')
Z_DISTANCE= rospy.get_param('robot_arm/Z_DISTANCE')
X_DISTANCE = rospy.get_param('robot_arm/X_DISTANCE')
GRABE_WIDE = rospy.get_param('robot_arm/GRAB_WIDTH')
Raw_X_axis= rospy.get_param('robot_arm/Raw_X_AXIS')
Raw_X_Object = rospy.get_param('robot_arm/Raw_X_OBJECT')
ANGLE_DISTANCE = rospy.get_param('robot_arm/ANGLE_DISTANCE')
Z_AXIS_LENGTH = rospy.get_param('robot_arm/Z_AXIS_LENGTH')
'''
Base class for all high-level task inferitance
'''
class ExecuteTaskNode():
    def __init__(self):
        self.joint_state = None
        self.point_cloud = None
        self.base_controller = base_controller()
        self.base_controller.__init__()
        self.faceID = None
        self.arm_comtroller = arm_controller()
        self.arm_comtroller.__init__()
        self.object_pose_matrix = None
        self.object_pose_euler = None
        self.joint_states_lock = threading.Lock()
        self.move_lock = threading.Lock()
        self.moving_event = threading.Event()
        self.position_cache = None
        self.obstacle_pixel_thresh = 10
        
    def detect_obstacle_distance(self, point_cloud_msg):
        if point_cloud_msg:
            cloud_time = point_cloud_msg.header.stamp
            cloud_frame = point_cloud_msg.header.frame_id
            point_cloud = rn.numpify(point_cloud_msg)
            # xyz value of point cloud
            xyz = rn.point_cloud2.get_xyz_points(point_cloud)
            if len(xyz) == 0: return None
            dist_total = 0
            for point in xyz:
                dist_total += point[2]
            dist_avg = dist_total/len(xyz)
            return dist_avg
        
    def set_moving_event(self):
        self.moving_event.set()

    def unset_moving_event(self):
        self.moving_event.clear()
        rospy.loginfo("moving event set false")

    def get_moving_event_status(self):
        return self.moving_event.is_set()

    def get_object_pose_euler(self):
        return self.object_pose_euler

    def get_object_pose_matrix(self):
        return self.object_pose_matrix

    def get_faceID(self):
        return self.faceID

    '''
        #points_seq = [-1.5,0.9,0, -1.6,0.4,0, -0.02,-1.6,0]
        #angles_seq = [0,20,45]
    '''
    def move_to_position(self, trans, rot):
        # send target x,y, position and angle of the 
        # robot to move_base action server
        # send target position
        self.base_controller.set_target_position(trans, rot)
        ret = self.base_controller.move_to_pose()
        # clear_points_
        self.base_controller.clear_points_seq()
        return ret

    def arm_to_position(self, arm_position):
        #input x_movement, z_movement, grasp_angle, grabe_wide
        # send arm target position to controller and move
        return self.arm_comtroller.move_position_command(arm_position)

    def arm_to_init_position(self):
        self.arm_comtroller.move_init_position()

    def move_base_arm(self,X,Y,Z):
        move_base_forward,  Z, base_angle = inverse_kinematics_base(X,Y,Z)
        self.turn_around_with_angle(base_angle)  # input left <0 ,right>0
        self.move_forward_with_distance(move_base_forward)

    def point_cloud_callback(self, point_cloud):
        self.point_cloud = point_cloud

    def get_point_cloud_array(self):
        return self.point_cloud

    def send_voice_message(self, msg):
        try:
            self.voice_publisher.publish(msg)
        except:
            rospy.loginfo("send voice failed")
            return

    def faceID_callback(self, faceID):
        self.faceID = faceID.strings

    def get_p1_to_p2_matrix(self, p1_frame_id, p2_frame_id, lookup_time=None, timeout_s=None):
        try:
            if lookup_time is None:
                lookup_time = rospy.Time(0) # most recent transform
            if timeout_s is None:
                timeout_s = rospy.Duration(0.1)
            else:
                timeout_s = rospy.Duration(timeout_s)
            trans, rot = self.tf_listener.lookupTransform(p1_frame_id, p2_frame_id, rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException) as e:
            rospy.logdebug('  exception = ', e)
        return None, None

    def get_position(self):
        try:
            trans, rot = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException) as e:
            rospy.loginfo("TF has exception error")
            rospy.logdebug('  exception = ', e)
            return
        return Point(*trans)


    def get_position_in_map_xyza(self):
        try:
            trans, rot = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException) as e:
            rospy.loginfo("TF has exception error")
            rospy.logdebug('  exception = ', e)
            return
        euler = euler_from_quaternion(rot)
        euler_d = []
        for e in euler:
            euler_d.append(e*180/pi)
        return trans, euler_d


    def get_yaw_angle(self):
        try:
            trans, rot = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except:
            rospy.loginfo("TF has exception error")
            return
        yaw = euler_from_quaternion(rot)[2]*180/pi
        return yaw

    def get_camera_pose_to_base(self):
        try:
            trans, rot = self.tf_listener.lookupTransform(self.base_frame, self.camera_frame, rospy.Time(0))
        except:
            rospy.loginfo("TF has exception error")
            return
        return trans, rot

    '''
        camera axis to base axis
    '''
    
    def get_object_pose_in_base_frame(self,camera_3d):
        time_now = rospy.Time.now()
        if (time_now - camera_3d.header.stamp) < rospy.Duration.from_sec(0.2):
            trans, rot = self.get_camera_pose_to_base()
            trans_matrix = tf_to_transformation_matrix(trans, rot)
            pose_matrix = np.linalg.inv(trans_matrix)
            pose_matrix[0, 3] = camera_3d.X
            pose_matrix[1, 3] = camera_3d.Y
            pose_matrix[2, 3] = camera_3d.Z
            object_pose_matrix = np.dot(trans_matrix, pose_matrix)
            object_pose_euler = matrix2euler_deg(object_pose_matrix)
            self.object_pose_matrix = object_pose_matrix
            self.object_pose_euler = object_pose_euler
        else:
            self.object_pose_matrix = None
            self.object_pose_euler = None
            return
    '''
        arm axis to base axis
    '''
    def tranT_armTobase(self): 
        try:
            trans, rot = self.tf_listener.lookupTransform(self.arm_frame, self.base_frame, rospy.Time(0))
        except:
            rospy.loginfo("TF has exception error")
        T_mat = tf_to_transformation_matrix(trans, rot)

        robot_pose_matrix =self.get_object_pose_matrix()
        arm_pose_mat = np.dot(T_mat, robot_pose_matrix)
        arm_pose = matrix2euler_deg(arm_pose_mat)
        return arm_pose_mat, arm_pose

    def stop_the_robot(self):
        # stop the robot by sending all zeros
        move_cmd = Twist()
        move_cmd.linear = Vector3(0,0,0)
        move_cmd.angular = Vector3(0,0,0)
        self.cmd_vel_publisher.publish(move_cmd)

    '''
        + anticlockwise
        - clockwise
    '''
    def turn_around_with_angle(self, given_angle):
        tolerance = 1 # 5 degree tolerance
        keep_moving = True
        current_angle = self.get_yaw_angle()
        yaw_start = current_angle
        rate = rospy.Rate(5)
        while self.moving_event.isSet():
            move_cmd = Twist()
            if keep_moving:
                #rospy.loginfo("rotating in etn, move_event %s", self.moving_event.is_set())
                current_angle= self.get_yaw_angle()
                angle_diff = angle_diff_deg(current_angle, yaw_start)
                error = given_angle - angle_diff
                #rospy.loginfo("error is %s", error)
                if not keep_moving or abs(error) < tolerance:
                    keep_moving = False
                else:
                    move_cmd.angular.z = copysign(self.turn_speed, error)
            else:
                rospy.loginfo('turned %s ', given_angle)
                break
            #rospy.loginfo("send speed command %s" ,move_cmd)
            self.cmd_vel_publisher.publish(move_cmd)
            rate.sleep()
            # stop the robot
            move_cmd = Twist()
            self.cmd_vel_publisher.publish(move_cmd)

    '''
        + forward
        - backward
    '''
    def move_forward_with_distance(self, given_distance):
        tolerance = 0.05 # 5cm tolerance
        keep_moving = True
        self.position = self.get_position()
        if not self.position is None:
            x_start = self.position.x
            y_start = self.position.y
        else:
            self.position = self.get_position()
        rate = rospy.Rate(5)
        while self.moving_event.isSet():
            move_cmd = Twist()
            if keep_moving:
                self.position = self.get_position()
                # moved distance always positive
                moved_distance = distance_between_two_points(self.position.x, x_start, self.position.y, y_start)
                moved_distance = copysign(moved_distance, given_distance)
                error = moved_distance - given_distance
                if not keep_moving or abs(error) < tolerance:
                    keep_moving = False
                else:
                    move_cmd.linear.x = copysign(self.move_speed, -1*error)
            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y
                keep_moving = False
                rospy.loginfo('moved %s forward', given_distance)
                break
            self.cmd_vel_publisher.publish(move_cmd)
            rate.sleep()
        # stop the robot
        move_cmd = Twist()
        self.cmd_vel_publisher.publish(move_cmd)

    '''
        move forward until certain distance to obstacle
    '''
    def move_forward_until_contact(self, thresh):
        rate = rospy.Rate(5)
        self.position = self.get_position()
        if not self.position is None:
            x_start = self.position.x
            y_start = self.position.y
        else:
            self.position = self.get_position()
        keep_moving = True
        while not rospy.is_shutdown():
            move_cmd = Twist()
            if keep_moving:
                move_cmd.linear.x = self.move_speed
                self.cmd_vel_publisher.publish(move_cmd)
                point_cloud = self.get_point_cloud_array()
                dist = self.detect_obstacle_distance(point_cloud)
                rospy.loginfo("point cloud obstacle distance %s", dist)
                # recorded moved distance
                self.position = self.get_position()
                moved_distance = distance_between_two_points(self.position.x, x_start, self.position.y, y_start)
                rospy.loginfo("moved distance %s", moved_distance)
                if (dist and dist < thresh) or (moved_distance > 0.8):
                    rospy.loginfo("moved until contact, stop")
                    self.stop_the_robot()
                    break
                rate.sleep()
        self.stop_the_robot()
        
    def recognize_face_multiple_times(self):
        list_faceID = []
        for i in range(0,4):
            faceID = self.get_faceID() # faceID is a list of all faces saw
            rospy.loginfo("faceID %s", faceID)
            if faceID: # if not empty
                # only pick the first face
                face = faceID[0]
                rospy.loginfo("saw face: " + face)
                list_faceID.append(face)
                name = max(list_faceID, key=list_faceID.count)
                return name
        else: 
            return "我还没见过你"

    def shutdown(self):
        rospy.loginfo('Shutdown ROS node ' + rospy.get_name() +' connected to /stop_the_robot service.')
        self.stop_the_robot()
        return  

    def get_object_location_from_map_server(self, name):
        rospy.wait_for_service('SemanticServer')
        #trans, rot = [2.690,0.252,0.000], [0.00,0.00,172.38]
        try:
            get_goal_func = rospy.ServiceProxy('SemanticServer', SemanticMapQuery)
            resp = get_goal_func(name)
            trans = list(resp.translation)
            rot = list(resp.rotation)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s",e)
        return trans, rot

    def main(self, wait_for_first_pointcloud=False, wait_for_audio_tts=False):
        # camera 2 point cloud subscriber
        self.point_cloud_subscriber = rospy.Subscriber('/cam_2/depth/color/points/filtered', PointCloud2, self.point_cloud_callback)
        if wait_for_first_pointcloud:
            # Do not start until a point cloud has been received
            point_cloud_msg = self.point_cloud
            rospy.loginfo('etn waiting to receive first point cloud.')
            while point_cloud_msg is None:
                rospy.sleep(0.1)
                point_cloud_msg = self.point_cloud
        
        self.voice_publisher = rospy.Publisher('/speak', String, queue_size=2)
        self.faceID_subscriber = rospy.Subscriber('/faces/faceID', StringArray, self.faceID_callback)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.camera_pose_subscriber = rospy.Subscriber('/Camera3D_object', Robot3D_NEW, self.get_object_pose_in_base_frame)

        self.tf_listener = tf.TransformListener()
        self.map_frame = rospy.get_param('~map_frame', '/map')
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')
        self.odom_frame = rospy.get_param('~odom_frame', "/odom")
        self.arm_frame = rospy.get_param('~robot_arm','/arm_link')
        self.odom_linear_scale_corretion = rospy.get_param('~odom_linear_correction', 1.0)
        self.move_speed = rospy.get_param('~speed', 0.2)
        self.turn_speed = rospy.get_param('~theta_speed', 0.30)
        self.camera_frame = rospy.get_param('~camera','/cam_2_link')
        
        if wait_for_audio_tts:
            voice_ok = False
            duration = rospy.Duration.from_sec(3)
            start_time = rospy.Time.now()
            while not voice_ok or duration>rospy.Time.now()-start_time:
                try:
                    self.send_voice_message("Hello")
                except rospy.ROSInternalException as e:
                    rospy.loginfo("voice topic not available %s", e)
                voice_ok = True
            rospy.loginfo("voice TTS started, checked")  

        rospy.sleep(2) # give if some time to fill the buffer
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
        rospy.loginfo("Init all publisher and subscribers")  
        rospy.on_shutdown(self.shutdown)

def angle_diff_rad(target_rad, current_rad):
    diff_rad = target_rad - current_rad
    diff_rad = ((diff_rad + pi) % (2.0 * pi)) - pi
    return diff_rad

def angle_diff_deg(target_deg, current_deg):
    diff_deg = target_deg - current_deg
    diff_deg = ((diff_deg + 180.0) % 360.0) - 180.0
    return diff_deg

def distance_between_two_points(x_target, x_current, y_target, y_current):
    return sqrt(pow((x_target - x_current), 2) + pow((y_target - y_current), 2))

def inverse_kinematics_grasp(pose):  #input m
    X, Y, Z = int(pose[0] * 1000), int(pose[1] * 1000), int(pose[2] * 1000)   
    if abs(Y) >Gripping_length:
        rospy.loginfo("move base failed!!! Y >Gripping length")
        return 0,0,0 
    theta = 180*(math.asin(abs(Y)/Gripping_length))/math.pi
    z = int(Z - Z_AXIS_LENGTH + 20)
    x = int(abs(X-Gripping_length*math.cos(math.pi*theta/180)-Raw_X_axis))  #
    print("The inverse_kinemateics resule Z axis ",z,"x axis",x,"angle",theta)
    return z, x, theta

def inverse_kinematics_base(pose):  #输入为底盘坐标系
    X,Y,Z =pose[0],pose[1],pose[2]
    theta = 180 *(math.atan(Y/X))/math.pi
    move_base_forward = round(X/ math.cos(math.pi*theta/180) - Raw_X_Object/1000- 0.2 ,2)#0.15
    #move_arm_forward =0.125 #500-375 底盘离目标500，375 中心离夹爪末端
    return move_base_forward,  Z, theta

def tf_to_transformation_matrix(trans, rot):
    """
    Parse the ROS message to a 4x4 pose format
    @param msg The ros message containing a pose
    @return A 4x4 transformation matrix containing the pose
    as read from the message
    """
    # Get translation and rotation (from Euler angles)
    pose_matrix = tf.transformations.quaternion_matrix(np.array(
        [rot[0], rot[1], rot[2], rot[3]]))
    pose_matrix[0, 3] = trans[0]
    pose_matrix[1, 3] = trans[1]
    pose_matrix[2, 3] = trans[2]
    return pose_matrix

def matrix2euler_deg(m):
    rx, ry, rz = rotation_matrix_to_euler_angels(m[0:3, 0:3])
    pos = np.squeeze(m[0:3, 3:4])
    return np.array([pos[0], pos[1], pos[2], math.degrees(rx), math.degrees(ry), math.degrees(rz)])

def is_rotation_matrix(R):
    R_t = np.transpose(R)
    should_be_identity = np.dot(R_t, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - should_be_identity)
    return n < 1e-6

def rotation_matrix_to_euler_angels(R):
    assert (is_rotation_matrix(R))
    sy = sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2,1], R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x,y,z])
