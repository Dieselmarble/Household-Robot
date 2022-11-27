#!/usr/bin/env python3

from numpy import angle
import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import ros_numpy as rn

class MoveBaseSeq():
    def __init__(self):
        self.pose_seq = list()
        self.goal_cnt = 0
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            #rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

    def set_target_position(self, points_seq, angles_seq):
        # rospy.set_param('move_base_seq/p_seq', [-1.5,0.9,0, -1.6,0.4,0, -0.02,-1.6,0])
        # rospy.set_param('move_base_seq/yea_seq', [0,20,45])
        # points_seq = rospy.get_param('move_base_seq/p_seq')
        # yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
        n = 3
        quat_seq = list()    
        for i in range(0, len(angles_seq), n):
            # omit the first two columns for serv
            quat_seq.append(Quaternion(*(quaternion_from_euler(angles_seq[i], angles_seq[i+1], angles_seq[i+2]*math.pi/180, axes='sxyz'))))

        points = [Point(*points_seq[i:i+n]) for i in range(0, len(points_seq), n)]

        for (point, angle) in zip(points, quat_seq):
            self.pose_seq.append(Pose(point, angle))

    def clear_points_seq(self):
        self.pose_seq.clear()
        self.goal_cnt = 0
        
    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                #rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            #rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            #rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def move_to_pose(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        try:
            self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
            wait = self.client.wait_for_result()
            result = False
            if not wait:
                rospy.logerr("Action server not available [move_to_pose]!")
                #rospy.signal_shutdown("Action server not available!")
            else:
                result = self.client.get_result()
            if result:
                rospy.loginfo("Action server goal execution done!")
                return True
            return False
        except rospy.ROSInternalException:
            rospy.loginfo("Navigation failed")
            
if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

