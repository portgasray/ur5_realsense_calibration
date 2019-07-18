#!/usr/bin/env python

from math import pi
import sys
import copy
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
import cv2
import cv_bridge
from copy import deepcopy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal=True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    return True

def image_callback(msg):
    image=cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow=np.array([0, 43, 46])
    upper_yellow=np.array([34, 255, 255])
    mask=cv2.inRange(hsv, lower_yellow, upper_yellow)
    (_, cnts, _)=cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h, w, d = image.shape
    M=cv2.moments(mask)
    if M['m00'] > 0:
        cx=int(M['m10']/M['m00'])
        cy=int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 10, (0,0,0), -1)
        cv2.putText(image, "({}, {})".format(int(cx), int(cy)), (int(cx-5), int(cy+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.drawContours(image, cnts, -1, (255, 255, 255),1)
    cv2.namedWindow("window", 1)
    cv2.imshow("window", image)
    cv2.waitKey(0)
        # print ("============ cx= ", cx, "      cy= ", cy)
    return cx,cy

class MoveGroupTutorial(object):
    def __init__(self):
        super(MoveGroupTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('test_move', anonymous=True)
        robot=moveit_commander.RobotCommander()
        scene=moveit_commander.PlanningSceneInterface()
        group_name="manipulator"
        group=moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher=rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        planning_frame=group.get_planning_frame()
        print ("============ Reference frame: %s" % planning_frame)
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        # self.track_flag=False
        # self.default_pose_flag=True
        self.cx=400.0
        self.cy=400.0
        self.bridge=cv_bridge.CvBridge()
        self.image_sub=rospy.Subscriber('/camera/color/image_raw', Image, image_callback)

    def go_to_ready_pose(self):
        group=self.group
        joint_goal=group.get_current_joint_values()
        print(type(joint_goal), joint_goal)

        joint_goal[0]= pi * 0.5
        joint_goal[1]= -pi * 0.5
        joint_goal[2]= -pi * 0.5
        joint_goal[3]= -pi * 0.5
        joint_goal[4]= pi * 0.5
        joint_goal[5]= 0

        group.go(joint_goal, wait=True)
        group.stop()
        current_joints=self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):
        group=self.group
        current_pose=group.get_current_pose().pose
        print("Current pose: ", current_pose)
        pose_goal=geometry_msgs.msg.Pose()
        pose_goal.orientation.x= 0.5
        pose_goal.orientation.y= 0.5
        pose_goal.orientation.z= -0.5
        pose_goal.orientation.w= 0.5
        pose_goal.position.x= 0.0 #0
        pose_goal.position.y= -0.5 #-0.5
        pose_goal.position.z= 0.07 #0.44
        group.set_pose_target(pose_goal)
        plan=group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        current_pose=group.get_current_pose().pose
        print("New current pose: ", current_pose)
        return all_close(pose_goal, current_pose, 0.01)

    def go_to_put_down(self):
        group=self.group
        current_pose=group.get_current_pose().pose
        print("Current pose: ", current_pose)
        pose_goal=geometry_msgs.msg.Pose()
        pose_goal.orientation.x= 0.5
        pose_goal.orientation.y= 0.5
        pose_goal.orientation.z= -0.5
        pose_goal.orientation.w= 0.5
        pose_goal.position.x= -0.20 #0
        pose_goal.position.y= -0.8 #-0.5
        pose_goal.position.z= 0.08 #0.44
        group.set_pose_target(pose_goal)
        plan=group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        current_pose=group.get_current_pose().pose
        print("New current pose: ", current_pose)
        return all_close(pose_goal, current_pose, 0.01)


#   using OpenCV here to get XYZ
#   def find_position(self):
# def image_callback(msg):
#     image=cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
#     hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#     lower_yellow=np.array([0, 43, 46])
#     upper_yellow=np.array([34, 255, 255])
#     mask=cv2.inRange(hsv, lower_yellow, upper_yellow)
#     (_, cnts, _)=cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     h, w, d = image.shape
#     M=cv2.moments(mask)
#     if M['m00'] > 0:
#         cx=int(M['m10']/M['m00'])
#         cy=int(M['m01']/M['m00'])
#         # cv2.namedWindow("window", 1)
#         # cv2.imshow("window", image)
#         # cv2.waitKey(0)
#         #print ("============ cx= ", cx, "      cy= ", cy)
#     return cx,cy
# #   def open_xipan(self):

# #   def close_xipan(self):


# tutorial=MoveGroupTutorial()
def main():
    try:
        print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander"
        raw_input()
        tutorial=MoveGroupTutorial()

        print "============ Press `Enter` to go to ready pose"
        raw_input()
        tutorial.go_to_ready_pose()

        print "============ Press `Enter` to go to pose goal and get the box"
        raw_input()
        tutorial.go_to_pose_goal()
        # cx,cy=image_callback()
        # print ("============ cx,cy", cx,cy)
        #tutorial.image_callback(self, msg)
       # tutorial.open_xipan()

        print "============ Press `Enter` to put down the box"
        raw_input()
        tutorial.go_to_put_down()
        #tutorial.close_xipan()

        print "============ Press 'Enter' to return to ready pose"
        raw_input()
        tutorial.go_to_ready_pose()
        raw_input()
        print "============ Finished"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__=='__main__':
    main()
