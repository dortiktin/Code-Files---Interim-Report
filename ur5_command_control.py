#!/usr/bin/env python

####### מסמן שלא היה בסרטון של יחידה3 במנפולציה
import sys
import rospy
import moveit_commander  # Communicate with RViz, provides with `MoveGroupCommander`,`PlanningSceneInterface`,`RobotCommander` classes (More on these below)
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String ############
from moveit_commander.conversions import pose_to_list ############

class MoveGroup_PythonInteface(object):
  # Finished
  def __init__(self):
    super(MoveGroup_PythonInteface, self).__init__()  ###############
    # First initialize `moveit_commander`
    moveit_commander.roscpp_initialize(sys.argv)
    # Initialize a `rospy` node
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    # Instantiate a `RobotCommander` object. This object is the outer-level interface to the robot
    self.robot = moveit_commander.RobotCommander()
    # Instantiate a `PlanningSceneInterface` object. This object is an interface to the world surrounding the robot
    self.scene = moveit_commander.PlanningSceneInterface()
    # Instantiate a `MoveGroupCommander` object. This object is an interface to one group of joints.
    # In this case the group is the UR5 joints. This allow to interact with this group
    group_name = "manipulator"
    self.group = moveit_commander.MoveGroupCommander(group_name)
    # Ceate a `DisplayTrajectory` publisher which is used later to publish trajectories for RViz to visualize
    self.display_trajectory_publisher  = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    # Various self variables
    self.planning_frame = self.group.get_planning_frame()  # Name of the reference frame for the robot
    ## self.eef_link = self.group.get_end_effector_link()     # Name of the end-effector link for this group
    self.group_names = self.robot.get_group_names()        # List of all the groups in robot
    # Print basic information on the robot
    print("============Reference frame: %s" % self.planning_frame)
    print("============Robot groups:", self.robot.get_group_names())
    print("============Robot state:", self.robot.get_current_state())
    print("")
    
  # Finished, goal,actual are a list of floats, a Pose or a PoseStamped, tolerance is a float, return bool
  def all_close(goal, actual, tolerance): # Check if a list of values are within a tolerance of their counterparts in another list
    all_equal = True
    if type(goal) is list:
      for index in range(len(goal)):
        if abs(actual[index] - goal[index]) > tolerance:
          return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
      return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
      return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

  # Finished and work!
  def go_to_joint_state(self, joints_goal):  # Planning to a Joints Goal, joints_goal is list of 6 joints values
    # Call the planner to compute the plan and execute it 
    self.group.go(joints_goal, wait=True)
    # Ensures that there is no residual movement
    self.group.stop()
    # For testing:
    # current_joints = self.group.get_current_joint_values()
    # return all_close(joints_goal, current_joints, 0.01)

  # Finished and work!, goal_position = dict[orientation_xyzw, position_xyz]
  def go_to_pose_goal(self, goal_position):  # Plan a motion for this group to a desired pose for the eff
    self.group.set_pose_target(goal_position)
    # Call the planner to compute the plan and execute it
    self.group.go(wait=True)
    # Ensures that there is no residual movement
    self.group.stop()
    # Clear your targets after planning with poses. Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()
    ## For testing:
    ## current_pose = self.group.get_current_pose().pose
    ## return all_close(goal_position, current_pose, 0.01)

  # Finished
  def plan_cartesian_path(self, waypoints, scale=1):  # Plan a cartesian path directly by specifying a list of waypoints for the eef to go through
    # Interpolated at a resolution of 0.01[m] and disable the jump threshold by setting it to 0.0[m]
    (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction
    
def main():
  try:
    my_MoveGroup = MoveGroup_PythonInteface()

    print("============Execute a movement using a joint state goal")
    # joints = ['shoulder',  'upper_arm',  'fore_arm', 'wrist1', 'wrist2', 'wrist3']
    joints_target = [0,          -pi/2,        -pi/4,      0,        0,        0]
    my_MoveGroup.go_to_joint_state(joints_target)

    ##print("============Execute a movement using a pose goal")
    ##pose = my_MoveGroup.group.get_current_pose().pose
    ##pose.position.z -= 0.2  # First move up (z)
    ##pose.position.y += 0.2  # and sideways (y)
    ##my_MoveGroup.go_to_pose_goal(pose)

    ## way_points = [pose1, pose2, pose3]
    ## print("============Plan and display a Cartesian path")
    ## cartesian_plan, fraction = my_MoveGroup.plan_cartesian_path(way_points)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()