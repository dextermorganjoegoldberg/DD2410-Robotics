#!/usr/bin/env python3
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint
from geometry_msgs.msg import Twist
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot

# Global variables
tf_buffer, listener, goal_client, control_client, pub = None, None, None, None, None
robot_frame_id = "base_link" # Robot's reference frame
max_linear_velocity = 0.5 # Maximum allowed linear velocity
max_angular_velocity = 1.0 # Maximum allowed angular velocity

def compute_twist_from_setpoint(setpoint):
    """Compute a Twist message to move towards a given setpoint."""
    Twist_msg = Twist()
    # Calculate angular velocity based on the direction to the setpoint
    Twist_msg.angular.z = atan2(setpoint.point.y, setpoint.point.x)
    # Calculate linear velocity based on the distance to the setpoint
    Twist_msg.linear.x = hypot(setpoint.point.x, setpoint.point.y)
    
    # Clip velocities to ensure they are within allowed limits
    Twist_msg.linear.x = min(max_linear_velocity, Twist_msg.linear.x)
    if Twist_msg.angular.z >= 0:
        Twist_msg.angular.z = min(max_angular_velocity, Twist_msg.angular.z)
    else: 
        Twist_msg.angular.z = max(-max_angular_velocity, Twist_msg.angular.z)
    
    return Twist_msg

def move(path):
    """Follow a given path by sending appropriate Twist messages."""
    global control_client, robot_frame_id, pub
    
    while path.poses:
        response_service = control_client(path)
        new_path = response_service.new_path
        setpoint = response_service.setpoint

        # Transform the setpoint to the robot's reference frame
        transform = tf_buffer.lookup_transform(robot_frame_id, setpoint.header.frame_id , rospy.Time())
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)
        
        # Calculate and publish the necessary motion commands
        pub.publish(compute_twist_from_setpoint(transformed_setpoint))
        rate.sleep()

        path = new_path

    # Stop robot after completing the path
    pub.publish(Twist())
    rate.sleep()
    get_path()

def get_path():
    """Request a new path from the action server and start moving on it."""
    global action_client
    
    action_client.wait_for_server()
    get_goal = irob_assignment_1.msg.GetNextGoalGoal()
    action_client.send_goal(get_goal)
    action_client.wait_for_result()
    
    move(action_client.get_result().path)

if __name__ == "__main__":
    rospy.init_node("controller")
    rate = rospy.Rate(10.0)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    action_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rospy.wait_for_service('get_setpoint')
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)

    get_path()
    rospy.spin()
