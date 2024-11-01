#!/usr/bin/env python3
import os
import rospy
import subprocess
from geometry_msgs.msg import PoseStamped
from pyproj import Transformer
import numpy as np
from actionlib_msgs.msg import GoalStatusArray



def gps_to_cartesian(lat, lon):
    # Convert GPS coordinates to UTM using pyproj
    transformer = Transformer.from_crs("epsg:4326", "epsg:32718", always_xy=True)
    utm_x, utm_y = transformer.transform(lon, lat)
    print('GPS to UTM ', utm_x, utm_y)
    rospy.loginfo(f"GPS to UTM: UTM X = {utm_x}, UTM Y = {utm_y}")
    return utm_x, utm_y

def send_navigation_goal(goal_x, goal_y):
    # Publish navigation goal to move_base_simple/goal
    goal = PoseStamped()
    goal.header.frame_id = "odom"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = goal_x
    goal.pose.position.y = goal_y
    goal.pose.orientation.w = 1.0

    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10, latch=True)
    rospy.sleep(1)  # Allow time for publisher to initialize

    rospy.loginfo(f"Sending navigation goal: x={goal_x}, y={goal_y}")
    goal_pub.publish(goal)

def calculate_relative_position(utm_x, utm_y, o_T_u):
    # Calculate relative position in Odom frame
    utm_point = np.array([utm_x, utm_y, 1])
    odom_point = np.dot(o_T_u, utm_point)
    odom_x = odom_point[0]
    odom_y = odom_point[1]
    rospy.loginfo(f"Goal in Odom Frame: X = {odom_x}, Y = {odom_y}")
    return odom_x, odom_y







goal_reached = False

def status_callback(msg):
    """ Callback for move_base status updates """
    global goal_reached
    for status in msg.status_list:
        if status.status == 3:  # Status 3 means the goal was reached successfully
            rospy.loginfo("Goal reached!")
            goal_reached = True

def perform_navigation_task(lat_lon_tuple):
    global goal_reached
    goal_reached = False  # Reset goal status before each task

    # Start the gmapping_navigation.launch process
    rospy.loginfo("Launching gmapping_navigation.launch...")

    # Find the path to the `husky_navigation` package using `rospack`
    husky_navigation_path = subprocess.check_output(["rospack", "find", "husky_navigation"]).decode("utf-8").strip()
    print(f"Husky Navigation Path: {husky_navigation_path}")

    # Construct the path to the workspace's setup.bash file (adjusted to user's workspace path)
    workspace_setup_path = os.path.expanduser("~/catkin_ws/devel/setup.bash")

    # Run source and roslaunch command sequentially in one subprocess
    gmapping_process = subprocess.Popen(
        f"source {workspace_setup_path} && roslaunch husky_navigation gmapping_navigation.launch",
        shell=True, executable="/bin/bash"
    )

    try:
        
        lat, lon = lat_lon_tuple
        
        rospy.loginfo(f"Received GPS coordinates: Latitude = {lat}, Longitude = {lon}")

        
        utm_initial_x, utm_initial_y = gps_to_cartesian(38.041764, -75.373154)
        
        
        
        # Create transformation matrix
        o_T_u = np.array([[1, 0, -utm_initial_x], [0, 1, -utm_initial_y], [0, 0, 1]])

        

        # Convert the target GPS to UTM
        utm_goal_x, utm_goal_y = gps_to_cartesian(lat, lon)

        

        # Calculate relative position in odom frame
        odom_goal_x, odom_goal_y = calculate_relative_position(utm_goal_x, utm_goal_y, o_T_u)
        
        
        # Send navigation goal to the robot
        send_navigation_goal(odom_goal_x, odom_goal_y)

        # Subscribe to move_base status to track goal completion
        status_subscriber = rospy.Subscriber("/move_base/status", GoalStatusArray, status_callback)

        # Wait until the goal is reached
        rate = rospy.Rate(1)  # Check every 1 second
        while not rospy.is_shutdown():
            if goal_reached:
                rospy.loginfo("Navigation task completed.")
                break
            rate.sleep()

        # Unsubscribe after the goal is reached
        status_subscriber.unregister()

        # Terminate gmapping after task is done
        rospy.loginfo("Terminating gmapping_navigation.launch...")
        gmapping_process.terminate()

        return True

    except Exception as e:
        rospy.logerr(f"Error occurred during navigation: {e}")
        gmapping_process.terminate()

        return False
    



