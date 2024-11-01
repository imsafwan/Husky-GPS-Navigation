#!/usr/bin/env python3

import rospy
from husky_gps_navigation_task import perform_navigation_task  # Import the function from your original script
from std_msgs.msg import Float64MultiArray  # Message type for receiving latitude and longitude

# Initialize the variable to store the target coordinates
current_target = None

def target_callback(msg):
    global current_target
    # Extract latitude and longitude from the message
    current_target = (msg.data[0], msg.data[1])
    rospy.loginfo(f"Received new target GPS coordinates: {current_target}")

def move_to_target():
    global current_target
    rospy.init_node("target_navigation_node")

    # Set up the subscriber to get target coordinates
    rospy.Subscriber("/target_gps_coordinates", Float64MultiArray, target_callback)
    
    rate = rospy.Rate(1)  # Check for new targets at 1 Hz

    while not rospy.is_shutdown():
        # Check if there is a current target available
        if current_target is not None:
            rospy.loginfo(f"Moving to target GPS coordinates: {current_target}")
            
            # Move to the target
            success = perform_navigation_task(current_target)
            
            if success:
                rospy.loginfo(f"Reached target {current_target} successfully. You can set a new target by clicking on the map.")
            else:
                rospy.logwarn(f"Failed to reach target {current_target}. Awaiting new target.")
            
            # Clear current target after attempt
            current_target = None  # Reset to None after attempting to reach target
        else:
            rospy.loginfo("No target set. You can set a new target by clicking on the map.")

        rate.sleep()  # Wait before checking for new targets

if __name__ == "__main__":
    try:
        move_to_target()
    except rospy.ROSInterruptException:
        rospy.loginfo("Target navigation interrupted.")
