#!/usr/bin/env python3

import rospy
from husky_gps_navigation_task import perform_navigation_task  # Import the function from your original script

def move_to_target():
    rospy.init_node("target_navigation_node")
    
    # Define your target GPS coordinates (latitude, longitude)
    target_coordinates = [
        (38.041388, -75.375071),  # Example target 1
    ]
    
    # Move to each target sequentially
    for target in target_coordinates:
        rospy.loginfo(f"Moving to target GPS coordinates: {target}")
        success = perform_navigation_task(target)
        
        if success:
            rospy.loginfo(f"Reached target {target} successfully.")
        else:
            rospy.logwarn(f"Failed to reach target {target}. Moving to the next one.")
        
        # Add a delay between targets if needed
        rospy.sleep(5)
    
    rospy.loginfo("Navigation to all targets completed.")

if __name__ == "__main__":
    try:
        move_to_target()
    except rospy.ROSInterruptException:
        rospy.loginfo("Target navigation interrupted.")