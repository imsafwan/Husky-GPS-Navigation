#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import Float64MultiArray  # Message type for sending latitude and longitude
from sensor_msgs.msg import NavSatFix  # Message type for receiving GPS position of Husky
import matplotlib.pyplot as plt

# Initialize ROS node
rospy.init_node("map_click_publisher")

# Create a publisher for the target GPS coordinates
gps_pub = rospy.Publisher("/target_gps_coordinates", Float64MultiArray, queue_size=10)

# Global variable to store the current Husky position
husky_position = None

# Get the directory of the current script
script_dir = os.path.dirname(os.path.abspath(__file__))

# Define the image path relative to the script's directory
image_path = os.path.join(script_dir, "map.png")

# Read the map image
map_image = plt.imread(image_path)

# Define the GPS bounds of the map image
map_bounds = {
    "top_left": (38.042175, -75.373660),       # (latitude, longitude) of top-left corner
    "bottom_right": (38.041331, -75.372218)    # (latitude, longitude) of bottom-right corner
}

# Display the map image
fig, ax = plt.subplots()
ax.imshow(map_image)
plt.title("Click on the map to set a target GPS coordinate")

# Calculate pixel-to-GPS scale factors
img_height, img_width = map_image.shape[0], map_image.shape[1]
lat_range = map_bounds["top_left"][0] - map_bounds["bottom_right"][0]
lon_range = map_bounds["bottom_right"][1] - map_bounds["top_left"][1]
lat_per_pixel = lat_range / img_height
lon_per_pixel = lon_range / img_width

# Initialize the Husky and target markers
husky_marker, = ax.plot([], [], marker="s", markersize=8, markerfacecolor="yellow", markeredgecolor="black", markeredgewidth=2, label="Husky Position")  # Blue square for Husky
target_marker, = ax.plot([], [], marker="o", markersize=10, color="red", markerfacecolor="none", label="Target Position")  # Red circle outline for target

# Update Husky position on the map
def update_husky_marker():
    if husky_position is not None:
        lat, lon = husky_position
        # Calculate pixel position for the Husky's GPS coordinates
        x_pixel = (lon - map_bounds["top_left"][1]) / lon_per_pixel
        y_pixel = (map_bounds["top_left"][0] - lat) / lat_per_pixel
        # Update the marker position
        husky_marker.set_data(x_pixel, y_pixel)

# Callback function to update the Husky's position
def husky_gps_callback(msg):
    global husky_position
    husky_position = (msg.latitude, msg.longitude)

# Subscriber to receive Husky's GPS coordinates
rospy.Subscriber("/navsat/fix", NavSatFix, husky_gps_callback)

# Mouse click event function to get coordinates and set target marker
def on_click(event):
    if event.xdata is not None and event.ydata is not None:
        # Convert pixel coordinates to GPS coordinates
        lat = map_bounds["top_left"][0] - event.ydata * lat_per_pixel
        lon = map_bounds["top_left"][1] + event.xdata * lon_per_pixel
        print(f"Clicked GPS Coordinates: Latitude = {lat}, Longitude = {lon}")

        # Publish GPS coordinates as a Float64MultiArray
        gps_msg = Float64MultiArray(data=[lat, lon])
        gps_pub.publish(gps_msg)
        print("Published target GPS coordinates.")

        # Update target marker position
        target_marker.set_data(event.xdata, event.ydata)

# Connect the click event
fig.canvas.mpl_connect("button_press_event", on_click)

# Continuous plot updates
while not rospy.is_shutdown():
    update_husky_marker()  # Update the Husky marker position
    plt.legend()
    plt.pause(0.1)  # Allow for non-blocking updates

plt.show()