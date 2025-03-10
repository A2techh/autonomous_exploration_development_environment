#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion
import math

pseq = 0
mseq = 0
pointCounter = 0
max_points = 5
markers = []

def way_point_callback(msg):
    global pointCounter

    if pointCounter < max_points:
        # Callback function for the /way_point topic
        transform = get_transform("map", "sensor")

        #print("transform", transform)
        if transform is not None:
            # Publish marker and point
            rot = transform.transform.rotation
            euler = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            z = euler[2]
            #print("z", z)
            publish_marker(msg.point, transform.transform.translation, z)
            publish_point(msg.point, transform.transform.translation, z)
            pointCounter += 1

def publish_marker(point, translation, z):
    # Create a visualization marker
    global mseq 
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.header.seq = mseq
    marker.id = pointCounter
    mseq = mseq + 1
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = (point.x*math.cos(z)-point.y*math.sin(z)) + translation.x
    marker.pose.position.y = (point.x*math.sin(z)+point.y*math.cos(z)) + translation.y
    marker.pose.position.z = point.z
    marker.scale.x = 0.4
    marker.scale.y = 0.4
    marker.scale.z = 0.8
    marker.color.a = 1.0  # Fully opaque
    marker.color.r = 1.0  # Red
    marker.color.g = 0.0  # Green
    marker.color.b = 0.0  # Blue

    # Publish the marker to a new topic, e.g., /visualization_marker
    #marker_pub.publish(marker)
    markers.append(marker)  # Add the sphere marker to the list

    # Create a visualization marker for the text
    text_marker = Marker()
    text_marker.header.frame_id = "map"
    text_marker.header.stamp = rospy.Time.now()
    text_marker.header.seq = mseq
    text_marker.id = mseq + 100
    mseq += 1
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    text_marker.pose.position.x = marker.pose.position.x
    text_marker.pose.position.y = marker.pose.position.y
    text_marker.pose.position.z = marker.pose.position.z + 0.5  # Adjust the z position for text
    text_marker.text = str(pointCounter)  # Display pointCounter as text
    text_marker.scale.z = 0.5
    text_marker.color.a = 1.0  # Fully opaque
    text_marker.color.r = 1.0  # Red
    text_marker.color.g = 1.0  # Green
    text_marker.color.b = 1.0  # Blue

    markers.append(text_marker)  # Add the text marker to the list

    # Create and publish a MarkerArray with all markers
    marker_array = MarkerArray(markers=markers)
    marker_pub.publish(marker_array)

def publish_point(point, translation, z):
    global pseq
    # Create a visualization marker
    pointS = PointStamped()
    pointS.header.frame_id = "map"
    pointS.header.seq = pseq
    pseq = pseq + 1
    pointS.header.stamp = rospy.Time.now()
    pointS.point.x = (point.x*math.cos(z)-point.y*math.sin(z)) + translation.x
    pointS.point.y = (point.x*math.sin(z)+point.y*math.cos(z)) + translation.y
    pointS.point.z = point.z
    point_pub.publish(pointS)

def get_transform(target_frame, source_frame):
    # Get the transform between the target and source frames
    try:
        transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
        return transform
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #rospy.logwarn(f"Failed to get transform: {e}")
        return None

def waypoints_end_callback(msg):
    global pointCounter, markers
    if msg.data:
        print("remove all markers")
        pointCounter = 0
        markers = []  # Clear the list of markers when waypoints end
        markerDeletes = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.action = marker.DELETEALL
        markerDeletes.markers.append(marker)
        marker_pub.publish(markerDeletes)

def main():
    rospy.init_node('way_point_marker_publisher', anonymous=True)

    # Initialize tf2 listener
    global tf_buffer
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)


    # Subscribe to the /way_point topic
    rospy.Subscriber('/way_point_fox', PointStamped, way_point_callback)
    rospy.Subscriber('/waypoints_end', Bool, waypoints_end_callback)

    # Spin to keep the script alive
    rospy.spin()

if __name__ == '__main__':
    marker_pub = rospy.Publisher('/way_point_show', MarkerArray, queue_size=10)
    point_pub = rospy.Publisher('/way_point', PointStamped, queue_size=10)
    main()

