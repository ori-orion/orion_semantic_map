import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose

def rois_to_marker_array(rois):
    markers = []
    for roi in rois:
        points = []
        poses = roi.posearray.poses
        total_x = 0.0
        total_y = 0.0
        total_z = 0.0
        for pose in poses:
            total_x = total_x + pose.position.x
            total_y = total_y + pose.position.y
            total_z = total_z + pose.position.z
        avg_x = total_x/len(poses)
        avg_y = total_y/len(poses)
        avg_z = total_z/len(poses)
        poses.append(poses[0])
        for pose in poses:
            points.append(Point(pose.position.x, pose.position.y, pose.position.z))
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "roi"
        marker.id = len(markers)
        marker.type = 4
        marker.scale.x = 0.07
        marker.color.a = 1.0
        marker.color.r = 0.6
        marker.points = points

        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "text_roi"
        text_marker.id = len(markers)
        text_marker.type = 9
        text_marker.scale.z = 0.15
        text_marker.color.a = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.8
        pose = Pose()
        pose.position.x = avg_x
        pose.position.y = avg_y
        pose.position.z = avg_z + 0.1
        text_marker.pose = pose

        text_marker.text = roi.name
        markers.append(text_marker)
        markers.append(marker)
    return markers

def update_objects(object_store):
    pass
