import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def get_transform(target_frame, source_frame):
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))
        return transform
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"Failed to lookup transform from {source_frame} to {target_frame}: {e}")
        return None

def publish_path_from_bag(bag_file, topic_name, target_frame='map'):
    # Initialize publishers, messages, etc.

    # Get transform from 'odom' to 'map'
    transform = get_transform(target_frame, 'odom')
    if transform is None:
        rospy.logerr("Transform from 'odom' to 'map' not available.")
        return

    # Use the transform to adjust pose before publishing
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/mobile_base_controller/odom':
                # Transform pose from 'odom' to 'map'
                transformed_pose = tf2_geometry_msgs.do_transform_pose(msg.pose.pose, transform)
                # Construct and populate path_msg with transformed poses
                # Publish the path message

if __name__ == '__main__':
    rospy.init_node("publish_path", anonymous=True)
    bag_file = "trajectory5.bag"
    topic = "/path5"
    publish_path_from_bag(bag_file, topic)
