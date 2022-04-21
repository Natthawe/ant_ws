import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_tf2_broadcaster')
        self._tf_publisher = StaticTransformBroadcaster(self)
        self._tf_publisher.sendTransform(self.make_transforms())

    def make_transforms(self):
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = self.get_clock().now().to_msg()
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.translation.x = 0.0
        map_to_odom.transform.translation.y = 0.0
        map_to_odom.transform.translation.z = 0.0
        map_to_odom.transform.rotation.x = 0.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0

        odom_to_base_footprint = TransformStamped()
        odom_to_base_footprint.header.stamp = self.get_clock().now().to_msg()
        odom_to_base_footprint.header.frame_id = 'odom'
        odom_to_base_footprint.child_frame_id = 'base_footprint'
        odom_to_base_footprint.transform.translation.x = 0.0
        odom_to_base_footprint.transform.translation.y = 0.0
        odom_to_base_footprint.transform.translation.z = 0.0
        odom_to_base_footprint.transform.rotation.x = 0.0
        odom_to_base_footprint.transform.rotation.y = 0.0
        odom_to_base_footprint.transform.rotation.z = 0.0
        odom_to_base_footprint.transform.rotation.w = 1.0

        base_footprint_to_base_link = TransformStamped()
        base_footprint_to_base_link.header.stamp = self.get_clock().now().to_msg()
        base_footprint_to_base_link.header.frame_id = 'base_footprint'
        base_footprint_to_base_link.child_frame_id = 'base_link'
        base_footprint_to_base_link.transform.translation.x = 0.0
        base_footprint_to_base_link.transform.translation.y = 0.0
        base_footprint_to_base_link.transform.translation.z = 0.095
        base_footprint_to_base_link.transform.rotation.x = 0.0
        base_footprint_to_base_link.transform.rotation.y = 0.0
        base_footprint_to_base_link.transform.rotation.z = 0.0
        base_footprint_to_base_link.transform.rotation.w = 1.0

        base_link_to_laser = TransformStamped()
        base_link_to_laser.header.stamp = self.get_clock().now().to_msg()
        base_link_to_laser.header.frame_id = 'base_link'
        base_link_to_laser.child_frame_id = 'laser'
        base_link_to_laser.transform.translation.x = 0.22
        base_link_to_laser.transform.translation.y = 0.0
        base_link_to_laser.transform.translation.z = 0.23
        base_link_to_laser.transform.rotation.x = 0.0
        base_link_to_laser.transform.rotation.y = 0.0
        base_link_to_laser.transform.rotation.z = 0.0
        base_link_to_laser.transform.rotation.w = 1.0

        base_link_to_bno055 = TransformStamped()
        base_link_to_bno055.header.stamp = self.get_clock().now().to_msg()
        base_link_to_bno055.header.frame_id = 'base_link'
        base_link_to_bno055.child_frame_id = 'bno055'
        base_link_to_bno055.transform.translation.x = -0.09
        base_link_to_bno055.transform.translation.y = -0.20
        base_link_to_bno055.transform.translation.z = 0.20
        base_link_to_bno055.transform.rotation.x = 0.0
        base_link_to_bno055.transform.rotation.y = 0.0
        base_link_to_bno055.transform.rotation.z = 0.0
        base_link_to_bno055.transform.rotation.w = 1.0

        return(
            map_to_odom,
            odom_to_base_footprint,
            base_footprint_to_base_link,
            base_link_to_laser,
            base_link_to_bno055,
        )

def main():
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()            
