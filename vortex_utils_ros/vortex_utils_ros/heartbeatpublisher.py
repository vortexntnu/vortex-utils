from rclpy.node import Node
from std_msgs.msg import String


def add_heartbeat_publisher(node: Node, node_name: str):
    publisher = node.create_publisher(String, "heartbeat", 10)

    def publish_heartbeat():
        msg = String()
        msg.data = node_name
        publisher.publish(msg)

    node.create_timer(1.0, publish_heartbeat)
