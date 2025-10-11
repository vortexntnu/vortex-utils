from rclpy import qos


def sensor_data_profile(depth: int = 5) -> qos.QoSProfile:
    return qos.QoSProfile(
        history=qos.HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=qos.ReliabilityPolicy.BEST_EFFORT,
    )


def reliable_profile(depth: int = 10) -> qos.QoSProfile:
    return qos.QoSProfile(history=qos.HistoryPolicy.KEEP_LAST, depth=depth)
