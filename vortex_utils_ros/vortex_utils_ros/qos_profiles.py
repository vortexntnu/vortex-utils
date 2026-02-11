from rclpy import qos


def sensor_data_profile(depth: int = 5) -> qos.QoSProfile:
    return qos.QoSProfile(
        history=qos.HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=qos.ReliabilityPolicy.BEST_EFFORT,
    )


def reliable_profile(depth: int = 10) -> qos.QoSProfile:
    return qos.QoSProfile(
        history=qos.HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=qos.ReliabilityPolicy.RELIABLE,
    )


def reliable_transient_local_profile(depth: int = 1) -> qos.QoSProfile:
    return qos.QoSProfile(
        history=qos.HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=qos.ReliabilityPolicy.RELIABLE,
        durability=qos.DurabilityPolicy.TRANSIENT_LOCAL,
    )
