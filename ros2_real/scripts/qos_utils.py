import yaml
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

def load_qos_profile(topic_name):
    qos_file = os.path.expanduser('~/ros2_ws/src/my_tb3_sim/config/qos_profiles.yaml')
    try:
        with open(qos_file, 'r') as f:
            qos_data = yaml.safe_load(f)
        profile = qos_data.get(topic_name, {})
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if profile.get('reliability') == 'reliable' else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if profile.get('durability') == 'transient_local' else DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=int(profile.get('depth', 1))
        )
    except Exception as e:
        print(f"[WARN] Failed to load QoS for '{topic_name}', using default. {e}")
        return QoSProfile(depth=1)
