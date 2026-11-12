import os

MIR_IP = "192.168.12.20"
ROS_MASTER_URI = f"http://{MIR_IP}:11311"

def configure_ros():
    os.environ["ROS_MASTER_URI"] = ROS_MASTER_URI
    os.environ["ROS_IP"] = "192.168.12.131"
    os.environ.pop("ROS_HOSTNAME", None)