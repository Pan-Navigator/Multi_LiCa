from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import math

path = "/home/pan-navigator/calibration_20260731_110805/calibration_20260731_110805_0.mcap"

def quat_to_rpy(x, y, z, w):
    # roll
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2*(w*y - z*x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

with open(path, "rb") as f:
    reader = make_reader(f, decoder_factories=[DecoderFactory()])
    for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=["/tf_static"]):
        for tr in ros_msg.transforms:
            t = tr.transform.translation
            q = tr.transform.rotation
            r,p,y = quat_to_rpy(q.x,q.y,q.z,q.w)
            print(f"{tr.header.frame_id:>12} -> {tr.child_frame_id:<16} xyz=({t.x:.5f},{t.y:.5f},{t.z:.5f}) rpy=({r:.5f},{p:.5f},{y:.5f})")
