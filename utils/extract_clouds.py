import numpy as np, base64, json, math
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

BAG = "/home/pan-navigator/calibration_20260731_110805/calibration_20260731_110805_0.mcap"
N_TARGET = 40000
SEED = 42

TOPIC_TO_SENSOR = {
    "/lidar/front/rslidar_points": "front",
    "/lidar/back/rslidar_points": "back",
    "/lidar/left/rslidar_points": "left",
    "/lidar/right/rslidar_points": "right",
    "/lidar/front/top/rslidar_points": "fronttop",
    "/lidar/back/top/rslidar_points": "backtop",
}

def quat_to_rpy(x, y, z, w):
    sinr_cosp = 2*(w*x + y*z); cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = math.asin(sinp)
    siny_cosp = 2*(w*z + x*y); cosy_cosp = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def extract_xyz(ros_msg):
    step = ros_msg.point_step
    n = ros_msg.width * ros_msg.height
    buf = np.frombuffer(bytes(ros_msg.data), dtype=np.uint8)
    buf = buf[: n * step].reshape(n, step)
    x = buf[:, 0:4].copy().view("<f4").reshape(n)
    y = buf[:, 4:8].copy().view("<f4").reshape(n)
    z = buf[:, 8:12].copy().view("<f4").reshape(n)
    return x, y, z

rng = np.random.default_rng(SEED)

seeds = {}
with open(BAG, "rb") as f:
    reader = make_reader(f, decoder_factories=[DecoderFactory()])
    for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=["/tf_static"]):
        for tr in ros_msg.transforms:
            t, q = tr.transform.translation, tr.transform.rotation
            r, p, y = quat_to_rpy(q.x, q.y, q.z, q.w)
            seeds[tr.child_frame_id] = {"xyz": [t.x, t.y, t.z], "rpy": [r, p, y]}

frame_to_sensor = {
    "rslidarfront": "front", "rslidarback": "back", "rslidarleft": "left",
    "rslidarright": "right", "rslidarfronttop": "fronttop", "rslidarbacktop": "backtop",
}
seed_by_sensor = {frame_to_sensor[k]: v for k, v in seeds.items() if k in frame_to_sensor}

sensors_out = {}
counts_per_topic = {t: 0 for t in TOPIC_TO_SENSOR}
picked = {}
with open(BAG, "rb") as f:
    reader = make_reader(f, decoder_factories=[DecoderFactory()])
    for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=list(TOPIC_TO_SENSOR)):
        topic = channel.topic
        counts_per_topic[topic] += 1
        if counts_per_topic[topic] == 35:  # a representative mid-sequence frame
            picked[topic] = ros_msg

for topic, sensor in TOPIC_TO_SENSOR.items():
    ros_msg = picked.get(topic)
    if ros_msg is None:
        raise RuntimeError(f"no message picked for {topic}, only {counts_per_topic[topic]} messages seen")
    x, y, z = extract_xyz(ros_msg)
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    nonzero = ~((x == 0) & (y == 0) & (z == 0))
    mask = finite & nonzero
    x, y, z = x[mask], y[mask], z[mask]
    total = x.shape[0]
    if total > N_TARGET:
        idx = rng.choice(total, size=N_TARGET, replace=False)
        x, y, z = x[idx], y[idx], z[idx]
    n = x.shape[0]
    interleaved = np.empty(n * 3, dtype=np.float64)
    interleaved[0::3] = x; interleaved[1::3] = y; interleaved[2::3] = z
    q = np.round(interleaved * 100.0)
    q = np.clip(q, -32768, 32767).astype("<i2")
    b64 = base64.b64encode(q.tobytes()).decode("ascii")
    sensors_out[sensor] = {"n": n, "b64": b64}
    print(sensor, "raw_valid:", total, "encoded:", n, "bytes_b64:", len(b64))

out = {"seeds": seed_by_sensor, "sensors": sensors_out}
with open("pcd_data_p03v2.json", "w") as f:
    json.dump(out, f)
print("wrote pcd_data_p03v2.json", sum(len(v['b64']) for v in sensors_out.values()), "b64 chars total")
