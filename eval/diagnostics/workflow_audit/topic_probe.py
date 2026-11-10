"""Topic audit during a SINS run: subscribes to EVERY topic (raw CDR, no deserialization), logs per message the
receive time, header stamp, frame_id and element count (PointCloud2 width*height, Detection arrays / MarkerArray /
TFMessage sequence length), and every 5 s the graph (publishers / subscribers per topic, node names).
usage (container): python3 topic_probe.py <out_prefix> [duration_s]"""
import sys, time, struct, json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message

SEQ_FIRST = {'visualization_msgs/msg/MarkerArray', 'tf2_msgs/msg/TFMessage'}
HDR_SEQ = {'vision_msgs/msg/Detection2DArray', 'vision_msgs/msg/Detection3DArray'}


def parse(tp, b):
    """(stamp, frame_id, n) from raw CDR (little-endian, 4-byte encapsulation header)."""
    try:
        if tp in SEQ_FIRST:
            return None, '', struct.unpack_from('<I', b, 4)[0]
        sec, nsec, ln = struct.unpack_from('<iII', b, 4)
        fid = b[16:16 + ln - 1].decode(errors='replace')
        off = 16 + ln
        off = (off - 4 + 3) // 4 * 4 + 4          # align 4 (CDR alignment counted after the encapsulation)
        n = None
        if tp == 'sensor_msgs/msg/PointCloud2':
            h, w = struct.unpack_from('<II', b, off); n = h * w
        elif tp in HDR_SEQ:
            n = struct.unpack_from('<I', b, off)[0]
        elif tp in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'):
            n = len(b)
        return sec + nsec * 1e-9, fid, n
    except Exception:
        return None, '?', None


class Probe(Node):
    def __init__(self, out, dur):
        super().__init__('topic_probe')
        self.out = open(out + '_msgs.csv', 'w'); self.out.write('topic,type,recv,stamp,frame,n,bytes\n')
        self.graph = open(out + '_graph.jsonl', 'w')
        self.subs = {}; self.t_end = time.time() + dur
        self.create_timer(1.0, self.discover); self.create_timer(5.0, self.dump_graph)

    def discover(self):
        for name, types in self.get_topic_names_and_types():
            if name in self.subs or name.startswith('/topic_probe'):
                continue
            tp = types[0]
            try:
                cls = get_message(tp)
            except Exception:
                continue
            pubs = [p for p in self.get_publishers_info_by_topic(name) if p.node_name != 'topic_probe']
            if not pubs:
                continue          # subscribe once a publisher exists (its QoS is known then)
            # BEST_EFFORT matches reliable and best-effort publishers; TRANSIENT_LOCAL only if every publisher is latched
            durable = all(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in pubs)
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL if durable else DurabilityPolicy.VOLATILE)
            self.subs[name] = self.create_subscription(cls, name, lambda m, n=name, t=tp: self.cb(n, t, m), qos, raw=True)
        if time.time() > self.t_end:
            raise SystemExit

    def cb(self, name, tp, raw):
        st, fid, n = parse(tp, raw)
        self.out.write(f'{name},{tp},{time.time():.6f},{"" if st is None else f"{st:.6f}"},{fid},{"" if n is None else n},{len(raw)}\n')

    def dump_graph(self):
        g = {'t': time.time(), 'topics': {}}
        for name, types in self.get_topic_names_and_types():
            g['topics'][name] = {'type': types[0],
                                 'pub': sorted(p.node_name for p in self.get_publishers_info_by_topic(name)),
                                 'sub': sorted(s.node_name for s in self.get_subscriptions_info_by_topic(name))}
        self.graph.write(json.dumps(g) + '\n'); self.graph.flush(); self.out.flush()


def main():
    rclpy.init()
    p = Probe(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 600)
    try:
        rclpy.spin(p)
    except SystemExit:
        pass
    p.out.close(); p.graph.close()


if __name__ == '__main__':
    main()
