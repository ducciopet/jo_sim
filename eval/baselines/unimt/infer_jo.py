#! /usr/bin/env python3
# UniMT (CMDT detector) + SimpleTrack on the JO robot. Copy of upstream perception_test/scripts/infer2.py
# (github.com/TossherO/ros_packages): detection model, config, pipeline, score threshold, tracker and its config are
# unchanged. Only the I/O is adapted, marked # JO:
#  - topics of the JO bags; image converted without cv_bridge (not built for python3.10)
#  - camera intrinsics / LiDAR->camera extrinsics of the JO robot (URDF) instead of the authors' wheelchair
#  - ego2global = pose of the velodyne in odom (odometry of base_link composed with base_link->velodyne)
#  - trajectory prediction skipped (not evaluated); output = MarkerArray LINE_LIST per alive track, marker id =
#    track id, frame odom, stamp = cloud stamp (read by eval/adapters/marker_adapter.py)
# Run from the workspace root (/ws) as upstream: the config imports src.detection.scripts.cmdt.

import rospy
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import sys
import os.path as osp
sys.path.append(osp.abspath('./'))
sys.path.append(osp.abspath('./src/tracking/scripts'))
import yaml
from pyquaternion import Quaternion
import numpy as np
import torch
from mmengine.config import Config
from mmdet3d.registry import MODELS, TRANSFORMS
from mmdet3d.utils import register_all_modules
from mmengine.runner import load_checkpoint
from mmdet3d.structures import LiDARPoints
from mmdet3d.structures import LiDARInstance3DBoxes
from mmdet3d.structures import Box3DMode
from mot_3d.mot import MOTModel
from mot_3d.data_protos import BBox
from mot_3d.frame_data import FrameData

# JO: color camera of the bag (1280x720, from camera_info); the pipeline's ResizeCropFlipImage scales it to the
# model's 480x640 and updates cam2img itself
CAM2IMG = np.array([[[644.308, 0., 650.24, 0.],
                     [0., 643.637, 374.757, 0.],
                     [0., 0., 1., 0.],
                     [0., 0., 0., 1.]]])
# JO: URDF, base_link -> front_camera_color_optical_frame and base_link -> velodyne
BODY2CAM = np.array([[0., 0., 1., 0.336], [-1., 0., 0., -0.045], [0., -1., 0., 0.278], [0., 0., 0., 1.]])
BODY2LIDAR = np.array([[1., 0., 0., 0.03], [0., 1., 0., 0.029], [0., 0., 1., 0.6], [0., 0., 0., 1.]])
LIDAR2CAM = (np.linalg.inv(BODY2CAM) @ BODY2LIDAR)[None]
EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def cloud_xyzi(msg):  # JO: numpy read instead of pc2.read_points (same fields, NaNs skipped)
    f = {fl.name: fl.offset for fl in msg.fields}
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    cols = [buf[:, f[k]:f[k] + 4].copy().view(np.float32)[:, 0] for k in ('x', 'y', 'z', 'intensity')]
    p = np.stack(cols, axis=1)
    return p[np.isfinite(p).all(axis=1)]


def point_cloud_callback(data):

    global data_dict
    if data_dict['count'] % 2 == 0:
        data_dict['point_clouds'] = cloud_xyzi(data)
        data_dict['pc_timestamp'] = data.header.stamp
        data_dict['count'] += 1
        if data_dict['images'] is not None and data_dict['ego2global'] is not None:
            data_dict['ready'] = True
    else:
        data_dict['count'] += 1


def image_callback(data):

    global data_dict
    if data_dict['ready'] is False:
        # JO: rgb8/bgr8 -> bgr8 numpy, as bridge.imgmsg_to_cv2(desired_encoding='bgr8')
        image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, 3)
        if data.encoding == 'rgb8':
            image = image[:, :, ::-1]
        data_dict['images'] = [np.ascontiguousarray(image)]
        data_dict['img_timestamp'] = data.header.stamp
        if data_dict['point_clouds'] is not None and data_dict['ego2global'] is not None:
            data_dict['ready'] = True


def pose_callback(data):

    global data_dict
    if data_dict['ready'] is False:
        pose = data.pose.pose
        x, y, z, qx, qy, qz, qw = pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        ego2global = np.eye(4)
        ego2global[:3, :3] = Quaternion(qw, qx, qy, qz).rotation_matrix
        ego2global[:3, 3] = np.array([x, y, z])
        data_dict['ego2global'] = ego2global @ BODY2LIDAR  # JO: detections are in the velodyne frame
        if data_dict['point_clouds'] is not None and data_dict['images'] is not None:
            data_dict['ready'] = True


def box_marker(bbox, tid, label, stamp):  # JO: output for the evaluation
    x, y, z, l, w, h, o = bbox[:7]
    c, s = np.cos(o), np.sin(o)
    corners = []
    for zz in (z, z + h):  # tracker boxes: z is the bottom (mmdet3d LiDAR boxes)
        for dx, dy in ((l / 2, w / 2), (-l / 2, w / 2), (-l / 2, -w / 2), (l / 2, -w / 2)):
            corners.append(Point(x + c * dx - s * dy, y + s * dx + c * dy, zz))
    m = Marker()
    m.header.frame_id = 'odom'
    m.header.stamp = stamp
    m.ns = 'unimt_' + label
    m.id = int(tid)
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.05
    m.color.r, m.color.a = 1.0, 1.0
    m.points = [corners[k] for e in EDGES for k in e]
    return m


if __name__ == '__main__':

    rospy.init_node('perception_infer', anonymous=True)
    data_dict = {  # JO: defined before the subscribers (upstream defines it after them)
        'point_clouds': None,
        'images': None,
        'ego2global': None,
        'pc_timestamp': None,
        'img_timestamp': None,
        'count': 0,
        'ready': False
    }
    rospy.Subscriber("/velodyne_points", PointCloud2, point_cloud_callback, queue_size=1, buff_size=2**24)  # JO (buff_size: rospy drops large msgs with queue_size=1 otherwise)
    rospy.Subscriber("/front_camera/camera/color/image_raw", Image, image_callback, queue_size=1, buff_size=2**24)  # JO
    rospy.Subscriber("/odometry/filtered", Odometry, pose_callback, queue_size=1)  # JO
    pub = rospy.Publisher('/unimt/tracks', MarkerArray, queue_size=10)  # JO
    pub_det = rospy.Publisher('/unimt/detections', MarkerArray, queue_size=10)  # JO: CMDT detections before tracking (EagerMOT input)

    # load detection model
    cfg = Config.fromfile('./src/detection/scripts/configs/cmdt_wheelchair.py')
    checkpoint = './src/detection/ckpts/cmdt_wheelchair.pth'
    register_all_modules()
    detect_model = MODELS.build(cfg.model)
    pipeline = []
    for transform in cfg.test_dataloader.dataset.pipeline:
        pipeline.append(TRANSFORMS.build(transform))
    checkpoint = load_checkpoint(detect_model, checkpoint, map_location='cpu')
    detect_model.cuda().eval()

    # load tracking model
    config_path = './src/tracking/scripts/configs/coda_configs/diou.yaml'
    track_config = yaml.load(open(config_path, 'r'), Loader=yaml.Loader)
    class_labels = [0, 1]
    class_names = ['pedestrian', 'cyclist']
    trackers = [MOTModel(track_config, class_names[label]) for label in class_labels]


    print("finish init infer node", flush=True)  # JO: marker the run script waits for

    while not rospy.is_shutdown():

        if data_dict['ready'] is True:
            timestamp = data_dict['pc_timestamp'].to_sec()
            ego2global = data_dict['ego2global']
            time = rospy.get_time()

            input = {}
            input['points'] = LiDARPoints(data_dict['point_clouds'], points_dim=4)
            input['img'] = data_dict['images']
            input['timestamp'] = timestamp
            input['img_shape'] = (480, 640)
            input['ori_shape'] = (480, 640)
            input['pad_shape'] = (480, 640)
            input['cam2img'] = CAM2IMG.copy()  # JO
            input['lidar2cam'] = LIDAR2CAM.copy()  # JO
            input['box_type_3d'] = LiDARInstance3DBoxes
            input['box_mode_3d'] = Box3DMode.LIDAR

            # detection
            with torch.no_grad():
                for i in range(2, len(pipeline)):
                    input = pipeline[i](input)
                input['data_samples'] = [input['data_samples']]
                input['inputs']['points'] = [input['inputs']['points']]
                input['inputs']['img'] = [input['inputs']['img']]
                output = detect_model.data_preprocessor(input, training=False)
                output = detect_model(**output, mode='predict')
                bboxes_3d = output[0].get('pred_instances_3d')['bboxes_3d']
                labels_3d = output[0].get('pred_instances_3d')['labels_3d']
                scores_3d = output[0].get('pred_instances_3d')['scores_3d']
                bboxes_3d = bboxes_3d[scores_3d > 0.3].tensor.cpu().numpy()
                labels_3d = labels_3d[scores_3d > 0.3].cpu().numpy()
                scores_3d = scores_3d[scores_3d > 0.3].cpu().numpy()

            # tracking
            ma = MarkerArray()
            ma_det = MarkerArray()  # JO
            for i, label in enumerate(class_labels):
                mask = labels_3d == label
                dets = np.concatenate([bboxes_3d[mask], scores_3d[mask][:, None]], axis=1).tolist()
                frame_data = FrameData(dets=dets, ego=ego2global, pc=None, det_types=labels_3d[mask], time_stamp=float(timestamp))
                frame_data.dets = [BBox.bbox2world(ego2global, det) for det in frame_data.dets]
                for j, (d, sc) in enumerate(zip(frame_data.dets, scores_3d[mask])):  # JO: world-frame detections, score in color.g
                    md = box_marker(BBox.bbox2array(d), len(ma_det.markers), 'det_' + class_names[label], data_dict['pc_timestamp'])
                    md.color.g = float(sc)
                    ma_det.markers.append(md)
                results = trackers[i].frame_mot(frame_data)
                for trk in results:
                    if trk[2].split('_')[0] == 'alive':  # as upstream: only alive tracks are published
                        # JO: ids are per-class in SimpleTrack; offset the cyclist ones to keep them distinct
                        ma.markers.append(box_marker(BBox.bbox2array(trk[0]), trk[1] + 100000 * i, class_names[label],
                                                     data_dict['pc_timestamp']))
            if not ma.markers:  # JO: an empty frame still marks that the method ran
                m = Marker(); m.header.frame_id = 'odom'; m.header.stamp = data_dict['pc_timestamp']
                m.action = Marker.DELETEALL; ma.markers.append(m)
            pub.publish(ma)
            if not ma_det.markers:  # JO
                m = Marker(); m.header.frame_id = 'odom'; m.header.stamp = data_dict['pc_timestamp']
                m.action = Marker.DELETEALL; ma_det.markers.append(m)
            pub_det.publish(ma_det)  # JO

            print("infer time: %.3f n=%d" % (rospy.get_time() - time, len(ma.markers)), flush=True)
            data_dict['ready'] = False
            data_dict['point_clouds'] = None
            data_dict['images'] = None
            data_dict['ego2global'] = None

        rospy.sleep(0.05)
