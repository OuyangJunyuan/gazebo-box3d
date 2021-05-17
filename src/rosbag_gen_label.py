#!/home/ou/software/anaconda3/envs/py2/bin/python
import numpy as np
from scipy.spatial.transform import Rotation
from scipy import misc
from scipy.spatial import Delaunay, qhull
import tf
import rospy
import ros_numpy as rnp
import message_filters as mf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from pathlib import Path
import os

"""
odom and lidar msg should be synch carefully
"""
is_large = False
topics = [['/excavator/LiDAR_80_1', PointCloud2],
          ['/excavator/LiDAR_32_1', PointCloud2],
          ['/excavator/LiDAR_16_1', PointCloud2],
          ['/excavator/LiDAR_16_2', PointCloud2],
          ['/excavator/gt_pose/base_link', Odometry]]

ROI_R = 50

ROI = dict({'x': {'l': -ROI_R, 'h': ROI_R},
            'y': {'l': -ROI_R, 'h': ROI_R},
            'z': {'l': -5, 'h': 9}})

cfg = {'truck1': {'pts_in_box': 500.0, 'corner_num': 2.0, 'enlarge': [0.1, 0.1, 0.0],
                  'dxyz2lwh': [0, 2, 1], 'cvt_head': True},
       'truck3': {'pts_in_box': 500.0, 'corner_num': 2.0, 'enlarge': [0.1, 0.1, 0.0]},
       'person': {'pts_in_box': 10.0, 'corner_num': 4.0, 'enlarge': [0.15, 0.15, 0.0]},
       'stone': {'pts_in_box': 1000, 'corner_num': 2.0, 'enlarge': [0.0, 0.0, 0.0]}}

split = 'training'
box_gt_path = "/home/ou/Documents/dump_gt_box/label1.txt"
dataset_dir = "/home/ou/Documents/dataset/my_kitti/object"
kitti_viz_dir = "/home/ou/Documents/dataset/kitti/kitti_object_vis-master"

cnt = [0]
main_lidar = topics[0][0]
lidar = topics[:len(topics) - 1]


def limit_period(val, offset=0.5, period=np.pi):
    ans = val - np.floor(val / period + offset) * period
    return ans


def get_object_from_file(calib_file):
    with open(calib_file) as f:
        lines = f.readlines()
    objs = []
    # example :
    # ['person_standing', 'person',   '0.544', '0.438', '1.934',  '171.069', '-27.765', '0.967', '-0', '0','-0.919']
    for line in lines[1:]:
        obj = line.strip().rsplit()
        cls = obj[1]
        l_idx, w_idx, h_idx = cfg[cls]['dxyz2lwh'] if 'dxyz2lwh' in cfg[cls].keys() else [0, 1, 2]
        dim_l = float(obj[2 + l_idx]) * (1.0 + cfg[cls]['enlarge'][0])
        dim_w = float(obj[2 + w_idx]) * (1.0 + cfg[cls]['enlarge'][1])
        dim_h = float(obj[2 + h_idx]) * (1.0 + cfg[cls]['enlarge'][2])

        loc = np.array([obj[5], obj[6], obj[7]], dtype=np.float32)

        heading = float(obj[10])
        heading = heading - np.pi if 'cvt_head' in cfg[cls].keys() else heading
        # heading = heading + 2 * np.pi if heading < -np.pi else heading
        # heading = heading - 2 * np.pi if heading > np.pi else heading
        heading = limit_period(heading, period=2 * np.pi)

        objs.append({'cls': cls, 'loc': loc, 'heading': heading,
                     'l': dim_l, 'w': dim_w, 'h': dim_h
                     })
    return objs


def cart_to_hom(pts):
    pts_hom = np.hstack((pts, np.ones((pts.shape[0], 1), dtype=np.float32)))
    return pts_hom


def get_transform(r, t):
    _T = np.eye(4)
    _T[:3, :3] = Rotation.from_quat(r).as_dcm()
    _T[:3, 3] = np.array(t).T
    return _T


def pointcloud_transform(pts_lidar, _T):
    pts_lidar_hom = cart_to_hom(pts_lidar)
    pts_rect = np.dot(pts_lidar_hom, _T[:3, :4].T)
    return pts_rect


def world_to_lidar(obj_list, T_world_to_lidar):
    # print Rotation.from_dcm(lidar_pose[:3,:3]).as_euler('xyz')
    new_obj_list = []

    T_wl = T_world_to_lidar
    T_lw = np.linalg.inv(T_wl)

    for obj in obj_list:  # cls loc rpy size
        T_wo = get_transform(Rotation.from_euler('xyz', [0, 0, obj['heading']]).as_quat(), obj['loc'])
        T_lo = np.dot(T_lw, T_wo)
        loc = T_lo[:3, 3]
        heading = Rotation.from_dcm(T_lo[:3, :3]).as_euler('xyz')[2]
        new_obj_list.append({'cls': obj['cls'], 'loc': loc, 'heading': heading,
                             'l': obj['l'], 'w': obj['w'], 'h': obj['h']
                             })
    return new_obj_list


def lidar_to_camera(obj_list):
    T_cl = np.array([0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1]).reshape(-1, 4)
    for obj in obj_list:
        T_lo = get_transform(Rotation.from_euler('xyz', [0, 0, obj['heading']]).as_quat(), obj['loc'])
        T_co = np.dot(T_cl, T_lo)
        loc = T_co[:3, 3]
        obj['loc'] = loc
        obj['heading'] = -(np.pi / 2 + obj['heading'])
        # heading = Rotation.from_dcm(T_lo[:3, :3]).as_euler('xyz')[2]main_lidar


def obj_filter(obj_list, points):
    def in_hull(p, hull):
        """
        :param p: (N, K) test points
        :param hull: (M, K) M corners of a box
        :return (N) bool
        """
        try:
            if not isinstance(hull, Delaunay):
                hull = Delaunay(hull)
            flag = hull.find_simplex(p) >= 0
        except qhull.QhullError:
            print('Warning: not a hull %s' % str(hull))
            flag = np.zeros(p.shape[0], dtype=np.bool)

        return flag

    def get_corner_3d_in_lidar(box):
        template = np.array((
            [1, 1, -1], [1, -1, -1],
            [-1, -1, -1], [-1, 1, -1],
            [1, 1, 1], [1, -1, 1],
            [-1, -1, 1], [-1, 1, 1],
        ), dtype=np.float) / 2.0
        size = np.array([box['l'], box['w'], box['h']])
        T = np.zeros([3, 4])
        T[:3, :3] = Rotation.from_euler('z', box['heading']).as_dcm()
        T[:3, 3] = box['loc']

        return np.dot(np.hstack((template * size, np.ones([8, 1]))), T.T)

    new_obj_list = []
    roi_hull = Delaunay(np.array(
        [[ROI['x']['h'], ROI['y']['h'], ROI['z']['l'] * 2], [ROI['x']['h'], ROI['y']['l'], ROI['z']['l'] * 2],
         [ROI['x']['l'], ROI['y']['l'], ROI['z']['l'] * 2], [ROI['x']['l'], ROI['y']['h'], ROI['z']['l'] * 2],
         [ROI['x']['h'], ROI['y']['h'], ROI['z']['h'] * 2], [ROI['x']['h'], ROI['y']['l'], ROI['z']['h'] * 2],
         [ROI['x']['l'], ROI['y']['l'], ROI['z']['h'] * 2], [ROI['x']['l'], ROI['y']['h'], ROI['z']['h'] * 2]]))

    for this_obj in obj_list:
        cls = this_obj['cls']
        corner3d = get_corner_3d_in_lidar(this_obj)
        corner_sum = in_hull(corner3d, roi_hull).sum()
        points_num = in_hull(points, Delaunay(corner3d)).sum()
        if in_hull(np.array([this_obj['loc']]), roi_hull).sum() == 0:
            continue
        if corner_sum <= cfg[cls]['corner_num'] or points_num <= cfg[cls]['pts_in_box']:
            continue
        new_obj_list.append(this_obj)
    return new_obj_list


# kitti evaluation result - OpenPCDet
def generate_once_kitti_dataset(frame_id, object_list, points, output_dir=None):
    assert output_dir is not None, "output dir is None"
    output_dir = Path(output_dir)

    def creat_file(file_dir, file_path):
        if not Path.exists(file_dir):
            file_dir.mkdir(parents=True)
        path = file_dir / file_path
        return path

    frame_id = "%06d" % frame_id

    label_2_file = creat_file(output_dir / 'label_2', '%s.txt' % frame_id)
    calib_file = creat_file(output_dir / 'calib', '%s.txt' % frame_id)
    image_2_file = creat_file(output_dir / 'image_2', '%s.png' % frame_id)
    lidar_file = creat_file(output_dir / 'velodyne', '%s.bin' % frame_id)

    # gen label {'cls': cls, 'size': size, 'loc': loc, 'rpy': rpy}
    while 1:
        with open(str(label_2_file), 'w') as f:
            for obj in object_list:
                name = obj['cls']
                l, w, h = obj['l'], obj['w'], obj['h']
                loc = obj['loc']
                yaw = obj['heading']
                #           cls t o a   2d(4)         hwl(3)      loc_z0(3) yaw
                label_str = '%s 0 0 0 0 0 1 1 %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n' % \
                            (name,
                             h, w, l,
                             loc[0], loc[1] + h / 2, loc[2], yaw  # box at bottom center so y+h/2
                             )
                f.write(label_str)
            f.flush()
        break

    # gen calib
    while 1:
        with open(str(calib_file), 'w') as f:
            Pi = "P%s: 1 0 0.5 0 0 1 0.5 0 0 0 1 0\n"
            R0 = "R0_rect: 1 0 0 0 1 0 0 0 1\n"
            Tr = "%s: 0 -1 0 0 0 0 -1 0 1 0 0 0\n"
            # Tr = "%s: 1 0 0 0 0 1 0 0 0 0 1 0\n"
            calib_str = "%s%s%s%s%s%s%s" % (Pi % "0", Pi % "1", Pi % "2", Pi % "3", R0, Tr % "Tr_velo_to_cam",
                                            Tr % "Tr_imu_to_velo")
            f.write(calib_str)
            f.flush()
        break

    # gen image
    while 1:
        image_array = np.ones([1, 1])
        misc.imsave(image_2_file, image_array)
        break

    # gen velodyne
    while 1:
        with open(str(lidar_file), 'w') as f:
            bin_cloud = np.zeros([len(points), 4], dtype=np.float32)
            bin_cloud[:, :3] = points
            bin_cloud.reshape(-1, 4).astype(np.float32)
            bin_cloud.tofile(f)
        break


def callback(*args):
    def viz():
        vis = np.zeros(len(all_points), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        vis['x'] = all_points[:, 0]
        vis['y'] = all_points[:, 1]
        vis['z'] = all_points[:, 2]

        vis = rnp.msgify(PointCloud2, vis)
        vis.header.frame_id = "excavator/LiDAR_80_1"
        publisher.publish(vis)

    print "=================================="
    print cnt
    print "args num: ", len(args)
    all_points = np.zeros([0, 0])
    # merge cloud
    for [topic, _], msg in zip(lidar, args):
        if topic == topics[1][0]:
            continue
        points = rnp.point_cloud2.pointcloud2_to_xyz_array(msg)
        try:
            (trans, rot) = listener.lookupTransform(main_lidar, topic, rospy.Time(0))
            T = get_transform(rot, trans)
            all_points = np.concatenate((all_points.reshape(-1, 3), pointcloud_transform(points, T).reshape(-1, 3)),
                                        axis=0)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    trans_bl, rot_bl = listener.lookupTransform("excavator/base_link", main_lidar, rospy.Time(0))

    base_pose = args[len(topics) - 1]
    object_list = args[len(topics)]
    publisher = args[len(topics) + 1]

    # ROI crop
    valida_flag_x = np.logical_and(all_points[:, 0] >= ROI['x']['l'], all_points[:, 0] <= ROI['x']['h'])
    valida_flag_y = np.logical_and(all_points[:, 1] >= ROI['y']['l'], all_points[:, 1] <= ROI['y']['h'])
    valida_flag_z = np.logical_and(all_points[:, 2] >= ROI['z']['l'], all_points[:, 2] <= ROI['z']['h'])
    valida_flag_merge = np.logical_and(np.logical_and(valida_flag_x, valida_flag_y), valida_flag_z)
    all_points = all_points[valida_flag_merge]
    print "ROI points shape: ", all_points.shape

    t_wb = base_pose.pose.pose.position
    r_wb = base_pose.pose.pose.orientation

    T_wb = get_transform([r_wb.x, r_wb.y, r_wb.z, r_wb.w], [t_wb.x, t_wb.y, t_wb.z])
    T_bl = get_transform(rot_bl, trans_bl)
    T_wl = np.dot(T_wb, T_bl)
    cur_obj_list = world_to_lidar(object_list, T_wl)
    cur_obj_list = obj_filter(cur_obj_list, all_points)
    lidar_to_camera(cur_obj_list)
    # generate one frame
    generate_once_kitti_dataset(cnt[0], cur_obj_list, all_points, output_dir=os.path.join(dataset_dir, split))
    cnt[0] += 1
    viz()


def main():
    obj_list = get_object_from_file(box_gt_path)
    os.system('rm -r %s' % dataset_dir)
    subscriber = []
    for it in topics:
        subscriber.append(mf.Subscriber(it[0], it[1]))

    time_syn = mf.ApproximateTimeSynchronizer(subscriber, 10, 0.1, allow_headerless=True)
    publisher = rospy.Publisher("merged", PointCloud2, queue_size=1)
    time_syn.registerCallback(callback, obj_list, publisher)

    # publisher = rospy.Publisher("mergerd", PointCloud2)
    rospy.spin()

    # to use kitti viz tools
    os.system('rm -r %s' % os.path.join(kitti_viz_dir, 'data/object'))
    os.system('cp -r %s %s' % (dataset_dir, os.path.join(kitti_viz_dir, 'data')))
    os.system('cp -r %s %s' % (os.path.join(kitti_viz_dir, 'data/object/training/label_2'),
                               os.path.join(kitti_viz_dir, 'data/object/training/pred')))


if __name__ == '__main__':
    rospy.init_node("bag_to_label")
    listener = tf.TransformListener()
    main()
