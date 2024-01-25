import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import time
class Map_processor:
    def __init__(self) -> None:
        self.map_sub = rospy.Subscriber("/nvblox_node/occupancy", PointCloud2, self.callback, queue_size=1)
        self.map_pub = rospy.Publisher("new_map", PointCloud2, queue_size=5)
        self.map_intensity = 0.5
        self.x_min, self.x_max = -1, 1
        self.y_min, self.y_max = -1, 1
        self.z_min, self.z_max = -1, 1
        self.step_size = 0.05
    def callback(self, data):
        processed_map = self.process(data)
        map_msg = self.convert2msg(processed_map, parent_frame='panda_link0')
        self.map_pub.publish(map_msg)
        print(">>> In callback")
    def process(self, input_data):
        # get data
        t0 = time.time()
        pc0 = ros_numpy.numpify(input_data)
        t1 = time.time()
        self.points = np.zeros((pc0.shape[0],4))
        self.points[:, 0] = pc0['x']
        self.points[:, 1] = pc0['y']
        self.points[:, 2] = pc0['z']
        self.points[:, 3] = pc0['intensity']
        # create workspace
        x_range = np.arange(self.x_min, self.x_max+self.step_size, self.step_size)
        y_range = np.arange(self.y_min, self.y_max+self.step_size, self.step_size)
        z_range = np.arange(self.z_min, self.z_max+self.step_size, self.step_size)
        # wksp = np.array([[x, y, z, self.map_intensity] for x in x_range for y in y_range for z in z_range])
        X, Y, Z = np.meshgrid(x_range, y_range, z_range, indexing='ij')
        wksp = np.stack((X, Y, Z), axis=-1)
        intensity = np.full(wksp.shape[:-1] + (1,), self.map_intensity)
        wksp = np.concatenate((wksp, intensity), axis=-1)
        wksp = wksp.reshape(-1, 4)
        t2 = time.time()
        # check within wksp
        within_workspace_mask = np.where(
            (self.points[:, 0] >= self.x_min) & (self.points[:, 0] <= self.x_max) &
            (self.points[:, 1] >= self.y_min) & (self.points[:, 1] <= self.y_max) &
            (self.points[:, 2] >= self.z_min) & (self.points[:, 2] <= self.z_max)
        )
        self.new_points = self.points[within_workspace_mask[0]]
        t3 = time.time()
        # update wksp
        num_z = round((self.z_max-self.z_min)/self.step_size)+1
        num_y = round((self.y_max-self.y_min)/self.step_size)+1
        num_x = round((self.x_max-self.x_min)/self.step_size)+1
        map_ind = np.round(np.array(
            np.round((self.new_points[:, 2]-self.z_min)/self.step_size) +
            np.round((self.new_points[:, 1]-self.y_min)/self.step_size)*num_z +
            np.round((self.new_points[:, 0]-self.x_min)/self.step_size)*(num_z*num_y)
            )).astype(np.int32)
        # print(map_ind)
        wksp[map_ind, 3] = self.new_points[:, 3]
        # print(wksp.shape)
        t4 = time.time()
        # print(">>>>>>>", t1-t0, t2-t1, t3-t2, t4-t3)
        return wksp
    def convert2msg(self, points, parent_frame):
        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        # points = np.zeros((1, 4))
        data = points.astype(dtype).tobytes()
        fields = [sensor_msgs.PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]
        fields.append(sensor_msgs.PointField(name="intensity", offset=3*itemsize, datatype=ros_dtype, count=1))
        header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.now())
        return sensor_msgs.PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 4),
            row_step=(itemsize * 4 * points.shape[0]),
            data=data
        )
rospy.init_node('map_processor', anonymous=True)
rate = rospy.Rate(100)
map_node = Map_processor()
t_old = time.time()
while not rospy.is_shutdown():
    t_ = time.time()
    # print(t_ - t_old)
    t_old = t_
    rate.sleep()