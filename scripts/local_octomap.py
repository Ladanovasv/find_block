import rospy
from geometry_msgs.msg import Point
import numpy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class LocalOctomap:
    def __init__(self) -> None:
        rospy.init_node('local_octomap')
        self.center_pcd_pub = rospy.Publisher(
            '/center_points_local_map', PointCloud2, queue_size=10)
        
        self.marker_array_pub = rospy.Publisher(
            '/marker_array_local_map', MarkerArray, queue_size=10)
        
        self.center_pcd_sub = rospy.Subscriber(
            '/octomap_point_cloud_centers',
            PointCloud2,
            self.callback_pcd
        )
        self.point_sub = rospy.Subscriber(
            '/point',
            Point,
            self.callback_point
        )
        


    def callback_pcd(self, pcd_msg):
        self.pcd_numpy = self.pcd2pcd_numpy(pcd_msg)
        self.octree = self.pointcloud2octree(self.pcd_numpy)

    def callback_point(self, point_msg):

        node, node_info = self.octree.locate_leaf_node(numpy.array([point_msg.x, point_msg.y, point_msg.z]))
        if node is None:
            arr = None
        else:
            arr = self.pcd_numpy[node.indices]
        self.add_pointcloud(arr)
        self.add_markers(arr)
    
    def pcd2pcd_numpy(self, pcd_msg):
        pcd = pc2.read_points_list(
                        pcd_msg, field_names=("x", "y", "z"), skip_nans=True)
        pcd_numpy = numpy.array(pcd)
        return pcd_numpy
    
    def pointcloud2octree(self, pcd_numpy):
        octree = o3d.geometry.Octree(max_depth=2)
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = o3d.utility.Vector3dVector(pcd_numpy)
        octree.convert_from_point_cloud(pointcloud, size_expand=0.05)
        return octree
    
    def add_pointcloud(self, arr):
        if arr is None:
            centroid_pc2 = PointCloud2()
        else:
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),]
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "local_map_lidar"
            centroid_pc2 = pc2.create_cloud(header, fields, arr)
        self.center_pcd_pub.publish(centroid_pc2)
        

    def add_markers(self, arr):
        marker_ests = MarkerArray()
        if not (arr is None):
            marker_ests.markers = []
            for idx in range(len(arr)):
                marker_est = Marker()
                marker_est.header.frame_id = "local_map_lidar"
                marker_est.header.stamp = rospy.Time.now()
                marker_est.id = idx
                marker_est.type = Marker.CUBE
                marker_est.color.r, marker_est.color.g, marker_est.color.b = (255, 255, 255)
                marker_est.color.a = 1
                marker_est.scale.x, marker_est.scale.y, marker_est.scale.z = (0.1, 0.1, 0.1)
                marker_est.pose.position.x = arr[idx][0]
                marker_est.pose.position.y = arr[idx][1]
                marker_est.pose.position.z = arr[idx][2]
                marker_ests.markers.append(marker_est)
        self.marker_array_pub.publish(marker_ests) 
        

def main(args=None):
    node = LocalOctomap()
    rospy.spin()


if __name__ == '__main__':
    main()


