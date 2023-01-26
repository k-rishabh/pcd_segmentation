import rospy
from open3d_ros_helper import open3d_ros_helper as orh
from sensor_msgs.msg import PointCloud2
from segmentation import *
import open3d as o3d
from convert import convert

def callback(msg):
    global curr_pc, msgflag
    curr_pc = msg
    msgflag = 1

def main():
    global msgflag, curr_pc
    curr_pc = PointCloud2()
    msgflag = 0
    points = []
    test_points = []

    #initialise ros subscriber node
    rospy.init_node('pc2_subscriber')
    ros_pc = rospy.Subscriber('/rslidar_points', PointCloud2, callback)

    while(not msgflag):
        msgflag = msgflag

    while(msgflag):
        msgflag = 0
        #pcd = o3d.io.read_point_cloud("/home/alpha1/PCD_segmentation/TLS_kitchen.ply")
        pcd = orh.rospc_to_o3dpc(curr_pc)
        geom = cluster(pcd)
        for pc in geom:
            if(len(pc.points)>100):
                tm = convert(pc)
            test_points.append(tm)
        break
    
    test_points = np.array(test_points)
    np.save('test_points', test_points, allow_pickle=False)

    rospy.spin()

if __name__ == "__main__":
    main()