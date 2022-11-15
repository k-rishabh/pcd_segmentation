import rospy
import threading
from open3d_ros_helper import open3d_ros_helper as orh
from sensor_msgs.msg import PointCloud2
from segmentation import *
import open3d as o3d

#initialise visualiser
vis = o3d.visualization.Visualizer()
vis.create_window()

def callback(msg):
    global curr_pc, msgflag
    curr_pc = msg
    msgflag = 1

def visualize(geom):
    for pc in geom:
        vis.update_geometry(pc)
    vis.poll_events()
    vis.update_renderer()

def main():
    global msgflag, curr_pc
    curr_pc = PointCloud2()
    msgflag = 0

    #initialise ros subscriber node
    rospy.init_node('pc2_subscriber')
    ros_pc = rospy.Subscriber('/rslidar_points', PointCloud2, callback)
    
    while(not msgflag):
        msgflag = msgflag

    msgflag = 0
    pcd = orh.rospc_to_o3dpc(curr_pc)
    geom = cluster(pcd)
    for pc in geom:
        vis.add_geometry(pc)
    
    t1 = threading.Thread(target = visualize, args=(geom,))
    t1.start()
    while(msgflag):
        msgflag = 0
        pcd = orh.rospc_to_o3dpc(curr_pc)
        geom = cluster(pcd)
        
    rospy.spin()

if __name__ == "__main__":
    main()