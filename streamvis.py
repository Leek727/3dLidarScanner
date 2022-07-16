import numpy as np
import open3d 
from process_data import pointcloud
import math


pcd = open3d.geometry.PointCloud()
pcd.points = open3d.utility.Vector3dVector(pointcloud)

flag = True

def updatecloud(f):
    global flag
    flag = not flag
    if flag:
        return

    print("Updating...")
    global pointcloud

    data = []
    with open("data.csv", "r") as f:
        data = f.read().split("\n")

    # get first 3 nums
    data = [x.split(",")[0:4] for x in data][int(len(pointcloud) + len(pointcloud)/3):len(data)]

    xstep = 2
    ystep = 2 # hardcoded value in ardu code that determines steps per lidar point

    tempcloud = []
    for pt in data:
        try:
            pt = list(map(int, pt))
            pt[2],pt[3]
        except:
            continue
        
        if abs(pt[2]) > 2000 or pt[3] < 300 or (pt[1] < 10 and pt[2] > 50):
            #print(pt[2])
            continue

        # 3d polar to cartesian
        r = pt[2]
        phi = ((pt[0] * xstep) / 4076) * (2 * math.pi)
        theta = ((pt[1] * ystep) / 4076) * (2 * math.pi)

        x = r * math.sin(theta) * math.cos(phi)
        y = r * math.sin(theta) * math.sin(phi)
        z = r * math.cos(theta)
        tempcloud.append([x,y,z])

    pointcloud += tempcloud
    pcd.points = open3d.utility.Vector3dVector(pointcloud)
    vis.update_geometry(pcd)
    vis.update_renderer()
    vis.poll_events()
    vis.run()

vis = open3d.visualization.VisualizerWithKeyCallback()
vis.create_window()
vis.register_key_callback(65, updatecloud)
vis.add_geometry(pcd)
vis.run()

