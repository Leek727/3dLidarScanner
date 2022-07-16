import math

data = []
with open("data.csv", "r") as f:
    data = f.read().split("\n")

# get first 3 nums
data = [x.split(",")[0:4] for x in data]

xstep = 2
ystep = 2 # hardcoded value in ardu code that determines steps per lidar point

pointcloud = []
cumsum = 0
total = 0
for pt in data:
    try:
        pt = list(map(int, pt))
        # large outlier detection
        pt[2],pt[3]
    except:
        continue
    
    if abs(pt[2]) > 1000 or pt[3] < 300 or (pt[1] < 10 and pt[2] > 50):
        #print(pt[2])
        continue
    #if pt[1] < 10

    # 3d polar to cartesian
    r = pt[2]
    phi = ((pt[0] * xstep) / 4076) * (2 * math.pi)
    theta = ((pt[1] * ystep) / 4076) * (2 * math.pi)

    #print(pt[0],pt[1])
    #print((phi/(2 * math.pi)) * 360,(theta/(2 * math.pi)) *360)

    x = r * math.sin(theta) * math.cos(phi)
    y = r * math.sin(theta) * math.sin(phi)
    z = r * math.cos(theta)
    pointcloud.append([x,y,z])