import numpy as np
from math import sin, cos, atan2, radians, degrees
import pymap3d
import matplotlib.pyplot as plt
path = []
def run(vEgo, position, yawRate, cte=0):
    global path
    geo_path = []
    with open('/home/jourmain/mobinha/selfdrive/car/test/libs/path_log.txt', 'r') as file:
    # with open('path_log.txt', 'r') as file:
        lines = file.readlines() 

    for line in lines:
        stripped_line = line.strip()
        lat, long = stripped_line.split(',')
        geo_path.append((float(lat), float(long)))
    base_lat = geo_path[0][0]
    base_lon = geo_path[0][1]

    for i in range(len(geo_path)):
        x, y, _ = pymap3d.geodetic2enu(
        geo_path[i][0], geo_path[i][1], 0, base_lat, base_lon, 0)
        path.append((x,y))

    lfd = 5 + 1 * vEgo / 3.6  
    lfd = np.clip(lfd, 4, 60) 
    steering_angle = 0.  
    lx, ly = path[0] 
    for point in path:
        diff = np.asarray((point[0] - position[0], point[1] - position[1]))
        rotation_matrix = np.array(
            ((np.cos(radians(yawRate)), -np.sin(radians(yawRate))), 
                (np.sin(radians(yawRate)), np.cos(radians(yawRate)))))
        rotated_diff = rotation_matrix.dot(diff)
        if rotated_diff[0] > 0:
            plt.plot([position[0], position[0]+rotated_diff[0]/np.linalg.norm(rotated_diff)], [position[1], position[1]+rotated_diff[1]/np.linalg.norm(rotated_diff)])
            dis = np.linalg.norm(rotated_diff - np.array([0, 0]))
            if dis >= lfd: 
                theta = np.arctan2(rotated_diff[1], rotated_diff[0]) 
                steering_angle = np.arctan2(2 * 3 * np.sin(theta), lfd)
                # steering_angle = steering_angle + np.arctan2(0.1 * cte, vEgo) if vEgo > 6 else steering_angle
                lx = point[0]  
                ly = point[1]  
                break
    return degrees(steering_angle), (lx, ly)  # Return the steering angle in degrees and the target point
if __name__ == "__main__":
    v =1.8446180555555556
    pos = (-20.396328791938615, 27.43853350614983)
    heading = -143.58772961264225 +360
    angle, (tx, ty) = run(v, pos, heading)
    print("result : ", angle)
    print("target : ", tx, ty)
    plt.scatter(pos[0], pos[1])
    plt.plot([pt[0] for pt in path], [pt[1] for pt in path])
    plt.plot([pos[0], pos[0]+cos(radians(heading))], [pos[1], pos[1]+cos(radians(heading))])
    plt.scatter([tx], [ty])
    plt.grid(True)
    plt.show()

    # vEgo, path, position, yawRate, cte
    #v : 1.8489583333333333, pos: (-20.275103792422193, 27.635642078066144), heading: -144.01293900697044, target: 0.11273516760735534, -0.5387912950040479
#     v : 1.8446180555555556, pos: -20.396328791938615. 27.43853350614983, heading: -143.58772961264225, target: 0.11273516760735534, -0.5387912950040479, idx: 83
# error occurred
