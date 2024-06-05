import numpy as np
import matplotlib.pyplot as plt
import pymap3d
def lla2enu(file_name):
    with open(file_name, 'r') as file:
            lines = file.readlines() 
    
    geo_path = []
    for line in lines:
        stripped_line = line.strip()
        lat, long = stripped_line.split(',')
        geo_path.append((float(lat), float(long)))
    base_lat = geo_path[0][0]
    base_lon = geo_path[0][1]

    path = []
    for i in range(len(geo_path)):
        x, y, _ = pymap3d.geodetic2enu(
        geo_path[i][0], geo_path[i][1], 0, base_lat, base_lon, 0)
        path.append((x,y))

    return path

def calculate_curvature(points):
    curvatures = []

    for i in range(1, len(points) - 1):
        x1, y1 = points[i - 1]
        x2, y2 = points[i]
        x3, y3 = points[i + 1]

        # 삼각형의 면적 A
        A = 0.5 * abs(x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2))
        
        # 세 점 사이의 거리 계산
        d1 = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        d2 = np.sqrt((x3 - x2)**2 + (y3 - y2)**2)
        d3 = np.sqrt((x3 - x1)**2 + (y3 - y1)**2)
        
        if d1 * d2 * d3 == 0:
            curvature = 0
        else:
            # 곡률 계산
            curvature = (2 * A) / (d1 * d2 * d3)
        
        curvatures.append(curvature)

    return curvatures

def moving_average_filter(data, window_size):
    filter = np.ones(window_size) / window_size
    return np.convolve(data, filter, mode='same')

threshold = [0.09, 0.1, 0.75, 0.1, 0.06]
window_size = 10
for i, file_name in enumerate(['14.txt', '18.txt', '22.txt', '26.txt', '32.txt',]):
    points = lla2enu(file_name)
    plt.plot([el[0] for el in points], [el[1] for el in points])
    plt.grid()
    plt.axis('equal')
    plt.show()

    curvatures = calculate_curvature(points)
    filtered_curvatures = [el for el in curvatures if el < threshold[i]]
    smoothed_curvatures = moving_average_filter(filtered_curvatures, window_size)

    plt.plot(smoothed_curvatures[len(smoothed_curvatures)//2:], label="curvature")
    plt.title(file_name)
    plt.legend()
    plt.grid()
    plt.show()
    print(max(smoothed_curvatures))

plt.plot([14, 18, 22, 26, 32], [0.07, 0.06, 0.045, 0.043, 0.035])
plt.grid()
plt.show()
    # 14 : 0.07
    # 18 : 0.06
    # 22 : 0.045
    # 26 : 0.043
    # 32 : 0.035