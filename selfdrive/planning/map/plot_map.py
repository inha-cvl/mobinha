import geopandas as gpd
import pymap3d
import matplotlib.pyplot as plt
import utm
import os
import numpy as np

def rotate_points(x, y, angle):
    angle_rad = np.radians(angle)
    x = np.array(x)
    y = np.array(y)
    x_rotated = x * np.cos(angle_rad) - y * np.sin(angle_rad)
    y_rotated = x * np.sin(angle_rad) + y * np.cos(angle_rad)
    return x_rotated, y_rotated

class PathAnalysis:
    def __init__(self, map_name, map_file, base_lla, rotation_angle, mode='save', is_utm=True, route_file=None):
        self.map = map_file
        self.base_lla = base_lla
        self.rotation_angle = rotation_angle
        self.is_utm = is_utm
        self.clicked_points = []
        self.click_counter = 0
        self.mode = mode
        self.route_file = route_file

        # 确认目录是否存在，不存在则创建
        self.route_dir = '/workspace/mobinha/src/mobinha/selfdrive/planning/map/route_generate/'
        os.makedirs(self.route_dir, exist_ok=True)

        if self.mode == 'save':
            # 获取现有同名文件数量
            existing_files = [f for f in os.listdir(self.route_dir) if f.startswith(f'{map_name}_route') and f.endswith('.txt')]
            file_index = len(existing_files) + 1

            # 创建保存路径文件的路径
            self.save_route_txt = f'{self.route_dir}{map_name}_route_{file_index}.txt'

        elif self.mode == 'load' and self.route_file:
            # 从指定的文件中读取数据
            self.load_route_txt = self.route_file
            self.load_clicked_points()

    def load_clicked_points(self):
        # 从指定文件中读取点击的点
        with open(self.load_route_txt, 'r') as f:
            for line in f:
                idx, coords = line.split(': ')
                x, y = map(float, coords.split(', '))
                self.clicked_points.append((x, y))
                self.click_counter = int(idx)

    def to_cartesian(self, tx, ty, alt=None):
        if self.is_utm:
            lat, lon = utm.to_latlon(tx, ty, 52, 'N')
        else:
            lat, lon = ty, tx
        if alt is None:
            x, y, _ = pymap3d.geodetic2enu(lat, lon, self.base_lla[2], self.base_lla[0], self.base_lla[1], self.base_lla[2])
            return x, y
        else:
            x, y, z = pymap3d.geodetic2enu(lat, lon, alt, self.base_lla[0], self.base_lla[1], self.base_lla[2])
            return x, y, z

    def process_surface_line(self):
        shp_file = f'{self.map}B2_SURFACELINEMARK.shp'
        gdf = gpd.read_file(shp_file)
        lines_x, lines_y = [], []
        for index, row in gdf.iterrows():
            if row['geometry'].geom_type == 'LineString':
                line_x, line_y = [], []
                for point in row['geometry'].coords:
                    lon, lat = point[0], point[1]
                    x, y = self.to_cartesian(lon, lat)
                    line_x.append(x)
                    line_y.append(y)
                rotated_x, rotated_y = rotate_points(np.array(line_x), np.array(line_y), self.rotation_angle)
                lines_x.append(rotated_x)
                lines_y.append(rotated_y)
        self.surface_lines = (lines_x, lines_y)

    def plot_data(self):
        self.fig, self.ax = plt.subplots()
        
        # 绘制地表线
        for line_x, line_y in zip(*self.surface_lines):
            self.ax.plot(line_x, line_y, color='black', linewidth=0.5)

        # 在读取模式下，绘制加载的点
        if self.mode == 'load':
            for i, (x, y) in enumerate(self.clicked_points):
                self.ax.plot(x, y, 'bo')
                self.ax.text(x+1, y, str(i + 1), color='red', fontsize=10)

        self.ax.set_xlabel('x (m)')
        self.ax.set_ylabel('y (m)')
        self.ax.grid(False)
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        # 连接事件处理程序
        self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_move)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)

        self.coord_text = self.ax.text(0.95, 0.95, "", transform=self.ax.transAxes, fontsize=10,
                                       verticalalignment='top', horizontalalignment='right')
        self.dragging = False
        self.start_drag = None

        plt.show()

    def update_coord_text(self):
        coord_str = '\n'.join([f'{i + 1}: {x:.2f}, {y:.2f}' for i, (x, y) in enumerate(self.clicked_points)])
        self.coord_text.set_text(coord_str)
        plt.draw()

    def redraw(self):
        # 保存当前视图限制
        cur_xlim = self.ax.get_xlim()
        cur_ylim = self.ax.get_ylim()
        
        self.ax.clear()
        # 重新绘制地表线
        for line_x, line_y in zip(*self.surface_lines):
            self.ax.plot(line_x, line_y, color='black', linewidth=0.5)
        # 重新绘制蓝点
        for i, (x, y) in enumerate(self.clicked_points):
            self.ax.plot(x, y, 'bo')
            self.ax.text(x, y, str(i + 1), color='blue', fontsize=8)
        self.ax.set_xlabel('x (m)')
        self.ax.set_ylabel('y (m)')
        self.ax.grid(False)
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.update_coord_text()
        # 恢复视图限制
        self.ax.set_xlim(cur_xlim)
        self.ax.set_ylim(cur_ylim)
        plt.draw()

    def onclick(self, event):
        if self.mode == 'save' and event.button == 1:  # Left click
            if event.xdata and event.ydata:
                x, y = event.xdata, event.ydata
                self.clicked_points.append((x, y))
                self.click_counter += 1
                print(f'Clicked at: x={x}, y={y}')
                with open(self.save_route_txt, 'a') as f:
                    f.write(f'{self.click_counter}: {x}, {y}\n')
                self.redraw()
        elif event.button == 3:  # Right click
            self.dragging = True
            self.start_drag = (event.xdata, event.ydata)

    def on_key(self, event):
        if self.mode == 'save' and event.key == 'a' and self.clicked_points:
            self.clicked_points.pop()
            self.click_counter -= 1
            print('Removed last clicked point')
            with open(self.save_route_txt, 'w') as f:
                for idx, (x, y) in enumerate(self.clicked_points, 1):
                    f.write(f'{idx}: {x}, {y}\n')
            self.redraw()

    def on_scroll(self, event):
        base_scale = 1.1
        cur_xlim = self.ax.get_xlim()
        cur_ylim = self.ax.get_ylim()
        
        xdata = event.xdata
        ydata = event.ydata
        
        if event.button == 'up':
            scale_factor = 1 / base_scale
        elif event.button == 'down':
            scale_factor = base_scale
        else:
            scale_factor = 1
            print(event.button)
        
        new_width = (cur_xlim[1] - cur_xlim[0]) * scale_factor
        new_height = (cur_ylim[1] - cur_ylim[0]) * scale_factor
        
        relx = (cur_xlim[1] - xdata) / (cur_xlim[1] - cur_xlim[0])
        rely = (cur_ylim[1] - ydata) / (cur_ylim[1] - cur_ylim[0])
        
        self.ax.set_xlim([xdata - new_width * (1 - relx), xdata + new_width * (relx)])
        self.ax.set_ylim([ydata - new_height * (1 - rely), ydata + new_height * (rely)])
        plt.draw()

    def on_move(self, event):
        if self.dragging and self.start_drag:
            dx = event.xdata - self.start_drag[0]
            dy = event.ydata - self.start_drag[1]
            cur_xlim = self.ax.get_xlim()
            cur_ylim = self.ax.get_ylim()
            self.ax.set_xlim(cur_xlim[0] - dx, cur_xlim[1] - dx)
            self.ax.set_ylim(cur_ylim[0] - dy, cur_ylim[1] - dy)
            self.start_drag = (event.xdata, event.ydata)
            plt.draw()

    def on_release(self, event):
        if event.button == 3:  # Right click release
            self.dragging = False
            self.start_drag = None

if __name__ == '__main__':
    map_name_list = ['songdo', 'kcity']
    mode_list = ['save', 'load']
    
    map_name = map_name_list[1]
    mode = mode_list[1] # 或 'load'
    
    route_file = '/workspace/mobinha/src/mobinha/selfdrive/planning/map/route_generate/kcity_route_4.txt'  # 如果是 'load' 模式，需要提供路径，如 '/workspace/mobinha/src/mobinha/selfdrive/planning/map/route_generate/kcity_route_1.txt'

    if map_name == 'songdo':
        base_lla = (37.3888319, 126.6428739, 7.369)
        rotation_angle = 0
        map_file = '/workspace/mobinha/src/mobinha/selfdrive/planning/map/songdo/'
        is_utm = False

    elif map_name == 'kcity':
        base_lla = (37.2292221592864, 126.76912499027308, 29.18400001525879)
        rotation_angle = 0
        map_file = '/workspace/mobinha/src/mobinha/selfdrive/planning/map/KCity/'
        is_utm = True

    analyzer = PathAnalysis(map_name, map_file, base_lla, rotation_angle, mode, is_utm, route_file)

    analyzer.process_surface_line()

    # 绘制数据
    analyzer.plot_data()
