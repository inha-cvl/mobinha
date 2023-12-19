import sys
import math
import utm
import pymap3d
import numpy as np
from tqdm import tqdm

from ngiiParser import NGIIParser
from libs.quadratic_spline_interpolate import QuadraticSplineInterpolate

from shapely.geometry import LineString, Polygon
from scipy.interpolate import splprep, splev

def convert_2_360(angle):
    if angle >= 0 and angle <= math.pi:
        return angle
    else:
        return math.radians(360) + angle


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def get_yaw_error(yaw, angle):
    angle1 = convert_2_360(yaw)
    angle2 = convert_2_360(angle)
    return pi_2_pi(angle1 - angle2)


def euc_distance(pt1, pt2):
    return np.sqrt((pt2[0]-pt1[0])**2+(pt2[1]-pt1[1])**2)


def find_nearest_idx(pts, pt):
    min_dist = sys.maxsize
    min_idx = 0

    for idx, pt1 in enumerate(pts):
        dist = euc_distance(pt1, pt)
        if dist < min_dist:
            min_dist = dist
            min_idx = idx

    return min_idx


def interpolate(points, precision):
    def filter_same_points(points):
        filtered_points = []
        pre_pt = None

        for pt in points:
            if pre_pt is None or pt != pre_pt:
                filtered_points.append(pt)

            pre_pt = pt

        return filtered_points

    points = filter_same_points(points)
    if len(points) < 2:
        return points, None, None, None

    wx, wy = zip(*points)
    itp = QuadraticSplineInterpolate(list(wx), list(wy))

    itp_points = []
    s = []
    yaw = []
    k = []

    for n, ds in enumerate(np.arange(0.0, itp.s[-1], precision)):
        s.append(ds)
        x, y = itp.calc_position(ds)
        dyaw = itp.calc_yaw(ds)

        dk = itp.calc_curvature(ds)

        itp_points.append((float(x), float(y)))
        yaw.append(dyaw)
        k.append(dk)

    return itp_points, s, yaw, k


class NGII2LANELET:
    def __init__(self,
                 folder_path,
                 precision,
                 base_lla,
                 is_utm):

        a1_path = '%s/A1_NODE.shp'%(folder_path)
        a2_path = '%s/A2_LINK.shp'%(folder_path)
        # a3_path = '%s/A3_DRIVEWAYSECTION.shp'%(folder_path)
        # a4_path = '%s/A4_SUBSIDIARYSECTION.shp'%(folder_path)

        # b1_path = '%s/B1_SAFETYSIGN.shp'%(folder_path)
        b2_path = '%s/B2_SURFACELINEMARK.shp'%(folder_path)
        b3_path = '%s/B3_SURFACEMARK.shp'%(folder_path)

        c1_path = '%s/C1_TRAFFICLIGHT.shp'%(folder_path)
        # c3_path = '%s/C3_VEHICLEPROTECTIONSAFETY.shp'%(folder_path)
        # c4_path = '%s/C4_SPEEDBUMP.shp'%(folder_path)
        # c6_path = '%s/C6_POSTPOINT.shp'%(folder_path)

        ngii = NGIIParser(
            a1_path,
            a2_path,
            # a3_path,
            # a4_path,
            # b1_path, 
            b2_path, 
            b3_path,
            c1_path)
            # c3_path,
            # c4_path,
            # c6_path)
        self.base_lla = base_lla
        self.is_utm = is_utm
        self.generate_lanelet(ngii, precision, self.base_lla, self.is_utm)

    def to_cartesian(self, tx, ty, alt=None):
        if self.is_utm:
            lat, lon = utm.to_latlon(tx, ty, 52, 'N')
        else:
            lat, lon = ty, tx

        if self.base_lla is None:
            self.base_lla = (lat, lon, alt)

        if alt is None:
            x, y, _ = pymap3d.geodetic2enu(lat, lon, self.base_lla[2], self.base_lla[0], self.base_lla[1], self.base_lla[2])
            return x, y
        else:
            x, y, z = pymap3d.geodetic2enu(lat, lon, alt, self.base_lla[0], self.base_lla[1], self.base_lla[2])
            return x, y, z

    def generate_lanelet(self, ngii, precision, base_lla, is_utm):
        self.link_id_data = {}
        self.map_data = {}
        lanelets = {}

        for_vis = []

        ori2new = {}
        self.new2ori = {}

        to_node = {}
        from_node = {}

        # side_lanelets = {} # outer lines
        stoplines = {}
        safetysigns = {}
        surfacemarks = {}
        trafficlights = {}
        vehicleprotectionsafetys = {}
        speedbumps = {}
        postpoints = {}

        stoppoints = []
        for b2_surfacelinemark in ngii.b2_surfacelinemark:
            if b2_surfacelinemark.Kind == '530':            
                for tx, ty, alt in b2_surfacelinemark.geometry.coords:
                    x, y, z = self.to_cartesian(tx, ty, alt)
                    stoppoints.append((x, y))     
        
        def search_L_LinkID(id_):
            left_ids = []
            if len(id_) != 0:
                while True:
                    if lanelets[id_]['adjacentLeft'] is not None:
                        left_ids.append(lanelets[id_]['adjacentLeft'])
                        id_ = lanelets[id_]['adjacentLeft']
                    else:
                        break
                return left_ids

        def search_R_LinKID(id_):
            right_ids = []
            if len(id_) != 0:
                while True:
                    if lanelets[id_]['adjacentRight'] is not None:
                        right_ids.append(lanelets[id_]['adjacentRight'])
                        id_ = lanelets[id_]['adjacentRight']
                    else:
                        break
                return right_ids
            
        def pre_link_stop_point(id_,stop_point, sum_link_length):
            while sum_link_length < 100:
                pre_ids = lanelets[id_]['predecessor']
                if len(pre_ids) != 0:
                    for i in pre_ids:
                        lanelets[i]['stoplineID'].append(stop_point)
                        sum_link_length += lanelets[i]['length']
                        pre_link_stop_point(i, stop_point, sum_link_length)
                else:
                    break
                      
        def get_min_stop_point(id_, idx):
            min_end_length = math.sqrt((stoppoints[0][0]-lanelets[id_]['waypoints'][idx][0])**2 
                                + (stoppoints[0][1]-lanelets[id_]['waypoints'][idx][1])**2)
            min_end_point = (stoppoints[0][0], stoppoints[0][1])
            for p in stoppoints:
                end_val = math.sqrt((p[0]-lanelets[id_]['waypoints'][idx][0])**2 
                                    + (p[1]-lanelets[id_]['waypoints'][idx][1])**2)
                if min_end_length > end_val:
                    min_end_length = end_val
                    min_end_point = (p[0], p[1])
            return min_end_point

        def get_intersection_direction(id_, inter_id):
            link = lanelets[inter_id]
            p = (link['waypoints'][len(link['waypoints'])//5][0],
                link['waypoints'][len(link['waypoints'])//5][1])
            q = (link['waypoints'][len(link['waypoints'])//2][0],
                link['waypoints'][len(link['waypoints'])//2][1])
            r = (link['waypoints'][len(link['waypoints'])//5*4][0],
                link['waypoints'][len(link['waypoints'])//5*4][1])
            val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            threshold = 10
            if val <= -threshold:
                lanelets[id_]['leftTurn'] = True
            if val >= threshold:
                lanelets[id_]['rightTurn'] = True

        for n, a2_link in tqdm(enumerate(ngii.a2_link), desc="a2_link: ", total=len(ngii.a2_link)):
            if a2_link.Length == 0:
                continue

            new_id = str(n)
            ori_id = a2_link.ID

            ori2new[ori_id] = new_id
            self.new2ori[new_id] = ori_id

            lanelets[new_id] = {}

            waypoints = []
            
            for tx, ty, alt in a2_link.geometry.coords:
                x, y, z = self.to_cartesian(tx, ty, alt)
                waypoints.append((x, y))

            waypoints, s, yaw, k = interpolate(waypoints, precision)

            lanelets[new_id]['waypoints'] = waypoints
            lanelets[new_id]['idx_num'] = len(waypoints)
            lanelets[new_id]['yaw'] = yaw
            lanelets[new_id]['s'] = s
            lanelets[new_id]['k'] = k
            lanelets[new_id]['length'] = s[-1]  # a2_link.Length
            lanelets[new_id]['laneNo'] = a2_link.LaneNo
            
            lanelets[new_id]['leftTurn'] = False
            lanelets[new_id]['rightTurn'] = False
            lanelets[new_id]['uTurn'] = False
            lanelets[new_id]['direction'] = []
            lanelets[new_id]['trafficLight'] = []
            lanelets[new_id]['stoplineID'] = []
            lanelets[new_id]['crosswalkID'] = []

            lanelets[new_id]['leftBound'] = []
            lanelets[new_id]['leftType'] = []
            lanelets[new_id]['rightBound'] = []
            lanelets[new_id]['rightType'] = []
            
            lanelets[new_id]['ROI'] = []

            if to_node.get(a2_link.ToNodeID) is None:
                to_node[a2_link.ToNodeID] = []

            to_node[a2_link.ToNodeID].append(new_id)

            if from_node.get(a2_link.FromNodeID) is None:
                from_node[a2_link.FromNodeID] = []

            from_node[a2_link.FromNodeID].append(new_id)

            if a2_link.LinkType == '1':
                lanelets[new_id]['intersection'] = True
            else:
                lanelets[new_id]['intersection'] = False

            if str(a2_link.LaneNo)[0] == '9':
                lanelets[new_id]['leftTurn'] = True
            else:
                lanelets[new_id]['leftTurn'] = False

            # if int(a2_link.MaxSpeed) is None or int(a2_link.MaxSpeed) == 0.0:
            lanelets[new_id]['speedLimit'] = 50
            # else:
                # lanelets[new_id]['speedLimit'] = int(a2_link.MaxSpeed)

        for a2_link in ngii.a2_link:
            if a2_link.Length == 0:
                continue

            ori_id = a2_link.ID
            new_id = ori2new[ori_id]
            # if not lanelets[new_id]['intersection']:
            lanelets[new_id]['adjacentLeft'] = ori2new.get(a2_link.L_LinKID)
            lanelets[new_id]['adjacentRight'] = ori2new.get(a2_link.R_LinkID)
            # else:
                # lanelets[new_id]['adjacentLeft'] = None
                # lanelets[new_id]['adjacentRight'] = None

            lanelets[new_id]['predecessor'] = to_node[a2_link.FromNodeID] if to_node.get(a2_link.FromNodeID) is not None else []
            lanelets[new_id]['successor'] = from_node[a2_link.ToNodeID] if from_node.get(a2_link.ToNodeID) is not None else []

        for a2_link in tqdm(ngii.a2_link, desc="link2crosswalk_id_matching: ", total=len(ngii.a2_link)):
            if a2_link.Length == 0:
                continue

            ori_id = a2_link.ID
            new_id = ori2new[ori_id]

            link = LineString(lanelets[new_id]['waypoints'])

            for b3_surfacemark in ngii.b3_surfacemark:
                polygon = []
                if b3_surfacemark.Type == '5':
                    for tx, ty, alt in b3_surfacemark.geometry.exterior.coords:
                        x, y, z = self.to_cartesian(tx, ty, alt)
                        polygon.append((x, y))
            
                    #check cross a2 link and b3 surfacemark
                    polygon = Polygon(polygon)

                    intersects = link.intersects(polygon)
                    if intersects:
                        lanelets[new_id]['crosswalkID'].append(b3_surfacemark.ID)
        
        for a2_link in tqdm(ngii.a2_link, desc="link2stopline_id_matching: ", total=len(ngii.a2_link)):
            def extend_line(coordinates, extension_meters=3):
                """
                주어진 좌표 리스트의 시작점과 끝점을 주어진 미터 단위로 늘린다.
                """
                # 좌표가 최소 두 개 이상 필요
                if len(coordinates) < 2:
                    raise ValueError("좌표는 최소 두 개 이상 필요합니다.")

                # 시작점 처리
                start_first = coordinates[0]
                start_second = coordinates[1]
                start_bearing = math.atan2(start_second[1] - start_first[1], start_second[0] - start_first[0])
                new_start = [start_first[0] - extension_meters * math.cos(start_bearing),
                            start_first[1] - extension_meters * math.sin(start_bearing)]

                # 끝점 처리
                end_first = coordinates[-2]
                end_second = coordinates[-1]
                end_bearing = math.atan2(end_second[1] - end_first[1], end_second[0] - end_first[0])
                new_end = [end_second[0] + extension_meters * math.cos(end_bearing),
                        end_second[1] + extension_meters * math.sin(end_bearing)]

                # 새로운 좌표 리스트 생성
                new_coordinates = [new_start] + coordinates + [new_end]

                return new_coordinates

            if a2_link.Length == 0:
                continue

            ori_id = a2_link.ID
            new_id = ori2new[ori_id]
            extended_link = extend_line(lanelets[new_id]['waypoints'])
            link = LineString(extended_link)
            for b2_surfacelinemark in ngii.b2_surfacelinemark:
                stopline = []
                if b2_surfacelinemark.Kind == '530':
                    for tx, ty, alt in b2_surfacelinemark.geometry.coords:
                        x, y, z = self.to_cartesian(tx, ty, alt)
                        stopline.append((x, y))
                    stopline = LineString(stopline)
                    intersects = link.intersects(stopline)
                    if intersects:
                        lanelets[new_id]['stoplineID'].append(b2_surfacelinemark.ID)


        # Correct map error
        for id_, data in lanelets.items():
            left_id = data['adjacentLeft']
            if left_id is not None:
                left_data = lanelets[left_id]
                if left_data['adjacentRight'] != id_:
                    data['adjacentLeft'] = None

            right_id = data['adjacentRight']
            if right_id is not None:
                right_data = lanelets[right_id]
                if right_data['adjacentLeft'] != id_:
                    data['adjacentRight'] = None

        # Grouping
        groups = []
        g_closed = []
        for id_, data in tqdm(lanelets.items(), desc="not groups: ", total=len(groups)):
            if id_ not in g_closed:
                left_id = data['adjacentLeft']
                right_id = data['adjacentRight']

                if left_id is not None or right_id is not None:
                    group_n = len(groups)
                    lanelets[id_]['group'] = group_n
                    group = [id_]
                    g_closed.append(id_)
                    while left_id is not None:
                        lanelets[left_id]['group'] = group_n
                        group.insert(0, left_id)
                        g_closed.append(left_id)
                        left_id = lanelets[left_id]['adjacentLeft']

                    while right_id is not None:
                        lanelets[right_id]['group'] = group_n
                        group.append(right_id)
                        g_closed.append(right_id)
                        right_id = lanelets[right_id]['adjacentRight']

                    groups.append(group)

                else:
                    data['group'] = None

        # for b1_safetysign in tqdm(ngii.b1_safetysign, desc="b1_safetysign: ", total=len(ngii.b1_safetysign)):
        #     obj_id = b1_safetysign.ID

        #     points = []

        #     for tx, ty, alt in b1_safetysign.geometry.exterior.coords:
        #         x, y, z = self.to_cartesian(tx, ty, alt)
        #         points.append((x, y, z))

        #     safetysigns[obj_id] = points

        for b2_surfacelinemark in tqdm(ngii.b2_surfacelinemark, desc="b2_surfacelinemark: ", total=len(ngii.b2_surfacelinemark)):
            if b2_surfacelinemark.Kind is not None:
                if b2_surfacelinemark.Kind != '530':
                    ## right link
                    ori_id = b2_surfacelinemark.R_linkID
                    right_id = ori2new.get(ori_id)

                    if b2_surfacelinemark.geometry is not None:
                        leftBound = []
                        for tx, ty, alt in b2_surfacelinemark.geometry.coords:
                            x, y, z = self.to_cartesian(tx, ty, alt)
                            leftBound.append((x, y))

                        leftBound, s, yaw, k = interpolate(leftBound, precision)

                        if len(leftBound) > 1:
                            if right_id is not None:
                                lanelets[right_id]['leftBound'].append(leftBound)
                                lanelets[right_id]['leftType'].append('solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted')
                            else:
                                for_vis.append([leftBound, 'solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted'])
                    ## left link
                    ori_id = b2_surfacelinemark.L_linkID
                    left_id = ori2new.get(ori_id)

                    if b2_surfacelinemark.geometry is not None:
                        rightBound = []
                        for tx, ty, alt in b2_surfacelinemark.geometry.coords:
                            x, y, z = self.to_cartesian(tx, ty, alt)
                            rightBound.append((x, y))

                        rightBound, s, yaw, k = interpolate(rightBound, precision)

                        if len(rightBound) > 1:
                            if left_id is not None:
                                lanelets[left_id]['rightBound'].append(rightBound)
                                lanelets[left_id]['rightType'].append('solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted')
                            else:
                                for_vis.append([rightBound, 'solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted'])

                else:  # stop line
                    stopline_id = b2_surfacelinemark.ID
                    lines = []
                    for tx, ty, alt in b2_surfacelinemark.geometry.coords:
                        x, y, z = self.to_cartesian(tx, ty, alt)
                        lines.append((x, y))

                    stoplines[stopline_id] = lines
                    for_vis.append([lines, 'stop_line'])
        for id_, data in lanelets.items():
            if data['length'] < 35.0:
                yaw_err = get_yaw_error(data['yaw'][0], data['yaw'][-1])
                if abs(math.degrees(yaw_err)) > 150:
                    data['uTurn'] = True

        #TODO: CREATE ROI for front 100m
        # for n, group in tqdm(enumerate(groups), desc="roi: ", total=len(groups)):
        #     if len(group)==0:
        #         continue

        #     def get_direction(chunk):
        #         start = chunk[0]
        #         end = chunk[-1]
        #         return (end[0] - start[0], end[1] - start[1])
        #     def sort_key(chunk, all_chunks):
        #         # 모든 청크들에 대한 방향을 계산합니다.
        #         directions = [get_direction(c) for c in all_chunks]
        #         # 전체 청크들의 평균 방향을 계산합니다.
        #         avg_direction = (sum(d[0] for d in directions) / len(directions), sum(d[1] for d in directions) / len(directions))
        #         start = chunk[0]
        #         # 시작점과 평균 방향 벡터의 내적으로 정렬
        #         return start[0] * avg_direction[0] + start[1] * avg_direction[1]
        #     def get_ordered_chunks(chunks):
        #         # sorted 함수에서 여러 인자를 전달하기 위해 lambda를 사용
        #         sorted_chunks = sorted(chunks, key=lambda chunk: sort_key(chunk, chunks))
        #         return sorted_chunks
        #     def simplify_coords(coords, tolerance=0.005):
        #         if len(coords) <= 2:
        #             return coords
        #         line = LineString(coords)
        #         simplified = line.simplify(tolerance)
        #         return list(simplified.coords)
        #     def find_next_id(current_id, lanelets):
        #         next_id = lanelets[current_id]['successor']
        #         if lanelets[current_id]['uTurn']:
        #             current_id = group[1]
        #             next_id = lanelets[current_id]['successor']
        #         if next_id is not None and len(next_id) >= 1:
        #             next_id = next_id[0]
        #         else:
        #             return None

        #         return next_id
        #     def spline(bound):
        #         if len(bound) < 2:
        #             return bound
        #         x, y = zip(*bound)

        #         # 스플라인 보간
        #         tck, u = splprep([x, y], s=0)
        #         unew = np.linspace(0, 1, 100)
        #         xnew, ynew = splev(unew, tck)

        #         # 보간된 좌표를 리스트 형태로 변환
        #         return [[xi, yi] for xi, yi in zip(xnew, ynew)]
        #     def find_edge_id(flag, adjacent_id, lanelets):
        #         if flag == 'l':
        #             while adjacent_id is not None:
        #                 if lanelets[adjacent_id]['adjacentLeft'] is None:
        #                     break
        #                 adjacent_id = lanelets[adjacent_id]['adjacentLeft']
        #         elif flag == 'r':
        #             while adjacent_id is not None:
        #                 if lanelets[adjacent_id]['adjacentRight'] is None:
        #                     break
        #                 adjacent_id = lanelets[adjacent_id]['adjacentRight']
        #         return adjacent_id

        #     length = 0
        #     edge_left_new_id = group[0] # edge left link id
        #     edge_right_new_id = group[-1] # edge right link id

        #     length = lanelets[edge_left_new_id]['length']
        #     left_bound = lanelets[edge_left_new_id]['leftBound']
        #     right_bound = lanelets[edge_right_new_id]['rightBound']

        #     ordered_left = get_ordered_chunks(left_bound)
        #     ordered_right = get_ordered_chunks(right_bound)

        #     next_left_id = find_next_id(edge_left_new_id, lanelets)
        #     next_id = next_left_id
        #     next_right_id = find_edge_id('r', next_left_id, lanelets)

        #     # while length<250:
        #     #     if next_left_id is None:
        #     #         break
        #     #     length += lanelets[next_left_id]['length']
                
        #     #     next_left_id = find_edge_id('l', next_left_id, lanelets)

        #     #     left_bound = lanelets[next_left_id]['leftBound']
        #     #     right_bound = lanelets[next_right_id]['rightBound'] if next_right_id is not None else []

        #     #     ordered_left = ordered_left + get_ordered_chunks(left_bound)
        #     #     ordered_right = ordered_right + get_ordered_chunks(right_bound)

        #     #     next_left_id = find_next_id(next_left_id, lanelets)
        #     #     next_right_id = find_edge_id('r', next_left_id, lanelets)

        #     flat_left_bound = [item for sublist in ordered_left for item in sublist]
        #     flat_right_bound = [item for sublist in ordered_right for item in sublist]

        #     # all_coords = flat_left_bound + flat_right_bound[::-1]
        #     # all_coords_simplified = simplify_coords(all_coords)
        #     # spline_all_coords = spline(all_coords_simplified)
            
        #     flat_left_bound = spline(flat_left_bound)
        #     flat_right_bound = spline(flat_right_bound)

        #     simplified_left = simplify_coords(flat_left_bound)
        #     simplified_right = simplify_coords(flat_right_bound[::-1])
        #     # Combine
        #     all_coords_simplified = simplified_left + simplified_right

        #     for i in range(len(group)):
        #         lanelets[group[i]]['ROI'] = [list(coord) for coord in all_coords_simplified]
        #     # if next_id and lanelets[next_id]['intersection']:
        #         # lanelets[next_id]['ROI'] = [list(coord) for coord in all_coords_simplified]
                
            
        for id_, data in lanelets.items():
            data['leftChange'] = [True for _ in range(len(data['waypoints']))]
            data['rightChange'] = [True for _ in range(len(data['waypoints']))]

            for leftBound, leftType in zip(data['leftBound'], data['leftType']):
                if leftType == 'solid':
                    idx_s = find_nearest_idx(data['waypoints'], leftBound[0])
                    idx_f = find_nearest_idx(data['waypoints'], leftBound[-1])
                    data['leftChange'][idx_s:idx_f] = [
                        False for _ in range(idx_f-idx_s)]

            for rightBound, rightType in zip(data['rightBound'], data['rightType']):
                if rightType == 'solid':
                    idx_s = find_nearest_idx(data['waypoints'], rightBound[0])
                    idx_f = find_nearest_idx(data['waypoints'], rightBound[-1])
                    data['rightChange'][idx_s:idx_f] = [
                        False for _ in range(idx_f-idx_s)]

        for b3_surfacemark in tqdm(ngii.b3_surfacemark, desc="surfacemark: ", total=len(ngii.b3_surfacemark)):
            obj_id = b3_surfacemark.ID

            points = []

            for tx, ty, alt in b3_surfacemark.geometry.exterior.coords:
                x, y, z = self.to_cartesian(tx, ty, alt)
                points.append((x, y))

            surfacemarks[obj_id] = points

        for b3_surfacemark in tqdm(ngii.b3_surfacemark, desc="surfacemark: ", total=len(ngii.b3_surfacemark)):
            if b3_surfacemark.LinkID is not None:
                ori_id = b3_surfacemark.LinkID
                new_id = ori2new.get(ori_id)

                if new_id is not None:
                    if b3_surfacemark.Type == '1':
                        if b3_surfacemark.Kind == '5371':
                            lanelets[new_id]['direction'].append('S')

                        elif b3_surfacemark.Kind == '5372':
                            lanelets[new_id]['direction'].append('L')

                        elif b3_surfacemark.Kind == '5373':
                            lanelets[new_id]['direction'].append('R')

                        elif b3_surfacemark.Kind == '5374':
                            lanelets[new_id]['direction'].append('L')
                            lanelets[new_id]['direction'].append('R')

                        elif b3_surfacemark.Kind == '5379':
                            lanelets[new_id]['direction'].append('L')
                            lanelets[new_id]['direction'].append('S')
                            lanelets[new_id]['direction'].append('R')

                        elif b3_surfacemark.Kind == '5381':
                            lanelets[new_id]['direction'].append('L')
                            lanelets[new_id]['direction'].append('S')

                        elif b3_surfacemark.Kind == '5382':
                            lanelets[new_id]['direction'].append('S')
                            lanelets[new_id]['direction'].append('R')

                        elif b3_surfacemark.Kind == '5383':
                            lanelets[new_id]['direction'].append('S')
                            lanelets[new_id]['direction'].append('U')

                        elif b3_surfacemark.Kind == '5391':
                            lanelets[new_id]['direction'].append('U')

                        elif b3_surfacemark.Kind == '5392':
                            lanelets[new_id]['direction'].append('L')
                            lanelets[new_id]['direction'].append('U')

                        lanelets[new_id]['direction'] = list(set(lanelets[new_id]['direction']))

                        if 'R' in lanelets[new_id]['direction']:
                            right_data = None
                            for id_ in lanelets[new_id]['successor']:
                                if lanelets[id_]['intersection']:
                                    if right_data is None:
                                        right_data = [
                                            id_, lanelets[id_]['laneNo']]
                                    else:
                                        if right_data[1] < lanelets[id_]['laneNo']:
                                            right_data[0] = id_
                                            right_data[1] = lanelets[id_]['laneNo']
                        # check plz
                            if right_data is not None:
                                lanelets[right_data[0]]['rightTurn'] = True

        for n, c1_trafficlight in tqdm(enumerate(ngii.c1_trafficlight), desc="trafficlight: ", total=len(ngii.c1_trafficlight)):
            obj_id = c1_trafficlight.ID

            tx, ty, alt = list(c1_trafficlight.geometry.coords)[0]
            x, y, z = self.to_cartesian(tx, ty, alt)

            trafficlights[obj_id] = (x, y)
            
                # lanelets[new_id]['trafficLight'].append(c1_trafficlight.ID)
                # ref_lane = int(c1_trafficlight.Ref_Lane)

                # ref_n = 0
                # right_id = lanelets[new_id]['adjacentRight']
                # while right_id is not None:
                #     if ref_n < ref_lane:
                #         lanelets[right_id]['trafficLight'].append(
                #             c1_trafficlight.ID)

                #     right_id = lanelets[right_id]['adjacentRight']
                #     ref_n += 1

            #STOP LINE LINKING
            # ori_id = c1_trafficlight.LinkID
            # new_id = ori2new.get(ori_id)
            # if new_id is not None:
            #     # prev + left&right
            #     if lanelets[new_id]['intersection']:
            #         pre_new_id = lanelets[new_id]['predecessor'] # intersection prev id is only one
            #         if len(pre_new_id) != 0:
            #             min_end_point = get_min_stop_point(pre_new_id[0], -1)
            #             lanelets[pre_new_id[0]]['stopline'].append(min_end_point)
            #             sum_link_length = lanelets[pre_new_id[0]]['length']
            #             pre_link_stop_point(pre_new_id[0], min_end_point, sum_link_length)
        
            #             l_ids = search_L_LinkID(pre_new_id[0])
            #             r_ids = search_R_LinKID(pre_new_id[0])
            #             for i in l_ids:
            #                 min_end_point = get_min_stop_point(i, -1)
            #                 lanelets[i]['stopline'].append(min_end_point)
            #                 sum_link_length = lanelets[i]['length']
            #                 pre_link_stop_point(i, min_end_point, sum_link_length)
            #             for i in r_ids:
            #                 min_end_point = get_min_stop_point(i, -1)
            #                 lanelets[i]['stopline'].append(min_end_point)
            #                 sum_link_length = lanelets[i]['length']
            #                 pre_link_stop_point(i, min_end_point, sum_link_length)
            #     # left+right
            #     else:
            #         min_end_point = get_min_stop_point(new_id, len(lanelets[new_id]['waypoints'])//2)
            #         lanelets[new_id]['stopline'].append(min_end_point)
            #         sum_link_length = lanelets[new_id]['length']
            #         pre_link_stop_point(new_id, min_end_point, sum_link_length)

            #         l_ids = search_L_LinkID(new_id)
            #         r_ids = search_R_LinKID(new_id)
            #         for i in l_ids:
            #             min_end_point = get_min_stop_point(i, len(lanelets[i]['waypoints'])//2)
            #             lanelets[i]['stopline'].append(min_end_point)
            #             sum_link_length = lanelets[i]['length']
            #             pre_link_stop_point(i, min_end_point, sum_link_length)
            #         for i in r_ids:
            #             min_end_point = get_min_stop_point(i, len(lanelets[i]['waypoints'])//2)
            #             lanelets[i]['stopline'].append(min_end_point)
            #             sum_link_length = lanelets[i]['length']
            #             pre_link_stop_point(i, min_end_point, sum_link_length)

        # for c1_trafficlight in tqdm(ngii.c1_trafficlight, desc="trafficlight: ", total=len(ngii.c1_trafficlight)):


        # for c3_vehicleprotectionsafety in tqdm(ngii.c3_vehicleprotectionsafety, desc="vehicleprotectionsafety: ", total=len(ngii.c3_vehicleprotectionsafety)):
        #     obj_id = c3_vehicleprotectionsafety.ID

        #     points = []

        #     for tx, ty, alt in c3_vehicleprotectionsafety.geometry.coords:
        #         x, y, z = self.to_cartesian(tx, ty, alt)
        #         points.append((x, y, z))

        #     vehicleprotectionsafetys[obj_id] = points
        
        # for c4_speedbump in tqdm(ngii.c4_speedbump, desc="speedbump: ", total=len(ngii.c4_speedbump)):
        #     obj_id = c4_speedbump.ID

        #     for tx, ty, alt in c4_speedbump.geometry.exterior.coords:
        #         x, y, z = self.to_cartesian(tx, ty, alt)
        #         points.append((x, y, z))

        #     speedbumps[obj_id] = (x, y)
        
        # for c6_postpoint in tqdm(ngii.c6_postpoint, desc="postpoint: ", total=len(ngii.c6_postpoint)):
        #     obj_id = c6_postpoint.ID

        #     tx, ty, alt = list(c6_postpoint.geometry.coords)[0]
        #     x, y, z = self.to_cartesian(tx, ty, alt)

        #     postpoints[obj_id] = (x, y, z)

        # for b1_safetysign in tqdm(ngii.b1_safetysign, desc="safetysign: ", total=len(ngii.b1_safetysign)):

        #     if b1_safetysign.LinkID is not None:
        #         ori_id = b1_safetysign.LinkID
        #         new_id = ori2new.get(ori_id)
        #         if b1_safetysign.SubType == '311':
        #             group_n = lanelets[new_id]['group']
        #             if group_n is not None:
        #                 id_ = groups[group_n][0]
        #                 if 'U' not in lanelets[id_]['direction']:
        #                     lanelets[id_]['direction'].append('U')
        
        for id_, data in lanelets.items():
            self.link_id_data[id_]=self.new2ori[id_]

        self.map_data['base_lla'] = base_lla
        self.map_data['precision'] = precision
        self.map_data['lanelets'] = lanelets
        self.map_data['groups'] = groups
        self.map_data['for_vis'] = for_vis
        # self.map_data['side_lanelets'] = side_lanelets
        self.map_data['stoplines'] = stoplines
        self.map_data['safetysigns'] = safetysigns
        self.map_data['surfacemarks'] = surfacemarks
        self.map_data['trafficlights'] = trafficlights
        self.map_data['vehicleprotectionsafetys'] = vehicleprotectionsafetys
        self.map_data['speedbumps'] = speedbumps
        self.map_data['postpoints'] = postpoints
