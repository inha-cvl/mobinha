import sys
import math
import utm
import pymap3d
import numpy as np
from tqdm import tqdm

from ngii.ngiiParser import NGIIParser
from quadratic_spline_interpolate import QuadraticSplineInterpolate


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

        a2_path = '%s/A2_LINK.shp'%(folder_path)
        b1_path = '%s/B1_SAFETYSIGN.shp'%(folder_path)
        b2_path = '%s/B2_SURFACELINEMARK.shp'%(folder_path)
        b3_path = '%s/B3_SURFACEMARK.shp'%(folder_path)
        c1_path = '%s/C1_TRAFFICLIGHT.shp'%(folder_path)

        ngii = NGIIParser(a2_path, b1_path, b2_path, b3_path, c1_path)
        self.generate_lanelet(ngii, precision, base_lla, is_utm)

    def generate_lanelet(self, ngii, precision, base_lla, is_utm):
        self.map_data = {}

        lanelets = {}
        for_vis = []

        ori2new = {}
        self.new2ori = {}

        to_node = {}
        from_node = {}

        for n, a2_link in tqdm(enumerate(ngii.a2_link), desc="links: ", total=len(ngii.a2_link)):
            if a2_link.Length == 0:
                continue

            new_id = str(n)
            ori_id = a2_link.ID

            ori2new[ori_id] = new_id
            self.new2ori[new_id] = ori_id

            lanelets[new_id] = {}

            waypoints = []
            for tx, ty, alt in a2_link.geometry.coords:
                if is_utm:
                    lat, lon = utm.to_latlon(tx, ty, 52, 'N')
                else:
                    lat, lon = tx, ty

                if base_lla is None:
                    base_lla = (lat, lon, alt)

                x, y, z = pymap3d.geodetic2enu(lat, lon, alt, base_lla[0], base_lla[1], base_lla[2])
                waypoints.append((x, y))

            waypoints, s, yaw, k = interpolate(waypoints, precision)
            lanelets[new_id]['waypoints'] = waypoints
            lanelets[new_id]['idx_num'] = len(waypoints)
            lanelets[new_id]['yaw'] = yaw
            lanelets[new_id]['s'] = s
            lanelets[new_id]['k'] = k
            lanelets[new_id]['length'] = s[-1] # a2_link.Length
            lanelets[new_id]['laneNo'] = a2_link.LaneNo
            lanelets[new_id]['rightTurn'] = False
            lanelets[new_id]['uTurn'] = False
            lanelets[new_id]['direction'] = []
            lanelets[new_id]['trafficLight'] = []
            lanelets[new_id]['leftBound'] = []
            lanelets[new_id]['leftType'] = []
            lanelets[new_id]['rightBound'] = []
            lanelets[new_id]['rightType'] = []

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

            if lanelets[new_id]['intersection'] and str(a2_link.LaneNo)[0] == '9':
                lanelets[new_id]['leftTurn'] = True
            else:
                lanelets[new_id]['leftTurn'] = False

            if int(a2_link.MaxSpeed) is None or int(a2_link.MaxSpeed) == 0.0:
                lanelets[new_id]['speedLimit'] = 300
            else:
                lanelets[new_id]['speedLimit'] = int(a2_link.MaxSpeed)
 
        for a2_link in ngii.a2_link:
            if a2_link.Length == 0:
                continue

            ori_id = a2_link.ID
            new_id = ori2new[ori_id]
            if not lanelets[new_id]['intersection']:
                lanelets[new_id]['adjacentLeft'] = ori2new.get(a2_link.L_LinkID)
                lanelets[new_id]['adjacentRight'] = ori2new.get(a2_link.R_LinkID)
            else:
                lanelets[new_id]['adjacentLeft'] = None
                lanelets[new_id]['adjacentRight'] = None

            lanelets[new_id]['predecessor'] = to_node[a2_link.FromNodeID] if to_node.get(a2_link.FromNodeID) is not None else []
            lanelets[new_id]['successor'] = from_node[a2_link.ToNodeID] if from_node.get(a2_link.ToNodeID) is not None else []

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

        for b2_surfacelinemark in tqdm(ngii.b2_surfacelinemark, desc="surfacelinemark: ", total=len(ngii.b2_surfacelinemark)):
            if b2_surfacelinemark.Kind is not None:
                if b2_surfacelinemark.Kind != '530':
                    ori_id = b2_surfacelinemark.R_linkID
                    right_id = ori2new.get(ori_id)

                    if b2_surfacelinemark.geometry is not None:
                        leftBound = []
                        for tx, ty, alt in b2_surfacelinemark.geometry.coords:
                            if is_utm:
                                lat, lon = utm.to_latlon(tx, ty, 52, 'N')
                            else:
                                lat, lon = tx, ty

                            x, y, z = pymap3d.geodetic2enu(lat, lon, alt, base_lla[0], base_lla[1], base_lla[2])
                            leftBound.append((x, y))

                        leftBound, s, yaw, k = interpolate(leftBound, precision)

                        if len(leftBound) > 1:
                            if right_id is not None:
                                lanelets[right_id]['leftBound'].append(leftBound)
                                lanelets[right_id]['leftType'].append('solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted')
                            else:
                                for_vis.append([leftBound, 'solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted'])

                    ori_id = b2_surfacelinemark.L_linkID
                    left_id = ori2new.get(ori_id)

                    if b2_surfacelinemark.geometry is not None:
                        rightBound = []
                        for x, y, alt in b2_surfacelinemark.geometry.coords:
                            if is_utm:
                                lat, lon = utm.to_latlon(tx, ty, 52, 'N')
                            else:
                                lat, lon = tx, ty

                            x, y, z = pymap3d.geodetic2enu(lat, lon, alt, base_lla[0], base_lla[1], base_lla[2])
                            rightBound.append((x, y))

                        rightBound, s, yaw, k = interpolate(rightBound, precision)

                        if len(rightBound) > 1:
                            if left_id is not None:
                                lanelets[left_id]['rightBound'].append(rightBound)
                                lanelets[left_id]['rightType'].append('solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted')
                            else:
                                for_vis.append([rightBound, 'solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted'])

                else: # stop line
                    lines = []
                    for x, y, alt in b2_surfacelinemark.geometry.coords:
                        if is_utm:
                            lat, lon = utm.to_latlon(tx, ty, 52, 'N')
                        else:
                            lat, lon = tx, ty

                        x, y, z = pymap3d.geodetic2enu(lat, lon, alt, base_lla[0], base_lla[1], base_lla[2])
                        lines.append((x, y))

                    for_vis.append([lines, 'stop_line'])

        for id_, data in lanelets.items():
            data['leftChange'] = [True for _ in range(len(data['waypoints']))]
            data['rightChange'] = [True for _ in range(len(data['waypoints']))]

            for leftBound, leftType in zip(data['leftBound'], data['leftType']):
                if leftType == 'solid':
                    idx_s = find_nearest_idx(data['waypoints'], leftBound[0])
                    idx_f = find_nearest_idx(data['waypoints'], leftBound[-1])
                    data['leftChange'][idx_s:idx_f] = [False for _ in range(idx_f-idx_s)]

            for rightBound, rightType in zip(data['rightBound'], data['rightType']):
                if rightType == 'solid':
                    idx_s = find_nearest_idx(data['waypoints'], rightBound[0])
                    idx_f = find_nearest_idx(data['waypoints'], rightBound[-1])
                    data['rightChange'][idx_s:idx_f] = [False for _ in range(idx_f-idx_s)]

        for n, c1_trafficlight in tqdm(enumerate(ngii.c1_trafficlight), desc="trafficlight: ", total=len(ngii.c1_trafficlight)):
            ori_id = c1_trafficlight.LinkID
            new_id = ori2new.get(ori_id)
            if new_id is not None:
                lanelets[new_id]['trafficLight'].append(c1_trafficlight.ID)
                ref_lane = int(c1_trafficlight.Ref_Lane)

                ref_n = 0
                right_id = lanelets[new_id]['adjacentRight']
                while right_id is not None:
                    if ref_n < ref_lane:
                        lanelets[right_id]['trafficLight'].append(c1_trafficlight.ID)

                    right_id = lanelets[right_id]['adjacentRight']
                    ref_n += 1

        for b3_surfacemark in tqdm(ngii.b3_surfacemark, desc="surfacemark: ", total=len(ngii.b3_surfacemark)):
            if b3_surfacemark.LinkID is not None:
                ori_id = b3_surfacemark.LinkID
                new_id = ori2new.get(ori_id)
                if new_id is not None:
                    if b3_surfacemark.Type == '1':
                        if b3_surfacemark.Kind == '5371':
                            lanelets[new_id]['direction'] = ['S']

                        elif b3_surfacemark.Kind == '5372':
                            lanelets[new_id]['direction'] = ['L']

                        elif b3_surfacemark.Kind == '5373':
                            lanelets[new_id]['direction'] = ['R']

                        elif b3_surfacemark.Kind == '5374':
                            lanelets[new_id]['direction'] = ['L', 'R']

                        elif b3_surfacemark.Kind == '5379':
                            lanelets[new_id]['direction'] = ['L', 'S', 'R']

                        elif b3_surfacemark.Kind == '5381':
                            lanelets[new_id]['direction'] = ['L', 'S']

                        elif b3_surfacemark.Kind == '5382':
                            lanelets[new_id]['direction'] = ['S', 'R']

                        elif b3_surfacemark.Kind == '5383':
                            lanelets[new_id]['direction'] = ['S', 'U']

                        elif b3_surfacemark.Kind == '5391':
                            lanelets[new_id]['direction'] = ['U']

                        elif b3_surfacemark.Kind == '5392':
                            lanelets[new_id]['direction'] = ['L', 'U']

                        if 'R' in lanelets[new_id]['direction']:
                            right_data = None
                            for id_ in lanelets[new_id]['successor']:
                                if lanelets[id_]['intersection']:
                                    if right_data is None:
                                        right_data = [id_, lanelets[id_]['laneNo']]
                                    else:
                                        if right_data[1] < lanelets[id_]['laneNo']:
                                            right_data[0] = id_
                                            right_data[1] = lanelets[id_]['laneNo']

                            if right_data is not None:
                                lanelets[right_data[0]]['rightTurn'] = True

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
            if data['length'] < 35.0:
                yaw_err = get_yaw_error(data['yaw'][0], data['yaw'][-1])
                if abs(math.degrees(yaw_err)) > 160:
                    data['uTurn'] = True

        self.map_data['base_lla'] = base_lla
        self.map_data['precision'] = precision
        self.map_data['lanelets'] = lanelets
        self.map_data['groups'] = groups
        self.map_data['for_vis'] = for_vis