import geopandas as gpd


class NGIIParser:
    def __init__(self,
        a1_path,
        a2_path,
        a3_path,
        a4_path,
        b1_path, 
        b2_path, 
        b3_path, 
        c1_path,
        c3_path,
        c4_path,
        c6_path):

        self.a1_node = self.parse_a1_node(a1_path)
        self.a2_link = self.parse_a2_link(a2_path)
        self.a3_drivewaysection = self.parse_a3_drivewaysection(a3_path)
        self.a4_subsidiarysection = self.parse_a4_subsidiarysection(a4_path)

        self.b1_safetysign = self.parse_b1_safetysign(b1_path)
        self.b2_surfacelinemark = self.parse_b2_surfacelinemark(b2_path)
        self.b3_surfacemark = self.parse_b3_surfacemark(b3_path)

        self.c1_trafficlight = self.parse_c1_trafficlight(c1_path)
        self.c3_vehicleprotectionsafety = self.parse_c3_vehicleprotectionsafety(c3_path)
        self.c4_speedbump = self.parse_c4_speedbump(c4_path)
        self.c6_postpoint = self.parse_c6_postpoint(c6_path)

    def parse_a1_node(self, path):
        shape_data = gpd.read_file(path)
        a1_node = []
        for index, data in shape_data.iterrows():
            a1_node.append(data)
        return a1_node

    def parse_a2_link(self, path):
        shape_data = gpd.read_file(path)
        a2_link = []
        for index, data in shape_data.iterrows():
            a2_link.append(data)
        return a2_link

    def parse_a3_drivewaysection(self, path):
        shape_data = gpd.read_file(path)
        a3_drivewaysection = []
        for index, data in shape_data.iterrows():
            a3_drivewaysection.append(data)
        return a3_drivewaysection

    def parse_a4_subsidiarysection(self, path):
        shape_data = gpd.read_file(path)
        a4_subsidiarysection = []
        for index, data in shape_data.iterrows():
            a4_subsidiarysection.append(data)
        return a4_subsidiarysection

    def parse_b1_safetysign(self, path):
        shape_data = gpd.read_file(path)
        b1_safetysign = []
        for index, data in shape_data.iterrows():
            b1_safetysign.append(data)
        return b1_safetysign

    def parse_b2_surfacelinemark(self, path):
        shape_data = gpd.read_file(path)
        b2_surfacelinemark = []
        for index, data in shape_data.iterrows():
            b2_surfacelinemark.append(data)
        return b2_surfacelinemark

    def parse_b3_surfacemark(self, path):
        shape_data = gpd.read_file(path)
        b3_surfacemark = []
        for index, data in shape_data.iterrows():
            b3_surfacemark.append(data)
        return b3_surfacemark

    def parse_c1_trafficlight(self, path):
        shape_data = gpd.read_file(path)
        c1_trafficlight = []
        for index, data in shape_data.iterrows():
            c1_trafficlight.append(data)
        return c1_trafficlight

    def parse_c3_vehicleprotectionsafety(self, path):
        shape_data = gpd.read_file(path)
        c3_vehicleprotectionsafety = []
        for index, data in shape_data.iterrows():
            c3_vehicleprotectionsafety.append(data)
        return c3_vehicleprotectionsafety

    def parse_c4_speedbump(self, path):
        shape_data = gpd.read_file(path)
        c4_speedbump = []
        for index, data in shape_data.iterrows():
            c4_speedbump.append(data)
        return c4_speedbump

    def parse_c6_postpoint(self, path):
        shape_data = gpd.read_file(path)
        c6_postpoint = []
        for index, data in shape_data.iterrows():
            c6_postpoint.append(data)
        return c6_postpoint