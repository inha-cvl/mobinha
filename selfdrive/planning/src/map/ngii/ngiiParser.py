import geopandas as gpd

from .a2_link import A2_LINK
from .b1_safetysign import B1_SAFETYSIGN
from .b2_surfacelinemark import B2_SURFACELINEMARK
from .b3_surfacemark import B3_SURFACEMARK
from .c1_trafficlight import C1_TRAFFICLIGHT


# NGII Parser for 2019 version
class NGIIParser:
    def __init__(self,
        a2_path,
        b1_path,
        b2_path,
        b3_path,
        c1_path):

        self.a2_link = self.parse_a2_link(a2_path)
        self.b1_safetysign = self.parse_b1_safetysign(b1_path)
        self.b2_surfacelinemark = self.parse_b2_surfacelinemark(b2_path)
        self.b3_surfacemark = self.parse_b3_surfacemark(b3_path)
        self.c1_trafficlight = self.parse_c1_trafficlight(c1_path)

    def parse_a2_link(self, path):
        shape_data = gpd.read_file(path)
        a2_link = []
        for index, data in shape_data.iterrows():
            a2_link.append(A2_LINK(data))
        return a2_link

    def parse_b1_safetysign(self, path):
        shape_data = gpd.read_file(path)
        b1_safetysign = []
        for index, data in shape_data.iterrows():
            b1_safetysign.append(B1_SAFETYSIGN(data))
        return b1_safetysign

    def parse_b2_surfacelinemark(self, path):
        shape_data = gpd.read_file(path)
        b2_surfacelinemark = []
        for index, data in shape_data.iterrows():
            b2_surfacelinemark.append(B2_SURFACELINEMARK(data))
        return b2_surfacelinemark

    def parse_b3_surfacemark(self, path):
        shape_data = gpd.read_file(path)
        b3_surfacemark = []
        for index, data in shape_data.iterrows():
            b3_surfacemark.append(B3_SURFACEMARK(data))
        return b3_surfacemark

    def parse_c1_trafficlight(self, path):
        shape_data = gpd.read_file(path)
        c1_trafficlight = []
        for index, data in shape_data.iterrows():
            c1_trafficlight.append(C1_TRAFFICLIGHT(data))
        return c1_trafficlight