from ngiiParser import NGIIParser
from tqdm import tqdm

class NGII2LANELET:
    def __init__(self, folder_path):
        a1_path = '%s/A1_NODE.shp' % folder_path
        a2_path = '%s/A2_LINK.shp' % folder_path
        a3_path = '%s/A3_DRIVEWAYSECTION.shp' % folder_path
        b2_path = '%s/B2_SURFACELINEMARK.shp' % folder_path
        b3_path = '%s/B3_SURFACEMARK.shp' % folder_path
        c1_path = '%s/C1_TRAFFICLIGHT.shp' % folder_path
        self.ngii = NGIIParser(
            a1_path,
            a2_path,
            a3_path,
            b2_path, 
            b3_path,
            c1_path)

    def check_data(self, data_attr):
        data_to_check = getattr(self.ngii, data_attr, None)
        if data_to_check is None:
            print(f"No such attribute '{data_attr}' in ngii object.")
            return

        for n, info in tqdm(enumerate(data_to_check), desc=f"Processing {data_attr}: ", total=len(data_to_check)):
            print(info["geometry"].exterior)

if __name__ == "__main__":
    map_name = "KCity"
    data_attr = "a3_drivewaysection"
    
    lanelet = NGII2LANELET('./%s' % map_name)
    lanelet.check_data(data_attr)

