import geopandas as gpd

gdf = gpd.read_file("./KCity/B1_SAFETYSIGN.shp")

print(gdf.head())

print(gdf.columns)
