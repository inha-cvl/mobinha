# NAVER LABS Road Layout Database

## Road Layout Database Format
Road Layout database of NAVER LABS HD map consists of multiple shape files.

The database has the following files
    A1_LANE_3D.{shx, shp, dbf, prj}
      - Information of road lanes
      - Type: Polyline

    A2_STOP_3D.{shx, shp, dbf, prj}
      - Information of stop lines
      - Type: Polyline

    A3_LINK_3D.{shx, shp, dbf, prj}
      - Information of the center line of road lanes
      - Type: Polyline

    C1_NODE_3D.{shx, shp, dbf, prj}
      - Information of the vertex connecting links
      - Type: Point

    B2_SURFSIGN_LINE_3D.{shx, shp, dbf, prj}
      - Information of road marker indicating guide line
      - Type: Polyline
    
    B2_SURFSIGN_DIRECTION_3D.{shx, shp, dbf, prj}
      - Information of road marker indicating driving direction sign and prohibitive sign
      - Type: Polygon

    B2_SURFSIGN_PLANE_3D.{shx, shp, dbf, prj}
      - Information of road marker indicating non-direction such as yield signs, crosswalks.
      - Type: Polygon

Please refer to the LABS_RoadLayout_Catalog.pdf for details.

## Coorindate System
UTM ZONE 52N Ellipsoid Height

## How to read the database
Go the following website and download QGIS program.
 - https://qgis.org
Open up the shape file(*.shp) in the QGIS program. 

## Release Notes
- Version 1.0.0 (2019.10.28): first release, pangyo and sangam included.
