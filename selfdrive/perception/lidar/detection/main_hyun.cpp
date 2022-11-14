#include "utility.h"
#include "math.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "process/processPointClouds.h"
#include "lib/processPointClouds.cpp"
#include "marker/customMarker.h"
#include "tracking/tracker.h"

#define PI 3.14159265359

class Process{

private:
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud;
    ros::Subscriber subPathpoint;
    ros::Subscriber subPillarbox;

    ros::Publisher pubGroundRemovedCloud;
    ros::Publisher pub_cluster_box;
    ros::Publisher pub_track_box;
    ros::Publisher pub_track_text;
    ros::Publisher pub_track_pillar_box;
    ros::Publisher pub_track_pillar_text;
    ros::Publisher pub_far_object_marker;
    ros::Publisher pub_path_marker;

    ros::Publisher pubFrontCameraFOVCloud;
    ros::Publisher pubBackCameraFOVCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr frontCameraFOVCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr backCameraFOVCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn;
    pcl::PointCloud<pcl::PointXYZI>::Ptr fullCloud; // projected ouster raw cloud, but saved in the form of 1-D matrix

    pcl::PointCloud<pcl::PointXYZI>::Ptr groundRemovedCloud;

    visualization_msgs::Marker far_object_mark;
    jsk_recognition_msgs::BoundingBoxArray cluster_bbox_array;
    jsk_recognition_msgs::BoundingBoxArray track_bbox_array;
    visualization_msgs::MarkerArray track_text_array;
    // visualization_msgs::MarkerArray path_marker_array;
    jsk_recognition_msgs::BoundingBoxArray track_pillarbox_array;
    visualization_msgs::MarkerArray track_pillartext_array;

    pcl::PointXYZI nanPoint; // fill in fullCloud at each iteration
    
    std_msgs::Header cloudHeader;

    std_msgs::Float32MultiArray pathpoint_msg;
    jsk_recognition_msgs::BoundingBoxArray pillarhandler_msg;

    bool pillarhandler_flag = false;
    bool pointcloud_convert_flag = false;
    bool crossroad_flag = false;
    float crop_y_max = 20;
    float crop_y_min = -20;
    float crop_y_center = 0.0;
    float road_width_max = 8.5;
    float road_width_min = 8.5;

    int far_object_detected[4] = {0, 0, 0, 0};
	std::vector<float> pose;

    CustomMarker customMarker;
    Tracker tracker;
    Tracker tracker_fusion;
    Tracker tracker_pillar;

public:
    Process():
        nh("~"){

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1, &Process::cloudHandler, this);
        subPillarbox = nh.subscribe<jsk_recognition_msgs::BoundingBoxArray>("/pillar_box_array", 1, &Process::pillarHandler, this);
        subPathpoint = nh.subscribe<std_msgs::Float32MultiArray>("/crop", 1, &Process::cropHandler, this);
        nh.subscribe<std_msgs::Float32MultiArray>("/odom", 1, &Process::odomCB, this);
        pubGroundRemovedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/roi_cloud", 1);
        pub_cluster_box = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("lidar/cluster_box", 1);
        pub_track_box = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("lidar/track_box", 1);
        pub_track_text = nh.advertise<visualization_msgs::MarkerArray>("lidar/track_text", 1);
        pub_track_pillar_box = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("lidar/track_pillar_box", 1);
        pub_track_pillar_text = nh.advertise<visualization_msgs::MarkerArray>("lidar/track_pillar_text", 1);
        pub_far_object_marker = nh.advertise<visualization_msgs::Marker>("lidar/far_obejct_marker", 1);
        pub_path_marker = nh.advertise<visualization_msgs::MarkerArray>("/path_marker", 1);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

    ~Process(){}

    void allocateMemory(){
        laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
        fullCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        groundRemovedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
    }

    void resetParameters(){
        laserCloudIn->clear();
        groundRemovedCloud->clear();
        // path_marker_array.markers.clear();
        pathpoint_msg.data.clear();
        cluster_bbox_array.boxes.clear();
        track_bbox_array.boxes.clear();
        track_text_array.markers.clear();
        pillarhandler_msg.boxes.clear();
        track_pillarbox_array.boxes.clear();
        track_pillartext_array.markers.clear();

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
    }
    void cloudHandler(const sensor_msgs::PointCloud2 laserCloudMsg){
        clock_t astart = clock();
        copyPointCloud(laserCloudMsg);
        projectPointCloud();
        groundRemoval();
        cloudPublish();
        farObject();
        traking();
        publishResult();
        resetParameters();
        printf("%f s-all\n", (float)(clock() - astart) / CLOCKS_PER_SEC);
    }

    void cropHandler(std_msgs::Float32MultiArray msg){
        pointcloud_convert_flag = true;
        pathpoint_msg = msg;
        int num = msg.data.size()/2;
        int path_id = 0;

        //for visualization
        // for(int i = 0; i < num; i++){
        //     int x = msg.data[0+2*i];
        //     int y = msg.data[1+2*i];
        //     //location marker
        //     visualization_msgs::Marker path_mark;
        //     path_mark.header.frame_id = "os_sensor";
        //     path_mark.header.stamp = ros::Time::now();
        //     path_mark.type = visualization_msgs::Marker::SPHERE;
        //     path_mark.id = path_id;
        //     path_mark.action = visualization_msgs::Marker::ADD;
        //     path_mark.lifetime = ros::Duration(0.3);
        //     path_mark.pose.orientation.w = 1.0;
        //     path_mark.pose.position.x = x;
        //     path_mark.pose.position.y = y;
        //     path_mark.color.g = 0.5;
        //     path_mark.color.a = 1.0;
        //     path_mark.scale.x = 1.0;
        //     path_mark.scale.y = 1.0;
        //     path_marker_array.markers.push_back(path_mark);
        //     ++path_id;
        // }

    }

    void pillarHandler(jsk_recognition_msgs::BoundingBoxArray msg){

        pillarhandler_msg = msg;
        pillarhandler_flag = true;
    }

    void copyPointCloud(const sensor_msgs::PointCloud2 laserCloudMsg){
        cloudHeader = laserCloudMsg.header;
        cloudHeader.stamp = ros::Time::now();
        pcl::fromROSMsg(laserCloudMsg, *laserCloudIn);
    }

    void projectPointCloud(){
        fullCloud = laserCloudIn;
    }

    void groundRemoval(){
        // clock_t groundtime = clock();
        size_t lowerInd;
        
        far_object_detected[0] = 0;
        far_object_detected[1] = 0;
        far_object_detected[2] = 0;
        far_object_detected[3] = 0;

        int pathpoint_num = pathpoint_msg.data.size()/2;
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            for (size_t i = 0; i < N_SCAN; ++i){
                lowerInd = j + (i)*Horizon_SCAN;

                //pre filtering
                if (isnan(fullCloud->points[lowerInd].x) || isnan(fullCloud->points[lowerInd].y) || isnan(fullCloud->points[lowerInd].z) ||
                    isinf(fullCloud->points[lowerInd].x) || isinf(fullCloud->points[lowerInd].y) || isinf(fullCloud->points[lowerInd].z)){
                    continue;
                }
                
                //for far objects
                if (fullCloud->points[lowerInd].x > 37 && fullCloud->points[lowerInd].x < 40 && fullCloud->points[lowerInd].y > -1 && fullCloud->points[lowerInd].y < 1 && fullCloud->points[lowerInd].z < 0){
                    far_object_detected[0]++;
                }
                else if (fullCloud->points[lowerInd].x > 40 && fullCloud->points[lowerInd].x < 45 && fullCloud->points[lowerInd].y > -1 && fullCloud->points[lowerInd].y < 1 && fullCloud->points[lowerInd].z < 0){
                    far_object_detected[1]++;
                }
                else if (fullCloud->points[lowerInd].x > 45 && fullCloud->points[lowerInd].x < 50 && fullCloud->points[lowerInd].y > -1 && fullCloud->points[lowerInd].y < 1 && fullCloud->points[lowerInd].z < 0){
                    far_object_detected[2]++;
                }
                else if (fullCloud->points[lowerInd].x > 50 && fullCloud->points[lowerInd].x < 55 && fullCloud->points[lowerInd].y > -1 && fullCloud->points[lowerInd].y < 1 && fullCloud->points[lowerInd].z < 0){
                    far_object_detected[3]++;
                }

                if (fullCloud->points[lowerInd].x < -25 || fullCloud->points[lowerInd].x > 45 || fullCloud->points[lowerInd].x == -0 || fullCloud->points[lowerInd].y == -0 || fullCloud->points[lowerInd].z > 0.5 || fullCloud->points[lowerInd].z < -3){
                    continue;
                }

                if (pathpoint_num != 0 and (pathpoint_msg.data[0] - 0 <= 0 or pathpoint_msg.data[2] - pathpoint_msg.data[0] <= 0 or pathpoint_msg.data[4] - pathpoint_msg.data[2] <= 0 or pathpoint_msg.data[6] - pathpoint_msg.data[4] <= 0 or pathpoint_msg.data[8] - pathpoint_msg.data[6] <= 0)){
                    crossroad_flag = true;
                }

                if (crossroad_flag){
                    crop_y_max = 15;
                    crop_y_min = -15;
                }
                else if (pointcloud_convert_flag && fullCloud->points[lowerInd].x > 0){
                    if (fullCloud->points[lowerInd].x > 0 && fullCloud->points[lowerInd].x < pathpoint_msg.data[0]){
                        crop_y_center = ((pathpoint_msg.data[1] - 0)/(pathpoint_msg.data[0] - 0)) * (fullCloud->points[lowerInd].x - 0) + 0;
                        crop_y_max = crop_y_center + road_width_max;
                        crop_y_min = crop_y_center - road_width_min;
                    }
                    else if (fullCloud->points[lowerInd].x >= pathpoint_msg.data[0] && fullCloud->points[lowerInd].x < pathpoint_msg.data[2]){
                        crop_y_center = ((pathpoint_msg.data[3] - pathpoint_msg.data[1])/(pathpoint_msg.data[2] - pathpoint_msg.data[0])) * (fullCloud->points[lowerInd].x - pathpoint_msg.data[0]) + pathpoint_msg.data[1];
                        crop_y_max = crop_y_center + road_width_max;
                        crop_y_min = crop_y_center - road_width_min;
                    }
                    else if (fullCloud->points[lowerInd].x >= pathpoint_msg.data[2] && fullCloud->points[lowerInd].x < pathpoint_msg.data[4]){
                        crop_y_center = ((pathpoint_msg.data[5] - pathpoint_msg.data[3])/(pathpoint_msg.data[4] - pathpoint_msg.data[2])) * (fullCloud->points[lowerInd].x - pathpoint_msg.data[2]) + pathpoint_msg.data[3];
                        crop_y_max = crop_y_center + road_width_max;
                        crop_y_min = crop_y_center - road_width_min;
                    }
                    else if (fullCloud->points[lowerInd].x >= pathpoint_msg.data[4] && fullCloud->points[lowerInd].x < pathpoint_msg.data[6]){
                        crop_y_center = ((pathpoint_msg.data[7] - pathpoint_msg.data[5])/(pathpoint_msg.data[6] - pathpoint_msg.data[4])) * (fullCloud->points[lowerInd].x - pathpoint_msg.data[4]) + pathpoint_msg.data[5];
                        crop_y_max = crop_y_center + road_width_max;
                        crop_y_min = crop_y_center - road_width_min;
                    }
                    else if (fullCloud->points[lowerInd].x >= pathpoint_msg.data[6]){
                        crop_y_center = ((pathpoint_msg.data[9] - pathpoint_msg.data[7])/(pathpoint_msg.data[8] - pathpoint_msg.data[6])) * (fullCloud->points[lowerInd].x - pathpoint_msg.data[6]) + pathpoint_msg.data[7];
                        crop_y_max = crop_y_center + road_width_max;
                        crop_y_min = crop_y_center - road_width_min;
                    }
                }
                //back
                else if(fullCloud->points[lowerInd].x < 0){
                    crop_y_max = 5;
                    crop_y_min = -5;
                }
                else{
                    crop_y_max = 15;
                    crop_y_min = -15;
                }


                if (fullCloud->points[lowerInd].y <= crop_y_max && fullCloud->points[lowerInd].y >= crop_y_min){
                    //not fusion
                    groundRemovedCloud->points.push_back(fullCloud->points[lowerInd]);
                }

                crossroad_flag = false;

            }
        }

    }
    void cloudPublish(){
        sensor_msgs::PointCloud2 laserCloudTemp;
        // ground removed cloud
        if (pubGroundRemovedCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundRemovedCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "os_sensor";
            pubGroundRemovedCloud.publish(laserCloudTemp);
        }
    }

	double ToRadian(float degree){
		return (degree * (PI/180));
	}

	void odomCB(std_msgs::Float32MultiArray msg){
		pose = msg.data;
	}

    void farObject(){
        float far_object_x = 200;           
        float theta = ToRadian(pose[2]);

        if(far_object_detected[0] >= 7){
            far_object_x = 36;
        }
        else if(far_object_detected[1] >= 5){
            far_object_x = 42.5;
        }
        else if(far_object_detected[2] >= 5){
            far_object_x = 47.5;
        }
        else if(far_object_detected[3] >= 5){
            far_object_x = 52.5;
        }

        if(far_object_x != 200){
            far_object_mark.header.frame_id = "world";
            far_object_mark.header.stamp = ros::Time::now();
            far_object_mark.type = visualization_msgs::Marker::SPHERE;
            far_object_mark.id = 0;
            far_object_mark.action = visualization_msgs::Marker::ADD;
            far_object_mark.lifetime = ros::Duration(0.3);
            far_object_mark.pose.orientation.w = 1.0;
            far_object_mark.pose.position.x = far_object_x * cos(theta) + pose[0];
            far_object_mark.pose.position.y = pose[1];
            far_object_mark.color.g = 0.5;
            far_object_mark.color.a = 1.0;
            far_object_mark.scale.x = 1.0;
            far_object_mark.scale.y = 1.0;
            far_object_mark.scale.z = 1.0;
        }
    }

    void traking(){
        // jsk_recognition_msgs::BoundingBoxArray filtered_bbox_array = tracker.filtering(cluster_bbox_array);
        // tracker.predictNewLocationOfTracks();
        // tracker.assignDetectionsTracks(filtered_bbox_array);
        // tracker.assignedTracksUpdate(filtered_bbox_array);
        // tracker.unassignedTracksUpdate();
        // tracker.deleteLostTracks();
        // tracker.createNewTracks(filtered_bbox_array);
        // pair<jsk_recognition_msgs::BoundingBoxArray, visualization_msgs::MarkerArray> bbox_text = tracker.displayTrack();
        // track_bbox_array = bbox_text.first;
        // track_text_array = bbox_text.second;

        jsk_recognition_msgs::BoundingBoxArray filtered_pillarbox_array = tracker_pillar.filtering(pillarhandler_msg);
        tracker_pillar.predictNewLocationOfTracks();
        tracker_pillar.assignDetectionsTracks(filtered_pillarbox_array);
        tracker_pillar.assignedTracksUpdate(filtered_pillarbox_array);
        tracker_pillar.unassignedTracksUpdate();
        tracker_pillar.deleteLostTracks();
        tracker_pillar.createNewTracks(filtered_pillarbox_array);
        pair<jsk_recognition_msgs::BoundingBoxArray, visualization_msgs::MarkerArray> pillarbox_text = tracker_pillar.displayTrack();
        track_pillarbox_array = pillarbox_text.first;
        track_pillartext_array = pillarbox_text.second;

    }


    void publishResult(){
        // Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;

        // ground removed cloud
        // sensor_msgs::PointCloud2 laserCloudTemp;
        // if (pubGroundRemovedCloud.getNumSubscribers() != 0){
        //     pcl::toROSMsg(*groundRemovedCloud, laserCloudTemp);
        //     laserCloudTemp.header.stamp = cloudHeader.stamp;
        //     laserCloudTemp.header.frame_id = "os_sensor";
        //     pubGroundRemovedCloud.publish(laserCloudTemp);
        // }

        //clster_box array
        // cluster_bbox_array.header.stamp = ros::Time::now();
        // cluster_bbox_array.header.frame_id = "os_sensor";
        // pub_cluster_box.publish(cluster_bbox_array);

        //track_box array
        // track_bbox_array.header.stamp = ros::Time::now();
        // track_bbox_array.header.frame_id = "os_sensor";
        // pub_track_box.publish(track_bbox_array);

        //track_text array
        // pub_track_text.publish(track_text_array);


        //far object marker array
        pub_far_object_marker.publish(far_object_mark);

        //track_box array
        track_pillarbox_array.header.stamp = ros::Time::now();
        track_pillarbox_array.header.frame_id = "os_sensor";
        pub_track_pillar_box.publish(track_pillarbox_array);

        //track_text array
        pub_track_pillar_text.publish(track_pillartext_array);

        //path marker publish
        // pub_path_marker.publish(path_marker_array);
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "lidar");
    Process P;

    ros::spin();
    return 0;
}
