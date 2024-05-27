#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <pcl/io/ply_io.h>

#include <nav_msgs/Path.h>
#include "../include/reflection_detection_lib.hpp"

#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>

#include <fstream>
#include <boost/filesystem.hpp>
using namespace std;
namespace fs = boost::filesystem;

struct Pose{
    double timestamp;
    Eigen::Vector3d pose;
    Eigen::Quaterniond orientation;
};

double getTimestampFromFileName(string file_name){
    size_t startPos = file_name.find_first_of("0123456789");
    size_t endPos = file_name.find_last_of("0123456789");
    string timestamp = file_name.substr(startPos, endPos - startPos + 1);
    return stod(timestamp);
}

void read_ply(string file_name, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, pcl::PointCloud<pcl::PointXYZINormal>::Ptr strong_cloud, pcl::PointCloud<pcl::PointXYZINormal>::Ptr lastcloud){
    ifstream file(file_name);
    if(!file.is_open()){
        cout << "Failed to open file: " << file_name << endl;
    }
    string line;
    while(std::getline(file, line)){
        if(line == "end_header"){
            break;
        }
    }
    while(getline(file, line)){
        double x,y,z,nx,ny,nz,intensity,ring,return_mode,label;
        if(sscanf(line.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &x, &y, &z, &nx, &ny, &nz, &intensity, &ring, &return_mode, &label )==10){
            pcl::PointXYZINormal p;
            p.x = x; p.y = y; p.z = z;
            p.intensity = intensity;
            p.normal_x = ring;
            p.normal_y = label;
            p.normal_z = int(round((180 + atan2(p.y, p.x) * 180 / PI) /0.6)) + 1;
            if(return_mode==1){
                strong_cloud->push_back(p);
            }else if(return_mode == 2){
                lastcloud->push_back(p);
            }else{
                cout<<"Other return mode: "<<return_mode<<endl;
            }
            cloud->push_back(p);
        }else{
            cout << "Failed to read line: " << line << endl;
        }
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "plane_mapping");
    ros::NodeHandle nh;
    ros::Publisher pubtrans = nh.advertise<sensor_msgs::PointCloud2>("cloud_transfered",30);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("glass_plane_marker", 10);
    ros::Publisher point_pub = nh.advertise<sensor_msgs::PointCloud2>("single_point", 10);
    ros::Publisher correct_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("Corrected_pointcloud", 10);
    
    ros::Rate loop_rate(2);
    string data_path = "/home/krasus/ShanghaiTech/Mars_Lab/Reflection/Reflection_ws/src/Reflection_detection/demo_data/3dRef_data/";
    string pose_file_path = "/home/krasus/ShanghaiTech/Mars_Lab/Reflection/Reflection_ws/src/Reflection_detection/demo_data/hesai_pose.txt";
    cout<<fs::current_path()<<endl;
    // Read point cloud file and pose file
    std::vector<std::pair<std::string, double>> fileTimestamp;
    for(const auto& entry : fs::directory_iterator(data_path)){
        if(fs::is_regular_file(entry.path()) && entry.path().extension() == ".ply"){
            fileTimestamp.emplace_back(entry.path().filename().string(), getTimestampFromFileName(entry.path().filename().string()));
        }
    }
    std::ifstream pose_file(pose_file_path);
    if(!pose_file.is_open()){
        std::cout<<"open pose file failed"<<std::endl;
        return -1;
    }

    //store pose for each frame
    std::string pose_line;
    std::vector<Eigen::Matrix4d> pose_vec;
    std::vector<Eigen::Affine3d > pose_trans_vec;
    std::vector<PlaneInfo> GlobalMapPlaneCoe = {};
    while(std::getline(pose_file, pose_line)){
        std::istringstream iss(pose_line);
        Pose cur_pose;
        iss >> cur_pose.timestamp >> cur_pose.pose.x() >> cur_pose.pose.y() >> cur_pose.pose.z() >> cur_pose.orientation.x() >> cur_pose.orientation.y() >> cur_pose.orientation.z() >> cur_pose.orientation.w();
        Eigen::Affine3d T_f = Eigen::Translation3d(cur_pose.pose) * cur_pose.orientation;
        pose_trans_vec.emplace_back(T_f);
    }

    sort(fileTimestamp.begin(), fileTimestamp.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
    loop_rate.sleep();
    loop_rate.sleep();
    
    int start_index = 0;
    int end_index = 0;

    start_index = 0;
    end_index = fileTimestamp.size();
    int frame_index = 0;

    std::vector<PlaneInfo> GlobalMapPlaneCoe_sel = {};

    
    for(int i=start_index;i<end_index;i++){
        auto p = fileTimestamp[i];
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan_strong(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan_last(new pcl::PointCloud<pcl::PointXYZINormal>);
        string filepath = (fs::path(data_path) / p.first).string();
        cout<<filepath<<endl;
        read_ply(filepath, scan_cloud, scan_strong, scan_last); 
        cout<<"pointnum:  "<<scan_cloud->size()<<endl;


        // transfer point cloud using pose
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr transfered_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::transformPointCloud(*scan_cloud, *transfered_cloud, pose_trans_vec[i]);
        frame_index++;
        sensor_msgs::PointCloud2::Ptr output_trans(new sensor_msgs::PointCloud2);
        output_trans->header.frame_id = "/map";
        pcl::toROSMsg(*transfered_cloud, *output_trans);
        output_trans->header.frame_id = "/map";
        pubtrans.publish(*output_trans);
        
        pcl::PointCloud<PointXYZ>::Ptr glass_pointcloud(new pcl::PointCloud<PointXYZ>);
        cloud_preprocess(scan_strong, scan_last, glass_pointcloud, correct_pc_pub);
        cout<<"glass_pointcloud size: "<<glass_pointcloud->points.size()<<endl;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr transfered_classify(new pcl::PointCloud<pcl::PointXYZINormal>);

        glassHandler(glass_pointcloud,pose_trans_vec[i], marker_pub, GlobalMapPlaneCoe);
        cout<<"frame_id "<<frame_index<<endl;
        ros::spinOnce();
        // loop_rate.sleep();
    }

    for(int i = 0; i < GlobalMapPlaneCoe.size(); i++){
        if(GlobalMapPlaneCoe[i].observedTime>=10){
            GlobalMapPlaneCoe_sel.push_back(GlobalMapPlaneCoe[i]);
        }
        // add_plane_maker(GlobalMapPlaneCoe[i].hull_points, marker_pub, i+1);
    }

    // 2nd round
    frame_index = 0;
    for(int i=start_index;i<fileTimestamp.size();i++){
        if(i>end_index){
            break;
        }

        auto p = fileTimestamp[i];

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan_strong(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan_last(new pcl::PointCloud<pcl::PointXYZINormal>);
        string filepath = (fs::path(data_path) / p.first).string();
        cout<<filepath<<endl;
        read_ply(filepath, scan_cloud, scan_strong, scan_last); 
        cout<<"pointnum:  "<<scan_cloud->size()<<endl;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr transfered_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr transfered_strong(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr transfered_last(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::transformPointCloud(*scan_cloud, *transfered_cloud, pose_trans_vec[i]);
        pcl::transformPointCloud(*scan_strong, *transfered_strong, pose_trans_vec[i]);
        pcl::transformPointCloud(*scan_last, *transfered_last, pose_trans_vec[i]);
        Eigen::Vector3d originpoint(0,0,0);
        Eigen::Vector3d trans_originpoint = pose_trans_vec[i]*originpoint;
        cout<<"pose_trans_vec[i]:  "<<pose_trans_vec[i].matrix()<<endl;
        cout<<"trans_originpoint:  "<<trans_originpoint.transpose()<<endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr origin_1_point(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ point_i;
        point_i.x = trans_originpoint[0];
        point_i.y = trans_originpoint[1];
        point_i.z = trans_originpoint[2];
        origin_1_point->points.push_back(point_i);
        sensor_msgs::PointCloud2::Ptr output_origin_1_point(new sensor_msgs::PointCloud2);
        output_origin_1_point->header.frame_id = "/map";
        pcl::toROSMsg(*origin_1_point, *output_origin_1_point);
        output_origin_1_point->header.frame_id = "/map";
        point_pub.publish(*output_origin_1_point);
        
        frame_index++;
        sensor_msgs::PointCloud2::Ptr output_trans(new sensor_msgs::PointCloud2);
        output_trans->header.frame_id = "/map";
        pcl::toROSMsg(*transfered_cloud, *output_trans);
        output_trans->header.frame_id = "/map";
        pubtrans.publish(*output_trans);

        pcl::PointCloud<PointXYZINormal>::Ptr glass_pointcloud(new pcl::PointCloud<PointXYZINormal>);
        cloud_preprocess_global(transfered_strong, transfered_last, GlobalMapPlaneCoe_sel, trans_originpoint, glass_pointcloud, correct_pc_pub);
        // cout<<"glass_pointcloud size: "<<glass_pointcloud->points.size()<<endl;
        cout<<"frame_id "<<i<<endl;

        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
}
