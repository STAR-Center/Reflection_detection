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
#include "plane_slam/PlanesMsg.h"
// #include "sophus/se3.hpp"
// #include "../include/DataStructure.hpp"
#include "../include/Reflect_detect_global.hpp"


// #include <plc/fea>pcaCentroid

#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>

#include <fstream>
// #include <filesystem>
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
    // cloud.reset();
    std::vector<int> lable_num_strong = std::vector<int>(7,0);
    std::vector<int> lable_num_last = std::vector<int>(7,0);
    ifstream file(file_name);
    if(!file.is_open()){
        cout << "Failed to open file: " << file_name << endl;
    }
    // cout<<"read_ply"<<endl;
    string line;
    while(std::getline(file, line)){
        if(line == "end_header"){
            break;
        }
        // cout<<line<<endl;
    }
    // cout<<file_name<<endl;
    double max_ring = -1;
    double max_lable = -1;
    while(getline(file, line)){
        // cout << line << endl;
        double x,y,z,nx,ny,nz,intensity,ring,return_mode,label;
        if(sscanf(line.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &x, &y, &z, &nx, &ny, &nz, &intensity, &ring, &return_mode, &label )==10){
            // cout << x << " " << y << " " << z << " " <<std::setprecision(20) <<nx << " " << ny << " " << nz << " " << intensity << " " << ring << " " << return_mode << " " << label << endl;
            // cout << x << " " << y << " " << z << " "<<endl;
            pcl::PointXYZINormal p;
            pcl::PointXYZ ps;
            pcl::PointXYZ pl;
            max_ring = max(max_ring,ring);
            max_lable = max(max_lable,label);
            p.x = x; p.y = y; p.z = z;
            // cloud->push_back(p);
            if(return_mode==1){
                p.intensity = intensity;
                p.normal_x = ring;
                p.normal_y = label;
                p.normal_z = int(round((180 + atan2(p.y, p.x) * 180 / PI) /0.6)) + 1;
                strong_cloud->push_back(p);
                if(p.normal_y!=0){
                    if(p.normal_y==2){
                        p.normal_y=3;
                    }
                    cloud->push_back(p);
                }
                
                lable_num_strong[(int)label]+=1;
            }else if(return_mode == 2){
                p.intensity = intensity;
                p.normal_x = ring;
                p.normal_y = label;
                p.normal_z = int(round((180 + atan2(p.y, p.x) * 180 / PI) /0.6)) + 1;
                lastcloud->push_back(p);
                if(p.normal_y!=0){
                    if(p.normal_y==2){
                        p.normal_y=3;
                    }
                    cloud->push_back(p);
                }
                lable_num_last[(int)label]+=1;
            }else{
                cout<<"Other return mode: "<<return_mode<<endl;
            }
            
            // cloud->po
            // lable_num[(int)label]+=1;

        }else{
            cout << "Failed to read line: " << line << endl;
        }
        // scan
        // cout<<sscanf(line.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &x, &y, &z, &nx, &ny, &nz, &intensity, &ring, &return_mode, &label )<<endl;
        // cout<<"read line done"<<endl;
    }
    // cout<<"strong lable:"<<endl;
    // for(int i=0;i<lable_num_strong.size();i++){
    //     cout<<"lable "<<i<<": "<<lable_num_strong[i]<<endl;
    // }

    // cout<<"last lable:"<<endl;
    // for(int i=0;i<lable_num_last.size();i++){
    //     cout<<"lable "<<i<<": "<<lable_num_last[i]<<endl;
    // }
    // cout<<max_ring<<" max ring"<<endl;
    // cout<<max_lable<<" max label"<<endl;
    // cout<<"read ply file done"<<endl;

}


int main(int argc, char** argv){
    ros::init(argc, argv, "plane_mapping");
    ros::NodeHandle nh;
    ros::Publisher pub =  nh.advertise<sensor_msgs::PointCloud2>("cloud_from_file",10);
    ros::Publisher pubtrans = nh.advertise<sensor_msgs::PointCloud2>("cloud_transfered",30);


    ros::Publisher pubstrong =  nh.advertise<sensor_msgs::PointCloud2>("cloud_strong_from_file",10);
    ros::Publisher publast =  nh.advertise<sensor_msgs::PointCloud2>("cloud_last_from_file",10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("glass_plane_marker", 10);
    ros::Publisher point_pub = nh.advertise<sensor_msgs::PointCloud2>("single_point", 10);
    ros::Publisher in_area_pub = nh.advertise<sensor_msgs::PointCloud2>("in_area_point", 10);

    ros::Publisher correct_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("Corrected_pointcloud", 10);
    
    ros::Rate loop_rate(2);
    string data_path = "/home/krasus/ShanghaiTech/Mars_Lab/Reflection/Dataset/NewData/sca/hesai/";
    std::vector<std::pair<std::string, double>> fileTimestamp;
    for(const auto& entry : fs::directory_iterator(data_path)){
        if(fs::is_regular_file(entry.path()) && entry.path().extension() == ".ply"){
            fileTimestamp.emplace_back(entry.path().filename().string(), getTimestampFromFileName(entry.path().filename().string()));
        }
    }
    string pose_file_path = "/home/krasus/ShanghaiTech/Mars_Lab/Reflection/Dataset/NewData/sca/hesai_pose.txt";

    std::ifstream pose_file(pose_file_path);
    if(!pose_file.is_open()){
        std::cout<<"open pose file failed"<<std::endl;
        return -1;
    }
    std::string pose_line;
    std::vector<Eigen::Matrix4d> pose_vec;
    std::vector<Eigen::Affine3d > pose_trans_vec;
    std::vector<PlaneInfo> GlobalMapPlaneCoe = {};
    std::vector<Pose> pose_vec_pose ={};
    std::vector<double> time_stam = {};
    // int indexxx = 0;
    while(std::getline(pose_file, pose_line)){
        // cout<<pose_line<<endl;
        std::istringstream iss(pose_line);
        Pose cur_pose;
        iss >> cur_pose.timestamp >> cur_pose.pose.x() >> cur_pose.pose.y() >> cur_pose.pose.z() >> cur_pose.orientation.x() >> cur_pose.orientation.y() >> cur_pose.orientation.z() >> cur_pose.orientation.w();
        time_stam.push_back(cur_pose.timestamp);
        pose_vec_pose.push_back(cur_pose);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = cur_pose.orientation.toRotationMatrix();
        T.block<3,1>(0,3) = cur_pose.pose;
        pose_vec.emplace_back(T);
        Eigen::Affine3d T_f = Eigen::Translation3d(cur_pose.pose) * cur_pose.orientation;
        // cout<<indexxx<<endl;
        // cout<<T_f.matrix()<<endl;
        pose_trans_vec.emplace_back(T_f);
        // indexxx++;
        // if(indexxx == 30){
        //     return 0;
        // }
    }

    // std::ofstream outfile("/home/krasus/ShanghaiTech/Mars_Lab/Reflection/Dataset/NewData/h2/hesai_pose_TT.txt", std::ios::app);
    // if(outfile.is_open()){
    //     for(int i=0;i<pose_trans_vec.size();i++){
    //         outfile<<"pose "<<i<<endl;
    //         outfile<<pose_trans_vec[i].matrix()*pose_trans_vec[0].matrix().inverse()<<endl<<endl;
    //     }
    // }
    // outfile.close();
    cout<<"pose writen"<<endl;
    // return 0;

    sort(fileTimestamp.begin(), fileTimestamp.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
    loop_rate.sleep();
    loop_rate.sleep();
    loop_rate.sleep();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    int start_index = 2132;
    int end_index = 2133;

    start_index = 0;
    end_index = 1000;
    // end_index = fileTimestamp.size();
    int frame_index = 0;
    // for(int i=0;i<fileTimestamp.size();i++){
    // for(int i=0;i<fileTimestamp.size();i++){
    //     if(fileTimestamp[i].second!=time_stam[i]){
    //         cout<<"i "<<i<<" "<<fileTimestamp[i].second<<" "<<time_stam[i]<<endl;
    //         return 0;
    //     }
    // }
    // return 0;
    pcl::PointCloud<pcl::PointXYZINormal> all_pointcloud;
    pcl::PointCloud<pcl::PointXYZINormal> all_pointcloud_single_classify;

    std::vector<PlaneInfo> GlobalMapPlaneCoe_sel = {};

    
    for(int i=start_index;i<end_index;i++){
        auto p = fileTimestamp[i];
        // if(i<=317 && i>=317){
            
        // }else{
        //     frame_index+=1;
        //     continue;
        // }
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan_strong(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan_last(new pcl::PointCloud<pcl::PointXYZINormal>);
        // cout << p.first << " " << p.second << endl;
        string filepath = (fs::path(data_path) / p.first).string();
        cout<<filepath<<endl;
        // ifstream ifs(filepath);
        read_ply(filepath, scan_cloud, scan_strong, scan_last); 
        cout<<"pointnum:  "<<scan_cloud->size()<<endl;
        // sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
        // output->header.frame_id = "/map";
        // pcl::toROSMsg(*scan_cloud, *output);
        // // cout<<output->header.stamp<<endl;
        // output->header.frame_id = "/map";
        // pub.publish(*output);

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr transfered_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::transformPointCloud(*scan_cloud, *transfered_cloud, pose_trans_vec[i]);
        frame_index++;
        sensor_msgs::PointCloud2::Ptr output_trans(new sensor_msgs::PointCloud2);
        output_trans->header.frame_id = "/map";
        pcl::toROSMsg(*transfered_cloud, *output_trans);
        // cout<<output->header.stamp<<endl;
        output_trans->header.frame_id = "/map";
        pubtrans.publish(*output_trans);
        // if(i%20==0){
        all_pointcloud+= *transfered_cloud;
        // }
        

        // sensor_msgs::PointCloud2::Ptr output_strong(new sensor_msgs::PointCloud2);
        // output_strong->header.frame_id = "/map";
        // pcl::toROSMsg(*scan_strong, *output_strong);
        // // cout<<output->header.stamp<<endl;
        // output_strong->header.frame_id = "/map";
        // pubstrong.publish(*output_strong);

        // sensor_msgs::PointCloud2::Ptr output_last(new sensor_msgs::PointCloud2);
        // output_last->header.frame_id = "/map";
        // pcl::toROSMsg(*scan_last, *output_last);
        // // cout<<output->header.stamp<<endl;
        // output_last->header.frame_id = "/map";
        // publast.publish(*output_last);
        // break;
        pcl::PointCloud<PointXYZ>::Ptr glass_pointcloud(new pcl::PointCloud<PointXYZ>);
        pcl::PointCloud<PointXYZINormal>::Ptr classify_result(new pcl::PointCloud<PointXYZINormal>);
        cloud_preprocess(scan_strong, scan_last, glass_pointcloud, correct_pc_pub, classify_result);
        cout<<"glass_pointcloud size: "<<glass_pointcloud->points.size()<<endl;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr transfered_classify(new pcl::PointCloud<pcl::PointXYZINormal>);
        // pcl::transformPointCloud(*classify_result, *transfered_classify, pose_trans_vec[i]);
        // if(i%20==0){
        all_pointcloud_single_classify+= *classify_result;
        // }

        glassHandler(glass_pointcloud,pose_trans_vec[i], marker_pub, GlobalMapPlaneCoe);
        cout<<"frame_id "<<frame_index<<endl;
        ros::spinOnce();
        // loop_rate.sleep();
    }
    // // return 0;
    // // pcl::io::savePCDFileASCII("/home/krasus/ShanghaiTech/Mars_Lab/Reflection/Dataset/NewData/sca/hesai_all_pointcloud.pcd", all_pointcloud);
    // // pcl::io::savePLYFile("/home/krasus/ShanghaiTech/Mars_Lab/Reflection/Dataset/NewData/sca/hesai_all_pointcloud.ply", all_pointcloud);
    pcl::PLYWriter writer;
    // writer.write("/home/krasus/ShanghaiTech/Mars_Lab/Reflection/Dataset/NewData/sist/hesai_all_pointcloud_900.ply", all_pointcloud);
    // writer.write("/home/krasus/ShanghaiTech/Mars_Lab/Reflection/Dataset/NewData/sist/hesai_all_pointcloud_900_classify.ply", all_pointcloud_single_classify);
    // writer.write("/home/krasus/ShanghaiTech/Mars_Lab/Reflection/Dataset/NewData/sist/glassresult.ply", all_pointcloud_single_classify);
    // return 0;


    for(int i = 0; i < GlobalMapPlaneCoe.size(); i++){
        if(GlobalMapPlaneCoe[i].observedTime>=5){
            cout<<"hull points size: "<<GlobalMapPlaneCoe[i].hull_points->size()<<"obverse time: "<<GlobalMapPlaneCoe[i].observedTime<<endl;
            cout<<GlobalMapPlaneCoe[i].coe[0]<<" "<<GlobalMapPlaneCoe[i].coe[1]<<" "<<GlobalMapPlaneCoe[i].coe[2]<<" "<<GlobalMapPlaneCoe[i].coe[3]<<endl;
            for(int j = 0; j < GlobalMapPlaneCoe[i].hull_points->size(); j++){
                cout<<"{"<<GlobalMapPlaneCoe[i].hull_points->points[j].x<<", "<<GlobalMapPlaneCoe[i].hull_points->points[j].y<<", "<<GlobalMapPlaneCoe[i].hull_points->points[j].z<<"},"<<endl;
            }
            // add_plane_maker(GlobalMapPlaneCoe[i].hull_points, marker_pub, i+1);
        }
        // cout<<"hull points size: "<<GlobalMapPlaneCoe[i].hull_points->size()<<"obverse time: "<<GlobalMapPlaneCoe[i].observedTime<<endl;
        // add_plane_maker(GlobalMapPlaneCoe[i].hull_points, marker_pub, i+1);
    }
    // return 0;

    for(int i = 0; i < GlobalMapPlaneCoe.size(); i++){
        if(GlobalMapPlaneCoe[i].observedTime>=10){
            GlobalMapPlaneCoe_sel.push_back(GlobalMapPlaneCoe[i]);
        }
        // cout<<"hull points size: "<<GlobalMapPlaneCoe[i].hull_points->size()<<"obverse time: "<<GlobalMapPlaneCoe[i].observedTime<<endl;
        // add_plane_maker(GlobalMapPlaneCoe[i].hull_points, marker_pub, i+1);
    }

    // 2nd round
    frame_index = 0;
    for(int i=start_index;i<fileTimestamp.size();i++){
        if(i>end_index){
            break;
        }
        // if(i>=900 && i<=1800){
        //     continue;
        // }
        auto p = fileTimestamp[i];
        // if(i<=317 && i>=317){
            
        // }else{
        //     frame_index+=1;
        //     continue;
        // }
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan_strong(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan_last(new pcl::PointCloud<pcl::PointXYZINormal>);
        // cout << p.first << " " << p.second << endl;
        string filepath = (fs::path(data_path) / p.first).string();
        cout<<filepath<<endl;
        // ifstream ifs(filepath);
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
        // point_i.header.frame_id = "/map";
        
        frame_index++;
        sensor_msgs::PointCloud2::Ptr output_trans(new sensor_msgs::PointCloud2);
        output_trans->header.frame_id = "/map";
        pcl::toROSMsg(*transfered_cloud, *output_trans);
        // cout<<output->header.stamp<<endl;
        output_trans->header.frame_id = "/map";
        pubtrans.publish(*output_trans);

        pcl::PointCloud<PointXYZINormal>::Ptr glass_pointcloud(new pcl::PointCloud<PointXYZINormal>);
        cloud_preprocess_global(transfered_strong, transfered_last, GlobalMapPlaneCoe_sel, trans_originpoint, glass_pointcloud, correct_pc_pub);
        cout<<"glass_pointcloud size: "<<glass_pointcloud->points.size()<<endl;
        cout<<"GlobalMapPlaneCoe_sel size: "<<GlobalMapPlaneCoe_sel.size()<<endl;
        if(i%20==0){
            all_pointcloud+= *glass_pointcloud;
        }
        // if(i==317){
            
        // }
        // glassHandler(glass_pointcloud,pose_trans_vec[i], marker_pub, GlobalMapPlaneCoe);
        sensor_msgs::PointCloud2::Ptr output_in_area(new sensor_msgs::PointCloud2);
        output_in_area->header.frame_id = "/map";
        pcl::toROSMsg(*glass_pointcloud, *output_in_area);
        // cout<<output->header.stamp<<endl;
        output_in_area->header.frame_id = "/map";
        in_area_pub.publish(*output_in_area);
        cout<<"frame_id "<<i<<endl;



        ros::spinOnce();
    //     // loop_rate.sleep();
    }
    // pcl::PLYWriter writer;
    // writer.write("/home/krasus/ShanghaiTech/Mars_Lab/Reflection/Dataset/NewData/sist/Classified_all_pointcloud_new.ply", all_pointcloud);
    // read_ply(data_path + fileTimestamp[0].first, scan_cloud);
    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer->setBackgroundColor(0, 0, 0);
    // viewer->addPointCloud<pcl::PointXYZ>(scan_cloud, "sample cloud");
    // viewer->spin();
    ros::spin();
}
