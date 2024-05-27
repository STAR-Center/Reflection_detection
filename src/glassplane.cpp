#include <cstdlib>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <cmath>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/transforms.h>

/* Include some RANSAC libraries */
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <visualization_msgs/Marker.h>

#include "../include/DataStructure.hpp"

using namespace std;
#define PI 3.14159265

ros::Publisher marker_pub;
std::vector<Eigen::Matrix4d> pose_vec;
std::vector<Eigen::Affine3f > pose_trans_vec;

struct Pose{
    double timestamp;
    Eigen::Vector3d pose;
    Eigen::Quaterniond orientation;
};

std::vector<Pose> pose_vector_for_time;
std::vector<PlaneInfo> GlobalMapPlaneCoe = {};

Eigen::Vector3d proportion_average_normal(int a, int b, std::vector<double> planeA, std::vector<double> planeB){
    Eigen::Vector3d vectorA(planeA[0],planeA[1],planeA[2]);
    Eigen::Vector3d vectorB(planeB[0],planeB[1],planeB[2]);
    vectorA = (a*vectorA + b*vectorB)/(a+b);
    vectorA = vectorA / vectorA.norm();
    return vectorA;
}

double normal_diff(std::vector<double> planeA, std::vector<double> planeB){
    Eigen::Vector3d vectorA(planeA[0],planeA[1],planeA[2]);
    Eigen::Vector3d vectorB(planeB[0],planeB[1],planeB[2]);
    double cos_theta = vectorA.dot(vectorB) / (vectorA.norm() * vectorB.norm());
    return cos_theta;
}

// if these two area didn't match, return true
bool calculate_area_mismatch(pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points, pcl::PointCloud<pcl::PointXYZ>::Ptr global_hull_points, std::vector<double> worldPlane, pcl::PointCloud<pcl::PointXYZ>::Ptr hull_both){
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    coefficients->values.resize(4);
    coefficients->values[0] = worldPlane[0];
    coefficients->values[1] = worldPlane[1];
    coefficients->values[2] = worldPlane[2];
    coefficients->values[3] = worldPlane[3];
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(hull_points);
    proj.setModelCoefficients(coefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    proj.filter(*cloud_projected);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_both(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_both = *cloud_projected+*global_hull_points;
    

    pcl::ProjectInliers<pcl::PointXYZ> proj2;
    proj2.setModelType(pcl::SACMODEL_PLANE);
    proj2.setInputCloud(cloud_both);
    proj2.setModelCoefficients(coefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_both(new pcl::PointCloud<pcl::PointXYZ>);
    proj2.filter(*cloud_projected_both);


    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud_projected_both);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr hull_both(new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*hull_both);
    // cout<<"here is a match area:"<<endl;
    double area1 = pcl::calculatePolygonArea(*cloud_projected), area2 = pcl::calculatePolygonArea(*global_hull_points);
    double area_both = pcl::calculatePolygonArea(*hull_both);
    cout<<"area:  "<<area1<<"   "<<area2<<"   "<<area_both<<endl;

    // //可视化
    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer->addPolygon<pcl::PointXYZ>(cloud_projected, "polygon1");
    // viewer->addPolygon<pcl::PointXYZ>(global_hull_points, "polygon2");
    // viewer->spin();
    if(area_both>area1+area2){
        // cout<<"find a mismatch !!!!!!!"<<endl;
        return true;
    }else{
        return false;
    }
}

bool rayPlaneIntersection(const Eigen::Vector3d& rayOrigin, const Eigen::Vector3d& rayDirection, const Eigen::Vector3d& planeNormal, const Eigen::Vector3d& planePoint, Eigen::Vector3d& intersectionPoint){

    double dotProduct = rayDirection.dot(planeNormal);
    if(std::abs(dotProduct) < 0.000001){
        return false;
    }
    double distance = (planeNormal.dot(planePoint - rayOrigin) / dotProduct);
    // ray is on the opposite direction
    if(distance<0){
        return false;
    }

    intersectionPoint = rayOrigin + rayDirection * distance;
    return true;
}

Eigen::Vector2d projectPointToPlane(const Eigen::Vector3d& planeNormal, const Eigen::Vector3d& planePoint, const Eigen::Vector3d& point){

    double distance = (planeNormal.dot(planePoint - point))/planeNormal.norm();
    cout<<"distance: "<<distance<<endl;
    Eigen::Vector3d projectedPoint = point - distance * (planeNormal / planeNormal.norm());

    cout<<"projectedPoint: "<<projectedPoint<<endl;
    
    Eigen::Vector3d basis1 = planeNormal.unitOrthogonal();
    Eigen::Vector3d basis2 = planeNormal.cross(basis1);
    
    
    Eigen::Vector2d projectedPoint2D;
    projectedPoint2D<< projectedPoint.dot(basis1), projectedPoint.dot(basis2);
    return projectedPoint2D;
}

int cross(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2){
    return (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]);
}

bool pointInConvex(const std::vector<Eigen::Vector2d>& convexHull, const Eigen::Vector2d& point){
    int n = convexHull.size();
    // cout<<"n: "<<n<<endl;
    // if(n<3) return false;

    // bool clockwise = cross(convexHull[0], convexHull[1], convexHull[2]) < 0;

    // int left = 1, right = n-1;
    // while(left<right){
    //     int mid = (left +right)/2;
    //     if(cross(convexHull[0], convexHull[mid], point) > 0 == clockwise){
    //         left = mid + 1;
    //     }else{
    //         right = mid;
    //     }
    // }

    // if(cross(convexHull[0], convexHull[left], point) == 0){
    //     return true;
    // }else{
    //     return false;
    // }
    int intersections = 0;
    for(int i=0;i<n;i++){
        
        int next = (i+1)%n;
        // cout<<"convex i "<<convexHull[i].transpose()<<endl;
        // cout<<"convex next "<<convexHull[next].transpose()<<endl;
        if(((convexHull[i][1] <= point[1] && point[1] < convexHull[next][1]) ||
           (convexHull[next][1] <= point[1] && point[1] < convexHull[i][1]))
           && point[0] <= convexHull[i][0] + ( convexHull[next][0] -  convexHull[i][0])  * (point[1] - convexHull[i][1]) / (convexHull[next][1] - convexHull[i][1])){
            if(point[0] == convexHull[i][0] + ( convexHull[next][0] -  convexHull[i][0])  * (point[1] - convexHull[i][1]) / (convexHull[next][1] - convexHull[i][1])){
                return true;
            }
            intersections++;
        }else if(convexHull[i][1] == point[1] && point[1] == convexHull[next][1]){
            // cout<<"other"<<endl;
            if(point[0]<=max(convexHull[i][0], convexHull[next][0]) && min(convexHull[i][0], convexHull[next][0]) <= point[0]){
                return true;
            }
        }
        // cout<<"first:"<<((convexHull[i][1] <= point[1] && point[1] < convexHull[next][1]) || (convexHull[next][1] <= point[1] && point[1] < convexHull[i][1]))<<"  ";
        // cout<<"second:"<<(point[0] < convexHull[i][0] + ( convexHull[next][0] -  convexHull[i][0])  * (point[1] - convexHull[i][1]) / (convexHull[next][1] - convexHull[i][1]))<<endl;
        // cout<< point[0]<<"  "<<convexHull[i][0] + ( convexHull[next][0] -  convexHull[i][0])  * (point[1] - convexHull[i][1]) / (convexHull[next][1] - convexHull[i][1])<<endl;
        // cout<<convexHull[i][0] <<" "<<( convexHull[next][0] -  convexHull[i][0])  <<" "<<
    }
    return intersections%2 == 1;

}



void add_plane_maker(pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points, ros::Publisher& marker_pub, int index, int x=0, int y=0, int z=0){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "global_plane";
    marker.id = index;
    index++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = static_cast<float>(std::rand()) / RAND_MAX;
    marker.color.g = static_cast<float>(std::rand()) / RAND_MAX;
    marker.color.b = static_cast<float>(std::rand()) / RAND_MAX;
    for(auto p: hull_points->points){
        geometry_msgs::Point point;
        point.x = p.x+x;
        point.y = p.y+y;
        point.z = p.z+z;
        marker.points.push_back(point);
    }
    geometry_msgs::Point point;
    point.x = hull_points->points[0].x+x;
    point.y = hull_points->points[0].y+y;
    point.z = hull_points->points[0].z+z;
    marker.points.push_back(point);
    marker_pub.publish(marker);
}

bool match_checker(PlaneInfo& new_plane, PlaneInfo& global_plane){
    std::vector<double>& newPlane = new_plane.coe;
    std::vector<double>& worldPlane = global_plane.coe;

    if(abs(newPlane[3]-worldPlane[3]) < 0.1){
        double normal_dif = normal_diff(newPlane, worldPlane);
        if(1-normal_dif<0.2){
            pcl::PointCloud<pcl::PointXYZ>::Ptr hull_both(new pcl::PointCloud<pcl::PointXYZ>);
            if(calculate_area_mismatch(new_plane.hull_points, global_plane.hull_points,worldPlane,hull_both)){
                return false;
            }
            //start update
            // cout<<"Matched"<<endl;
            int a,b;
            if(global_plane.observedTime==1){
                a = 1;
                b = 3;
            }else if(global_plane.observedTime>10){
                a = 1;
                b = 10;
            }else{
                a =1;
                b = global_plane.observedTime;
            }
            Eigen::Vector3d aver_norm = proportion_average_normal(a,b,newPlane, worldPlane);
            std::vector<double> aver = {aver_norm[0], aver_norm[1], aver_norm[2], (a*newPlane[3]+b*worldPlane[3])/(b+a)};
            global_plane.coe = aver;
            global_plane.observedTime++;

            // cout<<"hull_both"<<pcl::calculatePolygonArea(*hull_both)<<endl;
            // cout<<"hull_points"<<pcl::calculatePolygonArea(*(global_plane.hull_points))<<endl;
            global_plane.hull_points = hull_both;
            // cout<<"hull_points"<<pcl::calculatePolygonArea(*(global_plane.hull_points))<<endl;
            return true;
        }
        return false;
    }
}

int frame_index  = 0;
void glassHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*laserCloudMsg, *cloud);
    cout<<endl<<"glass cloud size: "<<cloud->size()<<endl<<endl;
    if(cloud->size()<=30){
        cout<<"no enough glass cloud"<<endl;
        frame_index++;
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr transfered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transfered_cloud, pose_trans_vec[frame_index]);
    cout<<std::setprecision(16)<<pose_vector_for_time[frame_index].timestamp<<endl;
    frame_index++;

        
    pcl::ModelCoefficients::Ptr tempCoeff(new pcl::ModelCoefficients);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> sac;
    sac.setInputCloud(transfered_cloud);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setModelType(pcl::SACMODEL_PLANE);
    sac.setDistanceThreshold(0.07);
    sac.setMaxIterations(400);
    sac.setProbability(0.9);
    sac.segment(*inliers, *tempCoeff);
    cout<<"inlier nums = "<<inliers->indices.size()<<endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(transfered_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane);

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(plane);
    proj.setModelCoefficients(tempCoeff);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    proj.filter(*cloud_projected);
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud_projected);
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points(new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*hull_points);
    add_plane_maker(hull_points, marker_pub, 0);


    std::vector<double> newPlane = {tempCoeff->values[0], tempCoeff->values[1], tempCoeff->values[2], tempCoeff->values[3]};
    PlaneInfo newPlaneInfo;
    newPlaneInfo.coe = newPlane;
    newPlaneInfo.keyframeNum = frame_index;
    newPlaneInfo.observedTime = 1;
    newPlaneInfo.hull_points = hull_points;
    int match_flag = 0;
    for(PlaneInfo& plane : GlobalMapPlaneCoe){
        if(match_checker(newPlaneInfo, plane)){
            match_flag = 1;
            break;
        }
    }
    if(match_flag == 0){
        cout<<"add new glass plane"<<endl;
        GlobalMapPlaneCoe.push_back(newPlaneInfo);
    }

    for(int i = 0; i < GlobalMapPlaneCoe.size(); i++){
        cout<<"hull points size: "<<GlobalMapPlaneCoe[i].hull_points->size()<<endl;
        add_plane_maker(GlobalMapPlaneCoe[i].hull_points, marker_pub, i+1);
    }
    
}


int main(int argc, char** argv){
    cout<<"hello world"<<endl;
    ros::init(argc, argv, "glassplane");
    ros::NodeHandle nh;
    string pose_file_path = "/home/krasus/ShanghaiTech/Mars_Lab/Reflection/Dataset/NewData/sca/hesai_pose.txt";
    std::ifstream pose_file(pose_file_path);
    if(!pose_file.is_open()){
        std::cout<<"open pose file failed"<<std::endl;
        return -1;
    }
    std::string pose_line;
    while(std::getline(pose_file, pose_line)){
        // cout<<pose_line<<endl;
        std::istringstream iss(pose_line);
        Pose cur_pose;
        iss >> cur_pose.timestamp >> cur_pose.pose.x() >> cur_pose.pose.y() >> cur_pose.pose.z() >> cur_pose.orientation.x() >> cur_pose.orientation.y() >> cur_pose.orientation.z() >> cur_pose.orientation.w();
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = cur_pose.orientation.toRotationMatrix();
        T.block<3,1>(0,3) = cur_pose.pose;
        pose_vec.emplace_back(T);
        Eigen::Affine3d T_f = Eigen::Translation3d(cur_pose.pose) * cur_pose.orientation;
        
        pose_trans_vec.emplace_back(T_f);
        pose_vector_for_time.push_back(cur_pose);
    }

    std::vector<Eigen::Vector3d> glass_po = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
    };

    Eigen::Vector3d rayOrigin(0,0,0);
    Eigen::Vector3d rayDirection(2,2,2);
    Eigen::Vector3d plane_normal(1,0,0);
    Eigen::Vector3d plane_point(10,0,0);

    Eigen::Vector3d test_point1(1,1,1);
    Eigen::Vector3d test_point2(2,2,2);
    Eigen::Vector3d test_point3(10,3,3);
    Eigen::Vector3d test_point4(7,9,6);

    Eigen::Vector3d intersection_point;
    // if(rayPlaneIntersection(rayOrigin, rayDirection, plane_normal, plane_point, intersection_point)){
    //     cout<<"have intersection: "<<intersection_point.transpose()<<endl;
    //     Eigen::Vector2d projected_point1 =  projectPointToPlane(plane_normal,  plane_point, intersection_point);
    //     cout<<"projected_point1: "<<projected_point1.transpose()<<endl;
    //     Eigen::Vector2d projected_point2 =  projectPointToPlane(plane_normal,  plane_point, plane_point);
    //     cout<<"projected_point2: "<<projected_point2.transpose()<<endl;
    //     Eigen::Vector2d projected_point3 =  projectPointToPlane(plane_normal,  plane_point, test_point1);
    //     cout<<"projected_point3: "<<projected_point3.transpose()<<endl;
    //     Eigen::Vector2d projected_point4 =  projectPointToPlane(plane_normal,  plane_point, test_point2);
    //     cout<<"projected_point4: "<<projected_point4.transpose()<<endl;
    //     Eigen::Vector2d projected_point5 =  projectPointToPlane(plane_normal,  plane_point, test_point3);
    //     cout<<"projected_point5: "<<projected_point5.transpose()<<endl;
    //     Eigen::Vector2d projected_point6 =  projectPointToPlane(plane_normal,  plane_point, test_point4);
    //     cout<<"projected_point6: "<<projected_point6.transpose()<<endl;
        
    // }else{
    //     cout<<"no intersection"<<endl;
    // }
    std::vector<Eigen::Vector2d> convexHull;
    Eigen::Vector2d test_point11(0, 0);
    Eigen::Vector2d test_point12(10, 0);
    Eigen::Vector2d test_point13(10, 10);
    Eigen::Vector2d test_point14(0, 10);
    Eigen::Vector2d test_point_in(5, 5);
    convexHull.push_back(test_point11);
    convexHull.push_back(test_point12);
    convexHull.push_back(test_point13);
    convexHull.push_back(test_point14);
    // test point in convex
    // std::vector<Eigen::Vector2d> test_pointset = {test_point_in,Eigen::Vector2d(3,0),Eigen::Vector2d(-1,3),Eigen::Vector2d(7,13),Eigen::Vector2d(13,13),Eigen::Vector2d(0,7),Eigen::Vector2d(10,7),Eigen::Vector2d(10,10)};
    // for(auto test_point : test_pointset){
    //     if(pointInConvex(convexHull, test_point)){
    //         cout<<"point in convex"<<endl;
    //         cout<<"test_point: "<<test_point.transpose()<<endl;
    //     }else{
    //         cout<<"point not in convex"<<endl;
    //         cout<<"test_point: "<<test_point.transpose()<<endl;
    //     }    
    // }
    

    // open3d::geometry::TriangleMesh glass_mesh;
    // auto glass_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    // for(int i=1;i<glass_po.size()-1;i++){
    //     glass_mesh->vertices_.push_back(glass_po[0]);
    //     glass_mesh->vertices_.push_back(glass_po[i]);
    //     glass_mesh->vertices_.push_back(glass_po[i+1]);
    //     glass_mesh->triangles_.push_back({3*(i-1),3*(i-1)+1,3*(i-1)+2});
    // }
    // open3d::io::WriteTriangleMesh("glass_mesh.ply", *glass_mesh);
    // open3d::visualization::DrawGeometries({glass_mesh});

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/GlassPointCloud", 100, glassHandler);
    marker_pub = nh.advertise<visualization_msgs::Marker>("glass_plane_marker", 10);
    ros::spin();
}