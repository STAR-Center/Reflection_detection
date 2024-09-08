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

#include <pcl/kdtree/kdtree_flann.h>


#include "../include/DataStructure.hpp"

#include <fstream>

#define PI 3.14159265

std::vector<int> label_num = {0,0,0,0,0,0,0};

ros::Publisher dual;
ros::Publisher filteredstrongestpub;
ros::Publisher filteredlastpub;
ros::Publisher intensityline;
ros::Publisher inlierPC;
ros::Publisher strongestIndoorPC;
ros::Publisher strongestOutdoorPC;
ros::Publisher lastOutdoorObstaclePC;
ros::Publisher ReflectionPC;
ros::Publisher glassPC;
ros::Publisher mirroredReflectionPC;
ros::Publisher slamPC;

ros::Publisher glass_origin_PC;

sensor_msgs::PointCloud2 inlierCloud2;
sensor_msgs::PointCloud2 mirroredReflectionCloud2;
sensor_msgs::PointCloud2 strongestIndoorCloud2;
sensor_msgs::PointCloud2 strongestOutdoorCloud2;
sensor_msgs::PointCloud2 lastOutdoorObstacleCloud2;
sensor_msgs::PointCloud2 ReflectionCloud2;
sensor_msgs::PointCloud2 GlassCloud2;
sensor_msgs::PointCloud2 slamCloud2;
sensor_msgs::PointCloud2 strongestcloud;
sensor_msgs::PointCloud2 lastcloud;
sensor_msgs::PointCloud2 dualcloud;

sensor_msgs::PointCloud2 GlassoriginCloud2;

std::vector<pcl::PointCloud<pcl::PointXYZINormal>> strongestring;
std::vector<pcl::PointCloud<pcl::PointXYZINormal>> lastring;
pcl::PointCloud<pcl::PointXYZI>::Ptr strongestcloud1d(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr lastcloud1d(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr strongestorganized(new pcl::PointCloud<pcl::PointXYZI>);
using namespace pcl;
using namespace std;

int frame_index = 0;


template<typename PointType>
void FillWithNaNs(pcl::PointCloud<PointType> &cloud, vector<int> &row_count, int col) {
    for (int k = 0; k < row_count.size(); k++) {
        if (row_count[k] <= col) {
            PointType p_nan;
            p_nan.x = numeric_limits<float>::quiet_NaN();
            p_nan.y = numeric_limits<float>::quiet_NaN();
            p_nan.z = numeric_limits<float>::quiet_NaN();
            cloud(col, k) = p_nan;
            row_count[k]++;
        }
    }
}


/** @brief Takes in a ROS PointCloud2 object and outputs a PCL templated
 *         point cloud type, that has been organized. The input MUST be a
 *         cloud from a Velodyne LiDAR unit
 *  @param msg   - The input ROS point cloud message from a Velodyne LiDAR
 *  @param cloud - The organized cloud output
 *  @param rings - The number of rings in the Velodyne LiDAR unit
 */
template<typename PointType>
void
converttoorganized(sensor_msgs::PointCloud2 msg,
                   pcl::PointCloud<PointType> &cloud,
                   int rings) {
    // Convert the PC to the PointXYZIR point type so we can access the ring
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_XYZIR
            (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg, pcl_pc2);
    fromPCLPointCloud2(pcl_pc2, *cloud_XYZIR);

    /* Because not every row is complete, we can't know in advance how big the
       organized cloud needs to be, so we deliberately over-size by 30%. We then
       create a temporary cloud, SPECIFICALLY with the organized cloud
       constructor*/
    int columns = (cloud_XYZIR->points.size() / rings) *
                  2.0; // NOLINT(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
    PointCloud<pcl::PointXYZRGBNormal>::Ptr tmp_cloud_ptr(new PointCloud<pcl::PointXYZRGBNormal>);
    tmp_cloud_ptr->points.resize(columns * rings);
    tmp_cloud_ptr->height = static_cast<uint32_t>(rings);
    tmp_cloud_ptr->width = static_cast<uint32_t>(columns);
    tmp_cloud_ptr->is_dense = false;

    /* Iterate through the XYZIR points and fill the TMP cloud where the
       ring number determines the row the point is inserted into */
    int ring;
    int col = 0;
    vector<int> row_count(rings, 0);
    for (int i = 0; i < cloud_XYZIR->points.size(); i++) {
        ring = cloud_XYZIR->points[i].ring;
        /* If true, we're just about to start processing the next 'column' of data
           and need to quickly fill all the holes in the current column with NaNs
           to act as dummy points - important for preserving the structure of the
           organized cloud */
        if (row_count[ring] > col) {
            FillWithNaNs(*tmp_cloud_ptr, row_count, col);
            col++;
        }
        PointType p;
        p.x = cloud_XYZIR->points[i].x;
        p.y = cloud_XYZIR->points[i].y;
        p.z = cloud_XYZIR->points[i].z;
        tmp_cloud_ptr->at(row_count[ring], ring) = p; // cloud(col, row)
        row_count[ring]++;
    }
    FillWithNaNs(*tmp_cloud_ptr, row_count, col); // Fill that last column

    /* Now we copy the organized tmp cloud to our output cloud, which we can now
       size correctly, knowing EXACTLY how many points are in each ring/row. But
       we have to do this point-by-point (rather than using memcpy) because we
       the points in memory that we want to keep are interpersed with points
       we want to leave behind */
    cloud = pcl::PointCloud<PointType>(row_count[0], rings);
    cloud.height = static_cast<uint32_t>(rings);
    cloud.width = static_cast<uint32_t>(row_count[0]);
    cloud.is_dense = false;
    for (int col = 0; col < row_count[0]; col++) {
        for (int row = 0; row < rings; row++) {
            cloud(col, row) = tmp_cloud_ptr->at(col, row);
        }
    }
}

void convertting_to_ring(pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud, 
        std::vector<pcl::PointCloud<pcl::PointXYZINormal> > &cloudVector, unsigned int rings){
    cloudVector = std::vector<pcl::PointCloud<pcl::PointXYZINormal> >(rings);
    for(int i =0;i<input_cloud->points.size();i++){
        pcl::PointXYZINormal p;
        p.x = input_cloud->points[i].x;
        p.y = input_cloud->points[i].y;
        p.z = input_cloud->points[i].z;
        p.intensity = input_cloud->points[i].intensity;
        p.normal_x = input_cloud->points[i].normal_x;
        p.normal_y = input_cloud->points[i].normal_y;
        p.normal_z = input_cloud->points[i].normal_z;
        int ring = static_cast<int>(input_cloud->points[i].normal_x);
        cloudVector[ring].push_back(p);
    }
    int max_ringNUm=0;
    for(int i=0;i<cloudVector.size();i++){
        // cout<<"ring "<<i<<" size: "<<cloudVector[i].size()<<endl;
        max_ringNUm = max_ringNUm>cloudVector[i].size()?max_ringNUm:cloudVector[i].size();
    }
    cout<<"max ring num: "<<max_ringNUm<<endl;
}


pcl::PointIndices::Ptr getRansacPlaneCoeff(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &Glass,
                                           pcl::ModelCoefficients::Ptr &coefficients,
                                           pcl::PointCloud<pcl::PointXYZINormal>::Ptr &inlierCloud, double DistanceThreshold,
                                           double Probability) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZINormal> sac;
    sac.setInputCloud(Glass);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setModelType(pcl::SACMODEL_PLANE);
    sac.setDistanceThreshold(DistanceThreshold);
    sac.setMaxIterations(400);
    sac.setProbability(Probability);
    sac.segment(*inliers, *coefficients);
//    pcl::ExtractIndices<pcl::PointXYZI> extract;
    // Extract the inliers
//    extract.setInputCloud (Glass);
//    extract.setIndices (inliers);
//    extract.setNegative (true);
//    extract.filter (*inlierCloud);
    return inliers;
}

void reflectTransform(const pcl::PointCloud<pcl::PointXYZINormal> &cloud_in,
                      pcl::PointCloud<pcl::PointXYZINormal> &cloud_out,
                      const pcl::ModelCoefficients &plane) {
    double a = plane.values[0];
    double b = plane.values[1];
    double c = plane.values[2];
    double d = plane.values[3];
    Eigen::Vector3d plane_normal(a, b, c);
    Eigen::Matrix3d transform = Eigen::Matrix3d::Identity() - (2 * plane_normal *
                                                               plane_normal.transpose());
    Eigen::Affine3d affine(transform);
    affine.translation() << -2 * a * d, -2 * b * d, -2 * c * d;
    pcl::transformPointCloud(cloud_in, cloud_out, affine);
}


void dualcheck(pcl::PointCloud<PointXYZ>::Ptr glass_pointcloud, ros::Publisher correct_pc_pub) {
    pcl::PointCloud<pcl::PointXYZINormal> filteredstrongest;
    pcl::PointCloud<pcl::PointXYZINormal> filteredlast;
    pcl::PointCloud<pcl::PointXYZINormal> strongestordered[64] = {};

    unsigned int num_readings = 610;
    double ranges[num_readings];
    double intensities[num_readings];
    int count = 0;
    for (unsigned int i = 0; i < num_readings; ++i) {
        ranges[i] = count;
        intensities[i] = 100 + count;
    }

    /* A new container for points in the potential glass planes */
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr Glass(new pcl::PointCloud<pcl::PointXYZINormal>);
    Glass->width = 0;
    Glass->height = 1;
    Glass->is_dense = 0;
    int glassLeft = 9999;
    int glassRight = -9999;
    int glassUp = -9999;
    int glassDown = 9999;

    cout<<"before ring for"<<endl;
    for (int ring = 0; ring < 64; ++ring) {
        pcl::PointCloud<pcl::PointXYZINormal> strongest = strongestring[ring];
        pcl::PointCloud<pcl::PointXYZINormal> strongestringtmp;
        strongestringtmp.width = 610;
        strongestringtmp.height = 1;
        strongestringtmp.is_dense = 0;
        strongestringtmp.points.resize(610);
        pcl::PointCloud<pcl::PointXYZINormal> lastringtmp;
        lastringtmp.width = 610;
        lastringtmp.height = 1;
        lastringtmp.is_dense = 0;
        lastringtmp.points.resize(610);

        for (int i = 0; i < strongestring[ring].width; i++) {
            int strongestindex = int(
                    round((180 + atan2(strongestring[ring].points[i].y, strongestring[ring].points[i].x) * 180 / PI) /
                          0.6)) + 1;
            if (strongestindex > 0) {
                strongestringtmp[strongestindex] = strongestring[ring].points[i];
            }
        }

        for (int i = 0; i < lastring[ring].width; i++) {
            int lastindex = int(
                    round((180 + atan2(lastring[ring].points[i].y, lastring[ring].points[i].x) * 180 / PI) /
                          0.6)) + 1;
            if (lastindex > 0) {
                lastringtmp[lastindex] = lastring[ring].points[i];
            }
        }

        pcl::PointCloud<pcl::PointXYZINormal> filteredstrongestringtmp;
        pcl::PointCloud<pcl::PointXYZINormal> filteredlastringtmp;
        pcl::copyPointCloud(strongestringtmp, filteredstrongestringtmp);
        pcl::copyPointCloud(lastringtmp, filteredlastringtmp);
        for (int i = 0; i < 610; i++) {
            float distance = pcl::geometry::distance(filteredstrongestringtmp.points[i], filteredlastringtmp.points[i]);
            if(filteredlastringtmp.points[i].x==0&&filteredlastringtmp.points[i].y==0&&filteredlastringtmp.points[i].z==0){
                continue;
            }
            // if strong and last not match, then we just add them to glass, and set them 0 in the strongring
            // if (distance > 0.1 and filteredstrongestringtmp.points[i].z > -0.4) {
             if (distance > 0.1 ) {
                /* Manually add 1 to the width and use the new width to resize the vector */
                Glass->points.resize(++(Glass->width));
                Glass->points.push_back(filteredstrongestringtmp.points[i]);
                filteredstrongestringtmp.points[i].intensity = 0;
            }
        }

        filteredstrongest += filteredstrongestringtmp;
        filteredlast += filteredlastringtmp;
        strongestordered[ring] = strongestringtmp;
    }

    ++(Glass->width);
    pcl::toROSMsg(*Glass, GlassoriginCloud2);
    GlassoriginCloud2.header = strongestcloud.header;

    for(int i = 0;i<Glass->points.size();i++){
        pcl::PointXYZ p;
        p.x = Glass->points[i].x;
        p.y = Glass->points[i].y;
        p.z = Glass->points[i].z;
        glass_pointcloud->points.push_back(p);
    }

    /* Searches for the closest reflective planes */
    /* We already have some points on a plane, so here we use RANSAC to find the plane,
     * put these planes into a vector and remove the inliers belong to these planes.
     * @iteration rules how many planes will be found in maximum */
    std::vector<pcl::ModelCoefficients::Ptr> planeVec;
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> planeCloudVec;
    pcl::PointCloud<pcl::PointXYZINormal> inlierCloud;
    int iterations = 3;
    for (int i = 0; i < iterations && Glass->width >= 300; ++i) {
        /* Make a temporary coefficients instance to store this plane's coefficients */
        pcl::ModelCoefficients::Ptr tempCoeff(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr inlierTmp(new pcl::PointCloud<pcl::PointXYZINormal>);
        /* Run RANSAC here with model plane */
        pcl::PointIndices::Ptr inliers = getRansacPlaneCoeff(Glass, tempCoeff, inlierTmp, 0.07, 0.90);
        cout<<"inlier nums = "<<inliers->indices.size()<<endl;
        
        pcl::ExtractIndices<pcl::PointXYZINormal> extractInlier;
        extractInlier.setInputCloud(Glass);
        extractInlier.setIndices(inliers);
        extractInlier.setNegative(false);
        extractInlier.filter(*inlierTmp);

        pcl::ExtractIndices<pcl::PointXYZINormal> extractNeg;
        extractNeg.setInputCloud(Glass);
        extractNeg.setIndices(inliers);
        extractNeg.setNegative(true);
        extractNeg.filter(*Glass);

        inlierCloud += *inlierTmp;
        /* If there are too few points in the plane or it is a horizontal plane, just ignore it
         * because it may contain either noises or not the plane we are looking for*/
        // if (inlierTmp->points.size() > 200 && fabs(tempCoeff->values[2]) < 0.5) {
        if (inlierTmp->points.size() > 200) {    
            /* Unify the sign of the normal vectors of the planes */
            if (tempCoeff->values[3] < 0) {
                tempCoeff->values[0] = -tempCoeff->values[0];
                tempCoeff->values[1] = -tempCoeff->values[1];
                tempCoeff->values[2] = -tempCoeff->values[2];
                tempCoeff->values[3] = -tempCoeff->values[3];
            }
            /* Stores the coefficients into a vector */
            planeVec.push_back(tempCoeff);
            planeCloudVec.push_back(inlierTmp);
        } else {
            continue;
        }
    }
    cout<<"plane vec size: "<<planeVec.size()<<endl;

    std::vector<int> indexHasGlass;
    for (pcl::PointXYZINormal p : inlierCloud.points) {
        double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
        double ringDegree = ringRad * 180 / PI;
        char *ringDegreeChar = (char *) malloc(32 * sizeof(char));
        sprintf(ringDegreeChar, "%.2f", ringDegree);
        string ringDegreeString(ringDegreeChar);
        free(ringDegreeChar);
        int ring = static_cast<int>(p.normal_x);
        int pIndex = int(
                round((180 + atan2(p.y, p.x) * 180 / PI) /
                      0.6)) + 1;
        indexHasGlass.push_back(pIndex);
        if (ring < glassDown) {
            glassDown = ring;
        }
        if (ring > glassUp) {
            glassUp = ring;
        }
    }
    glassDown -= 1;
    glassUp += 1;
    if (indexHasGlass.size() > 0) {
        std::sort(indexHasGlass.begin(), indexHasGlass.end());
        glassLeft = indexHasGlass[0];
        glassRight = indexHasGlass[indexHasGlass.size() - 1];
        if (glassLeft < 54 || 567 < glassRight) {
            std::sort(indexHasGlass.begin(), indexHasGlass.end());
            glassLeft = indexHasGlass[indexHasGlass.size() - 1];
            glassRight = indexHasGlass[0];
            for (int j = 1; j < indexHasGlass.size() - 1; ++j) {
                if (indexHasGlass[j] - glassRight < 54)
                    glassRight = indexHasGlass[j];
                if (glassLeft - indexHasGlass[indexHasGlass.size() - j] < 54)
                    glassLeft = indexHasGlass[indexHasGlass.size() - j];
            }
        }
    }
    if (glassLeft < 54 || 567 < glassRight || glassRight - glassLeft < 81) {
        glassLeft = 0;
        glassRight = 610;
    }

    pcl::PointCloud<pcl::PointXYZINormal> strongestIndoor;
    pcl::PointCloud<pcl::PointXYZINormal> strongestOutdoor;
    pcl::PointCloud<pcl::PointXYZINormal> lastIndoor;
    pcl::PointCloud<pcl::PointXYZINormal> lastOutdoor;
    pcl::PointCloud<pcl::PointXYZINormal> lastOutdoorObstacle;
    pcl::PointCloud<pcl::PointXYZINormal> Reflection;
    pcl::PointCloud<pcl::PointXYZINormal> mirroredReflection;
    pcl::PointCloud<pcl::PointXYZINormal> nearestPlaneCloud;
    pcl::PointCloud<pcl::PointXYZINormal> secondNearestPlaneCloud;
    cout<<"glass plane detected nums:  "<<planeVec.size()<<endl;
    cout<<"Start !planeVec.empty()"<<endl;
    if (!planeVec.empty()) {
        /* Decide which plane is closed to the origin point */
        pcl::ModelCoefficients::Ptr nearestPlane = planeVec[0];
        pcl::ModelCoefficients::Ptr secondNearestPlane = planeVec[0];
        nearestPlaneCloud = *planeCloudVec[0];

        double maxIndoorDistance = 0.0;
        cout<<"In filteredstrongest but not in strongIndoor:"<<endl;
        // find out those points that is not in the area of the glass and find the max indoor distance
        for (pcl::PointXYZINormal p : filteredstrongest.points) {
            double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
            double ringDegree = ringRad * 180 / PI;
            char *ringDegreeChar = (char *) malloc(32 * sizeof(char));
            sprintf(ringDegreeChar, "%.2f", ringDegree);
            string ringDegreeString(ringDegreeChar);
            free(ringDegreeChar);
            int ring = static_cast<int>(p.normal_x);
            int pIndex = int(
                    round((180 + atan2(p.y, p.x) * 180 / PI) /
                          0.6)) + 1;

            double distance = pcl::pointToPlaneDistanceSigned(p, nearestPlane->values[0], nearestPlane->values[1],
                                                              nearestPlane->values[2], nearestPlane->values[3]);
            if ((pIndex > glassLeft && pIndex < glassRight && glassDown < ring && ring < glassUp) ||
                (glassLeft > glassRight && (pIndex > glassLeft || pIndex < glassRight) && glassDown < ring &&
                 ring < glassUp)) {
                // in the area of glass, but not close to glass plane
                if (distance > 0.05 && p.intensity > 0) {
                    strongestIndoor.push_back(p);
                }
            // not in the glass area and have no difference with the last point distance
            } else if (p.intensity > 0) {
                strongestIndoor.push_back(p);
            }
            if (distance > maxIndoorDistance) {
                maxIndoorDistance = distance;
            }
        }

        for (pcl::PointXYZINormal p : filteredlast.points) {
            if(p.intensity <=0){
                continue;
            }
            int ring =  static_cast<int>(p.normal_x);
            int pIndex = int(
                    round((180 + atan2(p.y, p.x) * 180 / PI) /
                          0.6)) + 1;

            double distance = pcl::pointToPlaneDistanceSigned(p, nearestPlane->values[0], nearestPlane->values[1],
                                                              nearestPlane->values[2], nearestPlane->values[3]);
            if ((pIndex > glassLeft && pIndex < glassRight && glassDown < ring && ring < glassUp) ||
                (glassLeft > glassRight && (pIndex > glassLeft || pIndex < glassRight) && glassDown < ring &&
                 ring < glassUp)) {
                // 会有last indoor吗？ 应该只有last = strong的吧可能
                if (distance > 0.05) {
                    lastIndoor.push_back(p);
                // distance 为负数，应该是在窗户外面的意思
                } else if (distance < -0.1) {
                    // 如果比室内任何点都远，那么应该一定就是室外的点了
                    if (-distance > maxIndoorDistance) {
                        lastOutdoorObstacle.push_back(p);
                    // 否则的话应该存疑
                    } else {
                        lastOutdoor.push_back(p);
                    }
                }
            }
        }
        cout<<"lastOutdoorObstacle.size() = "<<lastOutdoorObstacle.points.size()<<endl;
        cout<<"lastIndoor.size() = "<<lastIndoor.points.size()<<endl;


        // h按照镜平面反投影，如果其距离同一方向上的点比较接近，那就觉得成功(但没做什么),不成功，则加入到lastOutdoorObstacle中
        // 现在无法这样做，因为无法根据点的位置来计算激光光束的ring和index了
        pcl::PointCloud<pcl::PointXYZINormal> mirroredOutdoor;
        reflectTransform(lastOutdoor, mirroredOutdoor, *nearestPlane);
        for (int j = 0; j < mirroredOutdoor.points.size(); ++j) {
            pcl::PointXYZINormal p = mirroredOutdoor.points[j];
            double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
            double ringDegree = ringRad * 180 / PI;
            char *ringDegreeChar = (char *) malloc(32 * sizeof(char));
            sprintf(ringDegreeChar, "%.2f", ringDegree);
            string ringDegreeString(ringDegreeChar);
            free(ringDegreeChar);
            int ring = p.normal_x;
            pcl::PointXYZINormal zero;
            zero.x = 0;
            zero.y = 0;
            zero.z = 0;
            int pIndex = int(
                    round((180 + atan2(p.y, p.x) * 180 / PI) /
                          0.6)) + 1;
            if (pIndex > 0) {
                if ((pcl::geometry::distance(strongestordered[ring].points[pIndex], zero) -
                     pcl::geometry::distance(p, zero)) > 0.1) {
                    lastOutdoorObstacle.push_back(lastOutdoor.points[j]);
                }
            }
        }

        // 通过室外的点来对strong进行检查，找出可能是经过了反射的strong，加入reflection
        for (pcl::PointXYZINormal p :lastOutdoorObstacle) {
            double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
            double ringDegree = ringRad * 180 / PI;
            char *ringDegreeChar = (char *) malloc(32 * sizeof(char));
            sprintf(ringDegreeChar, "%.2f", ringDegree);
            string ringDegreeString(ringDegreeChar);
            free(ringDegreeChar);
            int ring = p.normal_x;
            int pIndex = int(
                    round((180 + atan2(p.y, p.x) * 180 / PI) /
                          0.6)) + 1;

            if (pIndex > 0) {
                /* If the this point is not the same as in the strongest and it is not on the glass
                 * then this point comes from a reflection*/
                if (pcl::geometry::distance(p, strongestordered[ring].points[pIndex]) > 0.1
                    && pcl::pointToPlaneDistanceSigned(strongestordered[ring].points[pIndex], nearestPlane->values[0],
                                                       nearestPlane->values[1],
                                                       nearestPlane->values[2], nearestPlane->values[3]) < -0.1) {
                    Reflection.push_back(strongestordered[ring].points[pIndex]);
                }
            }
        }
        reflectTransform(Reflection, mirroredReflection, *nearestPlane);
    }

    /* Publish this nearest plane point cloud to ros */
    //green means glass, blue means outdoor obstacle, red means mirroredreflection, black means normal points
    PointCloud<PointXYZRGB> nearestPlaneCloudRGB;
    cout<<"mistake in nearestPlaneCloudRGB"<<endl;
    cout<<"nearestPlaneCloud.size()"<<nearestPlaneCloud.size()<<endl;
    for (PointXYZINormal p: nearestPlaneCloud.points) {
        PointXYZRGB prgb;
        prgb.x = p.x;
        prgb.y = p.y;
        prgb.z = p.z;
        prgb.r = 0;
        prgb.g = 255;
        prgb.b = 0;
        nearestPlaneCloudRGB.points.push_back(prgb);
    }
    cout<<endl;
    pcl::toROSMsg(nearestPlaneCloudRGB, GlassCloud2);
    GlassCloud2.header = strongestcloud.header;
    for(int i = 0;i<nearestPlaneCloudRGB.points.size();i++){
        pcl::PointXYZ p;
        p.x = nearestPlaneCloudRGB.points[i].x;
        p.y = nearestPlaneCloudRGB.points[i].y;
        p.z = nearestPlaneCloudRGB.points[i].z;
        // glass_pointcloud->points.push_back(p);
    }
    

    PointCloud<PointXYZRGB> strongestIndoorRGB;
    int indooraccurate_num = 0;
    int indoorstrong_0_num = 0;
    
    
    cout<<"mistake in strongestIndoorRGB"<<endl;
    if (strongestIndoor.points.size() < 100) {
        //fallback solution for no windows detected to perform SLAM
        PointCloud<PointXYZINormal>::Ptr cloud_filtered(new PointCloud<PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr filteredstrongestPtr(new pcl::PointCloud<pcl::PointXYZINormal>);
        filteredstrongestPtr->points = filteredstrongest.points;
        pcl::PassThrough<PointXYZINormal> intensitypass;
        intensitypass.setInputCloud(filteredstrongestPtr);
        intensitypass.setFilterFieldName("intensity");
        intensitypass.setFilterLimits(2, 255);
        intensitypass.filter(*cloud_filtered);
        for (PointXYZINormal p: cloud_filtered->points) {
            PointXYZRGB prgb;
            prgb.x = p.x;
            prgb.y = p.y;
            prgb.z = p.z;
            prgb.r = 0;
            prgb.g = 0;
            prgb.b = 0;
            strongestIndoorRGB.points.push_back(prgb);
        }
    } else {
        for (PointXYZINormal p: strongestIndoor.points) {
            PointXYZRGB prgb;
            prgb.x = p.x;
            prgb.y = p.y;
            prgb.z = p.z;
            prgb.r = 0;
            prgb.g = 0;
            prgb.b = 0;
            strongestIndoorRGB.points.push_back(prgb);
        }
    }
    pcl::toROSMsg(strongestIndoorRGB, strongestIndoorCloud2);
    strongestIndoorCloud2.header = strongestcloud.header;
    // strongestIndoorPC.publish(strongestIndoorCloud2);

    PointCloud<PointXYZRGB> lastOutdoorObstacleRGB;
    int lastOutdoorObstacle_correct_num = 0;
    int lastOutdoorObstacle_0_num = 0;
    for (PointXYZINormal p: lastOutdoorObstacle.points) {
        PointXYZRGB prgb;
        prgb.x = p.x;
        prgb.y = p.y;
        prgb.z = p.z;
        prgb.r = 0;
        prgb.g = 0;
        prgb.b = 255;
        lastOutdoorObstacleRGB.points.push_back(prgb);
    }
    cout<<endl;
    pcl::toROSMsg(lastOutdoorObstacleRGB, lastOutdoorObstacleCloud2);
    lastOutdoorObstacleCloud2.header = strongestcloud.header;
    // lastOutdoorObstaclePC.publish(lastOutdoorObstacleCloud2);

//    PointCloud<PointXYZRGB> ReflectionRGB;
//    copyPointCloud(Reflection, ReflectionRGB);
//    pcl::toROSMsg(ReflectionRGB, ReflectionCloud2);
//    ReflectionCloud2.header = strongestcloud.header;
//    ReflectionPC.publish(ReflectionCloud2);

    PointCloud<PointXYZRGB> mirroredReflectionRGB;
    int mirroredReflection_correct_num = 0;
    int mirroredReflection_0_num = 0;
    for (PointXYZINormal p: mirroredReflection.points) {
        PointXYZRGB prgb;
        prgb.x = p.x;
        prgb.y = p.y;
        prgb.z = p.z;
        prgb.r = 255;
        prgb.g = 0;
        prgb.b = 0;
        mirroredReflectionRGB.points.push_back(prgb);
        if(p.normal_y==5){
            mirroredReflection_correct_num++;
        }else if(p.normal_y==0){
            mirroredReflection_0_num+=1;
        }
    }
    pcl::toROSMsg(mirroredReflectionRGB, mirroredReflectionCloud2);
    mirroredReflectionCloud2.header = strongestcloud.header;
    // mirroredReflectionPC.publish(mirroredReflectionCloud2);


    PointCloud<PointXYZRGB> slamRGB;
    slamRGB += nearestPlaneCloudRGB;
    slamRGB += strongestIndoorRGB;
    slamRGB += lastOutdoorObstacleRGB;
    slamRGB += mirroredReflectionRGB;
    pcl::toROSMsg(slamRGB, slamCloud2);
    slamCloud2.header.frame_id = "/map";
    cout<<slamCloud2.header<<endl;
    // slamPC.publish(slamCloud2);
    correct_pc_pub.publish(slamCloud2);
    frame_index++;

    //start intendity peak detection
    // std::vector<pcl::PointXYZINormal> peaks;
    // int i = 16;
    // while (i < strongestring[23].width - 16) {//peak>70 // intensitypeak from ring 21 to 25, middle at 23
    //     if (strongestring[23].points[i].intensity > 35
    //         && strongestring[23].points[i - 10].intensity < 50 && strongestring[23].points[i - 15].intensity < 30
    //         && strongestring[23].points[i + 10].intensity < 50 && strongestring[23].points[i + 15].intensity < 30) {
    //         peaks.push_back(strongestring[23].points[i]);
    //         i = i + 30;
    //     }
    //     i++;
    // }
    // pcl::PointCloud<pcl::PointXYZINormal> intensitypeaktmp;
    // for (i = 0; i < peaks.size(); i++) {
    //     int index = int(round((180 + atan2(peaks[i].y, peaks[i].x) * 180 / PI) / 0.16)) + 1;
    //     pcl::PointXYZINormal max;
    //     max.intensity = -999;
    //     int maxi = 0;
    //     for (int j = index - 20; j <= index + 20; j++) {
    //         if (strongestordered[23].points[j].intensity > max.intensity) {
    //             max = strongestordered[23].points[j];
    //             maxi = j;
    //         }
    //     }
    //     if (
    //             strongestordered[23].points[maxi - 10].intensity < 60 &&
    //             strongestordered[23].points[maxi + 10].intensity < 60 &&
    //             strongestordered[23].points[maxi - 15].intensity < 30 &&
    //             strongestordered[23].points[maxi + 15].intensity < 30 &&
    //             strongestordered[21].points[maxi].intensity < 60 &&
    //             strongestordered[22].points[maxi].intensity < 100 &&
    //             strongestordered[24].points[maxi].intensity < 100 &&
    //             strongestordered[25].points[maxi].intensity < 60 &&
    //             strongestordered[21].points[maxi - 10].intensity < 30 &&
    //             strongestordered[22].points[maxi - 15].intensity < 30 &&
    //             strongestordered[24].points[maxi - 15].intensity < 30 &&
    //             strongestordered[25].points[maxi - 10].intensity < 30 &&
    //             strongestordered[21].points[maxi + 15].intensity < 30 &&
    //             strongestordered[24].points[maxi + 10].intensity < 30 &&
    //             strongestordered[22].points[maxi + 15].intensity < 30 &&
    //             strongestordered[25].points[maxi + 10].intensity < 30 &&
    //             sqrt(pow(strongestordered[23].points[maxi].x, 2) + pow(strongestordered[23].points[maxi].y, 2)) < 3) {
    //         for (int k = 21; k <= 25; ++k) {
    //             for (int j = maxi - 10; j <= maxi + 10; ++j) {
    //                 if (sqrt(pow(strongestordered[k].points[j].x, 2) + pow(strongestordered[k].points[j].y, 2)) < 3)
    //                     strongestordered[k].points[j].intensity = 255;
    //             }
    //         }

    //     }

    // }
    // for (int k = 21; k <= 25; ++k) {
    //     intensitypeaktmp += strongestordered[k];
    // }

    // sensor_msgs::PointCloud2 intensitypeak2;
    // pcl::toROSMsg(intensitypeaktmp, intensitypeak2);
    // intensitypeak2.header = strongestcloud.header;
    // intensityline.publish(intensitypeak2);

    sensor_msgs::PointCloud2 filteredstrongest2;
    pcl::toROSMsg(filteredstrongest, filteredstrongest2);
    filteredstrongest2.header = strongestcloud.header;
    // filteredstrongestpub.publish(filteredstrongest2);
    sensor_msgs::PointCloud2 filteredlast2;
    pcl::toROSMsg(filteredlast, filteredlast2);
    filteredlast2.header = lastcloud.header;
    // filteredlastpub.publish(filteredlast2);
//    printf("end dualcheck\n");
}






// judge if a ray hit plane, return bool and the intersection position
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

// project 3d point onto 3d plane, but transfer it to 2d coordinates
Eigen::Vector2d projectPointToPlane(const Eigen::Vector3d& planeNormal, const Eigen::Vector3d& planePoint, const Eigen::Vector3d& point){

    double distance = (planeNormal.dot(planePoint - point))/planeNormal.norm();
    // cout<<"distance: "<<distance<<endl;
    Eigen::Vector3d projectedPoint = point - distance * (planeNormal / planeNormal.norm());

    // cout<<"projectedPoint: "<<projectedPoint<<endl;
    
    Eigen::Vector3d basis1 = planeNormal.unitOrthogonal();
    Eigen::Vector3d basis2 = planeNormal.cross(basis1);
    
    
    Eigen::Vector2d projectedPoint2D;
    projectedPoint2D<< projectedPoint.dot(basis1), projectedPoint.dot(basis2);
    return projectedPoint2D;
}


int cross(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2){
    return (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]);
}

// judge if a 2d point is in the 2d convex hull or not, return bool
bool pointInConvex(const std::vector<Eigen::Vector2d>& convexHull, const Eigen::Vector2d& point){
    int n = convexHull.size();

    int intersections = 0;
    for(int i=0;i<n;i++){
        
        int next = (i+1)%n;
        if(((convexHull[i][1] <= point[1] && point[1] < convexHull[next][1]) ||
           (convexHull[next][1] <= point[1] && point[1] < convexHull[i][1]))
           && point[0] <= convexHull[i][0] + ( convexHull[next][0] -  convexHull[i][0])  * (point[1] - convexHull[i][1]) / (convexHull[next][1] - convexHull[i][1])){
            if(point[0] == convexHull[i][0] + ( convexHull[next][0] -  convexHull[i][0])  * (point[1] - convexHull[i][1]) / (convexHull[next][1] - convexHull[i][1])){
                return true;
            }
            intersections++;
        }else if(convexHull[i][1] == point[1] && point[1] == convexHull[next][1]){
            if(point[0]<=max(convexHull[i][0], convexHull[next][0]) && min(convexHull[i][0], convexHull[next][0]) <= point[0]){
                return true;
            }
        }
    }
    return intersections%2 == 1;

}

void dualcheck_for_global(std::vector<PlaneInfo>& GlobalMapPlaneCoe, Eigen::Vector3d& trans_originpoint, pcl::PointCloud<PointXYZINormal>::Ptr glass_pointcloud, ros::Publisher correct_pc_pub) {
//    printf("start dualcheck_for_global\n");
    pcl::PointCloud<pcl::PointXYZINormal> filteredstrongest;
    pcl::PointCloud<pcl::PointXYZINormal> filteredlast;
    pcl::PointCloud<pcl::PointXYZINormal> strongestordered[64] = {};

    // cout<<"GlobalMapPlaneCoe.size() = "<<GlobalMapPlaneCoe.size()<<endl;
    // cout<<GlobalMapPlaneCoe[0].coe[0]<<" "<<GlobalMapPlaneCoe[0].coe[1]<<" "<<GlobalMapPlaneCoe[0].coe[2]<<" "<<GlobalMapPlaneCoe[0].coe[3]<<endl;
    // cout<<"GlobalMapPlaneCoe hull points size = "<<GlobalMapPlaneCoe[0].hull_points->points.size()<<endl;
    // cout<<"trans_originpoint = "<<trans_originpoint.transpose()<<endl;
    // return;

    // compute the convex hull of all reflective planes, store the 2d convex hull in each plane
    std::vector<std::vector<Eigen::Vector2d>> convexHulls = {};
    for(PlaneInfo global_plane :GlobalMapPlaneCoe){
        std::vector<Eigen::Vector2d> convexHull = {};
        pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points = global_plane.hull_points;
        Eigen::Vector3d planeNormal(global_plane.coe[0], global_plane.coe[1], global_plane.coe[2]);
        Eigen::Vector3d planePoint(hull_points->points[0].x, hull_points->points[0].y, hull_points->points[0].z);
        for(auto hull_point: global_plane.hull_points->points){
            Eigen::Vector3d hull_poi(hull_point.x, hull_point.y, hull_point.z);
            Eigen::Vector2d point = projectPointToPlane(planeNormal, planePoint, hull_poi);
            convexHull.push_back(point);
            // cout<<"x "<<point[0]<<" y "<<point[1]<<" ";
        }
        // cout<<endl;
        convexHulls.push_back(convexHull);
    }

    unsigned int num_readings = 610;

    /* A new container for points in the potential glass planes */
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr Glass(new pcl::PointCloud<pcl::PointXYZINormal>);
    Glass->width = 0;
    Glass->height = 1;
    Glass->is_dense = 0;

    for (int ring = 0; ring < 64; ++ring) {
        pcl::PointCloud<pcl::PointXYZINormal> strongest = strongestring[ring];
        pcl::PointCloud<pcl::PointXYZINormal> strongestringtmp;
        strongestringtmp.width = 610;
        strongestringtmp.height = 1;
        strongestringtmp.is_dense = 0;
        strongestringtmp.points.resize(610);
        pcl::PointCloud<pcl::PointXYZINormal> lastringtmp;
        lastringtmp.width = 610;
        lastringtmp.height = 1;
        lastringtmp.is_dense = 0;
        lastringtmp.points.resize(610);

        for (int i = 0; i < strongestring[ring].width; i++) {
            
            int strongestindex = int(
                    round((180 + atan2(strongestring[ring].points[i].y, strongestring[ring].points[i].x) * 180 / PI) /
                          0.6)) + 1;
            strongestindex = int(strongestring[ring].points[i].normal_z);
            if (strongestindex > 0) {
                strongestringtmp[strongestindex] = strongestring[ring].points[i];
            }
        }

        for (int i = 0; i < lastring[ring].width; i++) {
            int lastindex = int(
                    round((180 + atan2(lastring[ring].points[i].y, lastring[ring].points[i].x) * 180 / PI) /
                          0.6)) + 1;
            lastindex = (lastring[ring].points[i].normal_z);
            if (lastindex > 0) {
                lastringtmp[lastindex] = lastring[ring].points[i];
            }
        }
        pcl::PointCloud<pcl::PointXYZINormal> filteredstrongestringtmp;
        pcl::PointCloud<pcl::PointXYZINormal> filteredlastringtmp;
        pcl::copyPointCloud(strongestringtmp, filteredstrongestringtmp);
        pcl::copyPointCloud(lastringtmp, filteredlastringtmp);
        for (int i = 0; i < 610; i++) {
            float distance = pcl::geometry::distance(filteredstrongestringtmp.points[i], filteredlastringtmp.points[i]);
            if(filteredlastringtmp.points[i].x==0&&filteredlastringtmp.points[i].y==0&&filteredlastringtmp.points[i].z==0){
                // no matching last point
                continue;
            }
            // if strong and last not match, then we just add them to glass, and set them 0 in the strongring
            // if (distance > 0.1 and filteredstrongestringtmp.points[i].z > -0.4) {
             if (distance > 0.1 ) {
                /* Manually add 1 to the width and use the new width to resize the vector */
                Glass->points.resize(++(Glass->width));
                Glass->points.push_back(filteredstrongestringtmp.points[i]);
                filteredstrongestringtmp.points[i].intensity = 0;
            }
        }

        filteredstrongest += filteredstrongestringtmp;
        filteredlast += filteredlastringtmp;
        strongestordered[ring] = strongestringtmp;
    }

    ++(Glass->width);
    pcl::toROSMsg(*Glass, GlassoriginCloud2);
    GlassoriginCloud2.header = strongestcloud.header;
    // glass_origin_PC.publish(GlassoriginCloud2);
    for(auto p : Glass->points){
        pcl::PointXYZ point_pcl;
        point_pcl.x = p.x;
        point_pcl.y = p.y;
        point_pcl.z = p.z;
        // glass_pointcloud->points.push_back(point_pcl);
    }

    /* Searches for the closest reflective planes */
    /* We already have some points on a plane, so here we use RANSAC to find the plane,
     * put these planes into a vector and remove the inliers belong to these planes.
     * @iteration rules how many planes will be found in maximum */
    std::vector<pcl::ModelCoefficients::Ptr> planeVec;
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> planeCloudVec;
    pcl::PointCloud<pcl::PointXYZINormal> inlierCloud;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr Glass_left(new pcl::PointCloud<pcl::PointXYZINormal>);
    int iterations = 3;
    int ii=0;
    pcl::PointIndices::Ptr inliers_max;
    for (int i = 0; i < iterations && Glass->width >= 300; ++i) {
        /* Make a temporary coefficients instance to store this plane's coefficients */
        pcl::ModelCoefficients::Ptr tempCoeff(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr inlierTmp(new pcl::PointCloud<pcl::PointXYZINormal>);
        /* Run RANSAC here with model plane */
        pcl::PointIndices::Ptr inliers = getRansacPlaneCoeff(Glass, tempCoeff, inlierTmp, 0.07, 0.90);

        pcl::ExtractIndices<pcl::PointXYZINormal> extractInlier;
        extractInlier.setInputCloud(Glass);
        extractInlier.setIndices(inliers);
        extractInlier.setNegative(false);
        extractInlier.filter(*inlierTmp);

        pcl::ExtractIndices<pcl::PointXYZINormal> extractNeg;
        extractNeg.setInputCloud(Glass);
        extractNeg.setIndices(inliers);
        extractNeg.setNegative(true);
        extractNeg.filter(*Glass);
        inlierCloud += *inlierTmp;
        /* If there are too few points in the plane or it is a horizontal plane, just ignore it
         * because it may contain either noises or not the plane we are looking for*/
        // if (inlierTmp->points.size() > 200 && fabs(tempCoeff->values[2]) < 0.5) {
        if (inlierTmp->points.size() > 200) {    
            if(ii==0){
                ii++;
                inliers_max = inliers;
                Glass_left->points = Glass->points;

            }
            /* Unify the sign of the normal vectors of the planes */
            if (tempCoeff->values[3] < 0) {
                tempCoeff->values[0] = -tempCoeff->values[0];
                tempCoeff->values[1] = -tempCoeff->values[1];
                tempCoeff->values[2] = -tempCoeff->values[2];
                tempCoeff->values[3] = -tempCoeff->values[3];
            }
            /* Stores the coefficients into a vector */
            planeVec.push_back(tempCoeff);
            planeCloudVec.push_back(inlierTmp);
        } else {
            continue;
        }
    }



    pcl::PointCloud<pcl::PointXYZINormal> strongestIndoor;
    pcl::PointCloud<pcl::PointXYZINormal> strongestOutdoor;
    pcl::PointCloud<pcl::PointXYZINormal> lastIndoor;
    pcl::PointCloud<pcl::PointXYZINormal> lastOutdoor;
    pcl::PointCloud<pcl::PointXYZINormal> lastOutdoorObstacle;
    pcl::PointCloud<pcl::PointXYZINormal> Reflection;
    pcl::PointCloud<pcl::PointXYZINormal> mirroredReflection;
    pcl::PointCloud<pcl::PointXYZINormal> nearestPlaneCloud;
    pcl::PointCloud<pcl::PointXYZINormal> secondNearestPlaneCloud;
    pcl::PointCloud<pcl::PointXYZINormal> strongest_unclassifiedyet;

    // if (!planeVec.empty()) {
    if (1) {
        /* Decide which plane is closed to the origin point */
        pcl::ModelCoefficients::Ptr nearestPlane(new pcl::ModelCoefficients());// = planeVec[0];
        // pcl::ModelCoefficients::Ptr secondNearestPlane = planeVec[0];
        // nearestPlaneCloud = *planeCloudVec[0];
        if(planeVec.empty()){
            nearestPlane.reset(new pcl::ModelCoefficients);
            nearestPlane->values.resize(4);
            nearestPlane->values[0] = GlobalMapPlaneCoe[0].coe[0];
            nearestPlane->values[1] = GlobalMapPlaneCoe[0].coe[1];
            nearestPlane->values[2] = GlobalMapPlaneCoe[0].coe[2];
            nearestPlane->values[3] = GlobalMapPlaneCoe[0].coe[3];
        }else{
            nearestPlane = planeVec[0];
            // secondNearestPlane = planeVec[0];
            nearestPlaneCloud = *planeCloudVec[0];
        }
        for(auto p: Glass_left->points){
            int push_flag = 0;
            Eigen::Vector3d p_point(p.x, p.y, p.z);
            Eigen::Vector3d rayDirection(p.x-trans_originpoint[0], p.y-trans_originpoint[1], p.z-trans_originpoint[2]);
            for(int i=0; i<GlobalMapPlaneCoe.size(); i++){
                PlaneInfo global_plane = GlobalMapPlaneCoe[i];
                pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points = global_plane.hull_points;
                Eigen::Vector3d planeNormal(global_plane.coe[0], global_plane.coe[1], global_plane.coe[2]);
                Eigen::Vector3d planePoint(hull_points->points[0].x, hull_points->points[0].y, hull_points->points[0].z);
                Eigen::Vector3d intersection_point;
                double distance = pcl::pointToPlaneDistanceSigned(p, global_plane.coe[0], global_plane.coe[1],
                                                              global_plane.coe[2], global_plane.coe[3]);
                if(rayPlaneIntersection(trans_originpoint, rayDirection, planeNormal, planePoint,  intersection_point)){
                    double distance_p = (p_point-trans_originpoint).norm();
                    double distance_i = (intersection_point-trans_originpoint).norm();
                    Eigen::Vector2d point = projectPointToPlane(planeNormal, planePoint, intersection_point);
                    if(pointInConvex(convexHulls[i], point)){
                        if(abs(distance) < 0.05){
                            if(pointInConvex(convexHulls[i], point)){
                                nearestPlaneCloud.points.push_back(p);
                                push_flag = 1;
                                break;
                            }
                        }
                        //outdoor
                        if(distance_p+0.1 > distance_i){
                            push_flag = 1;
                            strongest_unclassifiedyet.points.push_back(p);
                            break;
                        }
                    }
                }
                
            }
            if(push_flag == 0){
                strongestIndoor.points.push_back(p);
            }
        }


        
        double maxIndoorDistance = 0.0;
        int new_alg_count = 0;
        int old_alg_count = 0;
        // find out those points that is not in the area of the glass and find the max indoor distance
        for (pcl::PointXYZINormal p : filteredstrongest.points) {
            if(p.intensity<=0){
                continue;
            }
            int ring = static_cast<int>(p.normal_x);
            int pIndex = int(
                    round((180 + atan2(p.y, p.x) * 180 / PI) /
                          0.6)) + 1;
            pIndex = p.normal_z;
            double distance2 = pcl::pointToPlaneDistanceSigned(p, nearestPlane->values[0], nearestPlane->values[1],
                                                              nearestPlane->values[2], nearestPlane->values[3]);
            double distance = 0;
            if(p.x==0 && p.y==0 && p.z==0){
                continue;
            }
            Eigen::Vector3d p_point(p.x, p.y, p.z);
            Eigen::Vector3d rayDirection(p.x-trans_originpoint[0], p.y-trans_originpoint[1], p.z-trans_originpoint[2]);
            int inarea_flag = 0;
            int rayPlaneinter_flag = 0;
            double distance_p = 0;
            double distance_i = 0;
            for(int i=0; i<GlobalMapPlaneCoe.size(); i++){
                PlaneInfo global_plane = GlobalMapPlaneCoe[i];
                pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points = global_plane.hull_points;
                Eigen::Vector3d planeNormal(global_plane.coe[0], global_plane.coe[1], global_plane.coe[2]);
                Eigen::Vector3d planePoint(hull_points->points[0].x, hull_points->points[0].y, hull_points->points[0].z);
                Eigen::Vector3d intersection_point;
                distance = pcl::pointToPlaneDistanceSigned(p, global_plane.coe[0], global_plane.coe[1],
                                                              global_plane.coe[2], global_plane.coe[3]);

                // ray have intersection
                if(rayPlaneIntersection(trans_originpoint, rayDirection, planeNormal, planePoint,  intersection_point)){
                    rayPlaneinter_flag = 1;
                    distance_p = (p_point-trans_originpoint).norm();
                    distance_i = (intersection_point-trans_originpoint).norm();
                    Eigen::Vector2d point = projectPointToPlane(planeNormal, planePoint, intersection_point);

                    if(pointInConvex(convexHulls[i], point)){

                        if(abs(distance_p-distance_i) < 0.1){

                            if(pointInConvex(convexHulls[i], point)){
                                nearestPlaneCloud.points.push_back(p);
                                inarea_flag = 1;
                                break;
                            }
                            continue;
                        }
                        // point is beyond the intersection point
                        if(distance_p+0.1 > distance_i){
                            
                            // if(pointInConvex(convexHulls[i], point)){
                            // distance is positive, not beyond glass
                            if (distance > 0.05 && p.intensity > 0) {
                                // strongestIndoor.push_back(p);
                                pcl::PointXYZ point_pcl;
                                point_pcl.x = p.x;
                                point_pcl.y = p.y;
                                point_pcl.z = p.z;
                                // glass_pointcloud->points.push_back(point_pcl);
                            }else{
                                if(p.intensity > 0){
                                    strongest_unclassifiedyet.points.push_back(p);
                                }
                            }
                            inarea_flag = 1;
                            break;
                            // }
                        }
                    }
                }
            }
            if(inarea_flag==1){

            }else if(p.intensity>0){
                strongestIndoor.push_back(p);
                pcl::PointXYZ point_pcl;
                if (distance2 > maxIndoorDistance) {
                    maxIndoorDistance = distance2;
                }
            }
        }

        int count_lastfielted = 0;
        for (pcl::PointXYZINormal p : filteredlast.points) {
            if(p.intensity <=0){
                continue;
            }
            count_lastfielted++;
            int ring = static_cast<int>(p.normal_x);
            int pIndex = int(
                    round((180 + atan2(p.y, p.x) * 180 / PI) /
                          0.6)) + 1;
            pIndex = p.normal_z;
            double distance = 0;
            Eigen::Vector3d p_point(p.x, p.y, p.z);
            Eigen::Vector3d rayDirection(p.x-trans_originpoint[0], p.y-trans_originpoint[1], p.z-trans_originpoint[2]);
            int inarea_flag = 0;
            for(int i=0; i<GlobalMapPlaneCoe.size(); i++){
                PlaneInfo global_plane = GlobalMapPlaneCoe[i];
                pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points = global_plane.hull_points;
                Eigen::Vector3d planeNormal(global_plane.coe[0], global_plane.coe[1], global_plane.coe[2]);
                Eigen::Vector3d planePoint(hull_points->points[0].x, hull_points->points[0].y, hull_points->points[0].z);
                Eigen::Vector3d intersection_point;
                distance = pcl::pointToPlaneDistanceSigned(p, global_plane.coe[0], global_plane.coe[1],
                                                              global_plane.coe[2], global_plane.coe[3]);
                // ray have intersection
                
                if(rayPlaneIntersection(trans_originpoint, rayDirection, planeNormal, planePoint,  intersection_point)){
                    double distance_p = (p_point-trans_originpoint).norm();
                    double distance_i = (intersection_point-trans_originpoint).norm();
                    Eigen::Vector2d point = projectPointToPlane(planeNormal, planePoint, intersection_point);
                    //glass
                    if(abs(distance_p-distance_i) < 0.1){
                        if(pointInConvex(convexHulls[i], point)){
                            nearestPlaneCloud.points.push_back(p);
                            inarea_flag = 1;
                            break;
                        }
                        continue;
                    }
                    // point is beyond the intersection point
                    // if(distance_p+0.2 > distance_i){
                    
                    if(distance_p+0.1 > distance_i){
                        Eigen::Vector2d point = projectPointToPlane(planeNormal, planePoint, p_point);
                        // if(pointInConvex(convexHulls[i], point)){
                        if (distance > 0.05 ) {
                            // lastIndoor.push_back(p);
                            // inarea_flag = 1;
                            // break;
                        }else if(distance<-0.1){
                            // cout<<p.normal_y<<" ";
                            if (-distance > maxIndoorDistance+0.1) {
                                lastOutdoorObstacle.push_back(p);
                            // 否则的话应该存疑
                            } else {
                                lastOutdoor.push_back(p);
                            }
                            inarea_flag = 1;
                            break;
                        }
                        inarea_flag = 1;
                        break;
                        
                    }
                }
            }
            if(inarea_flag == 0){
                lastIndoor.push_back(p);
            }
        }


        pcl::PointCloud<pcl::PointXYZINormal> mirroredOutdoor;
        reflectTransform(lastOutdoor, mirroredOutdoor, *nearestPlane);
        // Have no ring num for mirrored point, can't do this step now
        // for (int j = 0; j < mirroredOutdoor.points.size(); ++j) {
        //     pcl::PointXYZINormal p = mirroredOutdoor.points[j];
        //     double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
        //     double ringDegree = ringRad * 180 / PI;
        //     char *ringDegreeChar = (char *) malloc(32 * sizeof(char));
        //     sprintf(ringDegreeChar, "%.2f", ringDegree);
        //     string ringDegreeString(ringDegreeChar);
        //     free(ringDegreeChar);
        //     int ring = p.normal_x;
        //     pcl::PointXYZINormal zero;
        //     zero.x = trans_originpoint[0];
        //     zero.y = trans_originpoint[1];
        //     zero.z = trans_originpoint[2];
        //     int pIndex = int(
        //             round((180 + atan2(p.y, p.x) * 180 / PI) /
        //                   0.6)) + 1;
        //     //problem here need to change
        //     pIndex = p.normal_z;
        //     if (pIndex > 0) {
        //         if ((pcl::geometry::distance(strongestordered[ring].points[pIndex], zero) -
        //              pcl::geometry::distance(p, zero)) > 0.1) {
        //             lastOutdoorObstacle.push_back(lastOutdoor.points[j]);
        //             cout<<lastOutdoor.points[j].normal_y<<" ";
        //         }
        //     }
        // }

        // 通过室外的点来对strong进行检查，找出可能是经过了反射的strong，加入reflection
        for (pcl::PointXYZINormal p :lastOutdoorObstacle) {
            double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
            double ringDegree = ringRad * 180 / PI;
            char *ringDegreeChar = (char *) malloc(32 * sizeof(char));
            sprintf(ringDegreeChar, "%.2f", ringDegree);
            string ringDegreeString(ringDegreeChar);
            free(ringDegreeChar);
            int ring = p.normal_x;
            int pIndex = int(
                    round((180 + atan2(p.y, p.x) * 180 / PI) /
                          0.6)) + 1;
            pIndex = p.normal_z;
            if (pIndex > 0) {
                /* If the this point is not the same as in the strongest and it is not on the glass
                 * then this point comes from a reflection*/
                if (pcl::geometry::distance(p, strongestordered[ring].points[pIndex]) > 0.1
                    && pcl::pointToPlaneDistanceSigned(strongestordered[ring].points[pIndex], nearestPlane->values[0],
                                                       nearestPlane->values[1],
                                                       nearestPlane->values[2], nearestPlane->values[3]) < -0.1) {
                    Reflection.push_back(strongestordered[ring].points[pIndex]);
                }
            }
        }

        strongest_unclassifiedyet+=lastOutdoor;

        pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr strongestIndoor_ptr(new pcl::PointCloud<pcl::PointXYZINormal>(strongestIndoor));
        kdtree.setInputCloud(strongestIndoor_ptr);
        int K=1;
        std::vector<int> point_indices(K);
        std::vector<float> point_distances(K);
        for(int i=0; i<GlobalMapPlaneCoe.size(); i++){
            nearestPlane.reset(new pcl::ModelCoefficients);
            nearestPlane->values.resize(4);
            nearestPlane->values[0] = GlobalMapPlaneCoe[i].coe[0];
            nearestPlane->values[1] = GlobalMapPlaneCoe[i].coe[1];
            nearestPlane->values[2] = GlobalMapPlaneCoe[i].coe[2];
            nearestPlane->values[3] = GlobalMapPlaneCoe[i].coe[3];
            pcl::PointCloud<pcl::PointXYZINormal> mirroredIndoor;
            reflectTransform(strongest_unclassifiedyet, mirroredIndoor, *nearestPlane);
            for(int ip = 0; ip< mirroredIndoor.points.size();ip++){
            // for (pcl::PointXYZINormal p :mirroredIndoor) {
                pcl::PointXYZINormal p = mirroredIndoor.points[ip];
                kdtree.nearestKSearch(p, K, point_indices, point_distances);
                if(point_distances[0]<0.1){
                    // Reflection.push_back(strongest_unclassifiedyet[ip]);
                    mirroredReflection.push_back(p);
                }
            }
            // cout<<endl;
        }
        pcl::PointCloud<pcl::PointXYZINormal> mirroredReflectiontmp;
        reflectTransform(Reflection, mirroredReflectiontmp, *nearestPlane);

        mirroredReflection+=mirroredReflectiontmp;

    }

    /* Publish this nearest plane point cloud to ros */
    //green means glass, blue means outdoor obstacle, red means mirroredreflection, black means normal points
    PointCloud<PointXYZRGB> nearestPlaneCloudRGB;
    for (PointXYZINormal p: nearestPlaneCloud.points) {
        PointXYZRGB prgb;
        prgb.x = p.x;
        prgb.y = p.y;
        prgb.z = p.z;
        prgb.r = 0;
        prgb.g = 255;
        prgb.b = 0;
        nearestPlaneCloudRGB.points.push_back(prgb);
        
    }
    // cout<<endl;
    pcl::toROSMsg(nearestPlaneCloudRGB, GlassCloud2);
    GlassCloud2.header = strongestcloud.header;
    // glassPC.publish(GlassCloud2);
    for(int i = 0;i<nearestPlaneCloudRGB.points.size();i++){
        pcl::PointXYZ p;
        p.x = nearestPlaneCloudRGB.points[i].x;
        p.y = nearestPlaneCloudRGB.points[i].y;
        p.z = nearestPlaneCloudRGB.points[i].z;
    }
    

    PointCloud<PointXYZRGB> strongestIndoorRGB;
    int indooraccurate_num = 0;
    int indoorstrong_0_num = 0;
    int indoorstrong_4_num = 0;
    if (strongestIndoor.points.size() < 100) {
        //fallback solution for no windows detected to perform SLAM
        PointCloud<PointXYZINormal>::Ptr cloud_filtered(new PointCloud<PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr filteredstrongestPtr(new pcl::PointCloud<pcl::PointXYZINormal>);
        filteredstrongestPtr->points = filteredstrongest.points;
        pcl::PassThrough<PointXYZINormal> intensitypass;
        intensitypass.setInputCloud(filteredstrongestPtr);
        intensitypass.setFilterFieldName("intensity");
        intensitypass.setFilterLimits(2, 255);
        intensitypass.filter(*cloud_filtered);
        for (PointXYZINormal p: cloud_filtered->points) {
            PointXYZRGB prgb;
            prgb.x = p.x;
            prgb.y = p.y;
            prgb.z = p.z;
            prgb.r = 0;
            prgb.g = 0;
            prgb.b = 0;
            strongestIndoorRGB.points.push_back(prgb);
        }
    } else {
        for (PointXYZINormal p: strongestIndoor.points) {
            PointXYZRGB prgb;
            prgb.x = p.x;
            prgb.y = p.y;
            prgb.z = p.z;
            prgb.r = 0;
            prgb.g = 0;
            prgb.b = 0;
            strongestIndoorRGB.points.push_back(prgb);

        }
        // cout<<endl;
    }
    pcl::toROSMsg(strongestIndoorRGB, strongestIndoorCloud2);
    strongestIndoorCloud2.header = strongestcloud.header;
    // strongestIndoorPC.publish(strongestIndoorCloud2);

    PointCloud<PointXYZRGB> lastOutdoorObstacleRGB;
    int lastOutdoorObstacle_correct_num = 0;
    int lastOutdoorObstacle_0_num = 0;
    for (PointXYZINormal p: lastOutdoorObstacle.points) {
        PointXYZRGB prgb;
        prgb.x = p.x;
        prgb.y = p.y;
        prgb.z = p.z;
        prgb.r = 0;
        prgb.g = 0;
        prgb.b = 255;
        lastOutdoorObstacleRGB.points.push_back(prgb);
    }
    // cout<<endl;
    pcl::toROSMsg(lastOutdoorObstacleRGB, lastOutdoorObstacleCloud2);
    lastOutdoorObstacleCloud2.header = strongestcloud.header;
    // lastOutdoorObstaclePC.publish(lastOutdoorObstacleCloud2);

//    PointCloud<PointXYZRGB> ReflectionRGB;
//    copyPointCloud(Reflection, ReflectionRGB);
//    pcl::toROSMsg(ReflectionRGB, ReflectionCloud2);
//    ReflectionCloud2.header = strongestcloud.header;
//    ReflectionPC.publish(ReflectionCloud2);

    PointCloud<PointXYZRGB> mirroredReflectionRGB;
    int mirroredReflection_correct_num = 0;
    int mirroredReflection_0_num = 0;
    for (PointXYZINormal p: mirroredReflection.points) {
        PointXYZRGB prgb;
        prgb.x = p.x;
        prgb.y = p.y;
        prgb.z = p.z;
        prgb.r = 255;
        prgb.g = 0;
        prgb.b = 0;
        mirroredReflectionRGB.points.push_back(prgb);
    }
    pcl::toROSMsg(mirroredReflectionRGB, mirroredReflectionCloud2);
    mirroredReflectionCloud2.header = strongestcloud.header;
    // mirroredReflectionPC.publish(mirroredReflectionCloud2);

    PointCloud<PointXYZRGB> slamRGB;
    slamRGB += nearestPlaneCloudRGB;
    slamRGB += strongestIndoorRGB;
    slamRGB += lastOutdoorObstacleRGB;
    slamRGB += mirroredReflectionRGB;
    pcl::toROSMsg(slamRGB, slamCloud2);
    slamCloud2.header.frame_id = "/map";
    // cout<<slamCloud2.header<<endl;
    // slamPC.publish(slamCloud2);
    correct_pc_pub.publish(slamCloud2);
    frame_index++;

    //start intendity peak detection
    // std::vector<pcl::PointXYZINormal> peaks;
    // int i = 16;
    // while (i < strongestring[23].width - 16) {//peak>70 // intensitypeak from ring 21 to 25, middle at 23
    //     if (strongestring[23].points[i].intensity > 35
    //         && strongestring[23].points[i - 10].intensity < 50 && strongestring[23].points[i - 15].intensity < 30
    //         && strongestring[23].points[i + 10].intensity < 50 && strongestring[23].points[i + 15].intensity < 30) {
    //         peaks.push_back(strongestring[23].points[i]);
    //         i = i + 30;
    //     }
    //     i++;
    // }
    // pcl::PointCloud<pcl::PointXYZINormal> intensitypeaktmp;
    // for (i = 0; i < peaks.size(); i++) {
    //     int index = int(round((180 + atan2(peaks[i].y, peaks[i].x) * 180 / PI) / 0.16)) + 1;
    //     pcl::PointXYZINormal max;
    //     max.intensity = -999;
    //     int maxi = 0;
    //     for (int j = index - 20; j <= index + 20; j++) {
    //         if (strongestordered[23].points[j].intensity > max.intensity) {
    //             max = strongestordered[23].points[j];
    //             maxi = j;
    //         }
    //     }
    //     if (
    //             strongestordered[23].points[maxi - 10].intensity < 60 &&
    //             strongestordered[23].points[maxi + 10].intensity < 60 &&
    //             strongestordered[23].points[maxi - 15].intensity < 30 &&
    //             strongestordered[23].points[maxi + 15].intensity < 30 &&
    //             strongestordered[21].points[maxi].intensity < 60 &&
    //             strongestordered[22].points[maxi].intensity < 100 &&
    //             strongestordered[24].points[maxi].intensity < 100 &&
    //             strongestordered[25].points[maxi].intensity < 60 &&
    //             strongestordered[21].points[maxi - 10].intensity < 30 &&
    //             strongestordered[22].points[maxi - 15].intensity < 30 &&
    //             strongestordered[24].points[maxi - 15].intensity < 30 &&
    //             strongestordered[25].points[maxi - 10].intensity < 30 &&
    //             strongestordered[21].points[maxi + 15].intensity < 30 &&
    //             strongestordered[24].points[maxi + 10].intensity < 30 &&
    //             strongestordered[22].points[maxi + 15].intensity < 30 &&
    //             strongestordered[25].points[maxi + 10].intensity < 30 &&
    //             sqrt(pow(strongestordered[23].points[maxi].x, 2) + pow(strongestordered[23].points[maxi].y, 2)) < 3) {
    //         for (int k = 21; k <= 25; ++k) {
    //             for (int j = maxi - 10; j <= maxi + 10; ++j) {
    //                 if (sqrt(pow(strongestordered[k].points[j].x, 2) + pow(strongestordered[k].points[j].y, 2)) < 3)
    //                     strongestordered[k].points[j].intensity = 255;
    //             }
    //         }

    //     }

    // }
    // for (int k = 21; k <= 25; ++k) {
    //     intensitypeaktmp += strongestordered[k];
    // }

    // sensor_msgs::PointCloud2 intensitypeak2;
    // pcl::toROSMsg(intensitypeaktmp, intensitypeak2);
    // intensitypeak2.header = strongestcloud.header;
    // intensityline.publish(intensitypeak2);

    sensor_msgs::PointCloud2 filteredstrongest2;
    pcl::toROSMsg(filteredstrongest, filteredstrongest2);
    filteredstrongest2.header = strongestcloud.header;
    // filteredstrongestpub.publish(filteredstrongest2);
    sensor_msgs::PointCloud2 filteredlast2;
    pcl::toROSMsg(filteredlast, filteredlast2);
    filteredlast2.header = lastcloud.header;
    // filteredlastpub.publish(filteredlast2);
   printf("end dualcheck\n");
}


void cloud_preprocess(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &strongest, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &last,  pcl::PointCloud<PointXYZ>::Ptr glass_pointcloud, ros::Publisher correct_pc_pub) {
    cout<<"Here is cloud_preprocess"<<endl;

    convertting_to_ring(strongest, strongestring, 64);
    convertting_to_ring(last, lastring, 64);


    cout<<"strongest size: "<<strongest->points.size()<<endl;
    cout<<"last size: "<<last->points.size() <<endl;
    for(int i = 0; i<label_num.size(); i++){
        label_num[i] = 0;
    }
    for(auto p:strongest->points){
        label_num[(int)p.normal_y]++;
    }
    for(auto p:last->points){
        label_num[(int)p.normal_y]++;
    }
    cout<<"end of cloud preprocessing"<<endl;

    dualcheck(glass_pointcloud, correct_pc_pub);
}

void cloud_preprocess_global(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &strongest, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &last, std::vector<PlaneInfo>& GlobalMapPlaneCoe, Eigen::Vector3d& trans_originpoint, pcl::PointCloud<PointXYZINormal>::Ptr glass_pointcloud, ros::Publisher correct_pc_pub){
    cout<<"Here is cloud_preprocess_global"<<endl;

    convertting_to_ring(strongest, strongestring, 64);
    convertting_to_ring(last, lastring, 64);


    cout<<"strongest size: "<<strongest->points.size()<<endl;
    cout<<"last size: "<<last->points.size() <<endl;
    for(int i = 0; i<label_num.size(); i++){
        label_num[i] = 0;
    }
    for(auto p:strongest->points){
        label_num[(int)p.normal_y]++;
    }
    for(auto p:last->points){
        label_num[(int)p.normal_y]++;
    }
    cout<<"end of cloud_preprocess_global"<<endl;

    // dualcheck(glass_pointcloud);
    dualcheck_for_global(GlobalMapPlaneCoe, trans_originpoint, glass_pointcloud, correct_pc_pub);
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

void glassHandler(const pcl::PointCloud<PointXYZ>::Ptr &cloud,Eigen::Affine3d T_f, ros::Publisher& marker_pub,  std::vector<PlaneInfo>& GlobalMapPlaneCoe){

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromROSMsg(*laserCloudMsg, *cloud);
    cout<<endl<<"glass cloud size: "<<cloud->size()<<endl<<endl;
    if(cloud->size()<=30){
        cout<<"no enough glass cloud"<<endl;
        // frame_index++;
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr transfered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transfered_cloud, T_f);
    // cout<<std::setprecision(16)<<pose_vector_for_time[frame_index].timestamp<<endl;
    // frame_index++;

    for(int i = 0;i<5 && transfered_cloud->size() > 300; i++)
    {
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

        if(inliers->indices.size() < 120){
            break;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(transfered_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane);

        pcl::ExtractIndices<pcl::PointXYZ> extract_Neg;
        extract_Neg.setInputCloud(transfered_cloud);
        extract_Neg.setIndices(inliers);
        extract_Neg.setNegative(true);
        extract_Neg.filter(*transfered_cloud);

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
        // add_plane_maker(hull_points, marker_pub, 0);
        std::vector<double> newPlane;
        if(tempCoeff->values[3]>0){
            newPlane = {tempCoeff->values[0], tempCoeff->values[1], tempCoeff->values[2], tempCoeff->values[3]};
        }else{
            newPlane = {-tempCoeff->values[0], -tempCoeff->values[1], -tempCoeff->values[2], -tempCoeff->values[3]};
        }
        
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
    }

    for(int i = 0; i < GlobalMapPlaneCoe.size(); i++){
        if(GlobalMapPlaneCoe[i].observedTime>=5){
            cout<<"hull points size: "<<GlobalMapPlaneCoe[i].hull_points->size()<<"obverse time: "<<GlobalMapPlaneCoe[i].observedTime<<endl;
            cout<<GlobalMapPlaneCoe[i].coe[0]<<" "<<GlobalMapPlaneCoe[i].coe[1]<<" "<<GlobalMapPlaneCoe[i].coe[2]<<" "<<GlobalMapPlaneCoe[i].coe[3]<<endl;
            add_plane_maker(GlobalMapPlaneCoe[i].hull_points, marker_pub, i+1);
        }
        // cout<<"hull points size: "<<GlobalMapPlaneCoe[i].hull_points->size()<<"obverse time: "<<GlobalMapPlaneCoe[i].observedTime<<endl;
        // add_plane_maker(GlobalMapPlaneCoe[i].hull_points, marker_pub, i+1);
    }
    
}
