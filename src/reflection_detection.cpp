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

#define PI 3.14159265


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

std::vector<pcl::PointCloud<pcl::PointXYZI>> strongestring;
std::vector<pcl::PointCloud<pcl::PointXYZI>> lastring;
pcl::PointCloud<pcl::PointXYZI>::Ptr strongestcloud1d(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr lastcloud1d(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr strongestorganized(new pcl::PointCloud<pcl::PointXYZI>);
using namespace pcl;
using namespace std;
map<string, int> ringMap = {
        {"-30.67", 0},
        {"-29.33", 1},
        {"-28.00", 2},
        {"-26.67", 3},
        {"-25.33", 4},
        {"-24.00", 5},
        {"-22.67", 6},
        {"-21.33", 7},
        {"-20.00", 8},
        {"-18.67", 9},
        {"-17.33", 10},
        {"-16.00", 11},
        {"-14.67", 12},
        {"-13.33", 13},
        {"-12.00", 14},
        {"-10.67", 15},
        {"-9.33",  16},
        {"-8.00",  17},
        {"-6.67",  18},
        {"-5.33",  19},
        {"-4.00",  20},
        {"-2.67",  21},
        {"-1.33",  22},
        {"0.00",   23},
        {"1.33",   24},
        {"2.67",   25},
        {"4.00",   26},
        {"5.33",   27},
        {"6.67",   28},
        {"8.00",   29},
        {"9.33",   30},
        {"10.67",  31}
};

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


template<typename PointT>
void converttorings(const pcl::PCLPointCloud2 &msg, pcl::PointCloud<PointT> &cloud1D,
                    std::vector<pcl::PointCloud<PointT> > &cloudVector, unsigned int rings) {
    cloud1D.header = msg.header;
    cloud1D.width = msg.width;
    cloud1D.height = msg.height;
    cloud1D.is_dense = msg.is_dense == 0;
    uint32_t num_points = msg.width * msg.height;
    cloud1D.points.resize(num_points);
    uint8_t *cloud_data1 = reinterpret_cast<uint8_t *>(&cloud1D.points[0]);
    cout<<"cloudD1 width:"<<cloud1D.width<<endl;
    cout<<"cloudD1 hight:"<<cloud1D.height<<endl;

    pcl::PointCloud<PointT> *cloudPerLaser = new pcl::PointCloud<PointT>[rings];
    uint8_t *cloud_data2[rings];

    unsigned int pointsCounter[rings] = {0};
    for (unsigned int i = 0; i < rings; ++i) {
        cloudPerLaser[i] = pcl::PointCloud<PointT>();
        cloudPerLaser[i].header = msg.header;
        cloudPerLaser[i].width = msg.width;
        cloudPerLaser[i].height = msg.height;
        cloudPerLaser[i].is_dense = msg.is_dense == 0;
        cloudPerLaser[i].points.resize(num_points);
        cloud_data2[i] = reinterpret_cast<uint8_t *>(&cloudPerLaser[i].points[0]);
    }

    for (uint32_t row = 0; row < msg.height; ++row) {
        const uint8_t *row_data = &msg.data[row * msg.row_step];

        for (uint32_t col = 0; col < msg.width; ++col) {
            const uint8_t *msg_data = row_data + col * msg.point_step;

            //float* x = (float*)msg_data;
            //float* y = (float*)(msg_data + 4);
            //float* z = (float*)(msg_data + 8);
            //float* i = (float*)(msg_data + 16);
            auto *ring = (uint16_t *) (msg_data + 20);
            memcpy(cloud_data2[*ring], msg_data, 22);
            memcpy(cloud_data1, msg_data, 22);
            pointsCounter[*ring]++;
            cloud_data1 += sizeof(PointT);
            cloud_data2[*ring] += sizeof(PointT);
        }
    }

    cloudVector = std::vector<pcl::PointCloud<PointT> >(rings);

    for (unsigned int i = 0; i < rings; ++i) {
        cloudPerLaser[i].width = pointsCounter[i];
        cloudPerLaser[i].height = 1;
        cloudPerLaser[i].points.resize(pointsCounter[i]);
        cloudVector[i] = (cloudPerLaser[i]);
    }

    delete[] cloudPerLaser;
}


/* Function to do RANSAC on potential glass points */
void
ransacOnGlassPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &Glass, std::vector<int> &inliers) {
    pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr
            model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(Glass));

    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
    ransac.setDistanceThreshold(.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
}

pcl::PointIndices::Ptr getRansacPlaneCoeff(pcl::PointCloud<pcl::PointXYZI>::Ptr &Glass,
                                           pcl::ModelCoefficients::Ptr &coefficients,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr &inlierCloud, double DistanceThreshold,
                                           double Probability) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> sac;
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

void reflectTransform(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                      pcl::PointCloud<pcl::PointXYZI> &cloud_out,
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


void dualcheck() {
//    printf("start dualcheck\n");
    pcl::PointCloud<pcl::PointXYZI> filteredstrongest;
    pcl::PointCloud<pcl::PointXYZI> filteredlast;
    pcl::PointCloud<pcl::PointXYZI> strongestordered[32] = {};//TODO Cause double free error in rosbag 470.5s

    unsigned int num_readings = 2260;
    double ranges[num_readings];
    double intensities[num_readings];
    int count = 0;
    for (unsigned int i = 0; i < num_readings; ++i) {
        ranges[i] = count;
        intensities[i] = 100 + count;
    }

    /* A new container for points in the potential glass planes */
    pcl::PointCloud<pcl::PointXYZI>::Ptr Glass(new pcl::PointCloud<pcl::PointXYZI>);
    Glass->width = 0;
    Glass->height = 1;
    Glass->is_dense = 0;
    int glassLeft = 9999;
    int glassRight = -9999;
    int glassUp = -9999;
    int glassDown = 9999;

    for (int ring = 0; ring < 32; ++ring) {
        pcl::PointCloud<pcl::PointXYZI> strongest = strongestring[ring];
        pcl::PointCloud<pcl::PointXYZI> strongestringtmp;
        strongestringtmp.width = 2260;
        strongestringtmp.height = 1;
        strongestringtmp.is_dense = 0;
        strongestringtmp.points.resize(2260);
        pcl::PointCloud<pcl::PointXYZI> lastringtmp;
        lastringtmp.width = 2260;
        lastringtmp.height = 1;
        lastringtmp.is_dense = 0;
        lastringtmp.points.resize(2260);
        for (int i = 0; i < strongestring[ring].width; i++) {
            int strongestindex = int(
                    round((180 + atan2(strongestring[ring].points[i].y, strongestring[ring].points[i].x) * 180 / PI) /
                          0.16)) + 1;
            if (strongestindex > 0) {
                strongestringtmp[strongestindex] = strongestring[ring].points[i];
            }
            int lastindex = int(
                    round((180 + atan2(lastring[ring].points[i].y, lastring[ring].points[i].x) * 180 / PI) /
                          0.16)) + 1;
            if (lastindex > 0) {
                lastringtmp[lastindex] = lastring[ring].points[i];
            }
        }
        pcl::PointCloud<pcl::PointXYZI> filteredstrongestringtmp;
        pcl::PointCloud<pcl::PointXYZI> filteredlastringtmp;
        pcl::copyPointCloud(strongestringtmp, filteredstrongestringtmp);
        pcl::copyPointCloud(lastringtmp, filteredlastringtmp);
        for (int i = 0; i < 2260; i++) {
            float distance = pcl::geometry::distance(filteredstrongestringtmp.points[i], filteredlastringtmp.points[i]);
            if (distance > 0.1 and filteredstrongestringtmp.points[i].z > -0.4) {
                /* Manually add 1 to the width and use the new width to resize the vector */
                Glass->points.resize(++(Glass->width));
//                printf("Size of potential glass point array %d\n", Glass->width);
                Glass->points.push_back(filteredstrongestringtmp.points[i]);
                filteredstrongestringtmp.points[i].intensity = 0;
            }
        }

        filteredstrongest += filteredstrongestringtmp;
        filteredlast += filteredlastringtmp;
        strongestordered[ring] = strongestringtmp;
    }

    /* Searches for the closest reflective planes */
    /* We already have some points on a plane, so here we use RANSAC to find the plane,
     * put these planes into a vector and remove the inliers belong to these planes.
     * @iteration rules how many planes will be found in maximum */
    std::vector<pcl::ModelCoefficients::Ptr> planeVec;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> planeCloudVec;
    pcl::PointCloud<pcl::PointXYZI> inlierCloud;
    int iterations = 3;
    for (int i = 0; i < iterations && Glass->width >= 300; ++i) {
        /* Make a temporary coefficients instance to store this plane's coefficients */
        pcl::ModelCoefficients::Ptr tempCoeff(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZI>::Ptr inlierTmp(new pcl::PointCloud<pcl::PointXYZI>);
        /* Run RANSAC here with model plane */
        pcl::PointIndices::Ptr inliers = getRansacPlaneCoeff(Glass, tempCoeff, inlierTmp, 0.07, 0.90);

        pcl::ExtractIndices<pcl::PointXYZI> extractInlier;
        extractInlier.setInputCloud(Glass);
        extractInlier.setIndices(inliers);
        extractInlier.setNegative(false);
        extractInlier.filter(*inlierTmp);

        pcl::ExtractIndices<pcl::PointXYZI> extractNeg;
        extractNeg.setInputCloud(Glass);
        extractNeg.setIndices(inliers);
        extractNeg.setNegative(true);
        extractNeg.filter(*Glass);

        inlierCloud += *inlierTmp;
        /* If there are too few points in the plane or it is a horizontal plane, just ignore it
         * because it may contain either noises or not the plane we are looking for*/
        if (inlierTmp->points.size() > 200 && fabs(tempCoeff->values[2]) < 0.5) {
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

    std::vector<int> indexHasGlass;
    for (pcl::PointXYZI p : inlierCloud.points) {
        double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
        double ringDegree = ringRad * 180 / PI;
        char *ringDegreeChar = (char *) malloc(32 * sizeof(char));
        sprintf(ringDegreeChar, "%.2f", ringDegree);
        string ringDegreeString(ringDegreeChar);
        free(ringDegreeChar);
        int ring = ringMap[ringDegreeString];
        int pIndex = int(
                round((180 + atan2(p.y, p.x) * 180 / PI) /
                      0.16)) + 1;
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
        // printf("before %d %d %d %d\n", glassLeft, glassRight, glassDown, glassUp);
        if (glassLeft < 200 || 2100 < glassRight) {
            std::sort(indexHasGlass.begin(), indexHasGlass.end());
            glassLeft = indexHasGlass[indexHasGlass.size() - 1];
            glassRight = indexHasGlass[0];
            for (int j = 1; j < indexHasGlass.size() - 1; ++j) {
                if (indexHasGlass[j] - glassRight < 200)
                    glassRight = indexHasGlass[j];
                if (glassLeft - indexHasGlass[indexHasGlass.size() - j] < 200)
                    glassLeft = indexHasGlass[indexHasGlass.size() - j];
            }
        }
        // printf("\nafter %d %d %d %d\n", glassLeft, glassRight, glassDown, glassUp);
    }
    if (glassLeft < 200 || 2100 < glassRight || glassRight - glassLeft < 300) {
        glassLeft = 0;
        glassRight = 2260;
    }

    pcl::PointCloud<pcl::PointXYZI> strongestIndoor;
    pcl::PointCloud<pcl::PointXYZI> strongestOutdoor;
    pcl::PointCloud<pcl::PointXYZI> lastIndoor;
    pcl::PointCloud<pcl::PointXYZI> lastOutdoor;
    pcl::PointCloud<pcl::PointXYZI> lastOutdoorObstacle;
    pcl::PointCloud<pcl::PointXYZI> Reflection;
    pcl::PointCloud<pcl::PointXYZI> mirroredReflection;
    pcl::PointCloud<pcl::PointXYZI> nearestPlaneCloud;
    pcl::PointCloud<pcl::PointXYZI> secondNearestPlaneCloud;
    if (!planeVec.empty()) {
        /* Decide which plane is closed to the origin point */
        pcl::ModelCoefficients::Ptr nearestPlane = planeVec[0];
        pcl::ModelCoefficients::Ptr secondNearestPlane = planeVec[0];
        nearestPlaneCloud = *planeCloudVec[0];
        for (int i = 1; i < planeVec.size(); ++i) // NOLINT(bugprone-too-small-loop-variable)
        {
            if (nearestPlane->values[3] > planeVec[i]->values[3]) {
                secondNearestPlane = nearestPlane;
                secondNearestPlaneCloud = nearestPlaneCloud;
                nearestPlane = planeVec[i];
                nearestPlaneCloud = *planeCloudVec[i];
            }
        }

        // std::cerr << "Plane coefficients: " << nearestPlane->values[0] << " "
                //   << nearestPlane->values[1] << " "
                //   << nearestPlane->values[2] << " "
                //   << nearestPlane->values[3] << " " << nearestPlaneCloud.points.size() << std::endl;
        double maxIndoorDistance = 0.0;
        for (pcl::PointXYZI p : filteredstrongest.points) {
            double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
            double ringDegree = ringRad * 180 / PI;
            char *ringDegreeChar = (char *) malloc(32 * sizeof(char));
            sprintf(ringDegreeChar, "%.2f", ringDegree);
            string ringDegreeString(ringDegreeChar);
            free(ringDegreeChar);
            int ring = ringMap[ringDegreeString];
            int pIndex = int(
                    round((180 + atan2(p.y, p.x) * 180 / PI) /
                          0.16)) + 1;

            double distance = pcl::pointToPlaneDistanceSigned(p, nearestPlane->values[0], nearestPlane->values[1],
                                                              nearestPlane->values[2], nearestPlane->values[3]);
            if ((pIndex > glassLeft && pIndex < glassRight && glassDown < ring && ring < glassUp) ||
                (glassLeft > glassRight && (pIndex > glassLeft || pIndex < glassRight) && glassDown < ring &&
                 ring < glassUp)) {
                if (distance > 0.05 && p.intensity > 0) {
                    strongestIndoor.push_back(p);
                }
            } else if (p.intensity > 0) {
                strongestIndoor.push_back(p);
            }
            if (distance > maxIndoorDistance) {
                maxIndoorDistance = distance;
            }
        }

        for (pcl::PointXYZI p : filteredlast.points) {
            double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
            double ringDegree = ringRad * 180 / PI;
            char *ringDegreeChar = (char *) malloc(32 * sizeof(char));
            sprintf(ringDegreeChar, "%.2f", ringDegree);
            string ringDegreeString(ringDegreeChar);
            free(ringDegreeChar);
            int ring = ringMap[ringDegreeString];
            int pIndex = int(
                    round((180 + atan2(p.y, p.x) * 180 / PI) /
                          0.16)) + 1;

            double distance = pcl::pointToPlaneDistanceSigned(p, nearestPlane->values[0], nearestPlane->values[1],
                                                              nearestPlane->values[2], nearestPlane->values[3]);
            if ((pIndex > glassLeft && pIndex < glassRight && glassDown < ring && ring < glassUp) ||
                (glassLeft > glassRight && (pIndex > glassLeft || pIndex < glassRight) && glassDown < ring &&
                 ring < glassUp)) {
                if (distance > 0.05) {
                    lastIndoor.push_back(p);
                } else if (distance < -0.1) {
                    if (-distance > maxIndoorDistance) {
                        lastOutdoorObstacle.push_back(p);
                    } else {
                        lastOutdoor.push_back(p);
                    }
                }
            }
        }

        if (lastOutdoorObstacle.points.size() < 50) {
            // printf("%d\n", lastOutdoorObstacle.points.size());
            // std::cerr << "Second Plane coefficients: " << secondNearestPlane->values[0] << " "
            //           << secondNearestPlane->values[1] << " "
            //           << secondNearestPlane->values[2] << " "
            //           << secondNearestPlane->values[3] << std::endl;
            nearestPlane = secondNearestPlane;
            nearestPlaneCloud = secondNearestPlaneCloud;
            double maxIndoorDistance = 0.0;
            strongestOutdoor.clear();
            strongestIndoor.clear();
            lastOutdoorObstacle.clear();
            lastIndoor.clear();
            lastOutdoor.clear();
            maxIndoorDistance = 0.0;
            for (pcl::PointXYZI p : filteredstrongest.points) {
                double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
                double ringDegree = ringRad * 180 / PI;
                char *ringDegreeChar = (char *) malloc(32 * sizeof(char));
                sprintf(ringDegreeChar, "%.2f", ringDegree);
                string ringDegreeString(ringDegreeChar);
                free(ringDegreeChar);
                int ring = ringMap[ringDegreeString];
                int pIndex = int(
                        round((180 + atan2(p.y, p.x) * 180 / PI) /
                              0.16)) + 1;


                double distance = pcl::pointToPlaneDistanceSigned(p, nearestPlane->values[0], nearestPlane->values[1],
                                                                  nearestPlane->values[2], nearestPlane->values[3]);
                if ((pIndex > glassLeft && pIndex < glassRight && glassDown < ring && ring < glassUp) ||
                    (glassLeft > glassRight && (pIndex > glassLeft || pIndex < glassRight) && glassDown < ring &&
                     ring < glassUp)) {
                    if (distance > 0.05 && p.intensity > 0) {
                        strongestIndoor.push_back(p);
                    }
                } else if (p.intensity > 0) {
                    strongestIndoor.push_back(p);
                }
                if (distance > maxIndoorDistance) {
                    maxIndoorDistance = distance;
                }
            }

            for (pcl::PointXYZI p : filteredlast.points) {
                double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
                double ringDegree = ringRad * 180 / PI;
                char *ringDegreeChar = (char *) malloc(32 * sizeof(char));
                sprintf(ringDegreeChar, "%.2f", ringDegree);
                string ringDegreeString(ringDegreeChar);
                free(ringDegreeChar);
                int ring = ringMap[ringDegreeString];
                int pIndex = int(
                        round((180 + atan2(p.y, p.x) * 180 / PI) /
                              0.16)) + 1;

                double distance = pcl::pointToPlaneDistanceSigned(p, nearestPlane->values[0], nearestPlane->values[1],
                                                                  nearestPlane->values[2], nearestPlane->values[3]);
                if ((pIndex > glassLeft && pIndex < glassRight && glassDown < ring && ring < glassUp) ||
                    (glassLeft > glassRight && (pIndex > glassLeft || pIndex < glassRight) && glassDown < ring &&
                     ring < glassUp)) {
                    if (distance > 0.05) {
                        lastIndoor.push_back(p);
                    } else if (distance < -0.1) {
                        if (-distance > maxIndoorDistance) {
                            lastOutdoorObstacle.push_back(p);
                        } else {
                            lastOutdoor.push_back(p);
                        }
                    }
                }
            }
            printf("%d\n", lastOutdoorObstacle.points.size());
        }
        nearestPlane->values[2] = 0;
        pcl::PointCloud<pcl::PointXYZI> mirroredOutdoor;
        reflectTransform(lastOutdoor, mirroredOutdoor, *nearestPlane);
        for (int j = 0; j < mirroredOutdoor.points.size(); ++j) {
            pcl::PointXYZI p = mirroredOutdoor.points[j];
            double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
            double ringDegree = ringRad * 180 / PI;
            char *ringDegreeChar = (char *) malloc(32 * sizeof(char));
            sprintf(ringDegreeChar, "%.2f", ringDegree);
            string ringDegreeString(ringDegreeChar);
            free(ringDegreeChar);
            int ring = ringMap[ringDegreeString];
            pcl::PointXYZI zero;
            zero.x = 0;
            zero.y = 0;
            zero.z = 0;
            int pIndex = int(
                    round((180 + atan2(p.y, p.x) * 180 / PI) /
                          0.16)) + 1;
            if (pIndex > 0) {
                if ((pcl::geometry::distance(strongestordered[ring].points[pIndex], zero) -
                     pcl::geometry::distance(p, zero)) > 0.1) {
                    lastOutdoorObstacle.push_back(lastOutdoor.points[j]);
                }
            }
        }
        for (pcl::PointXYZI p :lastOutdoorObstacle) {
            double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
            double ringDegree = ringRad * 180 / PI;
            char *ringDegreeChar = (char *) malloc(32 * sizeof(char));
            sprintf(ringDegreeChar, "%.2f", ringDegree);
            string ringDegreeString(ringDegreeChar);
            free(ringDegreeChar);
            int ring = ringMap[ringDegreeString];
            int pIndex = int(
                    round((180 + atan2(p.y, p.x) * 180 / PI) /
                          0.16)) + 1;

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
//        pcl::PointCloud<pcl::PointXYZI>::Ptr nearestPlaneCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
//        nearestPlaneCloudPtr->points=nearestPlaneCloud.points;
//        pcl::ModelCoefficients::Ptr tempCoeff (new pcl::ModelCoefficients);
//        pcl::PointCloud<pcl::PointXYZI>::Ptr inlierTmp(new pcl::PointCloud<pcl::PointXYZI>);
//        /* Run RANSAC here with model plane */
//        pcl::PointIndices::Ptr inliers = getRansacPlaneCoeff(nearestPlaneCloudPtr, tempCoeff, inlierTmp,0.02,0.95);
        reflectTransform(Reflection, mirroredReflection, *nearestPlane);

    }

    /* Publish this nearest plane point cloud to ros */
    //green means glass, blue means outdoor obstacle, red means mirroredreflection, black means normal points
    PointCloud<PointXYZRGB> nearestPlaneCloudRGB;
    for (PointXYZI p: nearestPlaneCloud.points) {
        PointXYZRGB prgb;
        prgb.x = p.x;
        prgb.y = p.y;
        prgb.z = p.z;
        prgb.r = 0;
        prgb.g = 255;
        prgb.b = 0;
        nearestPlaneCloudRGB.points.push_back(prgb);
    }
    pcl::toROSMsg(nearestPlaneCloudRGB, GlassCloud2);
    GlassCloud2.header = strongestcloud.header;
    glassPC.publish(GlassCloud2);


    PointCloud<PointXYZRGB> strongestIndoorRGB;
    if (strongestIndoor.points.size() < 100) {
        //fallback solution for no windows detected to perform SLAM
        PointCloud<PointXYZI>::Ptr cloud_filtered(new PointCloud<PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filteredstrongestPtr(new pcl::PointCloud<pcl::PointXYZI>);
        filteredstrongestPtr->points = filteredstrongest.points;
        pcl::PassThrough<PointXYZI> intensitypass;
        intensitypass.setInputCloud(filteredstrongestPtr);
        intensitypass.setFilterFieldName("intensity");
        intensitypass.setFilterLimits(2, 255);
        intensitypass.filter(*cloud_filtered);
        for (PointXYZI p: cloud_filtered->points) {
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
        for (PointXYZI p: strongestIndoor.points) {
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
    strongestIndoorPC.publish(strongestIndoorCloud2);


//    PointCloud<PointXYZRGB> strongestOutdoorRGB;
//    copyPointCloud(strongestOutdoor, strongestOutdoorRGB);
//    pcl::toROSMsg(strongestOutdoorRGB, strongestOutdoorCloud2);
//    strongestOutdoorCloud2.header = strongestcloud.header;
//    strongestOutdoorPC.publish(strongestOutdoorCloud2);

//    PointCloud<PointXYZRGB> inlierCloudRGB;
//    for (pcl::PointXYZI p : filteredstrongest.points) {
//        double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
//        double ringDegree = ringRad * 180 / PI;
//        char *ringDegreeChar = (char *) malloc(32 * sizeof(char));
//        sprintf(ringDegreeChar, "%.2f", ringDegree);
//        string ringDegreeString(ringDegreeChar);
//        free(ringDegreeChar);
//        int ring = ringMap[ringDegreeString];
//        int pIndex = int(
//                round((180 + atan2(p.y, p.x) * 180 / PI) /
//                      0.16)) + 1;
//        if((pIndex>glassLeft && pIndex<glassRight && glassDown<ring &&ring<glassUp) || (glassLeft>glassRight && (pIndex>glassLeft || pIndex<glassRight) && glassDown<ring &&ring<glassUp))  {
//            PointXYZRGB prgb;
//            prgb.x = p.x;
//            prgb.y = p.y;
//            prgb.z = p.z;
//            prgb.r = 255;
//            prgb.g = 255;
//            prgb.b = 255;
//            inlierCloudRGB.points.push_back(prgb);
//        }
//    }
//    pcl::toROSMsg(inlierCloudRGB, inlierCloud2);
//    inlierCloud2.header = strongestcloud.header;
//    inlierPC.publish(inlierCloud2);

    PointCloud<PointXYZRGB> lastOutdoorObstacleRGB;
    for (PointXYZI p: lastOutdoorObstacle.points) {
        PointXYZRGB prgb;
        prgb.x = p.x;
        prgb.y = p.y;
        prgb.z = p.z;
        prgb.r = 0;
        prgb.g = 0;
        prgb.b = 255;
        lastOutdoorObstacleRGB.points.push_back(prgb);
    }
    pcl::toROSMsg(lastOutdoorObstacleRGB, lastOutdoorObstacleCloud2);
    lastOutdoorObstacleCloud2.header = strongestcloud.header;
    lastOutdoorObstaclePC.publish(lastOutdoorObstacleCloud2);

//    PointCloud<PointXYZRGB> ReflectionRGB;
//    copyPointCloud(Reflection, ReflectionRGB);
//    pcl::toROSMsg(ReflectionRGB, ReflectionCloud2);
//    ReflectionCloud2.header = strongestcloud.header;
//    ReflectionPC.publish(ReflectionCloud2);

    PointCloud<PointXYZRGB> mirroredReflectionRGB;
    for (PointXYZI p: mirroredReflection.points) {
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
    mirroredReflectionPC.publish(mirroredReflectionCloud2);


    PointCloud<PointXYZRGB> slamRGB;
    slamRGB += nearestPlaneCloudRGB;
    slamRGB += strongestIndoorRGB;
    slamRGB += lastOutdoorObstacleRGB;
    slamRGB += mirroredReflectionRGB;
    pcl::toROSMsg(slamRGB, slamCloud2);
    slamCloud2.header = strongestcloud.header;
    cout<<slamCloud2.header<<endl;
    slamPC.publish(slamCloud2);

    //start intendity peak detection
    std::vector<pcl::PointXYZI> peaks;
    int i = 16;
    while (i < strongestring[23].width - 16) {//peak>70 // intensitypeak from ring 21 to 25, middle at 23
        if (strongestring[23].points[i].intensity > 35
            && strongestring[23].points[i - 10].intensity < 50 && strongestring[23].points[i - 15].intensity < 30
            && strongestring[23].points[i + 10].intensity < 50 && strongestring[23].points[i + 15].intensity < 30) {
            peaks.push_back(strongestring[23].points[i]);
            i = i + 30;
        }
        i++;
    }
    pcl::PointCloud<pcl::PointXYZI> intensitypeaktmp;
    for (i = 0; i < peaks.size(); i++) {
        int index = int(round((180 + atan2(peaks[i].y, peaks[i].x) * 180 / PI) / 0.16)) + 1;
        pcl::PointXYZI max;
        max.intensity = -999;
        int maxi = 0;
        for (int j = index - 20; j <= index + 20; j++) {
            if (strongestordered[23].points[j].intensity > max.intensity) {
                max = strongestordered[23].points[j];
                maxi = j;
            }
        }
        if (
                strongestordered[23].points[maxi - 10].intensity < 60 &&
                strongestordered[23].points[maxi + 10].intensity < 60 &&
                strongestordered[23].points[maxi - 15].intensity < 30 &&
                strongestordered[23].points[maxi + 15].intensity < 30 &&
                strongestordered[21].points[maxi].intensity < 60 &&
                strongestordered[22].points[maxi].intensity < 100 &&
                strongestordered[24].points[maxi].intensity < 100 &&
                strongestordered[25].points[maxi].intensity < 60 &&
                strongestordered[21].points[maxi - 10].intensity < 30 &&
                strongestordered[22].points[maxi - 15].intensity < 30 &&
                strongestordered[24].points[maxi - 15].intensity < 30 &&
                strongestordered[25].points[maxi - 10].intensity < 30 &&
                strongestordered[21].points[maxi + 15].intensity < 30 &&
                strongestordered[24].points[maxi + 10].intensity < 30 &&
                strongestordered[22].points[maxi + 15].intensity < 30 &&
                strongestordered[25].points[maxi + 10].intensity < 30 &&
                sqrt(pow(strongestordered[23].points[maxi].x, 2) + pow(strongestordered[23].points[maxi].y, 2)) < 3) {
            for (int k = 21; k <= 25; ++k) {
                for (int j = maxi - 10; j <= maxi + 10; ++j) {
                    if (sqrt(pow(strongestordered[k].points[j].x, 2) + pow(strongestordered[k].points[j].y, 2)) < 3)
                        strongestordered[k].points[j].intensity = 255;
                }
            }

        }

    }
    for (int k = 21; k <= 25; ++k) {
        intensitypeaktmp += strongestordered[k];
    }

    sensor_msgs::PointCloud2 intensitypeak2;
    pcl::toROSMsg(intensitypeaktmp, intensitypeak2);
    intensitypeak2.header = strongestcloud.header;
    intensityline.publish(intensitypeak2);

    sensor_msgs::PointCloud2 filteredstrongest2;
    pcl::toROSMsg(filteredstrongest, filteredstrongest2);
    filteredstrongest2.header = strongestcloud.header;
    filteredstrongestpub.publish(filteredstrongest2);
    sensor_msgs::PointCloud2 filteredlast2;
    pcl::toROSMsg(filteredlast, filteredlast2);
    filteredlast2.header = lastcloud.header;
    filteredlastpub.publish(filteredlast2);
//    printf("end dualcheck\n");
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &strongest, const sensor_msgs::PointCloud2ConstPtr &last) {
    strongestcloud = *strongest;
    lastcloud = *last;
    pcl::concatenatePointCloud(*strongest, *last, dualcloud);
    dual.publish(dualcloud);
    pcl::PCLPointCloud2 strongestpcl;
    pcl_conversions::toPCL(*strongest, strongestpcl);
    converttorings(strongestpcl, *strongestcloud1d, strongestring, 32);
    pcl::PCLPointCloud2 lastpcl;
    pcl_conversions::toPCL(*last, lastpcl);
    converttorings(lastpcl, *lastcloud1d, lastring, 32);
    dualcheck();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Reflection_detection");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> substrongest(nh, "/left_velodyne/velodyne_points_strongest", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sublast(nh, "/left_velodyne/velodyne_points_last", 1);
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync(substrongest, sublast, 10);
    sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

    dual = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_dual", 1);
    filteredstrongestpub = nh.advertise<sensor_msgs::PointCloud2>("/filteredstrongest", 1);
    filteredlastpub = nh.advertise<sensor_msgs::PointCloud2>("/filteredlast", 1);
    intensityline = nh.advertise<sensor_msgs::PointCloud2>("/intensityline", 1);
    inlierPC = nh.advertise<sensor_msgs::PointCloud2>("/Debug/inlierPointCloud", 1);
    strongestIndoorPC = nh.advertise<sensor_msgs::PointCloud2>("/StrongestIndoorPointCloud", 1);
    strongestOutdoorPC = nh.advertise<sensor_msgs::PointCloud2>("/StrongestOutdoorPointCloud", 1);
    lastOutdoorObstaclePC = nh.advertise<sensor_msgs::PointCloud2>("/LastOutdoorObstaclePointCloud", 1);
    ReflectionPC = nh.advertise<sensor_msgs::PointCloud2>("/ReflectionPointCloud", 1);
    glassPC = nh.advertise<sensor_msgs::PointCloud2>("/GlassPointCloud", 1);
    mirroredReflectionPC = nh.advertise<sensor_msgs::PointCloud2>("/MirroredReflectionPointCloud", 1);
    slamPC = nh.advertise<sensor_msgs::PointCloud2>("/slamPointCloud", 1);


    ros::spin();
}