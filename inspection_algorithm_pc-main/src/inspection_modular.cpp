
#include<string>
#include<iostream>
#include<fstream>
#include <vector>
#include<memory> // NEW

#include <json/value.h> // read detected bounding box coordianates
#include <json/json.h>// read detected bounding box coordianates

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/affine.hpp>


#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h> // extract borders


#include <pcl/registration/icp.h>// ICP algorit

#include <pcl/filters/voxel_grid.h> // voxel grid downsampling

//#include <pcl/2d/morphology.h> // closing in pc
#include <pcl/filters/morphological_filter.h>// closing in pc

#include<pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h> // filter point cloud
#include <pcl/filters/crop_box.h> // crop bbs

// segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h> // RA
#include <pcl/sample_consensus/sac_model_line.h> // line segmentation

// Circle Detection in Point cloud
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/octree/octree_search.h> // octree search

#include <pcl/filters/statistical_outlier_removal.h> // removeal

#include <pcl/features/moment_of_inertia_estimation.h>// boundingBox

#include <pcl/features/normal_3d.h> // normal estimation
#include <pcl/features/integral_image_normal.h>

#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/surface/mls.h> // smoothen surface
#include <pcl/kdtree/kdtree_flann.h>



#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_hull.h>


#include <pcl/io/vtk_lib_io.h>


#include "data_type.h"


void filterPointCloudWithOrientedBB(const std::string& cloudPath, std::vector<Eigen::Vector3f>& area, const Eigen::Vector3f& marginXYZ){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud);

     std::cout << " Cloud size: " << cloud->size() <<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_area(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < area.size(); i++){
        cloud_area->points.emplace_back(area[i].x(),area[i].y(),area[i].z());
    }


     pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cloud_area);//default clould
    feature_extractor.compute ();
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    Eigen::Vector3f rot = rotational_matrix_OBB.eulerAngles(0,1,2);

    pcl::CropBox<pcl::PointXYZ> boxFilter;
     boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x- marginXYZ.x(), min_point_OBB.y - marginXYZ.y() , min_point_OBB.z - marginXYZ.z(), 1.0));
     boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x + marginXYZ.x(),  max_point_OBB.y +  marginXYZ.y() ,max_point_OBB.z +  marginXYZ.z(), 1.0));
     Eigen::Affine3f t;
     t.translation() = position;
     t.linear() = rotational_matrix_OBB;
     boxFilter.setTransform(t.inverse());

     boxFilter.setInputCloud(cloud);
     boxFilter.filter(*cloud_filtered);

     std::cout << " Cloud Filtered with oriented bounding box: " << cloud_filtered->size() <<std::endl;

    pcl::io::savePCDFileBinaryCompressed (cloudPath, *cloud_filtered);

}

void removeOutliers(const std::string& cloudPath, const int& k_neighbor, const float& distanceThreshold){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud);
    std::cout << " Cloud size: " << cloud->size() <<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_removal (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (k_neighbor);
    sor.setStddevMulThresh (distanceThreshold);
    sor.setNegative (false);
    sor.filter (*cloud_filtered_removal);

    pcl::io::savePCDFileBinaryCompressed (cloudPath, *cloud_filtered_removal);

    std::cout << " Cloud after outlier removal: " << cloud_filtered_removal->size() <<std::endl;

}

void createRangeImage(const std::string& cloudPath,
                     const std::string& rangeImagePath, 
                    const Eigen::Affine3f& sensorPose,
                    const float& angular_resolution,
                    const float& maxAngleWidth_,
                    const float& maxAngleHeight_,
                    const int& borderSize_){


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud);


    
    float angularResolution_x = (float) (angular_resolution * (M_PI/180.0f));  // ADAPT IT!!
    float angular_resolution_y = (float) (angular_resolution * (M_PI/180.0f));
    float maxAngleWidth     = (float) (maxAngleWidth_ * (M_PI/180.0f));  // 180.0 degree in radians
    float maxAngleHeight    = (float) (maxAngleHeight_ * (M_PI/180.0f));  // 180.0 degree in radians
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.0f;
    float minRange = 0.0f;
    int borderSize = borderSize_;

    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(*cloud, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    std::cout << " range image is created! "<< std::endl;
    cv::Mat range_map(rangeImage.height, rangeImage.width, CV_8UC3);
   
    rangeImage.setUnseenToMaxRange();
    // normalize
    float min, max;
    rangeImage.getMinMaxRanges(min,max);

    // create range map or binary image
        for (int h = 0; h < rangeImage.height; h++){
            for (int w = 0; w < rangeImage.width; w++){
                    if(isinf(rangeImage.at(w,h).range)){
                range_map.at<cv::Vec3b>(h,w)[0] = 0;
                range_map.at<cv::Vec3b>(h,w)[1] = 0;
                range_map.at<cv::Vec3b>(h,w)[2] = 0;
            }else{
                range_map.at<cv::Vec3b>(h,w)[0]= ((rangeImage.at(w,h).range) / max)*255; // Normalize for color image
                range_map.at<cv::Vec3b>(h,w)[1]= ((rangeImage.at(w,h).range) / max)*255;
                range_map.at<cv::Vec3b>(h,w)[2]= ((rangeImage.at(w,h).range) / max)*255;

            }
        }
    }
    if(!range_map.empty()){
        cv::imwrite(rangeImagePath,range_map);
        std::cout << " range image is saved! "<< std::endl;
    }else{
        std::cout << "range image is empty!" << std::endl;
    }
}



void createRangeImageForBorderExtraction(const std::string& cloudPath,
                                        const std::string& cloudBorderPath, 
                                        const Eigen::Affine3f& sensorPose,
                                        const float& angular_resolution,
                                        const float& maxAngleWidth_,
                                        const float& maxAngleHeight_,
                                        const int& borderSize_){


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud);


    
    float angularResolution_x = (float) (angular_resolution * (M_PI/180.0f));  // ADAPT IT!!
    float angular_resolution_y = (float) (angular_resolution * (M_PI/180.0f));
    float maxAngleWidth     = (float) (maxAngleWidth_ * (M_PI/180.0f));  // 180.0 degree in radians
    float maxAngleHeight    = (float) (maxAngleHeight_ * (M_PI/180.0f));  // 180.0 degree in radians
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.0f;
    float minRange = 0.0f;
    int borderSize = borderSize_;

    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(*cloud, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    
    rangeImage.setUnseenToMaxRange();
    std::cout << " Range image is created for border extraction! "<< std::endl;
    
    std::cout <<" Extracting border Points" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr border_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RangeImageBorderExtractor border_extractor (&rangeImage);
    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    border_extractor.compute(border_descriptions);

    for (int y=0; y< (int)rangeImage.height; ++y){
        for (int x=0; x< (int)rangeImage.width; ++x) {
            if (border_descriptions[y*rangeImage.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER]){
            pcl::PointXYZ p{rangeImage[y*rangeImage.width + x].x,rangeImage[y*rangeImage.width + x].y,rangeImage[y*rangeImage.width + x].z};
            border_points->points.emplace_back(p);
            }
        }
    }

    pcl::io::savePCDFileBinaryCompressed (cloudBorderPath, *border_points);
    std::cout << " Border Extaction is completed! " << border_points->points.size() << std::endl;
}

std::vector<float> detectHoleCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    std::vector<float> res;
    pcl::ModelCoefficients::Ptr coefficients_p (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_p (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg_p;
    // Optional
    seg_p.setOptimizeCoefficients (true);
    // Mandatory
    seg_p.setModelType (pcl::SACMODEL_PLANE);
    seg_p.setMethodType (pcl::SAC_RANSAC);
    seg_p.setDistanceThreshold (0.5);
    seg_p.setInputCloud (cloud);
    seg_p.segment (*inliers_p, *coefficients_p);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_p);
    extract.setNegative (false);
    extract.filter(*cloud_p);   

    std::cout << "Loaded width " << cloud_p->size() << " data points " << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_CIRCLE2D);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setRadiusLimits(0,5);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (1);

    seg.setInputCloud (cloud_p);
    seg.segment (*inliers, *coefficients);

    res.emplace_back(coefficients->values[0]);
    res.emplace_back(coefficients->values[1]);
    res.emplace_back(coefficients->values[2]);
    return res;
}


std::vector<std::vector<Eigen::Vector3f>> findClosestPointsWithOCCTree(std::vector<Eigen::Vector3f> targets, const int& k_neighbor){


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_targets(new pcl::PointCloud<pcl::PointXYZ>);
    for(const auto& t : targets){
            cloud_targets->points.emplace_back(t.x(),t.y(),t.z());
        }
    
    // Octree
     float resolution = 128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
    octree.setInputCloud (cloud_targets);
    octree.addPointsFromInputCloud ();
    int K = k_neighbor;
    std::vector<std::vector<Eigen::Vector3f>> all_neighbor_list;

    for (size_t i = 0; i < targets.size(); i++){
        std::vector<Eigen::Vector3f> neighbor_list;
        pcl::PointXYZ searchPoint{targets.at(i).x(),targets.at(i).y(),targets.at(i).z()};
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        for (size_t i = 0; i < k_neighbor; i++){
            if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
                Eigen::Vector3f neighbor{(*cloud_targets)[ pointIdxNKNSearch[1] ].x ,
                (*cloud_targets)[ pointIdxNKNSearch[1] ].y,
                (*cloud_targets)[ pointIdxNKNSearch[1] ].z};
                neighbor_list.emplace_back(neighbor);
            }       
        }
        all_neighbor_list.emplace_back(neighbor_list);
    }

    return all_neighbor_list;
}



Eigen::Quaternionf eulerToQuaternion(const float& yaw, const float& pitch, const float& roll) {
    // yaw (Z), pitch (Y), roll (X)
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    Eigen::Quaternionf q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;

    return q;
}

Eigen::Vector3f quartenionToEulerAngles(const Eigen::Quaternionf& q) {
    Eigen::Vector3f angles;
    // ZYX Convention
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    float cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles.x() = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles.y() = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.y() = std::asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    float cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles.z() = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}


 Eigen::Matrix3f eulerToRotation(float roll, float pitch, float yaw){
    // roll and pitch and yaw in radian
    float su = sin(roll);
    float cu = cos(roll);
    float sv = sin(pitch);
    float cv = cos(pitch);
    float sw = sin(yaw);
    float cw = cos(yaw);
    Eigen::Matrix3f Rot_matrix;
    Rot_matrix(0, 0) = cv*cw;
    Rot_matrix(0, 1) = su*sv*cw - cu*sw;
    Rot_matrix(0, 2) = su*sw + cu*sv*cw;
    Rot_matrix(1, 0) = cv*sw;
    Rot_matrix(1, 1) = cu*cw + su*sv*sw;
    Rot_matrix(1, 2) = cu*sv*sw - su*cw;
    Rot_matrix(2, 0) = -sv;
    Rot_matrix(2, 1) = su*cv;
    Rot_matrix(2, 2) = cu*cv;
    return Rot_matrix;
}


Eigen::Matrix4f calculateTransformationWithICP(const std::string& sourcePath,const std::string& targetPath, const int& iterations,const float& maxCorrespondenceDistance){

        pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
    
        pcl::io::loadPCDFile<pcl::PointXYZ> (sourcePath, *source);
         pcl::io::loadPCDFile<pcl::PointXYZ> (targetPath, *target);
    if(source->points.size() > 3 && target->points.size()>3){

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        int iterations = 10;
        icp.setInputSource (source);
        icp.setInputTarget (target);
        icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance);
        icp.setMaximumIterations (iterations);
        icp.align (*source);
        return icp.getFinalTransformation();
    }else{
        throw std::string("Number of points in the pc must be greate than 3");   
    }   
        
}