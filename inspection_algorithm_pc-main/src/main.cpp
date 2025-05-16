
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

#include "data_type.h"
#include "helper.h"

std::vector<drillTarget> startInspection(const std::vector<boundingBox>& BBs,
                                            const std::vector<drillTarget>& drillTargets_CAD,
                                            const params& params,
                                              const std::string& cloudPath);


std::vector<drillTarget> matchHoles(const std::vector<boundingBox>& BBs,
                                        const std::vector<drillTarget>& drillTargets,
                                        const params& params,
                                       const std::string& cloudPath);

// returns the final poses of drill targets and check edge Tolerances
std::vector<drillTarget> inspectDrillTargets(const std::vector<drillTarget>& drillTargets,
                                            const std::vector<drillTarget>& drillTargets_CAD,
                                            const params& params,
                                            const std::string& cloudPath);

std::vector<drillTarget> findCorrespondences(const std::vector<drillTarget>& drillTargets_CAD,
                                                            const std::vector<drillTarget>& detectedHoles,
                                                            const params& params );

// Given the detected Bounnding boxes and the corresponding range image, it segments the holes in the point cloud and returns the centers
std::vector<drillTarget>  extractHoles(const  std::vector<boundingBox>& BBs, const std::vector<drillTarget> drillTargets_cad,const std::string& auditFolder, const params& params);


std::vector<drillTarget>  extractHoles2DCircle(const std::vector<boundingBox>& BBs,
                                        const std::vector<drillTarget> drillTargets_cad,
                                        const std::string& cloudPath,
                                        const params& params);

std::vector<float> detect3DCirclePC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
//pcl::ModelCoefficients detect2DCirclePC_Coeff(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);


std::vector<float> detect2DCirclePC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered); // return center of the holes in point cloud

pcl::PointCloud<pcl::PointXYZ>::Ptr createRangeImageBorderExtaction(const std::vector<drillTarget>& drillTargets,
                                            const std::vector<drillTarget>& drillTargets_CAD,
                                            const params& params,
                                            const Eigen::Affine3f& profilometerPose,
                                            const std::string& cloudPath); // create another range iamge with lower resolution to extract borders

pcl::PointCloud<pcl::PointXYZ>::Ptr borderExtraction(const pcl::RangeImage& range_image); // returns border points on the range image
std::vector<float> segmentLine(pcl::PointCloud<pcl::PointXYZ>::Ptr border_points_cloud);

std::vector<drillTarget> checkEdgeTolerances(const std::vector<drillTarget>& drillTargets,
                                            const std::vector<drillTarget>& drillTargets_CAD,
                                            const params& params,
                                            const Eigen::Affine3f& profilometerPose,
                                            const std::string& cloudPath); // Creates a octree and finds the k closest neighbor

std::vector<drillTarget> checkCenterTolerances(const std::vector<drillTarget>& drillTargets,
                                                const params& params);

// It uses Icp to align the drill targets extracted from Cad model and the detected piloted&&fullsized holes
std::vector<drillTarget>  alignUnpilotedDrillTargets(const std::vector<drillTarget>& drillTargets,
                                                    const std::vector<drillTarget>& detectedHoles,
                                                    const params& params,
                                                     const std::string& cloudPath);

Eigen::Affine3f calculateProfilometerPose(const std::vector<drillTarget>& drillTargets_CAD,const params& params);
Eigen::Affine3f calculateProfilometerPoseBBMiddle(const std::vector<drillTarget>& drillTargets_CAD,const params& params);

void transformDrillTarges(std::vector<drillTarget>& drillTargets,const Eigen::Matrix4f& transform_matrix); // transfrom all drilltargets

Eigen::Vector3f getPointLocationInWorldFrame(const cv::Point2i& image_point, const float& range,const range_image& range_image_info);
cv::Point2f getPixelCoordinates(const Eigen::Vector3f& point3D,const range_image& range_image_info );
range_image readRangeImageFile(const std::string& auditFolder);
std::vector<drillTarget> setParameters(const std::vector<drillTarget> drillTargets_CAD);
Eigen::Matrix3f calculateQuartenions(const std::vector<float>& hole_coeffecients);
drillTarget setParameters3DCamera(const drillTarget drillTargets_CAD);




pcl::PointCloud<pcl::PointXYZ>::Ptr readPCDFile(const std::string& cloudPath); // read pcd
pcl::PointCloud<pcl::PointXYZ>::Ptr readPLYFile(const std::string& cloudPath); // read ply
float atanCalc (float y, float x); // helper function for 3d-2d mapping
std::vector<drillTarget> readPosesFromFile (const std::string& pathTOAlignedHolesLoc); // read poses
 Eigen::Matrix3f rotation_from_euler(float roll, float pitch, float yaw); //
Quaternion ToQuaternion(const float& yaw, const float& pitch, const float& roll); // yaw (Z), pitch (Y), roll (X)
EulerAngles ToEulerAngles(const Eigen::Quaternionf& q);

// ffilter the floor
void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
					const std::string& axis,
					const float (&limits)[2]);

pcl::RangeImage convertPointCloud2RangeMap(const std::vector<drillTarget>& drillTargets_cad,const std::string& cloudPath,const params& params);

cv::Mat createRangeImage(const std::vector<drillTarget>& drillTargets_cad,
                         const params& params,
                         const std::string& cloudPath,
                         const std::string& storagePath_rangeImage,
                         const std::string& storagePath_rangeFile);


void writeRangeImageFile(const pcl::RangeImage& rangeImage,const std::string& auditFolder,const Eigen::Affine3f& profilometerPose );
 cv::Mat createRangeImage3DCamera(const drillTarget& cameraPose,
                                    const params& params,
                                    const std::string& cloudPath,
                                    const std::string& storagePath_rangeImage);




    std::vector<drillTarget> startInspection3DCamera(const std::vector<boundingBox>& BBs,
                                               const std::vector<drillTarget>& drillTargets_CAD,
                                               const drillTarget& cameraPose,
                                               const params& params,
                                               const std::string& cloudPath);

    std::vector<drillTarget> inspectDrillTargets3DCamera(const std::vector<drillTarget>& drillTargets,
                                                 const std::vector<drillTarget>& drillTargets_CAD,
                                                const drillTarget& cameraPose,
                                                const params& params,
                                                const std::string& cloudPath);

    pcl::PointCloud<pcl::PointXYZ>::Ptr createRangeImageBorderExtaction3DCamera( const std::vector<drillTarget>& drillTargets,
                                                                         const std::vector<drillTarget>& drillTargets_CAD,
                                                                        const params& params,
                                                                        const Eigen::Affine3f& sensorPose_,
                                                                        const std::string& cloudPath);


    std::vector<drillTarget> matchHoles3DCamera(const std::vector<boundingBox>& BBs,
                                               const std::vector<drillTarget>& drillTargets_CAD,
                                               const drillTarget& cameraPose,
                                               const params& params,
                                               const std::string& cloudPath);


    std::vector<drillTarget>  extractHoles3DCamera(const std::vector<boundingBox>& BBs,
                                            const std::vector<drillTarget> drillTargets_cad,
                                            const drillTarget& cameraPose,
                                            const std::string& cloudPath,
                                            const params& params);

    std::vector<drillTarget> checkEdgeTolerances_3DCamera(const std::vector<drillTarget>& drillTargets,
                                                 const std::vector<drillTarget>& drillTargets_CAD,
                                                 const params& params,
                                                 const Eigen::Affine3f& sensorPose,
                                                 const std::string& cloudPath);
int main(int argc, char **argv) {


        // TO DO:

          /*

        1- write range image info needs correection
        2- center tolerances, find the ones on the same line
        3- set iteration number
        4- skip if there is no point--> add it after filtering producre
        */
        params params;

        params.auditFolder = "/home/oguz/vs_code/datasets/19-04-2022-lma/DENSE_15MMS/DENSE";
       //params.auditFolder = "/home/oguz/vs_code/datasets/tests_musp_11_04_2022/15MMS_50HZ";


        std::vector<int> audit_id{4,15,16,17,18,21,22,27,28,29,30,32,32,33};
       // std::vector<int> audit_id{0,1,2,3,4,6,7,8,9,11,12,13,14,16,17,18,20};
        //std::vector<int> audit_id{15,7,8,17,18,21,22,27,28,29,30,32,32,33};


        //  for (size_t j = 0; j < audit_id.size(); j++){
        //     // std::string pathToHoles_= params.auditFolder + cv::format("/profi_tcp_pose1.txt",audit_id.at(j));
        //     // std::vector<drillTarget> cameraPoses = readPosesFromFile(pathToHoles_);
        //     // drillTarget cameraPose;
        //     // for (const auto c : cameraPoses){
        //     //     if(c.cluster == audit_id.at(j))
        //     //         cameraPose = c;
        //     // }
        //     std::cout << "Range Image " << audit_id.at(j) <<std::endl;
        //    // std::string pathToHoles_= params.auditFolder +cv::format("/drill_tcp_pose_CLUSTER%d.txt",audit_id.at(j));
        //     std::string pathToHoles_= params.auditFolder +cv::format("/drill_tcp_pose1_19_04_2022.txt");

        //      std::vector<drillTarget> drillTargets_CAD = readPosesFromFile(pathToHoles_);
        //     std::vector<drillTarget> drillTargets_CAD_cluster; 
        //      for (const auto d : drillTargets_CAD){
        //         if(d.cluster == audit_id.at(j))
        //             drillTargets_CAD_cluster.emplace_back(d);
        //     }

        //     std::string cloudPath =  params.auditFolder + cv::format("/PointCloud_%d.pcd",audit_id.at(j));
        //     std::string storagePath_rangeImage = params.auditFolder + cv::format("/output_new_parameter/rangeImage_%d.png",audit_id.at(j));
        //     std::string storagePath_rangeFile = params.auditFolder + cv::format("/output_new_parameter/rangeImage_%d.txt",audit_id.at(j));

        //     cv::Mat image  = createRangeImage(drillTargets_CAD_cluster,params,cloudPath,storagePath_rangeImage,storagePath_rangeFile);
        //   //  cv::Mat iamge =  createRangeImage3DCamera(cameraPose,params, cloudPath, storagePath_rangeImage);
        // }



      



        
        for (int i = 0; i < audit_id.size(); i++){
            //read bounding boxes from detections
             std::cout << "Inspection " << audit_id.at(i) <<std::endl;

            std::vector<boundingBox> boundingBoxes;

            std::string cloudPath =  params.auditFolder + cv::format("/PointCloud_%d.pcd",audit_id.at(i));
            params.cloudPath = params.auditFolder + cv::format("/PointCloud_%d.pcd",audit_id.at(i));

            std::string pathToBBs= params.auditFolder + cv::format("/results/output_removal_15_rangeImage_%d.json",audit_id.at(i));
            std::ifstream predictions(pathToBBs, std::ifstream::binary);
            if(predictions.is_open()){
                Json::Value object;
                predictions >> object;
                for (auto obj : object){
                    boundingBox bb;
                    bb.cluster = audit_id.at(i);
                    bb.topLeft.x = obj["x1"].asInt();
                    bb.topLeft.y = obj["y1"].asInt();
                    bb.bottomRight.x = obj["x2"].asInt();
                    bb.bottomRight.y = obj["y2"].asInt();
                    bb.addMargin(10); // add margins
                    boundingBoxes.emplace_back(bb);
                }
            }

        
        //std::string pathToHoles_= params.auditFolder +cv::format("/drill_tcp_pose_CLUSTER%d.txt",audit_id.at(i));
          
        std::string pathToHoles_= params.auditFolder +cv::format("/drill_tcp_pose1_19_04_2022.txt");
        std::vector<drillTarget> drillTargets_CAD = readPosesFromFile(pathToHoles_);
       

        std::vector<drillTarget> drillTargets_CAD_cluster;
        for (size_t j = 0; j < drillTargets_CAD.size(); j++){
            if(drillTargets_CAD.at(j).cluster == audit_id.at(i) )
                drillTargets_CAD_cluster.emplace_back(drillTargets_CAD.at(j));
        }

        // std::string pathToCamera_= params.auditFolder + cv::format("/profi_tcp_pose1.txt",audit_id.at(i));
        // std::vector<drillTarget> cameraPoses = readPosesFromFile(pathToCamera_);
        // drillTarget cameraPose;
        // for (const auto c : cameraPoses){
        //     if(c.cluster == audit_id.at(i))
        //         cameraPose = c;
        // }
            


        //std::vector<drillTarget> inspected_drillTargets = startInspection3DCamera(boundingBoxes,drillTargets_CAD,cameraPose,params,cloudPath);
        std::vector<drillTarget> inspected_drillTargets = startInspection(boundingBoxes,drillTargets_CAD_cluster,params,cloudPath);

        // std::string writeFilePath =  params.auditFolder + cv::format("/output_drill/drill_targets_%d.txt",audit_id.at(i));
        // writeDetectionandInspection(writeFilePath,inspected_drillTargets);
     }






























        //     params.auditFolder = "/home/oguz/vs_code/datasets/3Dcamera/17_03_2022_X36/X36/dataset";

        // // std::vector<int> audit_id{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37};
        // //std::vector<int> audit_id{4,10,11,12,13,14,15,18,20,22,23,25,26,28,29,33};
        //  std::vector<int> audit_id{18};
        // for (int i = 0; i < audit_id.size(); i++){
        //     //read bounding boxes from detections

        //     std::vector<boundingBox> boundingBoxes;

        //     std::string cloudPath =  params.auditFolder + cv::format("/PointCloud_sparse%d.pcd",audit_id.at(i));
        //     params.cloudPath = params.auditFolder + cv::format("/PointCloud_sparse%d.pcd",audit_id.at(i));

        //     std::string pathToBBs= params.auditFolder + cv::format("/rangeImage_rangeImage_%d.json",audit_id.at(i));
        //     std::ifstream predictions(pathToBBs, std::ifstream::binary);
        //     if(predictions.is_open()){
        //         Json::Value object;
        //         predictions >> object;
        //         for (auto obj : object){
        //             boundingBox bb;
        //             bb.cluster = audit_id.at(i);
        //             bb.topLeft.x = obj["x1"].asInt();
        //             bb.topLeft.y = obj["y1"].asInt();
        //             bb.bottomRight.x = obj["x2"].asInt();
        //             bb.bottomRight.y = obj["y2"].asInt();
        //             bb.addMargin(10); // add margins
        //             boundingBoxes.emplace_back(bb);
        //         }
        //     }

        // std::string pathToHoles_= params.auditFolder +cv::format("/drill_tcp_pose1.txt",audit_id.at(i));
        // std::vector<drillTarget> drillTargets_CAD = readPosesFromFile(pathToHoles_);

        // std::vector<drillTarget> drillTargets_CAD_cluster;
        // for (size_t i = 0; i < drillTargets_CAD.size(); i++){
        //     if(drillTargets_CAD.at(i).cluster == 8 )
        //         drillTargets_CAD_cluster.emplace_back(drillTargets_CAD.at(i));
        // }

       
        
    //     std::vector<drillTarget> inspected_drillTargets = startInspection(boundingBoxes,drillTargets_CAD_cluster,params,cloudPath);


    //     std::vector<drillTarget> holes;
    //     for (size_t i = 0; i < inspected_drillTargets.size(); i++){
    //     if(inspected_drillTargets.at(i).misPrediction == false && inspected_drillTargets.at(i).differentCluster==false)
    //         holes.emplace_back(inspected_drillTargets.at(i));
        
    //     }
        
    //     std::string writeFilePath =  params.auditFolder + cv::format("/output/drill_targets_%d.txt",audit_id.at(i));
    //     writeDetectionandInspection(writeFilePath,holes);
    // }

    // std::string cloudPath = "/home/oguz/vs_code/datasets/ENSENSO_X36/NxView-angolo13_2.ply";
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	// pcl::PLYReader Reader;
	// Reader.read(cloudPath, *cloud);
	// if ( cloud->width <1){
    // 	//PCL_ERROR ("Couldn't read file point_cloud.ply \n");
    // 	throw std::runtime_error("Couldn't read cloud ply file \n");
	// }
    // pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Extracted Holes"));
    // viewer->setBackgroundColor (0, 0, 0);
    //  viewer->addCoordinateSystem (1.0);

        
    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    // viewer->addPointCloud<pcl::PointXYZRGB> (cloud,rgb, "bb");
    
    
    // while (!viewer->wasStopped ()) {viewer->spinOnce ();}    

  

    return 0;
}

 std::vector<drillTarget> startInspection(const std::vector<boundingBox>& BBs,
                                            const std::vector<drillTarget>& drillTargets_CAD,
                                            const params& params,
                                            const std::string& cloudPath){
        std::cout << "=====> Starting Inpection Cluster " << drillTargets_CAD.front().cluster << std::endl;

        std::cout << "=====> Matching Holes" << std::endl;

        std::vector<drillTarget> allDrillTargets = matchHoles(BBs,drillTargets_CAD,params,cloudPath);

        showDrillTargets(allDrillTargets,cloudPath);

        std::cout << "=====> Inspecting  Holes" << std::endl;

        std::vector<drillTarget> allDrillTargets_inspected = inspectDrillTargets(allDrillTargets,drillTargets_CAD,params,cloudPath);

        //showNormals (allDrillTargets_inspected,cloudPath);

        return allDrillTargets_inspected;
    }

 std::vector<drillTarget> matchHoles(const std::vector<boundingBox>& BBs,
                                            const std::vector<drillTarget>& drillTargets_CAD,
                                            const params& params,
                                            const std::string& cloudPath){

    std::cout << "=====> Launch drill target characteristics examination!!! " << std::endl;    

    std::vector<drillTarget> res;

    std::vector<drillTarget> drillTargetsset_CAD = setParameters(drillTargets_CAD);

    std::cout << "=====> Parameters set " << std::endl;

    if(!BBs.empty()){
         std::cout << "=====> Extracting holes in Bounding boxes " << std::endl;


        std::vector<drillTarget> detectedHoles = extractHoles(BBs,drillTargetsset_CAD,cloudPath,params); // extract hole centers and Labels; unpiloted,piloted,fullsized

        //std::vector<drillTarget> detectedHoles = extractHoles2DCircle(BBs,drillTargetsset_CAD,cloudPath,params); // extract hole centers and Labels; unpiloted,piloted,fullsized

        std::cout << "=====> Finding corresponding points " << std::endl;

        std::vector<drillTarget> detectedHoles_matched = findCorrespondences(drillTargetsset_CAD,detectedHoles, params); // find the correspondence between cad targets&detected targets discard misprediction

           std::cout << "=====> Applying transformation to unpiloted holes " << std::endl;

        std::vector<drillTarget> allDrillTargets = alignUnpilotedDrillTargets(drillTargetsset_CAD,detectedHoles_matched,params,cloudPath); // Apply transformation

        for (const auto dT : allDrillTargets){res.emplace_back(dT);}

    }
    else{
        std::cout << "=====> No detection found " << std::endl;

        for (auto& dT :  drillTargetsset_CAD){
            dT.unpiloted = true;
            dT.position_aligned = dT.position;
            dT.quartenion_cad = dT.quartenion;
            res.emplace_back(dT);}
    }

    std::cout << "=====> Drill target characteristics examination is over!!! " << std::endl;    

    return res;
}
std::vector<drillTarget> setParameters(const std::vector<drillTarget> drillTargets_CAD){
    std::vector<drillTarget> res(drillTargets_CAD);

    for (size_t i = 0; i < res.size(); i++){
            EulerAngles ea = ToEulerAngles(res.at(i).quartenion);
            res.at(i).rotation_matrix = rotation_from_euler(ea.roll,ea.pitch,ea.yaw);
            res.at(i).homogeneousMat.linear() = res.at(i).rotation_matrix;
            res.at(i).homogeneousMat.translation() = res.at(i).position;
    }
    return res;
}

/*

std::vector<drillTarget>  extractHoles(const std::vector<boundingBox>& BBs,
                                        const std::vector<drillTarget> drillTargets_cad,
                                        const std::string& cloudPath,
                                        const params& params){



    std::vector<drillTarget> detectedHoles;
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
        throw std::runtime_error("Couldn't read cloud pcd file \n");
        }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_locations(new pcl::PointCloud<pcl::PointXYZ>);
     for (size_t i = 0; i < drillTargets_cad.size(); i++){
        cad_locations->points.emplace_back(drillTargets_cad.at(i).position[0],drillTargets_cad.at(i).position[1],drillTargets_cad.at(i).position[2]);
        }

 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cad_locations);//default clould
    feature_extractor.compute ();
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getAABB(min_point_AABB,max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
   Eigen::Quaternionf quat(rotational_matrix_OBB);
    Eigen::Vector3f rot = rotational_matrix_OBB.eulerAngles(0,1,2);
     pcl::CropBox<pcl::PointXYZ> boxFilter;
     boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x- params.marginBBXaxis, min_point_OBB.y - params.marginBBYaxis , min_point_OBB.z - params.marginBBZaxis, 1.0));
     boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x + params.marginBBXaxis,  max_point_OBB.y +  params.marginBBYaxis ,max_point_OBB.z +  params.marginBBZaxis, 1.0));
     Eigen::Affine3f t;
     t.translation() = position;
     t.linear() = rotational_matrix_OBB;
     boxFilter.setTransform(t.inverse());

    // boxFilter.setRotation(quat.toRotationMatrix().eulerAngles(1,0,2));
    // boxFilter.setTranslation(position);
     boxFilter.setInputCloud(cloud);
     boxFilter.filter(*cloud_filtered);

     std::cout << " Cloud Filtered with oriented bounding box: " <<cloud_filtered->size() <<std::endl;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_removal (new pcl::PointCloud<pcl::PointXYZ>);

    // // if(params.modeOutLierRemovalActive == true){
    // //     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // //     sor.setInputCloud (cloud_filtered);
    // //     sor.setMeanK (params.meanK);
    // //     sor.setStddevMulThresh (params.setStddevMulThresh);
    // //     sor.setNegative (false);
    // //     sor.filter (*cloud_filtered_removal);
    // // }

    // std::cout << " Cloud after removal outliers: " <<cloud_filtered_removal->size() <<std::endl;
    std::vector<Eigen::Vector3f> positions_cicle_extractions;
    std::vector<Eigen::Vector3f> positions_centroid;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_all(new pcl::PointCloud<pcl::PointXYZ>); // for visualization//#######################
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_all_Real(new pcl::PointCloud<pcl::PointXYZ>); // for visualization//#######################
        std::vector<pcl::PointXYZ> add_lines; // for visualization//#######################
    std::vector<std::vector<float>> coefficient_all;
    //pcl::io::savePCDFileBinaryCompressed ("/home/oguz/vs_code/datasets/tests_musp_15_02_2022/test_pcd.pcd", *cloud_filtered);
    if(!cloud_filtered->points.empty()){


    
    float angularResolution_x = (float) (params.angular_resolution * (M_PI/180.0f));  // ADAPT IT!!
    float angular_resolution_y = (float) (params.angular_resolution * (M_PI/180.0f));
    float maxAngleWidth     = (float) (params.maxAngleWidth * (M_PI/180.0f));  // 180.0 degree in radians
    float maxAngleHeight    = (float) (params.maxAngleHeight * (M_PI/180.0f));  // 180.0 degree in radians

    Eigen::Affine3f sensorPose = calculateProfilometerPose(drillTargets_cad,params);

    
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.0f;
    float minRange = 0.0f;
    int borderSize = params.borderSize;

    pcl::RangeImage rangeImage;
    //Creating Range image


    
        
    rangeImage.createFromPointCloud(*cloud_filtered, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    std::cout << " range image is ready! "<< std::endl;
       
        //pcl::RangeImage rangeImage = convertPointCloud2RangeMap(drillTargets_cad,cloudPath,params);

        for (int k = 0; k < BBs.size(); k++){
            std::cout << "Extracting Bounding Box: " << k << std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB(new pcl::PointCloud<pcl::PointXYZ>);
                for (int u = BBs.at(k).topLeft.y; u < BBs.at(k).bottomRight.y; u++){
                    for (int v = BBs.at(k).topLeft.x; v <  BBs.at(k).bottomRight.x; v++){
                        if(isinf(rangeImage.at(v,u).range)){
                    }else{
                        pcl::PointXYZ p{rangeImage.at(v,u).x,rangeImage.at(v,u).y,rangeImage.at(v,u).z};
                        cloud_BB->points.emplace_back(p);
                        cloud_BB_all->points.emplace_back(p); // for visualization//#######################
                        }

                }
            }

            //NEw
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_real (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cloud_BB);//default clould
    feature_extractor.compute ();
    //pcl::PointXYZ min_point_AABB;
    //pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    //feature_extractor.getAABB(min_point_AABB,max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
     pcl::CropBox<pcl::PointXYZ> boxFilter;
     boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x, min_point_OBB.y , min_point_OBB.z - 5, 1.0));
     boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x,  max_point_OBB.y  ,max_point_OBB.z + 5 , 1.0));
     Eigen::Affine3f t;
     t.translation() = position;
     t.linear() = rotational_matrix_OBB;
     boxFilter.setTransform(t.inverse());
  
     boxFilter.setInputCloud(cloud_filtered);
     boxFilter.filter(*cloud_BB_real);
     for (size_t i = 0; i < cloud_BB_real->points.size(); i++){
        cloud_BB_all_Real->points.emplace_back(cloud_BB_real->points[i]);
     }
     
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_BB_real,centroid);
    Eigen::Vector3f pos_centroid{centroid.x(),centroid.y(),centroid.z()};

    std::cout << " cloud with the BB created! "<< std::endl;

    drillTarget detectedHole;
    detectedHole.BB = BBs.at(k);
    detectedHole.position_aligned = pos_centroid;
    detectedHole.cluster = drillTargets_cad.front().cluster;

    int width = detectedHole.BB.bottomRight.x - detectedHole.BB.topLeft.x ;
    int height= detectedHole.BB.bottomRight.y - detectedHole.BB.topLeft.y ;
    std::cout << width << "," << height << std::endl;
    if(std::abs(width) > params.fullSizeHole || std::abs(height) > params.fullSizeHole) // this value is critical to decide whether it is full or piloted
        detectedHole.fullSized =true;
    else
        detectedHole.piloted =true;


    //pcl::ModelCoefficients coeffient_show(detect3DCirclePC_Coeff(cloud_BB));
    std::vector<float> hole_coeeffecients(detect3DCirclePC(cloud_BB));
    coefficient_all.emplace_back(hole_coeeffecients);
    if(!hole_coeeffecients.empty()){
        Eigen::Vector3f pos{hole_coeeffecients.at(0),hole_coeeffecients.at(1),hole_coeeffecients.at(2)};
        Eigen::Vector3f nor{hole_coeeffecients.at(4),hole_coeeffecients.at(5),hole_coeeffecients.at(6)};
        drillTarget detectedHole;
        detectedHole.normals = nor;
        detectedHole.detectedHoleDiameter = hole_coeeffecients.at(3) * 2;
        
        std::vector<float> coeff;
        coeff.emplace_back(hole_coeeffecients.at(4));
        coeff.emplace_back(hole_coeeffecients.at(5));
        coeff.emplace_back(hole_coeeffecients.at(6));
        Eigen::Matrix3f rotation_matrix_ = calculateQuartenions(coeff);
        Eigen::Quaternionf q(rotation_matrix_);
        detectedHole.quartenion = q;

                        //#######################
            // for visualization
        pcl::PointXYZ p1{hole_coeeffecients.at(0) + (hole_coeeffecients.at(4) * -5),
        hole_coeeffecients.at(1) + (hole_coeeffecients.at(5) * -5),
        hole_coeeffecients.at(2) + (hole_coeeffecients.at(6) * -5)};
        pcl::PointXYZ p2{hole_coeeffecients.at(0) + (hole_coeeffecients.at(4) * +5),
        hole_coeeffecients.at(1) + (hole_coeeffecients.at(5) * +5),
        hole_coeeffecients.at(2) + (hole_coeeffecients.at(6) * +5)};
        add_lines.emplace_back(p1);
        add_lines.emplace_back(p2);
    
        positions_cicle_extractions.emplace_back(pos);
        positions_centroid.emplace_back(pos_centroid);
        
        // for visualization
        //#######################
        }else{
                std::cout << " Hole normals can not be extracted " <<std::endl;
            }
        detectedHoles.emplace_back(detectedHole);

        }

    }else{
        std::cout << "Cloud is empty after filtering, no extraction is possible" << std::endl;
    }

    std::cout << "Extraction is done! Total number detected holes is: " << detectedHoles.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_detected (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < detectedHoles.size(); i++){
            pilot_holes_detected->points.emplace_back(detectedHoles.at(i).position_aligned[0],detectedHoles.at(i).position_aligned[1],detectedHoles.at(i).position_aligned[2]);
    }  
   // showCircles(cloud_BB_all,params.cloudPath,coefficient_all);
    //showExtractedHoles(cloud_BB_all,params.cloudPath,add_lines);
    //extractedDrillTargets(pilot_holes_detected,params.cloudPath);
    showCenters(positions_cicle_extractions,positions_centroid,params.cloudPath,cloud_BB_all_Real);
     showCenters(positions_cicle_extractions,positions_centroid,params.cloudPath,cloud_BB_all);




    return detectedHoles;
}
*/

std::vector<drillTarget>  extractHoles(const std::vector<boundingBox>& BBs,
                                        const std::vector<drillTarget> drillTargets_cad,
                                        const std::string& cloudPath,
                                        const params& params){



    std::vector<drillTarget> detectedHoles;
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
        throw std::runtime_error("Couldn't read cloud pcd file \n");
        }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_locations(new pcl::PointCloud<pcl::PointXYZ>);
     for (size_t i = 0; i < drillTargets_cad.size(); i++){
        cad_locations->points.emplace_back(drillTargets_cad.at(i).position[0],drillTargets_cad.at(i).position[1],drillTargets_cad.at(i).position[2]);
        }

 
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//      pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
//     feature_extractor.setInputCloud (cad_locations);//default clould
//     feature_extractor.compute ();
//     pcl::PointXYZ min_point_AABB;
//     pcl::PointXYZ max_point_AABB;
//     pcl::PointXYZ min_point_OBB;
//     pcl::PointXYZ max_point_OBB;
//     pcl::PointXYZ position_OBB;
//     Eigen::Matrix3f rotational_matrix_OBB;
//     feature_extractor.getAABB(min_point_AABB,max_point_AABB);
//     feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
//     Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
//    Eigen::Quaternionf quat(rotational_matrix_OBB);
//     Eigen::Vector3f rot = rotational_matrix_OBB.eulerAngles(0,1,2);
//      pcl::CropBox<pcl::PointXYZ> boxFilter;
//      boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x- params.marginBBXaxis, min_point_OBB.y - params.marginBBYaxis , min_point_OBB.z - params.marginBBZaxis, 1.0));
//      boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x + params.marginBBXaxis,  max_point_OBB.y +  params.marginBBYaxis ,max_point_OBB.z +  params.marginBBZaxis, 1.0));
//      Eigen::Affine3f t;
//      t.translation() = position;
//      t.linear() = rotational_matrix_OBB;
//      boxFilter.setTransform(t.inverse());

//     // boxFilter.setRotation(quat.toRotationMatrix().eulerAngles(1,0,2));
//     // boxFilter.setTranslation(position);
//      boxFilter.setInputCloud(cloud);
//      boxFilter.filter(*cloud_filtered);

//      std::cout << " Cloud Filtered with oriented bounding box: " <<cloud_filtered->size() <<std::endl;


    std::vector<Eigen::Vector3f> positions_cicle_extractions;
    std::vector<Eigen::Vector3f> positions_centroid;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_all(new pcl::PointCloud<pcl::PointXYZ>); // for visualization//#######################
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_all_Real(new pcl::PointCloud<pcl::PointXYZ>); // for visualization//#######################
        std::vector<pcl::PointXYZ> add_lines; // for visualization//#######################
    std::vector<std::vector<float>> coefficient_all;
    //pcl::io::savePCDFileBinaryCompressed ("/home/oguz/vs_code/datasets/tests_musp_15_02_2022/test_pcd.pcd", *cloud_filtered);
    if(!cloud->points.empty()){


    
    float angularResolution_x = (float) (params.angular_resolution * (M_PI/180.0f));  // ADAPT IT!!
    float angular_resolution_y = (float) (params.angular_resolution * (M_PI/180.0f));
    float maxAngleWidth     = (float) (params.maxAngleWidth * (M_PI/180.0f));  // 180.0 degree in radians
    float maxAngleHeight    = (float) (params.maxAngleHeight * (M_PI/180.0f));  // 180.0 degree in radians

    Eigen::Affine3f sensorPose = calculateProfilometerPose(drillTargets_cad,params);

    
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.0f;
    float minRange = 0.0f;
    int borderSize = params.borderSize;

    pcl::RangeImage rangeImage;
    //Creating Range image


    
        
    rangeImage.createFromPointCloud(*cloud, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    std::cout << " range image is ready! "<< std::endl;
       
        //pcl::RangeImage rangeImage = convertPointCloud2RangeMap(drillTargets_cad,cloudPath,params);

        for (int k = 0; k < BBs.size(); k++){
            std::cout << "Extracting Bounding Box: " << k << std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB(new pcl::PointCloud<pcl::PointXYZ>);
                for (int u = BBs.at(k).topLeft.y; u < BBs.at(k).bottomRight.y; u++){
                    for (int v = BBs.at(k).topLeft.x; v <  BBs.at(k).bottomRight.x; v++){
                        if(isinf(rangeImage.at(v,u).range)){
                    }else{
                        pcl::PointXYZ p{rangeImage.at(v,u).x,rangeImage.at(v,u).y,rangeImage.at(v,u).z};
                        cloud_BB->points.emplace_back(p);
                        cloud_BB_all->points.emplace_back(p); // for visualization//#######################
                        }

                }
            }

            //NEw
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_real (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cloud_BB);//default clould
    feature_extractor.compute ();
    //pcl::PointXYZ min_point_AABB;
    //pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    //feature_extractor.getAABB(min_point_AABB,max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
     pcl::CropBox<pcl::PointXYZ> boxFilter;
     boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x, min_point_OBB.y , min_point_OBB.z - 5, 1.0));
     boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x,  max_point_OBB.y  ,max_point_OBB.z + 5 , 1.0));
     Eigen::Affine3f t;
     t.translation() = position;
     t.linear() = rotational_matrix_OBB;
     boxFilter.setTransform(t.inverse());
  
     boxFilter.setInputCloud(cloud);
     boxFilter.filter(*cloud_BB_real);
     for (size_t i = 0; i < cloud_BB_real->points.size(); i++){
        cloud_BB_all_Real->points.emplace_back(cloud_BB_real->points[i]);
     }
     
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_BB_real,centroid);
    Eigen::Vector3f pos_centroid{centroid.x(),centroid.y(),centroid.z()};

    std::cout << " cloud with the BB created! "<< std::endl;

    drillTarget detectedHole;
    detectedHole.BB = BBs.at(k);
    detectedHole.position_aligned = pos_centroid;
    detectedHole.cluster = drillTargets_cad.front().cluster;

     int width = detectedHole.BB.bottomRight.x - detectedHole.BB.topLeft.x ;
     int height= detectedHole.BB.bottomRight.y - detectedHole.BB.topLeft.y ;
     std::cout << width << "," << height << std::endl;
    if(std::abs(width) >= params.fullSizeHole || std::abs(height) >= params.fullSizeHole) // this value is critical to decide whether it is full or piloted
        detectedHole.fullSized =true;
    else
        detectedHole.piloted =true;


    //pcl::ModelCoefficients coeffient_show(detect3DCirclePC_Coeff(cloud_BB));
    std::vector<float> hole_coeeffecients(detect3DCirclePC(cloud_BB));
    std::cout << "diameter: " << (hole_coeeffecients.at(3) * 2) <<std::endl;
    // if((hole_coeeffecients.at(3) * 2) > params.diameteFullSized) // this value is critical to decide whether it is full or piloted
    //     detectedHole.fullSized =true;
    // else
    //     detectedHole.piloted =true;
    coefficient_all.emplace_back(hole_coeeffecients);
    if(!hole_coeeffecients.empty()){
        Eigen::Vector3f pos{hole_coeeffecients.at(0),hole_coeeffecients.at(1),hole_coeeffecients.at(2)};
      //  Eigen::Vector3f nor{hole_coeeffecients.at(4),hole_coeeffecients.at(5),hole_coeeffecients.at(6)};
        drillTarget detectedHole;
       // detectedHole.normals = nor;
        detectedHole.detectedHoleDiameter = hole_coeeffecients.at(3) * 2;
        
        // std::vector<float> coeff;
        // coeff.emplace_back(hole_coeeffecients.at(4));
        // coeff.emplace_back(hole_coeeffecients.at(5));
        // coeff.emplace_back(hole_coeeffecients.at(6));
        // Eigen::Matrix3f rotation_matrix_ = calculateQuartenions(coeff);
        // Eigen::Quaternionf q(rotation_matrix_);
        // detectedHole.quartenion = q;

                        //#######################
        //     // for visualization
        // pcl::PointXYZ p1{hole_coeeffecients.at(0) + (hole_coeeffecients.at(4) * -5),
        // hole_coeeffecients.at(1) + (hole_coeeffecients.at(5) * -5),
        // hole_coeeffecients.at(2) + (hole_coeeffecients.at(6) * -5)};
        // pcl::PointXYZ p2{hole_coeeffecients.at(0) + (hole_coeeffecients.at(4) * +5),
        // hole_coeeffecients.at(1) + (hole_coeeffecients.at(5) * +5),
        // hole_coeeffecients.at(2) + (hole_coeeffecients.at(6) * +5)};
        // add_lines.emplace_back(p1);
        // add_lines.emplace_back(p2);
    
        positions_cicle_extractions.emplace_back(pos);
        positions_centroid.emplace_back(pos_centroid);
        
        // for visualization
        //#######################
        }else{
                std::cout << " Hole normals can not be extracted " <<std::endl;
            }
        detectedHoles.emplace_back(detectedHole);

        }

    }else{
        std::cout << "Cloud is empty after filtering, no extraction is possible" << std::endl;
    }

    std::cout << "Extraction is done! Total number detected holes is: " << detectedHoles.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_detected (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < detectedHoles.size(); i++){
            pilot_holes_detected->points.emplace_back(detectedHoles.at(i).position_aligned[0],detectedHoles.at(i).position_aligned[1],detectedHoles.at(i).position_aligned[2]);
    }  
   // showCircles(cloud_BB_all,params.cloudPath,coefficient_all);
    //showExtractedHoles(cloud_BB_all,params.cloudPath,add_lines);
    //extractedDrillTargets(pilot_holes_detected,params.cloudPath);
    showCenters(positions_cicle_extractions,positions_centroid,params.cloudPath,cloud_BB_all_Real);
     showCenters(positions_cicle_extractions,positions_centroid,params.cloudPath,cloud_BB_all);




    return detectedHoles;
}

std::vector<drillTarget>  extractHoles2DCircle(const std::vector<boundingBox>& BBs,
                                        const std::vector<drillTarget> drillTargets_cad,
                                        const std::string& cloudPath,
                                        const params& params){



    std::vector<drillTarget> detectedHoles;
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
        throw std::runtime_error("Couldn't read cloud pcd file \n");
        }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_locations(new pcl::PointCloud<pcl::PointXYZ>);
     for (size_t i = 0; i < drillTargets_cad.size(); i++){
        cad_locations->points.emplace_back(drillTargets_cad.at(i).position[0],drillTargets_cad.at(i).position[1],drillTargets_cad.at(i).position[2]);
        }

 
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//      pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
//     feature_extractor.setInputCloud (cad_locations);//default clould
//     feature_extractor.compute ();
//     pcl::PointXYZ min_point_AABB;
//     pcl::PointXYZ max_point_AABB;
//     pcl::PointXYZ min_point_OBB;
//     pcl::PointXYZ max_point_OBB;
//     pcl::PointXYZ position_OBB;
//     Eigen::Matrix3f rotational_matrix_OBB;
//     feature_extractor.getAABB(min_point_AABB,max_point_AABB);
//     feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
//     Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
//    Eigen::Quaternionf quat(rotational_matrix_OBB);
//     Eigen::Vector3f rot = rotational_matrix_OBB.eulerAngles(0,1,2);
//      pcl::CropBox<pcl::PointXYZ> boxFilter;
//      boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x- params.marginBBXaxis, min_point_OBB.y - params.marginBBYaxis , min_point_OBB.z - params.marginBBZaxis, 1.0));
//      boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x + params.marginBBXaxis,  max_point_OBB.y +  params.marginBBYaxis ,max_point_OBB.z +  params.marginBBZaxis, 1.0));
//      Eigen::Affine3f t;
//      t.translation() = position;
//      t.linear() = rotational_matrix_OBB;
//      boxFilter.setTransform(t.inverse());

//     // boxFilter.setRotation(quat.toRotationMatrix().eulerAngles(1,0,2));
//     // boxFilter.setTranslation(position);
//      boxFilter.setInputCloud(cloud);
//      boxFilter.filter(*cloud_filtered);

     std::cout << " Cloud loaded with : " <<cloud->size() << "points" <<std::endl;


    std::vector<Eigen::Vector3f> positions_cicle_extractions;
    std::vector<Eigen::Vector3f> positions_centroid;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_all(new pcl::PointCloud<pcl::PointXYZ>); // for visualization//#######################
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_all_Real(new pcl::PointCloud<pcl::PointXYZ>); // for visualization//#######################
        std::vector<pcl::PointXYZ> add_lines; // for visualization//#######################
    std::vector<std::vector<float>> coefficient_all;
    //pcl::io::savePCDFileBinaryCompressed ("/home/oguz/vs_code/datasets/tests_musp_15_02_2022/test_pcd.pcd", *cloud_filtered);
    if(!cloud->points.empty()){


    
    float angularResolution_x = (float) (params.angular_resolution * (M_PI/180.0f));  // ADAPT IT!!
    float angular_resolution_y = (float) (params.angular_resolution * (M_PI/180.0f));
    float maxAngleWidth     = (float) (params.maxAngleWidth * (M_PI/180.0f));  // 180.0 degree in radians
    float maxAngleHeight    = (float) (params.maxAngleHeight * (M_PI/180.0f));  // 180.0 degree in radians

    Eigen::Affine3f sensorPose = calculateProfilometerPose(drillTargets_cad,params);

    
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.0f;
    float minRange = 0.0f;
    int borderSize = params.borderSize;

    pcl::RangeImage rangeImage;
    //Creating Range image


    
        
    rangeImage.createFromPointCloud(*cloud, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    std::cout << " range image is ready! "<< std::endl;
       
        //pcl::RangeImage rangeImage = convertPointCloud2RangeMap(drillTargets_cad,cloudPath,params);

        for (int k = 0; k < BBs.size(); k++){
            std::cout << "Extracting Bounding Box: " << k << std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB(new pcl::PointCloud<pcl::PointXYZ>);
                for (int u = BBs.at(k).topLeft.y; u < BBs.at(k).bottomRight.y; u++){
                    for (int v = BBs.at(k).topLeft.x; v <  BBs.at(k).bottomRight.x; v++){
                        if(isinf(rangeImage.at(v,u).range)){
                    }else{
                        pcl::PointXYZ p{rangeImage.at(v,u).x,rangeImage.at(v,u).y,rangeImage.at(v,u).z};
                        cloud_BB->points.emplace_back(p);
                        cloud_BB_all->points.emplace_back(p); // for visualization//#######################
                        }

                }
            }

            //NEw
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_real (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cloud_BB);//default clould
    feature_extractor.compute ();

    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
     pcl::CropBox<pcl::PointXYZ> boxFilter;
     boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x, min_point_OBB.y , min_point_OBB.z - 5, 1.0));
     boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x,  max_point_OBB.y  ,max_point_OBB.z + 5 , 1.0));
     Eigen::Affine3f t;
     t.translation() = position;
     t.linear() = rotational_matrix_OBB;
     boxFilter.setTransform(t.inverse());
  
     boxFilter.setInputCloud(cloud);
     boxFilter.filter(*cloud_BB_real);
     for (size_t i = 0; i < cloud_BB_real->points.size(); i++){
        cloud_BB_all_Real->points.emplace_back(cloud_BB_real->points[i]);
     }
     
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_BB_real,centroid);
    Eigen::Vector3f pos_centroid{centroid.x(),centroid.y(),centroid.z()};

    std::cout << " cloud with the BB created! "<< std::endl;



    //pcl::ModelCoefficients coeffient_show(detect3DCirclePC_Coeff(cloud_BB));
    std::vector<float> hole_coeeffecients(detect2DCirclePC(cloud_BB));
    coefficient_all.emplace_back(hole_coeeffecients);



    if(!hole_coeeffecients.empty()){
        Eigen::Vector3f pos{hole_coeeffecients.at(0),hole_coeeffecients.at(1),centroid.z()};
      //  Eigen::Vector3f nor{hole_coeeffecients.at(4),hole_coeeffecients.at(5),hole_coeeffecients.at(6)};
        drillTarget detectedHole;
        detectedHole.cluster = drillTargets_cad.front().cluster;
        detectedHole.position_aligned =pos;
        detectedHole.BB = BBs.at(k);
        detectedHole.ROI = BBs[k].ROI;


        if((hole_coeeffecients[2]*2) > params.diameteFullSized ) // this value is critical to decide whether it is full or piloted
            detectedHole.fullSized =true;
        else
            detectedHole.piloted =true;
        detectedHole.detectedHoleDiameter = hole_coeeffecients.at(2) * 2;
        
       
    
        positions_cicle_extractions.emplace_back(pos);
        positions_centroid.emplace_back(pos_centroid);
        detectedHoles.emplace_back(detectedHole);

        }else{
                std::cout << " Hole normals can not be extracted " <<std::endl;
            }
        }

    }else{
        std::cout << "Cloud is empty after filtering, no extraction is possible" << std::endl;
    }

    std::cout << "Extraction is done! Total number detected holes is: " << detectedHoles.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_detected (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < detectedHoles.size(); i++){
            pilot_holes_detected->points.emplace_back(detectedHoles.at(i).position_aligned[0],detectedHoles.at(i).position_aligned[1],detectedHoles.at(i).position_aligned[2]);
    }  
   // showCircles(cloud_BB_all,params.cloudPath,coefficient_all);
    //showExtractedHoles(cloud_BB_all,params.cloudPath,add_lines);
    //extractedDrillTargets(pilot_holes_detected,params.cloudPath);
    showCenters(positions_cicle_extractions,positions_centroid,params.cloudPath,cloud_BB_all_Real);
    showCenters(positions_cicle_extractions,positions_centroid,params.cloudPath,cloud_BB_all);




    return detectedHoles;
}

/*
std::vector<drillTarget>  extractHoles(const std::vector<boundingBox>& BBs,
                                        const std::vector<drillTarget> drillTargets_cad,
                                        const std::string& cloudPath,
                                        const params& params){



    std::vector<drillTarget> detectedHoles;
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
        throw std::runtime_error("Couldn't read cloud pcd file \n");
        }


//     pcl::PointCloud<pcl::PointXYZ>::Ptr cad_locations(new pcl::PointCloud<pcl::PointXYZ>);
//      for (size_t i = 0; i < drillTargets_cad.size(); i++){
//         cad_locations->points.emplace_back(drillTargets_cad.at(i).position[0],drillTargets_cad.at(i).position[1],drillTargets_cad.at(i).position[2]);
//         }

 
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//      pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
//     feature_extractor.setInputCloud (cad_locations);//default clould
//     feature_extractor.compute ();
//     pcl::PointXYZ min_point_AABB;
//     pcl::PointXYZ max_point_AABB;
//     pcl::PointXYZ min_point_OBB;
//     pcl::PointXYZ max_point_OBB;
//     pcl::PointXYZ position_OBB;
//     Eigen::Matrix3f rotational_matrix_OBB;
//     feature_extractor.getAABB(min_point_AABB,max_point_AABB);
//     feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
//     Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
//    Eigen::Quaternionf quat(rotational_matrix_OBB);
//     Eigen::Vector3f rot = rotational_matrix_OBB.eulerAngles(0,1,2);
//      pcl::CropBox<pcl::PointXYZ> boxFilter;
//      boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x- params.marginBBXaxis, min_point_OBB.y - params.marginBBYaxis , min_point_OBB.z - params.marginBBZaxis, 1.0));
//      boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x + params.marginBBXaxis,  max_point_OBB.y +  params.marginBBYaxis ,max_point_OBB.z +  params.marginBBZaxis, 1.0));
//      Eigen::Affine3f t;
//      t.translation() = position;
//      t.linear() = rotational_matrix_OBB;
//      boxFilter.setTransform(t.inverse());

//     // boxFilter.setRotation(quat.toRotationMatrix().eulerAngles(1,0,2));
//     // boxFilter.setTranslation(position);
//      boxFilter.setInputCloud(cloud);
//      boxFilter.filter(*cloud_filtered);

//      std::cout << " Cloud Filtered with oriented bounding box: " <<cloud_filtered->size() <<std::endl;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_removal (new pcl::PointCloud<pcl::PointXYZ>);

//     if(params.modeOutLierRemovalActive == true){
//         pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//         sor.setInputCloud (cloud_filtered);
//         sor.setMeanK (params.meanK);
//         sor.setStddevMulThresh (params.setStddevMulThresh);
//         sor.setNegative (false);
//         sor.filter (*cloud_filtered_removal);
//     }

    // std::cout << " Cloud after removal outliers: " <<cloud_filtered_removal->size() <<std::endl;

    //pcl::io::savePCDFileBinaryCompressed ("/home/oguz/vs_code/datasets/tests_musp_15_02_2022/test_pcd.pcd", *cloud_filtered);
    if(!cloud->points.empty()){


    
    float angularResolution_x = (float) (params.angular_resolution * (M_PI/180.0f));  // ADAPT IT!!
    float angular_resolution_y = (float) (params.angular_resolution * (M_PI/180.0f));
    float maxAngleWidth     = (float) (params.maxAngleWidth * (M_PI/180.0f));  // 180.0 degree in radians
    float maxAngleHeight    = (float) (params.maxAngleHeight * (M_PI/180.0f));  // 180.0 degree in radians

    //Eigen::Affine3f sensorPose = calculateProfilometerPose(drillTargets_cad,params);
        //NEW

    std::string pathToTCP= params.auditFolder + cv::format("/Coords18.txt");
    std::vector<drillTarget> tcp_ls = readPosesFromFile(pathToTCP);
    std::vector<drillTarget> tcp_ls_set = setParameters(tcp_ls);
    Eigen::Affine3f tcp = tcp_ls_set.at(1).homogeneousMat;
    // Profilometer calibration
    Eigen::Matrix3f profi_rotation = rotation_from_euler(params.profilometerCalibRotation[0],params.profilometerCalibRotation[1],params.profilometerCalibRotation[2]);
    Eigen::Vector3f profi_translation{params.profilometerCalibTranslation};

    Eigen::Vector3f profiToSensor = profi_rotation * profi_translation;
    Eigen::Affine3f sensorPose = tcp.translate((-1)*profiToSensor);
    //NEW
    
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.0f;
    float minRange = 0.0f;
    int borderSize = params.borderSize;

    pcl::RangeImage rangeImage;
    //Creating Range image


    
        
    rangeImage.createFromPointCloud(*cloud, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    std::cout << " range image is ready! "<< std::endl;
       
        //pcl::RangeImage rangeImage = convertPointCloud2RangeMap(drillTargets_cad,cloudPath,params);
        std::vector<pcl::PointXYZ> add_lines; // for visualization//#######################
        std::vector<Eigen::Vector3f> positions_cicle_extractions;
        std::vector<Eigen::Vector3f> positions_centroid;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_all(new pcl::PointCloud<pcl::PointXYZ>); // for visualization//#######################
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_all_Real(new pcl::PointCloud<pcl::PointXYZ>); // for visualization//#######################

        for (int k = 0; k < BBs.size(); k++){
            std::cout << "Extracting Bounding Box: " << k << std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB(new pcl::PointCloud<pcl::PointXYZ>);
                for (int u = BBs.at(k).topLeft.y; u < BBs.at(k).bottomRight.y; u++){
                    for (int v = BBs.at(k).topLeft.x; v <  BBs.at(k).bottomRight.x; v++){
                        if(isinf(rangeImage.at(v,u).range)){
                    }else{
                        pcl::PointXYZ p{rangeImage.at(v,u).x,rangeImage.at(v,u).y,rangeImage.at(v,u).z};
                        cloud_BB->points.emplace_back(p);
                        cloud_BB_all->points.emplace_back(p); // for visualization//#######################
                        }

                }
            }

            //NEw
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_real (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cloud_BB);//default clould
    feature_extractor.compute ();
    //pcl::PointXYZ min_point_AABB;
    //pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    //feature_extractor.getAABB(min_point_AABB,max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
     pcl::CropBox<pcl::PointXYZ> boxFilter;
     boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x, min_point_OBB.y , min_point_OBB.z - 5, 1.0));
     boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x,  max_point_OBB.y  ,max_point_OBB.z + 5 , 1.0));
     Eigen::Affine3f t;
     t.translation() = position;
     t.linear() = rotational_matrix_OBB;
     boxFilter.setTransform(t.inverse());

     boxFilter.setInputCloud(cloud);
     boxFilter.filter(*cloud_BB_real);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_BB_real,centroid);
    Eigen::Vector3f pos_centroid{centroid.x(),centroid.y(),centroid.z()};

    std::cout << " cloud with the BB created! "<< std::endl;

    drillTarget detectedHole;
    detectedHole.BB = BBs.at(k);
    detectedHole.position_aligned = pos_centroid;
    detectedHole.cluster = drillTargets_cad.front().cluster;

    int width = detectedHole.BB.bottomRight.x - detectedHole.BB.topLeft.x ;
    int height= detectedHole.BB.bottomRight.y - detectedHole.BB.topLeft.y ;
    std::cout << width << "," << height << std::endl;
    if(std::abs(width) > params.fullSizeHole || std::abs(height) > params.fullSizeHole) // this value is critical to decide whether it is full or piloted
        detectedHole.fullSized =true;
    else
        detectedHole.piloted =true;


    //pcl::ModelCoefficients coeffient_show(detect3DCirclePC_Coeff(cloud_BB));
    std::vector<float> hole_coeeffecients(detect3DCirclePC(cloud_BB));
    //coefficient_all.emplace_back(coeffient_show);
    if(!hole_coeeffecients.empty()){
        Eigen::Vector3f pos{hole_coeeffecients.at(0),hole_coeeffecients.at(1),hole_coeeffecients.at(2)};
        Eigen::Vector3f nor{hole_coeeffecients.at(4),hole_coeeffecients.at(5),hole_coeeffecients.at(6)};
        drillTarget detectedHole;
        detectedHole.normals = nor;
        detectedHole.detectedHoleDiameter = hole_coeeffecients.at(3) * 2;
        
        std::vector<float> coeff;
        coeff.emplace_back(hole_coeeffecients.at(4));
        coeff.emplace_back(hole_coeeffecients.at(5));
        coeff.emplace_back(hole_coeeffecients.at(6));
        Eigen::Matrix3f rotation_matrix_ = calculateQuartenions(coeff);
        Eigen::Quaternionf q(rotation_matrix_);
        detectedHole.quartenion = q;

                        //#######################
            // for visualization
        pcl::PointXYZ p1{hole_coeeffecients.at(0) + (hole_coeeffecients.at(4) * -5),
        hole_coeeffecients.at(1) + (hole_coeeffecients.at(5) * -5),
        hole_coeeffecients.at(2) + (hole_coeeffecients.at(6) * -5)};
        pcl::PointXYZ p2{hole_coeeffecients.at(0) + (hole_coeeffecients.at(4) * +5),
        hole_coeeffecients.at(1) + (hole_coeeffecients.at(5) * +5),
        hole_coeeffecients.at(2) + (hole_coeeffecients.at(6) * +5)};
        add_lines.emplace_back(p1);
        add_lines.emplace_back(p2);
    
        positions_cicle_extractions.emplace_back(pos);
        positions_centroid.emplace_back(pos_centroid);
        
        // for visualization
        //#######################
            }else{
                std::cout << " Hole normals can not be extracted " <<std::endl;
            }
        detectedHoles.emplace_back(detectedHole);

        }

        showExtractedHoles(cloud_BB_all,params.cloudPath,add_lines);
        //showCircles(cloud_BB_all,params.cloudPath,coefficient_all);
        //showCenters(positions_cicle_extractions,positions_centroid,params.cloudPath);

    }else{
        std::cout << "Cloud is empty after filtering, no extraction is possible" << std::endl;
    }

    std::cout << "Extraction is done! Total number detected holes is: " << detectedHoles.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_detected (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < detectedHoles.size(); i++){
            pilot_holes_detected->points.emplace_back(detectedHoles.at(i).position_aligned[0],detectedHoles.at(i).position_aligned[1],detectedHoles.at(i).position_aligned[2]);
    }    
    extractedDrillTargets(pilot_holes_detected,params.cloudPath);




    return detectedHoles;
}
*/

std::vector<float> detect2DCirclePC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered){



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

  seg_p.setInputCloud (cloud_filtered);
  seg_p.segment (*inliers_p, *coefficients_p);

  pcl::ExtractIndices<pcl::PointXYZ> extract;

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

// Extract the inliers
    extract.setInputCloud (cloud_filtered);
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
    //seg.setModelType (pcl::SACMODEL_CIRCLE3D);
    seg.setModelType (pcl::SACMODEL_CIRCLE2D);

    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setRadiusLimits(0,5);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (1);
    //seg.setEpsAngle(0.01);


    seg.setInputCloud (cloud_p);
    seg.segment (*inliers, *coefficients);

res.emplace_back(coefficients->values[0]);
res.emplace_back(coefficients->values[1]);
res.emplace_back(coefficients->values[2]);


// pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Correspondences"));
// viewer->setBackgroundColor (0, 0, 0);
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes(new pcl::PointCloud<pcl::PointXYZ>);
// cloud_holes->points.emplace_back(coefficients->values[0],coefficients->values[1],coefficients->values[2]);

// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_center(cloud_holes, 255, 255, 0);
// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_p(cloud_p, 255, 255, 0);

//   pcl::PointXYZ p1{coefficients->values[0],coefficients->values[1],cloud_p->points.front().z};
//  pcl::PointXYZ p2{coefficients->values[0]+coefficients->values[2],coefficients->values[1],cloud_p->points.front().z};
// //  pcl::PointXYZ p1{coefficients->values[0],coefficients->values[1],coefficients->values[2]};
// // pcl::PointXYZ p2{coefficients->values[0]+coefficients->values[3],coefficients->values[1],coefficients->values[2]};
// std::cout << "radius " << coefficients->values[2] << std::endl;
// viewer->addLine(p1,p2,122.0,133.0,250.0,cv::format("line"));

// viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "cloud_filtered"); 

// viewer->addPointCloud<pcl::PointXYZ>(cloud_p, color_p,"cloud_p"); 

// viewer->addPointCloud<pcl::PointXYZ>(cloud_holes, color_center,"cloud_holes"); 
// //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_holes");

// while (!viewer->wasStopped ()) {viewer->spinOnce ();}  





    return res;
}


std::vector<float> detect3DCirclePC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered){

    std::vector<float> res;


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType (pcl::SAC_RANSAC);

    seg.setRadiusLimits(0,5);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.1);
    //seg.setEpsAngle(0.01);


    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);




   if (inliers->indices.size () != 0){
        res.emplace_back(coefficients->values[0]);
        res.emplace_back(coefficients->values[1]);
        res.emplace_back(coefficients->values[2]);
        res.emplace_back(coefficients->values[3]);
        res.emplace_back(coefficients->values[4]);
        res.emplace_back(coefficients->values[5]);
        res.emplace_back(coefficients->values[6]);
    }
    return res;
}


// pcl::ModelCoefficients detect3DCirclePC_Coeff(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered){

//     std::vector<float> res;


//     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

//     pcl::SACSegmentation<pcl::PointXYZ> seg;
//     // Optional
//     seg.setOptimizeCoefficients (true);
//     // Mandatory
//     seg.setModelType (pcl::SACMODEL_CIRCLE3D);
//     seg.setMethodType (pcl::SAC_RANSAC);

//     seg.setRadiusLimits(0,5);
//     seg.setMaxIterations(1000);
//     seg.setDistanceThreshold (0.1);
//     //seg.setEpsAngle(0.01);


//     seg.setInputCloud (cloud_filtered);
//     seg.segment (*inliers, *coefficients);
//    if (inliers->indices.size () != 0){
//         res.emplace_back(coefficients->values[0]);
//         res.emplace_back(coefficients->values[1]);
//         res.emplace_back(coefficients->values[2]);
//         res.emplace_back(coefficients->values[3]);
//         res.emplace_back(coefficients->values[4]);
//         res.emplace_back(coefficients->values[5]);
//         res.emplace_back(coefficients->values[6]);
//     }
  
//     return *coefficients;
// }


/*
std::vector<drillTarget> findCorrespondences(const std::vector<drillTarget>& drillTargets_CAD,
                                                const std::vector<drillTarget>& detectedHoles,
                                                const params& params ){


    std::cout << "number of points(CAD) in the cluster : " << drillTargets_CAD.size()  <<std::endl;
    std::vector<drillTarget> res(detectedHoles);// NEW
    pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_locations_cad (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_locations_detected (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < drillTargets_CAD.size(); i++){
            pilot_holes_locations_cad->points.emplace_back(drillTargets_CAD.at(i).position[0],drillTargets_CAD.at(i).position[1],drillTargets_CAD.at(i).position[2]);
    }



        // CREATE BOUNDING BOX AROUND THE CLUSTER
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (pilot_holes_locations_cad);//default clould
    feature_extractor.compute ();
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    feature_extractor.getAABB(min_point_AABB,max_point_AABB);
    pcl::PointXYZ min{min_point_AABB.x - params.marginBBXYaxis, min_point_AABB.y - params.marginBBXYaxis, min_point_AABB.z -params.marginBBZaxis};
    pcl::PointXYZ max{max_point_AABB.x + params.marginBBXYaxis, max_point_AABB.y + params.marginBBXYaxis, max_point_AABB.z + params.marginBBZaxis};
    // CREATE BOUNDING BOX AROUND THE CLUSTER

    showDrillTargets(pilot_holes_locations_cad,params.cloudPath);


    // Label as differentcluster or not
    for (size_t i = 0; i < res.size(); i++){
        if(res.at(i).position_aligned[0]<max.x && res.at(i).position_aligned[1]<max.y &&
        res.at(i).position_aligned[0]>min.x && res.at(i).position_aligned[1]>min.y ){
            std::cout << " In Bounding BOX " << std::endl;
            res.at(i).differentCluster = false;
        }else{
            res.at(i).differentCluster = true;
            res.at(i).cluster = -1;
            std::cout << " Out Bounding BOX " << std::endl;
        }
    }

        // create the point cloud with holes provided after dense alignment // find the correspding point and discard it
    for (size_t i = 0; i < res.size(); i++){
        if(res.at(i).differentCluster == false){
            pilot_holes_locations_detected->points.emplace_back(res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2]);
        }
    }

    // The Iterative Closest Point algorithm
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    int iterations = 1000;
    icp.setInputSource (pilot_holes_locations_cad);
    icp.setInputTarget (pilot_holes_locations_detected);
    icp.setMaxCorrespondenceDistance (params.maxCorrespondenceDistance);
    icp.setMaximumIterations (iterations);
    icp.align (*pilot_holes_locations_cad);
     Eigen::Matrix4f transform = icp.getFinalTransformation();
     Eigen::Affine3f transform_mat(transform);
    // The Iterative Closest Point algorithm

    // Octree
    for (size_t i = 0; i < res.size(); i++){
        if (res.at(i).differentCluster == false){

            pcl::PointXYZ searchPoint{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2]};
            float resolution = 128.0f;
            pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
            octree.setInputCloud (pilot_holes_locations_cad);
            octree.addPointsFromInputCloud ();

            int K = 1; // closest neighbour,
            std::vector<int> pointIdxNKNSearch;
            std::vector<float> pointNKNSquaredDistance;

            if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
                if( std::sqrt(pointNKNSquaredDistance[0]) < params.maxCorrespondenceDistance)  {
                     Eigen::Vector3f position_matched{(*pilot_holes_locations_cad)[ pointIdxNKNSearch[0] ].x ,
                    (*pilot_holes_locations_cad)[ pointIdxNKNSearch[0] ].y,
                    (*pilot_holes_locations_cad)[ pointIdxNKNSearch[0] ].z};
                    Eigen::Vector3f position_matched_back = transform_mat.inverse() * position_matched; // Due to ICP
                    res.at(i).position = position_matched_back;
                    res.at(i).misPrediction=false;
                    pcl::PointCloud<pcl::PointXYZ>::iterator index = pilot_holes_locations_cad->begin(); // delete the point
                    pilot_holes_locations_cad->erase(index + pointIdxNKNSearch[0]);
                }else{
                std::cout << "mispredicted hole" << std::endl;
                res.at(i).misPrediction=true;
                }
            }
        }
    }

    // Due to icp back transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_locations_cad_backTransformed (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*pilot_holes_locations_cad, *pilot_holes_locations_cad_backTransformed, icp.getFinalTransformation().inverse());


    // Add unpiloted holes in results
    for (const auto point : pilot_holes_locations_cad_backTransformed->points){
        for (size_t i = 0; i < drillTargets_CAD.size(); i++){
            if((int)drillTargets_CAD.at(i).position.x() == (int)point.x &&
            (int)drillTargets_CAD.at(i).position.y() == (int)point.y &&
            (int)drillTargets_CAD.at(i).position.z() == (int)point.z ) {
                drillTarget unpiloted_point;
                unpiloted_point.quartenion_cad = drillTargets_CAD.at(i).quartenion;
                unpiloted_point.rotation_matrix = drillTargets_CAD.at(i).rotation_matrix ;
                unpiloted_point.position = drillTargets_CAD.at(i).position;
                unpiloted_point.unpiloted =true;
                unpiloted_point.cluster =drillTargets_CAD.at(i).cluster;
                res.emplace_back(unpiloted_point);
           }
        }
    }


    // Add quarnions from cad in piloted holes
    for (size_t i = 0; i < res.size(); i++){
         if(res.at(i).piloted == true || res.at(i).fullSized == true ){
            for (size_t j = 0; j < drillTargets_CAD.size(); j++){
                if((int)drillTargets_CAD.at(j).position.x() == (int)res.at(i).position.x() &&
                (int)drillTargets_CAD.at(j).position.y() == (int)res.at(i).position.y() &&
                (int)drillTargets_CAD.at(j).position.z() == (int)res.at(i).position.z()) {
                    res.at(i).quartenion_cad = drillTargets_CAD.at(j).quartenion;         
                    res.at(i).rotation_matrix = drillTargets_CAD.at(j).rotation_matrix;
                    res.at(i).homogeneousMat.linear() = res.at(i).rotation_matrix;
                    res.at(i).homogeneousMat.translation() = res.at(i).position_aligned;
                }
            }
        }
    }

    std::cout << "number of unmatched points(unpiloted): " << pilot_holes_locations_cad->points.size()  <<std::endl;

    showCorrespendences(res,params.cloudPath); // Visualize results

    return res;

}
*/

std::vector<drillTarget> findCorrespondences(const std::vector<drillTarget>& drillTargets_CAD,
                                            const std::vector<drillTarget>& detectedHoles,
                                            const params& params ){

    /*
    -- What happenss if the cad points are not all matched
    */

    std::cout << "number of points(CAD) in the cluster : " << drillTargets_CAD.size()  <<std::endl;

    std::vector<drillTarget> res(detectedHoles);// NEW

    pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_locations_cad (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_locations_detected (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < drillTargets_CAD.size(); i++){
            pilot_holes_locations_cad->points.emplace_back(drillTargets_CAD.at(i).position[0],drillTargets_CAD.at(i).position[1],drillTargets_CAD.at(i).position[2]);
    }



        // CREATE BOUNDING BOX AROUND THE CLUSTER
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (pilot_holes_locations_cad);//default clould
    feature_extractor.compute ();
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    feature_extractor.getAABB(min_point_AABB,max_point_AABB);
    pcl::PointXYZ min{min_point_AABB.x - params.marginBBXaxis, min_point_AABB.y - params.marginBBYaxis, min_point_AABB.z -params.marginBBZaxis};
    pcl::PointXYZ max{max_point_AABB.x + params.marginBBXaxis, max_point_AABB.y + params.marginBBYaxis, max_point_AABB.z + params.marginBBZaxis};
    // CREATE BOUNDING BOX AROUND THE CLUSTER


    // Label as differentcluster or not
    for (size_t i = 0; i < res.size(); i++){
        if(res.at(i).position_aligned[0]<max.x && res.at(i).position_aligned[1]<max.y &&
        res.at(i).position_aligned[0]>min.x && res.at(i).position_aligned[1]>min.y ){
            std::cout << " In Bounding BOX " << std::endl;
            res.at(i).differentCluster = false;
        }else{
            res.at(i).differentCluster = true;
            res.at(i).cluster = -1;
            res.at(i).ID = -1;
            std::cout << " Out Bounding BOX " << std::endl;
        }
    }

        // create the point cloud with holes provided after dense alignment // find the correspding point and discard it
    for (size_t i = 0; i < res.size(); i++){
        if(res.at(i).differentCluster == false){
            pilot_holes_locations_detected->points.emplace_back(res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2]);
        }
    }


    // Octree
    for (size_t i = 0; i < res.size(); i++){
        if(pilot_holes_locations_cad->points.size() > 0){

            if (res.at(i).differentCluster == false){

                pcl::PointXYZ searchPoint{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2]};
                float resolution = 128.0f;
                pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
                octree.setInputCloud (pilot_holes_locations_cad);
                octree.addPointsFromInputCloud ();

                int K = 1; // closest neighbour,
                std::vector<int> pointIdxNKNSearch;
                std::vector<float> pointNKNSquaredDistance;

                if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
                    if( std::sqrt(pointNKNSquaredDistance[0]) < params.maxCorrespondenceDistance)  {
                        Eigen::Vector3f position_matched{(*pilot_holes_locations_cad)[ pointIdxNKNSearch[0] ].x ,
                        (*pilot_holes_locations_cad)[ pointIdxNKNSearch[0] ].y,
                        (*pilot_holes_locations_cad)[ pointIdxNKNSearch[0] ].z};
                        res.at(i).position = position_matched;
                        res.at(i).misPrediction=false;
                        pcl::PointCloud<pcl::PointXYZ>::iterator index = pilot_holes_locations_cad->begin(); // delete the point
                        pilot_holes_locations_cad->erase(index + pointIdxNKNSearch[0]);
                    }else{
                    std::cout << "mispredicted hole" << std::endl;
                    res.at(i).misPrediction=true;
                    res.at(i).ID = -1;
                    res.at(i).quartenion_cad.w() = 0;
                    res.at(i).quartenion_cad.x() = 0;
                    res.at(i).quartenion_cad.y() = 0;
                    res.at(i).quartenion_cad.z() = 0;

                    }
                }
            }
        }else{
            std::cout << "mispredicted hole" << std::endl;
            res.at(i).misPrediction=true;
            res.at(i).ID = -1;
            res.at(i).quartenion_cad.w() = 0;
            res.at(i).quartenion_cad.x() = 0;
            res.at(i).quartenion_cad.y() = 0;
            res.at(i).quartenion_cad.z() = 0;
        }
    }




    // Add unpiloted holes in results
    for (const auto point : pilot_holes_locations_cad->points){
        for (size_t i = 0; i < drillTargets_CAD.size(); i++){
            if((int)drillTargets_CAD.at(i).position.x() == (int)point.x &&
            (int)drillTargets_CAD.at(i).position.y() == (int)point.y &&
            (int)drillTargets_CAD.at(i).position.z() == (int)point.z ) {
                drillTarget unpiloted_point;
                 unpiloted_point.ID =drillTargets_CAD.at(i).ID;
                unpiloted_point.quartenion_cad = drillTargets_CAD.at(i).quartenion;
                unpiloted_point.rotation_matrix = drillTargets_CAD.at(i).rotation_matrix ;
                unpiloted_point.position = drillTargets_CAD.at(i).position;
                unpiloted_point.unpiloted =true;
                unpiloted_point.cluster =drillTargets_CAD.at(i).cluster;
                res.emplace_back(unpiloted_point);
           }
        }
    }


    // Add quarnions from cad in piloted&Fullsized holes
    for (size_t i = 0; i < res.size(); i++){
         if(res.at(i).piloted == true || res.at(i).fullSized == true ){
            for (size_t j = 0; j < drillTargets_CAD.size(); j++){
                if((int)drillTargets_CAD.at(j).position.x() == (int)res.at(i).position.x() &&
                (int)drillTargets_CAD.at(j).position.y() == (int)res.at(i).position.y() &&
                (int)drillTargets_CAD.at(j).position.z() == (int)res.at(i).position.z()) {
                    res.at(i).ID =drillTargets_CAD.at(j).ID;
                    res.at(i).quartenion_cad = drillTargets_CAD.at(j).quartenion;         
                    res.at(i).rotation_matrix = drillTargets_CAD.at(j).rotation_matrix;
                    res.at(i).homogeneousMat.linear() = res.at(i).rotation_matrix;
                    res.at(i).homogeneousMat.translation() = res.at(i).position_aligned;
                }
            }
        }
    }

    std::cout << "number of unmatched points(unpiloted): " << pilot_holes_locations_cad->points.size()  <<std::endl;

    showCorrespendences(res,params.cloudPath); // Visualize results

    return res;

}


std::vector<drillTarget>  alignUnpilotedDrillTargets(const std::vector<drillTarget>& drillTargets_CAD,
                                                    const std::vector<drillTarget>& detectedHoles,
                                                    const params& params,
                                                    const std::string& cloudPath){

    std::vector<drillTarget> res; // store all detectedHoles
    std::vector<drillTarget> unpilotedDrillTargets;
    std::vector<drillTarget> sizedDrillTargets;
     std::vector<drillTarget> otherDrillTargets;
    for (const auto dT : detectedHoles){
        if(dT.differentCluster == false && dT.misPrediction == false){
            if(dT.fullSized == true || dT.piloted == true){
                sizedDrillTargets.emplace_back(dT);
                res.emplace_back(dT);
            }else{
                unpilotedDrillTargets.emplace_back(dT);
            }
        }else{
            otherDrillTargets.emplace_back(dT);
            res.emplace_back(dT);
        }
    }



    if(sizedDrillTargets.size() > 3 && !unpilotedDrillTargets.empty()){
            pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_locations_cad (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_locations (new pcl::PointCloud<pcl::PointXYZ>);
    
            // create the point cloud with holes provided after dense alignment // find the correspding point and discard it
            for (size_t i = 0; i < sizedDrillTargets.size(); i++){
                pilot_holes_locations_cad->points.emplace_back(sizedDrillTargets.at(i).position[0],sizedDrillTargets.at(i).position[1],sizedDrillTargets.at(i).position[2]);
                pilot_holes_locations->points.emplace_back(sizedDrillTargets.at(i).position_aligned[0],sizedDrillTargets.at(i).position_aligned[1],sizedDrillTargets.at(i).position_aligned[2]);
                
            }

            
            // The Iterative Closest Point algorithm
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            int iterations = 10;
            icp.setInputSource (pilot_holes_locations_cad);
            icp.setInputTarget (pilot_holes_locations);
            icp.setMaxCorrespondenceDistance (params.maxCorrespondenceDistance);
            icp.setMaximumIterations (iterations);
            icp.align (*pilot_holes_locations_cad);

            if (icp.hasConverged ()){
                Eigen::Matrix4f  transformation_matrix = icp.getFinalTransformation();
                transformDrillTarges(unpilotedDrillTargets,transformation_matrix); // apply alignment to the drill targes
                std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;

                // Find normals
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
                if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
                    throw std::runtime_error("Couldn't read cloud pcd file \n");
                }
                float resolution = 128.0f;
                pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
                octree.setInputCloud (cloud);
                octree.addPointsFromInputCloud ();


                for (size_t i = 0; i < unpilotedDrillTargets.size(); i++) {
                    // Neighbors within radius search
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hole (new pcl::PointCloud<pcl::PointXYZ>);

                    pcl::PointXYZ searchPoint{unpilotedDrillTargets.at(i).position_aligned.x(),unpilotedDrillTargets.at(i).position_aligned.y(),unpilotedDrillTargets.at(i).position_aligned.z()};
                    std::vector<int> pointIdxRadiusSearch;
                    std::vector<float> pointRadiusSquaredDistance;

                    float radius = 3.0f;

                    if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
                            for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                                pcl::PointXYZ p{(*cloud)[ pointIdxRadiusSearch[i] ].x,
                                            (*cloud)[ pointIdxRadiusSearch[i] ].y,
                                        (*cloud)[ pointIdxRadiusSearch[i] ].z };
                                cloud_hole->points.emplace_back(p);
                        }
                    }
                }
            }

         std::cout << "Unpiloted holes are transformed, their regions are examined to extract normals!" << std::endl;
            }else{
                std::cout << "alignment was not possible " << std::endl;
                for (size_t i = 0; i <unpilotedDrillTargets.size(); i++){
                    unpilotedDrillTargets.at(i).position_aligned = unpilotedDrillTargets.at(i).position;
                    }
                
                }
    
        // ADD aligned unpiloted drill targets in the points to be drilled.
    //    for (size_t i = 0; i <unpilotedDrillTargets.size(); i++){
    //         res.emplace_back(unpilotedDrillTargets.at(i));
    //     }
            for (const auto unpiloted_dT : unpilotedDrillTargets){
                    res.emplace_back(unpiloted_dT);
                }
        
        

        std::cout << " Number of sized Holes " << sizedDrillTargets.size() << std::endl;
        std::cout << " Number of unpiloted Holes " << unpilotedDrillTargets.size() << std::endl;
        std::cout << " Number of other Holes " << otherDrillTargets.size() << std::endl;


  return res;

}

void transformDrillTarges(std::vector<drillTarget>& drillTargets,const Eigen::Matrix4f& transform_matrix){
            Eigen::Affine3f transform_mat(transform_matrix);
        for (size_t i = 0; i < drillTargets.size(); i++){
            if(drillTargets.at(i).fullSized == false && drillTargets.at(i).piloted == false){
                drillTargets.at(i).position_aligned = transform_mat * drillTargets.at(i).position;
            }
    }

}


 Eigen::Matrix3f calculateQuartenions(const std::vector<float>& hole_coeffecients){

            // CAlculate quartenions

        double normalLength = sqrt((hole_coeffecients.at(0) * hole_coeffecients.at(0)) + (hole_coeffecients.at(1) * hole_coeffecients.at(1)) + (hole_coeffecients.at(2)*hole_coeffecients.at(2)));

        pcl::PointXYZ normalXY;
        pcl::PointXYZ normalSurface;
        pcl::PointXYZ result;
        // normalXY.x = 0.0;
        // normalXY.y = 0.0;
        // normalXY.z = 1.0;

        // normalSurface.x =hole_coeffecients.at(0)/normalLength;
        // normalSurface.y = hole_coeffecients.at(1)/normalLength;
        // normalSurface.z = hole_coeffecients.at(2)/normalLength;

        normalSurface.x = 0.0;
        normalSurface.y = 0.0;
        normalSurface.z = -1.0;

        normalXY.x =hole_coeffecients.at(0)/normalLength;
        normalXY.y = hole_coeffecients.at(1)/normalLength;
        normalXY.z = hole_coeffecients.at(2)/normalLength;

        result.x = normalSurface.y * normalXY.z - normalXY.y*normalSurface.z;
        result.y = normalXY.x*normalSurface.z - normalSurface.x*normalXY.z;
        result.z = normalSurface.x*normalXY.y - normalSurface.y*normalXY.x;
        double resultLenght = sqrt((result.x * result.x) + (result.y * result.y) +(result.z * result.z));
        result.x /=resultLenght;
        result.y /=resultLenght;
        result.z /=resultLenght;
        double theta = std::acos(normalSurface.z/sqrt((normalSurface.x* normalSurface.x)+(normalSurface.y*normalSurface.y)+(normalSurface.z*normalSurface.z)));
        //std::cout << "The crossproduct is " << result.x << " "<< result.y<< " "<< result.z <<std::endl;
        //std::cout << "norm " << std::sqrt((result.x * result.x  )+(result.y * result.y)+ (result.z * result.z))<< std::endl;

        double xx = (result.x * result.x) + ((1 - (result.x * result.x)) * std::cos(theta));
        double xy = (result.x * result.y)* (1 - std::cos(theta)) - (result.z * std::sin(theta));
        double xz = (result.x * result.z)* (1 - std::cos(theta)) + (result.y * std::sin(theta));
        double yx = (result.x * result.y)* (1 - std::cos(theta)) + (result.z * std::sin(theta));
        double yy = (result.y * result.y) + ((1 - (result.y * result.y)) * std::cos(theta));
        double yz = (result.y * result.z)* (1 - std::cos(theta)) - (result.x * std::sin(theta));
        double zx = (result.x * result.z)* (1 - std::cos(theta)) - (result.y * std::sin(theta));
        double zy = (result.y * result.z)* (1 - std::cos(theta)) + (result.x * std::sin(theta));
        double zz = (result.z * result.z) + ((1 - (result.z * result.z)) * std::cos(theta));

        Eigen::Matrix3f transform_matrix;
        //transformXY = Eigen::Matrix4f::Identity();

        transform_matrix (0,0) = xx; //cos (theta);
        transform_matrix (0,1) = xy;// -sin(theta)
        transform_matrix (0,2) = xz;
        transform_matrix (1,0) = yx; // sin (theta)
        transform_matrix (1,1) = yy;
        transform_matrix (1,2) = yz;
        transform_matrix (2,0) = zx ;
        transform_matrix (2,1) = zy;
        transform_matrix (2,2) = zz;

    return   transform_matrix;
         //Eigen::Affine3f t(transformXY.inverse());
         //Eigen::Quaternionf q(transformXY.inverse().matrix());



}

std::vector<drillTarget> inspectDrillTargets(const std::vector<drillTarget>& drillTargets,
                                        const std::vector<drillTarget>& drillTargets_CAD,
                                            const params& params,
                                            const std::string& cloudPath){

            std::cout << "=====> Launch Edge&Center Tolerance inspection!!! " << std::endl;

        std::vector<drillTarget> drillTargetsset_CAD = setParameters(drillTargets_CAD);

            std::cout << "=====> Parameters set " << std::endl;

        Eigen::Affine3f profilometerPose = calculateProfilometerPose(drillTargetsset_CAD,params);

            std::cout << "=====> Inspecting edge tolerances " << std::endl;

        std::vector<drillTarget> drillTargets_edge_inspected =  checkEdgeTolerances(drillTargets,drillTargets_CAD,params,profilometerPose,cloudPath); // Check tolerace to edges

            std::cout << "=====> Inspecting center tolerances " << std::endl;

        std::vector<drillTarget> drillTargets_center_edge_inspected = checkCenterTolerances(drillTargets_edge_inspected,params); // check tolerance between centers

            std::cout << "=====> Edge&Center Tolerance inspection is over!!! " << std::endl;

        return drillTargets_center_edge_inspected;
}


Eigen::Affine3f calculateProfilometerPose(const std::vector<drillTarget>& drillTargets_CAD,const params& params){

    std::vector<drillTarget> drillTargets_detected;
    for(const auto dT : drillTargets_CAD){
        if(dT.misPrediction == false && dT.differentCluster == false)
            drillTargets_detected.emplace_back(dT);
    }

    drillTarget middlePoint = drillTargets_detected.at((std::floor(drillTargets_detected.size()/2)));
    Eigen::Vector3f offset{0,0,params.sensorDistanceOffset};
    Eigen::Vector3f translationInSensorFrame = middlePoint.rotation_matrix * offset;
    Eigen::Affine3f robot_tcp = middlePoint.homogeneousMat.translate(translationInSensorFrame); //

    // Profilometer calibration
    Eigen::Matrix3f profi_rotation = rotation_from_euler(params.profilometerCalibRotation[0],params.profilometerCalibRotation[1],params.profilometerCalibRotation[2]);
    Eigen::Vector3f profi_translation{params.profilometerCalibTranslation};

    Eigen::Vector3f profiToSensor = profi_rotation * profi_translation;
    Eigen::Affine3f sensorPoseFinal = robot_tcp.translate((-1)*profiToSensor);

    return sensorPoseFinal;

    }

    Eigen::Affine3f calculateProfilometerPoseBBMiddle(const std::vector<drillTarget>& drillTargets_CAD,const params& params){

    std::vector<drillTarget> drillTargets_detected;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_locations(new pcl::PointCloud<pcl::PointXYZ>);
     for (size_t i = 0; i < drillTargets_CAD.size(); i++){
        if(drillTargets_CAD.at(i).misPrediction == false && drillTargets_CAD.at(i).differentCluster == false){
            cad_locations->points.emplace_back(drillTargets_CAD.at(i).position[0],drillTargets_CAD.at(i).position[1],drillTargets_CAD.at(i).position[2]);
            drillTargets_detected.emplace_back(drillTargets_CAD.at(i));
        }
    }


     pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cad_locations);//default clould
    feature_extractor.compute ();
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat(rotational_matrix_OBB);

    Eigen::Affine3f middlePoint;
    middlePoint.linear() = drillTargets_detected.at((std::floor(drillTargets_detected.size()/2))).rotation_matrix;
    middlePoint.translation() = position;

    Eigen::Vector3f offset{0,0,params.sensorDistanceOffset};
    Eigen::Vector3f translationInSensorFrame = middlePoint.rotation() * offset;
    Eigen::Affine3f robot_tcp = middlePoint.translate(translationInSensorFrame); //

    // Profilometer calibration
    Eigen::Matrix3f profi_rotation = rotation_from_euler(params.profilometerCalibRotation[0],params.profilometerCalibRotation[1],params.profilometerCalibRotation[2]);
    Eigen::Vector3f profi_translation{params.profilometerCalibTranslation};

    Eigen::Vector3f profiToSensor = profi_rotation * profi_translation;
    Eigen::Affine3f sensorPoseFinal = robot_tcp.translate((-1)*profiToSensor);

    return sensorPoseFinal;

    }


    std::vector<Eigen::Affine3f> calculateProfilometerPosewithClusters(const std::vector<drillTarget>& drillTargets_CAD,const params& params){

        std::vector<Eigen::Affine3f> res;

        std::vector<drillTarget> drillTargets_detected;

        for(const auto dT : drillTargets_CAD){
            if(dT.misPrediction == false && dT.differentCluster == false)
                drillTargets_detected.emplace_back(dT);
        }

    for (size_t i = 0; i < ((drillTargets_detected.size() / 3) + 1); i++){    
    
        if(!drillTargets_detected.empty()){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<drillTarget>::iterator it = drillTargets_detected.begin();
            std::vector<drillTarget> cluster_target;
            for (size_t i = 0; i < 3; i++){
                pcl::PointXYZ p{drillTargets_detected.at(i).position.x(),drillTargets_detected.at(i).position.y(),drillTargets_detected.at(i).position.z()};
                cluster->points.emplace_back(p);
                cluster_target.emplace_back(drillTargets_detected.at(i));
                drillTargets_detected.erase(it + i);
            }
    drillTarget middlePoint = cluster_target.at((std::floor(cluster_target.size()/2)));
    Eigen::Vector3f offset{0,0,params.sensorDistanceOffset};
    Eigen::Vector3f translationInSensorFrame = middlePoint.rotation_matrix * offset;
    Eigen::Affine3f robot_tcp = middlePoint.homogeneousMat.translate(translationInSensorFrame); //

        // Profilometer calibration
        Eigen::Matrix3f profi_rotation = rotation_from_euler(params.profilometerCalibRotation[0],params.profilometerCalibRotation[1],params.profilometerCalibRotation[2]);
        Eigen::Vector3f profi_translation{params.profilometerCalibTranslation};
        Eigen::Vector3f profiToSensor = profi_rotation * profi_translation;
        Eigen::Affine3f sensorPoseFinal = robot_tcp.translate((-1)*profiToSensor);

        res.emplace_back(sensorPoseFinal);
        }
    }
        return res;

    }


std::vector<drillTarget> checkEdgeTolerances(const std::vector<drillTarget>& drillTargets,
                                            const std::vector<drillTarget>& drillTargets_CAD,
                                            const params& params,
                                            const Eigen::Affine3f& profilometerPose,
                                            const std::string& cloudPath){

    std::vector<drillTarget> res(drillTargets);


    pcl::PointCloud<pcl::PointXYZ>::Ptr border_points_ptr_cloud = createRangeImageBorderExtaction(drillTargets,drillTargets_CAD,params,profilometerPose,cloudPath); // Extract border points

    showBorderPoints(border_points_ptr_cloud);


    // Octree
     float resolution = 128.0f;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
    octree.setInputCloud (border_points_ptr_cloud);
    octree.addPointsFromInputCloud ();
    int K = params.numberOfBorderNeighbour;
    if(!border_points_ptr_cloud->points.empty()){
        for (size_t i = 0; i < res.size(); i++){
            if(res.at(i).misPrediction == false && res.at(i).differentCluster == false){
                pcl::PointXYZ searchPoint{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2]};
                std::vector<int> pointIdxNKNSearch;
                std::vector<float> pointNKNSquaredDistance;

                if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
                    pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_neigbour_ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    for (size_t i = 0; i < K; i++){
                        pcl::PointXYZ p{(*border_points_ptr_cloud)[ pointIdxNKNSearch[i] ].x,(*border_points_ptr_cloud)[ pointIdxNKNSearch[i] ].y,(*border_points_ptr_cloud)[ pointIdxNKNSearch[i] ].z};
                        nearest_neigbour_ptr_cloud->points.emplace_back(p);
                    }

                    // Segment a line
                    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                    // Create the segmentation object
                    pcl::SACSegmentation<pcl::PointXYZ> seg;
                    // Optional
                    seg.setOptimizeCoefficients (true);
                    // Mandatory
                    seg.setModelType (pcl::SACMODEL_LINE);
                    seg.setMethodType (pcl::SAC_RANSAC);
                    seg.setMaxIterations(100);
                    seg.setDistanceThreshold (5);
                    //seg.setEpsAngle(0.01);
                    seg.setInputCloud (nearest_neigbour_ptr_cloud);
                    seg.segment (*inliers, *coefficients);

                    Eigen::Vector4f position{coefficients->values[0],coefficients->values[1],coefficients->values[2],0};
                    Eigen::Vector4f direction{ coefficients->values[3],coefficients->values[4],coefficients->values[5],0};
                    Eigen::Vector4f point{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2],0};


                    //#############
                    Eigen::Vector3f pos_first{coefficients->values[0] - (coefficients->values[3] * 5 ),(coefficients->values[1] - coefficients->values[4] * 5) ,(coefficients->values[2] - coefficients->values[5] * 5 )};
                    Eigen::Vector3f position_second{coefficients->values[0] + (coefficients->values[3] * 5 ),(coefficients->values[1]+ coefficients->values[4] * 5) ,(coefficients->values[2]+coefficients->values[5] * 5 )};
                    Eigen::Vector3f pos_center{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2]};

                    Eigen::Vector3f dif = pos_first - position_second;
                    float LineMag = std::sqrt(std::pow(dif.x(),2)+ std::pow(dif.y(),2) +std::pow(dif.z(),2));
                    float U = (((pos_center.x() - pos_first.x()) * (position_second.x() - pos_first.x())) +
                                ((pos_center.y() - pos_first.y()) * (position_second.y() - pos_first.y())) +
                                ((pos_center.z() - pos_first.z()) * (position_second.z() - pos_first.z()))) /
                        (LineMag * LineMag);

                    Eigen::Vector3f Intersection = pos_first + (U * (position_second- pos_first));
                    Eigen::Vector3f dif_fo = pos_center - Intersection;
                    float Distance = std::sqrt(std::pow(dif_fo.x(),2)+ std::pow(dif_fo.y(),2) +std::pow(dif_fo.z(),2));
                    //#############

                    double distance  = std::sqrt(pcl::sqrPointToLineDistance(point,position,direction));
                    res.at(i).distanceToEdge = (distance/25.4);
                    res.at(i).position_edge = Intersection;

                    // Eigen::Vector3f position_edge{coefficients->values[0],coefficients->values[1],coefficients->values[2]};
                    //  Eigen::Vector3f difference = (res.at(i).position_aligned - position_edge);
                    // float difference_distance = std::sqrt(std::pow(difference.x(),2) + std::pow(difference.y(),2) + std::pow(difference.z(),2));

                    if(res.at(i).distanceToEdge >= params.edgeToleranceMin ){
                            res.at(i).withinEdgeTolerance = true;
                            std::cout << "True edge tolerance" <<std::endl;
                    }else{
                        res.at(i).withinEdgeTolerance = false;
                        std::cout <<  " false  edge tolerance" <<std::endl;
                    }


                    std::cout <<  "Edge tolerance is : " << res.at(i).withinEdgeTolerance <<" with the distance: " <<  res.at(i).distanceToEdge << std::endl;
                }
            }else{
                res.at(i).withinEdgeTolerance =false;
                res.at(i).distanceToEdge = 0;
            }
        }
    }else{
        for (size_t i = 0; i < res.size(); i++){
            res.at(i).withinEdgeTolerance =true;
            res.at(i).distanceToEdge = 0;
        }
    }
    


    showEdgeDistances(res,params.cloudPath);


    std::cout << "Edge Distances Calculated!" << std::endl;


    return res;
}

/*
std::vector<drillTarget> checkEdgeTolerances(const std::vector<drillTarget>& drillTargets,const params& params,const Eigen::Affine3f& profilometerPose, const std::string& cloudPath){

    std::vector<drillTarget> res(drillTargets);


    pcl::PointCloud<pcl::PointXYZ>::Ptr border_points = createRangeImageBorderExtaction(drillTargets,params,profilometerPose,cloudPath); // Extract border points

    showBorderPoints(border_points);







  // pcl::PointCloud<pcl::PointXYZ>::Ptr border_points_OUTLIERSREM (new pcl::PointCloud<pcl::PointXYZ>);
        // for (const auto& idx: inliers->indices){
        //     border_points_OUTLIERSREM->points.emplace_back(border_points_ptr_cloud->points[idx].x,border_points_ptr_cloud->points[idx].y,border_points_ptr_cloud->points[idx].z);
        // }









    // Octree
     float resolution = 128.0f;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
    octree.setInputCloud (border_points);
    octree.addPointsFromInputCloud ();
    int K = 100;

    for (size_t i = 0; i < res.size(); i++){
        if(res.at(i).misPrediction == false && res.at(i).differentCluster == false){
            pcl::PointXYZ searchPoint{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2]};
            std::vector<int> pointIdxNKNSearch;
            std::vector<float> pointNKNSquaredDistance;


            if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
                pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_neigbour_ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                for (size_t i = 0; i < K; i++){
                    pcl::PointXYZ p{(*border_points)[ pointIdxNKNSearch[i] ].x,(*border_points)[ pointIdxNKNSearch[i] ].y,(*border_points)[ pointIdxNKNSearch[i] ].z};
                    nearest_neigbour_ptr_cloud->points.emplace_back(p);
                }


                // Segment a line
                pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                // Create the segmentation object
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                // Optional
                seg.setOptimizeCoefficients (true);
                // Mandatory
                seg.setModelType (pcl::SACMODEL_LINE);
                seg.setMethodType (pcl::SAC_RANSAC);
                seg.setMaxIterations(10000);
                seg.setDistanceThreshold (5);
                //seg.setEpsAngle(0.01);
                seg.setInputCloud (nearest_neigbour_ptr_cloud);
                seg.segment (*inliers, *coefficients);

                Eigen::Vector4f position{coefficients->values[0],coefficients->values[1],coefficients->values[2],0};
                Eigen::Vector4f direction{ coefficients->values[3],coefficients->values[4],coefficients->values[5],0};
                Eigen::Vector4f point{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2],0};



                //#############
                 Eigen::Vector3f pos_first{coefficients->values[0] - (coefficients->values[3] * 5 ),(coefficients->values[1] - coefficients->values[4] * 5) ,(coefficients->values[2] - coefficients->values[5] * 5 )};
                 Eigen::Vector3f position_second{coefficients->values[0] + (coefficients->values[3] * 5 ),(coefficients->values[1]+ coefficients->values[4] * 5) ,(coefficients->values[2]+coefficients->values[5] * 5 )};
                Eigen::Vector3f pos_center{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2]};

                Eigen::Vector3f dif = pos_first - position_second;
                float LineMag = std::sqrt(std::pow(dif.x(),2)+ std::pow(dif.y(),2) +std::pow(dif.z(),2));
                float U = (((pos_center.x() - pos_first.x()) * (position_second.x() - pos_first.x())) +
                            ((pos_center.y() - pos_first.y()) * (position_second.y() - pos_first.y())) +
                            ((pos_center.z() - pos_first.z()) * (position_second.z() - pos_first.z()))) /
                    (LineMag * LineMag);

                Eigen::Vector3f Intersection = pos_first + (U * (position_second- pos_first));
                Eigen::Vector3f dif_fo = pos_center - Intersection;
                float Distance = std::sqrt(std::pow(dif_fo.x(),2)+ std::pow(dif_fo.y(),2) +std::pow(dif_fo.z(),2));
                //#############

                double distance  = std::sqrt(pcl::sqrPointToLineDistance(point,position,direction));
                res.at(i).distanceToEdge = (distance/25.4);
                res.at(i).position_edge = Intersection;
                std::cout <<  "center point: " <<res.at(i).position_aligned.matrix() <<" has the " << distance <<" distance to the edge " << res.at(i).position_edge.matrix() << std::endl;

                // Eigen::Vector3f position_edge{coefficients->values[0],coefficients->values[1],coefficients->values[2]};
                //  Eigen::Vector3f difference = (res.at(i).position_aligned - position_edge);
                // float difference_distance = std::sqrt(std::pow(difference.x(),2) + std::pow(difference.y(),2) + std::pow(difference.z(),2));
                std::cout << "ikiso arasi: " << res.at(i).distanceToEdge  << std::endl;



                if(res.at(i).distanceToEdge >= params.edgeToleranceMin ){
                        res.at(i).withinEdgeTolerance = true;
                        std::cout << "True edge tolerance" <<std::endl;
                }else{
                    res.at(i).withinEdgeTolerance = false;
                    std::cout <<  " false  edge tolerance" <<std::endl;
                }




            }
        }
    }


    showEdgeDistances(res,params.cloudPath);


    std::cout << "Edge Distances Calculated!" << std::endl;


    return res;
}

*/



pcl::PointCloud<pcl::PointXYZ>::Ptr createRangeImageBorderExtaction( const std::vector<drillTarget>& drillTargets,
                                                                const std::vector<drillTarget>& drillTargets_CAD,
                                                                    const params& params,
                                                                 const Eigen::Affine3f& profilometerPose,
                                                                    const std::string& cloudPath){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
        throw std::runtime_error("Couldn't read cloud pcd file \n");
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_locations(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < drillTargets_CAD.size(); i++){
             cad_locations->points.emplace_back(drillTargets.at(i).position_aligned.x(),drillTargets.at(i).position_aligned.y(),drillTargets.at(i).position_aligned.z());
    }




//      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//      pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
//     feature_extractor.setInputCloud (cad_locations);//default clould
//     feature_extractor.compute ();
//     pcl::PointXYZ min_point_AABB;
//     pcl::PointXYZ max_point_AABB;
//     pcl::PointXYZ min_point_OBB;
//     pcl::PointXYZ max_point_OBB;
//     pcl::PointXYZ position_OBB;
//     Eigen::Matrix3f rotational_matrix_OBB;
//     feature_extractor.getAABB(min_point_AABB,max_point_AABB);
//     feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
//     Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
//    Eigen::Quaternionf quat(rotational_matrix_OBB);
//     Eigen::Vector3f rot = rotational_matrix_OBB.eulerAngles(0,1,2);
//      pcl::CropBox<pcl::PointXYZ> boxFilter;
//      boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x- params.marginBBXaxis, min_point_OBB.y - params.marginBBYaxis , min_point_OBB.z - params.marginBBZaxis, 1.0));
//      boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x + params.marginBBXaxis,  max_point_OBB.y +  params.marginBBYaxis ,max_point_OBB.z +  params.marginBBZaxis, 1.0));

//        Eigen::Affine3f t;
//      t.translation() = position;
//      t.linear() = rotational_matrix_OBB;
//      boxFilter.setTransform(t.inverse());
//      boxFilter.setInputCloud(cloud);
//      boxFilter.filter(*cloud_filtered);




    // if(params.modeOutLierRemovalActiveForBorder == true){
    //     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    //     sor.setInputCloud (cloud_filtered);
    //     sor.setMeanK (params.meanKBorder);
    //     sor.setStddevMulThresh (params.setStddevMulThreshBorder);
    //     sor.setNegative (false);
    //     sor.filter (*cloud_filtered);
    //  }
     std::cout << " Cloud Filtered with oriented bounding box: " <<cloud->size() <<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr border_points(new pcl::PointCloud<pcl::PointXYZ>);

        if(!cloud->points.empty()){

        float angularResolution_x = (float) (params.angular_resolution_border * (M_PI/180.0f));  // ADAPT IT!!
        float angular_resolution_y = (float) (params.angular_resolution_border * (M_PI/180.0f));
        float maxAngleWidth     = (float) (params.maxAngleWidth * (M_PI/180.0f));  // 180.0 degree in radians
        float maxAngleHeight    = (float) (params.maxAngleHeight * (M_PI/180.0f));  // 180.0 degree in radians


        //Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);//
        Eigen::Affine3f sensorPose = profilometerPose;
 
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
        float noiseLevel=0.0f;
        float minRange = 0.0f;
        int borderSize = params.borderSize;


        pcl::RangeImage rangeImageBorder;
        // Creating Range image
        rangeImageBorder.createFromPointCloud(*cloud, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);


        rangeImageBorder.setUnseenToMaxRange();

        //pcl::PointCloud<pcl::PointXYZ>::Ptr border_points  = borderExtraction(rangeImageBorder);

        pcl::RangeImageBorderExtractor border_extractor (&rangeImageBorder);
        pcl::PointCloud<pcl::BorderDescription> border_descriptions;
        border_extractor.compute(border_descriptions);

        for (int y=0; y< (int)rangeImageBorder.height; ++y){
            for (int x=0; x< (int)rangeImageBorder.width; ++x) {
                if (border_descriptions[y*rangeImageBorder.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER]){
                pcl::PointXYZ p{rangeImageBorder[y*rangeImageBorder.width + x].x,rangeImageBorder[y*rangeImageBorder.width + x].y,rangeImageBorder[y*rangeImageBorder.width + x].z};
                border_points->points.emplace_back(p);
            }
        }
        }

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Neighbors within radius search
        for (size_t i = 0; i < drillTargets.size(); i++){
            pcl::PointXYZ searchPoint{drillTargets.at(i).position_aligned.x(),drillTargets.at(i).position_aligned.y(),drillTargets.at(i).position_aligned.z()};
            float resolution = 128.0f;
            pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
            octree.setInputCloud (border_points);
            octree.addPointsFromInputCloud ();
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;      
            if (octree.radiusSearch (searchPoint,  params.radiusBorderFiltering , pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
                for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                    inliers->indices.emplace_back(pointIdxRadiusSearch[i]);                     
                }
            }
        }
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(border_points);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*border_points);

        std::cout << " Border Extaction is completed! " << border_points->points.size() << std::endl;
        }else{
            std::cout << " Cloud has zero points, no border extraction "<<border_points->points.size() <<std::endl;

        }
    return border_points;

}

/*
pcl::PointCloud<pcl::PointXYZ>::Ptr createRangeImageBorderExtaction( const std::vector<drillTarget>& drillTargets,
                                                                const std::vector<drillTarget>& drillTargets_CAD,
                                                                    const params& params,
                                                                 const Eigen::Affine3f& profilometerPose,

                                                                    const std::string& cloudPath){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
        throw std::runtime_error("Couldn't read cloud pcd file \n");
    }


//     pcl::PointCloud<pcl::PointXYZ>::Ptr cad_locations(new pcl::PointCloud<pcl::PointXYZ>);
//     for (size_t i = 0; i < drillTargets_CAD.size(); i++){
//              cad_locations->points.emplace_back(drillTargets.at(i).position_aligned.x(),drillTargets.at(i).position_aligned.y(),drillTargets.at(i).position_aligned.z());
//     }




//      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//      pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
//     feature_extractor.setInputCloud (cad_locations);//default clould
//     feature_extractor.compute ();
//     pcl::PointXYZ min_point_AABB;
//     pcl::PointXYZ max_point_AABB;
//     pcl::PointXYZ min_point_OBB;
//     pcl::PointXYZ max_point_OBB;
//     pcl::PointXYZ position_OBB;
//     Eigen::Matrix3f rotational_matrix_OBB;
//     feature_extractor.getAABB(min_point_AABB,max_point_AABB);
//     feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
//     Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
//    Eigen::Quaternionf quat(rotational_matrix_OBB);
//     Eigen::Vector3f rot = rotational_matrix_OBB.eulerAngles(0,1,2);
//      pcl::CropBox<pcl::PointXYZ> boxFilter;
//      boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x- params.marginBBXaxis, min_point_OBB.y - params.marginBBYaxis , min_point_OBB.z - params.marginBBZaxis, 1.0));
//      boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x + params.marginBBXaxis,  max_point_OBB.y +  params.marginBBYaxis ,max_point_OBB.z +  params.marginBBZaxis, 1.0));

//        Eigen::Affine3f t;
//      t.translation() = position;
//      t.linear() = rotational_matrix_OBB;
//      boxFilter.setTransform(t.inverse());
//      boxFilter.setInputCloud(cloud);
//      boxFilter.filter(*cloud_filtered);




//     if(params.modeOutLierRemovalActiveForBorder == true){
//         pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//         sor.setInputCloud (cloud_filtered);
//         sor.setMeanK (params.meanKBorder);
//         sor.setStddevMulThresh (params.setStddevMulThreshBorder);
//         sor.setNegative (false);
//         sor.filter (*cloud_filtered);
//      }
//      std::cout << " Cloud Filtered with oriented bounding box: " <<cloud_filtered->size() <<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr border_points(new pcl::PointCloud<pcl::PointXYZ>);

        if(!cloud->points.empty()){

        float angularResolution_x = (float) (params.angular_resolution_border * (M_PI/180.0f));  // ADAPT IT!!
        float angular_resolution_y = (float) (params.angular_resolution_border * (M_PI/180.0f));
        float maxAngleWidth     = (float) (params.maxAngleWidth * (M_PI/180.0f));  // 180.0 degree in radians
        float maxAngleHeight    = (float) (params.maxAngleHeight * (M_PI/180.0f));  // 180.0 degree in radians


        //Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);//
        //Eigen::Affine3f sensorPose = profilometerPose;
        //NEW

        std::string pathToTCP= params.auditFolder + cv::format("/Coords18.txt");
        std::vector<drillTarget> tcp_ls = readPosesFromFile(pathToTCP);
        std::vector<drillTarget> tcp_ls_set = setParameters(tcp_ls);
        Eigen::Affine3f tcp = tcp_ls_set.at(1).homogeneousMat;
        // Profilometer calibration
        Eigen::Matrix3f profi_rotation = rotation_from_euler(params.profilometerCalibRotation[0],params.profilometerCalibRotation[1],params.profilometerCalibRotation[2]);
        Eigen::Vector3f profi_translation{params.profilometerCalibTranslation};

        Eigen::Vector3f profiToSensor = profi_rotation * profi_translation;
        Eigen::Affine3f sensorPose = tcp.translate((-1)*profiToSensor);
        //NEW
        
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
        float noiseLevel=0.0f;
        float minRange = 0.0f;
        int borderSize = params.borderSize;


        pcl::RangeImage rangeImageBorder;
        // Creating Range image
        rangeImageBorder.createFromPointCloud(*cloud, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);


        rangeImageBorder.setUnseenToMaxRange();

        //pcl::PointCloud<pcl::PointXYZ>::Ptr border_points  = borderExtraction(rangeImageBorder);

        pcl::RangeImageBorderExtractor border_extractor (&rangeImageBorder);
        pcl::PointCloud<pcl::BorderDescription> border_descriptions;
        border_extractor.compute(border_descriptions);

        for (int y=0; y< (int)rangeImageBorder.height; ++y){
            for (int x=0; x< (int)rangeImageBorder.width; ++x) {
                if (border_descriptions[y*rangeImageBorder.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER]){
                pcl::PointXYZ p{rangeImageBorder[y*rangeImageBorder.width + x].x,rangeImageBorder[y*rangeImageBorder.width + x].y,rangeImageBorder[y*rangeImageBorder.width + x].z};
                border_points->points.emplace_back(p);
            }
        }
        }

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Neighbors within radius search
        for (size_t i = 0; i < drillTargets.size(); i++){
            pcl::PointXYZ searchPoint{drillTargets.at(i).position_aligned.x(),drillTargets.at(i).position_aligned.y(),drillTargets.at(i).position_aligned.z()};
            float resolution = 128.0f;
            pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
            octree.setInputCloud (border_points);
            octree.addPointsFromInputCloud ();
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;      
            if (octree.radiusSearch (searchPoint,  params.radiusBorderFiltering , pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
                for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                    inliers->indices.emplace_back(pointIdxRadiusSearch[i]);                     
                }
            }
        }
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(border_points);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*border_points);

        std::cout << " Border Extaction is completed! " << border_points->points.size() << std::endl;
        }else{
            std::cout << " Cloud has zero points, no border extraction "<<border_points->points.size() <<std::endl;

        }
    return border_points;

}


*/




pcl::PointCloud<pcl::PointXYZ>::Ptr readPLYFile(const std::string& cloudPath){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader Reader;
	Reader.read(cloudPath, *cloud);
	if ( cloud->width <1){
    	//PCL_ERROR ("Couldn't read file point_cloud.ply \n");
    	throw std::runtime_error("Couldn't read cloud ply file \n");
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr readPCDFile(const std::string& cloudPath){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
    	//PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		throw std::runtime_error("Couldn't read cloud pcd file \n");

  	}
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points "
            << std::endl;

	return cloud;

}



void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
													pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
													const std::string& axis,
													const float (&limits)[2]){
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
 	pass.setInputCloud (cloud);
  	pass.setFilterFieldName (axis);
  	pass.setFilterLimits (limits[0],limits[1]);
  	//pass.setFilterLimitsNegative (true);
  	pass.filter (*cloud_filtered);

}




std::vector<drillTarget> readPosesFromFile (const std::string& pathTOAlignedHolesLoc){
    // read holes location from file
    std::vector<drillTarget> res;
    int count = 0;
    std::ifstream file(pathTOAlignedHolesLoc);
    if (file.is_open()) {
        std::string line;
        while (getline(file,line)){
             if(line[0] == 'i' || line.empty())
                 continue;
            else{
                drillTarget d;
                d.ID = count;
                std::stringstream sin(line);
                sin >> d.cluster >>  d.position.x() >> d.position.y() >> d.position.z() >> d.quartenion.w() >> d.quartenion.x() >> d.quartenion.y() >> d.quartenion.z();
                res.emplace_back(d);
                count++;
            }
        }
    }


    return res;
}
Eigen::Vector3f getPointLocationInWorldFrame(const cv::Point2i& image_point, const float& range,const range_image& range_image_info){

    // given pixel cooordinates and range, calculate the 3d points
     Eigen::Vector3f res;

    float angular_resolution_x=(float) (0.01f * (M_PI/180.0f));
    float angular_resolution_y =(float) (0.01f * (M_PI/180.0f));

    float angle_x, angle_y;
    // calculate ray angles
    angle_y = (image_point.x+static_cast<float> (range_image_info.image_offset_y))*(float)angular_resolution_x - 0.5f*static_cast<float> (M_PI);
    angle_x = (cosf (angle_y)==0.0f ? 0.0f : ( (image_point.y  + static_cast<float> (range_image_info.image_offset_x))*(float) angular_resolution_y - static_cast<float> (M_PI))/cosf (angle_y));

    res = Eigen::Vector3f (range * sinf (angle_x) * cosf (angle_y), range * sinf (angle_y), range * cosf (angle_x)*cosf (angle_y));
    Eigen::Affine3f sensorPose{range_image_info.sensorPose};
    res = sensorPose * res; // convert into world coordinate

    return res;
}

cv::Point2f getPixelCoordinates(const Eigen::Vector3f& point3D,const range_image& range_image_info){


    // Given the points in 3d calculate the pixel coordinates
    cv::Point2i res;
    // 3d points in sensor coordinate system. In case, the points are given
    //in world coordinate fram, transform them into sensor frame

    Eigen::Affine3f sensorPose{range_image_info.sensorPose};
    Eigen::Vector3f transformedPoint = sensorPose.inverse() * point3D; // convert into sensor coodinate
    auto range = transformedPoint.norm ();

    float angular_resolution_x_reciprocal_= 1.0/(float) (0.01f * (M_PI/180.0f)); // 1/ angular_resolution_x
    float angular_resolution_y_reciprocal_ =1.0/(float) (0.01f * (M_PI/180.0f)); // 1/ angular_resolution_y


    float angle_x = atanCalc (transformedPoint[0], transformedPoint[2]);
    float angle_y = asin(transformedPoint[1]/range);

    res.y = (angle_x* cos(angle_y) + static_cast<float> (M_PI))*angular_resolution_x_reciprocal_ - static_cast<float> (range_image_info.image_offset_x);
    res.x = (angle_y + 0.5f*static_cast<float> (M_PI))*angular_resolution_y_reciprocal_ - static_cast<float> (range_image_info.image_offset_y);

    return res;
}

range_image readRangeImageFile(const std::string& auditFolder){
    std::string rangeFile = auditFolder + "/rangeImage.txt";
    range_image range_image_info;

    std::ifstream configFile(rangeFile);
            if (configFile.is_open()) {
                std::string line;
                while (getline(configFile, line)) {
                    if(line[0] == '#' || line.empty())
                        continue;
                    std::istringstream sin(line.substr(line.find(":") + 1));
                    if (line.find("image_offset_x") != -1)
                        sin >> range_image_info.image_offset_x;
                    else if (line.find("image_offset_y") != -1)
                        sin >> range_image_info.image_offset_y;
                    else if (line.find("axis_x") != -1)
                        sin >> range_image_info.orientation[0];
                    else if (line.find("axis_y") != -1)
                        sin >> range_image_info.orientation[1];
                    else if (line.find("axis_z") != -1)
                        sin >> range_image_info.orientation[2];
                    else if (line.find("position_x") != -1)
                        sin >> range_image_info.translation[0];
                    else if (line.find("position_y") != -1)
                        sin >> range_image_info.translation[1];
                    else if (line.find("position_z") != -1)
                        sin >> range_image_info.translation[2];
                    else if (line.find("min") != -1)
                        sin >> range_image_info.min;
                    else if (line.find("max") != -1)
                        sin >> range_image_info.max;
                    }
            }
        cv::Affine3f profilometerPose{range_image_info.orientation,range_image_info.translation};
        range_image_info.sensorPose =profilometerPose;
        return range_image_info;
}






// Goes into math point cloud

float atanCalc (float y, float x){
  if (x==0 && y==0)
    return 0;
  float ret;
  if (fabsf (x) < fabsf (y)) {

    ret = atan(x/y) ;
    ret = static_cast<float> (x*y > 0 ? (180)/2-ret : -(180)/2-ret);
  }
  else
     ret = atan(y/x) ;
  if (x < 0)
    ret = static_cast<float> (y < 0 ? ret-(180) : ret+(180));

  return (ret);
}






Quaternion ToQuaternion(const float& yaw, const float& pitch, const float& roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

EulerAngles ToEulerAngles(const Eigen::Quaternionf& q) {
    EulerAngles angles;
    // ZYX Convention
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    float cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    float cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}


 Eigen::Matrix3f rotation_from_euler(float roll, float pitch, float yaw){
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










pcl::RangeImage convertPointCloud2RangeMap(const std::vector<drillTarget>& drillTargets_cad,const std::string& cloudPath,const params& params){


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
        throw std::runtime_error("Couldn't read cloud pcd file \n");
        }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_locations(new pcl::PointCloud<pcl::PointXYZ>);
     for (size_t i = 0; i < drillTargets_cad.size(); i++){
        cad_locations->points.emplace_back(drillTargets_cad.at(i).position[0],drillTargets_cad.at(i).position[1],drillTargets_cad.at(i).position[2]);
        }

    // // Estimate Lowest drill point
    // pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    // feature_extractor.setInputCloud (cad_locations);//default clould
    // feature_extractor.compute ();
    // pcl::PointXYZ min_point_AABB;
    // pcl::PointXYZ max_point_AABB;
    // feature_extractor.getAABB(min_point_AABB,max_point_AABB);

    // // Filter Point Cloud accordingly
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PassThrough<pcl::PointXYZ> pass;
 	// pass.setInputCloud (cloud);
  	// pass.setFilterFieldName ("z");
  	// pass.setFilterLimits ((min_point_AABB.z - params.marginBBZaxis),std::numeric_limits<float>::max());
  	// //pass.setFilterLimitsNegative (true);
  	// pass.filter (*cloud_filtered);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cad_locations);//default clould
    feature_extractor.compute ();
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getAABB(min_point_AABB,max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
   Eigen::Quaternionf quat(rotational_matrix_OBB);
    Eigen::Vector3f rot = rotational_matrix_OBB.eulerAngles(0,1,2);
     pcl::CropBox<pcl::PointXYZ> boxFilter;
     boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x- params.marginBBXaxis, min_point_OBB.y - params.marginBBYaxis , min_point_OBB.z - params.marginBBZaxis, 1.0));
     boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x + params.marginBBXaxis,  max_point_OBB.y +  params.marginBBYaxis ,max_point_OBB.z +  params.marginBBZaxis, 1.0));
     Eigen::Affine3f t;
     t.translation() = position;
     t.linear() = rotational_matrix_OBB;
     boxFilter.setTransform(t.inverse());

    // boxFilter.setRotation(quat.toRotationMatrix().eulerAngles(1,0,2));
    // boxFilter.setTranslation(position);
     boxFilter.setInputCloud(cloud);
     boxFilter.filter(*cloud_filtered);

     std::cout << " Cloud Filtered with oriented bounding box: " <<cloud_filtered->size() <<std::endl;

    if(params.modeOutLierRemovalActive == true){
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud_filtered);
        sor.setMeanK (params.meanK);
        sor.setStddevMulThresh (params.setStddevMulThresh);
        sor.setNegative (false);
        sor.filter (*cloud_filtered);
        }
    //pcl::io::savePCDFileBinaryCompressed ("/home/oguz/vs_code/datasets/tests_musp_15_02_2022/test_pcd.pcd", *cloud_filtered);


    
    float angularResolution_x = (float) (params.angular_resolution * (M_PI/180.0f));  // ADAPT IT!!
    float angular_resolution_y = (float) (params.angular_resolution * (M_PI/180.0f));
    float maxAngleWidth     = (float) (params.maxAngleWidth * (M_PI/180.0f));  // 180.0 degree in radians
    float maxAngleHeight    = (float) (params.maxAngleHeight * (M_PI/180.0f));  // 180.0 degree in radians

    Eigen::Affine3f sensorPose = calculateProfilometerPose(drillTargets_cad,params);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.0f;
    float minRange = 0.0f;
    int borderSize = params.borderSize;

    pcl::RangeImage rangeImage;
    //Creating Range image
    
    rangeImage.createFromPointCloud(*cloud_filtered, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    std::cout << " range image is ready! "<< std::endl;
    return rangeImage;
}


    std::vector<drillTarget> checkCenterTolerances(const std::vector<drillTarget>& drillTargets, const params& params){

    std::vector<drillTarget> res(drillTargets);

    pcl::PointCloud<pcl::PointXYZ>::Ptr drill_target_center_locations(new pcl::PointCloud<pcl::PointXYZ>);
    for(const auto d : res){
        if(d.misPrediction == false && d.differentCluster == false){
            pcl::PointXYZ p{d.position_aligned.x(),d.position_aligned.y(),d.position_aligned.z()};
            drill_target_center_locations->points.emplace_back(p);
        }
    }
    // Octree
     float resolution = 128.0f;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
    octree.setInputCloud (drill_target_center_locations);
    octree.addPointsFromInputCloud ();
    int K = 2;

    for (size_t i = 0; i < res.size(); i++){
        if(res.at(i).misPrediction == false && res.at(i).differentCluster == false){

        pcl::PointXYZ searchPoint{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2]};
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
            //if( std::sqrt(pointNKNSquaredDistance[0]) < maxCorrespondenceDistance)  {
                Eigen::Vector3f center_position_matched{(*drill_target_center_locations)[ pointIdxNKNSearch[1] ].x ,
                (*drill_target_center_locations)[ pointIdxNKNSearch[1] ].y,
                (*drill_target_center_locations)[ pointIdxNKNSearch[1] ].z};


              res.at(i).position_neigbour_center = center_position_matched;
                Eigen::Vector3f distance =    res.at(i).position_neigbour_center -    res.at(i).position_aligned;
                res.at(i).distanceToCenter = (std::sqrt(std::pow(distance.x(),2)+std::pow(distance.y(),2)+std::pow(distance.z(),2))/ 25.4) ;

                if(res.at(i).distanceToCenter >= params.centerToleranceMin)
                    res.at(i).withinCenterTolerance = true;
                else    
                    res.at(i).withinCenterTolerance = false;

                    std::cout <<" Center tolerance is: " << res.at(i).withinCenterTolerance  <<" with distance " << res.at(i).distanceToCenter <<" inch " <<std::endl;

            }
        }else{
            res.at(i).withinCenterTolerance =false;
            res.at(i).distanceToCenter = 0;
            res.at(i).position_neigbour_center = res.at(i).position_aligned; 
        }
    }

    std::cout <<"Center Distances Calculated" << std::endl;

    //showCenterDistances(res, params.cloudPath);

    return res;
    }



// void createRangeImages(const std::vector<drillTarget>& drillTargets_cad,
//                         const params& params,
//                          const std::string& cloudPath,
//                          const std::string& storagePath_rangeImage,
//                          const std::string& storagePath_rangeFile){



//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//     if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
//         throw std::runtime_error("Couldn't read cloud pcd file \n");
//     }


//     pcl::PointCloud<pcl::PointXYZ>::Ptr cad_locations(new pcl::PointCloud<pcl::PointXYZ>);
//      for (size_t i = 0; i < drillTargets_cad.size(); i++){
//         cad_locations->points.emplace_back(drillTargets_cad.at(i).position[0],drillTargets_cad.at(i).position[1],drillTargets_cad.at(i).position[2]);
//     }


    
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//      pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
//     feature_extractor.setInputCloud (cad_locations);//default clould
//     feature_extractor.compute ();
//     pcl::PointXYZ min_point_AABB;
//     pcl::PointXYZ max_point_AABB;
//     pcl::PointXYZ min_point_OBB;
//     pcl::PointXYZ max_point_OBB;
//     pcl::PointXYZ position_OBB;
//     Eigen::Matrix3f rotational_matrix_OBB;
//     feature_extractor.getAABB(min_point_AABB,max_point_AABB);
//     feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
//     Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
//    Eigen::Quaternionf quat(rotational_matrix_OBB);
//     Eigen::Vector3f rot = rotational_matrix_OBB.eulerAngles(0,1,2);
//      pcl::CropBox<pcl::PointXYZ> boxFilter;
//      boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x- params.marginBBXaxis, min_point_OBB.y - params.marginBBYaxis , min_point_OBB.z - params.marginBBZaxis, 1.0));
//      boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x + params.marginBBXaxis,  max_point_OBB.y +  params.marginBBYaxis ,max_point_OBB.z +  params.marginBBZaxis, 1.0));
//      Eigen::Affine3f t;
//      t.translation() = position;
//      t.linear() = rotational_matrix_OBB;
//      boxFilter.setTransform(t.inverse());

//     // boxFilter.setRotation(quat.toRotationMatrix().eulerAngles(1,0,2));
//     // boxFilter.setTranslation(position);
//      boxFilter.setInputCloud(cloud);
//      boxFilter.filter(*cloud_filtered);

//      std::cout << " Cloud Filtered with oriented bounding box: " << cloud_filtered->size() <<std::endl;


//     std::vector<drillTarget> drillTargets_cad_set = setParameters(drillTargets_cad);
//     for (size_t i = 0; i < ((drillTargets_cad_set.size() / 2) + 1); i++){    
    
//         if(!drillTargets_cad_set.empty()){
//             pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
//             std::vector<drillTarget>::iterator it = drillTargets_cad_set.begin();
//             std::vector<drillTarget> cluster_target;
//             for (size_t i = 0; i < 2; i++){
//                 pcl::PointXYZ p{drillTargets_cad_set.at(i).position.x(),drillTargets_cad_set.at(i).position.y(),drillTargets_cad_set.at(i).position.z()};
//                 cluster->points.emplace_back(p);
//                 cluster_target.emplace_back(drillTargets_cad_set.at(i));
//                 drillTargets_cad_set.erase(it + i);
//             }
//         Eigen::Affine3f sensor_poses = calculateProfilometerPose(cluster_target,params);


//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor_cluster;
//         feature_extractor_cluster.setInputCloud (cluster);//default clould
//         feature_extractor_cluster.compute();
//         pcl::PointXYZ min_point_OBB;
//         pcl::PointXYZ max_point_OBB;
//         pcl::PointXYZ position_OBB;
//         Eigen::Matrix3f rotational_matrix_OBB;
//         feature_extractor_cluster.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
//         Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);

//         pcl::CropBox<pcl::PointXYZ> boxFilter;
//         boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x- params.marginBBXaxis, min_point_OBB.y - params.marginBBYaxis , min_point_OBB.z - params.marginBBZaxis, 1.0));
//         boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x + params.marginBBXaxis,  max_point_OBB.y +  params.marginBBYaxis ,max_point_OBB.z +  params.marginBBZaxis, 1.0));
//         Eigen::Affine3f t;
//         t.translation() = position;
//         t.linear() = rotational_matrix_OBB;
//         boxFilter.setTransform(t.inverse());
//         boxFilter.setInputCloud(cloud_filtered);
//         boxFilter.filter(*cloud_filtered_cluster);

//          std::cout << " Cloud Filtered with oriented bounding box: " << cloud_filtered_cluster->size() <<std::endl;

//         float angularResolution_x = (float) (params.angular_resolution * (M_PI/180.0f));  // ADAPT IT!!
//         float angular_resolution_y = (float) (params.angular_resolution * (M_PI/180.0f));
//         float maxAngleWidth     = (float) (params.maxAngleWidth * (M_PI/180.0f));  // 180.0 degree in radians
//         float maxAngleHeight    = (float) (params.maxAngleHeight * (M_PI/180.0f));  // 180.0 degree in radians

//         //Eigen::Affine3f sensorPose = calculateProfilometerPose(drillTargets_cad_set,params);

//         pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//         float noiseLevel=0.0f;
//         float minRange = 0.0f;
//         int borderSize = params.borderSize;

//         pcl::RangeImage rangeImage;

//         //Creating Range image
//         rangeImage.createFromPointCloud(*cloud_filtered_cluster, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
//                                     sensor_poses, coordinate_frame, noiseLevel, minRange, borderSize);

        

//         cv::Mat range_map(rangeImage.height, rangeImage.width, CV_8UC3);

  
    
//         rangeImage.setUnseenToMaxRange();
//         // normalize
//         float min, max;
//         rangeImage.getMinMaxRanges(min,max);

//     // create range map or binary image
//         for (int h = 0; h < rangeImage.height; h++){
//             for (int w = 0; w < rangeImage.width; w++){
//                     if(isinf(rangeImage.at(w,h).range)){
//                 range_map.at<cv::Vec3b>(h,w)[0] = 0;
//                 range_map.at<cv::Vec3b>(h,w)[1] = 0;
//                 range_map.at<cv::Vec3b>(h,w)[2] = 0;

//             }else{
//                 range_map.at<cv::Vec3b>(h,w)[0]= ((rangeImage.at(w,h).range) / max)*255; // Normalize for color image
//                 range_map.at<cv::Vec3b>(h,w)[1]= ((rangeImage.at(w,h).range) / max)*255;
//                 range_map.at<cv::Vec3b>(h,w)[2]= ((rangeImage.at(w,h).range) / max)*255;

//             }
//         }
//     }
//         if(!range_map.empty()){
//             std::cout << " range image is stored! "<< std::endl;

//             //Eigen::Affine3f sensorPose_ = calculateProfilometerPose(drillTargets_cad,params);
//             //writeRangeImageFile(rangeImage,storagePath_rangeFile,sensorPose_);
//             std::string storagePath_rangeImage = params.auditFolder + cv::format("/output/rangeImage_%d.png",i);
//             cv::imwrite(storagePath_rangeImage,range_map);
//         }else{
//             std::cout << "range image is empty!" << std::endl;
//         }
//     }
//     }
// }



cv::Mat createRangeImage(const std::vector<drillTarget>& drillTargets_cad,
                        const params& params,
                         const std::string& cloudPath,
                         const std::string& storagePath_rangeImage,
                         const std::string& storagePath_rangeFile){

    std::vector<drillTarget> drillTargets_cad_set = setParameters(drillTargets_cad);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
        throw std::runtime_error("Couldn't read cloud pcd file \n");
        }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_locations(new pcl::PointCloud<pcl::PointXYZ>);
     for (size_t i = 0; i < drillTargets_cad.size(); i++){
        cad_locations->points.emplace_back(drillTargets_cad.at(i).position[0],drillTargets_cad.at(i).position[1],drillTargets_cad.at(i).position[2]);
        }


    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cad_locations);//default clould
    feature_extractor.compute ();
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getAABB(min_point_AABB,max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
   Eigen::Quaternionf quat(rotational_matrix_OBB);
    Eigen::Vector3f rot = rotational_matrix_OBB.eulerAngles(0,1,2);
     pcl::CropBox<pcl::PointXYZ> boxFilter;
     boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x- params.marginBBXaxis, min_point_OBB.y - params.marginBBYaxis , min_point_OBB.z - params.marginBBZaxis, 1.0));
     boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x + params.marginBBXaxis,  max_point_OBB.y +  params.marginBBYaxis ,max_point_OBB.z +  params.marginBBZaxis, 1.0));
     Eigen::Affine3f t;
     t.translation() = position;
     t.linear() = rotational_matrix_OBB;
     boxFilter.setTransform(t.inverse());

    // boxFilter.setRotation(quat.toRotationMatrix().eulerAngles(1,0,2));
    // boxFilter.setTranslation(position);
     boxFilter.setInputCloud(cloud);
     boxFilter.filter(*cloud_filtered);

     std::cout << " Cloud Filtered with oriented bounding box: " << cloud_filtered->size() <<std::endl;
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_removal (new pcl::PointCloud<pcl::PointXYZ>);

    if(params.modeOutLierRemovalActive == true){
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud_filtered);
        sor.setMeanK (params.meanK);
        sor.setStddevMulThresh (params.setStddevMulThresh);
        sor.setNegative (false);
        sor.filter (*cloud_filtered_removal);
        }
    pcl::io::savePCDFileBinaryCompressed (cloudPath, *cloud_filtered_removal);

     //std::cout << " Removal outlier: " << cloud_filtered_removal->size() <<std::endl;

    
    float angularResolution_x = (float) (params.angular_resolution * (M_PI/180.0f));  // ADAPT IT!!
    float angular_resolution_y = (float) (params.angular_resolution * (M_PI/180.0f));
    float maxAngleWidth     = (float) (params.maxAngleWidth * (M_PI/180.0f));  // 180.0 degree in radians
    float maxAngleHeight    = (float) (params.maxAngleHeight * (M_PI/180.0f));  // 180.0 degree in radians

    Eigen::Affine3f sensorPose = calculateProfilometerPose(drillTargets_cad_set,params);
   // Eigen::Affine3f sensorPose = calculateProfilometerPoseBBMiddle(drillTargets_cad_set,params);

    // //NEW
    // Eigen::Affine3f tcp = drillTargets_cad_set.at(1).homogeneousMat;
    // // Profilometer calibration
    // Eigen::Matrix3f profi_rotation = rotation_from_euler(params.profilometerCalibRotation[0],params.profilometerCalibRotation[1],params.profilometerCalibRotation[2]);
    // Eigen::Vector3f profi_translation{params.profilometerCalibTranslation};

    // Eigen::Vector3f profiToSensor = profi_rotation * profi_translation;
    // Eigen::Affine3f sensorPose = tcp.translate((-1)*profiToSensor);
    // //NEW
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.0f;
    float minRange = 0.0f;
    int borderSize = params.borderSize;

    pcl::RangeImage rangeImage;
    //Creating Range image
    if(params.modeOutLierRemovalActive == true){
        rangeImage.createFromPointCloud(*cloud_filtered_removal, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    }else{
        rangeImage.createFromPointCloud(*cloud_filtered, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    }
    std::cout << " range image is created! "<< std::endl;

    //pcl::RangeImage rangeImage = convertPointCloud2RangeMap(drillTargets_cad_set,cloudPath,params);

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
    //cv::resize(range_map, range_map, cv::Size(range_map.cols * 0.25 , range_map.rows * 0.25 ), 0, 0);
    //cv::imshow("p", range_map);
    //cv::waitKey(0);
    }
    if(!range_map.empty()){
        Eigen::Affine3f sensorPose_ = calculateProfilometerPose(drillTargets_cad,params);
        writeRangeImageFile(rangeImage,storagePath_rangeFile,sensorPose_);
        cv::imwrite(storagePath_rangeImage,range_map);
    }else{
        std::cout << "range image is empty!" << std::endl;
    }
    return range_map;

}

void writeRangeImageFile(const pcl::RangeImage& rangeImage,const std::string& auditFolder,const Eigen::Affine3f& profilometerPose ){

        //std::string cloudFolder = "/home/oguz/vs_code/lma_cam2D/inputs/new_dataset_mockup/3";
        //std::string rangeFile = auditFolder + "/rangeImage.txt";
        std::ofstream file (auditFolder);
        file << rangeImage << std::endl;
        file  <<"image_offset_x: " <<rangeImage.getImageOffsetX () << std::endl;
        file  <<"image_offset_y: " <<rangeImage.getImageOffsetY () << std::endl;
        float min,max;
        rangeImage.getMinMaxRanges (min,max);
        file  <<"max_range: " << max << std::endl;
        file  <<"min_range: " << min << std::endl;
         file  <<"Profilometer Pose"  << std::endl;
        file << "position_x: " <<  profilometerPose.translation() << std::endl;
        //file << "position_y: " <<  profilometerPose.translation()[1] << std::endl;
        //file << "position_z: " <<  profilometerPose.translation()[2] << std::endl;
        file << "axis_x: " <<  profilometerPose.linear() << std::endl;
        //file << "axis_y: " <<  profilometerPose.linear()[1] << std::endl;
        //file << "axis_z: " <<  profilometerPose.linear()[2] << std::endl;
        file.close();
}

     drillTarget setParameters3DCamera(const drillTarget drillTargets_CAD){
         drillTarget res(drillTargets_CAD);
         EulerAngles ea = ToEulerAngles(res.quartenion);
         res.rotation_matrix = rotation_from_euler(ea.roll,ea.pitch,ea.yaw);
         res.homogeneousMat.linear() = res.rotation_matrix;
         res.homogeneousMat.translation() = res.position;

         return res;
     }

 cv::Mat createRangeImage3DCamera(const drillTarget& cameraPose,
                                    const params& params,
                                    const std::string& cloudPath,
                                    const std::string& storagePath_rangeImage){


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud);


         float angularResolution_x = (float) (params.angular_resolution * (M_PI/180.0f));  // ADAPT IT!!
         float angular_resolution_y = (float) (params.angular_resolution * (M_PI/180.0f));
         float maxAngleWidth     = (float) (params.maxAngleWidth * (M_PI/180.0f));  // 180.0 degree in radians
         float maxAngleHeight    = (float) (params.maxAngleHeight * (M_PI/180.0f));  // 180.0 degree in radians

         //NEW
         drillTarget  cameraPose_set = setParameters3DCamera(cameraPose);
         Eigen::Affine3f tcp = cameraPose_set.homogeneousMat;
         // Profilometer calibration
         Eigen::Matrix3f profi_rotation = rotation_from_euler(params.profilometerCalibRotation[0],params.profilometerCalibRotation[1],params.profilometerCalibRotation[2]);
         Eigen::Vector3f profi_translation{params.profilometerCalibTranslation};

         Eigen::Vector3f profiToSensor = profi_rotation * profi_translation;
         Eigen::Affine3f sensorPose = tcp.translate((-1)*profiToSensor);
         //NEW
         pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
         float noiseLevel=0.0f;
         float minRange = 0.0f;
         int borderSize = params.borderSize;

         pcl::RangeImage rangeImage;
         //Creating Range image

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
             //Eigen::Affine3f sensorPose_ = calculateProfilometerPose(drillTargets_cad,params);
             //writeRangeImageFile(rangeImage,storagePath_rangeFile,sensorPose_);
             cv::imwrite(storagePath_rangeImage,range_map);
         }else{
             std::cout << "range image is empty!" << std::endl;
         }
         return range_map;
    }



    std::vector<drillTarget> startInspection3DCamera(const std::vector<boundingBox>& BBs,
                                               const std::vector<drillTarget>& drillTargets_CAD,
                                               const drillTarget& cameraPose,
                                               const params& params,
                                               const std::string& cloudPath){

            std::cout << "=====> Matching Holes" << std::endl;

           std::vector<drillTarget> allDrillTargets = matchHoles3DCamera(BBs,drillTargets_CAD,cameraPose,params,cloudPath);

        showDrillTargets(allDrillTargets,cloudPath);

           std::cout << "=====> Inspecting  Holes" << std::endl;

           std::vector<drillTarget> allDrillTargets_inspected = inspectDrillTargets3DCamera(allDrillTargets,drillTargets_CAD,cameraPose,params,cloudPath);

           return allDrillTargets_inspected;
       }


       std::vector<drillTarget> matchHoles3DCamera(const std::vector<boundingBox>& BBs,
                                               const std::vector<drillTarget>& drillTargets_CAD,
                                               const drillTarget& cameraPose,
                                               const params& params,
                                               const std::string& cloudPath){

       std::cout << "=====> Launch drill target characteristics examination!!! " << std::endl;

       std::vector<drillTarget> res;

       std::vector<drillTarget> drillTargetsset_CAD = setParameters(drillTargets_CAD);

       drillTarget cameraPose_set = setParameters3DCamera(cameraPose);

       std::cout << "=====> Parameters set " << std::endl;

       if(!BBs.empty()){

           std::cout << "=====> Extracting holes in Bounding boxes " << std::endl;

           std::vector<drillTarget> detectedHoles = extractHoles3DCamera(BBs,drillTargetsset_CAD,cameraPose_set,cloudPath,params); // extract hole centers and Labels; unpiloted,piloted,fullsized

           std::cout << "=====> Finding corresponding points " << std::endl;

           std::vector<drillTarget> detectedHoles_matched = findCorrespondences(drillTargetsset_CAD,detectedHoles, params); // find the correspondence between cad targets&detected targets discard misprediction

           std::cout << "=====> Applying transformation to unpiloted holes " << std::endl;

           std::vector<drillTarget> allDrillTargets = alignUnpilotedDrillTargets(drillTargetsset_CAD,detectedHoles_matched,params,cloudPath); // Apply transformation

           for (const auto dT : allDrillTargets){
               res.emplace_back(dT);
           }
       }
       else{
           std::cout << "=====> No detection found " << std::endl;
           for (auto& dT :  drillTargetsset_CAD){
               dT.unpiloted =true;
               dT.position_aligned=dT.position;
               dT.quartenion_cad = dT.quartenion;
               res.emplace_back(dT);}
       }

       std::cout << "=====> Drill target characteristics examination is over!!! " << std::endl;

       return res;
   }



    std::vector<drillTarget>  extractHoles3DCamera(const std::vector<boundingBox>& BBs,
                                            const std::vector<drillTarget> drillTargets_cad,
                                            const drillTarget& cameraPose,
                                            const std::string& cloudPath,
                                            const params& params){

            std::vector<drillTarget> detectedHoles;


           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
           pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud);



           float angularResolution_x = (float) (params.angular_resolution * (M_PI/180.0f));  // ADAPT IT!!
           float angular_resolution_y = (float) (params.angular_resolution * (M_PI/180.0f));
           float maxAngleWidth     = (float) (params.maxAngleWidth * (M_PI/180.0f));  // 180.0 degree in radians
           float maxAngleHeight    = (float) (params.maxAngleHeight * (M_PI/180.0f));  // 180.0 degree in radians

              //NEW
           Eigen::Affine3f tcp = cameraPose.homogeneousMat;
           // Profilometer calibration
           Eigen::Matrix3f profi_rotation = rotation_from_euler(params.profilometerCalibRotation[0],params.profilometerCalibRotation[1],params.profilometerCalibRotation[2]);
           Eigen::Vector3f profi_translation{params.profilometerCalibTranslation};

           Eigen::Vector3f profiToSensor = profi_rotation * profi_translation;
           Eigen::Affine3f sensorPose = tcp.translate((-1)*profiToSensor);
           //NEW

           pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
           float noiseLevel=0.0f;
           float minRange = 0.0f;
           int borderSize = params.borderSize;

           pcl::RangeImage rangeImage;
           //Creating Range image

           rangeImage.createFromPointCloud(*cloud, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                       sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

           std::cout << " range image is ready! "<< std::endl;

           std::vector<Eigen::Vector3f> positions_cicle_extractions;
           std::vector<Eigen::Vector3f> positions_centroid;
        std::vector<pcl::PointXYZ> add_lines; // for visualization//#######################
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_all(new pcl::PointCloud<pcl::PointXYZ>); // for visualization//#######################
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_all_Real(new pcl::PointCloud<pcl::PointXYZ>); // for visualization//#######################

           for (int k = 0; k < BBs.size(); k++){
               std::cout << "Extracting Bounding Box: " << k << std::endl;
               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB(new pcl::PointCloud<pcl::PointXYZ>);
                   for (int u = BBs.at(k).topLeft.y; u < BBs.at(k).bottomRight.y; u++){
                       for (int v = BBs.at(k).topLeft.x; v <  BBs.at(k).bottomRight.x; v++){
                           if(isinf(rangeImage.at(v,u).range)){
                       }else{
                           pcl::PointXYZ p{rangeImage.at(v,u).x,rangeImage.at(v,u).y,rangeImage.at(v,u).z};
                           cloud_BB->points.emplace_back(p);
                           cloud_BB_all->points.emplace_back(p);
                           }

                   }
               }

                   //NEw
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_real (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
            feature_extractor.setInputCloud (cloud_BB);//default clould
            feature_extractor.compute ();
            pcl::PointXYZ min_point_OBB;
            pcl::PointXYZ max_point_OBB;
            pcl::PointXYZ position_OBB;
            Eigen::Matrix3f rotational_matrix_OBB;
            feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
            Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
            pcl::CropBox<pcl::PointXYZ> boxFilter;
            boxFilter.setMin(Eigen::Vector4f(min_point_OBB.x, min_point_OBB.y , min_point_OBB.z - 5, 1.0));
            boxFilter.setMax(Eigen::Vector4f(max_point_OBB.x,  max_point_OBB.y  ,max_point_OBB.z + 5 , 1.0));
            Eigen::Affine3f t;
            t.translation() = position;
            t.linear() = rotational_matrix_OBB;
            boxFilter.setTransform(t.inverse());

            boxFilter.setInputCloud(cloud);
            boxFilter.filter(*cloud_BB_real);

           Eigen::Vector4f centroid;
           pcl::compute3DCentroid(*cloud_BB_real,centroid);
           Eigen::Vector3f pos_centroid{centroid.x(),centroid.y(),centroid.z()};

           std::cout << " cloud with the BB created! "<< std::endl;

           drillTarget detectedHole;
           detectedHole.BB = BBs.at(k);
           detectedHole.position_aligned = pos_centroid;
           detectedHole.cluster = drillTargets_cad.front().cluster;

           int width = detectedHole.BB.bottomRight.x - detectedHole.BB.topLeft.x ;
           int height= detectedHole.BB.bottomRight.y - detectedHole.BB.topLeft.y ;
           std::cout << width << "," << height << std::endl;

           if(width > params.fullSizeHole || height > params.fullSizeHole)
               detectedHole.fullSized =true;
           else
               detectedHole.piloted =true;

           std::vector<float> hole_coeeffecients(detect3DCirclePC(cloud_BB));
           if(!hole_coeeffecients.empty()){
               Eigen::Vector3f pos{hole_coeeffecients.at(0),hole_coeeffecients.at(1),hole_coeeffecients.at(2)};
               Eigen::Vector3f nor{hole_coeeffecients.at(4),hole_coeeffecients.at(5),hole_coeeffecients.at(6)};
               drillTarget detectedHole;
               detectedHole.normals = nor;
               detectedHole.detectedHoleDiameter = hole_coeeffecients.at(3) * 2;

               std::vector<float> coeff;
               coeff.emplace_back(hole_coeeffecients.at(4));
               coeff.emplace_back(hole_coeeffecients.at(5));
               coeff.emplace_back(hole_coeeffecients.at(6));
               Eigen::Matrix3f rotation_matrix_ = calculateQuartenions(coeff);
               Eigen::Quaternionf q(rotation_matrix_);
               detectedHole.quartenion = q;

                                       //#######################
            // for visualization
        pcl::PointXYZ p1{hole_coeeffecients.at(0) + (hole_coeeffecients.at(4) * -5),
        hole_coeeffecients.at(1) + (hole_coeeffecients.at(5) * -5),
        hole_coeeffecients.at(2) + (hole_coeeffecients.at(6) * -5)};
        pcl::PointXYZ p2{hole_coeeffecients.at(0) + (hole_coeeffecients.at(4) * +5),
        hole_coeeffecients.at(1) + (hole_coeeffecients.at(5) * +5),
        hole_coeeffecients.at(2) + (hole_coeeffecients.at(6) * +5)};
        add_lines.emplace_back(p1);
        add_lines.emplace_back(p2);
    
        positions_cicle_extractions.emplace_back(pos);
        positions_centroid.emplace_back(pos_centroid);
        
        // for visualization
        //#######################

            }else{
                   std::cout << " Hole normals can not be extracted " <<std::endl;
            }
             detectedHoles.emplace_back(detectedHole);
       }

       std::cout << "Extraction is done! Total number detected holes is: " << detectedHoles.size() << std::endl;
        showExtractedHoles(cloud_BB_all,params.cloudPath,add_lines);
        //showCircles(cloud_BB_all,params.cloudPath,coefficient_all);
        //showCenters(positions_cicle_extractions,positions_centroid,params.cloudPath);
       return detectedHoles;
    }





      

    std::vector<drillTarget> inspectDrillTargets3DCamera(const std::vector<drillTarget>& drillTargets,
                                                 const std::vector<drillTarget>& drillTargets_CAD,
                                                const drillTarget& cameraPose,
                                                const params& params,
                                                const std::string& cloudPath){

            std::cout << "=====> Launch Edge&Center Tolerance inspection!!! " << std::endl;

            std::vector<drillTarget> drillTargetsset_CAD = setParameters(drillTargets_CAD);

            drillTarget cameraPose_set = setParameters3DCamera(cameraPose);

            std::cout << "=====> Parameters set " << std::endl;

            std::cout << "=====> Inspecting edge tolerances " << std::endl;

            std::vector<drillTarget> drillTargets_edge_inspected =  checkEdgeTolerances_3DCamera(drillTargets,drillTargetsset_CAD,params,cameraPose_set.homogeneousMat,cloudPath); // Check tolerace to edges

            std::cout << "=====> Inspecting center tolerances " << std::endl;

            std::vector<drillTarget> drillTargets_center_edge_inspected = checkCenterTolerances(drillTargets_edge_inspected,params); // check tolerance between centers

            std::cout << "=====> Edge&Center Tolerance inspection is over!!! " << std::endl;

            return drillTargets_center_edge_inspected;
    }

 pcl::PointCloud<pcl::PointXYZ>::Ptr createRangeImageBorderExtaction3DCamera( const std::vector<drillTarget>& drillTargets,
                                                                         const std::vector<drillTarget>& drillTargets_CAD,
                                                                        const params& params,
                                                                        const Eigen::Affine3f& sensorPose_,
                                                                        const std::string& cloudPath){

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud);

        std::cout << "----->> Point cloud loaded" << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr border_points(new pcl::PointCloud<pcl::PointXYZ>);

        float angularResolution_x = (float) (params.angular_resolution_border * (M_PI/180.0f));  // ADAPT IT!!
        float angular_resolution_y = (float) (params.angular_resolution_border * (M_PI/180.0f));
        float maxAngleWidth     = (float) (params.maxAngleWidth * (M_PI/180.0f));  // 180.0 degree in radians
        float maxAngleHeight    = (float) (params.maxAngleHeight * (M_PI/180.0f));  // 180.0 degree in radians


        //Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);//
        Eigen::Affine3f sensorPose = sensorPose_;
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
        float noiseLevel=0.0f;
        float minRange = 0.0f;
        int borderSize = params.borderSize;


        pcl::RangeImage rangeImageBorder;
        // Creating Range image
        rangeImageBorder.createFromPointCloud(*cloud, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

        std::cout << "----->> Range image for border extraction is ready" << std::endl;

        rangeImageBorder.setUnseenToMaxRange();

        pcl::RangeImageBorderExtractor border_extractor (&rangeImageBorder);
        pcl::PointCloud<pcl::BorderDescription> border_descriptions;
        border_extractor.compute(border_descriptions);

        for (int y=0; y< (int)rangeImageBorder.height; ++y){
            for (int x=0; x< (int)rangeImageBorder.width; ++x) {
                if (border_descriptions[y*rangeImageBorder.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER]){
                    pcl::PointXYZ p{rangeImageBorder[y*rangeImageBorder.width + x].x,rangeImageBorder[y*rangeImageBorder.width + x].y,rangeImageBorder[y*rangeImageBorder.width + x].z};
                    border_points->points.emplace_back(p);
                }
        }
      }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
   // Neighbors within radius search
    for (size_t i = 0; i < drillTargets.size(); i++){
        pcl::PointXYZ searchPoint{drillTargets.at(i).position_aligned.x(),drillTargets.at(i).position_aligned.y(),drillTargets.at(i).position_aligned.z()};
        float resolution = 128.0f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
        octree.setInputCloud(border_points);
        octree.addPointsFromInputCloud();
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if (octree.radiusSearch (searchPoint,  params.radiusBorderFiltering , pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
            for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                inliers->indices.emplace_back(pointIdxRadiusSearch[i]);
            }
        }
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(border_points);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*border_points);

    std::cout << " Border Extaction is completed! "<<border_points->points.size() <<std::endl;

    return border_points;

    }
/*
    std::vector<drillTarget> checkEdgeTolerances_3DCamera(const std::vector<drillTarget>& drillTargets,
                                                 const std::vector<drillTarget>& drillTargets_CAD,
                                                 const params& params,
                                                 const Eigen::Affine3f& sensorPose,
                                                 const std::string& cloudPath){

        std::vector<drillTarget> res(drillTargets);

        pcl::PointCloud<pcl::PointXYZ>::Ptr border_points_ptr_cloud = createRangeImageBorderExtaction3DCamera(drillTargets,drillTargets_CAD,params,sensorPose,cloudPath); // Extract border points

        showBorderPoints(border_points_ptr_cloud);

        std::cout << "----->> Calculating distances to the borders " << std::endl;




        // Octree
         float resolution = 128.0f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
        octree.setInputCloud (border_points_ptr_cloud);
        octree.addPointsFromInputCloud ();
        int K = params.numberOfBorderNeighbour;
        if(!border_points_ptr_cloud->points.empty()){
            for (size_t i = 0; i < res.size(); i++){
                if(res.at(i).misPrediction == false && res.at(i).differentCluster == false){
                    pcl::PointXYZ searchPoint{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2]};
                    std::vector<int> pointIdxNKNSearch;
                    std::vector<float> pointNKNSquaredDistance;
                    if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
                        pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_neigbour_ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                        for (size_t i = 0; i < K; i++){
                            pcl::PointXYZ p{(*border_points_ptr_cloud)[ pointIdxNKNSearch[i] ].x,(*border_points_ptr_cloud)[ pointIdxNKNSearch[i] ].y,(*border_points_ptr_cloud)[ pointIdxNKNSearch[i] ].z};
                            nearest_neigbour_ptr_cloud->points.emplace_back(p);
                        }

                        // Segment a line
                        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                        // Create the segmentation object
                        pcl::SACSegmentation<pcl::PointXYZ> seg;
                        // Optional
                        seg.setOptimizeCoefficients (true);
                        // Mandatory
                        seg.setModelType (pcl::SACMODEL_LINE);
                        seg.setMethodType (pcl::SAC_RANSAC);
                        seg.setMaxIterations(100);
                        seg.setDistanceThreshold (5);
                        //seg.setEpsAngle(0.01);
                        seg.setInputCloud (nearest_neigbour_ptr_cloud);
                        seg.segment (*inliers, *coefficients);

                        Eigen::Vector4f position{coefficients->values[0],coefficients->values[1],coefficients->values[2],0};
                        Eigen::Vector4f direction{ coefficients->values[3],coefficients->values[4],coefficients->values[5],0};
                        Eigen::Vector4f point{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2],0};

                         Eigen::Vector3f pos_first{coefficients->values[0] - (coefficients->values[3] * 5 ),(coefficients->values[1] - coefficients->values[4] * 5) ,(coefficients->values[2] - coefficients->values[5] * 5 )};
                         Eigen::Vector3f position_second{coefficients->values[0] + (coefficients->values[3] * 5 ),(coefficients->values[1]+ coefficients->values[4] * 5) ,(coefficients->values[2]+coefficients->values[5] * 5 )};
                        Eigen::Vector3f pos_center{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2]};

                        Eigen::Vector3f dif = pos_first - position_second;
                        float LineMag = std::sqrt(std::pow(dif.x(),2)+ std::pow(dif.y(),2) +std::pow(dif.z(),2));
                        float U = (((pos_center.x() - pos_first.x()) * (position_second.x() - pos_first.x())) +
                                    ((pos_center.y() - pos_first.y()) * (position_second.y() - pos_first.y())) +
                                    ((pos_center.z() - pos_first.z()) * (position_second.z() - pos_first.z()))) /
                            (LineMag * LineMag);

                        Eigen::Vector3f Intersection = pos_first + (U * (position_second- pos_first));
                        Eigen::Vector3f dif_fo = pos_center - Intersection;
                        float Distance = std::sqrt(std::pow(dif_fo.x(),2)+ std::pow(dif_fo.y(),2) +std::pow(dif_fo.z(),2));


                        double distance  = std::sqrt(pcl::sqrPointToLineDistance(point,position,direction));
                        res.at(i).distanceToEdge = (float)(distance / 25.4);
                        res.at(i).position_edge = Intersection;

                        if(res.at(i).distanceToEdge >= params.edgeToleranceMin ){
                                res.at(i).withinEdgeTolerance = true;
                        }else{
                                res.at(i).withinEdgeTolerance = false;
                        }
                        std::cout <<  "Edge tolerance is: " <<res.at(i).withinEdgeTolerance <<" with the distance " << res.at(i).distanceToEdge << std::endl;
                    }
                }else{
                    res.at(i).withinEdgeTolerance =false;
                    res.at(i).distanceToEdge = 0;
                    res.at(i).position_edge =res.at(i).position_aligned ;
                    std::cout <<  "Edge tolerance is: " <<res.at(i).withinEdgeTolerance <<" with the distance " << res.at(i).distanceToEdge << std::endl;

                }
            }
        }else{
             std::cout <<  "Borders could not be extracted!!" << std::endl;

            for (size_t i = 0; i < res.size(); i++){
                       res.at(i).withinEdgeTolerance =false;
                       res.at(i).position_edge = res.at(i).position_aligned;
                       res.at(i).distanceToEdge = 0;
              }
        }

        showEdgeDistances(res,params.cloudPath);

        std::cout << "Edge Distances Calculated!" << std::endl;
        return res;
}
*/

    std::vector<drillTarget> checkEdgeTolerances_3DCamera(const std::vector<drillTarget>& drillTargets,
                                                 const std::vector<drillTarget>& drillTargets_CAD,
                                                 const params& params,
                                                 const Eigen::Affine3f& sensorPose,
                                                 const std::string& cloudPath){

        std::vector<drillTarget> res(drillTargets);

        pcl::PointCloud<pcl::PointXYZ>::Ptr border_points_ptr_cloud = createRangeImageBorderExtaction3DCamera(drillTargets,drillTargets_CAD,params,sensorPose,cloudPath); // Extract border points

        showBorderPoints(border_points_ptr_cloud);

        std::cout << "----->> Calculating distances to the borders " << std::endl;


        // NEW
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud);
    
        // NEW

        // Octree
         float resolution = 128.0f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
        octree.setInputCloud (border_points_ptr_cloud);
        octree.addPointsFromInputCloud ();
        int K = params.numberOfBorderNeighbour;
        if(!border_points_ptr_cloud->points.empty()){
            for (size_t i = 0; i < res.size(); i++){
                if(res.at(i).misPrediction == false && res.at(i).differentCluster == false){
                    pcl::PointXYZ searchPoint{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2]};
                    std::vector<int> pointIdxNKNSearch;
                    std::vector<float> pointNKNSquaredDistance;
                    if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
                        pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_neigbour_ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                        for (size_t i = 0; i < K; i++){
                            pcl::PointXYZ p{(*border_points_ptr_cloud)[ pointIdxNKNSearch[i] ].x,(*border_points_ptr_cloud)[ pointIdxNKNSearch[i] ].y,(*border_points_ptr_cloud)[ pointIdxNKNSearch[i] ].z};
                            nearest_neigbour_ptr_cloud->points.emplace_back(p);
                        }

                        // Segment a line
                        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                        // Create the segmentation object
                        pcl::SACSegmentation<pcl::PointXYZ> seg;
                        // Optional
                        seg.setOptimizeCoefficients (true);
                        // Mandatory
                        seg.setModelType (pcl::SACMODEL_LINE);
                        seg.setMethodType (pcl::SAC_RANSAC);
                        seg.setMaxIterations(100);
                        seg.setDistanceThreshold (5);
                        //seg.setEpsAngle(0.01);
                        seg.setInputCloud (nearest_neigbour_ptr_cloud);
                        seg.segment (*inliers, *coefficients);

                        Eigen::Vector4f position{coefficients->values[0],coefficients->values[1],coefficients->values[2],0};
                        Eigen::Vector4f direction{ coefficients->values[3],coefficients->values[4],coefficients->values[5],0};
                        Eigen::Vector4f point{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2],0};

                         Eigen::Vector3f pos_first{coefficients->values[0] - (coefficients->values[3] * 5 ),(coefficients->values[1] - coefficients->values[4] * 5) ,(coefficients->values[2] - coefficients->values[5] * 5 )};
                         Eigen::Vector3f position_second{coefficients->values[0] + (coefficients->values[3] * 5 ),(coefficients->values[1]+ coefficients->values[4] * 5) ,(coefficients->values[2]+coefficients->values[5] * 5 )};
                        Eigen::Vector3f pos_center{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2]};

                        Eigen::Vector3f dif = pos_first - position_second;
                        float LineMag = std::sqrt(std::pow(dif.x(),2)+ std::pow(dif.y(),2) +std::pow(dif.z(),2));
                        float U = (((pos_center.x() - pos_first.x()) * (position_second.x() - pos_first.x())) +
                                    ((pos_center.y() - pos_first.y()) * (position_second.y() - pos_first.y())) +
                                    ((pos_center.z() - pos_first.z()) * (position_second.z() - pos_first.z()))) /
                            (LineMag * LineMag);

                        Eigen::Vector3f Intersection = pos_first + (U * (position_second- pos_first));
                        Eigen::Vector3f dif_fo = pos_center - Intersection;
                        float Distance = std::sqrt(std::pow(dif_fo.x(),2)+ std::pow(dif_fo.y(),2) +std::pow(dif_fo.z(),2));


                        // NEW
                        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_acquired_PC(resolution);
                        octree_acquired_PC.setInputCloud (cloud);
                        octree_acquired_PC.addPointsFromInputCloud ();
                        int K = params.numberOfBorderNeighbour;

                         pcl::PointXYZ searchPoint_real{Intersection[0],Intersection[1],Intersection[2]};
                        std::vector<int> pointIdxNKNSearch_real;
                        std::vector<float> pointNKNSquaredDistance_real;
                        if (octree.nearestKSearch (searchPoint_real, K, pointIdxNKNSearch_real, pointNKNSquaredDistance_real) > 0){
                            pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_neigbour_ptr_cloud_real(new pcl::PointCloud<pcl::PointXYZ>);
                            for (size_t i = 0; i < K; i++){
                                pcl::PointXYZ p{(*cloud)[ pointIdxNKNSearch_real[i] ].x,(*cloud)[ pointIdxNKNSearch_real[i] ].y,(*cloud)[ pointIdxNKNSearch_real[i] ].z};
                                nearest_neigbour_ptr_cloud_real->points.emplace_back(p);
                            }
                        


                          // Segment a line
                        pcl::ModelCoefficients::Ptr coefficients_real (new pcl::ModelCoefficients);
                        pcl::PointIndices::Ptr inliers_real (new pcl::PointIndices);
                        // Create the segmentation object
                        pcl::SACSegmentation<pcl::PointXYZ> seg_real;
                        // Optional
                        seg_real.setOptimizeCoefficients (true);
                        // Mandatory
                        seg_real.setModelType (pcl::SACMODEL_LINE);
                        seg_real.setMethodType (pcl::SAC_RANSAC);
                        seg_real.setMaxIterations(100);
                        seg_real.setDistanceThreshold (5);
                        //seg.setEpsAngle(0.01);
                        seg_real.setInputCloud (nearest_neigbour_ptr_cloud_real);
                        seg_real.segment (*inliers_real, *coefficients_real);

                        Eigen::Vector4f position{coefficients_real->values[0],coefficients_real->values[1],coefficients_real->values[2],0};
                        Eigen::Vector4f direction{ coefficients_real->values[3],coefficients_real->values[4],coefficients_real->values[5],0};
                        Eigen::Vector4f point{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2],0};

                         Eigen::Vector3f pos_first_real{coefficients_real->values[0] - (coefficients_real->values[3] * 5 ),(coefficients_real->values[1] - coefficients_real->values[4] * 5) ,(coefficients_real->values[2] - coefficients_real->values[5] * 5 )};
                         Eigen::Vector3f position_second_real{coefficients_real->values[0] + (coefficients_real->values[3] * 5 ),(coefficients_real->values[1]+ coefficients_real->values[4] * 5) ,(coefficients_real->values[2]+coefficients_real->values[5] * 5 )};
                        Eigen::Vector3f pos_center_real{res.at(i).position_aligned[0],res.at(i).position_aligned[1],res.at(i).position_aligned[2]};

                        Eigen::Vector3f dif = pos_first_real - position_second_real;
                        float LineMag = std::sqrt(std::pow(dif.x(),2)+ std::pow(dif.y(),2) +std::pow(dif.z(),2));
                        float U = (((pos_center_real.x() - pos_first_real.x()) * (position_second_real.x() - pos_first_real.x())) +
                                    ((pos_center_real.y() - pos_first_real.y()) * (position_second_real.y() - pos_first_real.y())) +
                                    ((pos_center_real.z() - pos_first_real.z()) * (position_second_real.z() - pos_first_real.z()))) /
                            (LineMag * LineMag);

                        Eigen::Vector3f Intersection_real = pos_first_real + (U * (position_second_real- pos_first_real));
                        Eigen::Vector3f dif_fo_real = pos_center_real - Intersection_real;
                        float Distance_real = std::sqrt(std::pow(dif_fo_real.x(),2)+ std::pow(dif_fo_real.y(),2) +std::pow(dif_fo_real.z(),2));


                        double distance_real  = std::sqrt(pcl::sqrPointToLineDistance(point,position,direction));
                        res.at(i).distanceToEdge = (float)(distance_real / 25.4);
                        res.at(i).position_edge = Intersection_real;

                        if(res.at(i).distanceToEdge >= params.edgeToleranceMin ){
                                res.at(i).withinEdgeTolerance = true;
                        }else{
                                res.at(i).withinEdgeTolerance = false;
                        }
                        std::cout <<  "Edge tolerance is: " <<res.at(i).withinEdgeTolerance <<" with the distance " << res.at(i).distanceToEdge << std::endl;
                    }
                        // NEw
                    } 
                }else{
                    res.at(i).withinEdgeTolerance =false;
                    res.at(i).distanceToEdge = 0;
                    res.at(i).position_edge =res.at(i).position_aligned ;
                    std::cout <<  "Edge tolerance is: " <<res.at(i).withinEdgeTolerance <<" with the distance " << res.at(i).distanceToEdge << std::endl;
                }
            }
        
        }else{
            std::cout <<  "Borders could not be extracted!!" << std::endl;
            for (size_t i = 0; i < res.size(); i++){
                    res.at(i).withinEdgeTolerance =false;
                    res.at(i).position_edge = res.at(i).position_aligned;
                    res.at(i).distanceToEdge = 0;
            }
        }
        

        showEdgeDistances(res,params.cloudPath);

        std::cout << "Edge Distances Calculated!" << std::endl;
        return res;
}
