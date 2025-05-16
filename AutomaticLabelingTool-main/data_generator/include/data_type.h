#pragma once


// STL Libraries
#include <map>
#include <memory> //unique_ptr
#include <set>
#include <vector>
#include <Eigen/Dense>
#include <fstream>


// MRT Libraries
#include <locmap_io/load_poses.h>
#include <gnss_coordinate_transform/local_geographic_cs.hpp>
#include <generic_logger/generic_logger.hpp>
#include <boost/optional.hpp>
#include <transform3d/transform3d.h>
#include <boost/filesystem.hpp>// to checkt file paths are true
#include <opencv2/core/hal/interface.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>




namespace data_generation{


    struct LabelingType{
        std::string pixelWise;
        std::string boundingBox;
    };

    struct Elements{
        std::string trafficLight;
        std::string trafficSign;
        std::string roadMarking;
        std::string dashedLines;
        std::string solidLines;
        std::string roads;
        std::string terrains;
        std::string walkways;
        std::string otherLines;
        std::string dynamicClass;
    };

    struct OcclusionHandlings{
        std::string binaryMask;
        std::string stereoVision;
        std::string lidar;
    };
    struct Params {
        LabelingType labelingType;
        Elements element;
        OcclusionHandlings occlusionHandlings;

        std::string laneletMap;
        std::string cameraCalibFile;
        std::string poseFile;
        std::string binaryMaskDir;
        std::string maskStaticDir;
        std::string imageDir;
        std::string imageStampFile;
        std::string lidarStampFile;
        std::string disparityDir;
        std::string cloudDir;
        std::string outputDir;


        std::string cameraName;
        std::string cameraRefVehicle;
        std::string cameraRefBaseLine;
        std::string cameraRefLidar;


        double timeTolerance;
        double timeOffset;
        float camScale;
        bool project;
        double baseLat;
        double baseLon;
        uint64_t startId;
        double x_axis;
        double y_axis;
        float rate_expand;
        float rate_shrink;
        double orientedBox_x;
        double orientedBox_z;

        int16_t circleRadiusFlying;
        int16_t circleRadiusNotFlying;
        int16_t marginDisp;


        int16_t cluster_size;
        int16_t cluster_distance;

        std::string configPath;
        void loadParams() {
            std::ifstream configFile(configPath); // Update the path!!!
            if (configFile.is_open()) {
                std::string line;
                while (getline(configFile, line)) {
                    if(line[0] == '#' || line.empty())
                        continue;
                    std::istringstream sin(line.substr(line.find("=") + 1));
                    if (line.find("imageStampFile") != -1)
                        sin >> imageStampFile;
                    else if (line.find("cameraRefBaseLine") != -1)
                        sin >> cameraRefBaseLine;
                    else if (line.find("cameraRefVehicle") != -1)
                        sin >> cameraRefVehicle;
                    else if (line.find("cameraRefLidar") != -1)
                        sin >> cameraRefLidar;
                    else if (line.find("camScale") != -1)
                        sin >> camScale;
                    else if (line.find("rate_expand") != -1)
                        sin >> rate_expand;
                    else if (line.find("cluster_distance") != -1)
                        sin >> cluster_distance;
                    else if (line.find("cluster_size") != -1)
                        sin >> cluster_size;
                    else if (line.find("rate_shrink") != -1)
                        sin >> rate_shrink;
                    else if (line.find("circleRadiusFlying") != -1)
                        sin >> circleRadiusFlying;
                    else if (line.find("circleRadiusNotFlying") != -1)
                        sin >> circleRadiusNotFlying;
                    else if (line.find("binaryMaskDir") != -1)
                        sin >> binaryMaskDir;
                    else if (line.find("maskStaticDir") != -1)
                        sin >> maskStaticDir;
                    else if (line.find("marginDisp") != -1)
                        sin >> marginDisp;
                    else if (line.find("poseFile") != -1)
                        sin >> poseFile;
                    else if (line.find("lidarStampFile") != -1)
                        sin >> lidarStampFile;
                    else if (line.find("imageDir") != -1)
                        sin >> imageDir;
                    else if (line.find("disparityDir") != -1)
                        sin >> disparityDir;
                    else if (line.find("outputDir") != -1)
                        sin >> outputDir;
                    else if (line.find("laneletMap") != -1)
                        sin >> laneletMap;
                    else if (line.find("cameraCalibFile") != -1)
                        sin >> cameraCalibFile;
                    else if (line.find("cameraName") != -1)
                        sin >> cameraName;
                    else if (line.find("cloudDir") != -1)
                        sin >> cloudDir;
                    else if (line.find("timeOffset") != -1)
                        sin >> timeOffset;
                    else if (line.find("timeTolerance") != -1)
                        sin >> timeTolerance;
                    else if (line.find("baseLat") != -1)
                        sin >> baseLat;
                    else if (line.find("baseLon") != -1)
                        sin >> baseLon;
                    else if (line.find("x_axis") != -1)
                        sin >> x_axis;
                    else if (line.find("y_axis") != -1)
                        sin >> y_axis;
                    else if (line.find("orientedBox_x") != -1)
                        sin >> orientedBox_x;
                    else if (line.find("orientedBox_z") != -1)
                        sin >> orientedBox_z;
                    else if (line.find("startId") != -1)
                        sin >> startId;
                    else if (line.find("trafficLight") != -1)
                        sin >> element.trafficLight;
                    else if (line.find("trafficSign") != -1)
                        sin >> element.trafficSign;
                    else if (line.find("roadMarking") != -1)
                        sin >> element.roadMarking;
                    else if (line.find("dashedLines") != -1)
                        sin >> element.dashedLines;
                    else if (line.find("solidLines") != -1)
                        sin >> element.solidLines;
                    else if (line.find("roads") != -1)
                        sin >> element.roads;
                    else if (line.find("walkways") != -1)
                        sin >> element.walkways;
                    else if (line.find("terrains") != -1)
                        sin >> element.terrains;
                    else if (line.find("otherLines") != -1)
                        sin >> element.otherLines;
                    else if (line.find("dynamicClass") != -1)
                        sin >> element.dynamicClass;
                    else if (line.find("binaryMask") != -1)
                        sin >> occlusionHandlings.binaryMask;
                    else if (line.find("stereoVision") != -1)
                        sin >> occlusionHandlings.stereoVision;
                    else if (line.find("lidar") != -1)
                        sin >> occlusionHandlings.lidar;
                    else if (line.find("pixelWise") != -1)
                        sin >> labelingType.pixelWise;
                    else if (line.find("boundingBox") != -1)
                        sin >> labelingType.boundingBox;

                }
            }
        }
    };





    using Pose = Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign>;
    using Poses = std::vector<Pose>;
    //using Pose2D = Eigen::Transform<double, 2, Eigen::Isometry, Eigen::DontAlign>;
    //using Poses2D = std::vector<Pose2D>;

    using EulerAngles = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;

    using Area = Eigen::Matrix<double,2,1,Eigen::DontAlign>;

    using Pose2d = Eigen::Matrix<double,2,1,Eigen::DontAlign>;
    using Poses2d = std::vector<Pose2d>;
    using Stamp = long double;

    using imgCoord = int16_t;
    using imgCoords= std::vector<imgCoord>;

    using Disparity = int64_t;
    using Disparities = std::vector<int64_t>;

    struct DisparityRange{
        int64_t minDisp;
        int64_t medDisp;
        int64_t maxDisp;
    };

    using Histogram = std::vector<std::pair<int16_t, Disparity>> ;
    using HistogramClusters = std::vector<Histogram>;

    using realDepth = float;
    using realDepths= std::vector<realDepth>;

    using Direction = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
    using Position = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
    using Positions = std::vector<Position>;


    using TimedPose = std::pair<Stamp, Pose>;
    using TimedPoseList = std::vector<TimedPose>;

    using Point = cv::Point;
    using Points = std::vector<Point>;
    using Countours = std::vector<Points>;

    using LeftRight = std::pair<double, double>;

    using LeftRights = std::vector<LeftRight>;

    using pointCloudPoints = std::vector<cv::Point>;

    using FrameId = uint64_t;
    using FrameIds = std::vector<uint64_t>;

    using StampedFrames =  std::map<FrameId, Stamp>;
    using ImageCoordinate = Eigen::Matrix<int, 2, 1, Eigen::DontAlign>;
    using ImageCoordinateFloat = Eigen::Matrix<double, 2, 1, Eigen::DontAlign>;
    using ImageCoordinateFloats = std::vector<ImageCoordinateFloat>;

    using FramedPoses = std::map<FrameId, Pose>;
    enum class StampFileType { CSV, RAW };


    // Point Cloud  Data Types
    using Cloud = pcl::PointCloud<pcl::PointXYZ>;

    using CloudPosImg = std::pair<Position, ImageCoordinateFloat>;
    using CloudPosImgs = std::vector<CloudPosImg>;

    using StampedClouds =  std::map<FrameId, Stamp>; // frameId: From 0 to .. Stamp:. from file

    //using PosedCloud =std::pair<FrameId, Pose>;
    //using PosedClouds = std::vector<PosedCloud>;

    //using FramedStampsLidar = std::pair<FrameId, Stamp>; // frameId : ImageFrameId   stamp: and associated Lidar Stamp
    //using FramedStampsLidars = std::vector<FramedStampsLidar>;

    using FramedClouds = std::map<FrameId,FrameId>;  // image frame and associated lidar Frame
    //using FramedClouds = std::vector<FramedCloud>;



    struct ROI{
        int16_t greatest_x = std::numeric_limits<int16_t>::min();
        int16_t greatest_y = std::numeric_limits<int16_t>::min();
        int16_t smallest_x = std::numeric_limits<int16_t>::max();
        int16_t smallest_y = std::numeric_limits<int16_t>::max();

        int16_t maxDisparity =std::numeric_limits<int16_t>::min();
        int16_t minDisparity = std::numeric_limits<int16_t>::max();

        int16_t width = greatest_x - smallest_x;
        int16_t height = greatest_y - smallest_y;
        float area = width * height;
        Histogram histogram;
        bool partialOcclusion = false;
        bool nonOcclusion = false;
        Countours countours ;
        Points notOccludedPixels;
        int16_t num_objects = 0;
        float  mean; // stereo
        float stddev; // stereo
        int16_t stddevThreshold = 3; // stereo
        float meanDistance; // Lidar
    };
    using ROIClusters = std::vector<ROI>;

    class Light {
    public:
        Position t1;
        Position t2;
        Position t3;
        Position t4;
        Points imageCoordinates;
        ROI roi;
        Light (const std::vector<Eigen::Vector3d>& positions) {
        //    if (positions.size() != 9)
        //        throw std::runtime_error("Can't deal with traffic lights of size != 9");
            t1 = positions.at(0);
            t2 = positions.at(1);
            t3 = positions.at(2);
            t4 = positions.at(3);

        }
        Light(const Position &p1, const Position &p2, const Position &p3, const Position &p4) {
            t1 = p1;
            t2 = p2;
            t3 = p3;
            t4 = p4;

        }
        Position centerOfFour() const {
            return (t1 + t4) / 2;
        }
        Positions asList() const {
            return {t1, t2, t3, t4};
        }
        void setRoiCoordAndCountours(){
            for (auto& p : imageCoordinates) {
                getSmallestandGreatestX(roi,p.x);
                getSmallestandGreatestY(roi,p.y);
            }
            roi.countours.emplace_back(imageCoordinates);

        }

        void getSmallestandGreatestX(ROI& roi,const int16_t& x){
            if (x < roi.smallest_x) {
                roi.smallest_x = x;
            }
            if (x > roi.greatest_x) {
                roi.greatest_x = x;
            }
        }
        void getSmallestandGreatestY(ROI& roi,const int16_t& y){
            if (y < roi.smallest_y) {
                roi.smallest_y = y;
            }
            if (y > roi.greatest_y) {
                roi.greatest_y = y;
            }
        }


    };

    using Lights = std::vector<Light>;

    class Sign{
    public:
        Position t1;
        Position t2;
        Position t3;
        Position t4;
        Points imageCoordinates;
        ROI roi;
        Sign (const std::vector<Eigen::Vector3d>& positions) {
            //    if (positions.size() != 4)
            //        throw std::runtime_error("Can't deal with traffic lights of size != 9");
            t1 = positions.at(0);
            t2 = positions.at(1);
            t3 = positions.at(2);
            t4 = positions.at(3);
        }
        Sign (const Position &p1, const Position &p2, const Position &p3, const Position &p4) {
            t1 = p1;
            t2 = p2;
            t3 = p3;
            t4 = p4;
        }
        Position centerOfFour() const {
            return (t1 + t4) / 2;
        }
        Positions asList() const {
            return {t1, t2, t3, t4};
        }


        void setRoiCoordAndCountours(){
            for (auto& p : imageCoordinates) {
                getSmallestandGreatestX(roi,p.x);
                getSmallestandGreatestY(roi,p.y);
                roi.countours.at(0).emplace_back(p);
            }
            roi.countours.emplace_back(imageCoordinates);

        }

        void getSmallestandGreatestX(ROI& roi,const int16_t& x){
            if (x < roi.smallest_x) {
                roi.smallest_x = x;
            }
            if (x > roi.greatest_x) {
                roi.greatest_x = x;
            }
        }
        void getSmallestandGreatestY(ROI& roi,const int16_t& y) {
            if (y < roi.smallest_y) {
                roi.smallest_y = y;
            }
            if (y > roi.greatest_y) {
                roi.greatest_y = y;
            }
        }
    };

    using Signs = std::vector<Sign>;


    class DashedLine{
    public:
        Position t1;
        Position t2;
        Position t3;
        Position t4;
        Position t5;
        ROI roi;
        Points imageCoordinates;
        Positions posInBoundixBox;

        DashedLine (const std::vector<Eigen::Vector3d>& positions) {
            //    if (positions.size() != 4)
            //        throw std::runtime_error("Can't deal with traffic lights of size != 9");
            t1 = positions.at(0);
            t2 = positions.at(1);
            t3 = positions.at(2);
            t4 = positions.at(3);
            t5 = positions.at(4);

        }
        Positions getPositions() const {
            return {t1, t2, t3, t4, t5};
        }
        // deneme yap!!!!!
        Position centerOfFour() const {
            return (t1 + t5) / 2;
        }
    };

    using DashedLines = std::vector<DashedLine>;

    class SolidLine {
    public:
        Positions p;
        Points imageCoordinates;
        Positions posInBoundixBox;
        SolidLine (const std::vector<Eigen::Vector3d>& positions) {
            for (int j = 0; j < positions.size(); ++j) {
                p.emplace_back(positions.at(j));
            }
        }
        Positions getPositions()const{
            return p;
        }

    };
    using SolidLines = std::vector<SolidLine>;

    class OtherLine {
    public:
        Positions p;
        Points imageCoordinates;
        Positions posInBoundixBox;
        OtherLine (const std::vector<Eigen::Vector3d>& positions) {
            for (int j = 0; j < positions.size(); ++j) {
                p.emplace_back(positions.at(j));
            }
        }
        Positions getPositions()const{
            return p;
        }

    };

    using OtherLines = std::vector<OtherLine>;

    class Road {
    public:
        Positions p;
        Points imageCoordinates;
        Positions posInBoundixBox;
        Road (const std::vector<Eigen::Vector3d>& positions) {
            for (int j = 0; j < positions.size(); ++j) {
                p.emplace_back(positions.at(j));
            }
        }
        Positions getPositions()const{
            return p;
        }
    };

    using Roads = std::vector<Road>;


    class Terrain {
    public:
        Positions p;
        Points imageCoordinates;
        Positions posInBoundixBox;
        Terrain (const std::vector<Eigen::Vector3d>& positions) {
            for (int j = 0; j < positions.size(); ++j) {
                p.emplace_back(positions.at(j));
            }
        }
        Positions getPositions()const{
            return p;
        }
    };

    using Terrains = std::vector<Terrain>;


    class Walkway {
    public:
        Positions p;
        Points imageCoordinates;
        Positions posInBoundixBox;
        Walkway (const std::vector<Eigen::Vector3d>& positions) {
            for (int j = 0; j < positions.size(); ++j) {
                p.emplace_back(positions.at(j));
            }
        }
        Positions getPositions()const{
            return p;
        }
    };

    using Walkways = std::vector<Walkway>;

    struct MapElements{
        Lights lights;
        Signs signs;
        DashedLines dashedLines;
        SolidLines solidLines;
        OtherLines otherLines;
        Roads roads;
        Terrains terrains;
        Walkways walkways;
        //Markings markings
    };





    template<typename SignedType>
    inline SignedType signedDiff(const uint64_t &t1, const uint64_t &t2) {
        return (t1 > t2) ? static_cast<SignedType>(t1 - t2)
                         : static_cast<SignedType>(-1) * static_cast<SignedType>(t2 - t1);
    }









}




