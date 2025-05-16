
// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
#include <iostream>
#include <stdlib.h>
#include <ctime>


// my lib
#include "gtest/gtest.h"
#include "disparity_handler.h"
#include "cloud_handler.h"
#include "data_type.h"
#include "data_generator.h"
#include "map_handler.h"
#include "camera_info.h"
#include "data_prep.h"


#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


//External lib
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/*
TEST(pointCloud, LidarBinaryMask) {
    using namespace data_generation;
    Params params;
    params.loadParams();



    //params.startIdProj =1;
    camera_info camera(params);
    data_prep data(params);
    cloud_handler cl;
    cv::Mat image = data.getImage(1000);
    Cloud cloud = data.getCloudByID(1000);

    cv::Mat disp = data.getDisparityMap(5);
        int16_t c = data.getImage(1).cols;
        int16_t r = data.getImage(1).rows;

        CloudPosImgs cloudInImages = cl.getProjectedPointClouds(camera.getCameraLidar(), cloud, c, r);
        CloudPosImgs cloudObstacle;
        std::vector<double> clusters;
        ImageCoordinateFloats res;
        ImageCoordinateFloats res2;

        for (auto cld :cloudInImages) {
            if (cld.second.y() < 1200 && cld.second.y() > 1150) {
                float distance = sqrt(pow(cld.first.z(), 2) + pow(cld.first.y(), 2) + pow(cld.first.x(), 2));
                if (distance < 30) {
                    cloudObstacle.emplace_back(cld);
                    res.emplace_back(cld.second);
                    clusters.emplace_back(cld.second.x());
                }
            }
        }


        std::vector<std::vector<double>> seperated_clusters;
        std::sort(clusters.begin(), clusters.end());


        int num_cluster = 0;
        double first_point = clusters.at(0);
        std::vector<double> first_cluster;
        first_cluster.emplace_back(first_point);
        seperated_clusters.emplace_back(first_cluster);

        for (int j = 0; j < (clusters.size() - 1); ++j) {
            double a = clusters.at(j);
            double b = clusters.at(j + 1);
            if (std::abs(a - b) < 30) {
                seperated_clusters.at(num_cluster).emplace_back(b);
            } else {
                ++num_cluster;
                std::vector<double> other_clusters;
                other_clusters.emplace_back(b);
                seperated_clusters.emplace_back(other_clusters);
            }
        }


        LeftRights result;
        for (int l = 0; l < seperated_clusters.size(); ++l) {
            std::vector<double> clustersss = seperated_clusters.at(l);
            double leftest = clustersss.front();
            double rightest = clustersss.back();

            result.emplace_back(std::make_pair(leftest, rightest));
        }

        std::vector<float> distance_clusters;
        for (auto r : result) {
            std::vector<float> distance_cl;
            for (auto cld :cloudObstacle) {
                if (cld.second.y() < 1200 && cld.second.y() > 1150 &&
                    cld.second.x() > r.first && cld.second.x() < r.second) {
                    float distance = sqrt(pow(cld.first.z(), 2) + pow(cld.first.y(), 2) + pow(cld.first.x(), 2));
                    if (distance < 30) {
                        distance_cl.emplace_back(distance);
                    }
                }
            }
            std::sort(distance_cl.begin(), distance_cl.end());
            float first = distance_cl.front();
            float last = distance_cl.back();
            float average = (first + last) / 2;
            std::cout << average << std::endl;
            distance_clusters.emplace_back(average);

        }


        for (int m = 0; m < result.size(); ++m) {
            for (auto cl :cloudInImages) {
                if (cl.second.x() > result.at(m).first && cl.second.x() < result.at(m).second) {
                    float dist = sqrt(pow(cl.first.z(), 2) + pow(cl.first.y(), 2) + pow(cl.first.x(), 2));
                    if (dist < (distance_clusters.at(m) + 0.5) && dist > (distance_clusters.at(m) - 0.5)) {
                        res2.emplace_back(cl.second);
                    }
                }

            }
        }


        for (int k = 0; k < res2.size(); ++k) {
            cv::Point p(res2[k].x(), res2[k].y());
            cv::circle(image, p, 3, cv::Scalar(250, 120, 100), 3, 8);
        }
        /*
       for (int k = 0; k < res2.size(); ++k) {
           cv::Point p(res2.at(k).x,res2.at(k).y);
           cv::circle(image, p, 3, cv::Scalar(250, 120, 100), 3, 8);
       }


        if (image.rows > 1000 || image.cols > 1500)
            cv::resize(image, image, cv::Size(image.cols * 0.35, image.rows * 0.35), 0, 0);
        cv::imshow("pencere", image);

        //if (disp.rows > 1000 || disp.cols > 1500)
        //    cv::resize(disp, disp, cv::Size(disp.cols * 0.35, disp.rows * 0.35), 0, 0);
        //cv::imshow("disp",disp);
        //cv::imwrite( "/home/kirik/Desktop/projPCDsSample/imagePointCloud.png ", image );
        cv::waitKey(0);
    }


*/





/*d
TEST(pointCloud, dumbTimeStampsOfClouds) {
    using namespace data_generation;


    std::vector<std::string> seconds;
    std::vector<std::string> nseconds;

    std::ifstream File("/mrtstorage/users/kirik/data/rosbag_29_46/timestampraw.txt");
    if (File.is_open()) {
        std::string line;
        while (getline(File, line)) {
            if(line[0] == '-' || line.empty())
                continue;
            std::string secs;
            if(line[0] == 's'){
                std::istringstream sin(line.substr(line.find(":") + 2));
                secs =sin.str();
                seconds.emplace_back(secs);

            }
            if(line[0] == 'n'){
                std::istringstream sin(line.substr(line.find(":") + 2));
                std::string nsecs;
                sin >> nsecs;
                nseconds.emplace_back(nsecs);
            }
        }

    }
    std::ofstream imageTimeStamps("/mrtstorage/users/kirik/data/rosbag_29_46/imageTimeStampReal.txt");
    for (int j = 0; j < seconds.size(); ++j) {
        std::string timeStamp = seconds.at(j) + nseconds.at(j);
        std::cout << timeStamp.size() << std::endl;
        if(timeStamp.size() == 19){
            imageTimeStamps << timeStamp << std::endl;
        }else{
            std::string timeS = seconds.at(j) + "0" + nseconds.at(j);
            std::cout << timeS << std::endl;
            imageTimeStamps << timeS << std::endl;

        }
    }




}
 */
/*
TEST(pointCloud, timeStampsforTlMapper) {
    using namespace data_generation;
    Params params;
    params.loadParams();


    std::ofstream myfile;
    myfile.open ("/mrtstorage/users/kirik/lex_map_ost/Stamps.txt");

    std::ifstream timeStamps("/mrtstorage/users/kirik/lex_map_ost/timestamp.stereo_front_right_rect.txt");
    int counter =0;
    if (timeStamps.is_open()) {
        std::string line;
        while (getline(timeStamps, line)) {
            std::string name,name2;
            std::stringstream ss;
            std::istringstream sin(line.substr(0,10));
            sin >> name;
            std::istringstream sin2(line.substr(10));
            sin2 >> name2;


            ss << name << "," << name2 << ","<< counter << "\n";
            std::string s = ss.str();
            std::cout <<s << std::endl;
            myfile <<s;
            counter ++;
        }


    }
    myfile.close();



    for (int l = 0; l < 6351; ++l) {
        //std::cout << cloudPCDFIles.at(l) << std::endl;
        std::string path = cloudPCDFIles.at(l);
        std::size_t found = path.find_last_of("/");
        std::string name = path.substr(1 + found);
        std::size_t foundDot = name.find_last_of(".");
        std::string timeStamp = name.substr(0, foundDot);
        lidarTimeStamps << timeStamp << std::endl;
    }

}
*/
/*
TEST(pointCloud, loadPCDFile) {
    using namespace data_generation;
    Params params;
    params.loadParams();

    cloud_handler cloudHandler(params.cloudFilePath);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud = cloudHandler.getPointCloud(0);



    std::cout << "Loaded "
              << cloud.width * cloud.height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;
    for (size_t i = 0; i < cloud.points.size (); ++i)
        std::cout << "    " << cloud.points[i].x
                  << " "    << cloud.points[i].y
                  << " "    << cloud.points[i].z << std::endl;



}

*/
/*
TEST(pointCloud, associateLidarAndImage) {
    using namespace Projector;
    Params params;
    params.loadParams();

    Data data(params.poseFile,params.stampFileImage,params.timeTolerance, params.timeOffset);

    TimedPoseList vehiclePosesWithTime = data.getTimedPoseList();

    std::cout << vehiclePosesWithTime.at(0).first << std::endl;
}
 */

/*
TEST(pointCloud, projectPointsontoImagefromlidarFrameto) {
    using namespace data_generation;
    Params params;
    params.loadParams();



    params.startIdProj =1;
    camera_info camera(params);
    data_prep data(params);


    for (int l = 0; l < 6000; ++l) {


    // load a pcd file
    Cloud cloud;
    cloud = data.getCloudByID(l);

    // load an image
    const std::string globPath1 = params.imageDir + "/*.png";
    std::vector<cv::String> files;
    cv::glob(globPath1, files);
    cv::Mat image = cv::imread(files[l]);

    // load camera wrt lidar
    cs::CalibStorage calibStorage(params.cameraCalibFile);
    //calibStorage.showCameraNames();
    //calibStorage.getTransformationGraph().showNodes();
    auto camL = calibStorage.getCamera(params.cameraName, params.cameraRefLidar);
    camL.cameraPose.translation() *= 1.0;
    if (camL.cameraModel.get() == nullptr)
        throw std::runtime_error("Camera model " + params.cameraName + " from file " + params.cameraCalibFile +
                                 " not setted (nullptr)");




    Positions pointCloudImage;
    std::cout << "cloud size "<< cloud.points.size() << std::endl;

    for (size_t i = 0; i < cloud.points.size() ; ++i){
        Position cloudPointPos, cloudPointTransform;
        cloudPointPos= {cloud.points[i].x,cloud.points[i].y,cloud.points[i].z};
        cloudPointTransform = camL.cameraPose.inverse() * cloudPointPos;
        pointCloudImage.emplace_back(cloudPointTransform);
    }
    ImageCoordinateFloats projectedPoints;

    for (int j = 0; j < pointCloudImage.size(); ++j) {
        ImageCoordinateFloat imgCoordinates(-1, -1);

        camL.cameraModel->getImagePoint(pointCloudImage.at(j),imgCoordinates);
        if(imgCoordinates.x() >0 && imgCoordinates.x() <= image.cols &&
        imgCoordinates.y() > 0 && imgCoordinates.y() <= image.rows){
            //std::cout <<" point matrix\n"<<pointCloudImage.at(j).matrix() << std::endl;
            projectedPoints.emplace_back(imgCoordinates);
        }
    }




    std::cout << "projectedPoints size "<< projectedPoints.size() << std::endl;
    for (int k = 0; k < projectedPoints.size(); ++k) {
        cv::Point p(projectedPoints[k].x(),projectedPoints[k].y());
        cv::circle(image, p, 3, cv::Scalar(110, 220, 0), 2, 8);
    }

    if (image.rows > 1000 || image.cols > 1500)
        cv::resize(image, image, cv::Size(image.cols * 0.35, image.rows * 0.35), 0, 0);
    cv::imshow("pencere",image);
    cv::imwrite( "/home/kirik/Desktop/projPCDsSample/imagePointCloud.png ", image );
    cv::waitKey(0);
    }


}
*/
/*
TEST(pointCloud, projectPointsOntoImageFromCameraFrame) {
    using namespace data_generation;
    Params params;
    params.loadParams();

    camera_info camera(params.cameraCalibFile, params.cameraName, "sensor/lidar/velodyne/c", 1.0);
    cloud_handler ch(params.cloudDir);
    pcl::PointCloud<pcl::PointXYZ> cloud = ch.getPointCloud(0);

    const std::string globPath1 = params.imageDir + "/*.png";
    std::vector<cv::String> files;
    cv::glob(globPath1, files);
    cv::Mat image = cv::imread(files[4472]);

    CloudPosImgs p;
    p = ch.getProjectedPointClouds(camera, cloud, image);
    ch.projectPointsontoImage(image, p);

    cv::imshow("pe", image);
    cv::imwrite("/home/kirik/Desktop/imagePointCloud.png ", image);
    cv::waitKey(0);

}
*/





TEST(pointCloud, TypeofSensortForTransformation) {
    using namespace data_generation;
    Params params;
    params.loadParams();

    //params.cloudDir ="/mrtstorage/users/kirik/lex_map_02/lidar_new/velodyne/center";
    //params.cameraRefLidar
    //params.cameraCalibFile ="/mrtstorage/users/kirik/lex_map_02/calib/calibration.bin";
    //params.cameraName ="stereo_front_left_rect";
    //params.imageDir ="/mrtstorage/users/kirik/lex_map_02/gray_left";



    cs::CalibStorage calibStorage(params.cameraCalibFile);
    //calibStorage.showCameraNames();


    //calibStorage.showCameraNames();
    //auto cam = calibStorage.getCamera(params.cameraFrameId, "sensor/ibeo",1.0);
    //std::cout << cam.cameraPose.matrix() << std::endl;
    calibStorage.getTransformationGraph().showNodes();
}



/*
TEST(pointCloud, writePointCloudInCameraFrameLongVersion) {
    using namespace Projector;
    Params params;
    params.loadParams();
    std::vector<cv::String> cloudPCDFIles;

    const std::string globCloudPath = params.cloudFilePath + "/*.pcd";
    cv::glob(globCloudPath, cloudPCDFIles);
    CameraInfo cameraInfo(params.cameraCalibPath,params.cameraFrameId,"sensor/ibeo",1.0);


    for (int j = 0; j < cloudPCDFIles.size(); ++j) {

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPCDFIles[j], cloud);

        for (size_t i = 0; i < cloud.points.size (); ++i){
            Position cloudPointPos, cloudPointTransform;
            cloudPointPos= {cloud.points[i].x,cloud.points[i].y,cloud.points[i].z};
            cloudPointTransform = cameraInfo.getCamera().cameraPose.inverse() * cloudPointPos;
            cloud.points[i].x = cloudPointTransform.x();
            cloud.points[i].y = cloudPointTransform.y();
            cloud.points[i].z = cloudPointTransform.z();
        }
        std::string  pcdFile = std::filesystem::path(cloudPCDFIles[j]).filename();
        std::string path = "/home/kirik/Desktop/pointCloudCameraFrame/" + pcdFile;
        pcl::io::savePCDFileASCII(path, cloud);
    }
*/

/*
TEST(pointCloud, writePointCloudInCameraFrameSHORTVersion) {
    using namespace Projector;
    Params params;
    params.loadParams();

    CameraInfo cameraInfo(params.cameraCalibPath,params.cameraFrameId,"sensor/ibeo",1.0);
    cloudHandler cloudHandler(params.cloudFilePath);

    std::string path = "/home/kirik/Desktop/samplePointCloudCamera/";
    cloudHandler.writePointCloudinCameraFrame(cameraInfo,path);


}

*/
