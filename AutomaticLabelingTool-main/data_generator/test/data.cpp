
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

#include <json/json.h>
#include<json/writer.h>

TEST(data, dataClassTestFunction) {
    //using json = Json::j
    Json::Value j;
    j["imgHeight"] = 1536;
    j["imgWidth"] = 4096;
    j["objects"]["label"] = "road";

    j["objects"]["polygon"][0] = 123;
    std::ofstream o("/home/kirik/Desktop/pretty.json");
    o << std::setw(4) << j << std::endl;
}


/*
TEST(data, dataClassTestFunction) {
    const std::string globPath = "/mrtstorage/users/kirik/images_gray_left_rigt_stereo/2019-08-09-11-29-46/lidar/center/*.pcd";
    std::vector<cv::String> PCDfiles;
    cv::glob(globPath, PCDfiles);
    for (int j = 0; j < PCDfiles.size(); ++j) {
        std::cout << PCDfiles[j] << std::endl;

    }
}
/*
TEST(data, dataClassTestFunction) {
    using namespace Projector;
    Params params;
    params.loadParams();
    Data data(params.poseFile,params.stampFileImage,params.stampFileCloud,params.timeTolerance,params.timeOffset);

    FramedClouds c = data.getFramedClouds();
    TimedPoseList tp = data.getTimedPoseList();
    EulerAngles ea = data.getEulerAnglesFromRotMat(0);
    Pose p = data.getPoseByID(0);
    Eigen::Quaterniond q  = Eigen::Quaterniond(p.rotation());// = data.getQuartenion(0);
    //EulerAngles eaback = data.getEulerAnglesFromQuer(q);

    std::cout<<"p \n" << p.matrix() << std::endl;

    std::cout<<"ea \n" << ea.matrix() << std::endl;

    std::cout<<"q \n" << q.matrix()<<std::endl;

    //std::cout <<"eaback \n" << eaback.matrix() <<std::endl;


}
*/
/*
// Warning : Project point clouds on each image
TEST(data, associatingLidarDataandProjectingOntoImageLongVersionAndVideos) {
    using namespace Projector;
    Params params;
    params.loadParams();
    Data data(params.poseFile,params.stampFileImage,params.stampFileCloud,params.timeTolerance,params.timeOffset);

    StampedFrames images =  data.getStampedFramesImages();
    StampedClouds lidarStamps=  data.getStampedFramesClouds();

    FramedClouds results;
    FramedStampsLidars res;
    for(const auto& image : images){
        FrameId closesFrametoImageStamp;
        Stamp closestStampToImageStamp;
        int64_t diff = std::numeric_limits<int64_t>::max();
        for(const auto& lidarStamp : lidarStamps){
            auto curDiff = signedDiff<int64_t>(lidarStamp.second, image.second);
            if (std::abs(curDiff) < diff) {
                diff = std::abs(curDiff);
                closestStampToImageStamp = lidarStamp.second;
                closesFrametoImageStamp = lidarStamp.first;
            }
        }
        results.emplace_back(std::make_pair(image.first, closesFrametoImageStamp));
        res.emplace_back(std::make_pair(image.first, closestStampToImageStamp));
    }




    CameraInfo cameraInfo(params.cameraCalibPath,params.cameraFrameId,"sensor/ibeo",1.0);

    cloudHandler ch(params.cloudFilePath);
    const std::string globPath1 = params.imageDir + "/*.png";
    std::vector<cv::String> files;
    cv::glob(globPath1, files);

    for (int j = 0; j < files.size(); ++j) {
        Cloud cloud = ch.getPointCloud(results[j].second);
        cv::Mat image = cv::imread(files[j]);

        CloudPosImgs p;
        p = ch.getProjectedPointClouds(cameraInfo,cloud,image);


        ch.projectPointsontoImage(image,p);

        cv::imshow("pe",image);

        cv::waitKey(0);
    }
}
*/
/*
TEST(data, getEulerAngles) {
    using namespace Projector;
    Params params;
    params.loadParams();
    Data data(params.poseFile,params.stampFileImage,params.stampFileCloud,params.timeTolerance,params.timeOffset);

    Pose oguz = data.getPoseByID(0);

    std::cout  << oguz.matrix().row(0).col(0) << std::endl;


    //std::cout  <<  R.col(0).row(0).value()<< std::endl;
    float sy = sqrt(oguz.matrix().row(0).col(0).value() * oguz.matrix().row(0).col(0).value() +  oguz.matrix().row(1).col(0).value() * oguz.matrix().row(1).col(0).value());


    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(oguz.matrix().row(2).col(1).value() , oguz.matrix().row(2).col(2).value());
        y = atan2(-oguz.matrix().row(2).col(0).value(), sy);
        z = atan2(oguz.matrix().row(1).col(0).value(), oguz.matrix().row(0).col(0).value());
    }else
    {
        x = atan2(-oguz.matrix().row(1).col(2).value(),oguz.matrix().row(1).col(1).value());
        y = atan2(-oguz.matrix().row(2).col(0).value(), sy);
        z = 0;
    }

    std::cout << x <<"," <<y<< ","<<"," <<z << std::endl;

}
 */

/*
TEST(data, getQuartenion) {
    using namespace Projector;
    Params params;
    params.loadParams();
    Data data(params.poseFile,params.stampFileImage,params.stampFileCloud,params.timeTolerance,params.timeOffset);
    Pose p = data.getPoseByID(0);
    //Based on pose I get the rounded values
    //std::cout <<"Based on pose " <<p.rotation().eulerAngles(0,1,2) << std::endl;
    EulerAngles ea = data.getEulerAnglesFromRotMat(0);
    Eigen::Quaternionf q;

    q = Eigen::AngleAxisf(ea.x(),Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(ea.y(),Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(ea.z(),Eigen::Vector3f::UnitZ());
}
*/
/*
TEST(data, SLERPInterpolation) {
    using namespace Projector;
    Params params;
    params.loadParams();
    Data data(params.poseFile,params.stampFileImage,params.stampFileCloud,params.timeTolerance,params.timeOffset);

    FramedStampsLidars timeStampedLidarbyID = data.getCloudsFramedStamps();
    StampedFrames sT = data.getStampedFramesImages();

    Stamp tStart = sT[10];
    Stamp tMid = timeStampedLidarbyID[10].second;
    Stamp tEnd = sT[11];


    Pose poseMid;
    Eigen::Quaterniond qMid;

    const Pose poseStart = data.getPoseByID(10);
    const Pose poseEnd = data.getPoseByID(11);

    Eigen::Quaterniond qStart =Eigen::Quaterniond(poseStart.rotation());
    Eigen::Quaterniond qEnd =Eigen::Quaterniond(poseEnd.rotation());

    double a = double(tMid -tStart);
    double b = double(tEnd - tStart);
    double s = a/b;

    Position posStartToMid;
    Eigen::Quaterniond qStartToMid;

    posStartToMid = (1-s) * (poseEnd.translation() - poseStart.translation());
    qStartToMid = qStart.conjugate()* qStart.slerp(s,qEnd);
    qStartToMid.normalize();

    poseMid.translation() = poseStart.translation() + posStartToMid;
    qMid = qStartToMid * qStart;
    qMid.normalize();

    poseMid.linear() = qMid.toRotationMatrix();

}
*/
/*
TEST(data, checkTimeStamps) {
    using namespace Projector;
    Params params;
    params.loadParams();
    Data data(params.poseFile,params.stampFileImage,params.stampFileCloud,params.timeTolerance,params.timeOffset);

    for (int j = 10; j < 7182; ++j) {
        PosedCloud pc = data.getPosesOfCloud(j);
        std::cout <<"time stamp of Image" << pc.first;
        std::cout << "lidar interpolated pose " << pc.second.matrix() << std::endl;
    }

    FramedClouds oguz = data.getFramedClouds()
    for (int j = 0; j < 7182; ++j) {
        std::cout << oguz[j] << std::endl;
    }
}
*/