
#include "camera_info.h"


namespace data_generation {

    //camera_info::camera_info(){};

    camera_info::camera_info(const Params& params){

        camera_= getCameraCalibration(params.cameraCalibFile,params.cameraName,params.cameraRefVehicle,1.0);

        if(params.occlusionHandlings.stereoVision == "true"){
            camStereo = getCameraCalibration(params.cameraCalibFile,params.cameraName,params.cameraRefBaseLine,1.0);
            baseLine = calculateBaseline();
        }
        if(params.occlusionHandlings.lidar == "true"){
            camLidar = getCameraCalibration(params.cameraCalibFile,params.cameraName,params.cameraRefLidar,1.0);
        }
    };


    Position camera_info::calculateBaseline()const{
        Position baseLine;
        baseLine = (camStereo.cameraPose.translation() * (-1));
        return baseLine;
    }


    Camera camera_info::getCameraCalibration(const std::string& calibFileName,
                                             const std::string& cameraFrameName,
                                             const std::string& refFrame,
                                             const double& scale)const {
        cs::CalibStorage calibStorage(calibFileName);
        //calibStorage.showCameraNames();

        auto cam = calibStorage.getCamera(cameraFrameName, refFrame);


        cam.cameraPose.translation() *= scale;
        if (cam.cameraModel.get() == nullptr)
            throw std::runtime_error("Camera model " + cameraFrameName + " from file " + calibFileName +
                                     " not setted (nullptr)");

        return cam;
    }




    double camera_info::getAngleOfViewVertical()const {
        int imgW,imgH;
        double focalLength = camera_.cameraModel->getFocalLength();
        camera_.cameraModel->getImageSize(imgW,imgH);
        return 2*atan(imgW/(2*focalLength));
    }

    double camera_info::getAngleOfViewHorizantal()const {
      int imgW,imgH;
      double focalLength = camera_.cameraModel->getFocalLength();
      camera_.cameraModel->getImageSize(imgW,imgH);
      return 2*atan(imgH/(2*focalLength));
    }



}
