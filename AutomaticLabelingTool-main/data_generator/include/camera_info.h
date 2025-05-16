#pragma once


// Own classes
#include "data_type.h"
#include "data_prep.h"

// MRT libraries
#include <calib_storage/calib_storage.h>
#include <camera_models/camera.h>




namespace data_generation {

    class camera_info {
    public:

        camera_info();
    /**
     * @brief Depending of the mode of the tool, this contructor load 3 different cameras
     * camera reference to the vehicle
     * camera reference to the pair camera
     * camera reference to lidar sensor
     * @param params.cameraCalibFile path to calibration file
     * @param params.cameraName camera name
     * @param params.cameraRefVehicle camera reference to vehicle
     * @param params.cameraRefLidar camera reference to lidar
     * @param params.cameraRefBaseLine camera reference to  camera pair, stereo vision
     * @param params.occlusionHandlings.stereoVision if true, baseline is calculated
     * @param params.occlusionHandlings.lidar if true, cameraLidar is prepared
     * @return
     */

        camera_info(const Params& params);

    /**
     * @brief Returns camera object, reference Frame Vehicle
    */
        const Camera &getCamera() const {
            return camera_;
        }

    /**
    * @brief Returns camera object, reference Frame Lidar
    */
        const Camera &getCameraLidar() const {
            return camLidar;
        }

    /**
    * @brief Returns the baseline between camera pairs
    */
        const Position& getBaseLine()const{
            return baseLine;
        }

    /**
    * @brief calculates the baseline
    */
        Position calculateBaseline()const;

     /**
    * @brief load corresponding camera and returns it
    */
        Camera getCameraCalibration(const std::string &calibFileName,
                                    const std::string &cameraFrameName,
                                    const std::string &refFrame,
                                    const double &scale)const;

        /**
       * @brief returns vertical view angle
       */
       double getAngleOfViewVertical()const;
        /**
       * @brief returns horizontal view angle
       */
        double getAngleOfViewHorizantal()const;


    private:
        Camera camera_;
        Camera camStereo;
        Camera camLidar;
        Position baseLine;

    };

}
