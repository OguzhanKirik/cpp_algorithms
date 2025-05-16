#pragma once

// Own classes
#include "data_type.h"
#include "camera_info.h"

// MRT libraries
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// External libraries
#include <filesystem>



namespace data_generation {


    class cloud_handler {
    public:
    /**
    * @brief this contructor loads create an object in cloud_handler class
    * it is used to check for occlusions in region of interest as well as in the whole field of view.
     * The complete implementation of the class is not done!!!!
    */
        cloud_handler();

        ROI checkForOcclusionROI(const ROI &roiValues,
                                                const CloudPosImgs& cloudPosImgs,
                                                const float& groundTruthDistance) const;
        CloudPosImgs getProjectedPointClouds(const Camera& cameraLidar,
                                             const pcl::PointCloud<pcl::PointXYZ>& pointCloud,
                                             const int16_t& col_size,
                                             const int16_t& row_size)const;

    private:
        ROI calculateRoiValues(const ROI& roiValues, const CloudPosImgs& cloudPosImgs)const;



        ROI getOccludedROI() const;

        bool occlusionCheckForHomogeneousROI(const float &roiMean, const float &groundTruthDistance) const;

        CloudPosImgs getPointsInROI(const ROI& roiValues, const CloudPosImgs& cloudPosImgs)const;

        float calculateStandardDev(const float &meanRoi,const Positions& positions)const;

        float calculateMeanDistance(const CloudPosImgs& cloudPosImgs)const;

        float getEuclideanDistance(const Position& pos)const;

    };
}

