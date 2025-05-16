
#include "cloud_handler.h"

namespace data_generation {

    cloud_handler::cloud_handler(){};

    ROI cloud_handler::checkForOcclusionROI(const ROI &roiValues,
                                        const CloudPosImgs& cloudPosImgs,
                                        const float& groundTruthDistance) const {

        ROI roi = calculateRoiValues(roiValues, cloudPosImgs);

        bool nonOcclusion = occlusionCheckForHomogeneousROI(roi.meanDistance, groundTruthDistance);
        if (nonOcclusion) {
                return roi;
        } else {
                ROI roiOccluded = getOccludedROI();
                return roiOccluded;
        }

    }
    ROI cloud_handler::calculateRoiValues(const ROI& roiValues, const CloudPosImgs& cloudPosImgs)const{
        ROI res;
        res = roiValues;
        CloudPosImgs pointsInROI = getPointsInROI(roiValues,cloudPosImgs);
        res.meanDistance = calculateMeanDistance(pointsInROI);
        return res;
    }


    CloudPosImgs cloud_handler::getPointsInROI(const ROI& roiValues, const CloudPosImgs& cloudPosImgs)const{
        CloudPosImgs res;

        for(auto cloudPosImg : cloudPosImgs){
            if(cloudPosImg.second.x() < roiValues.greatest_x && cloudPosImg.second.x() > roiValues.smallest_x &&
            cloudPosImg.second.y() < roiValues.greatest_y && cloudPosImg.second.y() > roiValues.smallest_y){
                res.emplace_back(cloudPosImg);
            }
        }

        return res;
    }

    CloudPosImgs cloud_handler::getProjectedPointClouds(const Camera& cameraLidar,
                                      const pcl::PointCloud<pcl::PointXYZ>& pointCloud,
                                      const int16_t& col_size,
                                      const int16_t& row_size)const{
        CloudPosImgs res;
        ImageCoordinateFloats projectedPoints;
        //std::cout << "cloud size "<< pointCloud.points.size() << std::endl;
        for (size_t i = 0; i < pointCloud.points.size() ; ++i){
            Position cloudPointPos, cloudPointTransform;
            cloudPointPos= {pointCloud.points[i].x,pointCloud.points[i].y,pointCloud.points[i].z};
            cloudPointTransform = cameraLidar.cameraPose.inverse() * cloudPointPos;

            ImageCoordinateFloat imgCoordinates(-1, -1);
            cameraLidar.cameraModel->getImagePoint(cloudPointTransform,imgCoordinates);
            if(imgCoordinates.x() >0 && imgCoordinates.x() <= col_size &&
               imgCoordinates.y() > 0 && imgCoordinates.y() <= row_size){
                projectedPoints.emplace_back(imgCoordinates);
                res.emplace_back(std::make_pair(cloudPointTransform,imgCoordinates));
            }
        }
        return res;
    }


    float cloud_handler::calculateMeanDistance(const CloudPosImgs& cloudPosImgs)const{
        float total= 0.0;
        double size = cloudPosImgs.size();
        for(auto cloudPosImg : cloudPosImgs){

            total += (getEuclideanDistance(cloudPosImg.first));
        }
        return (total / size);
    }


    float cloud_handler::getEuclideanDistance(const Position& pos)const{
        return sqrt(pow(pos.z(),2) + pow(pos.y(),2)+pow(pos.x(),2));
    }

    float cloud_handler::calculateStandardDev(const float &meanRoi,const Positions& positions)const{
        float var = 0.0;
        double size = positions.size();
        for (auto &pos : positions) {
            var += (std::pow((meanRoi - pos.z()), 2) * size);
        }
        var = var / size;
        return var;
    }


    bool cloud_handler::occlusionCheckForHomogeneousROI(const float &roiMean, const float &groundTruthDistance) const {

        // Warning constant value, u can change them into parameters
        if (roiMean >= (groundTruthDistance - 0.5) && roiMean <= (groundTruthDistance + 0.5)) {
            std::cout << "Homogenegous ROI NOT OCCLUDED" << std::endl;
            return true;
        } else {
            std::cout << "Homogenegous ROI  OCCLUDED, NO PROJECTION" << std::endl;
            return false;
        }
    }

    ROI cloud_handler::getOccludedROI() const {
        ROI res;
        res.greatest_y = -1;
        res.smallest_y = -1;
        res.smallest_x = -1;
        res.greatest_x = -1;
        return res;
    }


}