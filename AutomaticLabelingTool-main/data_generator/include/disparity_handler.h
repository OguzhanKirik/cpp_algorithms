#pragma once

// Own classes
#include "data_type.h"
#include "camera_info.h"



namespace data_generation {


    class disparity_handler {
    public:

        /**
    * @brief  this contructor loads create an object in cloud_handler class,
     */
        disparity_handler();

        disparity_handler(std::string depthImagesPath_);

        /**
  * @brief  check occlusions in the region of the interest.
     * @param roiValues contains the pixel coordinates of ROI and its attributes
     * @param maskedDisparityMap contains the disparity map of ROI masked by the dynamic filter
     * @param poses_ contains the calculated ground-truth disparity range
  */
        ROI checkForOcclusionROI(const ROI &roiValues,
                                 const cv::Mat& maskedDisparityMap,
                                 const DisparityRange& grTrDispRange)const;
        /**
     * @brief  create a mask for static obstacles
        * @param camera is the camera objects
        * @param params is the params object
        * @param maskedDisparity is the dispariy map for the whole scene filtered by dynamic obstacle filter
        * @param curIndex is the current image frame to be processed
     */
        cv::Mat generateMaskForStaticObstacles(const camera_info& camera,
                const Params& params,
                const cv::Mat& maskedDisparity,
                const FrameId & curIndex)const;


    private:
        std::string disparityDir;
        std::vector<cv::String> disparityFiles;
    private:

        ROI getOccludedROI()const;

        bool checkHomogeneity(const ROI& roiValues)const;

        float calculateMean(const Histogram& dispHistogram)const;

        float calculateStandardDev(const float& meanRoi,const Histogram& dispHistogram)const;

        Histogram excludeOutliers(const ROI& roiValues,const cv::Mat& maskedDisparityMap)const;

        Histogram removeHistogramOutliers(const ROI& roiValues,const cv::Mat& maskedDisparityMap)const;

        cv::Mat thresholdingTOZEROInverse(const cv::Mat& Image, const ROI& ROICoordinates)const;

        cv::Mat applyDialation(const cv::Mat &img, const int16_t width, const int16_t height)const;

        cv::Mat thresHoldingTOZeroAndTOZeroInverse(const cv::Mat &Image, const DisparityRange &grTrDispRange)const;

        cv::Mat thresholdingTOZERO(const cv::Mat& Image, const DisparityRange& grTrDispRange)const;

        Disparity getDisparityValue(const cv::Point& point,const cv::Mat& maskedDisparityMap)const;

        Histogram getDisparityHistogramROI(const ROI& roiValues,const cv::Mat& maskedDisparityMap)const;

        ROI getMeanStdDevDispRange(const ROI& roiValues,const Histogram& dispHistogram)const;

        ROI getPartiallyOccludedROI(const ROI& roiValues,
                                    const DisparityRange& grTrDispRange,
                                    const cv::Mat& maskedDisparityMap)const;

        ROI calculateRoiValues(const ROI& roiValues,
                               const cv::Mat& maskedDisparityMap)const;

        bool occlusionCheckForHomogeneousROI(const ROI& roi,const DisparityRange& grTrDispRange)const;

        Countours getNonOccludedArea(const ROI& resROINotOccluded,const cv::Mat& maskedDisparityMap)const;

        cv::Mat thresholdingToBinaryImage(const cv::Mat& Image, const ROI& ROICoordinates)const;

        //Static MAsk
        cv::Mat drawPointsAndApplyClosing(const std::vector<Points>& pointlerler,const cv::Mat& maskedDisparity)const;

        void writeStaticMask(const cv::Mat& copiedImageInv,const FrameId & curIndex,const Params& params)const;

        std::vector<std::vector<int64_t>> detectClustersBoundary(const std::vector<Disparity>& xValues,const int16_t& cluster_distance)const;

        std::vector<DisparityRange> findClustersDisparityRange(const cv::Mat& maskedDisparity,
                LeftRights& result,
                const Disparity& d,const Disparity& dSecond, const int16_t& yValues)const;

        void detectObtacles(std::vector<Points>& total_points,
                const cv::Mat& maskedDisparity,
                const LeftRights& result,
                const std::vector<DisparityRange>& disparity_clusters)const;

        LeftRights setBoundaryForEachCluster(const std::vector<std::vector<int64_t>>& seperated_clusters,const int16_t& cluster_size)const;

    };

}





