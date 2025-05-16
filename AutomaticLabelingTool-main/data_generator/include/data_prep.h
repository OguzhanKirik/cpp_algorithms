#pragma once


// Own classes
#include "data_type.h"
#include "csv.h"

// MRT Libraries
#include <locmap_io/load_poses.h>
#include <transform3d/transform3d.h>

// External Libraries
#include <math.h>




namespace data_generation {

    class data_prep {
    public:

        /**
     * @brief  Depending on the preferred mode, this constructor will do the followings:
         * load vehicle pose
         * match vehicle pose with each image
         * match lidar data with each image
         * preprocessed binary images to remove false predictions
         * read disparity maps
         * read images to be labeled
     * @param params.posefile file path to the poses
     * @param timedPoses_ contains vehicle poses
     * @param poses_ contains vehicle poses that is asssociated with image frames



     * @param params.occlusionHandlings.lidar if true  lidar data is processed or not
     * @param stampedFramesClouds_ contains timestamps
     * @param cloudsFramedStamps_ constains matched image and clouds according to timestamps
     * @param params.stampImageFile file path to the image time stamps, if it exists, matching is done based on
         * timestamps, otherwise based on sequence


     * @param params.occlusionHandlings.binaryMask if true, binary images are processed
     * @param binaryMaskPreprocFiles  contains preprocessed binary images


     * @param params.occlusionHandlings.stereoVision if true, disparity maps are loaded
     * @param disparityMapFiles contains disparity maps
     * @param params.disparityDir folder path contains disparity maps

     * @param params.imageDir folder path to the images
     * @param imageFiles contains images
     */
        data_prep(const Params& params);

     /**
        * @brief Returns the number of images to be labeled
     */
        int64_t getFileSize()const;

     /**
     * @brief Returns the corresponding binary mask with the given frame number
     */
        cv::Mat getBinaryMask(const FrameId &frameId )const;

     /**
     * @brief Returns the corresponding dispartiy map with the given frame number
     */
        cv::Mat getDisparityMap(const FrameId &frame)const;

     /**
     * @brief Returns the corresponding image with the given frame number
     */
        cv::Mat getImage(const FrameId &frame)const;

     /**
     * @brief Returns corresponding vehicle pose with the given frame number
     */
        Pose getPoseByID(const FrameId &frameId) const;

        cv::Mat getMaskStaticObstacle(const FrameId &frameId )const;


        StampedFrames getStampedFramesImages() const;

        StampedClouds getStampedFramesClouds() const;

        FramedPoses getFramedPoses() const;

        FramedClouds getFramedClouds()const;

        TimedPoseList getTimedPoseList() const;

        Stamp getImageStampById(const FrameId &frameId) const;

        Cloud getCloudByID(const FrameId &frameId) const;


    private:
        StampedFrames stampedFramesImages_;
        StampedClouds stampedFramesClouds_;
        FramedPoses poses_;
        TimedPoseList timedPoses_;
        FramedClouds clouds_;
        std::vector<cv::String> binaryMaskPreprocFiles;
        std::vector<cv::String> maskStaticObstacles;
        std::vector<cv::String> disparityMapFiles;
        std::vector<cv::String> imageFiles;
        std::vector<cv::String> cloudPCDFIles;
        uint64_t fileSize;

        /**
        * @brief Returns loaded  vehicle poses
        */
        TimedPoseList loadTimedPoses(const std::string &posesFile)const;
        /**
        * @brief Returns loaded stamps from csv type
        */
        StampedFrames loadStampsCSV(const std::string &posesFile)const;
        /**
        * @brief Returns loaded stamps
        */
        StampedFrames loadStamps(const std::string &posesFile)const;
        /**
        * @brief Returns stamp file type
        */
        StampFileType getStampFileType(const std::string &stampFile)const;

        /**
        * @brief Returns the vehicle poses with their corresponding images based on time stamps
        */
        FramedPoses matchPoses(const TimedPoseList &poses,
                               const StampedFrames &frames,
                               const uint64_t tolerance = 50e6,
                               const uint64_t offset = 0ul)const;
        //FramedClouds matchImageFrameLidarFrame(const StampedFrames& imageFrames,
        //                                       const StampedFrames& cloudFrames)const;
        FramedClouds matchLidar(const StampedFrames& imageFrames,const StampedClouds& cloudFrames)const;

        /**
        * @brief Returns processed binary images folder, noises are removed
        */
        std::vector<cv::String> processBinaryMask(const std::string& binaryMaskFile)const;

        /**
        * @brief Returns the vehicle poses with their corresponding images based on sequence
        */
        FramedPoses framePoses(const TimedPoseList& poses)const;

        Cloud getPointCloud(const FrameId& frameId)const;

    };

}
