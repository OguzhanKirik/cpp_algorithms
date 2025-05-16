
#include "data_prep.h"

namespace data_generation {

    data_prep::data_prep(const Params& params){

        timedPoses_ = loadTimedPoses(params.poseFile);

        if (!params.imageStampFile.empty()) {
            stampedFramesImages_ = (getStampFileType(params.imageStampFile) == StampFileType::RAW) ? loadStamps(params.imageStampFile)
                                                                                            : loadStampsCSV(params.imageStampFile);
            poses_ = matchPoses(timedPoses_, stampedFramesImages_, params.timeTolerance, params.timeOffset);
            std::cout << "Data is prepared with time stamps " << std::endl;
        }else{
            poses_ = framePoses(timedPoses_);
            std::cout << "Data is prepared without time stamps" << std::endl;
        }
        if(params.occlusionHandlings.lidar == "true"){
            if(!params.cloudDir.empty()){
                const std::string globCloudPath = params.cloudDir + "/*.pcd";
                cv::glob(globCloudPath, cloudPCDFIles);
                std::cout << "Clouds are ready with " << cloudPCDFIles.size() << " PCD Files " << std::endl;
            }
            stampedFramesClouds_ = (getStampFileType(params.lidarStampFile) == StampFileType::RAW) ? loadStamps(params.lidarStampFile)
                                                                                            : loadStampsCSV(params.lidarStampFile);
            //clouds_ = matchImageFrameLidarFrame(stampedFramesImages_,stampedFramesClouds_);
            clouds_ = matchLidar(stampedFramesImages_,stampedFramesClouds_);
        }
        if(params.occlusionHandlings.binaryMask == "true"){
            if (!params.binaryMaskDir.empty()) {
                binaryMaskPreprocFiles = processBinaryMask(params.binaryMaskDir);
            }
        }
        if(params.occlusionHandlings.stereoVision =="true"){
            if (!params.disparityDir.empty()) {
                const std::string globPath = params.disparityDir + "/*.png";
                cv::glob(globPath, disparityMapFiles);
                assert(disparityMapFiles.size() > 0 && "No depth image is loaded");
                std::cout << "Disparity Maps are ready with " << disparityMapFiles.size() << " maps" << std::endl;
                // Generate Mask for Static Obstacles
                //maskStaticObstacles = generataMasksForStaticObstacles(params.maskStaticDir);
                //std::cout << "Masks for Static Obstacles are ready with " << maskStaticObstacles.size() << " maps" << std::endl;
            }
        }
        if(!params.imageDir.empty()){
            const std::string globPath = params.imageDir + "/*.png";
            cv::glob(globPath, imageFiles);
            assert(imageFiles.size() < 1 && "No image is loaded");
            std::cout << "Images are ready with " << imageFiles.size() << std::endl;
            fileSize = imageFiles.size();

        }


    };

    StampedFrames data_prep::getStampedFramesImages() const {
        return stampedFramesImages_;
    }



    StampedClouds data_prep::getStampedFramesClouds() const {
        return stampedFramesClouds_;
    }

    TimedPoseList data_prep::getTimedPoseList() const {
        return timedPoses_;
    }

    FramedPoses data_prep::getFramedPoses() const {
        return poses_;
    }

    FramedClouds data_prep::getFramedClouds()const{
        return clouds_;
    }

    Cloud data_prep::getCloudByID(const FrameId &frameId) const {
        FrameId LidarID;
        Cloud res;
        if (clouds_.find(frameId) != clouds_.end()) {
            LidarID = clouds_.at(frameId);
            res = getPointCloud(LidarID);
        }
        return res;
    }

    Cloud data_prep::getPointCloud(const FrameId& frameId)const{

        Cloud pointCloud;
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPCDFIles[frameId], pointCloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file \n");
        }
        return pointCloud;
    }


    Stamp data_prep::getImageStampById(const FrameId &frameId) const {
        return stampedFramesImages_.at(frameId);
    }



    Pose data_prep::getPoseByID(const FrameId &frameId) const {
        Pose res;
        if (poses_.find(frameId) != poses_.end()) {
            res = poses_.at(frameId);
        }
        return res;
    }

    cv::Mat data_prep::getDisparityMap(const FrameId &frame)const{
        cv::Mat disparityMap = cv::imread(disparityMapFiles[frame]);
        return disparityMap;
    }

    cv::Mat data_prep::getImage(const FrameId &frame)const{
        cv::Mat image = cv::imread(imageFiles[frame]);
        return image;
    }

    int64_t data_prep::getFileSize()const{
        return fileSize;
    }

    TimedPoseList data_prep::loadTimedPoses(const std::string& posesFile)const {
        auto posesMap = locmap_io::loadTrajectory<Pose>(posesFile);
        TimedPoseList poses;
        for (const auto& pose : posesMap) {
            poses.push_back(std::make_pair(pose.first, pose.second.second));
        }
        return poses;
    }

    StampFileType data_prep::getStampFileType(const std::string& stampFile)const {
        std::string line;
        std::ifstream file(stampFile);
        if (file.good()) {
            if (std::getline(file, line).good()) {
                auto found = line.find(",");
                // if there is "," then raw otherwise CSV
                if (found == std::string::npos) {
                    return StampFileType::RAW;
                } else {
                    return StampFileType::CSV;
                }
            }
            file.close();
        }
        throw std::runtime_error("Could not read file " + stampFile);
    }


    StampedFrames data_prep::loadStampsCSV(const std::string& posesFile)const {
        StampedFrames res;
        io::CSVReader<3> in(posesFile);
        in.read_header(io::ignore_no_column, "time_s", "time_ns", "frame");
        uint64_t secs;
        uint64_t nsecs;
        int frame;
        while (in.read_row(secs, nsecs, frame)) {
            res.insert(std::make_pair(frame, secs * static_cast<uint64_t>(1e9) + nsecs));
        }

        return res;
    }
    StampedFrames data_prep::loadStamps(const std::string& stampFile)const {
        StampedFrames res;
        std::string line;
        std::ifstream file(stampFile);
        FrameId frame(0);
        if (file.good()) {
            while (std::getline(file, line).good()) {
                //frame from 0 and pair it with line which is converted from string in int
                res.insert(std::make_pair(frame++, std::stoull(line)));
            }
            file.close();
        } else {
            throw std::runtime_error("Could not read file " + stampFile);
        }

        return res;
    }




    FramedPoses data_prep::matchPoses(const TimedPoseList& poses,
                                 const StampedFrames& frames,
                                 const uint64_t tolerance,
                                 const uint64_t offset)const {
        FramedPoses res;
        for (const auto& frame : frames) {
            Pose bestPose;

            int64_t diff = std::numeric_limits<int64_t>::max();

            const auto frameTime = frame.second;

            for (const auto& pose : poses) {

                auto curDiff = signedDiff<int64_t>(frameTime, pose.first + offset);
                //std::cout << "cur Diff: "<<curDiff << std::endl;
                if (std::abs(curDiff) < diff) {
                    diff = std::abs(curDiff);
                    bestPose = pose.second;
                }
            }
            res.insert(std::make_pair(frame.first, bestPose));
        }

        return res;
    }



    FramedPoses data_prep::framePoses(const TimedPoseList& poses)const {
        FramedPoses res;
        int16_t  counter = 0;
            for (const auto& pose : poses) {

                res.insert(std::make_pair(counter, pose.second));
                counter++;
            }

        return res;
    }


    FramedClouds data_prep::matchLidar(const StampedFrames& imageFrames,
                                      const StampedFrames& cloudFrames)const{
        // stampedclouds --> FrameID, Stamp
        // stampedFrames --> FrameID, Stamp
        FramedClouds result;
        for (const auto &imageFrame : imageFrames) {
            FrameId closestLidarId;
            int64_t diff = std::numeric_limits<int64_t>::max();
            for (const auto &cloudFrame : cloudFrames) {
                auto curDiff = signedDiff<int64_t>(cloudFrame.second, imageFrame.second);
                if (std::abs(curDiff) < diff) {
                    diff = std::abs(curDiff);
                    closestLidarId = cloudFrame.first;
                }
            }

            result.insert(std::make_pair(imageFrame.first, closestLidarId));
        }
        return result;
    }


    cv::Mat data_prep::getBinaryMask(const FrameId &frameId )const{
        cv::Mat binaryImage = cv::imread(binaryMaskPreprocFiles[frameId]);
        return binaryImage;
    }
    std::vector<cv::String> data_prep::processBinaryMask(const std::string& binaryMaskFile)const{
        const std::string globPathBinaryMask = binaryMaskFile + "/*.png";
        std::vector<cv::String> filesBinaryMask;
        cv::glob(globPathBinaryMask, filesBinaryMask);


        std::string path = binaryMaskFile + "/binaryMaskPreproc";
        boost::filesystem::path dir(path);



        if(boost::filesystem::is_directory(path)) {
            std::cout << "BinaryMaskPreprocFile already exists " << "\n";
        }else {
            boost::filesystem::create_directory(dir);
            std::cout << "Successfully create BinaryMaskPreprocFile" << "\n";

            for (int p = 0; p < filesBinaryMask.size(); ++p) {

                cv::Mat image = cv::imread(filesBinaryMask[p]);
                cv::Mat copiedImage = image.clone();
                cv::Mat greyCopiedImage, labels, stats, centroids;
                cv::cvtColor(copiedImage, greyCopiedImage, CV_BGR2GRAY);

                auto num_objects = cv::connectedComponentsWithStats(greyCopiedImage, labels, stats, centroids, 4);
                for (int j = 1; j < num_objects; ++j) {
                    int16_t leftX = stats.at<int>(j, cv::CC_STAT_LEFT);
                    int16_t height = stats.at<int>(j, cv::CC_STAT_HEIGHT);
                    int16_t width = stats.at<int>(j, cv::CC_STAT_WIDTH);
                    int16_t topY = stats.at<int>(j, cv::CC_STAT_TOP);
                    int16_t lowestY = topY + height;

                    //if (lowestY < ((image.rows / 3) * 2)) {
                    if (lowestY < 1170) {
                        for (int k = leftX; k < (leftX + width); ++k) {
                            for (int l = topY; l < (topY + height); ++l) {
                                copiedImage.at<cv::Vec3b>(l, k) = 0;
                            }
                        }
                    }
                }

                cv::Mat copiedImageInv;
                cv::bitwise_not(copiedImage,copiedImageInv);

                cv::Mat element= cv::getStructuringElement(0, cv::Size(10, 10));
                cv::dilate(copiedImageInv,copiedImageInv,element);
                cv::erode(copiedImageInv,copiedImageInv,element);

                std::stringstream ss;
                ss << path << "/mask" << std::setw(8) << std::setfill('0') << p << ".png";
                std::string s = ss.str();
                cv::imwrite(s, copiedImageInv);
            }
        }
        std::vector<cv::String> filesBinaryMaskPreProc;
        const std::string globPathBinaryMaskPreProc = path  + "/*.png";
        cv::glob(globPathBinaryMaskPreProc, filesBinaryMaskPreProc);

        std::cout << " Binary Masked are preprocessed "<< std::endl;

        return filesBinaryMaskPreProc;

    }


    cv::Mat data_prep::getMaskStaticObstacle(const FrameId &frameId )const{
        cv::Mat maskStatic = cv::imread(maskStaticObstacles[frameId]);
        return maskStatic;
    }






}