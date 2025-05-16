#include "disparity_handler.h"

namespace  data_generation {

    disparity_handler::disparity_handler() {
        std::cout << "Disparity Handler is created" << std::endl;

    };

    disparity_handler::disparity_handler(std::string disparityDir_)
            : disparityDir(disparityDir_) {

        if (!disparityDir.empty()) {
            const std::string globPath = disparityDir + "/*.png";
            cv::glob(globPath, disparityFiles);
            assert(disparityFiles.size() > 0 && "No depth image is loaded");
            std::cout << "Disparity Handler is ready with " << disparityFiles.size() << " maps" << std::endl;
        }


    }

    cv::Mat disparity_handler::thresholdingTOZERO(const cv::Mat &Image,const DisparityRange &grTrDispRange) const {
        // If the threshold value is deceeded, set the value to 0
        cv::Mat thresHoldImage;
        cv::threshold(Image, thresHoldImage, grTrDispRange.minDisp, 255, cv::THRESH_TOZERO);
        return thresHoldImage;
    }

    cv::Mat disparity_handler::thresholdingTOZEROInverse(const cv::Mat &Image, const ROI &ROICoordinates) const {
        // If the threshold value is exceed, set the value to 0
        cv::Mat thresHoldImage;
        cv::threshold(Image, thresHoldImage, ROICoordinates.maxDisparity, 255, cv::THRESH_TOZERO_INV);
        return thresHoldImage;
    }

    cv::Mat disparity_handler::thresholdingToBinaryImage(const cv::Mat &Image, const ROI &ROICoordinates) const {
        cv::Mat binaryImage;
        cv::threshold(Image, binaryImage, ROICoordinates.maxDisparity, 255, cv::THRESH_BINARY_INV);
        return binaryImage;
    }

    cv::Mat disparity_handler::thresHoldingTOZeroAndTOZeroInverse(const cv::Mat &Image, const DisparityRange &grTrDispRange) const {
        cv::Mat thresHoldImageMinMax;
        cv::threshold(Image, thresHoldImageMinMax, grTrDispRange.minDisp, 255, cv::THRESH_TOZERO);
        cv::threshold(thresHoldImageMinMax, thresHoldImageMinMax, grTrDispRange.maxDisp, 255,
                      cv::THRESH_TOZERO_INV);
        return thresHoldImageMinMax;
    }

    cv::Mat disparity_handler::applyDialation(const cv::Mat &img, const int16_t width, const int16_t height) const {
        // WARNING: put dial

        //cv::Mat dilationImage;
        cv::Mat element = cv::getStructuringElement(0, cv::Size(width, height));
        cv::dilate(img, img, element);
        return img;
    }

    Disparity disparity_handler::getDisparityValue(const cv::Point &point, const cv::Mat &maskedDisparityMap) const {
        cv::Vec3b intensity = maskedDisparityMap.at<cv::Vec3b>(Point(point.y, point.x));
        Disparity dis = Disparity(intensity.val[0]);
        return dis;
    }


    ROI disparity_handler::checkForOcclusionROI(const ROI &roiValues,
                                                const cv::Mat &maskedDisparityMap,
                                                const DisparityRange &grTrDispRange) const {

        ROI roi = calculateRoiValues(roiValues, maskedDisparityMap);;
        bool homogeneity = checkHomogeneity(roi);
        if (homogeneity) {
            bool nonOcclusion = occlusionCheckForHomogeneousROI(roi, grTrDispRange);
            if (nonOcclusion) {
                return roi;
            } else {
                ROI roiOccluded = getOccludedROI();
                return roiOccluded;
            }
        } else {
            ROI roiPartiallyOccluded = getPartiallyOccludedROI(roi, grTrDispRange, maskedDisparityMap);
            roiPartiallyOccluded.smallest_x = roiValues.smallest_x;
            roiPartiallyOccluded.smallest_y = roiValues.smallest_y;
            roiPartiallyOccluded.greatest_y = roiValues.greatest_y;
            roiPartiallyOccluded.greatest_x = roiValues.greatest_x;
            roiPartiallyOccluded.partialOcclusion =true;
            return roiPartiallyOccluded;
        }

    }


    ROI disparity_handler::calculateRoiValues(const ROI &roiValues,
                                              const cv::Mat &maskedDisparityMap) const {
        Histogram histogramExcludedOutliers = removeHistogramOutliers(roiValues, maskedDisparityMap);
        ROI res = getMeanStdDevDispRange(roiValues, histogramExcludedOutliers);
        return res;
    }


    Histogram
    disparity_handler::removeHistogramOutliers(const ROI &roiValues, const cv::Mat &maskedDisparityMap) const {
        Histogram res = excludeOutliers(roiValues, maskedDisparityMap);
        return res;
    }


    Histogram disparity_handler::excludeOutliers(const ROI &roiValues, const cv::Mat &maskedDisparityMap) const {
        Histogram res;
        Histogram dispHistogram = getDisparityHistogramROI(roiValues, maskedDisparityMap);
        ROI roiWithOutliers = getMeanStdDevDispRange(roiValues, dispHistogram);
        for (auto &dis : dispHistogram) {
            if (dis.second >= roiWithOutliers.minDisparity && dis.second <= roiWithOutliers.maxDisparity) {
                std::cout << " Disparity Frequence: " << dis.first << " Dispariy without outliers: " << dis.second
                          << std::endl;
                assert(roiWithOutliers.minDisparity < 0 && " Minimum Disparity is negative");
                res.push_back(dis);
            }
        }
        return res;
    }


    Histogram
    disparity_handler::getDisparityHistogramROI(const ROI &roiValues, const cv::Mat &maskedDisparityMap) const {
        std::unordered_map<Disparity, int16_t> dispHistogram;
        //for (int x = roiValues.smallest_x; x < roiValues.greatest_x; ++x) {
        //    for (int y = roiValues.smallest_y; y < roiValues.greatest_y ; ++y) {
        for (size_t x = 0; x < maskedDisparityMap.cols; ++x) {
            for (size_t y = 0; y < maskedDisparityMap.rows; ++y) {

                Point p(y, x);
                Disparity disp = getDisparityValue(p, maskedDisparityMap);
                // Objects represented with zeros as partial occlusion
                if ( disp!= 0 &&disp != 255) {
                    std::unordered_map<Disparity, int16_t>::iterator it = dispHistogram.find(disp);
                    if (it != dispHistogram.end()) {
                        it->second++;    // increment map's value for thedispa
                    } else {
                        dispHistogram.insert(std::make_pair(disp, 1));
                    }
                }
            }
        }
        Histogram histogram;
        for (auto &disp : dispHistogram) {
            histogram.push_back(std::make_pair(disp.second, disp.first));
            //std::cout << " Disparity Frequence: "<< disp.second << " Dispariy with outliers: " <<disp.first  << std::endl;
        }
        std::sort(histogram.begin(), histogram.end(), std::greater<std::pair<int16_t, int16_t >>());

        return histogram;
    }


    bool disparity_handler::checkHomogeneity(const ROI &roiValues) const {
        if (roiValues.stddev > roiValues.stddevThreshold) {
            std::cout << " ROI NOT HOMOGENEOUS" << std::endl;
            return false;
        } else {
            std::cout << " ROI HOMOGENEOUS" << std::endl;
            return true;
        }
    }


    ROI disparity_handler::getPartiallyOccludedROI(const ROI &roiValues,
                                                   const DisparityRange &grTrDispRange,
                                                   const cv::Mat &maskedDisparityMap) const {
        ROI roiNotOccluded, roiOccluded;
        ROI resROINotOccluded;
        for (auto &disp : roiValues.histogram) {
            if (disp.second <= grTrDispRange.maxDisp && disp.second >= grTrDispRange.minDisp) {
                std::cout <<"in" <<disp.second << std::endl;
                roiNotOccluded.histogram.emplace_back(disp);
            } else {
                std::cout <<"out" <<disp.second << std::endl;
                roiOccluded.histogram.emplace_back(disp);
            }
        }
        cv::Mat grayMasked;
        cv::cvtColor(maskedDisparityMap, grayMasked, cv::COLOR_BGR2GRAY);
        cv::Mat MaskedThresholdedDisparityMap = thresHoldingTOZeroAndTOZeroInverse(grayMasked,grTrDispRange);


        if (roiOccluded.histogram.empty()) {
            std::cout << " INHOMOGENEITY CAUSED BY PROJECTION ERROR" << std::endl;
            resROINotOccluded = roiValues;
        } else {
            if (!roiNotOccluded.histogram.empty()) {
                resROINotOccluded = getMeanStdDevDispRange(roiNotOccluded, roiNotOccluded.histogram);
                bool nonOcclusion = occlusionCheckForHomogeneousROI(resROINotOccluded, grTrDispRange);
                if (nonOcclusion) {
                    resROINotOccluded.countours = getNonOccludedArea(resROINotOccluded, MaskedThresholdedDisparityMap);
                    std::cout << "countours are found " << std::endl;
                }
            }
        }
        return resROINotOccluded;
    }


    Countours disparity_handler::getNonOccludedArea(const ROI &resROINotOccluded, const cv::Mat &MaskedThresholdedDisparityMap) const {
        Countours countours;
        cv::Mat element= cv::getStructuringElement(0, cv::Size(2, 2));
        cv::Mat output = MaskedThresholdedDisparityMap.clone();
        cv::findContours(output, countours, cv::noArray(), cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
        return countours;
    }

    ROI disparity_handler::getMeanStdDevDispRange(const ROI &roiValues, const Histogram &dispHistogram) const {
        ROI res = roiValues;
        res.histogram = dispHistogram;
        res.mean = calculateMean(dispHistogram);
        res.stddev = calculateStandardDev(res.mean, dispHistogram);
        res.minDisparity = res.mean - (2 * res.stddev); // u-2σ
        res.maxDisparity = res.mean + (2 * res.stddev); // u+2σ
        std::cout << " ROI mean: " << res.mean << std::endl;
        std::cout << " ROI Stddev: " << res.stddev << std::endl;
        //std::cout << " ROI histogram: " << res.histogram << std::endl;
        std::cout << " ROI minDisp: " << res.minDisparity << std::endl;
        return res;
    }


    float disparity_handler::calculateMean(const Histogram &dispHistogram) const {
        float total = 0.0;
        double size = 0;
        for (auto &dis : dispHistogram) {
            total += (dis.first * dis.second);
            size += dis.first;
        }
        return (total / size);
    }

    float disparity_handler::calculateStandardDev(const float &meanRoi, const Histogram &dispHistogram) const {
        float var = 0.0;
        double size = 0;
        for (auto &dis : dispHistogram) {
            var += (std::pow((meanRoi - dis.second), 2) * dis.first);
            size += dis.first;
        }
        var = var / size;
        return std::sqrt(var);
    }


    bool disparity_handler::occlusionCheckForHomogeneousROI(const ROI &roi, const DisparityRange &grTrDispRange) const {
        if (roi.mean >= grTrDispRange.minDisp && roi.mean <= grTrDispRange.maxDisp) {
            std::cout << "Homogenegous ROI NOT OCCLUDED" << std::endl;
            return true;
        } else {
            std::cout << "Homogenegous ROI  OCCLUDED, NO PROJECTION" << std::endl;
            return false;
        }
    }


    ROI disparity_handler::getOccludedROI() const {
        ROI res;
        res.greatest_y = -1;
        res.smallest_y = -1;
        res.smallest_x = -1;
        res.greatest_x = -1;
        return res;
    }


    cv::Mat disparity_handler::generateMaskForStaticObstacles(const camera_info& camera,
            const Params& params,
            const cv::Mat& maskedDisparity,
            const FrameId & curIndex)const{
            float baseLine = sqrt(pow(camera.getBaseLine().z(), 2) + pow(camera.getBaseLine().y(), 2) + pow(camera.getBaseLine().x(), 2));
            std::vector<int16_t > yValues = {1100,1100,1100,1100,1050,1050,1050,1050,1050,1050,1050,1050,1000,1000,1000,1000};
            std::vector<int16_t > dValues = {60,45,35,30,25,20,18,15,12,10,9,8,7,6,5,4,3,2};

            std::vector<Points> total_points;
            for (int i1 = 0; i1 < yValues.size(); ++i1) {
                Disparity dfirst = (baseLine * camera.getCamera().cameraModel->getFocalLength()) / dValues.at(i1);
                Disparity dSecond = (baseLine * camera.getCamera().cameraModel->getFocalLength()) / dValues.at(i1+1);
                std::vector<Disparity> xValues;
                for (int16_t x = 0; x < maskedDisparity.cols; ++x) {
                    for (int16_t y = yValues.at(i1); y < 1150; ++y) {
                        cv::Vec3b intensity = maskedDisparity.at<cv::Vec3b>(Point(x, y));
                        Disparity disparity = intensity.val[0];
                        if (disparity > dfirst && disparity < dSecond) {
                            xValues.emplace_back(x);
                        }
                    }
                }
                if( xValues.size() !=0){
                    // Boundaries
                    std::sort(xValues.begin(), xValues.end());
                    std::vector<std::vector<int64_t>> seperated_clusters = detectClustersBoundary(xValues,params.cluster_distance);
                    LeftRights boundaries = setBoundaryForEachCluster(seperated_clusters,params.cluster_size);

                    // Find Cluster's disparity Range
                    std::vector<DisparityRange> disparity_clusters = findClustersDisparityRange(maskedDisparity,boundaries,
                                                                                                dfirst,dSecond,yValues.at(i1));
                    //Add Points that meet requirements
                    detectObtacles(total_points,maskedDisparity,boundaries,disparity_clusters);

                }
            }
            cv::Mat copiedImageInv = drawPointsAndApplyClosing(total_points,maskedDisparity);
            //writeStaticMask(copiedImageInv,curIndex,params);
            return copiedImageInv;
    }





    void disparity_handler::detectObtacles(std::vector<Points>& total_points,
            const cv::Mat& maskedDisparity,
            const LeftRights& result,
            const std::vector<DisparityRange>& disparity_clusters)const{

        int counter2 = 0; // to tackle the problem of representation with zeros in disparity map
        for (int m = 0; m < result.size(); ++m) {
            Points pointler;
            for (int64_t x = result.at(m).first; x < (result.at(m).second + 1); ++x) {
                for (int16_t y = 1150; y < maskedDisparity.rows; ++y) {
                    cv::Vec3b intensity = maskedDisparity.at<cv::Vec3b>(Point(x, y));
                    Disparity disparity = Disparity(intensity.val[0]);
                    if (disparity >= (disparity_clusters.at(m).minDisp -3 ) &&
                        disparity <= (disparity_clusters.at(m).maxDisp +3 )) {
                        counter2 = 0;
                        pointler.emplace_back(Point(x, y));
                    }else{
                        if(counter2 ==3){
                            break;
                        }
                        counter2++;
                    }
                }
            }
            total_points.emplace_back(pointler);
        }

    }

    std::vector<DisparityRange> disparity_handler::findClustersDisparityRange(const cv::Mat& maskedDisparity,
                                                                              LeftRights& result,
                                                                              const Disparity& d,const Disparity& dSecond,
                                                                              const int16_t& yValues )const{
        std::vector<DisparityRange> disparity_clusters;
        int counter = -1;
        for (auto r : result) {
            ++counter;
            std::vector<Disparity> distance_cl;
            for (int16_t x = r.first; x < r.second; ++x) {
                for (int16_t y = yValues; y < 1150; ++y) {
                    cv::Vec3b intensity = maskedDisparity.at<cv::Vec3b>(Point(x, y));
                    Disparity disparity = Disparity(intensity.val[0]);
                    if (disparity > d && disparity < dSecond ) {
                        distance_cl.emplace_back(disparity);
                    }
                }
            }
            if (distance_cl.size() != 0) {
                DisparityRange dRange;
                std::sort(distance_cl.begin(), distance_cl.end());
                dRange.minDisp = distance_cl.front();
                dRange.maxDisp = distance_cl.back();
                disparity_clusters.push_back(dRange);
            } else {
                // if there is no suitable disparity memeber then delete the cluster
                result.erase(result.begin() + counter);
            }
        }
        return disparity_clusters;

    }



    std::vector<std::vector<int64_t>> disparity_handler::detectClustersBoundary(const std::vector<Disparity>& xValues,
                                                                                const int16_t& cluster_distance)const{

        std::vector<std::vector<int64_t>> result;
        // Seperate Clusters from each other
        int16_t num_cluster = 0;
        int16_t first_point = xValues.at(0);
        std::vector<int64_t> first_cluster;
        first_cluster.emplace_back(first_point);
        result.emplace_back(first_cluster);
        for (int j = 0; j < (xValues.size() - 1); ++j) {
            int16_t a = xValues.at(j);
            int16_t b = xValues.at(j + 1);
            // distance between pixels
            if (std::abs(a - b) < cluster_distance) {
                result.at(num_cluster).emplace_back(b);
            } else {
                ++num_cluster;
                std::vector<int64_t> other_clusters;
                other_clusters.emplace_back(b);
                result.emplace_back(other_clusters);
            }
        }
        return result;
    }

    LeftRights disparity_handler::setBoundaryForEachCluster(const std::vector<std::vector<int64_t>>& seperated_clusters,
                                                            const int16_t& cluster_size)const{
        LeftRights result;
        for (int l = 0; l < seperated_clusters.size(); ++l) {
            std::vector<int64_t> clustersss = seperated_clusters.at(l);
            int16_t leftest = clustersss.front();
            int16_t rightest = clustersss.back();
            if (std::abs(leftest - rightest) > cluster_size) {
                int16_t difference = std::abs(leftest - rightest);
                double numberOfCluster = difference / cluster_size;
                for (int j = 0; j < numberOfCluster; ++j) {
                    int16_t leftCluster = leftest;
                    int16_t rightCluster = leftest + cluster_size;
                    result.emplace_back(std::make_pair(leftCluster, rightCluster));
                    leftest = rightCluster;
                }
            } else {
                int16_t leftCluster = leftest;
                int16_t rightCluster = rightest;
                result.emplace_back(std::make_pair(leftCluster, rightCluster));
            }
        }
        return result;
    }

    cv::Mat disparity_handler::drawPointsAndApplyClosing(const std::vector<Points>& pointlerler,const cv::Mat& maskedDisparity)const{
        cv::Mat image(maskedDisparity.rows, maskedDisparity.cols, CV_8UC3, cv::Scalar(0, 0, 0));
        for (int k = 0; k < pointlerler.size(); ++k) {
            Points poiler = pointlerler.at(k);
            for (int j = 0; j < poiler.size(); ++j) {
                Point p = poiler.at(j);
                cv::circle(image, p, 3, cv::Scalar(255, 255, 255), 3, 8);
            }
        }
        cv::Mat element = cv::getStructuringElement(0, cv::Size(10, 10));
        cv::dilate(image,image,element);
        cv::erode(image,image,element);
        cv::Mat copiedImageInv;
        cv::bitwise_not(image, copiedImageInv);
        return copiedImageInv;
    }

    void disparity_handler::writeStaticMask(const cv::Mat& copiedImageInv,const FrameId & curIndex,const Params& params )const{
        std::stringstream ss;
        ss << params.maskStaticDir << "/maskStatic" << std::setw(8) << std::setfill('0') << curIndex << ".png";
        std::string s = ss.str();
        cv::imwrite(s, copiedImageInv);

    }


}







