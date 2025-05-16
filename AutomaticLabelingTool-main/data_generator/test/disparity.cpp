// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
#include <iostream>
#include <stdlib.h>
#include <ctime>
#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"
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


#include<stdio.h> // change file name



#include <random>  // Gaussian disturbution
/*
TEST(disparity, checkingDisparityMapsFromDistance) {
    using namespace data_generation;
    Params params;
    params.loadParams();

    camera_info camera(params);
    //Position b = camera.getBaseLine();
    //std::cout <<"focal" <<camera.getCamera().cameraModel->getFocalLength() << std::endl;
    //std::cout <<"baseline" <<   sqrt(pow(b.z(),2) + pow(b.y(),2)+pow(b.x(),2))<< std::endl;
    //std::cout << camera.getBaseLine().matrix() << std::endl;



    cv::Mat img = cv::imread("/home/kirik/Desktop/smallPole.png");//It returns a matrix object


    cv::Mat maskedDisparity;
    cv::Mat maskAdd = img.clone();
    for(int y=2;y<img.rows;y++){
        for(int x=1;x<img.cols;x++){
            // get pixel
            cv::Vec3b & color = maskAdd.at<cv::Vec3b>(y,x);
            if(color[0] == 0){
                // 7th class
                color[0] = 67;
                color[1] = 67;
                color[2] = 67;
            }else{
                color[0] = 0;
                color[1] = 0;
                color[2] = 0;
            }
        }
    }

    maskAdd.convertTo(maskAdd, CV_8U, 255);

    //cvtColor(maskAdd, img, cv::COLOR_RGB2GRAY);
    cv::Mat graymat;
    cv::applyColorMap(maskAdd,graymat,20);
    if (img.rows > 1000 || img.cols > 1500)
        cv::resize(img, img, cv::Size(img.cols * 0.35, img.rows * 0.35), 0, 0);
    if (graymat.rows > 1000 || graymat.cols > 1500)
        cv::resize(graymat, graymat, cv::Size(img.cols * 0.35, img.rows * 0.35), 0, 0);
    cv::imshow("s",graymat);
    cv::imshow("a",img);
    //cv::imwrite("/home/kirik/Desktop/vnl_Color_image.png",graymat);
    cv::waitKey(0);
}


*/

TEST(disparity, detectStaticObstacles) {


    using namespace data_generation;
    Params params;
    params.loadParams();

    camera_info camera(params);
    data_prep data(params);

    for (int curIndex = 0; curIndex < 6315; ++curIndex) {

        cv::Mat maskDynamic = data.getBinaryMask(curIndex);
        cv::Mat disp = data.getDisparityMap(curIndex);

        cv::Mat element2= cv::getStructuringElement(0, cv::Size(20, 20));
        cv::Mat maskedDisparitytoadd;
        disp.copyTo(maskedDisparitytoadd, maskDynamic);

        cv::Mat maskedDisparity;
        cv::Mat maskAdd = maskDynamic.clone();
        for(int y=0;y<maskDynamic.rows;y++){
            for(int x=0;x<maskDynamic.cols;x++){
                // get pixel
                cv::Vec3b & color = maskAdd.at<cv::Vec3b>(y,x);
                if(color[0] == 0){
                    // 7th class
                    color[0] = 7;
                    color[1] = 7;
                    color[2] = 7;
                }else{
                    color[0] = 0;
                    color[1] = 0;
                    color[2] = 0;
                }
            }
        }
        maskedDisparity = maskedDisparitytoadd + maskAdd;


        float baseLine = sqrt(
                pow(camera.getBaseLine().z(), 2) + pow(camera.getBaseLine().y(), 2) + pow(camera.getBaseLine().x(), 2));



        std::vector<int16_t > yValues = {1100,1100,1100,1100,1050,1050,1050,1050,1050,1050,1050,1050,1000,1000,1000,1000};
        std::vector<int16_t > dValues = {60,45,35,30,25,20,18,15,12,10,9,8,7,6,5,4,3,2};
        //std::vector<int16_t > yValues = {1050,1050,1050,1050};
        //std::vector<int16_t > dValues = {8,7,6,5,4};
        Points pointHoritzon;
        std::vector<Points> pointlerler;
        for (int i1 = 0; i1 < yValues.size(); ++i1) {
            Disparity d = (baseLine * camera.getCamera().cameraModel->getFocalLength()) / dValues.at(i1);
            Disparity dSecond = (baseLine * camera.getCamera().cameraModel->getFocalLength()) / dValues.at(i1+1);
            std::cout << " dispariy :" << d << "dSe" << dSecond << std::endl;
            std::cout << ",y1 " <<  yValues.at(i1 ) << std::endl;


            // save x points that represent obstacles
            std::vector<Disparity> xler;
            for (int16_t x = 0; x < maskedDisparity.cols; ++x) {
                for (int16_t y = yValues.at(i1); y < 1150; ++y) {
                    cv::Vec3b intensity = maskedDisparity.at<cv::Vec3b>(Point(x, y));
                    Disparity disparity = Disparity(intensity.val[0]);
                    if (disparity > d && disparity < dSecond) {
                        xler.emplace_back(x);
                        pointHoritzon.emplace_back(Point(x, y));
                    }
                }
            }
            if( xler.size() !=0){

                std::vector<std::vector<int64_t>> seperated_clusters;
                std::sort(xler.begin(), xler.end());


                // Seperate Clusters from each other
                int16_t num_cluster = 0;
                int16_t first_point = xler.at(0);
                std::vector<int64_t> first_cluster;
                first_cluster.emplace_back(first_point);
                seperated_clusters.emplace_back(first_cluster);
                for (int j = 0; j < (xler.size() - 1); ++j) {
                    int16_t a = xler.at(j);
                    int16_t b = xler.at(j + 1);
                    // distance between pixels
                    if (std::abs(a - b) < 10) {
                        seperated_clusters.at(num_cluster).emplace_back(b);
                    } else {
                        ++num_cluster;
                        std::vector<int64_t> other_clusters;
                        other_clusters.emplace_back(b);
                        seperated_clusters.emplace_back(other_clusters);
                    }
                }

                //std::cout << seperated_clusters.size() << std::endl;



                // Pair left and right X values
                LeftRights result;
                for (int l = 0; l < seperated_clusters.size(); ++l) {
                    std::vector<int64_t> clustersss = seperated_clusters.at(l);
                    int16_t leftest = clustersss.front();
                    int16_t rightest = clustersss.back();
                    //std::cout <<"left" << leftest << std::endl;
                    //std::cout <<"right" << rightest << std::endl;
                    // Divide Big CLusters into small Parts
                    if (std::abs(leftest - rightest) > 50) {
                        int16_t difference = std::abs(leftest - rightest);
                        double numberOfCluster = difference / 50;
                        for (int j = 0; j < numberOfCluster; ++j) {
                            int16_t leftCluster = leftest;
                            int16_t rightCluster = leftest + 50;
                            result.emplace_back(std::make_pair(leftCluster, rightCluster));
                            leftest = rightCluster;
                            //std::cout << "left" << leftCluster << std::endl;
                            //std::cout << "right" << rightCluster << std::endl;
                        }

                    } else {
                        int16_t leftCluster = leftest;
                        int16_t rightCluster = rightest;
                        //std::cout << "left" << leftCluster << std::endl;
                        //std::cout << "right" << rightCluster << std::endl;
                        result.emplace_back(std::make_pair(leftCluster, rightCluster));
                    }

                }


                //std::cout << "number of cluster" << result.size() << std::endl;

                // Find Cluster's disparity Range
                std::vector<DisparityRange> disparity_clusters;
                int counter = -1;
                for (auto r : result) {
                    ++counter;
                    // std::cout << "cluster " << counter << std::endl;
                    std::vector<Disparity> distance_cl;
                    for (int16_t x = r.first; x < r.second; ++x) {
                        for (int16_t y = yValues.at(i1); y < 1150; ++y) {
                            cv::Vec3b intensity = maskedDisparity.at<cv::Vec3b>(Point(x, y));
                            Disparity disparity = Disparity(intensity.val[0]);
                            if (disparity > d && disparity < dSecond ) {
                                distance_cl.emplace_back(disparity);
                            }
                        }
                    }
                    //std::cout << "number of acceptable disparity" << distance_cl.size() << std::endl;
                    // Warning: Maybe add a margin
                    if (distance_cl.size() != 0) {
                        DisparityRange dRange;
                        std::sort(distance_cl.begin(), distance_cl.end());
                        Disparity average =
                                accumulate(distance_cl.begin(), distance_cl.end(), 0.0) / distance_cl.size();
                        dRange.minDisp = distance_cl.front();
                        dRange.maxDisp = distance_cl.back();
                        //std::cout << "min" << dRange.minDisp << std::endl;
                        //std::cout << "max" << dRange.maxDisp << std::endl;
                        disparity_clusters.push_back(dRange);
                    } else {
                        // if there is no suitable disparity memeber then delete the cluster
                        result.erase(result.begin() + counter);
                    }

                }

                std::cout << result.size() << std::endl;

                //std::vector<Points> pointlerler;
                int counter2 = 0;
                for (int m = 0; m < result.size(); ++m) {
                    Points pointler;
                    for (int64_t x = result.at(m).first; x < (result.at(m).second + 1); ++x) {
                        for (int y = 1150; y < maskedDisparity.rows; ++y) {
                            cv::Vec3b intensity = maskedDisparity.at<cv::Vec3b>(Point(x, y));
                            Disparity disparity = Disparity(intensity.val[0]);
                            if (disparity >= (disparity_clusters.at(m).minDisp -3) &&
                                disparity <= (disparity_clusters.at(m).maxDisp + 3 ) ) {
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
                    pointlerler.emplace_back(pointler);
                }
            }
        }

            cv::Mat image(maskedDisparity.rows, maskedDisparity.cols, CV_8UC3, cv::Scalar(0, 0, 0));

            //std::cout <<"total point" << pointlerler.size() << std::endl;
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


        std::stringstream ss;
        ss << "/home/kirik/Desktop/deneme_1" << "/maskStatic" << std::setw(8) << std::setfill('0') << curIndex << ".png";
        std::string s = ss.str();
        cv::imwrite(s, copiedImageInv);
        /*
        cv::Mat original = data.getImage(curIndex);

        for (int k = 0; k < pointHoritzon.size(); ++k) {
            Point poiler = pointHoritzon.at(k);
            cv::circle(original, poiler, 3, cv::Scalar(0, 255, 255), 3, 8);
        }


         cv::Mat res;
         cv::Mat origImage = data.getImage(curIndex);
         double alpha = 0.5;
         double beta = 1 - alpha;
         cv::addWeighted(origImage,alpha,copiedImageInv,beta,0.0,res);



        std::stringstream ss;
        ss << "/mrtstorage/users/kirik/lex_map_ost/extras/new_wave2/mask" << "/mask" << std::setw(8) << std::setfill('0') << curIndex << ".png";
        std::string s = ss.str();
        cv::imwrite(s, copiedImageInv);

        std::stringstream ss2;
        ss2 << "/mrtstorage/users/kirik/lex_map_ost/extras/new_wave2/overlayed" << "/overlayed" << std::setw(8) << std::setfill('0') << curIndex << ".png";
        std::string s2 = ss2.str();
        cv::imwrite(s2, res);

    }

}


/*
TEST(disparity, detectStaticObstaclesShow) {


    using namespace data_generation;
    Params params;
    params.loadParams();

    camera_info camera(params);
    data_prep data(params);



        uint64_t pressedKey = 0;
        int curIndex = 302;
        uint64_t min = 0;
        uint64_t max = data.getFileSize() - 1;

        while (pressedKey != 120) {

        cv::Mat maskDynamic = data.getBinaryMask(curIndex);
        cv::Mat disp = data.getDisparityMap(curIndex);
        //cv::Mat element2= cv::getStructuringElement(0, cv::Size(20, 20));
        //cv::erode(maskDynamic,maskDynamic,element2);
        cv::Mat maskedDisparitytoadd;
        disp.copyTo(maskedDisparitytoadd, maskDynamic);


            cv::Mat maskedDisparity;
            cv::Mat maskAdd = maskDynamic.clone();
            for(int y=0;y<maskDynamic.rows;y++){
                for(int x=0;x<maskDynamic.cols;x++){
                    // get pixel
                    cv::Vec3b & color = maskAdd.at<cv::Vec3b>(y,x);
                    if(color[0] == 0){
                        // 7th class
                        color[0] = 7;
                        color[1] = 7;
                        color[2] = 7;
                    }else{
                        color[0] = 0;
                        color[1] = 0;
                        color[2] = 0;
                    }
                }
            }
            maskedDisparity = maskedDisparitytoadd + maskAdd;












        float baseLine = sqrt(
                pow(camera.getBaseLine().z(), 2) + pow(camera.getBaseLine().y(), 2) + pow(camera.getBaseLine().x(), 2));



        //std::vector<int16_t > yValues = {1100,1100,1100,1100,1100,1100,1100,1050,1050,1050,1050,1050,1000,1000,1000,1000};
        //std::vector<int16_t > dValues = {60,45,35,30,25,20,18,15,12,10,9,8,7,6,5,4,3,2};
        std::vector<int16_t > yValues = {1050,1050,1050,1050,1050,1050,1000,1000};
        std::vector<int16_t > dValues = {15,12,10,9,8,7,6,5,4};
        Points pointHoritzon;
        std::vector<Points> pointlerler;
        for (int i1 = 0; i1 < yValues.size(); ++i1) {
            Disparity d = (baseLine * camera.getCamera().cameraModel->getFocalLength()) / dValues.at(i1);
            Disparity dSecond = (baseLine * camera.getCamera().cameraModel->getFocalLength()) / dValues.at(i1+1);
            std::cout << " dispariy :" << d << "dSe" << dSecond << std::endl;
            std::cout << ",y1 " <<  yValues.at(i1 ) << std::endl;


            // save x points that represent obstacles
            std::vector<Disparity> xler;
            for (int16_t x = 0; x < disp.cols; ++x) {
                for (int16_t y = yValues.at(i1); y < 1150; ++y) {
                    cv::Vec3b intensity = maskedDisparity.at<cv::Vec3b>(Point(x, y));
                    Disparity disparity = Disparity(intensity.val[0]);
                    if (disparity > d && disparity < dSecond) {
                        xler.emplace_back(x);
                        pointHoritzon.emplace_back(Point(x, y));
                    }
                }
            }
            if( xler.size() !=0){

                std::vector<std::vector<int64_t>> seperated_clusters;
                std::sort(xler.begin(), xler.end());


                // Seperate Clusters from each other
                int16_t num_cluster = 0;
                int16_t first_point = xler.at(0);
                std::vector<int64_t> first_cluster;
                first_cluster.emplace_back(first_point);
                seperated_clusters.emplace_back(first_cluster);
                for (int j = 0; j < (xler.size() - 1); ++j) {
                    int16_t a = xler.at(j);
                    int16_t b = xler.at(j + 1);
                    // distance between pixels
                    if (std::abs(a - b) < 10) {
                        seperated_clusters.at(num_cluster).emplace_back(b);
                    } else {
                        ++num_cluster;
                        std::vector<int64_t> other_clusters;
                        other_clusters.emplace_back(b);
                        seperated_clusters.emplace_back(other_clusters);
                    }
                }

                //std::cout << seperated_clusters.size() << std::endl;



                // Pair left and right X values
                LeftRights result;
                for (int l = 0; l < seperated_clusters.size(); ++l) {
                    std::vector<int64_t> clustersss = seperated_clusters.at(l);
                    int16_t leftest = clustersss.front();
                    int16_t rightest = clustersss.back();
                    //std::cout <<"left" << leftest << std::endl;
                    //std::cout <<"right" << rightest << std::endl;
                    // Divide Big CLusters into small Parts
                    if (std::abs(leftest - rightest) > 200) {
                        int16_t difference = std::abs(leftest - rightest);
                        double numberOfCluster = difference / 200;
                        for (int j = 0; j < numberOfCluster; ++j) {
                            int16_t leftCluster = leftest;
                            int16_t rightCluster = leftest + 200;
                            result.emplace_back(std::make_pair(leftCluster, rightCluster));
                            leftest = rightCluster;
                            //std::cout << "left" << leftCluster << std::endl;
                            //std::cout << "right" << rightCluster << std::endl;
                        }

                    } else {
                        int16_t leftCluster = leftest;
                        int16_t rightCluster = rightest;
                        //std::cout << "left" << leftCluster << std::endl;
                        //std::cout << "right" << rightCluster << std::endl;
                        result.emplace_back(std::make_pair(leftCluster, rightCluster));
                    }

                }


                //std::cout << "number of cluster" << result.size() << std::endl;

                // Find Cluster's disparity Range
                std::vector<DisparityRange> disparity_clusters;
                int counter = -1;
                for (auto r : result) {
                    ++counter;
                    // std::cout << "cluster " << counter << std::endl;
                    std::vector<Disparity> distance_cl;
                    for (int16_t x = r.first; x < r.second; ++x) {
                        for (int16_t y = yValues.at(i1); y < 1150; ++y) {
                            cv::Vec3b intensity = maskedDisparity.at<cv::Vec3b>(Point(x, y));
                            Disparity disparity = Disparity(intensity.val[0]);
                            if (disparity > d && disparity < dSecond ) {
                                distance_cl.emplace_back(disparity);
                            }
                        }
                    }
                    //std::cout << "number of acceptable disparity" << distance_cl.size() << std::endl;
                    // Warning: Maybe add a margin
                    if (distance_cl.size() != 0) {
                        DisparityRange dRange;
                        std::sort(distance_cl.begin(), distance_cl.end());
                        Disparity average =
                                accumulate(distance_cl.begin(), distance_cl.end(), 0.0) / distance_cl.size();
                        dRange.minDisp = distance_cl.front();
                        dRange.maxDisp = distance_cl.back();
                        //std::cout << "min" << dRange.minDisp << std::endl;
                        //std::cout << "max" << dRange.maxDisp << std::endl;
                        disparity_clusters.push_back(dRange);
                    } else {
                        // if there is no suitable disparity memeber then delete the cluster
                        result.erase(result.begin() + counter);
                    }

                }

                std::cout << result.size() << std::endl;

                //std::vector<Points> pointlerler;
                int counter2 = 0;
                int counter0 = 0;
                for (int m = 0; m < result.size(); ++m) {
                    Points pointler;
                    for (int64_t x = result.at(m).first; x < (result.at(m).second + 1); ++x) {
                        for (int y = 1150; y < maskedDisparity.rows; ++y) {
                            cv::Vec3b intensity = maskedDisparity.at<cv::Vec3b>(Point(x, y));
                            Disparity disparity = Disparity(intensity.val[0]);
                            if (disparity >= (disparity_clusters.at(m).minDisp -3) &&
                                disparity <= (disparity_clusters.at(m).maxDisp + 3 ) ) {
                                counter2 = 0;
                                pointler.emplace_back(Point(x, y));
                            }else{
                                if(counter2 ==2){
                                    break;
                                }
                                if(disparity != 0)
                                    counter2++;
                            }
                        }
                    }
                    pointlerler.emplace_back(pointler);
                }
            }
        }

        cv::Mat image(disp.rows, disp.cols, CV_8UC3, cv::Scalar(0, 0, 0));
       
        //std::cout <<"total point" << pointlerler.size() << std::endl;
        for (int k = 0; k < pointlerler.size(); ++k) {
            Points poiler = pointlerler.at(k);
            for (int j = 0; j < poiler.size(); ++j) {
                Point p = poiler.at(j);
                cv::circle(image, p, 3, cv::Scalar(255, 255, 255), 3, 8);
            }
        }


        cv::Mat element= cv::getStructuringElement(0, cv::Size(20, 20));
        cv::erode(image,image,element);
        cv::dilate(image,image,element);



        cv::Mat copiedImageInv;
        cv::bitwise_not(image, copiedImageInv);


        //std::stringstream ss;
        //ss << "/mrtstorage/users/kirik/lex_map_ost/maskStatic" << "/maskStatic" << std::setw(8) << std::setfill('0') << curIndex << ".png";
        //std::string s = ss.str();
        //cv::imwrite(s, copiedImageInv);

        cv::Mat original = data.getImage(curIndex);

        for (int k = 0; k < pointHoritzon.size(); ++k) {
            Point poiler = pointHoritzon.at(k);
            cv::circle(original, poiler, 3, cv::Scalar(0, 255, 255), 3, 8);
        }


        cv::Mat res;
        cv::Mat origImage = data.getImage(curIndex);
        double alpha = 0.5;
        double beta = 1 - alpha;
        cv::addWeighted(origImage,alpha,copiedImageInv,beta,0.0,res);


         if (res.rows > 1000 || res.cols > 1500)
             cv::resize(res, res, cv::Size(res.cols * 0.35, res.rows * 0.35), 0, 0);

         if (original.rows > 1000 || original.cols > 1500)
             cv::resize(original, original, cv::Size(disp.cols * 0.35, disp.rows * 0.35), 0, 0);
         if (maskedDisparity.rows > 1000 || maskedDisparity.cols > 1500)
             cv::resize(maskedDisparity, maskedDisparity, cv::Size(disp.cols * 0.35, disp.rows * 0.35), 0, 0);
         cv:imshow("o",original);
         //cv::imshow("mask",copiedImageInv);
         cv::imshow("pencere", maskedDisparity);
         cv::imshow("mask",res);



        //std::stringstream ss;
        //ss << "/mrtstorage/users/kirik/lex_map_ost/extras/new_wave2/mask" << "/mask3980" << std::setw(8) << std::setfill('0') << curIndex << ".png";
        //std::string s = ss.str();
        //cv::imwrite(s, original);

        //std::stringstream ss2;
        //ss2 << "/mrtstorage/users/kirik/lex_map_ost/extras/new_wave2/overlayed" << "/overlayed3980" << std::setw(8) << std::setfill('0') << curIndex << ".png";
        //std::string s2 = ss2.str();
        //cv::imwrite(s2, res);


        pressedKey = cv::waitKey(0);
        if (pressedKey == 110) { // n
            if (curIndex == min) {
                curIndex = max;
            } else {
                --curIndex;
            }
        } else {
            if (curIndex == max) {
                curIndex = min;
            } else {
                ++curIndex;
            }
        }



    }




}

*/

/*
TEST(disparity, histogramClusters) {
    using namespace data_generation;
    Params params;
    params.loadParams();

    disparity_handler dh(params.disparityDir);
    cv::Mat image= dh.getDepthImage(140);
    cv::Mat cropedImage;
    ROI roi;
    roi.greatest_y = 1096;
    roi.smallest_y = 1061;
    roi.smallest_x = 2061;
    roi.greatest_x = 2073;
    int16_t h = roi.greatest_y - roi.smallest_y;
    int16_t w = roi.greatest_x - roi.smallest_x;
    image(cv::Rect(roi.smallest_x,roi.smallest_y,w,h)).copyTo(cropedImage);
    cv::Mat labels,centers;


}

*/






/*
TEST(disparity, checkingDisparityMapsFromDistance) {
    using namespace data_generation;
    Params params;
    params.loadParams();
    disparity_handler dh(params.depthMap);
    ROI roiValuesOriginal;
    cv::Mat depthMap = dh.getDepthImage(170);
    roiValuesOriginal.smallest_y = 1049;
    roiValuesOriginal.smallest_x = 1671;
    roiValuesOriginal.greatest_x = 1704;
    roiValuesOriginal.greatest_y = 1139;
    roiValuesOriginal.minDisparity = 19;
    roiValuesOriginal.maxDisparity = 23;
    ROI roiValuesWithMargin = dh.addMarginToROIVAlues(roiValuesOriginal, depthMap.cols, depthMap.rows,
                                                      params.rate_expand);


    //cv::Mat imageThresholdMinMax = dh.thresHoldingTOZeroAndTOZeroInverse(depthMap,roiValuesWithMargin);
    cv::Mat th = dh.thresHoldingTOZeroAndTOZeroInverse(depthMap,roiValuesWithMargin);

    int h = roiValuesOriginal.greatest_y - roiValuesOriginal.smallest_y;
    int w = roiValuesOriginal.greatest_x - roiValuesOriginal.smallest_x;

    cv::Mat TLimage = cv::Mat::zeros(cv::Size(th.cols,th.rows), CV_64FC1);
    //src(Rect(left,top,width, height)).copyTo(dst);
    th(cv::Rect(roiValuesOriginal.smallest_x,roiValuesOriginal.smallest_y,w,h)).copyTo(TLimage);
    cv::Mat greyMat, colorMat;
    cv::cvtColor(TLimage, greyMat, CV_BGR2GRAY);
    cv::Mat greyMatdl = dh.applyDialation(greyMat,2,2);

    cv::Mat labels, stats, centroids;

    auto num_objects = cv::connectedComponentsWithStats(greyMatdl,labels, stats, centroids,8);

    if(num_objects < 2){
        std::cout << "no" << std::endl;
    }else{
        std::cout << "yes "<< num_objects << std::endl;
    }
    for (auto i=0; i< num_objects; i++){
        std::cout << "Objects " << i << " with area : " << stats.at<int>(i,cv::CC_STAT_AREA) << std::endl;

    }

    for (auto i=0; i< num_objects; i++){
        std::cout << "Objects " << i << " with pos : " << centroids.at<cv::Point>(i) << std::endl;

    }

    for (auto i=0; i< num_objects; i++){
        std::cout << "Objects " << i << " lEFT  : " << stats.at<int>(i,cv::CC_STAT_LEFT) << std::endl;
        std::cout << "Objects " << i << " TOP : " << stats.at<int>(i,cv::CC_STAT_TOP) << std::endl;
        std::cout << "Objects " << i << " WIDTH : " << stats.at<int>(i,cv::CC_STAT_WIDTH) << std::endl;
        std::cout << "Objects " << i << " HEIGHT : " << stats.at<int>(i,cv::CC_STAT_HEIGHT) << std::endl;


    }


    if (th.rows > 1000 || th.cols > 1500)
        cv::resize(th, th, cv::Size(th.cols * 0.35, th.rows * 0.35), 0, 0);
    if (TLimage.rows > 1000 || TLimage.cols > 1500)
        cv::resize(TLimage, TLimage, cv::Size(TLimage.cols * 0.35, TLimage.rows * 0.35), 0, 0);

    cv::imshow("b",TLimage);
    cv::imshow("a",th);
    cv::imshow("c",greyMatdl);
    cv::waitKey(0);

}
*/






/*
TEST(disparity, testDisparityHandlerClass) {
    using namespace data_generation;
    Params params;
    params.loadParams();
    disparity_handler dh(params.depthMap);
    const std::string globPath1 = params.depthMap + "/*.png";
    std::vector<cv::String> files;
    cv::glob(globPath1, files);

    for (int j = 0; j < files.size(); ++j) {
        cv::Mat img = dh.getDepthImage(j);

        if (img.rows > 1000 || img.cols > 1500)
            cv::resize(img, img, cv::Size(img.cols * 0.35, img.rows * 0.35), 0, 0);

        cv::imshow("projection", img);
        cv::waitKey(0);
    }

}

*/





/*
TEST(disparity, implementDisparityHandlerInProjector) {
    using namespace Projector;
    Params params;
    params.loadParams();
    DisparityHandler dh(params.depthMap);
    cv::Mat sample = cv::imread("/home/kirik/Desktop/0000004460.png");
    cv::Rect r(558,214,9,45);
    cv::Mat roi(sample(r));

    /*
    cv::HOGDescriptor hog(
            cv::Size(sample.cols,sample.rows),
            cv::Size(20,20),
            cv::Size(10,10),
            cv::Size(10,10),9);
    std::vector<float> descriptors;
    hog.compute(sample,descriptors);


    cv::Mat roi1,roi2;
    roi1 = dh.applyHoughTransform(roi);
    cv::imshow("roi",roi);
    cv::imshow("roi1",roi1);

   // cv::imshow("roi2",roi2);


    cv::waitKey(0);

}
*/
/*
TEST(disparity, implementDisparityHandlerInProjector) {
    using namespace data_generation;
    Params params;
    params.loadParams();
    DisparityHandler disparityHandler(params.depthMap);


    // Algorithm

    cv::Mat sampleImg = disparityHandler.getDepthImage(130);

    ROI roiEdges = {734,244,683,118,200,30};
   // cv::Rect r(558,214,9,45);

    //cv::Mat roi(sampleImg(r));
    cv::Mat sampleImgT = disparityHandler.thresholdingTOZERO(sampleImg,roiEdges.minDisparity);
    cv::Mat sampleImgTT = disparityHandler.thresholdingTOZEROInverse(sampleImgT,roiEdges.maxDisparity);

    //cv::Mat VDis = disparityHandler.getVDisparityROI(sampleImgT, roiEdges);
    //cv::Mat UDis = disparityHandler.getUDisparityROI(sampleImgT, roiEdges);

    cv::Mat VDis = disparityHandler.getVDisparityWholeIMage(sampleImgT);
    cv::Mat UDis = disparityHandler.getUDisparityWholeIMage(sampleImgT);

    cv::Mat VDisp = disparityHandler.applyHoughTransform(VDis);
    cv::Mat UDisp = disparityHandler.applyHoughTransform(UDis);

    cv::Mat DVDisp = disparityHandler.applyDialation(VDisp,2,2);
    cv::Mat DUDisp = disparityHandler.applyDialation(UDisp,2,2);
    //std::vector<cv::Mat> imgs = disparityHandler.getROIvuDisparity(sampleImgTT,roiEdges);

    //cv::imshow("Vdisp",imgs.at(0) );
    //cv::imshow("Udisp",imgs.at(1));
    cv::imshow("pecrere",sampleImg);
    cv::imshow("pencere2", VDis);
    cv::imshow("p1",UDis);
    //cv::imshow("p2",VDispc);
    cv::waitKey(0);

}

*/
/*

TEST(disparity, TestDisparityHandlerClass) {
    using namespace Projector;
    std::string path = "/home/kirik/Desktop/VNL_Depth_Prediction/kitti_VNL";
    DisparityHandler disparityHandler(path);
    cv::Mat sampleImage = disparityHandler.getDepthImage(4400);


    //cv::Mat thresh = disparityHandler.thresholdingTOZEROInverse(sampleImage,50,255);
    //cv::Mat thresh = disparityHandler.thresholdingTOZERO(sampleImage,50,255);
    //cv::Mat U = disparityHandler.getUDisparity(sampleImage);
    cv::Mat V = disparityHandler.getVDisparity(sampleImage);
    //cv::Mat houghU = disparityHandler.applyHoughTransform(U);
    cv::Mat houghV = disparityHandler.applyHoughTransform(V);
    cv::Mat DiaV = disparityHandler.applyDialation(houghV,5,5);
    //cv::imshow("houghU",houghU);
    //cv::imshow("houghV",houghV);
    //cv::imshow("U",U);
    cv::imshow("V",V);
    cv::imshow("DiaV",DiaV);
    cv::imshow("sample",sampleImage);
    //cv::imshow("thresh",thresh);


    cv::waitKey(0);


}

*/



/*
TEST(disparity, HOG) {
    cv::Mat image = cv::imread("/home/kirik/Desktop/0000004490.png");
    cv::HOGDescriptor hog(cv::Size(10, 10), cv::Size(8, 8), cv::Size(2, 2),
            cv::Size(10, 10), 9,1,-1,
            0,0.2,1,64,1);


    std::vector<float> descriptors;
    hog.compute(image,descriptors);

    std::cout << descriptors.size() << std::endl;


    //hog.detect(image,found_locations,0,cv::Size(10,10),cv::Size(0,0));

}
*/


/*
TEST(disparity, objectDetection_Canny_Hough_copy_PartofImage) {
    // Gettting no results by croping interest regions in image and applying
    // hought transform for edge detection
    using namespace Projector;
    cv::Mat image = cv::imread("/home/kirik/Desktop/0000004490-raw.png");

    cv::Mat TLimage = cv::Mat::zeros(cv::Size(image.cols,image.rows), CV_64FC1);
    //src(Rect(left,top,width, height)).copyTo(dst);
    image(cv::Rect(585,195,25,45)).copyTo(TLimage);

    cv::Mat dst,cdst;
    cv::Canny(TLimage,dst,50,200,3);
    cv::cvtColor(dst,cdst, CV_GRAY2BGR);
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(dst,lines,CV_PI/180,100,0,0);

    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
    }
    cv::imshow("part of",TLimage);
    cv::imshow("part of1",dst);
    cv::imshow("part of2",cdst);


    cv::waitKey(0);


}
*/



/*
TEST(disparity, DepthMapPreProcessing) {
    cv::Mat image = cv::imread("/home/kirik/Desktop/disp00004003.png");
    cv::Mat sharpened;
    cv::Mat computeVDisparity;
    cv::Mat computeUDisparity;
    int maxDisp = 256;

    //double sigma = 1, threshold = 5, amount = 1;
    //cv::GaussianBlur(image, BlurImage, cv::Size(5,5), sigma, sigma);
    //cv::Mat lowContrastMask = abs(image - BlurImage) < threshold;
    //cv::Mat sharpened = image*(1+amount) + BlurImage*(-amount);
    //image.copyTo(sharpened, lowContrastMask);

    //cv::GaussianBlur(image,BlurImage,cv::Size(5,5),0,0);
    //cv::medianBlur(image,BlurImage,5);
    //TO increase the contrast
    //image.convertTo(sharpened,-1,2,0);
    //TO increase the contrast
    //Mean value filter
    //cv::boxFilter(image,sharpened,-1,cv::Size(5,5));
    //cv::Mat element = cv::getStructuringElement(0,cv::Size(5,5));
    //cv::dilate(image,sharpened,element);

    cv::threshold(image,sharpened,100,93,cv::THRESH_TOZERO_INV);
    cv::threshold(sharpened,sharpened,30,93,cv::THRESH_TOZERO);



    cv::Mat uDisp(maxDisp, sharpened.cols, CV_8U, cv::Scalar(0));
    for (int u = 160; u< 260; u++) {
        for (int v = 620; v < 670; v++) {
            cv::Vec3b intensity = sharpened.at<cv::Vec3b>(cv::Point(v, u));
            float blue = intensity[0];
            uchar blue2 = intensity.val[0];
            if (blue != 0)
                //if (blue <= 90 && blue >= 70)
            {
                uDisp.at<uchar>(blue2, v) += 30;
            }
            //else{
            //    uDisp.at<uchar>(1, v) += 30;
            //}
        }
    }
    cv::Mat vDisp(sharpened.rows, maxDisp, CV_8U, cv::Scalar(0));


    for (int u = 160; u<260; u++) {
        for (int v = 620; v < 670; v++) {
            cv::Vec3b intensity = sharpened.at<cv::Vec3b>(cv::Point(v, u));
            float blue = intensity[0];
            uchar blue2 = intensity.val[0];
            if(blue != 0){
                vDisp.at<uchar>(u, blue2) += 30;
            }
        }
    }
    cv::Mat dst,cdst;
    cv::Canny(vDisp,dst,50,200,3);
    cv::cvtColor(dst,cdst, CV_GRAY2BGR);
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(dst,lines,CV_PI/180,100,0,0);




    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
    }

    cv::Mat dst1,cdst1;
    cv::Canny(uDisp,dst1,50,200,3);
    cv::cvtColor(dst1,cdst1, CV_GRAY2BGR);
    std::vector<cv::Vec2f> lines1;
    cv::HoughLines(dst1,lines1,CV_PI/180,100,0,0);

    for( size_t i = 0; i < lines1.size(); i++ )
    {
        float rho = lines1[i][0], theta = lines1[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst1, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
    }









    //cv::GaussianBlur(image,BlurImage,cv::Size(5,5),0,0);
    //cv::medianBlur(image,BlurImage,5);
    cv::imshow("Image",image);
    cv::imshow("sharpened",sharpened);
    cv::imshow("uDisp",uDisp);
    cv::imshow("vDisp",vDisp);
    cv::imshow("cdst",cdst);
    cv::imshow("cdst1",cdst1);
    cv::waitKey(0);

}
*/

/*
TEST(disparity, V_U_disparityManualVersion) {
    cv::Mat image = cv::imread("/home/kirik/Desktop/disp00004003.png");
    cv::Mat computeVDisparity;
    cv::Mat computeUDisparity;
    int maxDisp = 256;

    cv::Mat ThresHOLDINGTOP,ThresHOLDING;
    cv::threshold(image,ThresHOLDINGTOP,90,93,cv::THRESH_TOZERO_INV);
    //cv::imshow("THRESHOLDING1",ThresHOLDINGTOP);
    cv::threshold(ThresHOLDINGTOP,ThresHOLDING,70,93,cv::THRESH_TOZERO);



    cv::Mat uDisp(maxDisp, ThresHOLDING.cols, CV_8U, cv::Scalar(0));
    for (int u = 580; u < 620; u++){
        int counterH = 0;
        int counterZ = 0;
        for (int v = 200; v < 260; v++){
            cv::Vec3b intensity = ThresHOLDING.at<cv::Vec3b>(cv::Point(u, v));
            //cv::Vec3b intensity = ThresHOLDING.at<cv::Vec3b>(v,u);
            float blue = intensity[0];
            //uchar blue2 = intensity.val[0];
            if (blue != 0)
            {counterH++;}
            else{counterZ++;}
        }

        if(counterH > counterZ) {
            uDisp.at<uchar>(100, u) = (counterH + counterZ);
        }else{
            uDisp.at<uchar>(100, u) = 0;
        }
    }
    cv::applyColorMap(uDisp, uDisp, cv::COLORMAP_JET);



    cv::Mat vDisp(image.rows, maxDisp, CV_8U, cv::Scalar(0));
    for (int v = 200; v< 240; v++) {
        int counterH = 0;
        int counterZ = 0;
        for (int u = 580; u<620; u++) {
            cv::Vec3b intensity = ThresHOLDING.at<cv::Vec3b>(cv::Point(u, v));
            float blue = intensity[0];
            uchar blue2 = intensity.val[0];
            if (blue != 0)
            {
                counterH++;
            }else{
                counterZ++;
            }
        }
        if(counterH > counterZ) {
            vDisp.at<uchar>(v, 100) = (counterH + counterZ);
        }else{
            vDisp.at<uchar>(v, 100) = 0;
        }
    }
    cv::applyColorMap(vDisp, vDisp, cv::COLORMAP_JET);



    cv::Mat dst,cdst;
    cv::Canny(vDisp,dst,50,200,3);
    cv::cvtColor(dst,cdst, CV_GRAY2BGR);
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(dst,lines,CV_PI/180,100,0,0);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
    }


    cv::Mat dst1,cdst1;
    cv::Canny(uDisp,dst1,50,200,3);
    cv::cvtColor(dst1,cdst1, CV_GRAY2BGR);
    std::vector<cv::Vec2f> lines1;
    cv::HoughLines(dst1,lines1,CV_PI/180,100,0,0);

    for( size_t i = 0; i < lines1.size(); i++ )
    {
        float rho = lines1[i][0], theta = lines1[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst1, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
    }

    // Dilation
    //cv::Mat dilationImageV,dilationImageU;
    //cv::Mat element = cv::getStructuringElement(0,cv::Size(2,2));
    //cv::dilate(cdst,dilationImageV,element);
    //cv::dilate(cdst1,dilationImageU,element);
    // Dilation
    /*
    std::vector<float> res;
    for (int j = 580; j < 620; ++j) {

        cv::Vec3b intensity = dilationImageU.at<cv::Vec3b>(cv::Point(j, 100));
        float a = intensity[0];
        if(a != 0){
            res.emplace_back(a);
        }
    }



    if (image.rows > 1000 || image.cols > 1500)
        cv::resize(image, image, cv::Size(image.cols * 0.35, image.rows * 0.35), 0, 0);

    if (vDisp.rows > 1000 || vDisp.cols > 1500)
        cv::resize(vDisp, vDisp, cv::Size(vDisp.cols * 0.35, vDisp.rows * 0.35), 0, 0);
    if (uDisp.rows > 1000 || uDisp.cols > 1500)
        cv::resize(uDisp, uDisp, cv::Size(uDisp.cols * 0.35, uDisp.rows * 0.35), 0, 0);


    cv::imshow("pen3",image);
    //cv::imshow("HOughTRansform1",cdst1);
    //cv::imshow("THRESHOLDING2",ThresHOLDING);
    //cv::imshow("HOughTRansform2",cdst);
    cv::imshow("VDISP",vDisp);
    cv::imshow("UDISP",uDisp);
    //cv::imshow("Dilatation",dilationImageV);
    //cv::imshow("Dilatation2",dilationImageU);
    cv::waitKey(0);
}
*/

/*
TEST(disparity, U_V_disparityForPresentation) {
    cv::Mat image = cv::imread( "/home/kirik/Desktop/disparity_maps/vnl_Color_image.png");
    //cvtColor(image, image, cv::COLOR_RGB2GRAY);



    cv::Mat res = cv::Mat::zeros(cv::Size(image.cols,image.rows), CV_64FC1);

    // Script
    //int16_t h = 536;
    //int16_t w = 4096;
    //image(cv::Rect(0,1000,w,h)).copyTo(res);


    // VNL
    int16_t h = 200;
    int16_t w = 180;
    image(cv::Rect(920,200,w,h)).copyTo(res);

    //image_pipeline
    //int16_t h = 300;
    //int16_t w = 300;
    //image(cv::Rect(850,150,w,h)).copyTo(res);


    unsigned int IMAGE_HEIGHT = res.rows;
    unsigned int IMAGE_WIDTH = res.cols;
    unsigned int MAX_DISP = 255;
    unsigned int CYCLE = 0;

    cv::Mat uhist = cv::Mat::zeros(MAX_DISP, IMAGE_WIDTH, CV_32F);
    cv::Mat vhist = cv::Mat::zeros(IMAGE_HEIGHT, MAX_DISP, CV_32F);
    cv::Mat tmpImageMat, tmpHistMat;
    float value_ranges[] = {(float) 0, (float) 255};
    const float *hist_ranges[] = {value_ranges};
    int channels[] = {0};
    int histSize[] = {(int) MAX_DISP};


    for (int i = 0; i < IMAGE_HEIGHT; i++) {
        //V-disparity space is a row based matrix
        tmpImageMat = res.row(i);
        vhist.row(i).copyTo(tmpHistMat);
        cv::calcHist(&tmpImageMat, 1, channels, cv::Mat(), tmpHistMat, 1, histSize, hist_ranges, true, false);
        vhist.row(i) =  tmpHistMat.t() / (float) IMAGE_HEIGHT;

    }

    for (int i = 0  ; i < IMAGE_WIDTH; i++) {
        //U-disparity space is a column based
        tmpImageMat = res.col(i);
        uhist.col(i).copyTo(tmpHistMat);
        cv::calcHist(&tmpImageMat, 1, channels, cv::Mat(), tmpHistMat, 1, histSize, hist_ranges, true, false);
        // Multiply by 3 so lines could be seen more easy on uhist
        // scale it depending on its distance to camera. Divide in 4 different parts

        uhist.col(i) =  tmpHistMat / (float) IMAGE_WIDTH;

    }

    uhist.convertTo(uhist, CV_8U, 255);
    cv::applyColorMap(uhist, uhist, cv::COLORMAP_JET);

    vhist.convertTo(vhist, CV_8U, 255);
    cv::applyColorMap(vhist, vhist, cv::COLORMAP_JET);








    cv::Mat dst,cdst;
    cv::Canny(vhist,dst,50,200,3);
    cv::cvtColor(dst,cdst, CV_GRAY2BGR);
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(dst,lines,CV_PI/180,100,0,0);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
    }


    cv::Mat dst1,cdst1;
    cv::Canny(uhist,dst1,50,200,3);
    cv::cvtColor(dst1,cdst1, CV_GRAY2BGR);
    std::vector<cv::Vec2f> lines1;
    cv::HoughLines(dst1,lines1,CV_PI/180,100,0,0);

    for( size_t i = 0; i < lines1.size(); i++ )
    {
        float rho = lines1[i][0], theta = lines1[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst1, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
    }









    if (image.rows > 1000 || image.cols > 1500)
        cv::resize(image, image, cv::Size(image.cols * 0.35, image.rows * 0.35), 0, 0);

    if (uhist.rows > 1000 || uhist.cols > 1500)
        cv::resize(uhist, uhist, cv::Size(uhist.cols * 0.35, uhist.rows * 0.35), 0, 0);
    if (vhist.rows > 1000 || vhist.cols > 1500)
        cv::resize(vhist, vhist, cv::Size(vhist.cols * 0.35, vhist.rows * 0.35), 0, 0);
    if (cdst1.rows > 1000 || cdst1.cols > 1500)
        cv::resize(cdst1, cdst1, cv::Size(cdst1.cols * 0.35, cdst1.rows * 0.35), 0, 0);
    if (cdst.rows > 1000 || cdst.cols > 1500)
        cv::resize(cdst, cdst, cv::Size(cdst.cols * 0.35, cdst.rows * 0.35), 0, 0);
    if (res.rows > 1000 || res.cols > 1500)
        cv::resize(res, res, cv::Size(res.cols * 0.35, res.rows * 0.35), 0, 0);



    cv::imwrite( "/home/kirik/Desktop/disparity_maps/vnl_results/image.png", image );
    cv::imwrite( "/home/kirik/Desktop/disparity_maps/vnl_results/vhist.png", vhist );
    cv::imwrite( "/home/kirik/Desktop/disparity_maps/vnl_results/uhist.png", uhist );
    cv::imwrite( "/home/kirik/Desktop/disparity_maps/vnl_results/cdst1.png", cdst1 );
    cv::imwrite( "/home/kirik/Desktop/disparity_maps/vnl_results/cdst.png", cdst );
    cv::imwrite( "/home/kirik/Desktop/disparity_maps/vnl_results/cdst.png", cdst );
    cv::imwrite( "/home/kirik/Desktop/disparity_maps/vnl_results/res.png", res );



    cv::imshow("image", image);
    cv::imshow("uhist", uhist);
    cv::imshow("vhist", vhist);
    cv::imshow("vhough", cdst1);
    cv::imshow("uhough", cdst);
    cv::imshow("crop", res);
    cv::waitKey(0);


}
 */
/*
TEST(disparity, U_V_disparityOpencvVersion) {

    cv::Mat image = cv::imread( "/home/kirik/Desktop/disp00004003.png");
    unsigned int IMAGE_HEIGHT = image.rows;
    unsigned int IMAGE_WIDTH = image.cols;
    unsigned int MAX_DISP = 255;
    unsigned int CYCLE = 0;

    cv::Mat uhist = cv::Mat::zeros(MAX_DISP, IMAGE_WIDTH, CV_32F);
    cv::Mat vhist = cv::Mat::zeros(IMAGE_HEIGHT, MAX_DISP, CV_32F);
    cv::Mat tmpImageMat, tmpHistMat;
    float value_ranges[] = {(float) 0, (float) 255};
    const float *hist_ranges[] = {value_ranges};
    int channels[] = {0};
    int histSize[] = {(int) MAX_DISP};

    cv::Mat ThresHOLDING;
    cv::threshold(image,ThresHOLDING,50,93,cv::THRESH_TOZERO_INV);
    cv::threshold(ThresHOLDING,ThresHOLDING,35,93,cv::THRESH_TOZERO);
    cv::imshow("THRESHOLDING",ThresHOLDING);

    for (int i = 0; i < 1536; i++) {
           //V-disparity space is a row based matrix
        tmpImageMat = ThresHOLDING.row(i);
        vhist.row(i).copyTo(tmpHistMat);
        cv::calcHist(&tmpImageMat, 1, channels, cv::Mat(), tmpHistMat, 1, histSize, hist_ranges, true, false);
        vhist.row(i) = tmpHistMat.t() / (float) IMAGE_HEIGHT;

    }


    for (int i = 0  ; i < 4096; i++) {
        //U-disparity space is a column based
        tmpImageMat = ThresHOLDING.col(i);
        uhist.col(i).copyTo(tmpHistMat);
        cv::calcHist(&tmpImageMat, 1, channels, cv::Mat(), tmpHistMat, 1, histSize, hist_ranges, true, false);
        // Multiply by 3 so lines could be seen more easy on uhist
        // scale it depending on its distance to camera. Divide in 4 different parts
        uhist.col(i) = (5*tmpHistMat) / (float) IMAGE_WIDTH;

    }

    vhist.convertTo(vhist, CV_8U, 255);
    cv::applyColorMap(vhist, vhist, cv::COLORMAP_JET);
    uhist.convertTo(uhist, CV_8U, 255);
    cv::applyColorMap(uhist, uhist, cv::COLORMAP_JET);

    

    
    

    // max and min of a row with their location
    //Might be Useful
    //double min,max;
    //cv::Point min_loC,max_LOc;
    //cv::minMaxLoc(vhist.row(200), &min, &max);
    //cv::minMaxLoc(image, &min, &max, &min_loC, &max_LOc);
    //std::cout << "min"  << min << "max " << max << "minlo" << min_loC << "maxloc "  << max_LOc << std::endl;



    cv::Mat dst,cdst;
    cv::Canny(vhist,dst,50,200,3);
    cv::cvtColor(dst,cdst, CV_GRAY2BGR);
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(dst,lines,CV_PI/180,100,0,0);

    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
    }

    cv::Mat dst1,cdst1;
    cv::Canny(uhist,dst1,50,200,3);

    cv::cvtColor(dst1,cdst1, CV_GRAY2BGR);
    std::vector<cv::Vec2f> lines1;
    cv::HoughLines(dst1,lines1,CV_PI/180,100,0,0);

    // thresholding
    //cv::Mat dst1thres;
    //cv::threshold(image,dst1thres,68,93,0);
    //cv::imshow("ghe",dst1thres);
    //thresholding

    for( size_t i = 0; i < lines1.size(); i++ )
    {
        float rho = lines1[i][0], theta = lines1[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst1, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
    }

    // thresholding
    //cv::Mat ThresHOLDING;
    //cv::threshold(cdst1,ThresHOLDING,93,93,cv::THRESH_TOZERO_INV);
    //cv::threshold(cdst1,ThresHOLDING,68,93,cv::THRESH_TOZERO);
    //cv::imshow("THRESHOLDING",ThresHOLDING);
    //thresholding


// Dilation
    cv::Mat dilationImageV,dilationImageU;
    cv::Mat element = cv::getStructuringElement(0,cv::Size(3,3));
    cv::dilate(cdst,dilationImageV,element);
    cv::dilate(cdst1,dilationImageU,element);
    cv::imshow("Dilatation",dilationImageV);
    cv::imshow("Dilatation2",dilationImageU);
    // Dilation




    // trying to get the object shape;
    //cv::Mat gx,gy,grad;
    //cv::Mat abs_x,abs_y;
    //cv::Sobel(vhist,gx,CV_32F,1,0,1);
    //cv::Sobel(vhist, gy, CV_32F, 0, 1, 1);
    //addWeighted( abs_x, 0.5, abs_y, 0.5, 0, grad );
    //cv::Mat mag,angle;
    //cv::cartToPolar(gx,gy,mag,angle,1);
    // trying to get the object shape;

    cv::imshow("part of",image);
    cv::imshow("part of1",cdst1);
    cv::imshow("part of2",cdst);
    cv::imshow("vhist", vhist);
    cv::imshow("uhist", uhist);

    //cv::imwrite( "/home/kirik/Desktop/papers_VU_Disparity/VU-DisparityResults/focusTL/image.png", image );
    //cv::imwrite( "/home/kirik/Desktop/papers_VU_Disparity/VU-DisparityResults/focusTL/vhist.png", vhist );
    //cv::imwrite( "/home/kirik/Desktop/papers_VU_Disparity/VU-DisparityResults/focusTL/uhist.png", uhist );
    //cv::imwrite( "/home/kirik/Desktop/papers_VU_Disparity/VU-DisparityResults/focusTL/cdst1.png", cdst1 );
    //cv::imwrite( "/home/kirik/Desktop/papers_VU_Disparity/VU-DisparityResults/focusTL/cdst.png", cdst );
    //cv::imwrite( "/home/kirik/Desktop/papers_VU_Disparity/VU-DisparityResults/focusTL/cdst.png", cdst );
    //cv::imwrite( "/home/kirik/Desktop/papers_VU_Disparity/VU-DisparityResults/focusTL/thres.png", ThresHOLDING );
    //cv::imwrite( "/home/kirik/Desktop/papers_VU_Disparity/VU-DisparityResults/focusTL/dilationImageV.png", dilationImageV );
    //cv::imwrite( "/home/kirik/Desktop/papers_VU_Disparity/VU-DisparityResults/focusTL/dilationImageU.png", dilationImageU );



    cv::waitKey(0);

}

*/

/*
TEST(disparity, v_disparity) {
    using namespace Projector;
    Params params;
    loadParams(params);
    cv::Mat image = cv::imread("/home/kirik/Desktop/sample.jpeg");
    std::cout << image.size << std::endl;
    std::cout << image.cols << std::endl;
    unsigned int IMAGE_HEIGHT = image.rows;
    unsigned int IMAGE_WIDTH = image.cols;
    unsigned int MAX_DISP = 255;
    unsigned int CYCLE = 0;

    cv::Mat uhist = cv::Mat::zeros(MAX_DISP, IMAGE_WIDTH, CV_32F);
    cv::Mat vhist = cv::Mat::zeros(IMAGE_HEIGHT, MAX_DISP, CV_32F);
    cv::Mat tmpImageMat, tmpHistMat;
    float value_ranges[] = {(float) 0, (float) MAX_DISP};
    const float *hist_ranges[] = {value_ranges};
    int channels[] = {0};
    int histSize[] = {(int) MAX_DISP};


    for (int i = 0; i < IMAGE_HEIGHT; i++) {
        tmpImageMat = image.row(i);

        vhist.row(i).copyTo(tmpHistMat);
        //std::cout << tmpHistMat.size << std::endl;

        cv::calcHist(&tmpImageMat, 1, channels, cv::Mat(), tmpHistMat, 1, histSize, hist_ranges, true, false);

        vhist.row(i) = tmpHistMat.t() / (float) IMAGE_HEIGHT;
    }


    for (int i = 0; i < IMAGE_WIDTH; i++) {
        tmpImageMat = image.col(i);
        uhist.col(i).copyTo(tmpHistMat);

        cv::calcHist(&tmpImageMat, 1, channels, cv::Mat(), tmpHistMat, 1, histSize, hist_ranges, true, false);

        uhist.col(i) = tmpHistMat / (float) IMAGE_WIDTH;
    }
    vhist.convertTo(vhist, CV_8U, 255);
    cv::applyColorMap(vhist, vhist, cv::COLORMAP_JET);
    uhist.convertTo(uhist, CV_8U, 255);
    cv::applyColorMap(uhist, uhist, cv::COLORMAP_JET);

    cv::imshow("image", image);
    cv::imshow("vhist", vhist);
    cv::imshow("uhist", uhist);
    cv::waitKey(0);


}
*/

