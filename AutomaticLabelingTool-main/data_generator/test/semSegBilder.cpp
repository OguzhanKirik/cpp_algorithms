// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
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

#include <boost/filesystem.hpp>



TEST(semSegBilder, createDynamicClass) {
    using namespace data_generation;
    Params params;
    params.loadParams();

    camera_info camera(params);
    data_prep data(params);
    cv::Mat disparityMap =cv::imread("/home/kirik/Desktop/disp00004003.png");
    cv::Mat res;
    cv::Mat binaryMask = data.getBinaryMask(4003);
    cv::Mat element = cv::getStructuringElement(0, cv::Size(10, 10));
    cv::erode(binaryMask,binaryMask,element);
    cv::dilate(binaryMask,binaryMask,element);


    disparityMap.copyTo(res,binaryMask);

    //cv::Mat copiedImageInv;
    //cv::bitwise_not(mask,copiedImageInv);

    //cv::imshow("a",res);
    //cv::waitKey(0);
    std::stringstream ss;
    ss << "/home/kirik/Desktop//maskedDisparity" << std::setw(8) << std::setfill('0') << 4003 << ".png";
    std::string s = ss.str();
    cv::imwrite(s, res);

}





/*
TEST(semSegBilder, ThesisExampleTrafficLight) {
    using namespace data_generation;
    cv::Mat MaskedThresholdedDisparityMap =cv::imread("/home/kirik/Desktop/örnekSem.png");
    cv::Mat MaskedThresholdedDisparityMap2 = MaskedThresholdedDisparityMap.clone();
    int count = 0;
    for(int y=0;y<MaskedThresholdedDisparityMap.rows;y++){
        for(int x=0;x<MaskedThresholdedDisparityMap.cols;x++){
            // get pixel
            cv::Vec3b & color = MaskedThresholdedDisparityMap2.at<cv::Vec3b>(y,x);

            if(color[0] == 0){
                if(x >10 && x <= 12  && y < 40 ){
                    color[0] = 36;
                    color[1] = 36;
                    color[2] = 36;

                }
                if(x >=12 && x <=17  && y <  42){
                    color[0] = 36;
                    color[1] = 36;
                    color[2] = 36;

                }
                if(x >=18 && x <=20  && y < 43 ){
                    color[0] = 36;
                    color[1] = 36;
                    color[2] = 36;

                }



            }
        }
    }


    cv::imshow("a",MaskedThresholdedDisparityMap2);
    cv::imshow("b",MaskedThresholdedDisparityMap);
    cv::imwrite("/home/kirik/Desktop/maskedDisparity.png",MaskedThresholdedDisparityMap2);
    cv::waitKey(0);

}
 */
/*
TEST(semSegBilder, disparityProcess){
    using namespace data_generation;
    Params params;
    params.loadParams();

    cv::Mat image = cv::imread("/home/kirik/Desktop/disparityProcess/mask_00000190.png");
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

    cv::Mat disp = cv::imread("/home/kirik/Desktop/disparityProcess/Disp00000190.png");
    cv::Mat maskedDisparitytoadd;
    disp.copyTo(maskedDisparitytoadd, copiedImageInv);

    cv::imwrite("/home/kirik/Desktop/disparityProcess/dispColorProcesses.png",maskedDisparitytoadd);
    cv::imwrite("/home/kirik/Desktop/disparityProcess/maskProcesses_00000190.png",copiedImageInv);

}
 */

/*
TEST(semSegBilder, createDynamicClass) {
    using namespace data_generation;
    Params params;
    params.loadParams();

    camera_info camera(params);
    data_prep data(params);
    cv::Mat mask = data.getBinaryMask(215);
    //cv::Mat maskAdd = mask.clone();
    //cv::Mat image = data.getImage(0);
    cv::Mat res = mask.clone();
    //image.copyTo(res,mask);

    //cv::Mat copiedImageInv;
    //cv::bitwise_not(mask, copiedImageInv);
    for(int y=0;y<mask.rows;y++)
    {
        for(int x=0;x<mask.cols;x++)
        {
            // get pixel
            cv::Vec3b & color = res.at<cv::Vec3b>(y,x);
            if(color[0] == 0){
                color[0] = 0;
                color[1] = 0;
                color[2] = 142;
            }else{
                color[0] = 0;
                color[1] = 0;
                color[2] = 0;
            }
        }
    }


    cv::imwrite("/home/kirik/Desktop/maskDyn215.png",res);
}
 */
/*
TEST(semSegBilder, createDynamicClass) {
    using namespace data_generation;
    Params params;
    params.loadParams();

    camera_info camera(params);
    data_prep data(params);
    cv::Mat mask = data.getBinaryMask(0);
    cv::Mat maskAdd = mask.clone();
    cv::Mat image = data.getImage(0);
    cv::Mat res;
    image.copyTo(res,mask);

    cv::Mat copiedImageInv;
    cv::bitwise_not(mask, copiedImageInv);
    for(int y=0;y<maskAdd.rows;y++)
    {
        for(int x=0;x<maskAdd.cols;x++)
        {
            // get pixel
            cv::Vec3b & color = maskAdd.at<cv::Vec3b>(y,x);
            if(color[0] == 0){
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
    cv::Mat resdyn;
    resdyn = res + maskAdd;

    if (maskAdd.rows > 1000 || maskAdd.cols > 1500)
        cv::resize(maskAdd, maskAdd, cv::Size(maskAdd.cols * 0.35, maskAdd.rows * 0.35), 0, 0);
    if (mask.rows > 1000 || mask.cols > 1500)
        cv::resize(mask, mask, cv::Size(mask.cols * 0.35, mask.rows * 0.35), 0, 0);
    if (resdyn.rows > 1000 || resdyn.cols > 1500)
        cv::resize(resdyn, resdyn, cv::Size(resdyn.cols * 0.35, resdyn.rows * 0.35), 0, 0);

    cv::imshow("a", mask);
    cv::imshow("b", maskAdd);
    cv::imshow("c", resdyn);
    cv::waitKey(0);
}
*/
/*
TEST(semSegBilder, readLabekl) {
    cv::Mat label = cv::imread("/mrtstorage/users/kirik/lex_map_01/extras/resized_labels/labelId00000030.png");
    cv::Mat original = cv::imread("/mrtstorage/users/kirik/lex_map_01/extras/cropped_labels/labelId00000030.png");
    cv::resize(original, original, cv::Size(original.cols * 0.35, original.rows * 0.35), 0, 0);
    cv::imshow("a", original);
    cv::imshow("b", label);
    cv::waitKey(0);
}

*/
/*
TEST(semSegBilder, applyMaskingandCreateHistogram) {
    cv::Mat mask = cv::imread("/mrtstorage/users/kirik/sim_image_ext/gray_right_sem/binary_mask_dynamic_preproc/mask00000262.png");

    cv::Mat output, maskInv;

    cv::bitwise_not(mask, maskInv);
    cv::imshow("b", maskInv);
    cv::waitKey(0);
}
*/
/*
TEST(semSegBilder, GetContourFromBinaryImage) {
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(0,0, 0));
    if (image.rows > 1000 || image.cols > 1500)
        cv::resize(image, image, cv::Size(image.cols * 0.35, image.rows * 0.35), 0, 0);
    cv::imshow("black",image);
    cv::waitKey(0);

}
 */
/*
TEST(semSegBilder, GetContourFromBinaryImage) {
    using namespace data_generation;
    Params params;
    params.loadParams();

    camera_info camera(params);
    data_prep data(params);

    disparity_handler disparityHandler;
    cloud_handler cloudHandler;

    data_generator dataGenerator(camera,data,disparityHandler,cloudHandler,params);
    cv::Mat maskedDisparityMap;
    cv::Mat binaryMask = data.getBinaryMask(546);





    Countours  countours;
    ROI roi;
    roi.greatest_y = 1000;
    roi.smallest_y = 700;
    roi.smallest_x = 3000;
    roi.greatest_x = 3200;

    cv::Mat cropedImage = cv::Mat::zeros(cv::Size(binaryMask.cols,binaryMask.rows), CV_64FC1);

    int16_t h = roi.greatest_y - roi.smallest_y;
    int16_t w = roi.greatest_x - roi.smallest_x;
    cv::Mat grayMasked;
    // conver to  gray image otherwise not work with FindContour
    cv::cvtColor(binaryMask,grayMasked,cv::COLOR_BGR2GRAY);
    grayMasked(cv::Rect(roi.smallest_x,roi.smallest_y,w,h)).copyTo(cropedImage);


    cv::findContours(cropedImage,countours,cv::noArray(),cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE,cv::Point(roi.smallest_x,roi.smallest_y));
    std::cout <<"countours size "  << countours.size() << std::endl;
    cv::Mat output = cropedImage.clone();


    if(countours.size() > 0) {
        size_t ext =  (countours.size() - 1);
        cv::fillConvexPoly(grayMasked,countours.back(),cv::Scalar(110, 220, 0), 8, 0);

    }


    if (grayMasked.rows > 1000 || grayMasked.cols > 1500)
        cv::resize(grayMasked, grayMasked, cv::Size(grayMasked.cols * 0.35, grayMasked.rows * 0.35), 0, 0);

    cv::imshow("real image",grayMasked);
    cv::imshow("outout",output);


    cv::waitKey(0);
}
 */
/*
TEST(semSegBilder, MaskDisparityMap) {
    using namespace data_generation;
    Params params;
    params.loadParams();

    camera_info camera(params);
    data_prep data(params);

    disparity_handler disparityHandler;
    cloud_handler cloudHandler;

    data_generator dataGenerator(camera,data,disparityHandler,cloudHandler,params);
    cv::Mat maskedDisparityMap;
    cv::Mat binaryMask = data.getBinaryMask(546);
    cv::Mat disparityMap = data.getDisparityMap(557);


    disparityMap.copyTo(maskedDisparityMap,binaryMask);
    if (maskedDisparityMap.rows > 1000 || maskedDisparityMap.cols > 1500)
        cv::resize(maskedDisparityMap, maskedDisparityMap, cv::Size(maskedDisparityMap.cols * 0.35, maskedDisparityMap.rows * 0.35), 0, 0);
    if (binaryMask.rows > 1000 || binaryMask.cols > 1500)
        cv::resize(binaryMask, binaryMask, cv::Size(binaryMask.cols * 0.35, binaryMask.rows * 0.35), 0, 0);
    if (disparityMap.rows > 1000 || disparityMap.cols > 1500)
        cv::resize(disparityMap, disparityMap, cv::Size(disparityMap.cols * 0.35, disparityMap.rows * 0.35), 0, 0);
    cv::imshow("window",cropedImage);
    cv::imshow("d",disparityMap);
    cv::imshow("b",binaryMask);
    cv::waitKey(0);
}
 */
/*
TEST(semSegBilder, applyMaskingandCreateHistogram) {
    //cv::Mat disparityMap = cv::imread("/mrtstorage/users/kirik/sim_image_ext/stereoImages/disp00000256.png");
    cv::Mat mask = cv::imread("/mrtstorage/users/kirik/sim_image_ext/gray_right_sem/binary_mask_dynamic_preproc/mask00000262.png");

    cv::Mat output,maskInv;

    cv::bitwise_not(mask,maskInv);
    //cv::Mat maskDInv = maskInv / 255;
    cv::imshow("b",maskInv);
    cv::waitKey(0);

    //disparityMap.copyTo(output,maskDInv);
    /*
    if (disparityMap.rows > 1000 || disparityMap.cols > 1500)
        cv::resize(disparityMap, disparityMap, cv::Size(disparityMap.cols * 0.35, disparityMap.rows * 0.35), 0, 0);
    if (output.rows > 1000 || output.cols > 1500)
        cv::resize(output, output, cv::Size(output.cols * 0.35, output.rows * 0.35), 0, 0);

    cv::imshow("original",disparityMap);
    cv::imshow("masked",output );
    cv::waitKey(0);

    cv::Mat cropedImage = cv::Mat::zeros(cv::Size(output.cols,output.rows), CV_64FC1);
    cv::Scalar mean,stddev;
    cv::meanStdDev(cropedImage,mean,stddev,cv::Mat());


    std::cout << mean << stddev << std::endl;

   // cv::calcHist(&output , 1, channels, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );


}
*/
/*
TEST(semSegBilder, DivisionandInverse) {
    cv::Mat image = cv::imread("/home/kirik/Desktop/binaryMask/vehicle_00000036.png");
    cv::Mat copiedImageInv, conv;
    copiedImageInv = image / 255;
    //cv::bitwise_not(image,copiedImageInv);


    if (copiedImageInv.rows > 1000 || copiedImageInv.cols > 1500)
        cv::resize(copiedImageInv, copiedImageInv, cv::Size(copiedImageInv.cols * 0.35, copiedImageInv.rows * 0.35), 0, 0);
    cv::imshow("window",copiedImageInv);
    cv::waitKey(0);
}


*/

/*
TEST(semSegBilder, implementSemSegImg) {
    std::string binaryMaskFile = "/home/kirik/Desktop/binaryMask";
    const std::string globPathBinaryMask = binaryMaskFile + "/*.png";
    std::vector<cv::String> filesBinaryMask;
    cv::glob(globPathBinaryMask, filesBinaryMask);


    std::string path = binaryMaskFile + "/binaryMaskPreproc";
    boost::filesystem::path dir(path);



    if(boost::filesystem::is_directory(path)) {
        std::cout << boost::filesystem::is_directory(path) << std::endl;
        std::cout << "BinaryMaskPreprocFile already exists " << "\n";


    }else {
        boost::filesystem::create_directory(dir);
        std::cout << "Successfully created BinaryMaskPreprocFile"<< std::endl;

        for (int p = 0; p < filesBinaryMask.size(); ++p) {

            cv::Mat image = cv::imread(filesBinaryMask[p]);
            cv::Mat copiedImage = image.clone();
            cv::Mat greyCopiedImage, labels, stats, centroids;
            cv::cvtColor(copiedImage, greyCopiedImage, CV_BGR2GRAY);

            auto num_objects = cv::connectedComponentsWithStats(greyCopiedImage, labels, stats, centroids, 8);
            for (int j = 1; j < num_objects; ++j) {
                int16_t leftX = stats.at<int>(j, cv::CC_STAT_LEFT);
                int16_t height = stats.at<int>(j, cv::CC_STAT_HEIGHT);
                int16_t width = stats.at<int>(j, cv::CC_STAT_WIDTH);
                int16_t topY = stats.at<int>(j, cv::CC_STAT_TOP);
                int16_t lowestY = topY + height;

                if (lowestY < (image.rows / 2)) {
                    for (int k = leftX; k < (leftX + width); ++k) {
                        for (int l = topY; l < (topY + height); ++l) {
                            copiedImage.at<cv::Vec3b>(l, k) = 0;
                        }
                    }
                }
            }
            std::cout << "h" << std::endl;
            cv::Mat copiedImageInv;
            cv::bitwise_not(copiedImage,copiedImageInv);
            std::stringstream ss;
            ss << path << "/mask" << std::setw(8) << std::setfill('0') << p << ".png";
            std::string s = ss.str();
            cv::imwrite(s, copiedImageInv);
        }
        const std::string globPathBinaryMaskPreProc = path + "/mask" + "/*.png";
        std::vector<cv::String> filesBinaryMaskPreProc;
        cv::glob(globPathBinaryMaskPreProc, filesBinaryMaskPreProc);


    }
}

*/


/*
TEST(semSegBilder, dumpingImageinFolderThresholding) {
    using namespace data_generation;
    Params params;
    loadParams(params);


    const std::string globPath = "/home/kirik/Desktop/filteredImages/*.png";
    std::vector<cv::String> files;

    cv::glob(globPath, files);

    //std::string path = "/home/kirik/Desktop/filteredImages/image";

    /*
    for (int l = 0; l < files.size(); ++l) {
        std::stringstream ss;
        ss << path;
        ss<< l<< ".png";
        cv::Mat img = cv::imread( files[l]);
        cv::Mat imageOutput;
        //cv::inRange(img,cv::Scalar(125,125,125), cv::Scalar(255,255,255),imageOutput);

        //imageOutput = img < 100;
        cv::threshold(img, imageOutput, 100, 255, cv::THRESH_BINARY);

        cv::imshow("projection", imageOutput);
        cv::waitKey(0);
        cv::imwrite(ss.str(),imageOutput);

    }



    cv::Mat img = cv::imread( files[0]);
    cv::Mat imageOutput;

    cv::imshow("projection", img);
    cv::waitKey(0);
    for (int j = 0; j < img.cols; ++j) {
        for (int k = 0; k < img.rows ; ++k) {
            cv::Vec3b intensity = img.at<cv::Vec3b>(cv::Point(j,k));
            uchar blue = intensity.val[0];
            uchar green = intensity.val[1];
            uchar red = intensity.val[2];
            if(blue == 0 && green == 0 && red ==0) {
                //std::cout << "hge" << std::endl;
            }
            else if(blue == 255 && green == 255 && red ==255){
                //std::cout << "whaat "<< std::endl;
            }
            else{
                std::cout << "whaat ne oldu "<< std::endl;
            }

        }

    }

}

*/


/*
TEST(semSegBilder, preprocessSemSegImage) {
    using namespace data_generation;
    Params params;
    params.loadParams();


    const std::string globPath = "/mrtstorage/users/kirik/sim_image_ext/gray_right_sem/binary_mask_dynamic/*.png";
    std::vector<cv::String> files;
    cv::glob(globPath, files);

    for (int p = 0; p < files.size(); ++p) {

    cv::Mat image = cv::imread(files[p]);
    cv::Mat copiedImage = image.clone();


    cv::Mat greyCopiedImage, labels, stats, centroids;
    cv::cvtColor(copiedImage, greyCopiedImage, CV_BGR2GRAY);


    auto num_objects = cv::connectedComponentsWithStats(greyCopiedImage, labels, stats, centroids, 8);
    std::cout << "number of objects in image : " << num_objects << std::endl;
    for (int j = 1; j < num_objects; ++j) {
        int16_t leftX = stats.at<int>(j, cv::CC_STAT_LEFT);
        int16_t height = stats.at<int>(j, cv::CC_STAT_HEIGHT);
        int16_t width = stats.at<int>(j, cv::CC_STAT_WIDTH);
        int16_t topY = stats.at<int>(j, cv::CC_STAT_TOP) ;

        //std::cout <<  topY << std::endl;
        //std::cout << height << std::endl;
        int16_t  lowestY = topY + height;
        //std::cout << "lowestY " << lowestY << std::endl;
        //std::cout << "image.rows/2 " << image.rows/2 << std::endl;

        if( lowestY < (image.rows/2) ){
            for (int k = leftX; k < (leftX + width); ++k) {
                for (int l = topY; l < (topY + height); ++l) {
                    copiedImage.at<cv::Vec3b>(l,k) = 0;
                    //std::cout << "l: " << l << "k: " << k<< std::endl;
                }
            }

        }

    }
    //if (copiedImage.rows > 1000 || copiedImage.cols > 1500)
    //    cv::resize(copiedImage, copiedImage, cv::Size(image.cols * 0.35, image.rows * 0.35), 0, 0);
    //cv::imshow("semSeg", copiedImage);

    std::stringstream ss;
    ss <<"/mrtstorage/users/kirik/sim_image_ext/gray_right_sem/binary_mask_dynamic_preproc/mask" << std::setw(8) << std::setfill('0') << p << ".png";
    std::string s = ss.str();
    cv::imwrite(s, copiedImage);
    }

}


*/

