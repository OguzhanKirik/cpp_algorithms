
#include "data_generator.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>


namespace data_generation {

    //data_generator::data_generator(){};
    //DEFAULT CONSTUCTOR

    data_generator::data_generator(const camera_info &camera,
                                   const data_prep &data,
                                   const map_handler& map_,
                                   const disparity_handler& disparityHandler_,
                                   const cloud_handler& cloudHandler_,
                                   const Params& params_)
            : camera{camera}, data(data),disparityHandler(disparityHandler_),
            map(map_),cloudHandler(cloudHandler_),params(params_)
    {
        std::cout << "Data Generator is on " << std::endl;
    }


    void data_generator::label()const {

        if(params.occlusionHandlings.binaryMask == "true" && params.occlusionHandlings.stereoVision == "true" &&
        params.occlusionHandlings.lidar == "true"){
            std::cout << " Stereo Vision & Binary Mask & Lidar" << std::endl;
            throw std::runtime_error("Lidar is not implemented yet, please deactivate lidar");
            labelDisparityLidarBinaryMask();
    }
    else if (params.occlusionHandlings.binaryMask == "true" && params.occlusionHandlings.stereoVision == "true" &&
        params.occlusionHandlings.lidar == "false"){
            std::cout << " Stereo Vision & Binary Mask " << std::endl;
            labelDisparityBinaryMask();
    }


    else if(params.occlusionHandlings.binaryMask == "false" && params.occlusionHandlings.stereoVision == "true" &&
            params.occlusionHandlings.lidar == "false"){
            std::cout << " Just Stereo Vision" << std::endl;
            labelDisparity();
    }
    else if(params.occlusionHandlings.binaryMask == "true" && params.occlusionHandlings.lidar == "false" &&
            params.occlusionHandlings.stereoVision == "false") {
            std::cout << " Just Binary Mask " << std::endl;
            labelBinaryMask();
    }
    else if (params.occlusionHandlings.binaryMask == "true" && params.occlusionHandlings.stereoVision == "false" &&
             params.occlusionHandlings.lidar == "true"){
            std::cout << " Binary Mask & Lidar " << std::endl;
            throw std::runtime_error(" \"Lidar is not implemented yet, please deactivate lidar");
            labelBinaryLidar();
    }
    else
        throw std::runtime_error(" There need to be Binary Mask or Stereo Vision picked");
}

    void data_generator::labelDisparityBinaryMask() const {
    uint64_t pressedKey = 0;
    uint64_t curIndex = params.startId;
    uint64_t min = 0;
    uint64_t max = data.getFileSize() - 1;

    for (int curIndex = params.startId; curIndex < data.getFileSize(); ++curIndex) {
        cv::Mat img = data.getImage(curIndex);
        assert(!img.empty());
        cv::Mat maskedDisparityMap = maskDisparityMap(curIndex);


        Pose camPoseMap = data.getPoseByID(curIndex) * camera.getCamera().cameraPose;
        MapElements mapElements = map.getMapElements(camPoseMap);
        MapElements mapElementsIn,nonOccludedElements;
        mapElementsIn = filterMapElements(curIndex, mapElements, img.cols, img.rows);

        if(mapElementsIn.lights.size() !=0){
            nonOccludedElements = handleOcclusionStereoVision(curIndex,mapElementsIn);
            mapElementsIn.lights = nonOccludedElements.lights;
        }

        createFolders();
        cv::Mat LabelID = getBlackImage(img);
        cv::Mat Overlayed = img.clone();

        labelMapElementsRepresentationWise(mapElementsIn,Overlayed);
        labelMapElementsClassWise(mapElementsIn,LabelID);


        cv::Mat maskStatic = disparityHandler.generateMaskForStaticObstacles(camera,params,maskedDisparityMap,curIndex);
        if (params.element.dynamicClass == "true"){
            cv::Mat DynamicClassOverlayed = maskImageDynamicDynClass(curIndex,Overlayed);// dynamic filter as dynamic class
            cv::Mat DynamicClassLabelID  = maskImageDynamicDynClassID(curIndex,LabelID);


            cv::Mat StaticLabelID = maskImageStatic(maskStatic,DynamicClassLabelID);
            cv::Mat StaticOverlayed = maskImageStatic(maskStatic,DynamicClassOverlayed);
            writeOverlayed(StaticOverlayed,curIndex);
            writeImages(img,curIndex);
            writeLabelID(StaticLabelID,curIndex);
            dumpInJsonFile(mapElementsIn, curIndex);
        }else{
            cv::Mat DynamicClassOverlayed = maskImageDynamic(curIndex,Overlayed);
            cv::Mat DynamicClassLabelID  = maskImageDynamic(curIndex,LabelID);
            cv::Mat StaticLabelID = maskImageStatic(maskStatic,DynamicClassLabelID);
            cv::Mat StaticOverlayed = maskImageStatic(maskStatic,DynamicClassOverlayed);

            writeOverlayed(StaticOverlayed,curIndex);
            writeImages(img,curIndex);
            writeLabelID(StaticLabelID,curIndex);
            dumpInJsonFile(mapElementsIn, curIndex);
        }
        std::cout << curIndex <<  " done " << std::endl;
    }
}

    void data_generator::labelBinaryMask() const {
    uint64_t pressedKey = 0;
    uint64_t curIndex = params.startId;
    uint64_t min = 0;
    uint64_t max = data.getFileSize() - 1;
    //while (pressedKey != 120) {
    for (int curIndex = params.startId; curIndex < data.getFileSize(); ++curIndex) {
        cv::Mat img = data.getImage(curIndex);
        assert(!img.empty());
        Pose camPoseMap = data.getPoseByID(curIndex) * camera.getCamera().cameraPose;

        MapElements mapElements = map.getMapElements(camPoseMap);
        MapElements mapElementsIn, NonOccludedElements;
        mapElementsIn = filterMapElements(curIndex, mapElements, img.cols, img.rows);

        createFolders();
        cv::Mat LabelID = getBlackImage(img);
        cv::Mat Overlayed = img.clone();
        labelMapElementsRepresentationWise(mapElementsIn, Overlayed);
        labelMapElementsClassWise(mapElementsIn, LabelID);

        if (params.element.dynamicClass == "true") {
            cv::Mat DynamicClassOverlayed = maskImageDynamicDynClass(curIndex,Overlayed);// dynamic filter as dynamic class
            cv::Mat DynamicClassLabelID = maskImageDynamicDynClassID(curIndex, LabelID);
            writeOverlayed(Overlayed, curIndex);
            writeImages(img, curIndex);
            writeLabelID(DynamicClassLabelID, curIndex);
            dumpInJsonFile(mapElementsIn, curIndex);
        } else {
            throw std::runtime_error(" Activate dynamic class");
        }
        std::cout << curIndex <<  " done " << std::endl;
    }
}

    void data_generator::labelDisparity() const {
    uint64_t pressedKey = 0;
    uint64_t curIndex = params.startId;
    uint64_t min = 0;
    uint64_t max = data.getFileSize() - 1;

    for (int curIndex = params.startId; curIndex < data.getFileSize(); ++curIndex) {
        cv::Mat img = data.getImage(curIndex);
        assert(!img.empty() && "No Image ");
        cv::Mat image = getBlackImage(img);
        cv::Mat disparityMap = data.getDisparityMap(curIndex);

        Pose camPoseMap = data.getPoseByID(curIndex) * camera.getCamera().cameraPose;
        MapElements mapElements = map.getMapElements(camPoseMap);
        MapElements mapElementsIn,nonOccludedElements;
        mapElementsIn = filterMapElements(curIndex, mapElements, img.cols, img.rows);

        if(mapElementsIn.lights.size() !=0){
            nonOccludedElements = handleOcclusionStereoVision(curIndex,mapElementsIn);
            mapElementsIn.lights = nonOccludedElements.lights;
        }

        createFolders();
        cv::Mat LabelID = getBlackImage(img);
        cv::Mat Overlayed = img.clone();

        labelMapElementsRepresentationWise(mapElementsIn, Overlayed);
        labelMapElementsClassWise(mapElementsIn, LabelID);


        cv::Mat maskStatic = disparityHandler.generateMaskForStaticObstacles(camera,params,disparityMap,curIndex);

        if (params.element.dynamicClass == "true")
            throw std::runtime_error("Dynamic Class is set true, please include binary mask");
        else{
            cv::Mat StaticLabelID = maskImageStatic(maskStatic,LabelID);
            cv::Mat StaticOverlayed = maskImageStatic(maskStatic,Overlayed);
            writeOverlayed(StaticOverlayed,curIndex);
            writeImages(img,curIndex);
            writeLabelID(StaticLabelID,curIndex);
            dumpInJsonFile(mapElementsIn, curIndex);
        }
        std::cout << curIndex <<  " done " << std::endl;
    }
}





    void data_generator::labelMapElementsClassWise(const MapElements& mapElements, const cv::Mat& img)const{

        // Hierarchy is essential!!!
        if(params.element.roads == "true")
            labelRoads(mapElements.roads,img);
        if(params.element.trafficSign == "true")
            labelTrafficSigns(mapElements.signs,img);
        if(params.element.trafficLight == "true")
            labelTrafficLights(mapElements.lights,img);
        if(params.element.terrains == "true")
            labelTerrains(mapElements.terrains,img);
        if(params.element.dashedLines == "true")
            labelDashedLines(mapElements.dashedLines,img);
        if(params.element.solidLines == "true")
            labelSolidLines(mapElements.solidLines,img);
        if(params.element.walkways == "true")
            labelWalkways(mapElements.walkways,img);
        if(params.element.otherLines == "true")
            labelOtherLines(mapElements.otherLines,img);

    }

    void data_generator::dumpInJsonFile(const MapElements& mapElements,const FrameId& curIndex)const{
        std::stringstream ss;
        ss << params.outputDir <<"/json/gtFine" << std::setw(8) << std::setfill('0') << curIndex << ".json";
        std::string s = ss.str();
        std::ofstream o(s);
        Json::Value j;
        j["imgHeight"] = 1536;
        j["imgWidth"] = 4096;
        j["objects"];

        if(mapElements.roads.size() != 0){
            j["objects"][0]["label"] = "road";
            for (int l = 0; l < mapElements.roads.size(); ++l) {
                for (int k = 0; k < mapElements.roads.at(l).imageCoordinates.size(); ++k) {
                    j["objects"][0]["polygon"][l][k][0] = mapElements.roads.at(l).imageCoordinates.at(k).x;
                    j["objects"][0]["polygon"][l][k][1] = mapElements.roads.at(l).imageCoordinates.at(k).y;
                }
            }
        }

        if(mapElements.terrains.size() != 0){
            j["objects"][1]["label"] = "terrain";
            for (int l = 0; l < mapElements.terrains.size(); ++l) {
                for (int k = 0; k < mapElements.terrains.at(l).imageCoordinates.size(); ++k) {
                    j["objects"][1]["polygon"][l][k][0] = mapElements.terrains.at(l).imageCoordinates.at(k).x;
                    j["objects"][1]["polygon"][l][k][1] = mapElements.terrains.at(l).imageCoordinates.at(k).y;
                }
            }
        }
        if(mapElements.walkways.size() != 0){
            j["objects"][2]["label"] = "walkway";
            for (int l = 0; l < mapElements.walkways.size(); ++l) {
                for (int k = 0; k < mapElements.walkways.at(l).imageCoordinates.size(); ++k) {
                    j["objects"][2]["polygon"][l][k][0] = mapElements.walkways.at(l).imageCoordinates.at(k).x;
                    j["objects"][2]["polygon"][l][k][1] = mapElements.walkways.at(l).imageCoordinates.at(k).y;
                }
            }
        }


        if(mapElements.dashedLines.size() != 0){
            j["objects"][3]["label"] = "dashedLine";
            for (int l = 0; l < mapElements.dashedLines.size(); ++l) {
                for (int k = 0; k < mapElements.dashedLines.at(l).imageCoordinates.size(); ++k) {
                    j["objects"][3]["polygon"][l][k][0] = mapElements.dashedLines.at(l).imageCoordinates.at(k).x;
                    j["objects"][3]["polygon"][l][k][1] = mapElements.dashedLines.at(l).imageCoordinates.at(k).y;
                }
            }
        }

        if(mapElements.solidLines.size() != 0){
            j["objects"][4]["label"] = "solidLine";
            //for(auto solidLine : mapElements.solidLines) {
            for (int l = 0; l < mapElements.solidLines.size(); ++l) {
                for (int k = 0; k < mapElements.solidLines.at(l).imageCoordinates.size(); ++k) {
                    j["objects"][4]["polygon"][l][k][0] = mapElements.solidLines.at(l).imageCoordinates.at(k).x;
                    j["objects"][4]["polygon"][l][k][1] = mapElements.solidLines.at(l).imageCoordinates.at(k).y;
                }
            }
        }



        if(mapElements.otherLines.size() != 0){
            j["objects"][5]["label"] = "otherLine";
            for (int l = 0; l < mapElements.otherLines.size(); ++l) {
                for (int k = 0; k < mapElements.otherLines.at(l).imageCoordinates.size(); ++k) {
                    j["objects"][5]["polygon"][l][k][0] = mapElements.otherLines.at(l).imageCoordinates.at(k).x;
                    j["objects"][5]["polygon"][l][k][1] = mapElements.otherLines.at(l).imageCoordinates.at(k).y;
                }
            }
        }

        o  << j << std::endl;

    }

    void data_generator::labelMapElementsRepresentationWise(const MapElements& mapElements, const cv::Mat& img)const{

        // Hierarchy is essential!!!
        if(params.element.roads == "true")
            labelRoadsColor(mapElements.roads,img);
        if(params.element.trafficSign == "true")
            labelTrafficSigns(mapElements.signs,img);
        if(params.element.trafficLight == "true")
            labelTrafficLights(mapElements.lights,img);
        if(params.element.terrains == "true")
            labelTerrainsColor(mapElements.terrains,img);
        if(params.element.dashedLines == "true")
            labelDashedLinesColor(mapElements.dashedLines,img);
        if(params.element.solidLines == "true")
            labelSolidLinesColor(mapElements.solidLines,img);
        if(params.element.walkways == "true")
            labelWalkwaysColor(mapElements.walkways,img);
        if(params.element.otherLines == "true")
            labelOtherLinesColor(mapElements.otherLines,img);


    }

    void data_generator::labelOtherLines(const OtherLines& otherLines,const cv::Mat& img)const{
        for(auto otherLine : otherLines) {
            if(otherLine.imageCoordinates.size() !=0){
                std::vector<Points> polygons;
                polygons.emplace_back(otherLine.imageCoordinates);
                cv::fillPoly(img,polygons,cv::Scalar(6, 6,6), 8, 0);
            }
            //cv::fillConvexPoly(img,otherLine.imageCoordinates,cv::Scalar(255,128,0), 8, 0);
        }
    }

    void data_generator::labelDashedLines(const DashedLines& dashedLines,const cv::Mat& img)const{
        for(auto dashedLine : dashedLines) {
            if(dashedLine.imageCoordinates.size() !=0){
                std::vector<Points> polygons;
                polygons.emplace_back(dashedLine.imageCoordinates);
                cv::fillPoly(img,polygons,cv::Scalar(4, 4,4), 8, 0);
            }
            // cv::fillConvexPoly(img,dashedLine.imageCoordinates,cv::Scalar(220,20,60), 8, 0);
        }
    }

    void data_generator::labelSolidLines(const SolidLines & solidLines,const cv::Mat& img)const{
        for(auto solidLine : solidLines) {
            if(solidLine.imageCoordinates.size() !=0){
                std::vector<Points> polygons;
                polygons.emplace_back(solidLine.imageCoordinates);
                cv::fillPoly(img,polygons,cv::Scalar(5, 5,5), 8, 0);
            }
            //cv::fillConvexPoly(img,solidLine.imageCoordinates,cv::Scalar(220,220,0), 8, 0);
        }
    }

    void data_generator::labelTerrains(const Terrains& terrains,const cv::Mat& img)const{
        for(auto terrain : terrains) {
            if(terrain.imageCoordinates.size() !=0){
                std::vector<Points> polygons;
                polygons.emplace_back(terrain.imageCoordinates);
                cv::fillPoly(img,polygons,cv::Scalar(2, 2,2), 8, 0);
            }
            //cv::fillConvexPoly(img,terrain.imageCoordinates,cv::Scalar(152,251,152), 8, 0);
        }
    }

    void data_generator::labelWalkways(const Walkways & walkways,const cv::Mat& img)const{
        for(auto walkway : walkways) {
            if(walkway.imageCoordinates.size() !=0){
                std::vector<Points> polygons;
                polygons.emplace_back(walkway.imageCoordinates);
                cv::fillPoly(img,polygons,cv::Scalar(3, 3,3), 8, 0);
            }
            //cv::fillConvexPoly(img,walkway.imageCoordinates,cv::Scalar(152,35,232), 8, 0);
        }
    }

    void data_generator::labelRoads(const Roads& roads, const cv::Mat& img)const{
        for(auto road : roads) {
            if(road.imageCoordinates.size() !=0){
                std::vector<Points> polygons;
                polygons.emplace_back(road.imageCoordinates);
                cv::fillPoly(img,polygons,cv::Scalar(1, 1,1), 8, 0);
            }
            //cv::fillConvexPoly(img,road.imageCoordinates,cv::Scalar(128,64,128), 8, 0);

        }
    }

    void data_generator::labelOtherLinesColor(const OtherLines& otherLines,const cv::Mat& img)const{
        for(auto otherLine : otherLines) {
            if(otherLine.imageCoordinates.size() !=0){
                std::vector<Points> polygons;
                polygons.emplace_back(otherLine.imageCoordinates);
                cv::fillPoly(img,polygons,cv::Scalar(255, 128,0), 8, 0);
            }
            //cv::fillConvexPoly(img,otherLine.imageCoordinates,cv::Scalar(255,128,0), 8, 0);
        }
    }

    void data_generator::labelDashedLinesColor(const DashedLines& dashedLines,const cv::Mat& img)const{
        for(auto dashedLine : dashedLines) {
            if(dashedLine.imageCoordinates.size() !=0){
                std::vector<Points> polygons;
                polygons.emplace_back(dashedLine.imageCoordinates);
                cv::fillPoly(img,polygons,cv::Scalar(220, 20,60), 8, 0);
            }
            // cv::fillConvexPoly(img,dashedLine.imageCoordinates,cv::Scalar(220,20,60), 8, 0);
        }
    }

    void data_generator::labelSolidLinesColor(const SolidLines & solidLines,const cv::Mat& img)const{
        for(auto solidLine : solidLines) {
            if(solidLine.imageCoordinates.size() !=0){
                std::vector<Points> polygons;
                polygons.emplace_back(solidLine.imageCoordinates);
                cv::fillPoly(img,polygons,cv::Scalar(220, 220,0), 8, 0);
            }
            //cv::fillConvexPoly(img,solidLine.imageCoordinates,cv::Scalar(220,220,0), 8, 0);
        }
    }

    void data_generator::labelTerrainsColor(const Terrains& terrains,const cv::Mat& img)const{
        for(auto terrain : terrains) {
            if(terrain.imageCoordinates.size() !=0){
                std::vector<Points> polygons;
                polygons.emplace_back(terrain.imageCoordinates);
                cv::fillPoly(img,polygons,cv::Scalar(152, 251,152), 8, 0);
            }
            //cv::fillConvexPoly(img,terrain.imageCoordinates,cv::Scalar(152,251,152), 8, 0);
        }
    }

    void data_generator::labelWalkwaysColor(const Walkways & walkways,const cv::Mat& img)const{
        for(auto walkway : walkways) {
            if(walkway.imageCoordinates.size() !=0){
                std::vector<Points> polygons;
                polygons.emplace_back(walkway.imageCoordinates);
                cv::fillPoly(img,polygons,cv::Scalar(152, 35,232), 8, 0);
            }
            //cv::fillConvexPoly(img,walkway.imageCoordinates,cv::Scalar(152,35,232), 8, 0);
        }
    }

    void data_generator::labelRoadsColor(const Roads& roads, const cv::Mat& img)const{
        for(auto road : roads) {
            if(road.imageCoordinates.size() !=0){
                std::vector<Points> polygons;
                polygons.emplace_back(road.imageCoordinates);
                cv::fillPoly(img,polygons,cv::Scalar(128, 64,128), 8, 0);
            }
            //cv::fillConvexPoly(img,road.imageCoordinates,cv::Scalar(128,64,128), 8, 0);

        }
    }

    void data_generator::labelTrafficLights(const Lights& lights, const cv::Mat& img)const{
        for(auto light : lights) {
            if(light.roi.countours.size() != 0 ){
                std::cout << light.roi.countours.size() << "contours size" << std::endl;
                if(light.roi.partialOcclusion == true){
                    std::cout << "partial" << std::endl;
                    cv::fillPoly(img,light.roi.countours,cv::Scalar(220,220, 0), 8, 0,cv::Point(light.roi.smallest_x,light.roi.smallest_y));
                }else{
                    cv::fillPoly(img,light.roi.countours,cv::Scalar(25,255, 255), 8, 0);

                }
                //BB Labeling
                //cv::Rect rect = cv::boundingRect(light.roi.countours.back());
                //cv::rectangle(img, rect.tl(), rect.br(), cv::Scalar(110, 220, 0), 8, 0);
            }
        }
    }

    void data_generator::labelTrafficSigns(const Signs& signs, const cv::Mat& img)const{
        for(auto sign : signs) {
            if(sign.roi.countours.size() != 0 ){
                cv::fillConvexPoly(img,sign.roi.countours.back(),cv::Scalar(220,220, 0), 8, 0);
            }
        }
    }

    bool data_generator::isProjectionInTheImage(const ImageCoordinateFloat &imgPoints, const int16_t& imageCol,
                                               const int16_t& imageRow)const {
        auto u = (imgPoints)(0);
        auto v = (imgPoints)(1);
        if (v >= 0 ) {

            return true;
        } else {
            return false;
        }
    }




    bool data_generator::isInCircleTerritoryFlying(const Position &ObjectPosToCamera, const Position &pos )const{
        float objectEuclideanDistanceToCamera = getEuclideanDistance(ObjectPosToCamera);

        if( objectEuclideanDistanceToCamera < params.circleRadiusFlying){
            return true;
        } else {
            return false;
        }
    }

    bool data_generator::isInCircleTerritoryNotFlying(const Position &ObjectPosToCamera, const Position &pos )const{
        float objectEuclideanDistanceToCamera = getEuclideanDistance(ObjectPosToCamera);
        if( objectEuclideanDistanceToCamera < params.circleRadiusNotFlying){
            return true;
        } else {
            return false;
        }
    }


    ImageCoordinateFloat data_generator::getImagePointsFlyingObjets(const FrameId &frame, const Position &position) const {
        Pose vehiclePose = data.getPoseByID(frame);
        Eigen::Isometry3d camPoseMap = vehiclePose * camera.getCamera().cameraPose;
        Eigen::Vector3d objectTOCamera = camPoseMap.inverse() * position;

        bool inCircle = isInCircleTerritoryFlying(objectTOCamera,position);
        ImageCoordinateFloat imgCoordinates(-1, -1);
        //if(inCircle){
            camera.getCamera().cameraModel->getImagePoint(objectTOCamera, imgCoordinates);
        //}
        return imgCoordinates;

    }

    ImageCoordinateFloat data_generator::getImagePointsNotFlyingObjets(const FrameId &frame, const Position &position) const {
        ImageCoordinateFloat imgCoordinates(-1, -1);
        //if(inCircle){
            camera.getCamera().cameraModel->getImagePoint(position, imgCoordinates);
        //}
        return imgCoordinates;

    }

    void data_generator::resizeImage(cv::Mat& image)const{
        if (image.rows > 1000 || image.cols > 1500)
            cv::resize(image, image, cv::Size(image.cols * 0.35, image.rows * 0.35), 0, 0);
    }

    cv::Mat data_generator::getOverlayedImage(const cv::Mat& image,const FrameId& curIndex)const{
        cv::Mat res;
        cv::Mat origImage = data.getImage(curIndex);
        double alpha = 0.5;
        double beta = 1 - alpha;
        cv::addWeighted(origImage,alpha,image,beta,0.0,res);
        return res;
    }

    void data_generator::writeOverlayed(const cv::Mat& image,const FrameId& curIndex)const{
        cv::Mat res = getOverlayedImage(image,curIndex);
        //Stamp timeStamp = data.getImageStampById(curIndex);
        //std::string timeS = boost::lexical_cast<std::string>(timeStamp);
        std::stringstream ss;
        ss << params.outputDir <<"/overlayed/overlayed" << std::setw(8) << std::setfill('0') << curIndex << ".png";
        //ss << "/mrtstorage/users/kirik/lex_map_ost/datasets/dataset_V1/dataset_dyn_stc/dynamic_static_overlayed/output" <<"/"<< timeS << ".png";
        std::string s = ss.str();
        cv::imwrite(s, res);
    }

    void data_generator::writeLabelID(const cv::Mat& image,const FrameId& curIndex)const {
        //Stamp timeStamp = data.getImageStampById(curIndex);
        //std::string timeS = boost::lexical_cast<std::string>(timeStamp);
        std::stringstream ss;
        ss <<params.outputDir <<"/label/label" << std::setw(8) << std::setfill('0') << curIndex << ".png";
        //ss << "/mrtstorage/users/kirik/lex_map_ost/datasets/dataset_V1/dataset_dyn_stc/dataset_dynamic_static_class/output" << "/" << timeS << ".png";
        std::string s = ss.str();
        cv::imwrite(s, image);
    }

    void data_generator::writeImages(const cv::Mat& image,const FrameId& curIndex)const {
        //Stamp timeStamp = data.getImageStampById(curIndex);
        //std::string timeS = boost::lexical_cast<std::string>(timeStamp);
        //ss << params.outputDir << "/" << timeS << ".png";
        std::stringstream ss;
        ss << params.outputDir <<"/image/image" << std::setw(8) << std::setfill('0') << curIndex << ".png";
        std::string s = ss.str();
        cv::imwrite(s, image);

    }

    MapElements data_generator::handleOcclusionLidar(const FrameId& curIndex,const MapElements& mapElements)const{
        MapElements res;

        Cloud pointCloud = data.getCloudByID(curIndex);
        int16_t col_size = data.getImage(curIndex).cols;
        int16_t row_size = data.getImage(curIndex).rows;

        CloudPosImgs cloudPosImgs = cloudHandler.getProjectedPointClouds(camera.getCameraLidar(),pointCloud,col_size,row_size );
        //if(mapElements.lights.size() != 0 )
        //    res.lights = checkTrafficLightsLidar(mapElements.lights,cloudPosImgs,curIndex);
        //if(mapElements.lights.size() != 0 )
        //    res.signs = checkTrafficSignsLidar(mapElements.signs,pointCloud,curIndex);
        //if(mapElements.dashedLines.size() != 0)
        //    res.dashedLines = checkDashedLinesLidar(mapElements.dashedLines,pointCloud,curIndex);
        //if(mapElements.solidLines.size() != 0)
        //    res.solidLines = checkSolidLinesLidar(mapElements.solidLines,pointCloud,curIndex);
        return res;
    }


    MapElements data_generator::handleOcclusionStereoVision(const FrameId& curIndex,const MapElements& mapElements)const{
        MapElements res;
        cv::Mat maskedDisparityMap;
        if(params.occlusionHandlings.binaryMask == "true") {
            maskedDisparityMap = maskDisparityMap(curIndex);
        }else{
            maskedDisparityMap = data.getDisparityMap(curIndex);
        }
        if(mapElements.lights.size() != 0 )
            res.lights = checkTrafficLightsStereoVision(mapElements.lights,maskedDisparityMap,curIndex);
        return res;
    }



    Lights data_generator::checkTrafficLightsStereoVision(const Lights& lights,const cv::Mat& maskedDisparityMap,const FrameId& curIndex)const{
        Lights res;
        for(auto light : lights){
            Light l = light;
            std::cout << "distance " << getEuclideanDistance(getGroundTruthDistance(curIndex,light.centerOfFour())) << std::endl;
            DisparityRange groundTruthDispRange = getGroundTruthDisparityRange(light.centerOfFour(),curIndex);
            cv::Mat cropedMaskedDisparityMap = extractROIFromImage(light.roi,maskedDisparityMap);
            l.roi = disparityHandler.checkForOcclusionROI(light.roi,cropedMaskedDisparityMap,groundTruthDispRange);
            std::cout << "curindex" << curIndex << "groundDisparity " << groundTruthDispRange.medDisp << std::endl;
            std::cout << "curindex" << curIndex << "mean disp " << l.roi.mean << std::endl;
            if(l.roi.greatest_y > 0)
                res.emplace_back(l);
        }
        return res;
    }


    Signs data_generator::checkTrafficSignsStereoVision(const Signs& signs,const cv::Mat& maskedDisparityMap,const FrameId& curIndex)const{
        Signs res;
        for(auto sign : signs){
            Sign s = sign;
            DisparityRange groundTruthDispRange = getGroundTruthDisparityRange(sign.centerOfFour(),curIndex);
            cv::Mat cropedMaskedDisparityMap = extractROIFromImage(sign.roi,maskedDisparityMap);
            s.roi = disparityHandler.checkForOcclusionROI(sign.roi,cropedMaskedDisparityMap,groundTruthDispRange);

            if(s.roi.greatest_y > 0)
                res.emplace_back(s);
        }
        return res;
    }

    cv::Mat data_generator::maskDisparityMap(const FrameId& curIndex)const{
        cv::Mat res;
        cv::Mat binaryMask = data.getBinaryMask(curIndex);
        cv::Mat disparityMap = data.getDisparityMap(curIndex);
        disparityMap.copyTo(res,binaryMask);
        return res;
    }
    cv::Mat data_generator::maskImageDynamic(const FrameId& curIndex, const cv::Mat& image)const{
        cv::Mat res;
        cv::Mat binaryMask = data.getBinaryMask(curIndex);
        image.copyTo(res,binaryMask);


        return res;
    }
    cv::Mat data_generator::maskImageDynamicDynClass(const FrameId& curIndex, const cv::Mat& image)const{
        cv::Mat res,resdyn;
        cv::Mat binaryMask = data.getBinaryMask(curIndex);
        image.copyTo(res,binaryMask);
        cv::Mat maskAdd = binaryMask.clone();
        for(int y=0;y<maskAdd.rows;y++){
            for(int x=0;x<maskAdd.cols;x++){
                // get pixel
                cv::Vec3b & color = maskAdd.at<cv::Vec3b>(y,x);
                if(color[0] == 0){
                    // 7th class
                    color[0] = 0; // 7
                    color[1] = 0; // 7
                    color[2] = 142; // 7
                }else{
                    color[0] = 0;
                    color[1] = 0;
                    color[2] = 0;
                }
            }
        }
        resdyn = res + maskAdd;
        return resdyn;
    }

    cv::Mat data_generator::maskImageDynamicDynClassID(const FrameId& curIndex, const cv::Mat& image)const{
        cv::Mat res,resdyn;
        cv::Mat binaryMask = data.getBinaryMask(curIndex);
        image.copyTo(res,binaryMask);
        cv::Mat maskAdd = binaryMask.clone();
        for(int y=0;y<maskAdd.rows;y++){
            for(int x=0;x<maskAdd.cols;x++){
                // get pixel
                cv::Vec3b & color = maskAdd.at<cv::Vec3b>(y,x);
                if(color[0] == 0){
                    // 7th class
                    color[0] = 7; // 7
                    color[1] = 7; // 7
                    color[2] = 7; // 7
                }else{
                    color[0] = 0;
                    color[1] = 0;
                    color[2] = 0;
                }
            }
        }
        resdyn = res + maskAdd;
        return resdyn;
    }
    cv::Mat data_generator::maskImageStatic(const cv::Mat& staticMask, const cv::Mat& image)const{
        cv::Mat res;
        image.copyTo(res,staticMask);
        return res;
    }
    cv::Mat data_generator::extractROIFromImage(const ROI& roiValues,  const cv::Mat& maskedDisparityMap)const{
        cv::Mat res = cv::Mat::zeros(cv::Size(maskedDisparityMap.cols,maskedDisparityMap.rows), CV_64FC1);
        int16_t h = roiValues.greatest_y - roiValues.smallest_y;
        int16_t w = roiValues.greatest_x - roiValues.smallest_x;
        maskedDisparityMap(cv::Rect(roiValues.smallest_x,roiValues.smallest_y,w,h)).copyTo(res);
        return res;
    }

    MapElements data_generator::filterMapElements(const FrameId& curIndex,
                                                 const MapElements& mapElements,
                                                 const int16_t& imageCol,
                                                 const int16_t& imageRow)const{
        MapElements res;

        if(mapElements.lights.size() != 0 )
            res.lights = filterLights(curIndex,mapElements.lights,imageCol,imageRow);

        if(mapElements.signs.size() != 0 )
            res.signs = filterSigns(curIndex,mapElements.signs,imageCol,imageRow);

        if(mapElements.solidLines.size() != 0)
            res.solidLines = filterSolidLines(curIndex,mapElements.solidLines,imageCol,imageRow);

        if(mapElements.roads.size() != 0)
            res.roads = filterRoads(curIndex,mapElements.roads,imageCol,imageRow);

        if(mapElements.terrains.size() != 0)
            res.terrains = filterTerrains(curIndex,mapElements.terrains,imageCol,imageRow);

        if(mapElements.dashedLines.size() != 0)
            res.dashedLines = filterDashedLines(curIndex,mapElements.dashedLines,imageCol,imageRow);

        if(mapElements.walkways.size() != 0)
            res.walkways = filterWalkways(curIndex,mapElements.walkways,imageCol,imageRow);

        if(mapElements.otherLines.size() != 0)
            res.otherLines = filterOtherLines(curIndex,mapElements.otherLines,imageCol,imageRow);
        return res;
    }


    SolidLines data_generator::filterSolidLines(const FrameId& curIndex,
                                                  const SolidLines& solidLines,
                                                  const int16_t& imageCol,
                                                  const int16_t& imageRow)const {
        SolidLines res;
        for (auto sl : solidLines) {
            //Pose camPoseMap = data.getPoseByID(curIndex) * camera.getCamera().cameraPose;
            sl.posInBoundixBox = fillHolesInPosition(sl.getPositions(),curIndex);
            //for (auto pt : sl.getPositions()) {
            for (auto pt : sl.posInBoundixBox) {
                //Position slTOCamera = camPoseMap.inverse() * pt;
                //sl.posInBoundixBox.emplace_back(slTOCamera);
                auto imgPoints = getImagePointsNotFlyingObjets(curIndex, pt);
                bool projection = isProjectionInTheImage(imgPoints, imageCol, imageRow);
                if (projection) {
                    Point p(imgPoints(0), imgPoints(1));
                    sl.imageCoordinates.emplace_back(p);
                }
            }
            res.emplace_back(sl);
        }
        return res;
    }

    Roads data_generator::filterRoads(const FrameId& curIndex,
                                                const Roads& roads,
                                                const int16_t& imageCol,
                                                const int16_t& imageRow)const {
        Roads res;
        for (auto r : roads) {
            r.posInBoundixBox = fillHolesInPosition(r.getPositions(),curIndex);
            //std::cout <<"size pos" <<r.posInBoundixBox.size() << std::endl;
            for (auto pt : r.posInBoundixBox) {
            //for (auto pt : r.getPositions()) {

                auto imgPoints = getImagePointsNotFlyingObjets(curIndex, pt);
                bool projection = isProjectionInTheImage(imgPoints, imageCol, imageRow);

                if (projection) {
                    Point p(imgPoints(0), imgPoints(1));
                    r.imageCoordinates.emplace_back(p);
                }
            }
            res.emplace_back(r);
        }
        return res;
    }


    Terrains data_generator::filterTerrains(const FrameId& curIndex,
                                        const Terrains& terrains,
                                        const int16_t& imageCol,
                                        const int16_t& imageRow)const {
    Terrains res;
    for (auto t : terrains) {
        t.posInBoundixBox = fillHolesInPosition(t.getPositions(),curIndex);

        for (auto pt : t.posInBoundixBox) {
            auto imgPoints = getImagePointsNotFlyingObjets(curIndex, pt);
            bool projection = isProjectionInTheImage(imgPoints, imageCol, imageRow);
            if (projection) {
                Point p(imgPoints(0), imgPoints(1));
                t.imageCoordinates.emplace_back(p);

            }
        }
        res.emplace_back(t);
    }
    return res;

}

    Walkways data_generator::filterWalkways(const FrameId& curIndex,
                                        const Walkways& walkways,
                                        const int16_t& imageCol,
                                        const int16_t& imageRow)const {
    Walkways res;
    for (auto w : walkways) {
        w.posInBoundixBox = fillHolesInPosition(w.getPositions(),curIndex);

        for (auto pt : w.posInBoundixBox) {
            auto imgPoints = getImagePointsNotFlyingObjets(curIndex, pt);
            bool projection = isProjectionInTheImage(imgPoints, imageCol, imageRow);
            if (projection) {
                Point p(imgPoints(0), imgPoints(1));
                w.imageCoordinates.emplace_back(p);
            }
        }
        res.emplace_back(w);
    }
    return res;
}



    DashedLines data_generator::filterDashedLines(const FrameId& curIndex,
                                              const DashedLines& dashedLines,
                                              const int16_t& imageCol,
                                              const int16_t& imageRow)const {
    DashedLines res;
    for (auto dl : dashedLines) {
        dl.posInBoundixBox = fillHolesInPosition(dl.getPositions(),curIndex);
        //for (auto pt : dl.getPositions()) {
        for (auto pt : dl.posInBoundixBox) {
            auto imgPoints = getImagePointsNotFlyingObjets(curIndex, pt);
            bool projection = isProjectionInTheImage(imgPoints, imageCol, imageRow);
            if (projection) {
                Point p(imgPoints(0), imgPoints(1));
                dl.imageCoordinates.emplace_back(p);
            }
        }
        res.emplace_back(dl);
    }
    return res;
}


    OtherLines data_generator::filterOtherLines(const FrameId& curIndex,
                                            const OtherLines& otherLines,
                                            const int16_t& imageCol,
                                            const int16_t& imageRow)const {
    OtherLines res;
    for (auto ol : otherLines) {
        //Pose camPoseMap = data.getPoseByID(curIndex) * camera.getCamera().cameraPose;
        ol.posInBoundixBox = fillHolesInPosition(ol.getPositions(),curIndex);
        //for (auto pt : ol.getPositions()) {
        for (auto pt : ol.posInBoundixBox) {
            //Position slTOCamera = camPoseMap.inverse() * pt;
            //sl.posInBoundixBox.emplace_back(slTOCamera);
            auto imgPoints = getImagePointsNotFlyingObjets(curIndex, pt);
            bool projection = isProjectionInTheImage(imgPoints, imageCol, imageRow);
            if (projection) {
                Point p(imgPoints(0), imgPoints(1));
                ol.imageCoordinates.emplace_back(p);
            }
        }
        res.emplace_back(ol);
    }
    return res;
}

    Lights data_generator::filterLights(const FrameId& curIndex,
                                    const Lights& lights,
                                    const int16_t& imageCol,
                                    const int16_t& imageRow)const{
    Lights res;
    for (auto l : lights) {
        for (auto pt : l.asList()) {

            auto imgPoints = getImagePointsFlyingObjets(curIndex, pt);
            bool projection = isProjectionInTheImage(imgPoints, imageCol, imageRow);

            if (projection) {
                Point p(imgPoints(0), imgPoints(1));
                l.imageCoordinates.emplace_back(p);
            }
        }
        if(l.imageCoordinates.size() == 4 ){
            l.setRoiCoordAndCountours();
            res.emplace_back(l);
        }

    }
    return res;
}

    Signs data_generator::filterSigns(const FrameId& curIndex,
                                  const Signs& signs,
                                  const int16_t& imageCol,
                                  const int16_t& imageRow)const{
    Signs res;
    for (auto s : signs) {
        for (auto pt : s.asList()) {
            auto imgPoints = getImagePointsFlyingObjets(curIndex, pt);
            bool projection = isProjectionInTheImage(imgPoints, imageCol, imageRow);
            if (projection) {
                Point p(imgPoints(0), imgPoints(1));
                s.imageCoordinates.emplace_back(p);
            }
        }
        if(s.imageCoordinates.size() >1 ){
            s.setRoiCoordAndCountours();
            res.emplace_back(s);
        }

    }
    return res;
}
    /*
    // Rear Camera
    Positions data_generator::fillHolesInPosition(const Positions& positions,const FrameId& curIndex)const{
        Positions res;
        Position inBB = {0,0,0};
        Position outBB = {0,0,0};
        Pose camPoseMap = data.getPoseByID(curIndex) * camera.getCamera().cameraPose;
        int16_t  counterInBB = 0;
        int16_t counterOutBB = 0;
        for (auto pt : positions) {
            Position ptTOCamera = camPoseMap.inverse() * pt;

            //if (ptTOCamera.y() < 10 && ptTOCamera.y() > -10 && ptTOCamera.z() < 150 && ptTOCamera.z() > 0) {
            if (ptTOCamera.y() < 30 && ptTOCamera.y() > -30 && ptTOCamera.x() < 200 && ptTOCamera.x() > 0) {
                std::cout << "position of map elemets in" << ptTOCamera.matrix() << std::endl;
                ++counterInBB;
                counterOutBB = 0;
                inBB = ptTOCamera;
                if (counterInBB == 1) {
                    //if (outBB.z() >= 0)
                    if (outBB.x() >= 0)
                        continue;
                    else {
                        // do interpolation and add two points in vector
                        Position intersectPoint;
                        Position r;
                        intersectPoint.x() = inBB.x() - outBB.x();
                        intersectPoint.y() = inBB.y() - outBB.y();
                        intersectPoint.z() = inBB.z() - outBB.z();
                        //r.z() = 0.5;
                        //double t;
                        //t = (r.z() - outBB.z()) / intersectPoint.z();
                        //r.y() = outBB.y() + (intersectPoint.y() * t);
                        //r.x() = outBB.x() + (intersectPoint.x() * t);
                        //res.emplace_back(r);

                        // Rear Camera
                        r.x() = 0.1;
                        double t;
                        t = (r.x() - outBB.x()) / intersectPoint.x();
                        r.y() = outBB.y() + (intersectPoint.y() * t);
                        r.z() = outBB.z() + (intersectPoint.z() * t);

                        res.emplace_back(r);




                    }
                }
                res.emplace_back(ptTOCamera);
            } else {
                //std::cout << "not in the bounding box" << std::endl;
                counterInBB = 0;
                ++counterOutBB;
                outBB = ptTOCamera;
                if (counterOutBB == 1) {
                    if (inBB.x() == 0 && inBB.y() == 0 && inBB.z() == 0)
                        continue;
                    else {
                        if (outBB.x() < 0) {
                        //if (outBB.x() < 0) {
                            // do interpolation and add one point in vector
                            Position intersectPoint;
                            Position r;
                            intersectPoint.x() = outBB.x() - inBB.x();
                            intersectPoint.y() = outBB.y() - inBB.y();
                            intersectPoint.z() = outBB.z() - inBB.z();

                            //r.z() = 0.5;
                            //double t;
                            //t = (r.z() - inBB.z()) / intersectPoint.z();
                            //r.y() = inBB.y() + (intersectPoint.y() * t);
                            //r.x() = inBB.x() + (intersectPoint.x() * t);
                            //res.emplace_back(r);


                            r.x() = 0.1;
                            double t;
                            t = (r.x() - inBB.x()) / intersectPoint.x();
                            r.y() = inBB.y() + (intersectPoint.y() * t);
                            r.z() = inBB.z() + (intersectPoint.z() * t);
                            res.emplace_back(r);





                        }
                    }
                }
            }

        }
        return res;
    }


    */

    Positions data_generator::fillHolesInPosition(const Positions& positions,const FrameId& curIndex)const{
        Positions res;
        Position inBB = {0,0,0};
        Position outBB = {0,0,0};
        Pose camPoseMap = data.getPoseByID(curIndex) * camera.getCamera().cameraPose;
        int16_t  counterInBB = 0;
        int16_t counterOutBB = 0;
        for (auto pt : positions) {
            Position ptTOCamera = camPoseMap.inverse() * pt;
            //std::cout <<"point wrt camera " << ptTOCamera.matrix() << std::endl;
            if (ptTOCamera.x() < 50 && ptTOCamera.x() > -50 && ptTOCamera.z() < 150 && ptTOCamera.z() > 0) {
                ++counterInBB;
                counterOutBB = 0;
                inBB = ptTOCamera;
                if (counterInBB == 1) {
                    if (outBB.z() >= 0)
                        continue;
                    else {
                        // do interpolation and add two points in vector
                        Position intersectPoint;
                        Position r;
                        intersectPoint.x() = inBB.x() - outBB.x();
                        intersectPoint.y() = inBB.y() - outBB.y();
                        intersectPoint.z() = inBB.z() - outBB.z();
                        r.z() = 0.5;
                        double t;
                        t = (r.z() - outBB.z()) / intersectPoint.z();
                        r.y() = outBB.y() + (intersectPoint.y() * t);
                        r.x() = outBB.x() + (intersectPoint.x() * t);
                        res.emplace_back(r);
                    }
                }
                res.emplace_back(ptTOCamera);
            } else {
                //std::cout << "not in the bounding box" << std::endl;
                counterInBB = 0;
                ++counterOutBB;
                outBB = ptTOCamera;
                if (counterOutBB == 1) {
                    if (inBB.x() == 0 && inBB.y() == 0 && inBB.z() == 0)
                        continue;
                    else {
                        if (outBB.z() < 0) {
                            // do interpolation and add one point in vector
                            Position intersectPoint;
                            Position r;
                            intersectPoint.x() = outBB.x() - inBB.x();
                            intersectPoint.y() = outBB.y() - inBB.y();
                            intersectPoint.z() = outBB.z() - inBB.z();
                            r.z() = 0.5;
                            double t;
                            t = (r.z() - inBB.z()) / intersectPoint.z();
                            r.y() = inBB.y() + (intersectPoint.y() * t);
                            r.x() = inBB.x() + (intersectPoint.x() * t);
                            res.emplace_back(r);
                        }
                    }
                }
            }

        }
        return res;
    }



    /*
    Positions data_generator::fillHolesInPosition(const Positions& positions,const FrameId& curIndex)const{
        Positions res;
        Position inBB = {0,0,0};
        Position outBB = {0,0,0};
        Pose camPoseMap = data.getPoseByID(curIndex) * camera.getCamera().cameraPose;
        int16_t  counterInBB = 0;
        int16_t counterOutBB = 0;
        for (auto pt : positions) {
            Position ptTOCamera = camPoseMap.inverse() * pt;
            if (ptTOCamera.y() < 10 && ptTOCamera.y() > -10 && ptTOCamera.z() < 100 && ptTOCamera.z() > 0) {

                counterOutBB = 0;
                auto imgPoints = getImagePointsNotFlyingObjets(curIndex, ptTOCamera);
                bool projection = isProjectionInTheImage(imgPoints, 4096, 1536);
                if (projection) {
                    counterInBB +=1;
                    inBB = ptTOCamera;
                }
                if (counterInBB == 1) {
                    if (outBB.z() >= 0)
                        continue;
                    else {
                        // do interpolation and add two points in vector
                        std::cout << "do interpolation out--> in " <<std::endl;
                        Position diff = inBB - outBB;

                        double distance = sqrt(pow(diff.x(), 2) + pow(diff.y(), 2) + pow(diff.z(), 2));
                        int nofSamplePts = static_cast<int>(round(distance / 0.01)) ;
                        Position stepVector = 1.0/(nofSamplePts )* diff;
                        for(int i = 1; i < nofSamplePts+2; i++){
                            res.push_back(outBB + stepVector * i);
                        }


                    }
                }
                res.emplace_back(ptTOCamera);
            } else {
                //std::cout << "not in the bounding box" << std::endl;
                counterInBB = 0;
                counterOutBB +=1;
                outBB = ptTOCamera;
                if (counterOutBB == 1) {
                    if (inBB.x() == 0 && inBB.y() == 0 && inBB.z() == 0)
                        continue;
                    else {
                        if (outBB.z() < 0) {
                            // do interpolation and add one point in vector
                            std::cout << "do interpolation in--> out " <<std::endl;
                            Position diff = outBB - inBB;
                            double distance = sqrt(pow(diff.x(), 2) + pow(diff.y(), 2) + pow(diff.z(), 2));
                            int nofSamplePts = static_cast<int>(round(distance /  0.01)) ;
                            Position stepVector = 1.0/(nofSamplePts) * diff;
                            for(int i = 1; i < nofSamplePts+2; i++){
                                res.push_back(inBB + stepVector * i);
                            }


                        }
                    }
                }
            }

        }
        return res;
    }

    */






    DisparityRange data_generator::getGroundTruthDisparityRange(const Position& pos,const FrameId& curIndex)const{
        Position groundTruthDistanceMatrix = getGroundTruthDistance(curIndex,pos);
        float groundTruthDistance = getEuclideanDistance(groundTruthDistanceMatrix);
        DisparityRange disp = getDisparityByDistance(groundTruthDistance);
        return disp;
    }

    DisparityRange data_generator::getDisparityByDistance(const float& groundTruthDistance)const{
        DisparityRange dispRange;
        float baseLine = getEuclideanDistance(camera.getBaseLine());
        dispRange.medDisp = (baseLine * camera.getCamera().cameraModel->getFocalLength()) / groundTruthDistance;
        // 3 m for a circle of 3 m radius
        dispRange.maxDisp = (baseLine * camera.getCamera().cameraModel->getFocalLength()) / (groundTruthDistance - 3);
        dispRange.minDisp = (baseLine * camera.getCamera().cameraModel->getFocalLength()) / (groundTruthDistance + 3);

        return dispRange;
    }


    float data_generator::getEuclideanDistance(const Position& pos)const{
        return sqrt(pow(pos.z(),2) + pow(pos.y(),2)+pow(pos.x(),2));
    }

    cv::Mat data_generator::getBlackImage(const cv::Mat img)const {
        cv::Mat image(img.rows, img.cols, CV_8UC3, cv::Scalar(0,0, 0));
        return image;
    }

    Position data_generator::getGroundTruthDistance(const FrameId& curIndex,const Position &pos)const{
        Pose vehiclePose = data.getPoseByID(curIndex);
        Eigen::Isometry3d camPoseMap = vehiclePose * camera.getCamera().cameraPose;
        Eigen::Vector3d objectTOCamera = camPoseMap.inverse() * pos;
        //std::cout << objectTOCamera.matrix() << std::endl;
        return objectTOCamera;
    }




    Position data_generator::getDistanceByDisparityMap(const Disparity& disparityValue)const{
        Position distanceMatrix = (camera.getBaseLine() * camera.getCamera().cameraModel->getFocalLength()) / disparityValue;
        return distanceMatrix;
    }



    bool data_generator::occlusionCheckROI(const float& mean,const DisparityRange& grTrDispRange)const {
        if(mean >= grTrDispRange.minDisp && mean <= grTrDispRange.maxDisp){
            std::cout <<"PROJECTION" << std::endl;
            return true;
        }else{
            std::cout <<"NO PROJECTION" << std::endl;
            return false;
        }
    }







    void data_generator::createFolders()const{
       std::string pathLabel = params.outputDir + "/label";
       std::string pathJson = params.outputDir + "/json";
       std::string pathOverlayed = params.outputDir + "/overlayed";
       std::string pathImage = params.outputDir + "/image";

       boost::filesystem::path dirLabel(pathLabel);
       boost::filesystem::path dirJson(pathJson);
       boost::filesystem::path dirOverlayed(pathOverlayed);
       boost::filesystem::path dirImage(pathImage);

       if(!boost::filesystem::is_directory(pathLabel)) {
           boost::filesystem::create_directory(dirLabel);
       }
       if(!boost::filesystem::is_directory(pathJson)) {
           boost::filesystem::create_directory(dirJson);
       }
       if(!boost::filesystem::is_directory(pathOverlayed)) {
           boost::filesystem::create_directory(dirOverlayed);
       }
       if(!boost::filesystem::is_directory(pathImage)) {
           boost::filesystem::create_directory(dirImage);
       }
    }

    MapElements data_generator::maskOutDynamicOcclusions(const MapElements& mapElements,const FrameId &frameId)const{
        MapElements res;
        cv::Mat binaryMask = data.getBinaryMask(frameId);

        if(mapElements.lights.size() != 0) {
            res.lights = maskedDynamicOcclusionsForLights(mapElements.lights,binaryMask);
        }

        if(mapElements.signs.size() != 0){
            res.signs = maskedDynamicOcclusionsForSigns(mapElements.signs,binaryMask);
        }
        return res;
    }

    Lights data_generator::maskedDynamicOcclusionsForLights(const Lights& lights,const cv::Mat& binaryMask)const{
        Lights res;
        cv::Mat grayMaskROI;
        cv::cvtColor(binaryMask,grayMaskROI,cv::COLOR_BGR2GRAY);
        for (auto &l : lights) {
            Light l_res = l;
            cv::Mat binaryMaskROI = extractROIFromImage(l_res.roi,grayMaskROI);
            cv::findContours(binaryMaskROI,l_res.roi.countours,cv::noArray(),cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE,cv::Point(l_res.roi.smallest_x,l_res.roi.smallest_y));
            if(l_res.roi.countours.size() > 0)
                res.emplace_back(l_res);
        }
        return res;
    }

    Signs data_generator::maskedDynamicOcclusionsForSigns(const Signs& signs,const cv::Mat& binaryMask)const{
        Signs res;
        cv::Mat grayMaskROI;
        cv::cvtColor(binaryMask,grayMaskROI,cv::COLOR_BGR2GRAY);
        for (auto &s : signs) {
            Sign s_res = s;
            cv::Mat binaryMaskROI = extractROIFromImage(s_res.roi,grayMaskROI);
            cv::findContours(binaryMaskROI,s_res.roi.countours,cv::noArray(),cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE,cv::Point(s_res.roi.smallest_x,s_res.roi.smallest_y));
            if(s_res.roi.countours.size() > 0)
                res.emplace_back(s_res);
        }
        return res;
    }

void data_generator::labelBinaryLidar() const {
    uint64_t pressedKey = 0;
    uint64_t curIndex = params.startId;
    uint64_t min = 0;
    uint64_t max = data.getFileSize() - 1;
    while (pressedKey != 120) {
        cv::Mat img = data.getImage(curIndex);
        assert(!img.empty() && "No Image ");
        cv::Mat image = getBlackImage(img);

        Pose camPoseMap = data.getPoseByID(curIndex) * camera.getCamera().cameraPose;
        MapElements mapElements = map.getMapElements(camPoseMap);
        MapElements mapElementsIn,NonOccludedElements;

        mapElementsIn = filterMapElements(curIndex, mapElements, img.cols, img.rows);

        if(mapElementsIn.roads.size() != 0 || mapElementsIn.signs.size() !=0 || mapElementsIn.dashedLines.size() != 0 ||
           mapElementsIn.terrains.size() != 0 || mapElementsIn.lights.size() !=0 || mapElementsIn.solidLines.size() != 0 ||
           mapElements.walkways.size() != 0) {

            // implement here lidar
            // implement here lidar

            labelMapElementsClassWise(NonOccludedElements,image);

            std::cout << " MASK OUT TRAFFIC ELEMENTS " << std::endl;
            cv::Mat res = maskImageDynamic(curIndex,image);

            //writeImages(res,curIndex);
            resizeImage(res);
            cv::imshow("Training Data", res);
        }else{
            resizeImage(img);
            cv::imshow(" Original ", img);
        }

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

void data_generator::labelDisparityLidarBinaryMask() const {
    uint64_t pressedKey = 0;
    uint64_t curIndex = params.startId;
    uint64_t min = 0;
    uint64_t max = data.getFileSize() - 1;

    while (pressedKey != 120) {
        cv::Mat img = data.getImage(curIndex);
        assert(!img.empty());

        cv::Mat image = getBlackImage(img);
        Pose camPoseMap = data.getPoseByID(curIndex) * camera.getCamera().cameraPose;
        MapElements mapElements = map.getMapElements(camPoseMap);
        MapElements mapElementsIn,NonOccludedElements;

        mapElementsIn = filterMapElements(curIndex, mapElements, img.cols, img.rows);

        if(mapElementsIn.roads.size() != 0 || mapElementsIn.signs.size() !=0 || mapElementsIn.dashedLines.size() != 0 ||
           mapElementsIn.terrains.size() != 0 || mapElementsIn.lights.size() !=0 || mapElementsIn.solidLines.size() != 0) {

            std::cout << " CHECK FOR OCCLUSION WITH DISPARITY " << std::endl;
            NonOccludedElements = handleOcclusionStereoVision(curIndex,mapElementsIn);


            if(mapElements.signs.size() != 0 || mapElements.lights.size() !=0 || mapElements.dashedLines.size() != 0)
                NonOccludedElements = handleOcclusionLidar(curIndex,NonOccludedElements);


            labelMapElementsClassWise(NonOccludedElements,image);

            std::cout << " MASK OUT TRAFFIC ELEMENTS " << std::endl;
            cv::Mat res = maskImageDynamic(curIndex,image);

            //writeImages(res,curIndex);
            resizeImage(res);
            cv::imshow("Training Data", res);
        }else{
            resizeImage(img);
            cv::imshow(" Original ", img);
        }


        resizeImage(img);
        cv::imshow("projection", img);

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



}



