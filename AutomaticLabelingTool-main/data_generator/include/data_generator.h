#pragma once


// Own classes
#include "data_type.h"
#include "cloud_handler.h"
#include "map_handler.h"
#include "camera_info.h"
#include "data_prep.h"
#include "disparity_handler.h"

// STL Libraries
#include <memory>

#include <json/json.h>
#include<json/writer.h>



namespace data_generation {

    class data_generator {
    public:
    /**
     * @brief This constructor will do followings:
     * load camera
     * load data
     * @param camera is the camera object
     * @param data  is the object of the data_prep which constains all the input data.
     * @param map_  is the map object
     * @param disparityHandler_ is the object of the disparity handler class for occlusion handling
     * @param cloudHandler_ is the object of the cloud handler class for occlusion handling
     * @param params_ contains user-defined parameters
     */

        data_generator(const camera_info &camera,
                       const data_prep &data,
                       const map_handler& map_,
                       const disparity_handler &disparityHandler_,
                       const cloud_handler &cloudHandler_,
                       const Params &params_);

        /**
         * @brief activates the labeling method chosen by the user
         */
        void label() const;
        /**
         * @brief labeling is done by using binary images and disparity maps
         */
        void labelDisparityBinaryMask() const;
        /**
         * @brief labeling is done by using disparity maps. Dynamic obstacles are not the suject of this method.
         */
        void labelDisparity() const;
        /**
         * @brief labeling is done by using binary images. Static obstacles are not the subject of this method.
         */
        void labelBinaryMask() const;



        /**
         * @brief returns the map elements that are located in the oriendted bounding box
         */
        MapElements filterMapElements(const FrameId &curIndex,
                                      const MapElements &mapElements,
                                      const int16_t &imageCol,
                                      const int16_t &imageRow) const;

        /**
         *
         * @param curIndex index of the current frame
         * @param mapElements contains map elements
         * @return non occluded flying objects or their parts (if partial occluded)
         */
        MapElements handleOcclusionStereoVision(const FrameId &curIndex, const MapElements &mapElements) const;


        /**
         *
         * @param mapElements contains elements
         * @param frameId is the index of the current frame
         * @return image where dynamic obstacles are removed
         */
        MapElements maskOutDynamicOcclusions(const MapElements &mapElements, const FrameId &frameId) const;

        /**
         *
         * @param staticMask a mask to filter static obstacles
         * @param image
         * @return image where static obstacles are excluded
         */
        cv::Mat maskImageStatic(const cv::Mat& staticMask, const cv::Mat& image)const;



    private:
        const camera_info &camera;
        const data_prep &data;
        const map_handler& map;
        const disparity_handler disparityHandler;
        const cloud_handler cloudHandler;
        const Params &params;


    private:
        void labelDisparityLidarBinaryMask() const;

        void labelBinaryLidar() const;

        void labelMapElementsClassWise(const MapElements &mapElements, const cv::Mat &img) const;

        void labelMapElementsRepresentationWise(const MapElements &mapElements, const cv::Mat &img) const;

        cv::Mat maskImageDynamic(const FrameId &curIndex, const cv::Mat &image) const;

        DisparityRange getGroundTruthDisparityRange(const Position &pos, const FrameId &curIndex) const;

        bool occlusionCheckROI(const float &mean, const DisparityRange &grTrDispRange) const;

        MapElements handleOcclusionLidar(const FrameId& curIndex,const MapElements& mapElements)const;

        Positions fillHolesInPosition(const Positions& positions,const FrameId& curIndex)const;

        cv::Mat maskImageDynamicDynClass(const FrameId& curIndex, const cv::Mat& image)const;

        cv::Mat maskImageDynamicDynClassID(const FrameId& curIndex, const cv::Mat& image)const;

        void writeImages(const cv::Mat &image,const FrameId &curIndex) const;

        void writeLabelID(const cv::Mat& image,const FrameId& curIndex)const;

        void writeOverlayed(const cv::Mat& image,const FrameId& curIndex)const;

        void dumpInJsonFile(const MapElements& mapElements,const FrameId& curIndex)const;

        cv::Mat maskDisparityMap(const FrameId &curIndex) const;

        cv::Mat extractROIFromImage(const ROI &roiValues, const cv::Mat &maskedDisparityMap) const;

        DisparityRange getDisparityByDistance(const float &groundTruthDistance) const;

        Position getDistanceByDisparityMap(const Disparity &disparityValue) const;

        bool isInCircleTerritoryFlying(const Position &ObjectPosToCamera, const Position &pos) const;

        bool isInCircleTerritoryNotFlying(const Position &ObjectPosToCamera, const Position &pos )const;

        bool isProjectionInTheImage(const ImageCoordinateFloat &imgPoints, const int16_t &imageCol,
                                    const int16_t &imageRow) const;

        ImageCoordinateFloat getImagePointsFlyingObjets(const FrameId &frame, const Position &position) const;

        ImageCoordinateFloat getImagePointsNotFlyingObjets(const FrameId &frame, const Position &position) const;

        float getEuclideanDistance(const Position &pos) const;

        Position getGroundTruthDistance(const FrameId &curIndex, const Position &pos) const;

        Lights checkTrafficLightsStereoVision(const Lights &lights,
                                              const cv::Mat &maskedDisparityMap,
                                              const FrameId &curIndex) const;

        Signs checkTrafficSignsStereoVision(const Signs &signs,
                                            const cv::Mat &maskedDisparityMap,
                                            const FrameId &curIndex) const;

        Lights filterLights(const FrameId &curIndex,
                            const Lights &lights,
                            const int16_t &imageCol,
                            const int16_t &imageRow) const;

        Signs filterSigns(const FrameId &curIndex,
                          const Signs &signs,
                          const int16_t &imageCol,
                          const int16_t &imageRow) const;

        DashedLines filterDashedLines(const FrameId &curIndex,
                                      const DashedLines &dashedLines,
                                      const int16_t &imageCol,
                                      const int16_t &imageRow) const;

        SolidLines filterSolidLines(const FrameId &curIndex,
                                    const SolidLines &solidLines,
                                    const int16_t &imageCol,
                                    const int16_t &imageRow) const;

        Roads filterRoads(const FrameId &curIndex,
                          const Roads &roads,
                          const int16_t &imageCol,
                          const int16_t &imageRow) const;

        Terrains filterTerrains(const FrameId &curIndex,
                             const Terrains &terrains,
                             const int16_t &imageCol,
                             const int16_t &imageRow) const;


        OtherLines filterOtherLines(const FrameId& curIndex,
                const OtherLines& otherLines,
                const int16_t& imageCol,
                const int16_t& imageRow)const;

        Walkways filterWalkways(const FrameId& curIndex,
                const Walkways& walkways,
                const int16_t& imageCol,
                const int16_t& imageRow)const;

            Lights maskedDynamicOcclusionsForLights(const Lights &lights, const cv::Mat &binaryMask) const;

            Signs maskedDynamicOcclusionsForSigns(const Signs &signs, const cv::Mat &binaryMask) const;

            void labelTrafficLights(const Lights &lights, const cv::Mat &img) const;

            void labelTrafficSigns(const Signs &signs, const cv::Mat &img) const;

            void labelDashedLines(const DashedLines &dashedLines, const cv::Mat &img) const;

            void labelSolidLines(const SolidLines &solidLines, const cv::Mat &img) const;

            void labelTerrains(const Terrains &terrains, const cv::Mat &img) const;

            void labelRoads(const Roads &roads, const cv::Mat &img) const;

            void labelWalkways(const Walkways & walkways,const cv::Mat& img)const;

            void labelOtherLines(const OtherLines& otherLines,const cv::Mat& img)const;

            void labelDashedLinesColor(const DashedLines &dashedLines, const cv::Mat &img) const;

            void labelSolidLinesColor(const SolidLines &solidLines, const cv::Mat &img) const;

            void labelTerrainsColor(const Terrains &terrains, const cv::Mat &img) const;

            void labelRoadsColor(const Roads &roads, const cv::Mat &img) const;

            void labelWalkwaysColor(const Walkways & walkways,const cv::Mat& img)const;

            void labelOtherLinesColor(const OtherLines& otherLines,const cv::Mat& img)const;

            cv::Mat getOverlayedImage(const cv::Mat& image,const FrameId& curIndex)const;

            void resizeImage(cv::Mat &image) const;

            cv::Mat getBlackImage(const cv::Mat img) const;

            void createFolders()const;

        };
    }

