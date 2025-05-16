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






#include <algorithm>    // std::max



TEST(projector, classTest) {
    using namespace data_generation;
    Params params;
    params.loadParams();
    params.configPath = "/home/kirik/generator_ws/src/datagenerator_tool/res/config.yaml";
    params.startIdProj =0;

    camera_info camera(params);

    map_handler map(lanelet::projection::UtmProjector(lanelet::Origin({params.baseLat, params.baseLon})),params.laneletMap,
            params);

    data_prep data(params);

    disparity_handler disparityHandler;

    cloud_handler cloudHandler;

    data_generator dataGenerator(camera,data,map,
           disparityHandler,cloudHandler,params);

    dataGenerator.label();


}


/*

TEST(projector, readRosbagFile) {
    using namespace Projector;
    Params params;
    loadParams(params);

    Data data(params.poseFile,params.stampFile,params.timeTolerance, params.timeOffset);

    Map map(lanelet::projection::UtmProjector(lanelet::Origin({params.baseLat, params.baseLon})),
            params.laneletMap);

    // Getting lanelet that the vehicle on
    TimedPoseList  Poses = data.getTimedPoseList();
    Position vehiclePosition = Poses.at(0).second.translation();


    lanelet::BasicPoint2d  vehiclePose2D;
    vehiclePose2D.x()= vehiclePosition.x();
    vehiclePose2D.y()= vehiclePosition.y();
    //std::cout << vehiclePose2D.matrix() << std::endl;



    std::vector<std::pair<double, lanelet::Lanelet>> currentLanelets = lanelet::geometry::findNearest(map.getMap()->laneletLayer,
            vehiclePose2D, 1);

    //std::cout << currentLanelets.at(0).second.attributes() << std::endl;









    // Creating a RoutingGrapf
    lanelet::traffic_rules::TrafficRulesPtr trafficRules;
    lanelet::routing::RoutingGraphUPtr routingGrapf;

    trafficRules= lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);

    routingGrapf = lanelet::routing::RoutingGraph::build(*map.getMap(), *trafficRules);

    lanelet::routing::LaneletPaths  paths = routingGrapf->possiblePaths(currentLanelets.at(0).second,300,0,true);
    std::cout << "paths1 front:  "<<paths.at(1).front().id() << std::endl;
    std::cout << "paths1 back:  "<<paths.at(1).back().id() << std::endl;
    std::cout << "paths front:  "<<paths.at(0).front().id() << std::endl;
    std::cout << "paths back:  "<<paths.at(0).back().id() << std::endl;




    lanelet::LaneletSequence lane = paths.at(0).getRemainingLane(paths.at(0).begin());

    auto trafficLightRegelems = lane.regulatoryElementsAs<lanelet::TrafficLight>();


    for (int j = 0; j < trafficLightRegelems.size(); ++j) {
        std::cout << "tf id " << trafficLightRegelems.at(j)->id() << std::endl;
    }



    //std::cout << "paths 2:  "<<paths.at(1).id() << std::endl;
    lanelet::ConstLanelets reachableSet = routingGrapf->reachableSet(currentLanelets.at(0).second,100,0);

    std::cout << "reachable  paths:  "<<reachableSet.size() << std::endl;
    //for (int j = 0; j < 24; ++j) {
    //    std::cout << reachableSet.at(j).id() << std::endl;
    //}
    //std::cout << reachableSet.at(j).id() << std::endl;

}


*/
/*
TEST(projector, SearchRegionforTrafficlights) {
    using namespace Projector;
    Params params;
    loadParams(params);

    Data data(params.poseFile, params.stampFile, params.timeTolerance, params.timeOffset);

    Map map(lanelet::projection::UtmProjector(lanelet::Origin({params.baseLat, params.baseLon})),
            params.laneletMap);

    // Getting lanelet that the vehicle on
    TimedPoseList Poses = data.getTimedPoseList();

    Pose currentPose = Poses.at(1000).second;



    Position vehiclePosition = Poses.at(1000).second.translation();
    lanelet::BasicPoint2d vehiclePose2D;
    vehiclePose2D.x() = vehiclePosition.x();
    vehiclePose2D.y() = vehiclePosition.y();
    lanelet::BoundingBox2d inRegion = lanelet::BoundingBox2d(lanelet::BasicPoint2d(vehiclePose2D.x(),vehiclePose2D.y()),
            lanelet::BasicPoint2d(vehiclePose2D.x()+100,vehiclePose2D.y()+100));

    auto tls = map.getMap()->polygonLayer.search(inRegion);
    std::cout << tls.size() << std::endl;


    // create an oriented Box
    // get the current and previos pose
    // substact the teta values
    // orient the bb bsed on teta
    //


    // convert radian to degree
    double PI = 3.14;
    auto radian = Poses.at(0).second;
    //auto degree = (radian * 180)/PI;

    Pose currentPose = Poses.at(1).second;
    Pose previosPose = Poses.at(0).second;



}

*/

/*
TEST(projector, OrientedBoundingBox) {

   Eigen::Matrix<double, 3, 3, Eigen::DontAlign> currentPoseRotation = Poses.at(5167).second.rotation();

   std::cout << currentPoseRotation.matrix()<< std::endl;
   Eigen::Matrix<double, 3, 1, Eigen::DontAlign> euler= currentPoseRotation.eulerAngles(0,1,2);

    using namespace Projector;
    Params params;
    loadParams(params);

    Data data(params.poseFile, params.stampFile, params.timeTolerance, params.timeOffset);
    TimedPoseList Poses = data.getTimedPoseList();
    EulerAngles angles = loadEachAngle(params.poseFile);


    Pose previousPose = Poses.at(0).second;
    Pose currentPose = Poses.at(1).second;

    // creating a BoundingBox
    EulerAngle previosAngleTest= {0,0,0};
    EulerAngle currentAngleTest = {0,0,-3.14159};
    Position  pos = {10,10,0};
    //BoundingBox BB= getOrientedBoundingBox(previosAngleTest,currentAngleTest,pos,100,3);

    BoundingBox BB = getOrientedBoundingBox(angles.at(0),angles.at(1),Poses.at(1).second,100,3);


    std::cout << BB.center << std::endl;
    std::cout <<"edge1\n"<<BB.edge1 << std::endl;
    std::cout << "edge2\n "<<BB.edge2 << std::endl;
    std::cout << "edge3 \n"<< BB.edge3 << std::endl;
    std::cout << "edge4\n"<<BB.edge4 << std::endl;

    Position tf = {-100,-10,3};
    if(tf.x() <= std::max(BB.edge1.x(),BB.edge3.x()) && tf.x() >= std::min(BB.edge1.x(),BB.edge3.x()) &&
            tf.y() <= std::max(BB.edge1.y(),BB.edge3.y()) && tf.y() >= std::min(BB.edge1.y(),BB.edge3.y())) {
        std::cout << "in" << std::endl;
    }
    else {
        std::cout << "out " << std::endl;
    }

}
*/
/*
TEST(projector, TriangleBOx) {
    using namespace Projector;
    Params params;
    loadParams(params);

    //auto cam = getCameraCalibration(params.cameraCalibPath,params.cameraFrameId,"vehicle",1.0);
    Data data(params.poseFile,params.stampFile,params.timeTolerance, params.timeOffset);


    CameraInfo camera(params.cameraCalibPath,params.cameraFrameId,"vehicle",1.0);


    Map map(lanelet::projection::UtmProjector(lanelet::Origin({params.baseLat, params.baseLon})),
            params.laneletMap);


    // from origin to vehicle frame
    Pose vehiclePose = data.getPoseByID(100);
    auto tls = map.getLights();

    std::cout << vehiclePose.matrix() << std::endl;
    std::cout << "\n" <<tls.at(0).position  <<  "\n"<<std::endl;

    //Eigen::Isometry3d camPoseMap = vehiclePose * camera.getCamera().cameraPose;
    Eigen::Vector3d tlPoseToCamera = vehiclePose.inverse() *  tls.at(0).position;

    std::cout << tlPoseToCamera.matrix() << std::endl;

}
*/
/*
TEST(projector, isInTriangleRegion) {
    using namespace Projector;
    Params params;
    loadParams(params);

    Data data(params.poseFile,params.stampFile,params.timeTolerance, params.timeOffset);


    CameraInfo camera(params.cameraCalibPath,params.cameraFrameId,"vehicle",1.0);


    Map map(lanelet::projection::UtmProjector(lanelet::Origin({params.baseLat, params.baseLon})),
            params.laneletMap);
    projector test_projector(camera,data,map,params.imageDir);

    bool oguz;
    Pose2d pose1 ={0,0};
    Pose2d pose2= {10,10};
    Pose2d pose3= {-10,10};
    Pose2d pose4 = {5,5};
    //oguz = isInOrientedTriangleRegion(pose1,pose2,pose3,pose4);
    //if (oguz= true){
    //    std::cout << "yes, In Region" << std::endl;
    //}

}

*/
/*
TEST(projector, getLightsbySearchRegion) {
    using namespace Projector;
    Params params;
    loadParams(params);

    Data data(params.poseFile,params.stampFile,params.timeTolerance, params.timeOffset);
    Map map(lanelet::projection::UtmProjector(lanelet::Origin({params.baseLat, params.baseLon})),
            params.laneletMap);

    //Pose vehiclePose = data.getPoseByID(4560);

    //map.getLightsByLanelets(vehiclePose);

    //lanelet::LaneletMapPtr mapPtr;
    lanelet::traffic_rules::TrafficRulesPtr trafficRulesPtr;
    lanelet::routing::RoutingGraphUPtr routingGraphPtr;

    trafficRulesPtr =lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    routingGraphPtr = lanelet::routing::RoutingGraph::build(*map.getMap(), *trafficRulesPtr);

    Pose vehiclePose = data.getPoseByID(4560);

    std::cout << vehiclePose.matrix() << std::endl;

    lanelet::BasicPoint2d currentVehiclePose ={vehiclePose.translation().x(),vehiclePose.translation().y()};


    auto currentLaneletsWithDistances = lanelet::geometry::findNearest(map.getMap()->laneletLayer, currentVehiclePose, 1);

    lanelet::Lanelet nearestLanelet = currentLaneletsWithDistances.at(0).second;




    std::cout << nearestLanelet.id() << std::endl;


    lanelet::routing::LaneletPaths paths = routingGraphPtr->possiblePaths(nearestLanelet,300,0,false);

    std::cout << paths.size() << std::endl;

    for (int k = 0; k < paths.size(); ++k) {


        lanelet::routing::LaneletPath path = paths.at(k);

        std::cout << "path begin:  "  << path.begin()->id() << std::endl;

        lanelet::LaneletSequence lane = path.getRemainingLane(path.begin());
        for (int j = 0; j < lane.size(); ++j) {
            std::cout << "Lane id:  "  << lane.ids().at(j) << std::endl;

            std::cout << lane.begin()->rightBound().id()<< std::endl;
        }

    }

    //auto leftLanelet = routingGraphPtr->adjacentLeft(nearestLanelet);

    r (int j = 0; j < leftLanelet->attributes().size() ; ++j) {
        std::cout << leftLanelet->regulatoryElements().at(j)->attributes() << std::endl;
    }//auto rightLanelet = routingGraphPtr->adjacentRight(nearestLanelet);

    Pose2d area = {20,0};
    lanelet::RegulatoryElementPtrs regElems = map.getMap()->regulatoryElementLayer.search(getSearchRegion(vehiclePose,area));
    LocalTrafficLights results;
    int count  = 0;
    for(auto regElem : regElems){
        if (regElem->hasAttribute(lanelet::AttributeName::Subtype) &&
            regElem->attributes().at(lanelet::AttributeName::Subtype) ==
            lanelet::AttributeValueString::TrafficLight) {

            std::cout << "found traffic light"<< std::endl;

            count ++;
            std::cout <<count << std::endl;
        }
        //results.emplace_back(LocalTrafficLight{l.center()});
    }

    */
/*
TEST(projector, getLightsbySearchRegion) {
    using namespace Projector;
    Params params;
    loadParams(params);

    //Data data(params.poseFile, params.stampFile, params.timeTolerance, params.timeOffset);
    Map map(lanelet::projection::UtmProjector(lanelet::Origin({params.baseLat, params.baseLon})),
            params.laneletMap);

    //Pose vehiclePose = data.getPoseByID(504);

    //Pose2d area = {100, 1000};

    //map.getLightsViaLanelets(vehiclePose,area);


   // lanelet::BoundingBox2d searchRegion = getSearchRegion(vehiclePose, area);
    //lanelet::Lanelets inRegion = map.getMap()->laneletLayer.search(searchRegion);
    //lanelet::RegulatoryElementPtrs inRegion = map.getMap()->regulatoryElementLayer.search(searchRegion);
    lanelet::RegulatoryElementPtr regElems = map.getMap()->regulatoryElementLayer.get(9220619800397638037);


    Positions allPts;
    Lights res;
    //for (const auto& regElem : regElems) {
        const auto& parameters = regElems->getParameters();
        if (regElems->attribute("subtype") == "traffic_light"){
            if(parameters.find("refers") != parameters.end()){// If the regulatory element refers to any physical elements
               for(auto landmarkVariant : parameters.find("refers")->second){
                   landmarkVariant.type();
                   lanelet::ConstPolygon3d landmark = boost::get<lanelet::ConstPolygon3d>(landmarkVariant);
                    std::cout << landmark <<"Traffic light!"<< std::endl;
                    Light l(landmark.basicLineString());
                    res.push_back(l);

                   for(auto pts : landmark){
                       std::cout << pts.basicPoint() <<"Each Point of traffic Light"<< std::endl;
                       allPts.push_back(pts.basicPoint());
                   }

               }
            }
        }

    //}

    for (int j = 0; j < 18; ++j) {
        std::cout <<" point matrix \n "<< res.at(1).centerOfNine() << std::endl;
        break;
    }


}
*/













