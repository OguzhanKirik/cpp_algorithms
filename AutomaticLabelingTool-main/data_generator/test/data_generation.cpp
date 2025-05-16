#include "gtest/gtest.h"
#include "disparity_handler.h"
#include "cloud_handler.h"
#include "data_type.h"
#include "data_generator.h"
#include "map_handler.h"
#include "camera_info.h"
#include "data_prep.h"





#include <algorithm>    // std::max


#include "gtest/gtest.h"

// A google test function (uncomment the next function, add code and
// change the names TestGroupName and TestName)
// TEST(${pkgname}, TestName) {
//     EXPECT_TRUE(true);
//     TODO: Add your test code here
//}







/*
TEST(data_generation, classTest) {

    // set verbosity level
    using namespace data_generation;

    Params params;
    params.loadParams();
    params.startIdProj =160;




    camera_info camera(params);

    map_handler map(lanelet::projection::UtmProjector(lanelet::Origin({params.baseLat, params.baseLon})),
            params.laneletMap);

    MapElements mapElements = map.getMapElements(params.element);

    data_prep data(params);

    disparity_handler disparityHandler;

    cloud_handler cloudHandler;

    data_generator dataGenerator(camera,data,disparityHandler,cloudHandler,params);

    dataGenerator.label(mapElements);





}

*/


