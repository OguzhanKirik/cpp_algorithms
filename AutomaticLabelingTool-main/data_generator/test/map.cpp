// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
#include "gtest/gtest.h"
#include "disparity_handler.h"
#include "cloud_handler.h"
#include "data_type.h"
#include "data_generator.h"
#include "map_handler.h"
#include "camera_info.h"
#include "data_prep.h"

/*
TEST(map, getMapElements) {
    using namespace data_generation;
    Params params;
    params.loadParams();


    lanelet::LaneletMapUPtr mapPtr_ = lanelet::load(params.laneletMap,
            lanelet::projection::UtmProjector(lanelet::Origin({params.baseLat, params.baseLon})));


    std::cout << mapPtr_->areaLayer.size() << std::endl;
    for (const auto &elem : mapPtr_->areaLayer) {
        if (elem.hasAttribute(lanelet::AttributeName::Subtype)){
            /*
            if(elem.attributes().at(lanelet::AttributeName::Subtype) == "road"){
                std::cout <<"road point size" <<elem.outerBoundPolygon().basicPolygon().size() << std::endl;
            }

            if(elem.attributes().at(lanelet::AttributeName::Subtype) == "terrain"){
                std::cout <<"terrain point size" <<elem.outerBoundPolygon().basicPolygon().size() << std::endl;
            }
            if(elem.attributes().at(lanelet::AttributeName::Subtype) == "line_solid"){
                std::cout <<"line solid point size" <<elem.outerBoundPolygon().basicPolygon().size() << std::endl;
            }
             */
            if(elem.attributes().at(lanelet::AttributeName::Subtype) == "walkway"){
                std::cout <<"walkway point size" <<elem.outerBoundPolygon().basicPolygon().size() << std::endl;
            }



        }


    }

*/
    /*
    DashedLines dls;

    std::cout << mapPtr_->areaLayer.size() << std::endl;
    for (const auto &elem : mapPtr_->areaLayer) {
        if (elem.hasAttribute(lanelet::AttributeName::Subtype)
                 && elem.attributes().at(lanelet::AttributeName::Subtype) == "line_dashed")
        {
            DashedLine dl(elem.outerBoundPolygon().basicPolygon());

            //std::cout << elem.outerBoundPolygon().size() << std::endl;
            //for (int j = 0; j < elem.outerBoundPolygon().size(); ++j) {
            //    std::cout << elem.outerBoundPolygon().basicPolygon().at(j)<< std::endl;
            //}

            dls.emplace_back(dl);

        }

    }


    */


}
