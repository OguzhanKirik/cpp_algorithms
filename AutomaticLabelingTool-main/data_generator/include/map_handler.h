#pragma once


// Own classes
#include "data_type.h"


// MRT libraries
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/utility/Utilities.h>


namespace data_generation {

    class map_handler : public lanelet::RegulatoryElement{
    public:

    /**
    * @brief this contructor loads the high-definition map
     * Map elements: traffic light, dashed line, solid line, other line, road, terrain, walkways are to obtain
     * @param params.baseLat  origin point lateral
     * @param params.baseLon origin point longtual
     * @param params.laneletMap path to HD map
     * @param area set the size of bounding box
     *
     * @param Lights contain traffic light element
     * @param DashedLines contain dashed line element
     * @param Terrains contain terrrain element
     * @param Roads contain road element
     * @param SolidLines contain solid line element
     * @param OtherLines contain other line elemen
     * @param Walkways contain walkway element
    */
        map_handler(const lanelet::projection::UtmProjector &projector,
                    const std::string& mapPath,
                    const Params &params_);
        ~map_handler();

    /**
    * @brief Returns the HD map
    */
        const lanelet::LaneletMapUPtr &getMap() const {
            return mapPtr_;
        }
    /**
    * @brief Returns map elements within the bounding box
    */
        MapElements getMapElements(const Pose& cameraPose) const;
    /**
    * @brief Returns a bounding box created based on the given size
    */
        lanelet::BoundingBox2d getSearchRegion(const Pose& vehiclePose, const Pose2d& area)const;

    private:
        lanelet::LaneletMapUPtr mapPtr_;
        lanelet::traffic_rules::TrafficRulesPtr trafficRulesPtr_;
        lanelet::routing::RoutingGraphUPtr routingGraphPtr_;
        std::unique_ptr<const lanelet::Projector> projector_;
        Area area;
        const Params &params;


    private:
        /**
        * @brief Returns the traffic lights in the search region
        */
        Lights getLights() const;
        /**
        * @brief Returns the dashed lines in the search region
        */
        DashedLines getDashedLines(const Pose& cameraPose)const;
        /**
        * @brief Returns the terrains in the search regionze
        */
        Terrains getTerrains(const Pose& cameraPose)const;
        /**
        * @brief Returns the Roads in the search region
        */
        Roads getRoads(const Pose& cameraPose)const;
        /**
        * @brief Returns the solid lines in the search region
        */
        SolidLines getSolidLines(const Pose& cameraPose)const;
        /**
        * @brief Returns the walkways in the search region
        */
        Walkways getWalkways(const Pose& cameraPose)const;
        /**
        * @brief Returns the other lines in the search region
        */
        OtherLines getOtherLines(const Pose& cameraPose)const;
    };
}

