

#include <data_type.h>
#include "map_handler.h"


namespace data_generation {
    //map_handler::map_handler(){};

    map_handler::map_handler(
            const lanelet::projection::UtmProjector &projector,
            const std::string &mapPath,
            const Params& params_):
            params(params_),
            mapPtr_(lanelet::load(mapPath, projector)),
            trafficRulesPtr_(lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle)),
            routingGraphPtr_(lanelet::routing::RoutingGraph::build(*mapPtr_, *trafficRulesPtr_)){
        area = {params_.x_axis,params_.y_axis};
        std::cout <<"Map is ready"<< std::endl;
    }

    map_handler::~map_handler() {

        std::cout << "map is destroyed" << std::endl;
    }

    Lights map_handler::getLights() const {
        Lights results;
        for (const auto &regElem : mapPtr_->polygonLayer) {
            if (regElem.hasAttribute(lanelet::AttributeName::Type) &&
                regElem.attributes().at(lanelet::AttributeName::Type) ==
                lanelet::AttributeValueString::TrafficLight) {
                if (regElem.size() != 4) {
                    throw std::runtime_error("Can't deal with traffic lights of size != 4");
                }

                Light l(regElem.basicLineString());
                results.push_back(l);
            }
        }
        return results;
    }


    DashedLines map_handler::getDashedLines(const Pose& cameraPose)const {
        DashedLines  res;
        lanelet::BoundingBox2d searchRegion = getSearchRegion(cameraPose,area);
        lanelet::Areas inRegion = mapPtr_->areaLayer.search(searchRegion);
        for (const auto &elem : inRegion) {
            if (elem.hasAttribute(lanelet::AttributeName::Subtype)
                && elem.attributes().at(lanelet::AttributeName::Subtype) == "line_dashed") {
                DashedLine dl(elem.outerBoundPolygon().basicPolygon());

                res.emplace_back(dl);
            }
        }
        return res;
    }


    SolidLines map_handler::getSolidLines(const Pose& cameraPose)const {
        SolidLines  res;
        lanelet::BoundingBox2d searchRegion = getSearchRegion(cameraPose,area);
        lanelet::Areas inRegion = mapPtr_->areaLayer.search(searchRegion);
        for (const auto &elem : inRegion) {
            if (elem.hasAttribute(lanelet::AttributeName::Subtype)
                && elem.attributes().at(lanelet::AttributeName::Subtype) == "line_solid") {
                SolidLine sl(elem.outerBoundPolygon().basicPolygon());
                res.emplace_back(sl);
            }
        }
        return res;
    }

    OtherLines map_handler::getOtherLines(const Pose& cameraPose)const {
        OtherLines  res;
        lanelet::BoundingBox2d searchRegion = getSearchRegion(cameraPose,area);
        lanelet::Areas inRegion = mapPtr_->areaLayer.search(searchRegion);
        for (const auto &elem : inRegion) {
            if (elem.hasAttribute(lanelet::AttributeName::Subtype)
                && elem.attributes().at(lanelet::AttributeName::Subtype) == "other_lines") {
                OtherLine ol(elem.outerBoundPolygon().basicPolygon());
                res.emplace_back(ol);
            }
        }
        return res;
    }


    Roads map_handler::getRoads(const Pose& cameraPose)const {
        Roads  res;
        lanelet::BoundingBox2d searchRegion = getSearchRegion(cameraPose,area);
        lanelet::Areas inRegion = mapPtr_->areaLayer.search(searchRegion);
        for (const auto &elem : inRegion) {
            if (elem.hasAttribute(lanelet::AttributeName::Subtype)
                && elem.attributes().at(lanelet::AttributeName::Subtype) == "road") {
                Road r(elem.outerBoundPolygon().basicPolygon());
                res.emplace_back(r);
            }
        }
        return res;
    }

    Terrains map_handler::getTerrains(const Pose& cameraPose)const {
        Terrains  res;
        lanelet::BoundingBox2d searchRegion = getSearchRegion(cameraPose,area);
        lanelet::Areas inRegion = mapPtr_->areaLayer.search(searchRegion);
        for (const auto &elem : inRegion) {
            if (elem.hasAttribute(lanelet::AttributeName::Subtype)
                && elem.attributes().at(lanelet::AttributeName::Subtype) == "terrain") {
                Terrain t(elem.outerBoundPolygon().basicPolygon());

                //std::cout << elem.outerBoundPolygon().size() << std::endl;
                //for (int j = 0; j < elem.outerBoundPolygon().size(); ++j) {
                //    std::cout << elem.outerBoundPolygon().basicPolygon().at(j)<< std::endl;
                //}
                res.emplace_back(t);
            }
        }
        return res;
    }

    Walkways map_handler::getWalkways(const Pose& cameraPose)const {
        Walkways  res;
        lanelet::BoundingBox2d searchRegion = getSearchRegion(cameraPose,area);
        lanelet::Areas inRegion = mapPtr_->areaLayer.search(searchRegion);
        for (const auto &elem : inRegion) {
            if (elem.hasAttribute(lanelet::AttributeName::Subtype)
                && elem.attributes().at(lanelet::AttributeName::Subtype) == "walkway") {
                Walkway w(elem.outerBoundPolygon().basicPolygon());
                res.emplace_back(w);
            }
        }
        return res;
    }



    MapElements map_handler::getMapElements(const Pose& cameraPose) const {
        MapElements mapElements;
        if(params.element.trafficLight == "true"){
            mapElements.lights = getLights();
        }
        /*
        if(elements.trafficSign == "true"){
            mapElements.signs = getSigns();
        }
        */
        if(params.element.dashedLines == "true"){
            mapElements.dashedLines = getDashedLines(cameraPose);
        }
        if(params.element.solidLines == "true"){
            mapElements.solidLines = getSolidLines(cameraPose);
        }
        if(params.element.roads == "true"){
            mapElements.roads = getRoads(cameraPose);
        }
        if(params.element.terrains == "true"){
            mapElements.terrains = getTerrains(cameraPose);
        }
        if(params.element.walkways == "true"){
            mapElements.walkways = getWalkways(cameraPose);
        }
        if(params.element.otherLines == "true"){
            mapElements.otherLines = getOtherLines(cameraPose);
        }



        return mapElements;

    }

    lanelet::BoundingBox2d map_handler::getSearchRegion(const Pose& vehiclePose, const Pose2d& area)const{
        lanelet::BasicPoint2d regionRearPoints = {vehiclePose.translation().x()- area.x(),vehiclePose.translation().y()-area.y()};

        //std::cout <<"regionRearPoints" << regionRearPoints << std::endl;
        lanelet::BasicPoint2d regionFrontPoints = {vehiclePose.translation().x()+area.x(),vehiclePose.translation().y()+area.y()};

        //std::cout <<"regionFrontPoints" <<regionFrontPoints << std::endl;

        lanelet::BoundingBox2d searchRegion = {regionRearPoints,regionFrontPoints};
        return searchRegion;
    }

}