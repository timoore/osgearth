/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <osgEarth/PowerlineLayer>
#include <osgEarth/AltitudeFilter>
#include <osgEarth/JoinPointsLinesFilter>
#include <osgEarth/ElevationQuery>
#include <osgEarth/PolygonizeLines>
#include <osgEarth/ECEF>

using namespace osgEarth;

#define LC "[PowerlineLayer]"

#define OE_TEST OE_NULL

REGISTER_OSGEARTH_LAYER(PowerlineModel, PowerlineLayer);

PowerlineLayer::Options::Options()
    : FeatureModelLayer::Options()
{
    fromConfig(_conf);
}

PowerlineLayer::Options::Options(const ConfigOptions& options)
    : FeatureModelLayer::Options(options)
{
    fromConfig(_conf);
}

void PowerlineLayer::Options::fromConfig(const Config& conf)
{
    LayerClient<FeatureSource>::fromConfig(conf, "line_features", _lineSourceLayer, _lineSource);
}

Config
PowerlineLayer::Options::getConfig() const
{
    Config conf = FeatureModelLayer::Options::getConfig();
    LayerClient<FeatureSource>::getConfig(conf, "line_features", _lineSourceLayer, _lineSource);
    return conf;
}

void PowerlineLayer::Options::mergeConfig(const Config& conf)
{
    FeatureModelLayer::Options::mergeConfig(conf);
    fromConfig(conf);
}

class PowerlineFeatureNodeFactory : public GeomFeatureNodeFactory
{
public:
    PowerlineFeatureNodeFactory(const PowerlineLayer::Options& options)
        : GeomFeatureNodeFactory(options),
          _lineSourceLayer(options.lineSourceLayer().get()),
          _lineSource(options.lineSource().get())
        {}
    bool createOrUpdateNode(FeatureCursor* cursor, const Style& style,
                            const FilterContext& context,
                            osg::ref_ptr<osg::Node>& node);
private:
    std::string _lineSourceLayer;
    FeatureSource::Options _lineSource;
};

FeatureNodeFactory*
PowerlineLayer::createFeatureNodeFactoryImplementation() const
{
    return new PowerlineFeatureNodeFactory(options());
}

namespace
{
    Feature* getPointFeature(PointMap& pointMap, const osg::Vec3d& key)
    {
        auto itr = findPoint(pointMap, key);
        if (itr == pointMap.end())
        {
            return nullptr;
        }
        else
        {
            return itr->second.pointFeature.get();
        }
    }
    
    FeatureList makeCableFeatures(FeatureList& powerFeatures, FeatureList& towerFeatures, const FilterContext& cx)

    {
        FeatureList result;
        const Session* session = cx.getSession();

        // the map against which we'll be doing elevation clamping
        osg::ref_ptr<const Map> map = session->getMap();
        if (!map.valid())
            return result;

        const SpatialReference* mapSRS = map->getSRS();
        osg::ref_ptr<const SpatialReference> featureSRS = cx.profile()->getSRS();

        // establish an elevation query interface based on the features' SRS.
        ElevationQuery eq(map.get());

        PointMap pointMap;
        for (auto& feature : towerFeatures)
        {
            Geometry* geom = feature->getGeometry();
            for (osg::Vec3d& pt : *geom)
            {
                getPoint(pointMap, pt) = PointEntry(feature);
            }
        }

        const SpatialReference* targetSRS = nullptr;
        if (cx.getSession()->isMapGeocentric())
        {
            targetSRS = cx.getSession()->getMapSRS();
        }
        else
        {
            targetSRS = featureSRS->getGeocentricSRS();
        }
        
        for (auto& feature : powerFeatures)
        {
            Geometry* geom = feature->getGeometry();
            if (geom->getType() == Geometry::TYPE_LINESTRING)
            {
                std::vector<float> elevations;
                eq.getElevations(geom->asVector(), feature->getSRS(), elevations);
                std::vector<osg::Vec3d> worldPts(geom->size());
                std::vector<osg::Matrixd> orientations(geom->size());
                for (int i = 0; i < geom->size(); ++i)
                {
                    osg::Vec3d geodeticPt((*geom)[i].x(), (*geom)[i].y(), elevations[i]);
                    ECEF::transformAndGetRotationMatrix(geodeticPt, featureSRS, worldPts[i],
                                                        targetSRS, orientations[i]);
                }
                // New feature for the cable
                const int size = geom->size();
                const osg::Vec3d attach[] = {{5.876, 0.0, 14.162}, {-5.876, 0.0, 14.162}};
                for (int cable = 0; cable < 2; ++cable)
                {
                    Feature* newFeature = new Feature(*feature);
                    LineString* newGeom = new LineString(size);

                    for (int i = 0; i < size; ++i)
                    {
                        double heading = 0.0;
                        auto itr = findPoint(pointMap, (*geom)[i]);
                        if (itr != pointMap.end())
                        {
                            heading = itr->second.pointFeature->getDouble("heading", 0.0);
                        }
                        osg::Matrixd headingMat;
                        headingMat.makeRotate(osg::DegreesToRadians(heading), osg::Vec3d(0.0, 0.0, 1.0));
                        osg::Vec3d worldAttach = attach[cable] * headingMat * orientations[i] + worldPts[i];
                        osg::Vec3d wgs84; // intermediate point
                        osg::Vec3d mapAttach;
                        featureSRS->getGeographicSRS()->transformFromWorld(worldAttach, wgs84);
                        featureSRS->getGeographicSRS()->transform(wgs84, featureSRS, mapAttach);
                        newGeom->push_back(mapAttach);
                    }
                    newFeature->setGeometry(newGeom);
                    result.push_back(newFeature);
                }

            }
        }
        return result;
    }
}

bool PowerlineFeatureNodeFactory::createOrUpdateNode(FeatureCursor* cursor, const Style& style,
                                                     const FilterContext& context,
                                                     osg::ref_ptr<osg::Node>& node)
{
    FilterContext sharedCX = context;
    FeatureList workingSet; 
    cursor->fill(workingSet);
    osgEarth::Util::JoinPointsLinesFilter pointsLinesFilter;
    pointsLinesFilter.lineSourceLayer() = "lines";
    pointsLinesFilter.lineSource() = _lineSource;
    FilterContext localCX = pointsLinesFilter.push(workingSet, sharedCX);
    // Render towers and lines (cables) seperately
    // Could write another filter for this?
    FeatureList pointSet;
    for (auto& feature : workingSet)
    {
        Geometry* geom = feature->getGeometry();
        if (geom->getType() == Geometry::TYPE_POINTSET)
        {
            pointSet.push_back(feature);
        }
    }
    osg::ref_ptr<FeatureListCursor> listCursor = new FeatureListCursor(pointSet);
    osg::ref_ptr<osg::Node> pointsNode;
    GeomFeatureNodeFactory::createOrUpdateNode(listCursor.get(), style, localCX, pointsNode);
    osg::ref_ptr<osg::Group> results(new osg::Group);
    results->addChild(pointsNode.get());
    FeatureList cableFeatures =  makeCableFeatures(workingSet, pointSet, localCX);

    Style lineStyle;
    osg::ref_ptr<LineSymbol> lineSymbol = lineStyle.getOrCreateSymbol<LineSymbol>();

    PolygonizeLinesFilter polyLineFilter(lineStyle);
    osg::Node* cables = polyLineFilter.push(cableFeatures, localCX);
    results->addChild(cables);
    node = results;
    return true;
}
