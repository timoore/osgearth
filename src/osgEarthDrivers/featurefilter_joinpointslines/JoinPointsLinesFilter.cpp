/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2008-2019 Pelican Mapping
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
#include "JoinPointsLinesFilterOptions"

#include <osgEarth/Filter>
#include <osgEarth/FeatureCursor>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/FeatureSource>
#include <osgEarth/FilterContext>
#include <osgEarth/Geometry>

#include <map>
#include <algorithm>

#define LC "[JointPointsLines FeatureFilter] "

using namespace osgEarth;
using namespace osgEarth::Drivers;

/**
   Add features from line strings to point features that make up their
   geometry. One use is to join Open Street Map nodes with the ways
   that contain them, a relation that is lost by OGR.

   Optionally this filter calculates a "heading" attribute for each
   point, which is the average (half-vector) of lines entering them.
 */

struct PointEntry
{
    PointEntry() {}
    PointEntry(Feature* feature) : pointFeature(feature) {}
    osg::ref_ptr<Feature> pointFeature;
    FeatureList lineFeatures;
    std::vector<osg::Vec2d> neighbors;
};

typedef std::map<osg::Vec2d, PointEntry> PointMap;

double calculateGeometryHeading(const osg::Vec2d& point, const std::vector<osg::Vec2d>& neighbors,
                                FilterContext& context)
{
    const SpatialReference* targetSRS = nullptr;
    if (context.getSession()->isMapGeocentric())
    {
        targetSRS = context.getSession()->getMapSRS();
    }
    else
    {
        targetSRS = context.profile()->getSRS()->getGeocentricSRS();
    }
    osg::Vec3d point3d(point, 0.0);
    std::vector<osg::Vec3d> neighbor3d(neighbors.size());
    std::transform(neighbors.begin(), neighbors.end(), neighbor3d.begin(),
                   [](osg::Vec2d& in) { return osg::Vec3d(in, 0.0); });

    osg::Matrixd orientation;
    osg::Vec3d world3d;
    
    ECEF::transformAndGetRotationMatrix(point3d, context.profile()->getSRS(), world3d,
                                        targetSRS, orientation);
    // XXX OSG bug weirdness
    osg::Matrixd toLocal(orientation);
    toLocal.transpose3x3(toLocal);
    std::vector<osg::Vec3d> neighborWorld(neighbors.size());

    for (int i = 0; i < neighbors.size(); ++i)
    {
        ECEF::transformAndLocalize(neighbor3d[i], context.profile()->getSRS(), neighborWorld[i],
                                   targetSRS);
    }
    osg::Vec3d halfVec(0.0, 0.0, 0.0);
    for (osg::Vec3d& neighWorld : neighborWorld)
    {
        osg::Vec3d directionWorld = neighWorld - world3d;
        osg::Vec3d direction = directionWorld * toLocal;
        direction.z() = 0.0;
        direction.normalize();
        halfVec += direction;
    }
    // The half vector points "half way" between the in and
    // out vectors. If they are parallel, then it will have 0
    // length.
    double heading;
    double halfLen = half.normalize();
    if (osg::equivalent(halfLen, 0.0))
    {
        
    }
    else
    {
        // Heading we want is rotated 90 degrees from half vector
        heading = std::atan2(-halfVec[1], -halfVec[0]);
     
    }
    return osg::RadiansToDegrees(heading);
}

class JoinPointsLinesFilter : public FeatureFilter, public JoinPointsLinesFilterOptions
{
public:
    JoinPointsLinesFilter(const ConfigOptions& options)
        : FeatureFilter(), JoinPointsLinesFilterOptions(options)
    {
    }

public:
    Status initialize(const osgDB::Options* readOptions)
    {
        return Status::OK();
    }

    FilterContext push(FeatureList& input, FilterContext& context)
    {
        PointMap pointMap;
        
        for (auto pFeature : input)
        {
            Feature* feature = pFeature.get();
            Geometry* geom = feature->getGeometry();
            if (geom->getType() == Geometry::TYPE_POINTSET)
            {
                // Are there multiple points? Does it matter?
                for (osg::Vec3d& pt : *geom)
                {
                    osg::Vec2d key(pt.x(), pt.y());
                    pointMap[key] = PointEntry(feature);
                }
            }
        }

        for (auto pFeature : input)
        {
            Feature* feature = pFeature.get();
            Geometry* geom = feature->getGeometry();
            if (geom->getType() == Geometry::TYPE_LINESTRING)
            {
                const int size = geom->size();
                for (int i = 0; i < size; ++i)
                {
                    osg::Vec2d key((*geom)[i].x(), (*geom)[i].y());
                    auto ptItr = pointMap.find(key);
                    if (ptItr != pointMap.end())
                    {
                        PointEntry &point = ptItr->second;
                        point.lineFeatures.push_back(feature);
                        if (i > 0)
                        {
                            point.neighbors.push_back(osg::Vec2d((*geom)[i - 1].x(),
                                                                 (*geom)[i - 1].y()));
            
                        }
                        if (i < size - 1)
                        {
                            point.neighbors.push_back(osg::Vec2d((*geom)[i + 1].x(),
                                                                 (*geom)[i + 1].y()));
                        }
                    }
                }
            }
        }
        for (auto& kv : pointMap)
        {
            PointEntry& entry = kv.second;
            Feature* pointFeature = entry.pointFeature.get();
            for (auto& lineFeature : entry.lineFeatures)
            {
                const AttributeTable& attrTable = lineFeature->getAttrs())
                for (const auto& attrEntry : attrTable)
                {
                    pointFeature->set(attrEntry.first, attrEntry.second);
                }
            }
            
        }
        return context;
    }

    class JoinFeatureFilterPlugin : public FeatureFilterDriver
{
public:
    JoinFeatureFilterPlugin() : FeatureFilterDriver()
    {
        this->supportsExtension("osgearth_featurefilter_join", className() );
    }
    
    const char* className() const
    {
        return "JoinFeatureFilterPlugin";
    }

    ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new JoinFeatureFilter( getConfigOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_featurefilter_join, JoinFeatureFilterPlugin);

};
