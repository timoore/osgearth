/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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

#include <osgEarth/FeatureEditMeshLayer>
#include <osgEarth/FeatureCursor>
#include <osgEarth/Map>
#include <osgEarth/Progress>

using namespace osgEarth;

#define LC "[FeatureEditMeshLayer] "
REGISTER_OSGEARTH_LAYER(featureeditmesh, FeatureEditMeshLayer);
REGISTER_OSGEARTH_LAYER(feature_edit_mesh, FeatureEditMeshLayer);

//........................................................................

void
FeatureEditMeshLayer::Options::fromConfig(const Config& conf)
{
    featureSource().get(conf, "features");
}

Config
FeatureEditMeshLayer::Options::getConfig() const
{
    Config conf = EditMeshLayer::Options::getConfig();
    featureSource().set(conf, "features");
    return conf;
}

//........................................................................

void
FeatureEditMeshLayer::setFeatureSource(FeatureSource* layer)
{
    options().featureSource().setLayer(layer);
}

FeatureSource*
FeatureEditMeshLayer::getFeatureSource() const
{
    return options().featureSource().getLayer();
}

Status
FeatureEditMeshLayer::openImplementation()
{
    Status parent = EditMeshLayer::openImplementation();
    if (parent.isError())
        return parent;

    Status fsStatus = options().featureSource().open(getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    return Status::NoError;
}

Config
FeatureEditMeshLayer::getConfig() const
{
    Config c = EditMeshLayer::getConfig();
    return c;
}

Util::RefVector*
FeatureEditMeshLayer::getOrCreateEditGeometry(float heightScale,
                                              const SpatialReference* srs,
                                              ProgressCallback* progress)
{
    FeatureSource* fs = getFeatureSource();

    if (fs == NULL)
        return 0L;
    Threading::ScopedMutexLock lock(_geometryMutex);
    if (_editGeometry.valid())
    {
        return _editGeometry.get();
    }

    osg::ref_ptr<FeatureCursor> cursor = fs->createFeatureCursor(progress);
    if (cursor.valid())
    {
        osg::ref_ptr<EditVector> edit = new EditVector;
        while (cursor->hasMore())
        {
            Feature* f = cursor->nextFeature();
            if (f && f->getGeometry())
            {
                f->transform(srs);
                edit->push_back(f->getGeometry()->createVec3dArray());
            }
        }
        _editGeometry = edit;
    }
    return _editGeometry.get();
}
