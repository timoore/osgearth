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
#ifndef OSG_TDTILESET_OSGTILE_H
#define OSG_TDTILESET_OSGTILE_H 1
#include <osg/Node>
#include <osgEarth/Map>
#include <osgEarth/TileKey>

namespace osgEarth
{
    class OsgTileBuilder
    {
    public:
        OsgTileBuilder(Map* map)
            : _map(map)
        {}
        osg::Node* operator(const TileKey& key);
    private:
        osg::ref_ptr<Map> _map;
        ElevationPool::WorkingSet _workingSet;
        
    };
}
#endif
