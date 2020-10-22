#include "osgtile.h"

using namespace osgEarth;

osg::Node* OsgTileBuilder::operator()(Map* map, const TileKey& key)
{
    ElevationPool* pool = map->getElevationPool();
    osg::ref_ptr<ElevationTexture> out_elev;
    if (pool->getTile(key, false, out_elev, _workingSet, nullptr))
    {
        
    }
}
