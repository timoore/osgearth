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

#include <osg/io_utils>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/fstream>
#include <osgDB/WriteFile>

#include <osgEarth/Common>
#include <osgEarth/JsonUtils>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <osgEarth/StringUtils>
#include <osgEarth/HTTPClient>
#include <osgEarth/TileHandler>
#include <osgEarth/TileVisitor>
#include <osgEarth/ImageLayer>
#include <osgEarth/ElevationLayer>
#include <osgEarth/TMS>

#include <osgEarth/OGRFeatureSource>

#include <osgEarth/TMSPackager>

#include <iostream>
#include <sstream>
#include <iterator>
#include <unordered_map>

using namespace osgEarth;
using namespace osgEarth::Contrib;
using namespace osgEarth::Util;

#define LC "[osgearth_tdtileset] "

// Variation on TileVisitor, in which the handler is responsible for
// recursing into the children by calling processKey.

template<typename Result> class RecursiveTileVisitor;

// This will do all the work.
template<typename Result>
class RecursiveTileHandler : public osg::Referenced
{
public:
    virtual Result handleTile(const TileKey& key, const RecursiveTileVisitor<Result>& tv) = 0;
    virtual bool hasData( const TileKey& key ) const
    {
        return true;
    }
};

template<typename Result>
class RecursiveTileVisitor : public TileVisitor
{
    // Support for storing implicit tiling coordinates and Morton code
    // in the visitor and restoring it after returning from
    // processKey()
    struct TileCoords
    {
        int lod;
        int x;
        int y;
        TileCoords()
            : lod(0), x(0), y(0)
        {
            
        }
        TileCoords(const TileKey& root, const TileKey& key)
        {
            // First, OE TileKey "coords"
            int deltaLod = key.getLOD() - root.getLOD(); // >= 0!
            int tileSize = 1 << deltaLod;
            // Root base coords at current lod
            int rootX = root.getTileX() * tileSize;
            int rootY = root.getTileY() * tileSize;
            // Flip y for implicit tiling scheme
            x = key.getTileX() - rootX;
            y = tileSize - 1 - (key.getTileY() - rootY);
            lod = deltaLod;
        }
        uint64_t morton() const
        {
            uint64_t result = 0;
            for (int i = 0; i < lod; ++i)
            {
                result = result << 2;
                result |= ((y << 1) >> (lod - 1 - i)) & 0x10;
                result |= (x >> (lod - 1 - i)) & 0x1;
            }
            return result;
        }
    };
    mutable TileCoords tileCoords;
    struct PushTileCoords
    {
        TileCoords& location;
        TileCoords save;
        PushTileCoords(TileCoords& location, const TileCoords& newTileCoords)
            : location(location), save(location)
        {
            location = newTileCoords;
        }
        ~PushTileCoords()
        {
            location = save;
        }
    };
    TileKey _root;
public:
    const TileCoords& getTileCoords() const
    {
        return tileCoords;
    }
        
    void run(const Profile* mapProfile) override
    {
    }

    virtual Result run(const Profile* mapProfile, const TileKey& key)
    {
        _profile = mapProfile;
        _root = key;
        // Reset the progress in case this visitor has been ran before.
        resetProgress();
        estimate();
        return processKey(key);
    }
    
    Result processKey(const TileKey& key) const
    {        
        // If we've been cancelled then just return.
        if (_progress && _progress->isCanceled())
        {        
            return Result();
        }    

        unsigned int x, y, lod;
        key.getTileXY(x, y);
        lod = key.getLevelOfDetail();    

        // Only process this key if it has a chance of succeeding.
        if (_tileHandler && _tileHandler->hasData(key) && intersects(key.getExtent())
            && lod <= _maxLevel)
            
        {
            PushTileCoords pusher(tileCoords, TileCoords(_root, key));
            return _tileHandler->handleTile(key, *this);
        }    
        else
        {
            return Result();
        }
    }
    
    bool processThis(const TileKey& key) const
    {
        return key.getLevelOfDetail() >= _minLevel;
    }
    Result processChild(const TileKey& key, int child) const
    {
        if (key.getLevelOfDetail() < _maxLevel)
        {
            TileKey k = key.createChildKey(child);
            return processKey(k);
        }
        else
        {
            return Result();
        }
    }

    void setTileHandler(RecursiveTileHandler<Result>* handler)
    {
        _tileHandler = handler;
    }
    
protected:
    osg::ref_ptr<RecursiveTileHandler<Result>> _tileHandler;
private:
    using TileVisitor::setTileHandler;
};

struct TileEntry
{
    osg::Matrixd local2world;
    osg::Matrixd local2parent;
    std::string contentPath;
    osgEarth::GeoHeightField* tileHeightField;
};

struct TDTilesetHandler : public RecursiveTileHandler<Json::Value>
{
    std::unordered_map<TileKey, TileEntry> tiles;
    Json::Value handleTile(const TileKey& key, const RecursiveTileVisitor<Json::Value>& tv) override;
};

Json::Value TDTilesetHandler::handleTile(const TileKey& key, const RecursiveTileVisitor<Json::Value>& tv)
{
    GeoExtent extent = key.getExtent();
    Json::Value tileObject;
    Json::Value region(Json::arrayValue);
    region[0u] = osg::DegreesToRadians(extent.west());
    region[1u] = osg::DegreesToRadians(extent.south());
    region[2u] = osg::DegreesToRadians(extent.east());
    region[3u] = osg::DegreesToRadians(extent.north());
    region[4u] = 0.0;
    region[5u] = 0.0;
    tileObject["region"] = region;
    auto coords = tv.getTileCoords();
    std::stringstream contentPath;
    contentPath << coords.lod << "/" << coords.x << "/" << coords.y << ".b3dm";
    Json::Value contentObject(Json::objectValue);
    contentObject["uri"] = contentPath.str();
    tileObject["content"] = contentObject;
    if (key.getLOD() < tv.getMaxLevel())
    {
        Json::Value children(Json::arrayValue);
        for (unsigned int i = 0; i < 4; i++)
        {
            children[i] = tv.processChild(key, i);
        }
        tileObject["children"] = children;
    }
    return tileObject;
}

struct TDTilesetVisitor : public RecursiveTileVisitor<Json::Value>
{
    TDTilesetVisitor(const std::string& rootFolder)
        : rootPath(rootFolder)
    {}
    void run(const Profile* mapProfile) override;
    Json::Value run(const Profile* mapProfile, const TileKey& origin) override;
    std::string rootPath;
};

void TDTilesetVisitor::run(const Profile* mapProfile)
{
    
    Json::Value root;
    Json::Value asset;
    asset["version"] = "1.0";
    root["asset"] = asset;
    std::string tileset = rootPath + "/tileset.json";
    osgDB::ofstream fout(tileset.c_str(), std::ios::out | std::ios::trunc);
    Json::StyledStreamWriter writer;
    writer.write(fout, root);
}

Json::Value TDTilesetVisitor::run(const Profile* mapProfile, const TileKey& origin)
{
    
    Json::Value root;
    Json::Value rootObject;
    Json::Value asset;
    std::string tileset = rootPath + "/tileset.json";
    root = RecursiveTileVisitor::run(mapProfile, origin);
    rootObject["root"] = root;
    asset["version"] = "1.0";
    rootObject["asset"] = asset;
    osgDB::ofstream fout(tileset.c_str(), std::ios::out | std::ios::trunc);
    Json::StyledStreamWriter writer;
    writer.write(fout, rootObject);
    return rootObject;
}

int
usage(const std::string& msg = "")
{
    if(!msg.empty())
    {
        std::cout << msg << std::endl;
    }
    std::cout
        << "\nUSAGE: osgearth_tdtileset <earth_file>"
        << "            <earth_file>                    : earth file defining layers to export (required)\n"
        << "            --out <path>                    : root output folder of the TMS repo (required)\n"
        << "            [--bounds xmin ymin xmax ymax]* : bounds to package (in map coordinates; default=entire map)\n"
        << "            [--max-level <num>]             : max LOD level for tiles (all layers; default=inf)\n"
        << std::endl;
    return -1;
}

/** Finds an argument with the specified extension. */
std::string
findArgumentWithExtension( osg::ArgumentParser& args, const std::string& ext )
{
    for( int i = 0; i < args.argc(); ++i )
    {
        std::string arg( args.argv()[i] );
        if( endsWith( toLower( trim( arg ) ), ".earth" ) )
            return arg;
    }
    return "";
}

int
makeTDTileset(osg::ArgumentParser& args)
{
    osgDB::Registry::instance()->getReaderWriterForExtension("png");
    osgDB::Registry::instance()->getReaderWriterForExtension("jpg");
    osgDB::Registry::instance()->getReaderWriterForExtension("tiff");
    osgDB::Registry::instance()->getReaderWriterForExtension("gltf");

        //Read the min level
    unsigned int minLevel = 0;
    while (args.read("--min-level", minLevel));

    //Read the max level
    unsigned int maxLevel = 5;
    while (args.read("--max-level", maxLevel));


    std::vector< Bounds > bounds;
    // restrict packaging to user-specified bounds.
    double xmin=DBL_MAX, ymin=DBL_MAX, xmax=DBL_MIN, ymax=DBL_MIN;
    while (args.read("--bounds", xmin, ymin, xmax, ymax))
    {
        Bounds b;
        b.xMin() = xmin, b.yMin() = ymin, b.xMax() = xmax, b.yMax() = ymax;
        bounds.push_back(b);
    }

    std::string tileList;
    while (args.read("--tiles", tileList));

    bool verbose = args.read("--verbose");

    // load up the map
    osg::ref_ptr<MapNode> mapNode = MapNode::load(args);
    if(!mapNode.valid())
        return usage("Failed to load a valid .earth file");
    // find a .earth file on the command line
    std::string earthFile = findArgumentWithExtension(args, ".earth");

    // folder to which to write the tileset
    std::string rootFolder;
    if(!args.read("--out", rootFolder))
        rootFolder = Stringify() << earthFile << ".3dtiles";

    std::string dbOptions;
    args.read("--db-options", dbOptions);
    std::string::size_type n = 0;
    while((n = dbOptions.find('"', n)) != dbOptions.npos)
    {
        dbOptions.erase(n, 1);
    }

    double x, y;
    bool hasOrigin = false;
    while(args.read("--origin", x, y))
    {
        hasOrigin = true;
    }
    osg::ref_ptr<osgDB::Options> options = new osgDB::Options(dbOptions);

    // create a folder for the output
    osgDB::makeDirectory(rootFolder);
    if(!osgDB::fileExists(rootFolder))
        return usage("Failed to create root output folder");

    Map* map = mapNode->getMap();

    osg::ref_ptr<TDTilesetVisitor > visitor = new TDTilesetVisitor(rootFolder);
    visitor->setTileHandler(new TDTilesetHandler);
    osg::ref_ptr< ProgressCallback > progress = new ConsoleProgressCallback();

    if (verbose)
    {
        visitor->setProgressCallback(progress.get());
    }

    visitor->setMinLevel(minLevel);
    visitor->setMaxLevel(maxLevel);
    if (hasOrigin)
    {
        TileKey originKey = map->getProfile()->createTileKey(x, y, minLevel);
        visitor->run(map->getProfile(), originKey);
    }
    else
    {
        visitor->run(map->getProfile());        
    }
    return 0;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser args( &argc, argv );
    return makeTDTileset(args);
}
