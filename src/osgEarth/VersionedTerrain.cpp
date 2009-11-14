/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarth/VersionedTerrain>
#include <osgEarth/Registry>
#include <osgEarth/Locators>
#include <osgEarth/EarthTerrainTechnique>
#include <osgEarth/Map>
#include <osgEarth/MapEngine>
#include <OpenThreads/ScopedLock>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Texture2D>

using namespace osgEarth;
using namespace OpenThreads;

//#define PREEMPTIVE_DEBUG 1

// this progress callback checks to see whether the request being serviced is 
// out of date with respect to the task service that is running it. It checks
// for a disparity in frame stamps, and reports that the request should be
// canceled if it appears the request has been abandoned by the Tile that
// originally scheduled it.
struct StampedProgressCallback : ProgressCallback
{
public:
    StampedProgressCallback(TaskRequest* request, TaskService* service):
      _request(request),
      _service(service)
    {
    }

    //todo: maybe we should pass TaskRequest in as an argument 
    bool reportProgress(double current, double total)
    {
        //Check to see if we were marked cancelled on a previous check
        if (_canceled) return _canceled;

        //osg::ref_ptr<TaskRequest> safeRequest = _request.get();
        //osg::ref_ptr<TaskService> safeService = _service.get();

        //if ( !safeReqeust.valid() || !safeService.valid() )
        //    return true;

        _canceled = (_service->getStamp() - _request->getStamp() > 2);

        //osg::notify(osg::NOTICE) << "Marking cancelled " << _request->getName() << std::endl;
        return _canceled;
    }

    //// don't make these ref_ptrs.. that's a leaker
    //osg::observer_ptr<TaskRequest> _request;
    //osg::observer_ptr<TaskService> _service;
    TaskRequest* _request;
    TaskService* _service;
};


/*****************************************************************************/

struct TileLayerRequest : public TaskRequest
{
    TileLayerRequest( const TileKey* key, Map* map, MapEngine* engine )
        : _key( key ), _map(map), _engine(engine) { }

    osg::ref_ptr<const TileKey> _key;
    osg::ref_ptr<Map>           _map;
    osg::ref_ptr<MapEngine>     _engine;
};

struct TileColorLayerRequest : public TileLayerRequest
{
    TileColorLayerRequest( const TileKey* key, Map* map, MapEngine* engine, unsigned int layerId )
        : TileLayerRequest( key, map, engine ), _layerId(layerId) { }

    void operator()( ProgressCallback* progress )
    {
        osg::ref_ptr<MapLayer> mapLayer = 0L;
        {
            ScopedReadLock lock( _map->getMapDataMutex() );
            for (unsigned int i = 0; i < _map->getImageMapLayers().size(); ++i)
            {
                if ( _map->getImageMapLayers()[i]->getId() == _layerId)
                {
                    mapLayer = _map->getImageMapLayers()[i].get();
                }
            }            
        }
        if ( mapLayer.valid() )
        {
            osg::ref_ptr<GeoImage> image = mapLayer->createImage( _key.get(), progress );
            if ( image.get() )
                _result = _engine->createImageLayer( _map.get(), _key.get(), image.get() );
        }
    }
    unsigned int _layerId;
};

struct TileElevationLayerRequest : public TileLayerRequest
{
    TileElevationLayerRequest( const TileKey* key, Map* map, MapEngine* engine )
        : TileLayerRequest( key, map, engine )
    {
    }

    void operator()( ProgressCallback* progress )
    {
        _result = _engine->createHeightFieldLayer( _map.get(), _key.get(), true ); //exactOnly=true
    }
};

struct TileElevationPlaceholderLayerRequest : public TileLayerRequest
{
    TileElevationPlaceholderLayerRequest( const TileKey* key, Map* map, MapEngine* engine, GeoLocator* keyLocator )
        : TileLayerRequest( key, map, engine ),
          _keyLocator(keyLocator)
    {
        _parentKey = key->createParentKey();
    }

    void setParentHF( osg::HeightField* parentHF )
    {
        _parentHF = parentHF; 
    }

    void setNextLOD( int nextLOD )
    {
        _nextLOD = nextLOD;
    }

    void operator()( ProgressCallback* progress )
    {
        _result = _engine->createPlaceholderHeightfieldLayer(
            _parentHF.get(),
            _parentKey.get(),
            _key.get(),
            _keyLocator.get() );
    }

    osg::ref_ptr<osg::HeightField> _parentHF;
    osg::ref_ptr<const TileKey> _parentKey;
    osg::ref_ptr<GeoLocator>    _keyLocator;
    int _nextLOD;
};

// A task request that rebuilds a tile's terrain technique in the background. It
// re-init's the geometry but does NOT swap the buffers (since this constitutes
// altering the scene graph and must therefore be done in the update traversal).
//
// NOTE! this doesn't work for multipass technique!
struct TileGenRequest : public TaskRequest
{
    TileGenRequest( osgTerrain::TerrainTechnique* tech ) :
        _tech( dynamic_cast<EarthTerrainTechnique*>(tech) ) { }

    void operator()( ProgressCallback* progress )
    {
        if ( _tech.valid() )
            _tech->init( false );
    }

    osg::ref_ptr<EarthTerrainTechnique> _tech;
    //osg::observer_ptr<EarthTerrainTechnique> _tech;
};


/*****************************************************************************/

// family tile indicies in the _family vector
#define PARENT 0
#define WEST   1
#define NORTH  2
#define EAST   3
#define SOUTH  4


VersionedTile::VersionedTile( const TileKey* key, GeoLocator* keyLocator ) :
_key( key ),
_keyLocator( keyLocator ),
_useLayerRequests( false ),       // always set this to false here; use setUseLayerRequests() to enable
_terrainRevision( -1 ),
_tileRevision( 0 ),
_geometryRevision( 0 ),
_requestsInstalled( false ),
_elevationLayerDirty( false ),
_colorLayersDirty( false ),
_usePerLayerUpdates( false ),     // only matters when _useLayerRequests==true
_elevationLayerUpToDate( true ),
_elevationLOD( key->getLevelOfDetail() ),
_tileRegisteredWithTerrain( false ),
_useTileGenRequest( true ),
_tileGenNeeded( false ),
_needsUpdate(false)
{
    this->setThreadSafeRefUnref( true );

    this->setTileID( key->getTileId() );

    // because the lowest LOD (1) is always loaded fully:
    _elevationLayerUpToDate = _key->getLevelOfDetail() <= 1;

    // initially bump the update requirement so that this tile will receive an update
    // traversal the first time through. It is on the first update traversal that we
    // know the tile is in the scene graph and that it can be registered with the terrain.
    this->setNumChildrenRequiringUpdateTraversal(
        this->getNumChildrenRequiringUpdateTraversal() + 1 );
}

VersionedTile::~VersionedTile()
{
    //osg::notify(osg::NOTICE) << "Destroying VersionedTile " << this->getKey()->str() << std::endl;
}

bool
VersionedTile::cancelRequests()
{
    // This method ensures that all requests owned by this object are stopped and released
    // by the corresponding task service prior to destructing the tile. Called by
    // VersionedTerrain::updateTileTable().

    bool done = true;

    //Cancel any pending requests
    if (_requestsInstalled)
    {
        for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
        {
            //if (i->get()->getState() == TaskRequest::STATE_IN_PROGRESS)
            //{
            //    osg::notify(osg::NOTICE) << "IR (" << _key->str() << ") in progress, cancelling " << std::endl;
            //}
            i->get()->cancel();

            if ( i->get()->isRunning() )
            {
                //osg::notify(osg::NOTICE) << "Tile ("<<_key->str()<<") IR still running ... " << std::endl;
                done = false;
            }
            //if ( i->get()->referenceCount() > 1 )
            //    done = false;
        }

        if ( _elevRequest.valid() )
        {
            _elevRequest->cancel();

            if ( _elevRequest->isRunning() )
            {
                //osg::notify(osg::NOTICE) << "Tile ("<<_key->str()<<") ER still running ... " << std::endl;
                done = false;
            }
            //if ( _elevRequest->referenceCount() > 1 )
            //    done = false;
        }

        if (_elevPlaceholderRequest.valid())
        {
            _elevPlaceholderRequest->cancel();

            if ( _elevPlaceholderRequest->isRunning() )
            {
                //osg::notify(osg::NOTICE) << "Tile ("<<_key->str()<<") PH still running ... " << std::endl;
                done = false;
            }
            //if ( _elevPlaceholderRequest->referenceCount() > 1 )
            //    done = false;
        }

        if (_tileGenRequest.valid())
        {
            _tileGenRequest->cancel();

            if ( _tileGenRequest->isRunning() )
            {
                //osg::notify(osg::NOTICE) << "Tile ("<<_key->str()<<") TG still running ... " << std::endl;
                done = false;
            }
            //if ( _tileGenRequest->referenceCount() > 1 )
            //    done = false;
        }
    }

    return done;
}


OpenThreads::ReadWriteMutex&
VersionedTile::getTileLayersMutex()
{
    return _tileLayersMutex;
}

const TileKey*
VersionedTile::getKey() const
{
    return _key.get();
}

void
VersionedTile::setElevationLOD( int lod )
{
    _elevationLOD = lod;
}

int
VersionedTile::getElevationLOD() const
{
    return _elevationLOD;
}

VersionedTerrain*
VersionedTile::getVersionedTerrain()
{
    if ( !_versionedTerrain.valid() )
        _versionedTerrain = static_cast<VersionedTerrain*>(getTerrain());
    return _versionedTerrain.get();
}

const VersionedTerrain*
VersionedTile::getVersionedTerrain() const
{
    return const_cast<VersionedTile*>(this)->getVersionedTerrain();
}

void
VersionedTile::setUseLayerRequests( bool value )
{
    _useLayerRequests = value;
}

int
VersionedTile::getTerrainRevision() const
{
    return _terrainRevision;
}

void
VersionedTile::setTerrainRevision( int revision )
{
    _terrainRevision = revision;
}

bool
VersionedTile::isInSyncWithTerrain() const
{
    return _terrainRevision == getVersionedTerrain()->getRevision();
}

int
VersionedTile::getTileRevision() const
{
    return _tileRevision;
}

void
VersionedTile::incrementTileRevision()
{
    _tileRevision++;
}

void
VersionedTile::setHasElevationHint( bool hint ) 
{
    _hasElevation = hint;
}

bool
VersionedTile::isElevationLayerUpToDate() const 
{
    return _elevationLayerUpToDate;
}

bool
VersionedTile::getTileGenNeeded() const
{
    return _tileGenNeeded;
}

void
VersionedTile::setTileGenNeeded( bool tileGenNeeded )
{
    _tileGenNeeded = tileGenNeeded;
}

bool
VersionedTile::getUseTileGenRequest() const
{
    return _useTileGenRequest;
}

// returns TRUE if it's safe for this tile to load its next elevation data layer.
bool
VersionedTile::readyForNewElevation()
{
    bool ready = true;

    if ( _elevationLOD == _key->getLevelOfDetail() )
    {
        ready = false;
    }
    else if ( _family[PARENT].elevLOD < 0 )
    {
        ready = false;
    }
    else
    {
        for( int i=PARENT; i<=SOUTH; i++) 
        {
            if ( _family[i].expected && _family[i].elevLOD >= 0 && _family[i].elevLOD < _elevationLOD )
            {
                ready = false;
                break;
            }
        }

        // if the next LOD is not the final, but our placeholder is up to date, we're not ready.
        if ( ready && _elevationLOD+1 < _key->getLevelOfDetail() && _elevationLOD == _family[PARENT].elevLOD )
        {
            ready = false;
        }
    }

#ifdef PREEMPTIVE_DEBUG
    osg::notify(osg::NOTICE)
        << "Tile (" << _key->str() << ") at (" << _elevationLOD << "), parent at ("
        << _family[PARENT].elevLOD << "), sibs at (";
    if ( _family[WEST].exists ) osg::notify( osg::NOTICE ) << "W=" << _family[WEST].elevLOD << " ";
    if ( _family[NORTH].exists ) osg::notify( osg::NOTICE ) << "N=" << _family[NORTH].elevLOD << " ";
    if ( _family[EAST].exists ) osg::notify( osg::NOTICE ) << "E=" << _family[EAST].elevLOD << " ";
    if ( _family[SOUTH].exists ) osg::notify( osg::NOTICE ) << "S=" << _family[SOUTH].elevLOD << " ";
    osg::notify(osg::NOTICE) << "), ready = " << (ready? "YES" : "no") << std::endl;
#endif

    return ready;
}


#define PRI_IMAGE_OFFSET 0.1f // priority offset of imagery relative to elevation
#define PRI_LAYER_OFFSET 0.1f // priority offset of image layer(x) vs. image layer(x+1)

void
VersionedTile::checkNeedsUpdate()
{
    //See if we have any completed requests
    bool hasCompletedRequests = false;

    if (_elevRequest.valid() && _elevRequest->isCompleted()) hasCompletedRequests = true;
    else if (_elevPlaceholderRequest.valid() && _elevPlaceholderRequest->isCompleted()) hasCompletedRequests = true;
    else if (_tileGenRequest.valid() && _tileGenRequest->isCompleted()) hasCompletedRequests = true;
    else if (_tileGenNeeded) hasCompletedRequests = true;
    for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
    {
        if (i->get()->isCompleted())
        {
            hasCompletedRequests = true;
        }
    }

    if (hasCompletedRequests && !_needsUpdate)
    {
        setNumChildrenRequiringUpdateTraversal( getNumChildrenRequiringUpdateTraversal() + 1 );
        _needsUpdate = true;
    }
    else if (!hasCompletedRequests && _needsUpdate)
    {
        setNumChildrenRequiringUpdateTraversal( getNumChildrenRequiringUpdateTraversal() - 1 );
        _needsUpdate = false;
    }
}

void
VersionedTile::installRequests( int stamp )
{
    VersionedTerrain* terrain = getVersionedTerrain();

    Map* map = terrain->getMap();
    OpenThreads::ScopedReadLock lock( map->getMapDataMutex() );

    MapEngine* engine = terrain->getEngine();

    bool hasElevationLayer;
    int numColorLayers;
    {
        ScopedReadLock lock( _tileLayersMutex );
        hasElevationLayer = this->getElevationLayer() != NULL;
        numColorLayers = this->getNumColorLayers();
    }

    if ( _hasElevation && hasElevationLayer )
    {        
        // this request will load real elevation data for the tile:
        _elevRequest = new TileElevationLayerRequest(_key.get(), map, engine );
        float priority = (float)_key->getLevelOfDetail();
        _elevRequest->setPriority( priority );
        std::stringstream ss;
        ss << "TileElevationLayerRequest " << _key->str() << std::endl;
        _elevRequest->setName( ss.str() );

        // this request will load placeholder elevation data for the tile:
        _elevPlaceholderRequest = new TileElevationPlaceholderLayerRequest(
            _key.get(), map, engine, _keyLocator.get() );
        _elevPlaceholderRequest->setPriority( priority );
        ss.str("");
        ss << "TileElevationPlaceholderLayerRequest " << _key->str() << std::endl;
        _elevPlaceholderRequest->setName( ss.str() );
    }

    // a tile generator request will rebuild tile geometry in the background:
    _tileGenRequest = new TileGenRequest( this->getTerrainTechnique() );

    // safely loop through the map layers and update the imagery for each:
    MapLayerList imageMapLayers;
    map->getImageMapLayers( imageMapLayers );

    for( int layerIndex = 0; layerIndex < numColorLayers; layerIndex++ )
    {
        if ( layerIndex < imageMapLayers.size() )
        {
            updateImagery( imageMapLayers[layerIndex]->getId(), map, engine );
        }
    }

    _requestsInstalled = true;
}

void
VersionedTile::updateImagery(unsigned int layerId, Map* map, MapEngine* engine)
{
    VersionedTerrain* terrain = getVersionedTerrain();

    MapLayer* mapLayer = NULL;
    unsigned int layerIndex = -1;
    for (unsigned int i = 0; i < map->getImageMapLayers().size(); ++i)
    {
        if (map->getImageMapLayers()[i]->getId() == layerId)
        {
            mapLayer = map->getImageMapLayers()[i];
            layerIndex = i;
            break;
        }
    }

    if (!mapLayer)
    {
        osg::notify(osg::NOTICE) << "updateImagery could not find MapLayer with id=" << layerId << std::endl;
        return;
    }

    if ( mapLayer->isKeyValid( _key.get() ) )
    {
        unsigned int layerId = mapLayer->getId();
        // imagery is slighty higher priority than elevation data
        TaskRequest* r = new TileColorLayerRequest( _key.get(), map, engine, layerId );
        std::stringstream ss;
        ss << "TileColorLayerRequest " << _key->str() << std::endl;
        r->setName( ss.str() );
        r->setState( osgEarth::TaskRequest::STATE_IDLE );
        r->setPriority( PRI_IMAGE_OFFSET + (float)_key->getLevelOfDetail());
        r->setProgressCallback( new StampedProgressCallback( r, terrain->getImageryTaskService( layerIndex ) ));

        //If we already have a request for this layer, remove it from the list and use the new one
        for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
        {
            TileColorLayerRequest* r2 = static_cast<TileColorLayerRequest*>( i->get() );
            if (r2->_layerId == layerId)
            {
                _requests.erase( i );
                break;
            }
        }
        //Add the new imagery request
        _requests.push_back( r );
    }
}

// This method is called from the CULL TRAVERSAL, from TileLoaderCullCallback in MapEngine.cpp.
void
VersionedTile::servicePendingImageRequests( int stamp )
{       
    // install our requests if they are not already installed:
    if ( !_requestsInstalled )
    {
        installRequests( stamp );
    }

    for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
    {
        TileColorLayerRequest* r = static_cast<TileColorLayerRequest*>( i->get() );

        //If a request has been marked as IDLE, the TaskService has tried to service it
        //and it was either deemed out of date or was cancelled, so we need to add it again.
        if ( r->isIdle() )
        {
            //osg::notify(osg::INFO) << "Queuing IR (" << _key->str() << ")" << std::endl;
            r->setStamp( stamp );
            getVersionedTerrain()->getImageryTaskService(r->_layerId)->add( r );
        }
        else if ( !r->isCompleted() )
        {
            r->setStamp( stamp );
        }
    }

    checkNeedsUpdate();
}

Relative*
VersionedTile::getFamily() {
    return _family;
}

// This method is called from the CULL TRAVERSAL, from VersionedTerrain::traverse.
void
VersionedTile::servicePendingElevationRequests( int stamp )
{
    // install our requests if they are not already installed:
    if ( !_requestsInstalled )
    {
        installRequests( stamp );
    }

    // GW: this is now called at the Terrain level.
    // make sure we know where to find our parent and sibling tiles:
    //if (getVersionedTerrain())
    //{
    //    getVersionedTerrain()->refreshFamily( getTileID(), _family );
    //}

    if ( _hasElevation && !_elevationLayerUpToDate && _elevRequest.valid() && _elevPlaceholderRequest.valid() )
    {  
        VersionedTerrain* terrain = getVersionedTerrain();

        // update the main elevation request if it's running:
        if ( !_elevRequest->isIdle() )
        {
#ifdef PREEMPTIVE_DEBUG
            osg::notify(osg::NOTICE) << "Tile (" << _key->str() << ") .. ER not idle" << std::endl;
#endif
            
            if ( !_elevRequest->isCompleted() )
            {
                _elevRequest->setStamp( stamp );
            }
        }

        // update the placeholder request if it's running:
        else if ( !_elevPlaceholderRequest->isIdle() )
        {
#ifdef PREEMPTIVE_DEBUG
            osg::notify(osg::NOTICE) << "Tile (" << _key->str() << ") .. PR not idle" << std::endl;
#endif
            if ( !_elevPlaceholderRequest->isCompleted() )
            {
               _elevPlaceholderRequest->setStamp( stamp );
            }
        }

        // otherwise, see if it is legal yet to start a new request:
        else if ( readyForNewElevation() )
        {
            if ( _elevationLOD + 1 == _key->getLevelOfDetail() )
            {
                _elevRequest->setStamp( stamp );
                _elevRequest->setProgressCallback( new ProgressCallback() );
                terrain->getElevationTaskService()->add( _elevRequest.get() );
#ifdef PREEMPTIVE_DEBUG
                osg::notify(osg::NOTICE) << "..queued FE req for (" << _key->str() << ")" << std::endl;
#endif
            }
            
            else if ( _family[PARENT].elevLOD > _elevationLOD )
            {
                osg::ref_ptr<VersionedTile> parentTile = _family[PARENT].tile.get();
                if ( _elevationLOD < _family[PARENT].elevLOD && parentTile.valid() )
                {
                    TileElevationPlaceholderLayerRequest* er = static_cast<TileElevationPlaceholderLayerRequest*>(_elevPlaceholderRequest.get());

                    er->setStamp( stamp );
                    er->setProgressCallback( new ProgressCallback() ); //( ck( er, terrain->getElevationTaskService()));
                    float priority = (float)_key->getLevelOfDetail();
                    er->setPriority( priority );
                    osgTerrain::HeightFieldLayer* hfLayer = static_cast<osgTerrain::HeightFieldLayer*>(parentTile->getElevationLayer());
                    er->setParentHF( hfLayer->getHeightField() );
                    er->setNextLOD( _family[PARENT].elevLOD );
                    terrain->getElevationTaskService()->add( er );
#ifdef PREEMPTIVE_DEBUG
                    osg::notify(osg::NOTICE) << "..queued PH req for (" << _key->str() << ")" << std::endl;
#endif
                }

                else 
                {
#ifdef PREEMPTIVE_DEBUG
                    osg::notify(osg::NOTICE) << "...tile (" << _key->str() << ") ready, but nothing to do." << std::endl;
#endif
                }
            }
        }
    }

    checkNeedsUpdate();
}

void
VersionedTile::markTileForRegeneration()
{
    if ( _useTileGenRequest )
    {
        _tileGenNeeded = true;
    }
    else
    {
        this->setDirty( true );
    }
}

// called from the UPDATE TRAVERSAL, because this method can potentially alter
// the scene graph.
void
VersionedTile::serviceCompletedRequests()
{
    Map* map = this->getVersionedTerrain()->getMap();
    if ( !_requestsInstalled )
        return;

    // First service the tile generator:
    if ( _tileGenRequest->isCompleted() )
    {
        EarthTerrainTechnique* tech = dynamic_cast<EarthTerrainTechnique*>( getTerrainTechnique() );
        if ( tech )
            tech->swapIfNecessary();
        _tileGenRequest->setState( TaskRequest::STATE_IDLE );
    }

    // Then the image requests:
    for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); )
    {
        bool increment = true;

        if ( dynamic_cast<TileColorLayerRequest*>( i->get() ) )
        {
            TileColorLayerRequest* r = static_cast<TileColorLayerRequest*>( i->get() );
            if ( r->isCompleted() )
            {
                int index = -1;
                {
                    // Lock the map data mutex, since we are querying the map model:
                    ScopedReadLock mapDataLock( map->getMapDataMutex() );

                    //See if we even care about the request
                    for (unsigned int j = 0; j < map->getImageMapLayers().size(); ++j)
                    {
                        if (map->getImageMapLayers()[j]->getId() == r->_layerId)
                        {
                            index = j;
                            break;
                        }
                    }
                }

                //The maplayer was probably deleted
                if (index < 0)
                {
                    osg::notify(osg::INFO) << "Layer " << r->_layerId << " no longer exists, ignoring TileColorLayerRequest " << std::endl;
                    i = _requests.erase(i);
                    increment = false;
                }
                else
                {
                    osg::ref_ptr<osgTerrain::ImageLayer> newImgLayer = static_cast<osgTerrain::ImageLayer*>( r->getResult() );
                    if ( newImgLayer.valid() )
                    {
                        // update the color layer safely:
                        {
                            OpenThreads::ScopedWriteLock layerLock( getTileLayersMutex() );
                            this->setColorLayer( index, newImgLayer.get() );
                        }

                        markTileForRegeneration();

                        //osg::notify(osg::INFO) << "Complete IR (" << _key->str() << ") layer=" << r->_layerId << std::endl;

                        // remove from the list (don't reference "r" after this!)
                        i = _requests.erase( i );
                        increment = false;
                    }
                    else
                    {  
                        //osg::notify(osg::INFO) << "IReq error (" << _key->str() << ") (layer " << r->_layerId << "), retrying" << std::endl;

                        //The color layer request failed, probably due to a server error. Reset it.
                        r->setState( TaskRequest::STATE_IDLE );
                        r->reset();
                    }
                }
            }
            else if ( r->isCanceled() )
            {
                //Reset the cancelled task to IDLE and give it a new progress callback.
                r->setState( TaskRequest::STATE_IDLE );
                r->setProgressCallback( new StampedProgressCallback(
                    r, getVersionedTerrain()->getImageryTaskService(r->_layerId)));
                r->reset();
            }
        }

        if ( increment )
            ++i;
    }

    // Finally, the elevation requests:
    if ( _hasElevation && !_elevationLayerUpToDate && _elevRequest.valid() && _elevPlaceholderRequest.valid() )
    {
        // First, check is the Main elevation request is done. If so, we will now have the final HF data
        // and can shut down the elevation requests for this tile.
        if ( _elevRequest->isCompleted() )
        {
            // if the elevation request succeeded, install the new elevation layer!
            TileElevationLayerRequest* r = static_cast<TileElevationLayerRequest*>( _elevRequest.get() );
            osg::ref_ptr<osgTerrain::HeightFieldLayer> newHFLayer = static_cast<osgTerrain::HeightFieldLayer*>( r->getResult() );
            if ( newHFLayer.valid() && newHFLayer->getHeightField() != NULL )
            {
                newHFLayer->getHeightField()->setSkirtHeight( 
                    getVersionedTerrain()->getEngine()->getEngineProperties().getSkirtRatio() * this->getBound().radius() );

                // need to write-lock the layer data since we'll be changing it:
                {
                    ScopedWriteLock lock( _tileLayersMutex );
                    this->setElevationLayer( newHFLayer.get() );
                }

                // the tile needs rebuilding. This will kick off a TileGenRequest.
                markTileForRegeneration();
                
                // finalize the LOD marker for this tile, so other tiles can see where we are.
                _elevationLOD = _key->getLevelOfDetail();

#ifdef PREEMPTIVE_DEBUG
                osg::notify(osg::NOTICE) << "Tile (" << _key->str() << ") final HF, LOD (" << _elevationLOD << ")" << std::endl;
#endif
                // this was the final elev request, so mark elevation as DONE.
                _elevationLayerUpToDate = true;
                
                // GW- just reset these and leave them alone and let cancelRequests() take care of cleanup later.
                // done with our Elevation requests!
                _elevRequest = 0L;
                _elevPlaceholderRequest = 0L;
            }
            else
            {
                _elevRequest->setState( TaskRequest::STATE_IDLE );
                _elevRequest->reset();
            }
        }

        else if ( _elevRequest->isCanceled() )
        {
            // If the request was canceled, reset it to IDLE and reset the callback. On the next
            // servicePendingRequests, the request will be re-scheduled.
            _elevRequest->setState( TaskRequest::STATE_IDLE );
            _elevRequest->setProgressCallback( new ProgressCallback() );            
            _elevRequest->reset();
        }

        else if ( _elevPlaceholderRequest->isCompleted() )
        {
            TileElevationPlaceholderLayerRequest* r = 
                static_cast<TileElevationPlaceholderLayerRequest*>(_elevPlaceholderRequest.get());

            osg::ref_ptr<osgTerrain::HeightFieldLayer> newPhLayer = static_cast<osgTerrain::HeightFieldLayer*>( r->getResult() );
            if ( newPhLayer.valid() && newPhLayer->getHeightField() != NULL )
            {
                // install the new elevation layer.
                {
                    ScopedWriteLock lock( _tileLayersMutex );
                    this->setElevationLayer( newPhLayer.get() );
                }

                // tile needs to be recompiled.
                markTileForRegeneration();

                // update the elevation LOD for this tile, now that the new HF data is installed. This will
                // allow other tiles to see where this tile's HF data is.
                _elevationLOD = r->_nextLOD;

#ifdef PREEMPTIVE_DEBUG
                osg::notify(osg::NOTICE) << "..tile (" << _key->str() << ") is now at (" << _elevationLOD << ")" << std::endl;
#endif
            }
            _elevPlaceholderRequest->setState( TaskRequest::STATE_IDLE );
            _elevPlaceholderRequest->reset();
        }

        else if ( _elevPlaceholderRequest->isCanceled() )
        {
            _elevPlaceholderRequest->setState( TaskRequest::STATE_IDLE );
            _elevPlaceholderRequest->setProgressCallback( new ProgressCallback() );
            _elevPlaceholderRequest->reset();
        }
    }

    // if we have a new TileGenRequest, queue it up now.
    if ( _tileGenNeeded && _tileGenRequest->isIdle() )
    {
        //osg::notify(osg::NOTICE) << "tile (" << _key->str() << ") queuing new tile gen" << std::endl;

        getVersionedTerrain()->getTileGenerationTaskSerivce()->add( _tileGenRequest.get() );
        _tileGenNeeded = false;
    }

    checkNeedsUpdate();
}

void
VersionedTile::traverse( osg::NodeVisitor& nv )
{
    bool isCull = nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR;
    bool isUpdate = nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR;

    if ( !_tileRegisteredWithTerrain && getVersionedTerrain() && (isCull || isUpdate) ) 
    {
        // register this tile with its terrain if we've not already done it.
        // we want to be sure that the tile is already in the scene graph at the
        // time of registration (otherwise VersionedTerrain will see its refcount
        // at 1 and schedule it for removal as soon as it's added. Therefore, we
        // make sure this is either a CULL or UPDATE traversal.
        getVersionedTerrain()->registerTile( this );
        _tileRegisteredWithTerrain = true;

        // we constructed this tile with an update traversal count of 1 so it would get
        // here and we could register the tile. Now we can decrement it back to normal.
        this->setNumChildrenRequiringUpdateTraversal(
            this->getNumChildrenRequiringUpdateTraversal() - 1 );
    }

    if ( isUpdate && _useLayerRequests )
    {
        serviceCompletedRequests();
    }

    osgTerrain::TerrainTile::traverse( nv );
}


void VersionedTile::releaseGLObjects(osg::State* state) const
{
    Group::releaseGLObjects(state);

    if (_terrainTechnique.valid())
    {
        //NOTE: crashes sometimes if OSG_RELEASE_DELAY is set -gw
        _terrainTechnique->releaseGLObjects( state );
        //osg::notify(osg::NOTICE) << "[osgEarth] VT releasing GL objects" << std::endl;
    }
}


/****************************************************************************/


VersionedTerrain::VersionedTerrain( Map* map, MapEngine* engine ) :
_map( map ),
_engine( engine ),
_revision(0),
_numAsyncThreads( 0 )
{
    this->setThreadSafeRefUnref( true );

    //See if the number of threads is explicitly provided
    const optional<int>& numThreads = engine->getEngineProperties().getNumLoadingThreads();
    if (numThreads.isSet())
    {
        _numAsyncThreads = numThreads.get();
    }

    //See if an environment variable was set
    const char* env_numTaskServiceThreads = getenv("OSGEARTH_NUM_PREEMPTIVE_LOADING_THREADS");
    if ( env_numTaskServiceThreads )
    {
        _numAsyncThreads = ::atoi( env_numTaskServiceThreads );
    }

    //See if a processors per core option was provided
    if (_numAsyncThreads == 0)
    {
        const optional<int>& threadsPerProcessor = engine->getEngineProperties().getNumLoadingThreadsPerLogicalProcessor();
        if (threadsPerProcessor.isSet())
        {
            _numAsyncThreads = OpenThreads::GetNumberOfProcessors() * threadsPerProcessor.get();
        }
    }

    //Default to using 2 threads per core
    if (_numAsyncThreads == 0 )
    {
        _numAsyncThreads = OpenThreads::GetNumberOfProcessors() * 2;
    }

    osg::notify(osg::INFO) << "Using " << _numAsyncThreads << " loading threads " << std::endl;
}

void
VersionedTerrain::incrementRevision()
{
    // no need to lock; if we miss it, we'll get it the next time around
    _revision++;
}

int
VersionedTerrain::getRevision() const
{
    // no need to lock; if we miss it, we'll get it the next time around
    return _revision;
}

void
VersionedTerrain::getVersionedTile( const osgTerrain::TileID& tileID,
                                   osg::ref_ptr<VersionedTile>& out_tile,
                                   bool lock )
{
    if ( lock )
    {
        ScopedReadLock lock( _tilesMutex );
        TileTable::iterator i = _tiles.find( tileID );
        out_tile = i != _tiles.end()? i->second.get() : 0L;
    }
    else
    {
        TileTable::iterator i = _tiles.find( tileID );
        out_tile = i != _tiles.end()? i->second.get() : 0L;
    }
}

//void
//VersionedTerrain::getVersionedTile(const osgTerrain::TileID& tileID,
//                                   osg::observer_ptr<VersionedTile>& out_tile,
//                                   bool lock )
//{
//    if ( lock )
//    {
//        ScopedReadLock lock( _tilesMutex );
//        TileTable::iterator i = _tiles.find( tileID );
//        out_tile = i != _tiles.end()? i->second.get() : 0L;
//    }
//    else
//    {
//        TileTable::iterator i = _tiles.find( tileID );
//        out_tile = i != _tiles.end()? i->second.get() : 0L;
//    }
//}

void
VersionedTerrain::getVersionedTiles( TileList& out_list )
{
    ScopedReadLock lock( _tilesMutex );
    for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); i++ )
        out_list.push_back( i->second.get() );
}

void
VersionedTerrain::getTerrainTiles( TerrainTileList& out_list )
{
    ScopedReadLock lock( _tilesMutex );
    for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); i++ )
    {
        out_list.push_back( i->second.get() );
    }
}

// This method is called by VersionedTerrain::traverse().
void
VersionedTerrain::refreshFamily(const osgTerrain::TileID& tileId,
                                Relative* family,
                                bool tileTableLocked )
{
    // geocentric maps wrap around in the X dimension.
    bool wrapX = _map->isGeocentric();
    unsigned int tilesX, tilesY;
    _map->getProfile()->getNumTiles( tileId.level, tilesX, tilesY );
    unsigned int x, y;

    // we examine each of 5 relatives: parent, north, south, east, and west tiles.
    // For each one:
    // a) First determine is it should exist at all (the "expected" member). This flag
    //    indicates whether we expect to find a tile there. For example, if the input
    //    tileId is for a tile on the southern edge of the map, we do not expect to
    //    find a southern neighbor.
    // b) If the tile is deemed to "exist", we move on and try to lookup that tile in
    //    the terrain's registry. It may or may be there, depending on whether it's in
    //    the scene graph at the moment.
    // c) If we find it, check to make sure it is not an orphaned tile (i.e. a tile that
    //    has been expired by the DBPager. We check for this be seeing if the tile has
    //    any parents. If not, it's orphaned and we discard our reference to it.
    // d) Fianlly, if the tile is valid, we read it's elevation LOD value and store it
    //    for later.

    // parent
    family[PARENT].expected = true;
    family[PARENT].elevLOD = -1;
    if ( ! family[PARENT].tile.valid() )
    {
        x = tileId.x/2;
        y = tileId.y/2;
        getVersionedTile( osgTerrain::TileID( tileId.level-1, x, y ), family[PARENT].tile, !tileTableLocked );
    }
    if ( family[PARENT].tile.valid() )
    {
        if ( family[PARENT].tile->getNumParents() > 0 )
            family[PARENT].elevLOD = family[PARENT].tile->getElevationLOD();
        else
            family[PARENT].tile = 0L;
    }        

    // west
    family[WEST].expected = tileId.x > 0 || wrapX;
    family[WEST].elevLOD = -1;
    if ( family[WEST].expected )
    {
        if ( !family[WEST].tile.valid() )
        {
            x = tileId.x > 0? tileId.x-1 : tilesX-1;
            y = tileId.y;
            getVersionedTile( osgTerrain::TileID( tileId.level, x, y ), family[WEST].tile, !tileTableLocked );
        }
        if ( family[WEST].tile.valid() )
        {
            if ( family[WEST].tile->getNumParents() > 0 )
                family[WEST].elevLOD = family[WEST].tile->getElevationLOD();
            else
                family[WEST].tile = 0L;
        }
    }

    // north
    family[NORTH].expected = tileId.y < tilesY-1;
    family[NORTH].elevLOD = -1;
    if ( family[NORTH].expected )
    {
        if ( !family[NORTH].tile.valid() )
        {
            x = tileId.x;
            y = tileId.y < tilesY-1 ? tileId.y+1 : 0;
            getVersionedTile( osgTerrain::TileID( tileId.level, x, y ), family[NORTH].tile, !tileTableLocked );
        }
        if ( family[NORTH].tile.valid() )
        {
            if ( family[NORTH].tile->getNumParents() > 0 )
                family[NORTH].elevLOD = family[NORTH].tile->getElevationLOD();
            else
                family[NORTH].tile = 0L;
        }
    }

    // east
    family[EAST].expected = tileId.x < tilesX-1 || wrapX;
    family[EAST].elevLOD = -1;
    if ( family[EAST].expected )
    {
        if ( !family[EAST].tile.valid() )
        {
            x = tileId.x < tilesX-1 ? tileId.x+1 : 0;
            y = tileId.y;
            getVersionedTile( osgTerrain::TileID( tileId.level, x, y ), family[EAST].tile, !tileTableLocked );
        }
        if ( family[EAST].tile.valid() )
        {
            if ( family[EAST].tile->getNumParents() > 0 )
                family[EAST].elevLOD = family[EAST].tile->getElevationLOD();
            else
                family[EAST].tile = 0L;
        }
    }

    // south
    family[SOUTH].expected = tileId.y > 0;
    family[SOUTH].elevLOD = -1;
    if ( family[SOUTH].expected )
    {
        if ( !family[SOUTH].tile.valid() )
        {   
            x = tileId.x;
            y = tileId.y > 0 ? tileId.y-1 : tilesY-1;
            getVersionedTile( osgTerrain::TileID( tileId.level, x, y ), family[SOUTH].tile, !tileTableLocked );
        }
        if ( family[SOUTH].tile.valid() )
        {
            if ( family[SOUTH].tile->getNumParents() > 0 )
                family[SOUTH].elevLOD = family[SOUTH].tile->getElevationLOD();
            else
                family[SOUTH].tile = 0L;
        }
    }
}

Map*
VersionedTerrain::getMap() {
    return _map.get();
}

MapEngine*
VersionedTerrain::getEngine() {
    return _engine.get();
}

void
VersionedTerrain::registerTile( VersionedTile* newTile )
{
    ScopedWriteLock lock( _tilesMutex );
    _tilesToAdd.push( newTile );
}

void
VersionedTerrain::traverse( osg::NodeVisitor &nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        int stamp = nv.getFrameStamp()->getFrameNumber();

        // update the internal Tile table. This block is the ONLY PLACE where _tiles
        // can be changed, hence the Write Lock. The only other time this mutex is
        // write-locked is in registerTile(), which simply pushes a new tile on to
        // the _tilesToAdd queue.
        {
            ScopedWriteLock lock( _tilesMutex );

            int oldSize = _tiles.size();

            for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); )
            {
                if ( i->second.valid() && i->second->referenceCount() == 1 )
                {
                    //osg::notify(osg::NOTICE) << "Tile (" << i->second->getKey()->str() << ") expired..." << std::endl;

                    _tilesToShutDown.push_back( i->second.get() );
                    TileTable::iterator j = i;
                    ++i;
                    _tiles.erase( j );
                }
                else
                {
                    ++i;
                }
            }
            
            //osg::notify(osg::NOTICE) << "Shutting down " << _tilesToShutDown.size() << " tiles." << std::endl;

            for( TileList::iterator i = _tilesToShutDown.begin(); i != _tilesToShutDown.end(); )
            {
                if ( i->get()->cancelRequests() )
                {
                    //osg::notify(osg::NOTICE) << "Tile (" << i->get()->getKey()->str() << ") shut down." << std::endl;
                    i = _tilesToShutDown.erase( i );
                }
                else
                    ++i;
            }

            // Add any newly registered tiles to the table. If a tile is in the _tilesToAdd
            // queue, we know it is already in the scene graph.
            while( _tilesToAdd.size() > 0 )
            {
                _tiles[ _tilesToAdd.front()->getTileID() ] = _tilesToAdd.front().get();
                _tilesToAdd.pop();
            }

            //if ( _tiles.size() != oldSize )
            //{
            //    osg::notify(osg::NOTICE) << "Tiles registered = " << _tiles.size() << std::endl;
            //}
        }

        // update the frame stamp on the task services. This is necessary to support 
        // automatic request cancelation for image requests.
        {
            ScopedLock<Mutex> lock( _taskServiceMutex );
            for (TaskServiceMap::iterator i = _taskServices.begin(); i != _taskServices.end(); ++i)
            {
                i->second->setStamp( stamp );
            }
        }

        // should probably find a way to not hold this mutex during the loop.. oh well
        {
            ScopedReadLock lock( _tilesMutex );

            // grow the vector if necessary:
            //_tilesToServiceElevation.reserve( _tiles.size() );

            for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); ++i )
            {
                if ( i->second.valid() && i->second->getUseLayerRequests() )
                {
                    refreshFamily( i->first, i->second->getFamily(), true );
                    i->second->servicePendingElevationRequests( stamp );
                    //_tilesToServiceElevation.push_back( i->second.get() );
                }
            }
        }

        //osg::notify(osg::NOTICE) << "Servicing " << _tiles.size() << " tiles." << std::endl;

        // now we can service all the without holding a lock on the tile table.
        //for( TileVector::iterator i = _tilesToServiceElevation.begin(); i != _tilesToServiceElevation.end(); ++i )
        //{
        //    i->get()->servicePendingElevationRequests( stamp );
        //}
        
        // clean up the vector so we can reuse it next time around.
        //_tilesToServiceElevation.clear();
    }

    osgTerrain::Terrain::traverse( nv );
}

TaskService*
VersionedTerrain::createTaskService( int id, int numThreads )
{
    ScopedLock<Mutex> lock( _taskServiceMutex );
    TaskService* service =  new TaskService( numThreads );
    _taskServices[id] = service;
    return service;
}

TaskService*
VersionedTerrain::getTaskService(int id)
{
    ScopedLock<Mutex> lock( _taskServiceMutex );
    TaskServiceMap::iterator itr = _taskServices.find(id);
    if (itr != _taskServices.end())
    {
        return itr->second.get();
    }
    return NULL;
}

#define ELEVATION_TASK_SERVICE_ID 9999
#define TILE_GENERATION_TASK_SERVICE_ID 10000
#define NUM_TILE_GENERATION_THREADS 4

TaskService*
VersionedTerrain::getElevationTaskService()
{
    TaskService* service = getTaskService( ELEVATION_TASK_SERVICE_ID );
    if (!service)
    {
        service = createTaskService( ELEVATION_TASK_SERVICE_ID, 1 );
    }
    return service;
}


TaskService*
VersionedTerrain::getImageryTaskService(int layerId)
{
    TaskService* service = getTaskService( layerId );
    if (!service)
    {
        service = createTaskService( layerId, 1 );
    }
    return service;
}

TaskService*
VersionedTerrain::getTileGenerationTaskSerivce()
{
    TaskService* service = getTaskService( TILE_GENERATION_TASK_SERVICE_ID );
    if (!service)
    {
        int numThreads = _engine->getEngineProperties().getNumTileGeneratorThreads().isSet() ?
            _engine->getEngineProperties().getNumTileGeneratorThreads().get() :
            NUM_TILE_GENERATION_THREADS;

        if ( numThreads < 1 )
            numThreads = 1;

        service = createTaskService( TILE_GENERATION_TASK_SERVICE_ID, numThreads );
    }
    return service;
}

void
VersionedTerrain::updateTaskServiceThreads()
{
    OpenThreads::ScopedReadLock lock(_map->getMapDataMutex());    

    //Get the maximum elevation weight
    float elevationWeight = 0.0f;
    for (MapLayerList::const_iterator itr = _map->getHeightFieldMapLayers().begin(); itr != _map->getHeightFieldMapLayers().end(); ++itr)
    {
        float w = itr->get()->getLoadWeight();
        if (w > elevationWeight) elevationWeight = w;
    }

    float totalImageWeight = 0.0f;
    for (MapLayerList::const_iterator itr = _map->getImageMapLayers().begin(); itr != _map->getImageMapLayers().end(); ++itr)
    {
        totalImageWeight += itr->get()->getLoadWeight();
    }

    float totalWeight = elevationWeight + totalImageWeight;

    if (elevationWeight > 0.0f)
    {
        //Determine how many threads each layer gets
        int numElevationThreads = (int)osg::round((float)_numAsyncThreads * (elevationWeight / totalWeight ));
        osg::notify(osg::NOTICE) << "HtFld Threads = " << numElevationThreads << std::endl;
        getElevationTaskService()->setNumThreads( numElevationThreads );
    }

    for (MapLayerList::const_iterator itr = _map->getImageMapLayers().begin(); itr != _map->getImageMapLayers().end(); ++itr)
    {
        int imageThreads = (int)osg::round((float)_numAsyncThreads * (itr->get()->getLoadWeight() / totalWeight ));
        osg::notify(osg::NOTICE) << "Image Threads for " << itr->get()->getName() << " = " << imageThreads << std::endl;
        getImageryTaskService( itr->get()->getId() )->setNumThreads( imageThreads );
    }

}

