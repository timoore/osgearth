#include "MeshEditor"

#define LC "[MeshEditor] "

using namespace osgEarth;
using namespace osgEarth::REX;

MeshEditor::MeshEditor(const TileKey& key, unsigned tileSize, const Map* map) :
    _key( key ), _tileSize(tileSize), _ndcMin(DBL_MAX, DBL_MAX, DBL_MAX), _ndcMax(-DBL_MAX, -DBL_MAX, -DBL_MAX),
    _tileLength(static_cast<double>(tileSize) - 1.0)
{
    MeshEditLayerVector editLayers;
    map->getLayers(editLayers);

    for(MeshEditLayerVector::const_iterator it = editLayers.begin();
        it != editLayers.end(); 
        ++it)
    {
        MeshEditLayer* layer = it->get();
        if ( layer->getMinLevel() <= key.getLevelOfDetail() )
        {
            addEditGeometry(layer->getOrCreateEditGeometry( 1.0, key.getExtent().getSRS(), (ProgressCallback*)0L ) );
        }
    }
}

void
MeshEditor::addEditGeometry(MeshEditLayer::EditVector *geometry)
{
    // Make a "locator" for this key so we can do coordinate conversion:
    GeoLocator geoLocator(_key.getExtent());n

    if ( geometry )
    {
        for (auto geomString : *geometry)
        {
            // Calculate the axis-aligned bounding box of the boundary polygon:
            osg::BoundingBoxd bbox = polygonBBox2d(*geomString);
            // convert that bounding box to "unit" space (0..1 across the tile)
            osg::Vec3d min_ndc, max_ndc;
            geoLocator.mapToUnit(bbox._min, min_ndc);
            geoLocator.mapToUnit(bbox._max, max_ndc);

            // true if boundary overlaps tile in X dimension:
            bool x_match = min_ndc.x() < 1.0 && max_ndc.x() >= 0.0;

            // true if boundary overlaps tile in Y dimension:
            bool y_match = min_ndc.y() < 1.0 && max_ndc.y() >= 0.0;

            if (x_match && y_match)
            {
                // yes, boundary overlaps tile, so expand the global NDC bounding
                // box to include the new mask:
                _ndcMin.x() = std::min(_ndcMin.x(), min_ndc.x());
                _ndcMin.y() = std::min(_ndcMin.y(), min_ndc.y());
                _ndcMax.x() = std::max(_ndcMax.x(), max_ndc.x());
                _ndcMax.y() = std::max(_ndcMax.y(), max_ndc.y());

            }


            // and add this mask to the list.
            _edits.push_back(EditGeometry())
            _maskRecords.push_back( MaskRecord(boundary, min_ndc, max_ndc, 0L) );
        }
    }
}
