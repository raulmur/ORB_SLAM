#include "FeatureGrid.h"
namespace ORB_SLAM
{   
    FeatureGrid::FeatureGrid(const int cell_size, const int nFeaturesInImage, const int nMaxX,
                const int nMinX, const int nMaxY, const int nMinY):
        mnCellSize(cell_size), mnMinX(nMinX), mnMinY(nMinY),
        mnGridCols( (nMaxX - nMinX)/ cell_size +2),
        mnGridRows( (nMaxY - nMinY)/ cell_size +2),
        mfGridElementWidthInv(1.f/cell_size),
        mfGridElementHeightInv(1.f/cell_size),
        mnPointPerCell(nFeaturesInImage/(mnGridCols*mnGridRows) +1)
    {
        mvMPGrid.resize(mnGridCols);
        for(auto it= mvMPGrid.begin(), ite= mvMPGrid.end(); it!=ite;++it)
        {
            it->resize(mnGridRows);
            for(auto it2= it->begin(), it2e= it->end(); it2!=it2e; ++it2)
                it2->reserve(mnPointPerCell);
        }
    }

    FeatureGrid::~FeatureGrid(){
        mvMPGrid.clear();
    }
    void FeatureGrid::resetGrid()
    {
        for(auto it= mvMPGrid.begin(), ite= mvMPGrid.end(); it!=ite;++it)
        {
            for(auto it2= it->begin(), it2e= it->end(); it2!=it2e; ++it2)
                it2->clear();
        }
    }

} //namespace ORB_SLAM


