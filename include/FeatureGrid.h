
#ifndef FEATUREGRID_H
#define FEATUREGRID_H

#include <vector>
#include <list>
#include <opencv2/features2d/features2d.hpp>
namespace ORB_SLAM
{

class FeatureGrid
{    
public:    
    FeatureGrid(const int cell_size, const int nFeaturesInImage, const int nMaxX,
                const int nMinX, const int nMaxY, const int nMinY);
    ~FeatureGrid();
    void resetGrid();
       
    inline bool IsPointEligible(const cv::KeyPoint& kpUn, int & posX, int & posY)
    {
        posX = round((kpUn.pt.x- mnMinX)*mfGridElementWidthInv);
        posY = round((kpUn.pt.y- mnMinY)*mfGridElementHeightInv);
        if(posX<0 || posX>=mnGridCols || posY<0 || posY>=mnGridRows || mvMPGrid[posX][posY].size()>= mnPointPerCell)
                return false;
        return true;
    }
    //check posX and posY are valid before calling this function
    inline void AddMapPoint(const int posX, const int posY, const int nIdInFrame)
    {
        mvMPGrid[posX][posY].push_back(nIdInFrame);
    }
    const int mnCellSize;
    const int mnMinX;
    const int mnMinY;
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;
    const float mnPointPerCell; //max number of map points per cell
    std::vector< std::vector< std::vector<size_t> > > mvMPGrid;// record observations of mappoints in the new keyframe
};

} //namespace ORB_SLAM

#endif

