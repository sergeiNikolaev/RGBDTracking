/******************************************************************************
  *  extract points cloud from images
 *****************************************************************************/
#ifndef SOFA_COMPONENT_POINT_CLOUD_EXTRACTOR
#define SOFA_COMPONENT_POINT_CLOUD_EXTRACTOR

/* include files */
#include <cstdint>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/visual/VisualParams.h>

// inlcude OPEN_CV
#include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// include pcl
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include "MeshProcessingForKalmanFilter.h"
#include "ClosestPointForceFieldForKalmanFilter.h"

// FIX: temporarily disabled as SofaSimpleFem is not supposed to depend on SofaOpenGLVisual
#define DRAW_COLORMAP

#include <sofa/helper/ColorMap.h>



namespace sofa {

namespace component {

namespace behavior {


/**
 *        class PointCloudSplitter
 * <p>
 *   description:
 *       class to split the obtained
 *       point on control and observastion sets
 */
class PointCloudSplitter : public core::objectmodel::BaseObject {

public:
    SOFA_CLASS(PointCloudSplitter, core::objectmodel::BaseObject);

    typedef helper::vector<defaulttype::Vec3d> VecCoordinates;

    // updating data
    void handleEvent(sofa::core::objectmodel::Event* event);


    // constructor and destructor
    PointCloudSplitter();
    virtual ~PointCloudSplitter();

    //virtual void reinit();
    virtual void init();
    void cleanup();

protected:
    // extract point cloud from images
    void registerPointCloudMask();

    // save data to file
    void saveFirstStepPointsToFiles();
    void saveFirstStepPoints(helper::ReadAccessor<Data<helper::vector<defaulttype::Vec3d>>>& data, sofa::core::objectmodel::DataFileName& fileName);
    void savePointsTracksToFiles();
    void savePointsTracks(helper::ReadAccessor<Data<helper::vector<defaulttype::Vec3d>>>& data, sofa::core::objectmodel::DataFileName& fileName);

    // extracted point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloudData;

    Eigen::Matrix3f rgbIntrinsicMatrix;

    bool m_initialiseSizes;
    Data<VecCoordinates> d_controlPointsData;
    Data<VecCoordinates> d_observationPointsData;
    Data<bool> d_registerPoints;
    Data<bool> d_writeDataToFile;
    Data<helper::vector<int>> d_globalVisibleMask;

    sofa::core::objectmodel::DataFileName d_pointCloudFile;

    sofa::core::objectmodel::DataFileName controlPointsFile;
    sofa::core::objectmodel::DataFileName observationPointsFile;

    sofa::core::objectmodel::MeshProcessingForKalmanFilter<defaulttype::Vec3dTypes>* meshProcessingData;
    sofa::component::forcefield::ClosestPointForceFieldForKalmanFilter<defaulttype::Vec3dTypes>* pointCloudFinder;

    helper::vector<unsigned int> controlMask;
    unsigned int reductionSize;
};

} // namespace behavior

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_POINT_CLOUD_EXTRACTOR
