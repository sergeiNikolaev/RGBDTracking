/******************************************************************************
  *  implementation of point cloud extractor
 *****************************************************************************/

/* include files */
#include <fstream>
#include <sofa/defaulttype/Vec.h>
#include <sofa/simulation/Node.h>
#include <sofa/helper/kdTree.inl>
#include "PointCloudSplitter.h"

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include "../../sofa/applications/plugins/RGBDTracking/DataIO.h"


namespace sofa {

namespace component {

namespace behavior {


/**
 *        PointCloudSplitter::PointCloudSplitter
 * <p>
 *   description:
 *       constructor
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
PointCloudSplitter::PointCloudSplitter()
    : d_controlPointsData( initData (&d_controlPointsData, "controlPoints", "set of control points for Kalman filter") )
    , d_observationPointsData( initData (&d_observationPointsData, "observationPoints", "set of observation points for Kalman filter") )
    , d_registerPoints( initData(&d_registerPoints, true, "registerPoints", "register first point cloud for object"))
    , d_writeDataToFile( initData(&d_writeDataToFile, false, "writeToFile", "should the data be writtem to file"))
    , d_pointCloudFile( initData(&d_pointCloudFile, "pointCloudFile", "point cloud file name"))
    , controlPointsFile( initData(&controlPointsFile, "controlPointsFile", "file name for control points"))
    , observationPointsFile( initData(&observationPointsFile, "observationPointsFile", "file name for observation points"))
{
    this->f_listening.setValue(true);

    m_initialiseSizes = true;
}



/**
 *        PointCloudSplitter::init
 * <p>
 *   description:
 *     initialise model
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
void PointCloudSplitter::init()
{
    sofa::simulation::Node::SPtr root = dynamic_cast<simulation::Node*>(this->getContext());
    root->get(pointCloudFinder);
    root->get(meshProcessingData);

    reductionSize = 10;
}



/**
 *        PointCloudSplitter::handleEvent
 * <p>
 *   description:
 *     a methood that handles events
 *
 * @see none
 *
 *  arguments:
 * @param  event - a pointer to some event
 *
 * @return  none
 *
 */
void PointCloudSplitter::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (sofa::simulation::AnimateBeginEvent::checkEventType(event)) {

        // don't handle the event at the first time
        float time = (float)this->getContext()->getTime();
        if (std::fabs(time) < 1e-07) return;


        helper::WriteAccessor<Data<VecCoordinates>> vControlPoints = d_controlPointsData;
        helper::WriteAccessor<Data<VecCoordinates>> vObservationsPoints = d_observationPointsData;
        helper::ReadAccessor<Data<helper::vector<unsigned int>>> vMaskIndices = meshProcessingData->d_controlPointsMask;
        helper::ReadAccessor<Data<VecCoordinates>> vClosestPoints = pointCloudFinder->d_closestPointsData;
        helper::ReadAccessor<Data<helper::vector<int>>> vClosestPointVisibility =  pointCloudFinder->d_closestPointsMask;

        // copy control mask for mesh processing component
        controlMask.resize(vMaskIndices.size());
        for (size_t index = 0; index < vMaskIndices.size(); index++) {
            controlMask[index] = vMaskIndices[index];
        }

        if (d_registerPoints.getValue()) {
            registerPointCloudMask();
        }

        if (m_initialiseSizes) {
            vControlPoints.resize(vMaskIndices.size());
            vObservationsPoints.resize(pointCloudFinder->d_closestPointsData.getValue().size() - vMaskIndices.size());
        }
        std::cout << "Target positions size: " << vClosestPoints.size() << std::endl;

        unsigned int controlPointsIndex = 0;
        unsigned int observationPointsIndex = 0;
        for (size_t iterationIndex = 0; iterationIndex < vClosestPoints.size(); iterationIndex++) {
            if (vClosestPointVisibility[iterationIndex] == 1) {
                bool found = false;
                std::cout << "Target positions: " << vClosestPoints[iterationIndex].x() << " " << vClosestPoints[iterationIndex].y() << " " << vClosestPoints[iterationIndex].z() << std::endl;
                for (size_t maskIndex = 0; maskIndex < vMaskIndices.size(); maskIndex++) {
                    if (vMaskIndices[maskIndex] == iterationIndex) {
                        found = true;
                        break;
                    }
                }

                if (found) {
                    vControlPoints[controlPointsIndex] = vClosestPoints[iterationIndex];
                    controlPointsIndex++;
                } else {
                    vObservationsPoints[observationPointsIndex] = vClosestPoints[iterationIndex];
                    observationPointsIndex++;
                }
            }
        }

        if (d_writeDataToFile.getValue()) {
            savePointsTracksToFiles();
        }

        if (d_registerPoints.getValue()) {
            if (d_writeDataToFile.getValue()) {
                saveFirstStepPointsToFiles();
            }
            d_registerPoints.setValue(false);
        }
    }
}


/**
 *        PointCloudSplitter::registerPointClouds
 * <p>
 *   description:
 *     register point cloud
 *   for now the porcess workd like this:
 *   1. at first step find and save all visible points
 *   2. for next step search only for points transformable from points of the first step
 *   3. if some point is not found, store its previous implementation
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
void PointCloudSplitter::registerPointCloudMask()
{
    helper::WriteAccessor<Data<helper::vector<int>>> vGlobalMask = d_globalVisibleMask;
    helper::ReadAccessor<Data<helper::vector<int>>> vPointMaskIndices = pointCloudFinder->d_closestPointsMask;

    vGlobalMask.resize(vPointMaskIndices.size());
    for (unsigned int index = 0; index < vGlobalMask.size(); index++) {
        vGlobalMask[index] = vPointMaskIndices[index];
    }
}



/**
 *        PointCloudSplitter::saveFirstStepPointsToFiles
 * <p>
 *   description:
 *     save points from the first step
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
void PointCloudSplitter::saveFirstStepPointsToFiles()
{
    helper::ReadAccessor<Data<helper::vector<defaulttype::Vec3d>>> vControls = d_controlPointsData;
    saveFirstStepPoints(vControls, controlPointsFile);
    helper::ReadAccessor<Data<helper::vector<defaulttype::Vec3d>>> vObservations = d_observationPointsData;
    saveFirstStepPoints(vObservations, observationPointsFile);
}



/**
 *        PointCloudSplitter::saveFirstStepPoints
 * <p>
 *   description:
 *     save points from the first step
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
void PointCloudSplitter::saveFirstStepPoints(helper::ReadAccessor<Data<helper::vector<defaulttype::Vec3d>>>& data, sofa::core::objectmodel::DataFileName& fileName)
{
    // write points at zero step to output file
    std::ofstream outputFile;
    outputFile.open(fileName.getFullPath() + ".vtk");
    outputFile << "# vtk DataFile Version 2.0" << std::endl;
    outputFile << "Exported VTK file" << std::endl;
    outputFile << "ASCII\n" << std::endl;
    outputFile << "DATASET UNSTRUCTURED_GRID" << std::endl;
    outputFile << "POINTS " << data.size() << " float" << std::endl;
    for (size_t index = 0; index < data.size(); index++) {
        outputFile << data[index][0] << " " << data[index][1] << " " << data[index][2] << "\n";
    }
    outputFile << "\nCELLS 0 0\n" << std::endl;
    outputFile << "CELL_TYPES 0\n" << std::endl;
    outputFile.close();
}



/**
 *        PointCloudSplitter::savePointsTracksToFiles
 * <p>
 *   description:
 *     save points from the first step
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
void PointCloudSplitter::savePointsTracksToFiles()
{
    helper::ReadAccessor<Data<helper::vector<defaulttype::Vec3d>>> vControls = d_controlPointsData;
    savePointsTracks(vControls, controlPointsFile);
    helper::ReadAccessor<Data<helper::vector<defaulttype::Vec3d>>> vObservations = d_observationPointsData;
    savePointsTracks(vObservations, observationPointsFile);
}



/**
 *        PointCloudSplitter::savePointsTracks
 * <p>
 *   description:
 *     save points from the first step
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
void PointCloudSplitter::savePointsTracks(helper::ReadAccessor<Data<helper::vector<defaulttype::Vec3d>>>& data, sofa::core::objectmodel::DataFileName& fileName)
{
    // write headers to output files
    std::ofstream trackFile;
    trackFile.open(fileName.getFullPath() + "_x.txt", std::fstream::app);

    if (d_registerPoints.getValue()) {
        trackFile << "# Gnuplot File : positions of " << data.size() << " particle(s) Monitored" <<  std::endl;
        trackFile << "# 1st Column : time, others : particle(s) number ";
        for (unsigned int index = 0; index < data.size(); index++)
            trackFile << index << " ";
        trackFile << std::endl;
    }

    sofa::core::objectmodel::BaseContext* currentContext = this->getContext();
    double time = currentContext->getTime();
    trackFile << time;
    for (size_t index = 0; index < data.size(); index++) {
        trackFile << "\t" << data[index][0] << " " << data[index][1] << " " << data[index][2];
    }
    trackFile << std::endl;
    trackFile.close();
}



/**
 *        PointCloudSplitter::cleanup
 * <p>
 *   description:
 *       executing function in exit
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
void PointCloudSplitter::cleanup()
{

}


/**
 *        PointCloudExtractor::~PointCloudExtractor
 * <p>
 *   description:
 *       destructor
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
PointCloudSplitter::~PointCloudSplitter()
{

}



} // namespace bahavior

} // namespace compoinent

} // namespace sofa
