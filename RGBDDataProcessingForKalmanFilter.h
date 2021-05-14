/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#ifndef SOFA_RGBDTRACKING_RGBDDATAPROCESSING_FORKALMANFILTER_H
#define SOFA_RGBDTRACKING_RGBDDATAPROCESSING_FORKALMANFILTER_H

#include "RGBDDataProcessing.inl"
#include "RGBDDataProcessing.h"



using namespace std;
using namespace cv;


namespace sofa
{

namespace core
{

namespace objectmodel
{

using helper::vector;
using namespace sofa::defaulttype;
using namespace sofa::component::topology;


template<class DataTypes>
class RGBDDataProcessingForKalmanFilter : public RGBDDataProcessing<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(RGBDDataProcessingForKalmanFilter, DataTypes), SOFA_TEMPLATE(RGBDDataProcessing, DataTypes));
	
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef Data<typename DataTypes::VecCoord> DataVecCoord;
    typedef Data<typename DataTypes::VecDeriv> DataVecDeriv;
    typedef sofa::defaulttype::Vector4 Vector4;
    typedef sofa::defaulttype::Vector3 Vec3;
	
    typedef defaulttype::ImageF DepthTypes;

    enum { N=Vec3dTypes::spatial_dimensions };
    typedef defaulttype::Mat<N,N,Real> Mat;
    typedef helper::fixed_array <unsigned int,3> tri;

    // additional strcutures
    static helper::vector<cv::Rect>& box_vector() {
        static helper::vector<cv::Rect> box_vectorPar;
        return box_vectorPar;
    }

    static int32_t & control_box_index() {
        static int32_t control_box_indexPar = -1;
        return control_box_indexPar;
    }

    static bool& control_selection() {
        static bool control_selectionPar = false;
        return control_selectionPar;
    }


public:

    RGBDDataProcessingForKalmanFilter();
    virtual ~RGBDDataProcessingForKalmanFilter();

    void init();
    void handleEvent(sofa::core::objectmodel::Event *event);

    virtual std::string getTemplateName() const override
    {
        return templateName(this);
    }

    static std::string templateName(const RGBDDataProcessingForKalmanFilter<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }

    // handle mouse in case of control point selection
    static void draw_control_box(cv::Mat _img);
    static void my_mouse_callback_forKalmanFilter(int event, int x, int y, int flags, void* param);

    void setRGBDData(cv::Mat &_color, cv::Mat &_depth)
    {
        RGBDDataProcessing<DataTypes>::setRGBDData(_color, _depth);
    }
	
	
    VecCoord getTargetPositions() { return RGBDDataProcessing<DataTypes>::getTargetPositions(); }
    VecCoord getTargetContourPositions() { return RGBDDataProcessing<DataTypes>::getTargetContourPositions(); }

    //void detectBorder(vector<bool> &border,const helper::vector< tri > &triangles) { RGBDDataProcessing<DataTypes>::detectBorder(border, triangles); }
    void computeCenter(vpImage<unsigned char> &Itemp, vpImagePoint &cog,double &angle, int &surface) { RGBDDataProcessing<DataTypes>::computeCenter(Itemp, cog, angle, surface); }
    void extractTargetPCD();
    void extractTargetPCDContour() { RGBDDataProcessing<DataTypes>::extractTargetPCDContour(); }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCDFromRGBD(cv::Mat& depthImage, cv::Mat& rgbImage);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCDContourFromRGBD(cv::Mat& depthImage, cv::Mat& rgbImage, cv::Mat& distImage, cv::Mat& dotImage)
        { return RGBDDataProcessing<DataTypes>::PCDContourFromRGBD(depthImage, rgbImage, distImage, dotImage); }
    void setCameraPose() { RGBDDataProcessing<DataTypes>::setCameraPose(); }

    void initSegmentation();
    void segment() { RGBDDataProcessing<DataTypes>::segment(); }
    void segmentSynth() { RGBDDataProcessing<DataTypes>::segmentSynth(); }
    void ContourFromRGBSynth(cv::Mat& rgbImage, cv::Mat& distImage, cv::Mat& dotImage) { RGBDDataProcessing<DataTypes>::ContourFromRGBSynth(rgbImage, distImage, dotImage); }
    void draw(const core::visual::VisualParams* vparams);

    // mask to split between control points and observations points
    Data<sofa::helper::vector<unsigned int>> d_controlMask;
    Data<bool> d_controlMaskIsNotExtracted;
};


#if defined(SOFA_EXTERN_TEMPLATE) && !defined(RGBDDataProcessingForKalmanFilter_CPP)
extern template class SOFA_RGBDTRACKING_API RGBDDataProcessingForKalmanFilter<defaulttype::Vec3Types>;
#endif


} //

} //

} // namespace sofa

#endif  // SOFA_RGBDTRACKING_RGBDDATAPROCESSING_FORKALMANFILTER_H
