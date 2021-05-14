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

#ifndef SOFA_RGBDTRACKING_MESHPROCESSING_FORKALMANFILTER_H
#define SOFA_RGBDTRACKING_MESHPROCESSING_FORKALMANFILTER_H

#include "MeshProcessing.inl"
#include "MeshProcessing.h"


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
class MeshProcessingForKalmanFilter : public MeshProcessing<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(MeshProcessingForKalmanFilter,DataTypes), SOFA_TEMPLATE(MeshProcessing,DataTypes));

    typedef typename DataTypes::VecCoord VecCoord;

    MeshProcessingForKalmanFilter();
    virtual ~MeshProcessingForKalmanFilter();

    void init();
    void bwdInit();
    void handleEvent(sofa::core::objectmodel::Event *event);

    virtual std::string getTemplateName() const override
    {
        return templateName(this);
    }

    static std::string templateName(const MeshProcessingForKalmanFilter<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }

    VecCoord getSourcePositions(){ return MeshProcessing<DataTypes>::getSourcePositions(); }
    VecCoord getSourceVisiblePositions(){ return MeshProcessing<DataTypes>::getSourceVisiblePositions(); }
    VecCoord getSourceContourPositions(){ return MeshProcessing<DataTypes>::getSourceContourPositions(); }
    //void setViewPoint() { MeshProcessing<DataTypes>::setViewPoint(); }

    //void updateSourceSurface() { MeshProcessing<DataTypes>::updateSourceSurface(); } // built k-d tree and identify border vertices

    void extractSourceContour() { MeshProcessing<DataTypes>::extractSourceContour(); }
    void extractSourceVisibleContour() { MeshProcessing<DataTypes>::updateSourceVisibleContour(); }
    void getSourceVisible(double znear, double zfar);
    void updateSourceVisible();
    void updateSourceVisibleContour() { MeshProcessing<DataTypes>::updateSourceVisibleContour(); }
    void draw(const core::visual::VisualParams* vparams) { MeshProcessing<DataTypes>::draw(vparams); }

    // mask to split between control points and observations points
    Data<sofa::helper::vector<unsigned int>> d_controlPointsMask;
    Data<bool> d_controlPointsAreNotExtracted;
};


#if defined(SOFA_EXTERN_TEMPLATE) && !defined(MeshProcessing_CPP)
extern template class SOFA_RGBDTRACKING_API MeshProcessingForKalmanFilter<defaulttype::Vec3Types>;
#endif


} //

} //

} // namespace sofa

#endif  // SOFA_RGBDTRACKING_MESHPROCESSING_FORKALMANFILTER_H
