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

#ifndef SOFA_RGBDTRACKING_CLOSESTPOINTFORCEFIELD_FORKALMANFILTER_H
#define SOFA_RGBDTRACKING_CLOSESTPOINTFORCEFIELD_FORKALMANFILTER_H

#include "../../sofa/applications/plugins/RGBDTracking/ClosestPointForceField.h"


using namespace std;
using namespace cv;

typedef struct point_struct{

  double x;
  double y;
  double z;
}point_struct;

namespace sofa
{

namespace component
{

namespace forcefield
{

using helper::vector;
using namespace sofa::defaulttype;


template<class DataTypes>
class ClosestPointForceFieldForKalmanFilter : public ClosestPointForceField<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ClosestPointForceFieldForKalmanFilter,DataTypes),SOFA_TEMPLATE(ClosestPointForceField,DataTypes));

    typedef sofa::core::objectmodel::BaseObject Inherit;
    typedef defaulttype::ImageF DepthTypes;

    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecReal VecReal;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef Data<typename DataTypes::VecCoord> DataVecCoord;
    typedef Data<typename DataTypes::VecDeriv> DataVecDeriv;
    typedef sofa::defaulttype::Vector4 Vector4;
    typedef sofa::defaulttype::Vector2 Vec2;
    typedef helper::vector<defaulttype::Vec3d> VecCoordinates;

    typedef core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename interactionforcefield::LinearSpring<Real> Spring;
    typedef helper::fixed_array <unsigned int,3> tri;

    //typedef typename Coord::value_type real;
    typedef std::pair<int,Real> Col_Value;
    typedef vector< Col_Value > CompressedValue;
    typedef vector< CompressedValue > CompressedMatrix;


    ClosestPointForceFieldForKalmanFilter(core::behavior::MechanicalState<DataTypes> *mm = NULL);
    virtual ~ClosestPointForceFieldForKalmanFilter() override;
	
    virtual std::string getTemplateName() const override
    {
        return templateName(this);
    }

    static std::string templateName(const ClosestPointForceFieldForKalmanFilter<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }

     core::behavior::MechanicalState<DataTypes>* getObject() { return this->mstate; }

     void reinit() override;
     void init() override;
     void addForce(const core::MechanicalParams* mparams,DataVecDeriv& f , const DataVecCoord& x , const DataVecDeriv& v) override;
     void addDForce(const core::MechanicalParams* mparams ,DataVecDeriv&   df , const DataVecDeriv&   dx) override
             { ClosestPointForceField<DataTypes>::addDForce(mparams, df, dx); }
     double getPotentialEnergy(const core::MechanicalParams* param,const DataVecCoord& coord) const override
             { return ClosestPointForceField<DataTypes>::getPotentialEnergy(param, coord); }
     //void addKToMatrix( const core::MechanicalParams* mparams,const sofa::core::behavior::MultiMatrixAccessor* matrix);
     virtual void addKToMatrix(sofa::defaulttype::BaseMatrix *m, SReal kFactor, unsigned int &offset) override
             { ClosestPointForceField<DataTypes>::addKToMatrix(m, kFactor, offset); }

     Real getStiffness() const { return ClosestPointForceField<DataTypes>::getStiffness(); }
     Real getDamping() const { return ClosestPointForceField<DataTypes>::getDamping(); }
     void setStiffness(Real _ks) { ClosestPointForceField<DataTypes>::setStiffness(_ks); }
     void setDamping(Real _kd) { ClosestPointForceField<DataTypes>::setDamping(_kd); }
     Real getArrowSize() const { return ClosestPointForceField<DataTypes>::getArrowSize(); }
     void setArrowSize(float s) { ClosestPointForceField<DataTypes>::setArrowSize(s); }
     int getDrawMode() const { return ClosestPointForceField<DataTypes>::getDrawMode(); }
     void setDrawMode(int m)  { ClosestPointForceField<DataTypes>::setDrawMode(m); }

     void draw(const core::visual::VisualParams* vparams) override;

     // -- Modifiers

     void clearSprings(int reserve=0) { ClosestPointForceField<DataTypes>::clearSprings(reserve); }

     void removeSpring(unsigned int idSpring) { ClosestPointForceField<DataTypes>::removeSpring(idSpring); }

     void addSpring(int m1, SReal ks, SReal kd ) { ClosestPointForceField<DataTypes>::addSpring(m1, ks, kd); }

     void addSpring(const Spring & spring) { ClosestPointForceField<DataTypes>::addSpring(spring); }

protected:

     virtual void addSpringForce(double& potentialEnergy, VecDeriv& f,const  VecCoord& p,const VecDeriv& v, int i, const Spring& spring) override
            { ClosestPointForceField<DataTypes>::addSpringForce(potentialEnergy, f, p, v, i, spring); }
     virtual void addStoredSpringForce(double& potentialEnergy, VecDeriv& f,const  VecCoord& p,const VecDeriv& v, int i, const Spring& spring) override
            { ClosestPointForceField<DataTypes>::addStoredSpringForce(potentialEnergy, f, p, v, i, spring); }
     virtual void addSpringForceWeight(double& potentialEnergy, VecDeriv& f,const  VecCoord& p,const VecDeriv& v, int i, int ivis, const Spring& spring) override
            { ClosestPointForceField<DataTypes>::addSpringForceWeight(potentialEnergy, f, p, v, i, ivis, spring); }
     /// Apply the stiffness, i.e. accumulate df given dx
     virtual void addSpringDForce(VecDeriv& df,const  VecDeriv& dx, int i, const Spring& spring, double kFactor, double bFactor) override
            { ClosestPointForceField<DataTypes>::addSpringDForce(df, dx, i, spring, kFactor, bFactor); }

     void resetSprings() { ClosestPointForceField<DataTypes>::resetSprings(); }
     void addForceMesh(const core::MechanicalParams* mparams,DataVecDeriv& _f , const DataVecCoord& _x , const DataVecDeriv& _v );

     double computeError(Vector3 sourcePoint, Vector3 targetPoint) { return ClosestPointForceField<DataTypes>::computeError(sourcePoint, targetPoint); }


public:
     Data<VecCoordinates> d_closestPointsData;
     Data<helper::vector<int>> d_closestPointsMask;

     Data<sofa::helper::vector<unsigned int>> d_sourceControlMask;
     Data<bool> d_controlMaskIsNotTransfered;
     Data<sofa::helper::vector<unsigned int>> d_targetControlMask;

     Data<bool> d_drawObservations;
};


#if defined(SOFA_EXTERN_TEMPLATE) && !defined(ClosestPointForceFieldForKalmanFilter_CPP)
extern template class SOFA_RGBDTRACKING_API ClosestPointForceFieldForKalmanFilter<defaulttype::Vec3Types>;
#endif


} //

} //

} // namespace sofa

#endif   // SOFA_RGBDTRACKING_CLOSESTPOINTFORCEFIELD_FORKALMANFILTER_H
