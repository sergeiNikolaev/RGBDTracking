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
#ifndef SOFA_RGBDTRACKING_SHAPESPRINGSFORCEFIELD_H
#define SOFA_RGBDTRACKING_SHAPESPRINGSFORCEFIELD_H

#include <RGBDTracking/config.h>
#include <sofa/core/core.h>
#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/helper/vector.h>
#include <sofa/defaulttype/VecTypes.h>


#ifdef SOFA_HAVE_EIGEN2
#include <sofa/component/linearsolver/EigenSparseMatrix.h>
#endif

namespace sofa
{
namespace core
{
namespace behavior
{
template< class T > class MechanicalState;

} // namespace behavior
} // namespace core
} // namespace sofa

namespace sofa
{

namespace component
{

namespace forcefield
{

/**
* @brief This class describes a simple elastic springs ForceField between DOFs positions and rest positions.
*
* Springs are applied to given degrees of freedom between their current positions and their rest shape positions.
* An external MechanicalState reference can also be passed to the ForceField as rest shape position.
*/
template<class DataTypes>
class ShapeSpringsForceField : public core::behavior::ForceField<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ShapeSpringsForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));

    typedef core::behavior::ForceField<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::CPos CPos;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::Real Real;
    typedef helper::vector< unsigned int > VecIndex;
    typedef helper::vector< Real >	 VecReal;

    typedef core::objectmodel::Data<VecCoord> DataVecCoord;
    typedef core::objectmodel::Data<VecDeriv> DataVecDeriv;

    VecCoord x0;

    Data< helper::vector< unsigned int > > points;
    Data< VecReal > stiffness;
    Data< VecReal > angularStiffness;
    Data< helper::vector< CPos > > pivotPoints;
    Data< std::string > external_rest_shape;
    Data< helper::vector< unsigned int > > external_points;
    Data< bool > recompute_indices;
    Data< bool > drawSpring;
    Data< sofa::defaulttype::Vec4f > springColor;

    Data< VecCoord > ext_points;

    sofa::core::behavior::MechanicalState< DataTypes > *restMState;
#ifdef SOFA_HAVE_EIGEN2
    linearsolver::EigenBaseSparseMatrix<typename DataTypes::Real> matS;    
#endif

    //VecDeriv Springs_dir;
protected:
    ShapeSpringsForceField();
public:
    /// BaseObject initialization method.
    void bwdInit();

    /// Add the forces.
    virtual void addForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& v);

    virtual void addDForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& df, const DataVecDeriv& dx);

    /// Brings ForceField contribution to the global system stiffness matrix.
    virtual void addKToMatrix(const core::MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix );

    virtual void addSubKToMatrix(const core::MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix, const helper::vector<unsigned> & addSubIndex );

    virtual void draw(const core::visual::VisualParams* vparams);

    virtual SReal getPotentialEnergy(const core::MechanicalParams* mparams, const DataVecCoord& x) const override
    {
        SOFA_UNUSED(mparams);
        SOFA_UNUSED(x);

        msg_error() << "Get potentialEnergy not implemented";
        return 0.0;
    }


    const DataVecCoord* getExtPosition() const;
    const VecIndex& getIndices() const { return m_indices; }
    const VecIndex& getExtIndices() const { return (useRestMState ? m_ext_indices : m_indices); }

protected :

    void recomputeIndices();

    VecIndex m_indices;
    VecReal k;
    VecIndex m_ext_indices;
    helper::vector<CPos> m_pivots;

#ifdef SOFA_HAVE_EIGEN2
    double lastUpdatedStep;
#endif
private :

    bool useRestMState; /// An external MechanicalState is used as rest reference.
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_RGBDTRACKING_SHAPESPRINGSFORCEFIELD_CPP)

using namespace sofa::defaulttype;
extern template class SOFA_RGBDTRACKING_API ShapeSpringsForceField<Vec3Types>;
//extern template class SOFA_RGBDTRACKING_API ShapeSpringsForceField<Vec2Types>;
extern template class SOFA_RGBDTRACKING_API ShapeSpringsForceField<Vec1Types>;
//extern template class SOFA_RGBDTRACKING_APIAPI ShapeSpringsForceField<Vec6Types>;
#endif // defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_RGBDTRACKING_SHAPESPRINGSFORCEFIELD_CPP)

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_RGBDTRACKING_SHAPESPRINGSFORCEFIELD_H
