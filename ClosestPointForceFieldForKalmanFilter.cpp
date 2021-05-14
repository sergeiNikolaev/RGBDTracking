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

#define SOFA_RGBDTRACKING_CLOSESTPOINTFORCEFIELD_FORKALMANFILTER_CPP

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/Mapping.inl>
#include <sofa/simulation/Simulation.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/gui/BaseGUI.h>
#include <sofa/gui/BaseViewer.h>
#include <sofa/gui/GUIManager.h>

#include "ClosestPointForceFieldForKalmanFilter.h"


using std::cerr;
using std::endl;

namespace sofa
{

namespace component
{

namespace forcefield
{
    using namespace sofa::defaulttype;

    SOFA_DECL_CLASS(ClosestPointForceFieldForKalmanFilter)

    // Register in the Factory
    int ClosestPointForceFieldForKalmanFilterClass = core::RegisterObject("Compute forces based on closest points from/to a target surface/point set")
    #ifndef SOFA_FLOAT
        .add< ClosestPointForceFieldForKalmanFilter<Vec3dTypes> >()
    #endif
    #ifndef SOFA_DOUBLE
        .add< ClosestPointForceFieldForKalmanFilter<Vec3fTypes> >()
    #endif
    ;

    #ifndef SOFA_FLOAT
      template class SOFA_RGBDTRACKING_API ClosestPointForceFieldForKalmanFilter<Vec3dTypes>;
    #endif
    #ifndef SOFA_DOUBLE
      template class SOFA_RGBDTRACKING_API ClosestPointForceFieldForKalmanFilter<Vec3fTypes>;
    #endif

using namespace helper;

/**
 *        ClosestPointForceFieldForKalmanFilter::ClosestPointForceFieldForKalmanFilter
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
template <class DataTypes>
ClosestPointForceFieldForKalmanFilter<DataTypes>::ClosestPointForceFieldForKalmanFilter(core::behavior::MechanicalState<DataTypes> *mm)
 : ClosestPointForceField<DataTypes>(mm)
 , d_closestPointsData( initData(&d_closestPointsData, "closestsPoints", "the closest points to current point set") )
 , d_closestPointsMask( initData(&d_closestPointsMask, "closestsPointsMask", "mask that show if the closest point exists") )
 , d_sourceControlMask( initData(&d_sourceControlMask, "sourceControlMask", "control mask for points from RGBD data") )
 , d_controlMaskIsNotTransfered( initData(&d_controlMaskIsNotTransfered, "controlMaskIsNotTransfered", "checkIfControlMaskWasTransfered"))
 , d_targetControlMask( initData(&d_targetControlMask, "targetControlMask", "control mask for result point cloud") )
 , d_drawObservations ( initData(&d_drawObservations, false, "drawObservations", "draw observation and control point cloud"))
{
    this->f_listening.setValue(true);
    d_controlMaskIsNotTransfered.setValue(true);
}



/**
 *        ClosestPointForceFieldForKalmanFilter::~ClosestPointForceFieldForKalmanFilter
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
template <class DataTypes>
ClosestPointForceFieldForKalmanFilter<DataTypes>::~ClosestPointForceFieldForKalmanFilter()
{
}



/**
 *        ClosestPointForceFieldForKalmanFilter::reinit
 * <p>
 *   description:
 *     reinitialise component
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
template <class DataTypes>
void ClosestPointForceFieldForKalmanFilter<DataTypes>::reinit()
{
    ClosestPointForceField<DataTypes>::reinit();

}



/**
 *        ClosestPointForceFieldForKalmanFilter::init
 * <p>
 *   description:
 *     initialise component
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
template <class DataTypes>
void ClosestPointForceFieldForKalmanFilter<DataTypes>::init()
{
    ClosestPointForceField<DataTypes>::init();

    helper::WriteAccessor<Data<VecCoordinates>> closestPoints = d_closestPointsData;
    closestPoints.resize(this->npoints);
    helper::WriteAccessor<Data<helper::vector<int>>> closestPointsMask = d_closestPointsMask;
    closestPointsMask.resize(this->npoints, 0);
}



/**
 *        ClosestPointForceFieldForKalmanFilter::addForce
 * <p>
 *   description:
 *     add force field to point cloud
 * @see none
 *
 *  arguments:
 * @param
 *
 * @return  none
 *
 */
template <class DataTypes>
void ClosestPointForceFieldForKalmanFilter<DataTypes>::addForce(const core::MechanicalParams* mparams,DataVecDeriv& _f , const DataVecCoord& _x , const DataVecDeriv& _v )
{
    double timeaddforce = (double)getTickCount();
    addForceMesh(mparams, _f, _x, _v);
    std::cout << "TIME ADDFORCE " <<  (getTickCount() - timeaddforce)/getTickFrequency() << std::endl;


    // get local pointers
    helper::WriteAccessor<Data<helper::vector<int>>> vClosestPointsMask = d_closestPointsMask;
    helper::WriteAccessor<Data<VecCoordinates>> closestPoints = d_closestPointsData;
    helper::vector< int > indicesData = this->indicesVisible.getValue();
    //std::cout << " vector indices: ";
    for (size_t index = 0; index < indicesData.size(); index++) {
        size_t vectorIndex = static_cast<size_t>(indicesData[index]);
        //std::cout << vectorIndex << " ";
    }
    //std::cout << std::endl;


    // copy found closest points to a separate vector
    for (size_t index = 0; index < (size_t)this->npoints; index++) {
        vClosestPointsMask[index] = 0;
    }

    for (size_t index = 0; index < indicesData.size(); index++) {
        size_t vectorIndex = static_cast<size_t>(indicesData[index]);
        vClosestPointsMask[vectorIndex] = 1;
        closestPoints[vectorIndex] = this->closestPos[vectorIndex];
    }
}



/**
 *        ClosestPointForceFieldForKalmanFilter::addForceMesh
 * <p>
 *   description:
 *     add force field to mesh element
 * @see none
 *
 *  arguments:
 * @param
 *
 * @return  none
 *
 */
template <class DataTypes>
void ClosestPointForceFieldForKalmanFilter<DataTypes>::addForceMesh(const core::MechanicalParams* mparams,DataVecDeriv& _f , const DataVecCoord& _x , const DataVecDeriv& _v )
{
    int t = (int)this->getContext()->getTime();
    helper::ReadAccessor<Data<helper::vector<unsigned int>>> vSourceControlMask = d_sourceControlMask;
    helper::WriteAccessor<Data<helper::vector<unsigned int>>> vTargetControlMask = d_targetControlMask;
    helper::vector<unsigned int> currentControlMask;

    sofa::helper::vector< tri > triangles;
    triangles = this->sourceTriangles.getValue();

    bool reinitv = false;

    helper::vector< bool > sourcevisible = this->sourceVisible.getValue();
    helper::vector< int > indicesvisible = this->indicesVisible.getValue();
    helper::vector< bool > sourceborder = this->sourceBorder.getValue();

        if (t%this->niterations.getValue() == 0)
        {

            if (this->npoints != (this->mstate->read(core::ConstVecCoordId::position())->getValue()).size())
            {
                reinit();
                reinitv = true;
            }
            this->npoints = (this->mstate->read(core::ConstVecCoordId::position())->getValue()).size();

        }

    double time = (double)getTickCount();
    double timef0 = (double)getTickCount();

        if(this->ks.getValue()==0) return;

    VecDeriv& f = *_f.beginEdit();       //WDataRefVecDeriv f(_f);
    const VecCoord& x = _x.getValue();			//RDataRefVecCoord x(_x);
    const VecDeriv& v = _v.getValue();			//RDataRefVecDeriv v(_v);
    ReadAccessor< Data< VecCoord > > tn(this->targetNormals);
    ReadAccessor< Data< VecCoord > > tp(this->targetPositions);
    ReadAccessor< Data< VecCoord > > tcp(this->targetContourPositions);

    if (t%this->niterations.getValue() == 0)
    {
        this->f_.resize(f.size());
        this->x_.resize(x.size());
        this->v_.resize(v.size());
    }

    const vector<Spring>& s = this->springs.getValue();
    this->dfdx.resize(s.size());
    this->closestPos.resize(s.size());
    if (d_controlMaskIsNotTransfered.getValue() == true) {
        currentControlMask.resize(s.size(), 0);
    }

    this->dfdx1.resize(s.size());
    this->m_potentialEnergy = 0;

    // get attraction/ projection factors
    Real attrF=(Real) this->blendingFactor.getValue();
    if(attrF<(Real)0.) attrF=(Real)0.;
    if(attrF>(Real)1.) attrF=(Real)1.;
    Real projF=((Real)1.-attrF);

    //closestpoint->updateClosestPointsGt();

    this->closestpoint->sourcePositions.setValue(this->mstate->read(core::ConstVecCoordId::position())->getValue());

    if (this->useVisible.getValue())
    this->closestpoint->sourceVisiblePositions.setValue(this->sourceVisiblePositions.getValue());

    this->closestpoint->targetPositions.setValue(this->targetPositions.getValue());
    this->closestpoint->targetBorder = this->targetBorder.getValue();
    this->closestpoint->sourceSurfacePositions.setValue(this->sourceSurfacePositions.getValue());
    this->closestpoint->sourceBorder = this->sourceBorder.getValue();

    time = (double)getTickCount();

    double error = 0;
    int nerror = 0;

        if(tp.size()==0)
            for (unsigned int i=0; i<s.size(); i++) this->closestPos[i]=x[i];
        else
        {
            if (!this->useContour.getValue())
                this->closestpoint->updateClosestPoints();
            else
            {
                if ((this->targetContourPositions.getValue()).size() > 0 && (this->sourceContourPositions.getValue()).size()>0 )
                {
                    this->closestpoint->targetContourPositions.setValue(this->targetContourPositions.getValue());
                    this->closestpoint->sourceContourPositions.setValue(this->sourceContourPositions.getValue());
                    this->closestpoint->sourceContourNormals.setValue(this->sourceContourNormals.getValue());
                    this->closestpoint->updateClosestPointsContours();
                }
                else this->closestpoint->updateClosestPoints();
            }

            double timeClosestPoint = ((double)getTickCount() - timef0)/getTickFrequency();

            std::cout << "TIME CLOSESTPOINT " << timeClosestPoint << std::endl;
            this->indices = this->closestpoint->getIndices();

            // count number of attractors
            this->cnt.resize(s.size()); this->cnt.fill(0);
                if(attrF>0)
                    if (!this->useVisible.getValue())
                    {
                        for (unsigned int i=0; i<tp.size(); i++)
                            if(!this->closestpoint->targetIgnored[i])
                                this->cnt[this->closestpoint->closestTarget[i].begin()->second]++;

                    }
                    else
                    {
                            if ((this->sourceVisiblePositions.getValue()).size()>0)
                            {
                                for (unsigned int i=0; i<tp.size(); i++)
                                {
                                    if(!this->closestpoint->targetIgnored[i])
                                        this->cnt[indicesvisible[this->closestpoint->closestTarget[i].begin()->second]]++;
                                }
                            }
                            else for (unsigned int i=0; i<tp.size(); i++) this->cnt[this->closestpoint->closestTarget[i].begin()->second]++;

                    }

                    if(this->theCloserTheStiffer.getValue())
                    {
                        // find the min and the max distance value from source point to target point
                        this->min=0;
                        this->max=0;
                            for (unsigned int i=0; i<x.size(); i++)
                            {
                                if((this->min)==0 || (this->min)>this->closestpoint->closestSource[i].begin()->first) this->min=this->closestpoint->closestSource[i].begin()->first;
                                if((this->max)==0 || (this->max)<this->closestpoint->closestSource[i].begin()->first) this->max=this->closestpoint->closestSource[i].begin()->first;
                            }
                    }

            // compute targetpos = projF*closestto + attrF* sum closestfrom / count
            int ivis=0;
            int kk = 0;
            unsigned int id;

            {
            if(projF>0)
            {
                if (!this->useVisible.getValue())
                {
                    if (this->useContour.getValue())//&& t%niterations.getValue() == 0)
                    {
                        if (this->targetContourPositions.getValue().size() > 0)
                        for (unsigned int i=0; i<s.size(); i++)
                        {
                            unsigned int id=this->closestpoint->closestSource[i].begin()->second;
                                if(!this->closestpoint->sourceIgnored[i])
                                {
                                    if(!sourceborder[i])
                                    {
                                        id=this->closestpoint->closestSource[i].begin()->second;
                                        if(this->projectToPlane.getValue() && tn.size()!=0) {
                                            this->closestPos[i]=/*(1-(Real)sourceWeights[i])**/(x[i]+tn[id]*dot(tp[id]-x[i],tn[id]))*projF;
                                        } else {
                                            this->closestPos[i]=/*(1-(Real)sourceWeights[i])**/tp[id]*projF;
                                        }
                                        if (d_controlMaskIsNotTransfered.getValue() == true) currentControlMask[i] = vSourceControlMask[id];

                                        if(!this->cnt[i]) this->closestPos[i]+=x[i]*attrF;
                                    }
                                    else
                                    {
                                        id=this->indices[kk];
                                        this->closestPos[i]=tcp[id]*projF;
                                        if (d_controlMaskIsNotTransfered.getValue() == true)  currentControlMask[i] = vSourceControlMask[id];

                                            if(!this->cnt[i]) this->closestPos[i]+=x[i]*attrF;
                                        kk++;
                                    }

                                }
                                else
                                {
                                    this->closestPos[i]=x[i]*projF;
                                        if(!this->cnt[i]) this->closestPos[i]+=x[i]*attrF;
                                }
                            }
                        }
                        else
                        {
                            for (unsigned int i=0; i<s.size(); i++)
                            {
                                unsigned int id=this->closestpoint->closestSource[i].begin()->second;
                                if(!this->closestpoint->sourceIgnored[i])
                                {
                                    if(this->projectToPlane.getValue() && tn.size()!=0) {
                                        this->closestPos[i]=(x[i]+tn[id]*dot(tp[id]-x[i],tn[id]))*projF;
                                        if (d_controlMaskIsNotTransfered.getValue() == true) currentControlMask[i] = vSourceControlMask[id];
                                    } else {
                                        this->closestPos[i]=tp[id]*projF;
                                        if (d_controlMaskIsNotTransfered.getValue() == true) currentControlMask[i] = vSourceControlMask[id];
                                    }
                                    /*if (sourceSurface[i])
                                    {
                                    closestPos[i]=(x[i]+ssn[i]*dot(tp[id]-x[i],ssn[i]))*projF;
                                    }
                                    else closestPos[i]=tp[id]*projF;*/
                                    if(!this->cnt[i]) this->closestPos[i]+=x[i]*attrF;
                                    //closestPos[i]+=x[i]*attrF;
                                }
                                else
                                {
                                    this->closestPos[i]=x[i]*projF;
                                    if(!this->cnt[i]) this->closestPos[i]+=x[i]*attrF;
                                }
                            }
                        }
                    }
                    else
                    {

                        if (this->useContour.getValue())
                        {
                            if (this->targetContourPositions.getValue().size() > 0)
                                for (unsigned int i=0; i<s.size(); i++)
                                {
                                    std::cout << "ii " << i<< " " << sourcevisible.size() << std::endl;


                                    //if(/*!closestpoint->sourceIgnored[i] &&*/ sourcevisible[i])
                                    {
                                        if (sourcevisible[i])
                                        {
                                            if(!sourceborder[i])
                                            {
                                            id=this->closestpoint->closestSource[ivis].begin()->second;

                                                if(this->projectToPlane.getValue() && tn.size()!=0) {
                                                    this->closestPos[i]=/*(1-(Real)sourceWeights[i])**/(x[i]+tn[id]*dot(tp[id]-x[i],tn[id]))*projF;
                                                } else {
                                                    this->closestPos[i]=/*(1-(Real)sourceWeights[i])**/tp[id]*projF;
                                                }
                                                if (d_controlMaskIsNotTransfered.getValue() == true) currentControlMask[i] = vSourceControlMask[id];
                                                if(!this->cnt[i]) this->closestPos[i]+=x[i]*attrF;
                                            }
                                            else
                                            {
                                                unsigned int id=this->indices[kk];
                                                this->closestPos[i]=tcp[id]*projF;
                                                if (d_controlMaskIsNotTransfered.getValue() == true) currentControlMask[i] = vSourceControlMask[id];
                                                if(!this->cnt[i]) this->closestPos[i]+=x[i]*attrF;
                                                kk++;
                                            }
                                            ivis++;
                                        }
                                        else
                                        {
                                            this->closestPos[i]=x[i]*projF;
                                            if(!this->cnt[i]) { this->closestPos[i]+=x[i]*attrF; }
                                        }

                                    }
                                }
                            }
                            else
                            {
                                for (unsigned int i=0; i<s.size(); i++)
                                {
                                    //if(/*!closestpoint->sourceIgnored[i] &&*/ sourcevisible[i])
                                    {
                                        if (sourcevisible[i])
                                        {
                                            unsigned int id=this->closestpoint->closestSource[ivis].begin()->second;
                                            if(!this->closestpoint->sourceIgnored[ivis])
                                            {
                                                this->closestPos[i]=tp[id]*projF;
                                                if (d_controlMaskIsNotTransfered.getValue() == true) currentControlMask[i] = vSourceControlMask[id];
                                                error += this->computeError(x[i],tp[id]);
                                                nerror++;
                                            }
                                            else this->closestPos[i]=x[i]*projF;
                                            ivis++;
                                        }
                                        else
                                        {
                                            this->closestPos[i]=x[i]*projF;
                                            //closestPos[i]=x[i];
                                            /*closestPos[i]=x[i]*projF;
                                            if(!cnt[i]) {closestPos[i]+=x[i]*attrF;}*/

                                        }
                                    if(!this->cnt[i]) { this->closestPos[i]+=x[i]*attrF; }

                                    }
                                }
                            }
                        }
                    }
                    else for (unsigned int i=0; i<s.size(); i++) { if(!this->cnt[i]) this->closestPos[i]=x[i]; else this->closestPos[i].fill(0); }

                // attraction

                if(attrF>0)
                        if(!this->useVisible.getValue())
                        {
                            for (unsigned int i=0; i<tp.size(); i++)
                            {
                                unsigned int id=this->closestpoint->closestTarget[i].begin()->second;
                                if( !this->useContour.getValue() && !this->closestpoint->targetIgnored[i] && t > this->niterations.getValue()) //&& !targetBackground[i])
                                {
                                    /*if (sourceSurface[id])
                                    {
                                            //std::cout << " ssn " << i << " " << ssn[id][1] << std::endl;

                                    closestPos[id]+=(tp[i]+ssn[id]*dot(x[id]-tp[i],ssn[id]))*attrF/(Real)cnt[id];
                                    }
                                    else*/
                                    {
                                            //closestPos[i]=(tp[i]+sn[id]*dot(x[id]-tp[i],sn[id]))*attrF/(Real)cnt[id];
                                            this->closestPos[id]+=tp[i]*attrF/(Real)this->cnt[id];
                                            if (d_controlMaskIsNotTransfered.getValue() == true) currentControlMask[id] |= vSourceControlMask[i];  // ???
                                    }
                                }
                                    else if (this->targetContourPositions.getValue().size() > 0)
                                    {
                                        if (this->useContour.getValue() && t > this->niterations.getValue() )//&& t%niterations.getValue() > 0)
                                            this->closestPos[id]+=tp[i]*attrF/(Real)this->cnt[id];
                                            if (d_controlMaskIsNotTransfered.getValue() == true) currentControlMask[id] |= vSourceControlMask[i];  // ???
                                    }
                                }

                        }
                        else
                        {

                            {
                                if( !this->useContour.getValue())
                                {
                                    int kkt = 0;
                                    for (unsigned int i=0; i<tp.size(); i++)
                                    {
                                        unsigned int id=this->closestpoint->closestTarget[i].begin()->second;
                                        if(!this->closestpoint->targetIgnored[i]) //&& !targetBackground[i])
                                        {
                                            /*if (sourceSurface[id])
                                            {
                                                //std::cout << " ssn " << i << " " << ssn[id][1] << std::endl;
                                                closestPos[id]+=(tp[i]+ssn[id]*dot(x[id]-tp[i],ssn[id]))*attrF/(Real)cnt[id];
                                            }
                                            else*/
                                            {
                                                //closestPos[i]=(tp[i]+sn[id]*dot(x[id]-tp[i],sn[id]))*attrF/(Real)cnt[id];
                                                //closestPos[id]+=tp[i]*attrF/(Real)cnt[id];
                                                unsigned int id1 = indicesvisible[id];
                                                this->closestPos[id1]+=tp[i]*attrF/(Real)this->cnt[id1];
                                                if (d_controlMaskIsNotTransfered.getValue() == true) currentControlMask[id1] |= vSourceControlMask[i];  // ???
                                            }
                                        }
                                    }
                                }
                                else if (this->targetContourPositions.getValue().size() > 0)
                                {
                                    for (unsigned int i=0; i<tp.size(); i++)
                                    {
                                        unsigned int id=this->closestpoint->closestTarget[i].begin()->second;
                                        if(!this->closestpoint->targetIgnored[i])
                                        {
                                            unsigned int id1;
                                            //if (!rgbddataprocessing->targetBorder[i])
                                            id1 = indicesvisible[id];
                                            /*else
                                            {
                                                id1 = indicesTarget[kkt];
                                                kkt++;
                                            }*/
                                            //if (sourceborder[id1] && rgbddataprocessing->targetBorder[i])
                                            this->closestPos[id1]+=tp[i]*attrF/(Real)this->cnt[id1];
                                            if (d_controlMaskIsNotTransfered.getValue() == true) currentControlMask[id1] |= vSourceControlMask[i];  // ???
                                        }
                                        //if(rgbddataprocessing->targetBorder[i])
                                    }
                                }
                            }
                        }
                    }
                }

        this->ind = 0;
        int ivis =0;
        this->sourcew.resize(s.size());
        int npointspen = 0;

            for (unsigned int i=0; i<s.size(); i++)
            {
                //serr<<"addForce() between "<<springs[i].m1<<" and "<<closestPos[springs[i].m1]<<sendl;
                if (t%(this->niterations.getValue()) == 0 && t > 1)
                {
                    //if( )

                    if (!this->useContourWeight.getValue() || this->targetContourPositions.getValue().size()==0 )
                    this->addSpringForce(this->m_potentialEnergy,f,x,v, i, s[i]);
                    else this->addSpringForceWeight(this->m_potentialEnergy,f,x,v, i,ivis, s[i]);//this->addSpringForceWeight(m_potentialEnergy,f,x,v, i, s[i]);
                    if (sourcevisible[i])
                        ivis++;
                }
            }

    _f.endEdit();

    if (d_controlMaskIsNotTransfered.getValue() == true) {
        d_controlMaskIsNotTransfered.setValue(false);
        vTargetControlMask.clear();
        for (size_t index = 0; index < currentControlMask.size(); index++) {
            if (currentControlMask[index] == 1) {
                vTargetControlMask.push_back(index);
            }
        }
    }
}



/**
 *        ClosestPointForceFieldForKalmanFilter::draw
 * <p>
 *   description:
 *     draw internal data
 * @see none
 *
 *  arguments:
 * @param  vparams - a pointer to drawing tool
 *
 * @return  none
 *
 */
template <class DataTypes>
void ClosestPointForceFieldForKalmanFilter<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    ClosestPointForceField<DataTypes>::draw(vparams);

    // draw control and observation points
    if (d_drawObservations.getValue() == true)
    {
        helper::ReadAccessor<Data<helper::vector<unsigned int>>> vTargetControlMask = d_targetControlMask;
        helper::vector< int > indicesData = this->indicesVisible.getValue();

        std::vector<Vector3> controlPos;
        controlPos.clear();
        std::vector<Vector3> observationPos;
        observationPos.clear();

        bool found;

        //for (size_t index = 0; index < this->closestPos.size(); index++) {
        for (size_t index = 0; index < indicesData.size(); index++) {
            size_t vectorIndex = static_cast<size_t>(indicesData[index]);
            found = false;
            for (size_t controlIndex = 0; controlIndex < vTargetControlMask.size(); controlIndex++) {
                if (vTargetControlMask[controlIndex] == vectorIndex) {
                    found = true;
                    break;
                }
            }

            if (found) {
                controlPos.push_back(this->closestPos[vectorIndex]);
            } else {
                observationPos.push_back(this->closestPos[vectorIndex]);
            }
        }

        vparams->drawTool()->drawPoints(controlPos, 8.0, Vec4f(1.0, 0.0, 0.0, 1.0)); // draw control points with red color
        vparams->drawTool()->drawPoints(observationPos, 8.0, Vec4f(0.0, 1.0, 0.0, 1.0)); // draw observation points with green color
    }

}

}
}
} // namespace sofa

//#endif  /* SOFA_COMPONENT_INTERACTIONFORCEFIELD_RegistrationForceFieldCam_INL */


