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

#define SOFA_RGBDTRACKING_MESHPROCESSING_FORKALMANFILTER_INL

#include "MeshProcessingForKalmanFilter.h"


using std::cerr;
using std::endl;

namespace sofa
{

namespace core
{

namespace objectmodel
{

using namespace sofa::defaulttype;
using namespace helper;



/**
 *        MeshProcessingForKalmanFilter::MeshProcessingForKalmanFilter
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
MeshProcessingForKalmanFilter<DataTypes>::MeshProcessingForKalmanFilter( )
 : MeshProcessing<DataTypes>()
 , d_controlPointsMask( initData(&d_controlPointsMask, "controlPointMask", "mask to detect control points") )
 , d_controlPointsAreNotExtracted( initData(&d_controlPointsAreNotExtracted, true, "controlPointsExtracted", "show if control points are extracted") )
{
    this->f_listening.setValue(true);

}



/**
 *        MeshProcessingForKalmanFilter::init
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
template <class DataTypes>
void MeshProcessingForKalmanFilter<DataTypes>::init()
{
    MeshProcessing<DataTypes>::init();
}


/**
 *        MeshProcessingForKalmanFilter::bwdinit
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
/**
 *  this function is not used anymore but is here to hold an example of working with sofa scene
 */
template <class DataTypes>
void MeshProcessingForKalmanFilter<DataTypes>::bwdInit()
{
    //// find the mechanical object
    //std::cout << "backward init for Mesh processing for Kalman filter class" << std::endl;
    //sofa::simulation::Node::SPtr currentNode = dynamic_cast<simulation::Node*>(this->getContext());
    //sofa::simulation::Node::SPtr root = dynamic_cast<simulation::Node*>(currentNode->getRootContext());
    //helper::vector<sofa::core::behavior::MechanicalState<DataTypes> *> mechanicalModels;
    //root->get<sofa::core::behavior::MechanicalState<DataTypes>>(&mechanicalModels, this->getTags(), sofa::core::objectmodel::BaseContext::SearchDown);
    //unsigned int modelIndex = 0;
    //for (size_t i = 0; i < mechanicalModels.size(); i++) {
    //    std::cout << "mechanical model " << mechanicalModels[i]->getName() << " found" << std::endl;
    //    if (!mechanicalModels[i]->getName().compare("Volume")) {
    //        modelIndex = i;
    //    }
    //}
    //std::cout << "right index is " << modelIndex << std::endl;
    //this->mstate = mechanicalModels[modelIndex];
}



/**
 *        MeshProcessingForKalmanFilter::getSourceVisible
 * <p>
 *   description:
 *     select the set of control points
 * @see none
 *
 *  arguments:
 * @param  znear - front cutting plane
 * @param  zfar - back cutting plane
 *
 * @return  none
 *
 */
template<class DataTypes>
void MeshProcessingForKalmanFilter<DataTypes>::getSourceVisible(double znear, double zfar)
{
    // select indices for control points only among visible elements
    helper::WriteAccessor<Data<helper::vector<unsigned int>>> vControlMask = d_controlPointsMask;

    Vector4 camParam = this->cameraIntrinsicParameters.getValue();

    this->rgbIntrinsicMatrix(0,0) = camParam[0];
    this->rgbIntrinsicMatrix(1,1) = camParam[1];
    this->rgbIntrinsicMatrix(0,2) = camParam[2];
    this->rgbIntrinsicMatrix(1,2) = camParam[3];

    //if (t%2 == 0)
    {

    cv::Mat _rtd0, depthr, depthu;
    this->renderingmanager->getDepths(depthr);
    this->depthrend = depthr.clone();

    this->wdth = this->depthrend.cols;
    this->hght = this->depthrend.rows;

    _rtd0.create(this->hght, this->wdth, CV_8UC1);
    double depthsN[this->hght * this->wdth ];

    std::vector<cv::Point> ptfgd;
    ptfgd.resize(0);
    cv::Point pt;

    std::cout << " nvisible0 " <<  std::endl;

    cv::Mat& data = this->depthrend;
    double clip_z;
    //double timef0 = (double)getTickCount();

    for (int j = 0; j < this->wdth; j++)
        for (int i = 0; i< this->hght; i++)
        {
            if ((double)data.at<float>(this->hght-i-1,j)	< 1  && (double)data.at<float>(this->hght-i-1,j)	> 0.001)
            {
                //if (j >= rectRtt.x && j < rectRtt.x + rectRtt.width && i >= rectRtt.y && i < rectRtt.y + rectRtt.height) {
                //if ((double)(float)depths1[j-rectRtt.x+(i-rectRtt.y)*(rectRtt.width)]	< 1){

                clip_z = (data.at<float>(this->hght-i-1,j) - 0.5) * 2.0;
                //double clip_z = (depths1[j-rectRtt.x+(i-rectRtt.y)*(rectRtt.width)] - 0.5) * 2.0;
                depthsN[j+i*this->wdth] = 2*znear*zfar/(clip_z*(zfar-znear)-(zfar+znear));
                //std::cout << " dpethsN " <<  depthsN[j+i*wdth] << std::endl;

                if ( depthsN[j+i*this->wdth] > -10 && depthsN[j+i*this->wdth] < -0.05)
                {
                pt.x = j;
                pt.y = i;
                ptfgd.push_back(pt);
                _rtd0.at<uchar>(this->hght-i-1,j) = 255;
                }
                else _rtd0.at<uchar>(this->hght-i-1,j) = 0;
            }
            else
            {
                _rtd0.at<uchar>(this->hght-i-1,j) = 0;
                depthsN[j+i*this->wdth] = 0;
            }

        }

    {
        cv::Rect rectrtt = cv::boundingRect(ptfgd);

        if (ptfgd.size()==0)
        {
            this->rectRtt.x = 0;
            this->rectRtt.y = 0;
            this->rectRtt.height = this->hght;
            this->rectRtt.width = this->wdth;
        }
        else //if ((double)rectrtt.height/rectRtt.height - 1 < 0.2 && (double)rectrtt.width/rectRtt.width - 1 < 0.2)
        {
             this->rectRtt = rectrtt;

            if (this->rectRtt.x >=10)
            this->rectRtt.x -= 10;
            if (this->rectRtt.y >=10)
            this->rectRtt.y -= 10;
            if (this->rectRtt.y + this->rectRtt.height < this->hght - 20)
            this->rectRtt.height += 20;
            if (this->rectRtt.x + this->rectRtt.width < this->wdth - 20)
            this->rectRtt.width += 20;

            std::cout << " rect1 " << this->rectRtt.x << " " << this->rectRtt.y << " rect2 " << this->rectRtt.width << " " << this->rectRtt.height << std::endl;

            this->depthMap = _rtd0.clone();
            /*depthr.convertTo(depthu, CV_8UC1, 10);
            cv::namedWindow("depth_map");
            cv::imshow("depth_map",depthMap);
            cv::waitKey(1);
            cv::imwrite("depth000.png",depthMap);*/
            //cv::imwrite("depth01.png", depthMap);
        }

        Vector4 bbox;
        bbox[0] = this->rectRtt.x;
        bbox[1] = this->rectRtt.y;
        bbox[2] = this->rectRtt.width;
        bbox[3] = this->rectRtt.height;
        this->BBox.setValue(bbox);
    }
    const VecCoord& x = this->mstate->read(core::ConstVecCoordId::position())->getValue();


    helper::vector<bool> sourcevisible;
    sourcevisible.resize(x.size());
    VecCoord sourceVis;
    Vector3 pos;

    helper::vector< int > indicesvisible;
    indicesvisible.resize(0);
    unsigned int delta = 100;
    unsigned int deltaIndex = 0;

    for (size_t k = 0; k < x.size(); k++)
    {
        int x_u = (int)(x[k][0]*this->rgbIntrinsicMatrix(0,0)/x[k][2] + this->rgbIntrinsicMatrix(0,2));
        int x_v = (int)(x[k][1]*this->rgbIntrinsicMatrix(1,1)/x[k][2] + this->rgbIntrinsicMatrix(1,2));
        //std::cout << " model value " << x[k][2] << std::endl;
        //std::cout << " depths0 " << (float)x_u << " " << (double)x_v << std::endl;
        //std::cout << " depths0 " << (float)depthsN[x_u+(hght-x_v-1)*wdth] << " " << (double)visibilityThreshold.getValue() << std::endl;

        if (x_u>=0 && x_u<this->wdth && x_v<this->hght && x_v >= 0) {
            if((float)abs(depthsN[x_u+(this->hght-x_v-1)*this->wdth]+(float)x[k][2]) < this->visibilityThreshold.getValue() || (float)depthsN[x_u+(this->hght-x_v-1)*this->wdth] == 0)
            {
                sourcevisible[k] = true;
                pos = x[k];
                sourceVis.push_back(pos);
                indicesvisible.push_back(k);
                deltaIndex++;
                if (deltaIndex > delta) {
                    vControlMask.push_back(k);
                    deltaIndex = 0;
                    delta = 100;
                }
            }
            else
            {
                sourcevisible[k] = false;
            }
        }
        else {sourcevisible[k] = false;}

    }

    this->sourceVisiblePositions.setValue(sourceVis);
    this->sourceVisible.setValue(sourcevisible);
    this->indicesVisible.setValue(indicesvisible);

    helper::ReadAccessor<Data<helper::vector<unsigned int>>> vReadControlMask = d_controlPointsMask;
    std::cout << "\n\n\n\n control mask data: " << std::endl;
    for (unsigned int index = 0; index < vReadControlMask.size(); index++) {
        std::cout << vReadControlMask[index] << " ";
    }
    std::cout << "\n\n\n\n" << std::endl;

    }
}



template<class DataTypes>
void MeshProcessingForKalmanFilter<DataTypes>::updateSourceVisible()
{
    // do nothing, the mask will be the same
}



/**
 *        MeshProcessingForKalmanFilter::handleEvent
 * <p>
 *   description:
 *     a methood that does something
 * @see none
 *
 *  arguments:
 * @param  event - a pointer to some event
 *
 * @return  none
 *
 */
template <class DataTypes>
void MeshProcessingForKalmanFilter<DataTypes>::handleEvent(sofa::core::objectmodel::Event *event)
{
    MeshProcessing<DataTypes>::handleEvent(event);

    if (dynamic_cast<simulation::AnimateBeginEvent*>(event))
    {
        if (d_controlPointsAreNotExtracted.getValue()) {
            double znear = this->renderingmanager->getZNear();
            double zfar = this->renderingmanager->getZFar();
            getSourceVisible(znear, zfar);
            d_controlPointsAreNotExtracted.setValue(false);
        } else {
            updateSourceVisible();
        }
    }
}



/**
 *        MeshProcessingForKalmanFilter::~MeshProcessingForKalmanFilter
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
MeshProcessingForKalmanFilter<DataTypes>::~MeshProcessingForKalmanFilter()
{
}


}
}
} // namespace sofa

