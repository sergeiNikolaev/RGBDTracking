/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#define SOFA_RGBDTRACKING_RENDERINGMANAGER_CPP

#include "ModRenderingManager.h"
#include <sofa/simulation/VisualVisitor.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/core/core.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/helper/accessor.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/helper/vector.h>
#include <SofaBaseTopology/TopologyData.h>

#include <sofa/helper/gl/Color.h>
#include <sofa/core/ObjectFactory.h>
#include <SofaBaseVisual/InteractiveCamera.h>
#include <sofa/core/behavior/ForceField.inl>
#include <sofa/simulation/Simulation.h>

#include <opencv2/opencv.hpp>



namespace sofa
{

namespace component
{

namespace visualmodel
{

    using namespace sofa::defaulttype;

      SOFA_DECL_CLASS(ModRenderingManager)

      // Register in the Factory
      int ModRenderingManagerClass = core::RegisterObject("Compute forces based on closest points from/to a target surface/point set")
        .add< ModRenderingManager >()
    ;


using namespace core::visual;

const std::string ModRenderingManager::DEPTH_OF_FIELD_VERTEX_SHADER = "shaders/depthOfField.vert";
const std::string ModRenderingManager::DEPTH_OF_FIELD_FRAGMENT_SHADER = "shaders/depthOfField.frag";

ModRenderingManager::ModRenderingManager()
    :zNear(initData(&zNear, (double) 1.0, "zNear", "Set zNear distance (for Depth Buffer)"))
    ,zFar(initData(&zFar, (double) 100.0, "zFar", "Set zFar distance (for Depth Buffer)"))
    ,useRenderAR(initData(&useRenderAR, true, "useRenderAR", "Option to enable augmented reality overlay"))
    ,postProcessEnabled (true)
{
    // TODO Auto-generated constructor stub

}

ModRenderingManager::~ModRenderingManager()
{

}


void ModRenderingManager::init()
{

}

void ModRenderingManager::initVisual()
{

}

void ModRenderingManager::preDrawScene(VisualParams* vp)
{

}

bool ModRenderingManager::drawScene(VisualParams* vp)
{
    sofa::simulation::Node::SPtr root = dynamic_cast<simulation::Node*>(this->getContext());
    sofa::component::visualmodel::BaseCamera::SPtr currentCamera;
    root->get(currentCamera);

    double znear = currentCamera->getZNear();
    double zfar = currentCamera->getZFar();

    zNear.setValue(znear);
    zFar.setValue(zfar);

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);

    int wdth = viewport[2];
    int hght = viewport[3];
    depths = new float[wdth * hght ];

    cv::Mat depthm;
    depthm.create(hght, wdth, CV_32F);

    std::cout << " znear1 " << znear << " zfar1 " << zfar << std::endl;
    std::cout << " viewport1 " << viewport[0] << " "<< viewport[1] << " " << viewport[2] << " " << viewport[3] << std::endl;

    int t = (int)this->getContext()->getTime();
    if (t > 1){

    glReadPixels(viewport[0], viewport[1], viewport[2], viewport[3], GL_DEPTH_COMPONENT, GL_FLOAT, depths);


    for (int j = 0; j < wdth; j++)
        for (int i = 0; i< hght; i++)
        {

                if ((double)(float)depths[j+i*wdth]	< 1  && (double)(float)depths[j+i*wdth]	> 0)
                {
                /*double clip_z = (depths[j+i*wdth] - 0.5) * 2.0;
                double zlin =  2*znear*zfar/(clip_z*(zfar-znear)-(zfar+znear));
                std::cout << " depth1 " << (double)depths[j+i*wdth] << " zlin " << zlin << std::endl;*/
                depthm.at<float>(hght-i-1,j) = depths[j+i*wdth];
                }
        }
        }

    depthmat = depthm.clone();

    cv::Mat depthmat1;
    depthmat.convertTo(depthmat1, CV_8UC1, 255);
    //cv::imwrite("depth00.png",depthmat1);
    //cv::imwrite("depthmat10.png", depthmat1);


    if (useRenderAR.getValue())
    {
        texturemat.create(hght,wdth, CV_8UC3);
        glReadBuffer(GL_FRONT);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(viewport[0], viewport[1], viewport[2], viewport[3], GL_RGB, GL_UNSIGNED_BYTE, texturemat.data);
        glReadBuffer(GL_BACK);
    }


    return false;
}

void ModRenderingManager::postDrawScene(VisualParams* /*vp*/)
{


}

void ModRenderingManager::handleEvent(sofa::core::objectmodel::Event* event)
{
if (dynamic_cast<simulation::AnimateBeginEvent*>(event))
    {

    }
}

} //visualmodel

} //component

} //sofa
