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

#define SOFA_RGBDTRACKING_IMAGECONVERTER_CPP

#include "ImageConverter.inl"

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/BaseContext.h>

using std::cerr;
using std::endl;

namespace sofa
{

namespace core
{

namespace objectmodel
{

    using namespace sofa::defaulttype;

    SOFA_DECL_CLASS(ImageConverter)

    // Register in the Factory
    int ImageConverterClass = core::RegisterObject("Compute forces based on closest points from/to a target surface/point set")
    .add< ImageConverter<Vec3Types,ImageUC> >()
    .add< ImageConverter<Vec3Types,ImageUS> >()
    .add< ImageConverter<Vec3Types,ImageF> >()
    ;

    template class SOFA_RGBDTRACKING_API ImageConverter<Vec3Types,ImageUC>;
    template class SOFA_RGBDTRACKING_API ImageConverter<Vec3Types,ImageUS>;
    template class SOFA_RGBDTRACKING_API ImageConverter<Vec3Types,ImageF>;

}

}

} // namespace sofa



