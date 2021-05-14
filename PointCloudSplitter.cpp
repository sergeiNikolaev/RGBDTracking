/******************************************************************************
  *  point extractor class
 *****************************************************************************/

/* include files */
#include "PointCloudSplitter.h"
#include "PointCloudSplitter.inl"
#include <sofa/core/ObjectFactory.h>


namespace sofa {

namespace component {

namespace behavior {

using namespace sofa::defaulttype;


SOFA_DECL_CLASS(PointCloudSplitter)

int PointCloudSplitterClass = core::RegisterObject("Solver toticulated system objects")
.add<PointCloudSplitter >();


} // namespace bahavior

} // namespace compoinent

} // namespace sofa
