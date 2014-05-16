/*
 * registration_info.cpp
 *
 *  Created on: 22.10.2012
 *      Author: josh
 */


// external includes:
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>

// package includes:
#include "cob_3d_registration/general_registration.h"
#include "cob_3d_registration/feature_container.h"
#include "cob_3d_registration/registration_correspondence.h"
#include "cob_3d_registration/impl/registration_correspondence.hpp"

PCL_INSTANTIATE(Registration_Corrospondence, PCL_XYZ_POINT_TYPES)
