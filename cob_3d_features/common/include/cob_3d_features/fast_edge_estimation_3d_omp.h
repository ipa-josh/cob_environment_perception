/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_mapping_features
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 10/2011
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef __FAST_EDGE_ESTIMATION_3D_OMP_H__
#define __FAST_EDGE_ESTIMATION_3D_OMP_H__

#include "cob_3d_features/fast_edge_estimation_3d.h"

namespace cob_3d_features
{
  template <typename PointInT, typename PointNT, typename PointOutT>
    class FastEdgeEstimation3DOMP : public FastEdgeEstimation3D<PointInT,PointNT,PointOutT>
  {
    public:

    using OrganizedFeatures<PointInT, PointOutT>::input_;
    using OrganizedFeatures<PointInT, PointOutT>::indices_;
    using OrganizedFeatures<PointInT, PointOutT>::surface_;
    using OrganizedFeatures<PointInT, PointOutT>::feature_name_;
    using FastEdgeEstimation3D<PointInT,PointNT,PointOutT>::normals_;

    typedef pcl::PointCloud<PointOutT> PointCloudOut;

    public:
      /** \brief Empty constructor. */
    FastEdgeEstimation3DOMP ()
      {
	feature_name_ = "FastEdgeEstimation3DOMP";
      };

    protected:

      void
	computeFeature (PointCloudOut &output);
  };
}

#endif  //#ifndef __FAST_EDGE_ESTIMATION_3D_OMP_H__
