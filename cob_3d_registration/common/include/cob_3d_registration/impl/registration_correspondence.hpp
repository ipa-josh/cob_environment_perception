/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: registration
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: goa-jh
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Nov 29, 2011
 * ToDo:
 *
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

namespace cob_3d_registration {

  template <typename Point>
  bool Registration_Corrospondence<Point>::compute_features() {
    if(register_.size()<1) {
      register_ = *this->input_org_;
      return true;
    }

    return keypoints_ && keypoints_->compute(register_, *this->input_org_);
  }

  template <typename Point>
  bool Registration_Corrospondence<Point>::compute_corrospondences() {

    all_correspondences_.reset(new pcl::registration::Correspondences);
    keypoints_->getCorrespondences (*all_correspondences_);

    //  ROS_INFO("rejectBadCorrespondences %d %d",all_correspondences->size(), remaining_correspondences.size());

    return true;
  }

  template <typename Point>
  bool Registration_Corrospondence<Point>::compute_transformation() {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // Find correspondences between keypoints in FPFH space
    pcl::registration::CorrespondencesPtr good_correspondences (new pcl::registration::Correspondences);

    rejectBadCorrespondences(all_correspondences_, *good_correspondences);

    if(good_correspondences->size()<3) {
      ROS_ERROR("not enough coresspondences");
      return false;
    }

    // Obtain the best transformation between the two sets of keypoints given the remaining correspondences
    pcl::registration::TransformationEstimationSVD<Point, Point> trans_est;
    trans_est.estimateRigidTransformation (*keypoints_->getSourcePoints(), *keypoints_->getTargetPoints(), *good_correspondences, transform);

    this->transformation_ = transform*this->transformation_;

    std::cout<<"transf\n"<<this->transformation_<<"\n";

    register_ = *this->input_org_;

    return true;
  }


  template <typename Point>
  void Registration_Corrospondence<Point>::rejectBadCorrespondences (const pcl::registration::CorrespondencesPtr &all_correspondences,
                                                                     pcl::registration::Correspondences &remaining_correspondences)
                                                                     {
    pcl::registration::CorrespondenceRejectorDistance rej;
    //rej.setInputCloud(keypoints_src);
    //rej.setInputTarget(keypoints_tgt);
    rej.setMaximumDistance (rejection_dis_);
    rej.setInputCorrespondences (all_correspondences);
    rej.getCorrespondeces (remaining_correspondences);

    ROS_INFO("rejectBadCorrespondences %d %d", (int)all_correspondences->size(), (int)remaining_correspondences.size());
                                                                     }


}

#define PCL_INSTANTIATE_Registration_Corrospondence(T) template class PCL_EXPORTS cob_3d_registration::Registration_Corrospondence<T>;

