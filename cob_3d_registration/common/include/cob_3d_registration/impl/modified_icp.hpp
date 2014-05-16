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
 *  ROS stack name: cob_vision
 * \note
 *  ROS package name: dynamic_tutorials
 *
 * \author
 *  Author: goa-jh
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: Oct 26, 2011
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
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

#pragma once

#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree.h>
#include "../feature_container.h"
#ifdef PCL_VERSION_COMPARE
  #include <pcl/registration/registration.h>
  #include <pcl/features/feature.h>
  #include <pcl/search/kdtree.h>
  #include <pcl/search/search.h>
#endif
#ifdef GICP_ENABLE
#include <pcl/registration/gicp.h>
#include <pcl/registration/transformation_estimation_lm.h>
#endif

/**
 * interface for setting up icp and derivats
 * needed to use different backends
 */
class ModifiedICP_G {
public:
  virtual void setLM()=0;
  virtual void setSearchFeatures(FeatureContainerInterface* features)=0;
};

/**
 * the modified icp allows (in pcl 1.1) to use own distance metric and obtaining the number of needed iterations
 */
template <typename ParentClass, typename PointT>
class ModifiedICP_T : public ParentClass, public ModifiedICP_G {
  const FeatureContainerInterface* features_;

  /**
   * own search tree which passes down search calls
   */
  template <typename PointT2>
  #ifdef PCL_VERSION_COMPARE
    #if PCL_MINOR_VERSION > 6 // not tested with pcl 1.4 yet
      class FeatureSearch: public pcl::search::KdTree<PointT2>
    #else
      class FeatureSearch: public pcl::KdTreeFLANN<PointT2>
    #endif
  #else
    class FeatureSearch : public pcl::KdTree<PointT2>
  #endif
  {
    FeatureContainerInterface* features_;

    void error() const {ROS_ERROR("unsupported");}

  public:
    FeatureSearch(FeatureContainerInterface* features):
      features_(features)
      { }

    /// searching through feature container
    virtual int
    nearestKSearch (const pcl::PointCloud<PointT2> &cloud, int index, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances)
    {
      features_->findFeatureCorrespondences (index, k_indices, k_sqr_distances);
      return k_indices.size();
    }

    /// not implemented
    virtual int
    nearestKSearch (const PointT2 &p_q, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances) {error();return 0;}

    /// not implemented
    virtual int
    nearestKSearch (int index, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances) {error();return 0;}

    /// not implemented
    virtual int
    radiusSearch (const pcl::PointCloud<PointT2> &cloud, int index, double radius, std::vector<int> &k_indices,
                  std::vector<float> &k_sqr_distances, int max_nn = INT_MAX) const {error();return 0;}

    /// not implemented
    virtual int
    radiusSearch (const PointT2 &p_q, double radius, std::vector<int> &k_indices,
                  std::vector<float> &k_sqr_distances, int max_nn = INT_MAX) const {error();return 0;}

    /// not implemented
    virtual int
    radiusSearch (int index, double radius, std::vector<int> &k_indices,
                  std::vector<float> &k_sqr_distances, int max_nn = INT_MAX) const {error();return 0;}

    #if PCL_MINOR_VERSION > 6
    virtual const std::string& getName() const {return "ICP Feature";}
    #else
    virtual std::string getName() const {return "ICP Feature";}
    #endif

  };

  public:

  /// get the needed iteration for computing
  int getNeededIterations() {return this->nr_iterations_;}

  /// use LM instead of SVD (since pcl 1.3)
  void setLM() {
#ifdef GICP_ENABLE
    this->min_number_correspondences_ = 4;
    this->reg_name_ = "IterativeClosestPointNonLinear";

    this->transformation_estimation_.reset (new pcl::registration::TransformationEstimationLM<PointT, PointT>);
#endif
  }

  /// setup distance metric (hack for features)
  void setSearchFeatures(FeatureContainerInterface* features) {
    this->features_ = features;
    //this->tree_.reset( static_cast<typename FeatureSearch<PointT>::BaseSearch*>(new FeatureSearch<PointT>(features)) );
    this->tree_.reset( new FeatureSearch<PointT>(features) );
  }

};

/**
 * modified icp with icp
 */
template <class PointT>
class ModifiedICP : public ModifiedICP_T<pcl::IterativeClosestPoint<PointT,PointT>, PointT>
{
};


#ifdef GICP_ENABLE
/**
 * modified icp with gicp
 * LM and features are not supported!
 */
template <class PointT>
class ModifiedGICP : public pcl::GeneralizedIterativeClosestPoint<PointT,PointT>, public ModifiedICP_G
{
  public:
  int getNeededIterations() {return 1;} //dummy

  void setLM() {
    std::cerr<<"ERROR: not supported\n";
  }

  void setSearchFeatures(FeatureContainerInterface* features) {
    std::cerr<<"ERROR: not supported\n";
  }
};
#endif

