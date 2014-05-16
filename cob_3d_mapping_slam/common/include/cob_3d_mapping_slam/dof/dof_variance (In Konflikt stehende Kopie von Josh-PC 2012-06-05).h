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
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
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
/*
 * dof_variance.h
 *
 *  Created on: 27.05.2012
 *      Author: josh
 */

#ifndef DOF_VARIANCE_H_
#define DOF_VARIANCE_H_

#include "euler.h"
#include <ros/time.h>

//! collecting descirptive transformations for 6-DOFs
namespace DOF6
{

  /**
   * use recursive to add n sources
   */
  template <typename _SOURCE1, typename _SOURCE2>
  class DOF6_Source
  {
  public:
    typedef _SOURCE1 SOURCE1;
    typedef _SOURCE2 SOURCE2;
    typedef typename SOURCE1::TYPE TYPE;
    typedef Eigen::Matrix<TYPE,3,1> Vector;
    typedef boost::shared_ptr<SOURCE1> Source1_Ptr;
    typedef boost::shared_ptr<SOURCE2> Source2_Ptr;

  private:
    Source1_Ptr src1_;
    Source2_Ptr src2_;

  public:

    DOF6_Source(Source1_Ptr src1, Source2_Ptr src2)
    :src1_(src1),src2_(src2)
    {
      ROS_ASSERT(src1_);
      ROS_ASSERT(src2_);
    }

    DOF6_Source()
    :src1_(new SOURCE1),src2_(new SOURCE2)
    {
    }

    void deepCopy(const DOF6_Source &o)
    {
        src1_->deepCopy(*o.src1_);
        src2_->deepCopy(*o.src2_);
    }

    Source1_Ptr getSource1() {return src1_;}
    Source2_Ptr getSource2() {return src2_;}

    TYPE getRotationVariance() const
    {
      const TYPE C1 = src1_->getRotationVariance();
      const TYPE C2 = src2_->getRotationVariance();
      return C1 - C1*C1/(C1+C2);
    }

    TYPE getTranslationVariance() const
    {
      const TYPE C1 = src1_->getTranslationVariance();
      const TYPE C2 = src2_->getTranslationVariance();
      return C1 - C1*C1/(C1+C2);
    }

    EulerAngles<TYPE> getRotation() const
        {
      const EulerAngles<TYPE> X1 = src1_->getRotation();
      const EulerAngles<TYPE> X2 = src2_->getRotation();
      const TYPE C1 = src1_->getRotationVariance();
      const TYPE C2 = src2_->getRotationVariance();
      return X1 + C1/(C1+C2) * (X2-X1);
        }

    Vector getTranslation() const
    {
      const Vector X1 = src1_->getTranslation();
      const Vector X2 = src2_->getTranslation();
      const TYPE C1 = src1_->getRotationVariance();
      const TYPE C2 = src2_->getRotationVariance();
      return X1 + C1/(C1+C2) * (X2-X1);
    }

    void reset() {
      src1_->reset();
      src2_->reset();
    }

    void setTime(const double time_in_sec)
    {
      src1_->setTime(time_in_sec);
      src2_->setTime(time_in_sec);
    }

  };

}



#endif /* DOF_VARIANCE_H_ */
