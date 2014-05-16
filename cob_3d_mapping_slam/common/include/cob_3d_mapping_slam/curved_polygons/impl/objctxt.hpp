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


#include <eigen3/Eigen/Dense>

template<typename _DOF6>
bool OBJCTXT<_DOF6>::registration(const OBJCTXT &ctxt, DOF6 &tf, typename DOF6::TYPE &probability_success_rate, typename DOF6::TYPE &probability_error_rate){

#ifdef DEBUG_
  ROS_INFO("registration %d %d",(int)objs_.size(), (int)ctxt.objs_.size());
  Debug::Interface::get().sayTook("reg. start");
  ROS_INFO("nextPoint ctr %d", Slam_Surface::___CTR___);
#endif

  tf.getSource1()->reset();

  //1. correspondences
  used_cors_.clear();
  //  findCorrespondences1(ctxt, used_cors_, tf);
  findCorrespondences3(ctxt, used_cors_, tf);

  Debug::Interface::get().sayTook("reg. cor");
  ROS_INFO("nextPoint ctr %d", Slam_Surface::___CTR___);

  //2. optimize
  //tf = optimizeLink(tf, cors);
#if 0
  if(tf.getRotationVariance()+tf.getTranslationVariance()<0.3f)
    tf = optimizeLink1(tf, used_cors_, tf.getRotationVariance(), tf.getTranslationVariance(), tf.getRotation(), tf.getTranslation());
  else
    tf = optimizeLink2(tf, used_cors_, tf.getRotationVariance(), tf.getTranslationVariance(), tf.getRotation(), tf.getTranslation());
#else
  tf = optimizeLink3(ctxt, used_cors_, tf, tf.getRotationVariance(), tf.getTranslationVariance(), tf.getRotation(), tf.getTranslation());
#endif

  Debug::Interface::get().sayTook("reg. opt");
  ROS_INFO("nextPoint ctr %d", Slam_Surface::___CTR___);

  std::cout<<"INPUT TF\n"<<tf<<"\n";

  //set result to time
  // if one registration failed
  // it can be retrieved over time
  tf.setVariance(  tf.getTranslationVariance(), tf.getTranslation(), tf.getRotationVariance(), tf.getRotation() );

  std::cout<<tf<<"\n";

  ROS_INFO("nextPoint ctr %d", Slam_Surface::___CTR___);

  if((tf.getSource1()->getTranslationVariance()+tf.getSource1()->getRotationVariance())>0.3)
  {
    tf.getSource1()->reset();
    ROS_INFO("failed to register");
    return false;
  }

  return true;
}


template<typename _DOF6>
bool OBJCTXT<_DOF6>::merge(const OBJCTXT &ctxt, const DOF6 &tf, std::map<typename OBJECT::Ptr,bool> &used_out, const BoundingBox::TransformedFoVBB &fov, const bool only_merge)
{
  Debug::Interface::get().sayTook("merge start");
  ROS_INFO("nextPoint ctr %d", (int)Slam_Surface::___CTR___);

  ++frames_;

  const size_t old = objs_.size();
  const float uncertainity = 0.01f;

  if(old>0 && (tf.getTranslationVariance()+tf.getRotationVariance())>0.3f )// && (tf.getSource1()->getTranslationVariance()+tf.getSource1()->getRotationVariance())>0.3 )
    return false;

  ROS_INFO("add ctxt %d", (int)used_out.size());
  std::cout<<tf<<"\n";

  Eigen::Vector3f edges[8];
  for(size_t i=0; i<objs_.size(); i++)
  {
    bool in=fov&objs_[i]->getNearestPoint();
    if(!in) {
      objs_[i]->getBB().get8Edges(edges);
      for(size_t i=0; i<8; i++)
        if(fov&edges[i]) {in=true; break;}
    }

    if(objs_[i]->getBB().extension()>2.5f)
      in=false;

    if( in ) {
#ifdef DEBUG_OUTPUT_
      ROS_INFO("FoV: in");
#endif
      objs_[i]->processed(); //TODO: check FoV
    }
    else {
#ifdef DEBUG_OUTPUT_
      ROS_INFO("FoV: out");
#endif
      objs_[i]->notProcessed(); //TODO: check FoV
    }
  }

  Debug::Interface::get().sayTook("merge 1");
  ROS_INFO("nextPoint ctr %d", (int)Slam_Surface::___CTR___);

  std::map<typename OBJECT::Ptr,std::vector<typename OBJECT::Ptr> > used;

  DOF6 tmp_link = tf.transpose();

  for(typename std::vector<SCOR>::const_iterator it = used_cors_.begin(); it!=used_cors_.end(); it++)
  {
    if(!it->a || !it->b || !it->used_) continue;

    //used_out[it->b] = true;

    typename OBJECT::Ptr o=it->b->makeShared();
    o->transform(tmp_link.getRotation(), tmp_link.getTranslation(),
                 tf.getRotationVariance()+uncertainity, tf.getTranslationVariance()+uncertainity);

    it->a->used();

    if(((*it->a) += *o)
    )
    {
      used_out[it->b] = true;

      used[it->b].push_back(it->a);
      ROS_INFO("update object");
    }
    else {
      //it->a->used();
      ROS_INFO("NOT update object");
    }
  }

  Debug::Interface::get().sayTook("merge 2");
  ROS_INFO("nextPoint ctr %d", (int)Slam_Surface::___CTR___);

  for(size_t i=0; i<ctxt.objs_.size(); i++)
  {
    typename std::map<typename OBJECT::Ptr,std::vector<typename OBJECT::Ptr> >::const_iterator it = used.find(ctxt.objs_[i]);

    if(used.end() == it)
    {
      if(used_out.find(ctxt.objs_[i])!=used_out.end())
        continue;

      if(!only_merge || (bb_.transform(tf.getRotation(), tf.getTranslation())&ctxt.objs_[i]->getNearestPoint()) ) //add to "first"/actual node or if its contained in this node
      {

        typename OBJECT::Ptr o=ctxt.objs_[i]->makeShared();
        o->transform(((Eigen::Matrix3f)tmp_link.getRotation()),tmp_link.getTranslation(),
                     tmp_link.getRotationVariance()+uncertainity, tmp_link.getTranslationVariance()+uncertainity);

        bool found = false;
        for(size_t j=0; j<objs_.size(); j++)
        {
          if( (objs_[j]->getBB()&(o->getBB()//.changeSize(0.8f)
          )) &&
          (((*objs_[j]) += *o) //&& objs_[j]->getData().isPlane()==o->getData().isPlane()
          ) )
          {
            ROS_INFO("found object");
            found = true;
            used_out[ctxt.objs_[i]] = true;
            used[ctxt.objs_[i]].push_back(objs_[j]);
          }
        }

        if(!found)
        {
          ROS_INFO("adding object");
          *this += o;
          used_out[ctxt.objs_[i]] = true;
        }
        else
          ROS_INFO("NOT adding object");
      }
    }

    it=used.find(ctxt.objs_[i]);
    if(used.end() != it && it->second.size()>1)
    {
      ROS_INFO("merging objects %d", (int)it->second.size());

      for(size_t k=1; k<it->second.size(); k++)
      {
        if(*(it->second)[0] += *(it->second)[k])
        {
          for(size_t j=0; j<objs_.size(); j++)
            if(objs_[j] == (it->second)[k] //&& objs_[j]->getData().isPlane()==(it->second)[k]->getData().isPlane()
            )
            {
              objs_.erase(objs_.begin() + j);
              --j;
              ROS_INFO("erase object");
            }
        }
      }
    }
  }
  Debug::Interface::get().sayTook("merge up2");

  if(enabled_all_merge_) {
    for(size_t i=0; i<objs_.size(); i++)
      for(size_t j=i+1; j<objs_.size(); j++)
      {
        int n=-1;
        bool b1, b2, b3;
        if( (b1=(objs_[i]->getBB().changeSize(0.7f)&(objs_[j]->getBB().setMinSize(0.05f).changeSize(0.7f))
        )) &&
        (b2=(objs_[j]->getBB().preassumption(objs_[i]->getData().getBB())>=std::cos(objs_[i]->getData().getBB().ratio()))) &&
        (((*objs_[i]).op_plus(*objs_[j],n)) //&& objs_[j]->getData().isPlane()==o->getData().isPlane()
        ) )
        {
          objs_.erase(objs_.begin() + j);
          --j;
          ROS_INFO("erase objectA23");
        }
        else if(n!=-1) {
          switch(n) {
            case 0:Debug::Interface::get().addArrow(objs_[i]->getNearestPoint(),objs_[j]->getNearestPoint(),255,0,0);break;
            case 1:Debug::Interface::get().addArrow(objs_[i]->getNearestPoint(),objs_[j]->getNearestPoint(),0,0,0);break;
            case 2:Debug::Interface::get().addArrow(objs_[i]->getNearestPoint(),objs_[j]->getNearestPoint(),0,0,255);break;
            case -1:
              if(!b1)
                Debug::Interface::get().addArrow(objs_[i]->getNearestPoint(),objs_[j]->getNearestPoint(),128,0,0);
              else if(!b2)
                Debug::Interface::get().addArrow(objs_[i]->getNearestPoint(),objs_[j]->getNearestPoint(),128,44,255);
              break;
          }
        }
      }
  }

  //  for(typename std::vector<SCOR>::const_iterator it = used_cors_.begin(); it!=used_cors_.end(); it++)
  //  {
  //    used_out[it->b] = true;
  //  }

  Debug::Interface::get().sayTook("merge up");
  ROS_INFO("nextPoint ctr %d", (int)Slam_Surface::___CTR___);

  update();

  Debug::Interface::get().sayTook("merge stop");
  ROS_INFO("nextPoint ctr %d", (int)Slam_Surface::___CTR___);
  Slam_Surface::___CTR___=0;

  return true;
}


template<typename _DOF6>
void OBJCTXT<_DOF6>::update()
{
  filter();
  updateBB();
}

template<typename _DOF6>
void OBJCTXT<_DOF6>::filter()
{
  for(size_t i=0; i<objs_.size(); i++)
  {
    size_t c = objs_[i]->getCreationCounter(), u = objs_[i]->getUsedCounter();
    const bool inv = objs_[i]->invalid();

    if((objs_[i]->getProcessed() && (c>5 && std::log(u)/std::log(c) < 0.6f)) ||
        inv)
    {
      objs_.erase(objs_.begin()+i);
      --i;
      ROS_INFO(inv?"removed object because INVALID %d %d":"removed object %d %d", c,u);
    }
  }
}

template<typename _DOF6>
void OBJCTXT<_DOF6>::updateBB()
{
  bb_.update(*this);
}

template<typename _DOF6>
typename OBJCTXT<_DOF6>::Ptr OBJCTXT<_DOF6>::clone() const
{
  OBJCTXT::Ptr r(new OBJCTXT(*this));
  for(size_t i=0; i<objs_.size(); i++)
    r->objs_[i].reset( new OBJECT(*objs_[i]) );
  return r;
}

template<typename _DOF6>
OBJCTXT<_DOF6> &OBJCTXT<_DOF6>::transform(const DOF6 &tf)
{
  for(size_t i=0; i<objs_.size(); i++)
    objs_[i]->transform(tf.getRotation(), tf.getTranslation(), tf.getRotationVariance(), tf.getTranslationVariance());
  updateBB();
  return *this;
}

