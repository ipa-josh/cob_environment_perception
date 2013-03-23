/*
 * test_2d_map.cpp
 *
 *  Created on: 23.03.2013
 *      Author: josh
 */


#include <gtest/gtest.h>

#include <cob_3d_feature_map/feature.h>
#include <cob_3d_feature_map/relation.h>
#include <cob_3d_feature_map/ft_map.h>

using namespace cob_3d_feature_map;

namespace Map2D {
  class Instance {
  public:
    Eigen::Vector2i pos_;
    Eigen::Matrix<float,2,1> ft_;
  };

  template<typename INST>
  class Context {
    std::vector<boost::shared_ptr<INST> > insts_;
    int activity_;
  public:

    void operator+=(boost::shared_ptr<INST> inst) {
      insts_.push_back(inst);
    }
  };

  template<typename INST>
  class BigContext {
    enum {WIDTH=100,HEIGHT=100,SIZE=100};
    Context<INST> ctxts_[WIDTH*HEIGHT];
  public:

    void operator+=(boost::shared_ptr<INST> inst) {
      int x = inst->pos(0)%SIZE;
      int y = inst->pos(1)%SIZE;

      assert(x>=0&&y>=0&&x<WIDTH&&y<HEIGHT);

      ctxts_[y*WIDTH+x] += inst;
    }
  };
}

TEST(Search, map2d)
{
  typedef RelationEnd<RelationDistribution<2>, Map2D::Context<Map2D::Instance> > Relation;
  typedef KDTree::KDTree<Relation::DIMENSION,Relation> RT_TREE;
  typedef FeatureInst<Map2D::Instance, RT_TREE> FeatureInst;
  typedef Tree_Feature<FeatureInst> FTMAP;

  FTMAP ft_map;
  Map2D::BigContext<Map2D::Instance> map;

  //generate data
}


int main(int argc, char **argv){
  //ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
