/*
 * map_2d.cpp
 *
 *  Created on: 04.04.2013
 *      Author: josh
 */


#include <ros/ros.h>

#include <cob_3d_feature_map/simplified.h>
#include <cob_3d_feature_map/tools/pgm_loader.h>

void simulate_scan(const float a1, float a2, const int x, const int y, const unsigned char * const pgm, const int w, const int h, std::vector<float> &scan, std::vector<Eigen::Vector2i> &scan2) {
  if(a2<a1) a2+=2*M_PI;

  int i=0;
  for(float a=a1; a<a2; a+=(a2-a1)/scan.size() )
  {
    float dx = std::cos(a);
    float dy = std::sin(a);

    Eigen::Vector2i s2 = Eigen::Vector2i::Zero();
    float v = std::numeric_limits<float>::max();
    if(std::abs(dx)<std::abs(dy)) {
      dx/=std::abs(dy);
      int p=0;
      while(p+y>=0 && p+y<h) {
        int px = x + p*dx;
        if(px<0 || px>=w) break;
        if(pgm[px+(p+y)*w]<128) {
          v = std::sqrt(p*p*(1+dx*dx));
          s2(0) = px-x;
          s2(1) = p;
          break;
        }
        p++;
      }
    }
    else {
      dy/=std::abs(dx);
      int p=0;
      while(p+x>=0 && p+x<w) {
        int py = y + p*dy;
        if(py<0 || py>=h) break;
        if(pgm[p+x+(py)*w]<128) {
          v = std::sqrt(p*p*(1+dy*dy));
          s2(0) = p;
          s2(1) = py-y;
          break;
        }
        p++;
      }
    }

    if(v>100) v = std::numeric_limits<float>::quiet_NaN();
    scan[i] = v;
    scan2[i]= s2;
    ++i;
  }
}

template<typename FT>
void generate_ft(const std::vector<float> &scan, const std::vector<Eigen::Vector2i> &scan2, std::vector<FT> &fts)
{

  for(size_t i=0; i<scan.size(); i++)
  {
    Eigen::Vector4f sum_v = Eigen::Vector4f::Zero();
    Eigen::Matrix4f sum_M = Eigen::Matrix4f::Zero();

    int n=0;
    for(int d=-10; d<=10; d++) {
      int ind = (int)i+d;
      if(ind<0) ind+=(int)scan.size();
      ind%=scan.size();

      if(scan[ind]!=scan[ind])// || std::abs(scan[ind]-scan[i])>5)
        continue;

      Eigen::Vector4f v;
      v(0)=1;
      v(1)=d;
      v(2)=d*d;
      v(2)=d*d*d;
      Eigen::Matrix4f M = v*v.transpose();
      v *= scan[ind];

      sum_v += v;
      sum_M += M;
      ++n;
    }

    sum_v = sum_M.inverse()*sum_v;
    /*sum_v(0) = std::min(10.f, std::max(-10.f,sum_v(0)/10));
    sum_v(1) = std::min(10.f, std::max(-10.f,std::atan(sum_v(1))));
    sum_v(2) = std::min(10.f, std::max(-10.f,std::atan(sum_v(2))));*/
    fts[i].getContent()(0) = scan2[i](0)/7;sum_v(2);
    fts[i].getContent()(1) = scan2[i](1)/7;sum_v(3);
  }
}

int main(int argc, char **argv) {
  if(argc<2) {
    ROS_ERROR("specify map");
    return 1;
  }

  typedef cob_3d_feature_map::SimplifiedInterface_Eigen<2,2> SI;
  typedef SI::FT FT;
  SI::treeType tree;
  std::vector<boost::shared_ptr<SI::CL> > clusters;

  ROS_INFO("reading map...");
  //load map
  int w,h;
  unsigned char *pgm = loadPGM(argv[1],w,h);

  //simulate laser scan at each point
  const int step = 20;
  ROS_INFO("generating feature map (%d x %d) ...",w/step,h/step);
  for(int x=0; x<w; x+=step)
    for(int y=0; y<h; y+=step)
    {
      for(int p=0; p<4; p++) {
        std::vector<float> scan(100);
        std::vector<Eigen::Vector2i> scan2(100);
        std::vector<SI::FT> fts(100);
        simulate_scan( (p/8.*2*M_PI)-M_PI/4, (p/8.*2*M_PI)+M_PI/4, x,y, pgm,w,h, scan, scan2);
        generate_ft(scan, scan2, fts);

        boost::shared_ptr<SI::CL> pcl(new SI::CL);
        clusters.push_back(pcl);
        for(size_t i=0; i<fts.size(); i++) {
          if(fts[i].getContent().sum()!=fts[i].getContent().sum()) continue;

          tree.insert(fts[i]);
          Eigen::Vector2f pt2;
          pt2(0) = scan2[i](0);
          pt2(1) = scan2[i](1);
          SI::CR pt;
          pt+=pt2;
          boost::shared_ptr<SI::INST> inst(new SI::INST(pt));
          (*clusters.back()) += inst;

          boost::shared_ptr<FT> ft(new FT(fts[i]));
          clusters.back()->getInstances().back()->append(ft);
        }
      }
      std::cerr<<".";
    }
  std::cerr<<"\n";

  //build descriptor
  ROS_INFO("build descriptor...");
  for(size_t c=0; c<clusters.size(); c++) {
    for(size_t i=0; i<clusters[c]->getInstances().size(); i++)
    {
      // now do the same with the kdtree.
      for(size_t j=0; j<clusters[c]->getInstances()[i]->getFeatures().size(); j++) {
        std::vector< std::vector<float> > descr;
        ROS_ASSERT(tree.end()!=tree.find_exact( *clusters[c]->getInstances()[i]->getFeatures()[j]));
        tree.find_exact_ex( *clusters[c]->getInstances()[i]->getFeatures()[j],back_insert_iterator2<std::vector<std::vector<float> > >(descr));
        ROS_ASSERT(descr.size()>0);
        clusters[c]->getInstances()[i]->getFeatures()[j]->setDescriptorFromVector(descr[0]);
        clusters[c]->getInstances()[i]->updateDescriptor(
            clusters[c]->getInstances()[i]->getFeatures()[j]
        );
      }
    }
    std::cerr<<".";
  }
  std::cerr<<"\n";

  //generate random search pos
  int sx = rand()%w;
  int sy = rand()%h;
  float alpha = (rand()%360)*M_PI/180;
  ROS_INFO("random pose (%d,%d,%f)",sx,sy,alpha);

  boost::shared_ptr<SI::CL> scl(new SI::CL);
  {
    std::vector<float> scan(100);
    std::vector<Eigen::Vector2i> scan2(100);
    simulate_scan( alpha-M_PI/4, alpha+M_PI/4, sx,sy, pgm,w,h, scan, scan2);
    std::vector<SI::FT> fts(100);
    generate_ft(scan, scan2, fts);

    for(size_t i=0; i<fts.size(); i++) {
      if(fts[i].getContent().sum()!=fts[i].getContent().sum()) continue;

      tree.insert(fts[i]);
      Eigen::Vector2f pt2;
      pt2(0) = scan2[i](0);
      pt2(1) = scan2[i](1);
      SI::CR pt;
      pt+=pt2;
      boost::shared_ptr<SI::INST> inst(new SI::INST(pt));
      (*scl) += inst;

      boost::shared_ptr<FT> ft(new FT(fts[i]));
      scl->getInstances().back()->append(ft);
    }
    for(size_t i=0; i<scl->getInstances().size(); i++)
    {
      // now do the same with the kdtree.
      for(size_t j=0; j<scl->getInstances()[i]->getFeatures().size(); j++) {
        std::vector< std::vector<float> > descr;
        tree.find_exact_ex( *tree.find_nearest(*scl->getInstances()[i]->getFeatures()[j]).first,back_insert_iterator2<std::vector<std::vector<float> > >(descr));
        scl->getInstances()[i]->getFeatures()[j]->setDescriptorFromVector(descr[0]);
        scl->getInstances()[i]->updateDescriptor(
            scl->getInstances()[i]->getFeatures()[j]
        );
      }
    }
  }

  //debug svg
  for(size_t c=0; c<clusters.size(); c++) {
    char fn[512];
    sprintf(fn,"/tmp/cluster%d.svg",(int)c);
    FILE *fp=fopen(fn,"w");
    fputs("<?xml version=\"1.0\" ?><svg width=\"200\" height=\"200\">",fp);
    for(size_t i=0; i<clusters[c]->getInstances().size(); i++)
    {
      sprintf(fn,"<circle cx=\"%f\" cy=\"%f\" r=\"3\" />",
              clusters[c]->getInstances()[i]->getRepresentation().getMean()(0)*2.f+10,
              clusters[c]->getInstances()[i]->getRepresentation().getMean()(1)*2.f+10);
      fputs(fn,fp);
    }
    fputs("</svg>",fp);
    fclose(fp);
  }
  {
    char fn[512];
    sprintf(fn,"/tmp/search.svg");
    FILE *fp=fopen(fn,"w");
    fputs("<?xml version=\"1.0\" ?><svg width=\"200\" height=\"200\">",fp);
    for(size_t i=0; i<scl->getInstances().size(); i++)
    {
      sprintf(fn,"<circle cx=\"%f\" cy=\"%f\" r=\"3\" />",
              scl->getInstances()[i]->getRepresentation().getMean()(0)*2.f+10,
              scl->getInstances()[i]->getRepresentation().getMean()(1)*2.f+10);
      fputs(fn,fp);
    }
    fputs("</svg>",fp);
    fclose(fp);
  }

  SI::COR_SET search_cluster(scl);
  std::vector<SI::COR_SET> sets;
  std::vector<float> result;
  for(std::vector<boost::shared_ptr<SI::CL> >::const_iterator it = clusters.begin()+0; it!=clusters.end(); ++it)
    sets.push_back( SI::COR_SET(*it, search_cluster.getPtr()) );
  size_t p = cob_3d_feature_map::haar_wavelet(result, sets, search_cluster, 0.2f);

  for(size_t i=0; i<result.size(); i++)
    std::cout<<"R: "<<result[i]<<std::endl;
  std::cout<<"best match: "<<p<<std::endl;
  std::cout<<"gt: "<<sx/step<<" "<<sy/step<<"  "<<8*(sy/step + (sx/step)*(h/step))<<std::endl;

  delete [] pgm;
  return 0;
}

