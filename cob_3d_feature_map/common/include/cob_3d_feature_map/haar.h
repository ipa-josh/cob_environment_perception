/*
 * haar.h
 *
 *  Created on: 26.03.2013
 *      Author: josh
 */

#ifndef HAAR_H_
#define HAAR_H_


namespace cob_3d_feature_map {

  template<typename Cluster>
  typename Cluster::TYPE __cmp_split(boost::shared_ptr<Cluster> &cs, const std::vector<typename Cluster::ClusterReprsentation> &o, const size_t ind, const typename Cluster::ClusterReprsentation &r2) //-->p[0,1]
  {
    typename Cluster::TYPE p=0, w=0;
    for(int i=0; i<Cluster::INSTANCE::DESCRIPTOR::CODES; i++) {
      typename Cluster::ClusterReprsentation r = cs->split(ind, i);
      typename Cluster::TYPE t = std::sqrt( (r.getWeight()+o[i].getWeight())/2 );
      //p += t * r.cmp(o[i]);
      p += t * r.cmp2(o[i],cs->getRepresentation(),r2);
      w += t;
    }
    return p/w;
  }

  template<typename ClusterSummarizer>
  typename ClusterSummarizer::element_type::TYPE __cmp_split(ClusterSummarizer &cs, const std::vector<typename ClusterSummarizer::element_type::ClusterReprsentation> &o, const size_t ind, const typename ClusterSummarizer::element_type::ClusterReprsentation &r2) //-->p[0,1]
  {
    typedef typename ClusterSummarizer::element_type Cluster;
    typename Cluster::TYPE p=0, w=0;
    for(int i=0; i<Cluster::INSTANCE::DESCRIPTOR::CODES; i++) {
      typename Cluster::ClusterReprsentation r = cs->split(ind, i);
      typename Cluster::TYPE t = std::sqrt( (r.getWeight()+o[i].getWeight())/2 );
      //p += t * r.cmp(o[i]);
      p += t * r.cmp2(o[i],cs->getRepresentation(),r2);
      w += t;

      cs.update(r,o[i],t);
    }
    cs.finish(w);
    return p/w;
  }

  template<typename ClusterSummarizer, typename Type>
  size_t haar_wavelet(std::vector<Type> &prob_result, std::vector<ClusterSummarizer> &hotspots, const ClusterSummarizer &input, const Type thr, const Type thr_term=0.8)
  {
    typedef typename ClusterSummarizer::element_type Cluster;
#ifdef CMP2
    std::vector<typename Cluster::ClusterReprsentation> dep_rep;
    typename Cluster::ClusterReprsentation dep_rep_cmp = input->getRepresentation();
#endif
    prob_result.resize(hotspots.size());

    size_t num = 0, ind=0;
    for(size_t i=0; i<hotspots.size(); i++) {
      Type p = hotspots[i]->cmp(*input);
      if(p!=p) p=0;

#ifdef CMP2
      dep_rep.push_back( hotspots[i]->getRepresentation() );
#endif

      prob_result[i] = p;
      std::cout<<"p("<<i<<") "<<p<<std::endl;

      if(prob_result[i]>=thr) {
        ++num;
        ind = i;
      }

    }

    size_t D_ind=0;
    while(num>1) {
      std::cout<<"type "<<D_ind%4<<std::endl;
      std::cout<<"found: "<<num<<std::endl;

      std::vector<typename Cluster::ClusterReprsentation> rep;
      if(input->split_all(D_ind, rep)<thr_term) break;

#ifdef CMP2
      {
        typename Cluster::ClusterReprsentation tmp = dep_rep_cmp;
        dep_rep_cmp = typename Cluster::ClusterReprsentation();
        for(size_t k=0; k<rep.size(); k++) {
          dep_rep_cmp += tmp*rep[k];
        }
      }
#endif

      {
        //debug svg
        char fn[512];
        sprintf(fn,"/tmp/haarCMP_%d.svg",(int)D_ind);
        FILE *fp=fopen(fn,"w");
        fputs("<?xml version=\"1.0\" ?><svg width=\"200\" height=\"200\">",fp);
        for(size_t ji=0; ji<input->getInstances().size(); ji++)
        {
          sprintf(fn,"<circle cx=\"%f\" cy=\"%f\" r=\"3\" fill=\"rgb(%d,%d,0)\" /><text x=\"%f\" y=\"%f\">%f</text>",
                  input->getInstances()[ji]->getRepresentation().getMean()(0)*20 + 344/100.f,
                  input->getInstances()[ji]->getRepresentation().getMean()(1)*20 + 344/100.f,
                  (int)(input->getInstances()[ji]->getAccDescr().weight(D_ind,0)*255),(int)(input->getInstances()[ji]->getAccDescr().weight(D_ind,0)*255),
                  input->getInstances()[ji]->getRepresentation().getMean()(0)*20 + 344/100.f,
                  input->getInstances()[ji]->getRepresentation().getMean()(1)*20 + 344/100.f,
                  (*input->getInstances()[ji]->getFeatures()[0])[D_ind%2]);
          fputs(fn,fp);
        }
        fputs("</svg>",fp);
        fclose(fp);
      }

      num=0;
      ind = (size_t)-1;

      for(size_t i=0; i<hotspots.size(); i++) {
        if(prob_result[i]<thr) continue;

        if(i==0||i==78) {
          //debug svg
          char fn[512];
          sprintf(fn,"/tmp/haar%d_%d.svg",(int)i,(int)D_ind);
          FILE *fp=fopen(fn,"w");
          fputs("<?xml version=\"1.0\" ?><svg width=\"200\" height=\"200\">",fp);
          for(size_t ji=0; ji<hotspots[i]->getInstances().size(); ji++)
          {
            sprintf(fn,"<circle cx=\"%f\" cy=\"%f\" r=\"3\" fill=\"rgb(%d,%d,0)\" /><text x=\"%f\" y=\"%f\">%f</text>",
                    hotspots[i]->getInstances()[ji]->getRepresentation().getMean()(0)*20 + 344/100.f,
                    hotspots[i]->getInstances()[ji]->getRepresentation().getMean()(1)*20 + 344/100.f,
                    (int)(hotspots[i]->getInstances()[ji]->getAccDescr().weight(D_ind,0)*255),(int)(hotspots[i]->getInstances()[ji]->getAccDescr().weight(D_ind,0)*255),
                    hotspots[i]->getInstances()[ji]->getRepresentation().getMean()(0)*20 + 344/100.f,
                    hotspots[i]->getInstances()[ji]->getRepresentation().getMean()(1)*20 + 344/100.f,
                    (*hotspots[i]->getInstances()[ji]->getFeatures()[0])[D_ind%2]);
            fputs(fn,fp);
          }
          fputs("</svg>",fp);
          fclose(fp);
        }

        Type p = __cmp_split(hotspots[i], rep, D_ind, input->getRepresentation());
        if(p!=p) p=0;

#ifdef CMP2
        {
          std::vector<typename Cluster::ClusterReprsentation> rep;
          hotspots[i]->split_all(D_ind, rep);

          {
            typename Cluster::ClusterReprsentation tmp = dep_rep[i];
            dep_rep[i] = typename Cluster::ClusterReprsentation();
            for(size_t k=0; k<rep.size(); k++) {
              dep_rep[i] += tmp*rep[k];
            }
          }
        }

        std::cout<<"P("<<i<<") "<<dep_rep[i].cmp(dep_rep_cmp)<<std::endl;
#endif

        std::cout<<"p("<<i<<") "<<p<<std::endl;
        std::cout<<"P("<<i<<") "<<hotspots[i].getP()<<std::endl;
        prob_result[i] *= p;    //independent probabilities

        if(prob_result[i]>=thr) {
          ++num;
          if((int)ind==-1 || prob_result[i]>prob_result[ind])
            ind = i;
        }
      }

      for(size_t i=0; i<prob_result.size(); i++)
        if(prob_result[i]>=thr) std::cout<<"R: "<<prob_result[i]<<std::endl;

      ++D_ind;
    }

    std::cout<<"iterations "<<D_ind+1<<std::endl;

    return ind;
  }

}


#endif /* HAAR_H_ */
