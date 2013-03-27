/*
 * haar.h
 *
 *  Created on: 26.03.2013
 *      Author: josh
 */

#ifndef HAAR_H_
#define HAAR_H_


namespace cob_3d_feature_map {

  template<typename Cluster, typename Type>
  size_t haar_wavelet(std::vector<Type> &prob_result, const std::vector<Cluster> &hotspots, const Cluster &input, const Type thr)
  {
    size_t num = 0, ind=0;
    for(size_t i=0; i<hotspots.size(); i++) {
      Type p = hotspots[i].cmp(input);

      prob_result[i] = p;

      if(prob_result[i]>=thr) {
        ++num;
        ind = i;
      }

    }

    size_t D_ind=0;
    while(num>1) {
      std::vector<Cluster::ClusterReprsentation> rep;
      input.split_all(D_ind, rep);

      std::vector<Type> p;
      for(size_t i=0; i<hotspots.size(); i++) {
        Type p = hotspots[i].cmp_split(rep, D_ind, p);

        prob_result[i] *= p;    //independent probabilities

        if(prob_result[i]>=thr) {
          ++num;
          ind = i;
        }
      }

      ++D_ind;
    }

    return ind;
  }

}


#endif /* HAAR_H_ */
