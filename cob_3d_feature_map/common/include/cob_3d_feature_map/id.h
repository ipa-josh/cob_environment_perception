/*
 * id.h
 *
 *  Created on: 24.03.2013
 *      Author: josh
 */

#ifndef ID_H_
#define ID_H_

namespace cob_3d_feature_map {
  class IDHandler {
  public:
    typedef size_t ID;
  protected:
    ID id_;
    static ID _sid_;

    IDHandler():id_(_sid_++)
    {}
  };
}

#endif /* ID_H_ */
