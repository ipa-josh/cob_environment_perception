/*
 * pgm_loader.h
 *
 *  Created on: 04.04.2013
 *      Author: josh
 */

#ifndef PGM_LOADER_H_
#define PGM_LOADER_H_

#include <string>

// This function loads a PGM image file
// returns pointer to image if successful, NULL on error
unsigned char* loadPGM(std::string filename, int& width, int& height);


#endif /* PGM_LOADER_H_ */
