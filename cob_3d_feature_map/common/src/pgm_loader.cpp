#include <cob_3d_feature_map/tools/pgm_loader.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// This function loads a PGM image file
// returns pointer to image if successful, NULL on error

unsigned char* loadPGM(std::string filename, int& width, int& height)
{
  const int BUFSIZE = 128;
  FILE* fp;
  unsigned int read;
  unsigned char* rawData;
  char buf[3][BUFSIZE];
  char* retval_fgets;
  size_t retval_sscanf;

  if ( (fp=fopen(filename.c_str(), "rb")) == NULL)
  {
    return NULL;
  }

  // Read magic number:
  retval_fgets=fgets(buf[0], BUFSIZE, fp);
  if (retval_fgets==NULL)
  {
    fclose(fp);
    return NULL;
  }
  if (strncmp("P5", buf[0], 2) != 0)
  {
    fclose(fp);
    return NULL;
  }

  // Read width and height:
  do
  {
    retval_fgets=fgets(buf[0], BUFSIZE, fp);
    if (retval_fgets==NULL)
    {
      fclose(fp);
      return NULL;
    }
  } while (buf[0][0] == '#');
  retval_sscanf=sscanf(buf[0], "%s %s", buf[1], buf[2]);
  if (retval_sscanf!=2)
  {
    fclose(fp);
    return NULL;
  }
  width  = atoi(buf[1]);
  height = atoi(buf[2]);

  // Read maxval:
  do
  {
    retval_fgets=fgets(buf[0], BUFSIZE, fp);
    if (retval_fgets==NULL)
    {
      fclose(fp);
      return NULL;
    }
  } while (buf[0][0] == '#');

  // Read image data:
  rawData = new unsigned char[width * height];
  read = fread(rawData, width * height, 1, fp);
  if (read != 1)
  {
    fclose(fp);
    delete[] rawData;
    return NULL;
  }

  fclose(fp);
  return rawData;
}
