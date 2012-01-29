#include <iostream>
#include <string>

#include "xtiffio.h"  /* for TIFF */
#include "geotiffio.h" /* for GeoTIFF */
enum {VERSION=0,MAJOR,MINOR};
int main(int argc, char*argv[])
{
  if(argc < 2)
  {
    std::cerr << "Required arguments: filename" << std::endl;
    return -1;
  }

  std::string filename = argv[1];
  
  TIFF *tif=(TIFF*)0;  /* TIFF-level descriptor */
  GTIF *gtif=(GTIF*)0; /* GeoKey-level descriptor */
  int versions[3];
  int cit_length;
  geocode_t model;    /* all key-codes are of this type */
  char *citation;

  /* Open TIFF descriptor to read GeoTIFF tags */
  tif=XTIFFOpen(filename.c_str(),"r");
  if (!tif) goto failure;

  /* Open GTIF Key parser; keys will be read at this time. */
  gtif = GTIFNew(tif);
  if (!gtif) goto failure;

  /* Get the GeoTIFF directory info */
  GTIFFDirectoryInfo(gtif,versions,0);
  if (versions[MAJOR] > 1)
  {
          printf("this file is too new for me\n"); goto failure;
  }
  if (!GTIFKeyGet(gtif, GTModelTypeGeoKey, &model, 0, 1))
  {
          printf("Yikes! no Model Type\n") goto failure;
  }

  /* ASCII keys are variable-length; compute size */
  cit_length = GTIFKeyInfo(gtif,GTCitationGeoKey,&size,&type);
  if (cit_length > 0)
  {
          citation = malloc(size*cit_length);
          if (!citation) goto failure;
          GTIFKeyGet(gtif, GTCitationGeoKey, citation, 0, cit_length);
          printf("Citation:%s\n",citation);
  }

  /* Get some TIFF info on this image */
  TIFFGetField(tif,TIFFTAG_IMAGEWIDTH,    &width);

  /* get rid of the key parser */
  GTIFFree(gtif);

  /* close the TIFF file descriptor */
  XTIFFClose(tif);

  exit (0);
failure:
  exit (-1);
}
