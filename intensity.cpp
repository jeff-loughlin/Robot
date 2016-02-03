#include <stdlib.h>
#include <stdio.h>
#include <jpeglib.h>

int *BMap;
int Height;
int Width;
int Depth;

int main(int argc, char **argv)
{
  const char *Name = argv[1];

  unsigned char r, g, b;
  int width;
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;

  FILE * infile;        /* source file */
  JSAMPARRAY pJpegBuffer;       /* Output row buffer */
  int row_stride;       /* physical row width in output buffer */
  if ((infile = fopen(Name, "rb")) == NULL)
  {
    fprintf(stderr, "can't open %s\n", Name);
    return 0;
  }
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);
  jpeg_stdio_src(&cinfo, infile);
  (void) jpeg_read_header(&cinfo, TRUE);
  (void) jpeg_start_decompress(&cinfo);
  width = cinfo.output_width;
//  height = cinfo.output_height;

//  unsigned char * pDummy = new unsigned char [width*height*4];
//  unsigned char * pTest = pDummy;
//  if (!pDummy)
//  {
//    printf("NO MEM FOR JPEG CONVERT!\n");
//    return 0;
//  }
  row_stride = width * cinfo.output_components;
  pJpegBuffer = (*cinfo.mem->alloc_sarray)
    ((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);

  int pixCount = 0;
  int total = 0;
  while (cinfo.output_scanline < cinfo.output_height)
  {
    (void) jpeg_read_scanlines(&cinfo, pJpegBuffer, 1);
    for (int x = 0; x < width; x++)
    {
//      a = 0; // alpha value is not supported on jpg
      r = pJpegBuffer[0][cinfo.output_components * x];
      if (cinfo.output_components > 2)
      {
        g = pJpegBuffer[0][cinfo.output_components * x + 1];
        b = pJpegBuffer[0][cinfo.output_components * x + 2];
      }
      else
      {
        g = r;
        b = r;
      }
//      *(pDummy++) = b;
//      *(pDummy++) = g;
//      *(pDummy++) = r;
//      *(pDummy++) = a;

      int intensity = (r + g + b) / 3;
      total += intensity;
      pixCount++;
    }
  }
  fclose(infile);
  (void) jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);

//  BMap = (int*)pTest;
//  Height = height;
  Width = width;
  Depth = 32;

//  free(pDummy);

  float avgIntensity = (float)total / (float)pixCount;

  printf("%2.2f\n", avgIntensity);

  return 0;
}

