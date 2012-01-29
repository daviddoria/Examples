// Written by zerm (IRC: freenode : #opencv)
#include <cstdio>

#include <opencv/cv.h>
#include <opencv/highgui.h>

//const double HESSIAN = 300.;
//const double MAX_DIST = .22;
const double HESSIAN = 1000.;
const double MAX_DIST = .1;
const unsigned NUM_NEIGHBORS = 4;

int main(int argc, char* argv[]) {
  IplImage *im1 = cvLoadImage(argv[1],0);


  CvMemStorage  *store;
  CvSeq     *keypts;
  CvSeq     *descs;
  CvMat     *feats;
  CvFeatureTree *tree;


  store = cvCreateMemStorage(0);
  cvExtractSURF(im1, NULL, &keypts, &descs, store, cvSURFParams(HESSIAN, 1));
  printf("left: %u features\n", keypts->total);

  {
    float *f = new float[keypts->total * 128];
    for(int i=0; i<keypts->total; ++i) {
      memcpy(&f[i*128], cvGetSeqElem(descs,i), 128*sizeof(float));
    }
    feats = cvCreateMatHeader(keypts->total, 128, CV_32FC1);
    cvSetData(feats, f, 128*sizeof(float));
  }
  tree = cvCreateKDTree(feats);

  /** rhs **/
  {
    IplImage *im2 = cvLoadImage(argv[2],0);
    CvMemStorage  *store_r;
    CvSeq     *keypts_r;
    CvSeq     *descs_r;
    CvMat     *feats_r;

    store_r = cvCreateMemStorage(0);
    cvExtractSURF(im2, NULL, &keypts_r, &descs_r, store_r, cvSURFParams(HESSIAN, 1));
    printf("right: %u features\n", keypts->total);

    float *f = new float[keypts_r->total * 128];
    for(int i=0; i<keypts_r->total; ++i) {
      memcpy(&f[i*128], cvGetSeqElem(descs_r,i), 128*sizeof(float));
    }
    feats_r = cvCreateMatHeader(keypts_r->total, 128, CV_32FC1);
    cvSetData(feats_r, f, 128*sizeof(float));


    CvMat *matches;
    CvMat *dists;

    matches = cvCreateMat(keypts_r->total, NUM_NEIGHBORS, CV_32SC1);
    dists = cvCreateMat(keypts_r->total, NUM_NEIGHBORS, CV_64FC1);
    cvFindFeatures(tree, feats_r, matches, dists, NUM_NEIGHBORS, 10);



    IplImage *cmp = cvCreateImage(cvSize(im1->width+im2->width,im1->height+im2->height), IPL_DEPTH_8U, 1);
    cvZero(cmp);

    cvSetImageROI(cmp, cvRect(0,0,im1->width,im1->height));
    cvCopy(im1, cmp);
    cvResetImageROI(cmp);

    cvSetImageROI(cmp, cvRect(im1->width,im1->height,im2->width,im2->height));
    cvCopy(im2, cmp);
    cvResetImageROI(cmp);

    IplImage *cmpRGB = cvCreateImage(cvSize(im1->width+im2->width,im1->height+im2->height), IPL_DEPTH_8U, 3);
    cvCvtColor(cmp,cmpRGB,CV_GRAY2RGB);


    for(int i=0; i<keypts_r->total; ++i) {
      for(unsigned j=0; j<NUM_NEIGHBORS; ++j) {
        int idx = CV_MAT_ELEM(*matches, int, i, j);
        if(idx<0) {
          continue;
        }
        double dist = CV_MAT_ELEM(*dists,double,i,j);
        if(dist>MAX_DIST) {
          continue;
        }

        CvSURFPoint *lpt = (CvSURFPoint*) cvGetSeqElem(keypts, idx);
        CvSURFPoint *rpt = (CvSURFPoint*) cvGetSeqElem(keypts_r, i);

        cvLine(cmpRGB,
            cvPoint( lpt->pt.x, lpt->pt.y ),
            cvPoint( im1->width+rpt->pt.x, im1->height+rpt->pt.y ),
            CV_RGB(0,255,0),
            2);
      }
    }


    cvNamedWindow("out",0);
    cvShowImage("out",cmpRGB);
    cvWaitKey(0);
  }

  return 0;
}
