/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/
#include<cassert> //for assert()
#include "matcher.h"
#include "triangle.h"
#include "filter.h"
#include <fstream> //for ofstream test
using namespace std;

//////////////////////
// PUBLIC FUNCTIONS //
//////////////////////
namespace libviso2{
void makeOffsets(int pixel[25], int rowStride, int patternSize)
{
    static const int offsets16[][2] =
    {
        {0,  3}, { 1,  3}, { 2,  2}, { 3,  1}, { 3, 0}, { 3, -1}, { 2, -2}, { 1, -3},
        {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3,  1}, {-2,  2}, {-1,  3}
    };

    static const int offsets12[][2] =
    {
        {0,  2}, { 1,  2}, { 2,  1}, { 2, 0}, { 2, -1}, { 1, -2},
        {0, -2}, {-1, -2}, {-2, -1}, {-2, 0}, {-2,  1}, {-1,  2}
    };

    static const int offsets8[][2] =
    {
        {0,  1}, { 1,  1}, { 1, 0}, { 1, -1},
        {0, -1}, {-1, -1}, {-1, 0}, {-1,  1}
    };

    const int (*offsets)[2] = patternSize == 16 ? offsets16 :
                              patternSize == 12 ? offsets12 :
                              patternSize == 8  ? offsets8  : 0;

    assert(pixel && offsets);

    int k = 0;
    for( ; k < patternSize; k++ )
        pixel[k] = offsets[k][0] + offsets[k][1] * rowStride;
    for( ; k < 25; k++ )
        pixel[k] = pixel[k - patternSize];
}

#if VERIFY_CORNERS
static void testCorner(const uchar* ptr, const int pixel[], int K, int N, int threshold) {
    // check that with the computed "threshold" the pixel is still a corner
    // and that with the increased-by-1 "threshold" the pixel is not a corner anymore
    for( int delta = 0; delta <= 1; delta++ )
    {
        int v0 = std::min(ptr[0] + threshold + delta, 255);
        int v1 = std::max(ptr[0] - threshold - delta, 0);
        int c0 = 0, c1 = 0;

        for( int k = 0; k < N; k++ )
        {
            int x = ptr[pixel[k]];
            if(x > v0)
            {
                if( ++c0 > K )
                    break;
                c1 = 0;
            }
            else if( x < v1 )
            {
                if( ++c1 > K )
                    break;
                c0 = 0;
            }
            else
            {
                c0 = c1 = 0;
            }
        }
        assert( (delta == 0 && std::max(c0, c1) > K) ||
                   (delta == 1 && std::max(c0, c1) <= K) );
    }
}
#endif

template<>
int cornerScore<16>(const unsigned char* ptr, const int pixel[], int threshold)
{
    const int K = 8, N = K*3 + 1;
    int k, v = ptr[0];
    short d[N];
    for( k = 0; k < N; k++ )
        d[k] = (short)(v - ptr[pixel[k]]);

#if CV_SSE2
    __m128i q0 = _mm_set1_epi16(-1000), q1 = _mm_set1_epi16(1000);
    for( k = 0; k < 16; k += 8 )
    {
        __m128i v0 = _mm_loadu_si128((__m128i*)(d+k+1));
        __m128i v1 = _mm_loadu_si128((__m128i*)(d+k+2));
        __m128i a = _mm_min_epi16(v0, v1);
        __m128i b = _mm_max_epi16(v0, v1);
        v0 = _mm_loadu_si128((__m128i*)(d+k+3));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k+4));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k+5));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k+6));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k+7));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k+8));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k));
        q0 = _mm_max_epi16(q0, _mm_min_epi16(a, v0));
        q1 = _mm_min_epi16(q1, _mm_max_epi16(b, v0));
        v0 = _mm_loadu_si128((__m128i*)(d+k+9));
        q0 = _mm_max_epi16(q0, _mm_min_epi16(a, v0));
        q1 = _mm_min_epi16(q1, _mm_max_epi16(b, v0));
    }
    q0 = _mm_max_epi16(q0, _mm_sub_epi16(_mm_setzero_si128(), q1));
    q0 = _mm_max_epi16(q0, _mm_unpackhi_epi64(q0, q0));
    q0 = _mm_max_epi16(q0, _mm_srli_si128(q0, 4));
    q0 = _mm_max_epi16(q0, _mm_srli_si128(q0, 2));
    threshold = (short)_mm_cvtsi128_si32(q0) - 1;
#else
    int a0 = threshold;
    for( k = 0; k < 16; k += 2 )
    {
        int a = std::min((int)d[k+1], (int)d[k+2]);
        a = std::min(a, (int)d[k+3]);
        if( a <= a0 )
            continue;
        a = std::min(a, (int)d[k+4]);
        a = std::min(a, (int)d[k+5]);
        a = std::min(a, (int)d[k+6]);
        a = std::min(a, (int)d[k+7]);
        a = std::min(a, (int)d[k+8]);
        a0 = std::max(a0, std::min(a, (int)d[k]));
        a0 = std::max(a0, std::min(a, (int)d[k+9]));
    }

    int b0 = -a0;
    for( k = 0; k < 16; k += 2 )
    {
        int b = std::max((int)d[k+1], (int)d[k+2]);
        b = std::max(b, (int)d[k+3]);
        b = std::max(b, (int)d[k+4]);
        b = std::max(b, (int)d[k+5]);
        if( b >= b0 )
            continue;
        b = std::max(b, (int)d[k+6]);
        b = std::max(b, (int)d[k+7]);
        b = std::max(b, (int)d[k+8]);

        b0 = std::min(b0, std::max(b, (int)d[k]));
        b0 = std::min(b0, std::max(b, (int)d[k+9]));
    }

    threshold = -b0-1;
#endif

#if VERIFY_CORNERS
    testCorner(ptr, pixel, K, N, threshold);
#endif
    return threshold;
}


// constructor (with default parameters)
Matcher::Matcher(parameters param) : param(param) {

  // init match ring buffer to zero
  m1p1 = 0; n1p1 = 0;
  m1p2 = 0; n1p2 = 0;
  m2p1 = 0; n2p1 = 0;
  m2p2 = 0; n2p2 = 0;
  m1c1 = 0; n1c1 = 0;
  m1c2 = 0; n1c2 = 0;
  m2c1 = 0; n2c1 = 0;
  m2c2 = 0; n2c2 = 0;
  I1p    = 0; I2p    = 0;
  I1c    = 0; I2c    = 0;
  I1p_du = 0; I1p_dv = 0;
  I2p_du = 0; I2p_dv = 0;
  I1c_du = 0; I1c_dv = 0;
  I2c_du = 0; I2c_dv = 0;
  I1p_du_full = 0; I1p_dv_full = 0;
  I2p_du_full = 0; I2p_dv_full = 0;
  I1c_du_full = 0; I1c_dv_full = 0;
  I2c_du_full = 0; I2c_dv_full = 0;

  // margin needed to compute descriptor + sobel responses
  margin = 5+1;
  
  // adjust match radius on half resolution
  if (param.half_resolution)
    this->param.match_radius /= 2;
}

// deconstructor
Matcher::~Matcher() {
  if (I1p)          _mm_free(I1p);
  if (I2p)          _mm_free(I2p);
  if (I1c)          _mm_free(I1c);
  if (I2c)          _mm_free(I2c);
  if (m1p1)         _mm_free(m1p1);
  if (m1p2)         _mm_free(m1p2);
  if (I1p_du)       _mm_free(I1p_du);
  if (I1p_dv)       _mm_free(I1p_dv);
  if (I1p_du_full)  _mm_free(I1p_du_full);
  if (I1p_dv_full)  _mm_free(I1p_dv_full);
  if (m2p1)         _mm_free(m2p1);
  if (m2p2)         _mm_free(m2p2);
  if (I2p_du)       _mm_free(I2p_du);
  if (I2p_dv)       _mm_free(I2p_dv);
  if (I2p_du_full)  _mm_free(I2p_du_full);
  if (I2p_dv_full)  _mm_free(I2p_dv_full);
  if (m1c1)         _mm_free(m1c1);
  if (m1c2)         _mm_free(m1c2);
  if (I1c_du)       _mm_free(I1c_du);
  if (I1c_dv)       _mm_free(I1c_dv);
  if (I1c_du_full)  _mm_free(I1c_du_full);
  if (I1c_dv_full)  _mm_free(I1c_dv_full);
  if (m2c1)         _mm_free(m2c1);
  if (m2c2)         _mm_free(m2c2);
  if (I2c_du)       _mm_free(I2c_du);
  if (I2c_dv)       _mm_free(I2c_dv);
  if (I2c_du_full)  _mm_free(I2c_du_full);
  if (I2c_dv_full)  _mm_free(I2c_dv_full);
}

void Matcher::pushBack (uint8_t *I1,uint8_t* I2,int32_t* dims,const bool replace) {

  // image dimensions
  int32_t width  = dims[0];
  int32_t height = dims[1];
  int32_t bpl    = dims[2];

  // sanity check
  if (width<=0 || height<=0 || bpl<width || I1==0) {
    cerr << "ERROR: Image dimension mismatch!" << endl;
    return;
  }

  if (replace) {
    if (I1c)         _mm_free(I1c);
    if (I2c)         _mm_free(I2c);
    if (m1c1)        _mm_free(m1c1);
    if (m1c2)        _mm_free(m1c2);
    if (I1c_du)      _mm_free(I1c_du);
    if (I1c_dv)      _mm_free(I1c_dv);
    if (I1c_du_full) _mm_free(I1c_du_full);
    if (I1c_dv_full) _mm_free(I1c_dv_full);
    if (m2c1)        _mm_free(m2c1);
    if (m2c2)        _mm_free(m2c2);
    if (I2c_du)      _mm_free(I2c_du);
    if (I2c_dv)      _mm_free(I2c_dv);
    if (I2c_du_full) _mm_free(I2c_du_full);
    if (I2c_dv_full) _mm_free(I2c_dv_full);
  } else {
    if (I1p)         _mm_free(I1p);
    if (I2p)         _mm_free(I2p);
    if (m1p1)        _mm_free(m1p1);
    if (m1p2)        _mm_free(m1p2);
    if (I1p_du)      _mm_free(I1p_du);
    if (I1p_dv)      _mm_free(I1p_dv);
    if (I1p_du_full) _mm_free(I1p_du_full);
    if (I1p_dv_full) _mm_free(I1p_dv_full);
    if (m2p1)        _mm_free(m2p1);
    if (m2p2)        _mm_free(m2p2);
    if (I2p_du)      _mm_free(I2p_du);
    if (I2p_dv)      _mm_free(I2p_dv);
    if (I2p_du_full) _mm_free(I2p_du_full);
    if (I2p_dv_full) _mm_free(I2p_dv_full);
    m1p1 = m1c1; n1p1 = n1c1;
    m1p2 = m1c2; n1p2 = n1c2;
    m2p1 = m2c1; n2p1 = n2c1;
    m2p2 = m2c2; n2p2 = n2c2;
    I1p         = I1c;
    I2p         = I2c;
    I1p_du      = I1c_du;
    I1p_dv      = I1c_dv;
    I1p_du_full = I1c_du_full;
    I1p_dv_full = I1c_dv_full;
    I2p_du      = I2c_du;
    I2p_dv      = I2c_dv;
    I2p_du_full = I2c_du_full;
    I2p_dv_full = I2c_dv_full;
    dims_p[0]   = dims_c[0];
    dims_p[1]   = dims_c[1];
    dims_p[2]   = dims_c[2];
  }

  // set new dims (bytes per line must be multiple of 16)
  dims_c[0] = width;
  dims_c[1] = height;
  dims_c[2] = width + 15-(width-1)%16;

  // copy images to byte aligned memory
  I1c = (uint8_t*)_mm_malloc(dims_c[2]*dims_c[1]*sizeof(uint8_t),16);
  I2c = (uint8_t*)_mm_malloc(dims_c[2]*dims_c[1]*sizeof(uint8_t),16);
  if (dims_c[2]==bpl) {
    memcpy(I1c,I1,dims_c[2]*dims_c[1]*sizeof(uint8_t));
    if (I2!=0)
      memcpy(I2c,I2,dims_c[2]*dims_c[1]*sizeof(uint8_t));
  } else {
    for (int32_t v=0; v<height; v++) {
      memcpy(I1c+v*dims_c[2],I1+v*bpl,dims_c[0]*sizeof(uint8_t));
      if (I2!=0)
        memcpy(I2c+v*dims_c[2],I2+v*bpl,dims_c[0]*sizeof(uint8_t));
    }
  }

  // compute new features for current frame
  computeFeatures(I1c,dims_c,m1c1,n1c1,m1c2,n1c2,I1c_du,I1c_dv,I1c_du_full,I1c_dv_full);
  if (I2!=0)
    computeFeatures(I2c,dims_c,m2c1,n2c1,m2c2,n2c2,I2c_du,I2c_dv,I2c_du_full,I2c_dv_full);
}
// this seems not to improve accuracy
void Matcher::refineFeatures(std::vector<p_match> & vMatches)
{
    //refine locations of features in current left image using FAST corner score
    if(param.half_resolution)
    {
       const int patternSize=16;
       int pixel[25];
       int32_t bpl    = dims_c[2];
       int buf[5]={0, -bpl, bpl, -1, 1}; // center, up, down, left, right

       makeOffsets(pixel, dims_c[0], patternSize);
       // pattern size 16, window size 7x7, while here descriptors are computed with 11x11 window
       for(unsigned int nFeat=0; nFeat<vMatches.size(); ++nFeat){
           int max_score=0, score=0, max_id=-1;
           unsigned char *ptr= I1c + getAddressOffsetImage(vMatches[nFeat].u1c,vMatches[nFeat].v1c,bpl);
           for( int i=0; i< 5; ++i){
               score=cornerScore<patternSize>(ptr+buf[i], pixel, 1);
               if(score>max_score)
               {
                   max_score=score;
                   max_id=i;
               }
           }
           switch(max_id){
           case 1:
               --vMatches[nFeat].v1c;
               break;
               case 2:
               ++vMatches[nFeat].v1c;
               break;
               case 3:
               --vMatches[nFeat].u1c;
               break;
               case 4:
               ++vMatches[nFeat].u1c;
               break;
           default: break; //cerr<<"error in refineFeatures!"<<endl;
           }
       }
    }
}
void Matcher::matchFeatures(int32_t method, const Matrix *Tr_delta) {
  
  //////////////////
  // sanity check //
  //////////////////
  
  // flow
  if (method==0) {
    if (m1p2==0 || n1p2==0 || m1c2==0 || n1c2==0)
      return;
    if (param.multi_stage)
      if (m1p1==0 || n1p1==0 || m1c1==0 || n1c1==0)
        return;
    
  // stereo
  } else if (method==1) {
    if (m1c2==0 || n1c2==0 || m2c2==0 || n2c2==0)
      return;
    if (param.multi_stage)
      if (m1c1==0 || n1c1==0 || m2c1==0 || n2c1==0)
        return;
    
  // quad matching
  } else {
    if (m1p2==0 || n1p2==0 || m2p2==0 || n2p2==0 || m1c2==0 || n1c2==0 || m2c2==0 || n2c2==0)
      return;
    if (param.multi_stage)
      if (m1p1==0 || n1p1==0 || m2p1==0 || n2p1==0 || m1c1==0 || n1c1==0 || m2c1==0 || n2c1==0)
        return;    
  }

  // clear old matches
  p_matched_1.clear();
  p_matched_2.clear();

  // double pass matching
  if (param.multi_stage) {

    // 1st pass (sparse matches)
    matching(m1p1,m2p1,m1c1,m2c1,n1p1,n2p1,n1c1,n2c1,p_matched_1,method,false,Tr_delta);
    removeOutliers(p_matched_1,method);
    
    // compute search range prior statistics (used for speeding up 2nd pass)
    computePriorStatistics(p_matched_1,method);      

    // 2nd pass (dense matches)
    matching(m1p2,m2p2,m1c2,m2c2,n1p2,n2p2,n1c2,n2c2,p_matched_2,method,true,Tr_delta);
    if (param.refinement>0){
  //      refineFeatures(p_matched_2);//refine u1c v1c
        refinement(p_matched_2,method);// refine others
    }
    removeOutliers(p_matched_2,method);

  // single pass matching
  } else {
    matching(m1p2,m2p2,m1c2,m2c2,n1p2,n2p2,n1c2,n2c2,p_matched_2,method,false,Tr_delta);
    if (param.refinement>0){
    //  refineFeatures(p_matched_2);//refine u1c v1c
      refinement(p_matched_2,method);//refine others
    }
    removeOutliers(p_matched_2,method);
  }
}

void Matcher::bucketFeatures(int32_t max_features,float bucket_width,float bucket_height) {

  // find max values
  float u_max = 0;
  float v_max = 0;
  for (vector<p_match>::iterator it = p_matched_2.begin(); it!=p_matched_2.end(); it++) {
    if (it->u1c>u_max) u_max=it->u1c;
    if (it->v1c>v_max) v_max=it->v1c;
  }

  // allocate number of buckets needed
  int32_t bucket_cols = (int32_t)floor(u_max/bucket_width)+1;
  int32_t bucket_rows = (int32_t)floor(v_max/bucket_height)+1;
  vector<p_match> *buckets = new vector<p_match>[bucket_cols*bucket_rows];

  // assign matches to their buckets
  for (vector<p_match>::iterator it=p_matched_2.begin(); it!=p_matched_2.end(); it++) {
    int32_t u = (int32_t)floor(it->u1c/bucket_width);
    int32_t v = (int32_t)floor(it->v1c/bucket_height);
    buckets[v*bucket_cols+u].push_back(*it);
  }
  
  // refill p_matched from buckets
  p_matched_2.clear();
  for (int32_t i=0; i<bucket_cols*bucket_rows; i++) {
    
    // shuffle bucket indices randomly
    std::random_shuffle(buckets[i].begin(),buckets[i].end());
    
    // add up to max_features features from this bucket to p_matched
    int32_t k=0;
    for (vector<p_match>::iterator it=buckets[i].begin(); it!=buckets[i].end(); it++) {
      p_matched_2.push_back(*it);
      k++;
      if (k>=max_features)
        break;
    }
  }

  // free buckets
  delete []buckets;
}

float Matcher::getGain (vector<int32_t> inliers) {

  // check if two images are provided and matched
  if (I1p==0 || I1c==0 || p_matched_2.size()==0 || inliers.size()==0)
    return 1;

  int32_t window_size = 3;
  float   gain        = 0;
  int32_t num         = 0;
  int32_t u_min,u_max,v_min,v_max;
  float   mean_prev,mean_curr;

  for (vector<int32_t>::iterator it=inliers.begin(); it!=inliers.end(); it++) {
    if (*it<(int32_t)p_matched_2.size()) {

      // mean in previous image
      u_min = min(max((int32_t)p_matched_2[*it].u1p-window_size,0),dims_p[0]);
      u_max = min(max((int32_t)p_matched_2[*it].u1p+window_size,0),dims_p[0]);
      v_min = min(max((int32_t)p_matched_2[*it].v1p-window_size,0),dims_p[1]);
      v_max = min(max((int32_t)p_matched_2[*it].v1p+window_size,0),dims_p[1]);
      mean_prev = mean(I1p,dims_p[2],u_min,u_max,v_min,v_max);

      // mean in current image
      u_min = min(max((int32_t)p_matched_2[*it].u1c-window_size,0),dims_p[0]);
      u_max = min(max((int32_t)p_matched_2[*it].u1c+window_size,0),dims_p[0]);
      v_min = min(max((int32_t)p_matched_2[*it].v1c-window_size,0),dims_p[1]);
      v_max = min(max((int32_t)p_matched_2[*it].v1c+window_size,0),dims_p[1]);
      mean_curr = mean(I1c,dims_c[2],u_min,u_max,v_min,v_max);

      if (mean_prev>10) {
        gain += mean_curr/mean_prev;
        num++;
      }
    }
  }

  if (num>0) return gain/=(float)num;
  else       return 1;
}

///////////////////////
// PRIVATE FUNCTIONS //
///////////////////////

void Matcher::nonMaximumSuppression (int16_t* I_f1,int16_t* I_f2,const int32_t* dims,vector<Matcher::maximum> &maxima,int32_t nms_n) {
  
  // extract parameters
  int32_t width  = dims[0];
  int32_t height = dims[1];
  int32_t bpl    = dims[2];
  int32_t n      = nms_n;
  int32_t tau    = param.nms_tau;
  
  // loop variables
  int32_t f1mini,f1minj,f1maxi,f1maxj,f2mini,f2minj,f2maxi,f2maxj;
  int32_t f1minval,f1maxval,f2minval,f2maxval,currval;
  int32_t addr;
  
  for (int32_t i=n+margin; i<width-n-margin;i+=n+1) {
    for (int32_t j=n+margin; j<height-n-margin;j+=n+1) {

      f1mini = i; f1minj = j; f1maxi = i; f1maxj = j;
      f2mini = i; f2minj = j; f2maxi = i; f2maxj = j;
      
      addr     = getAddressOffsetImage(i,j,bpl);
      f1minval = *(I_f1+addr);
      f1maxval = f1minval;
      f2minval = *(I_f2+addr);
      f2maxval = f2minval;

      for (int32_t i2=i; i2<=i+n; i2++) {
        for (int32_t j2=j; j2<=j+n; j2++) {
          addr    = getAddressOffsetImage(i2,j2,bpl);
          currval = *(I_f1+addr);
          if (currval<f1minval) {
            f1mini   = i2;
            f1minj   = j2;
            f1minval = currval;
          } else if (currval>f1maxval) {
            f1maxi   = i2;
            f1maxj   = j2;
            f1maxval = currval;
          }
          currval = *(I_f2+addr);
          if (currval<f2minval) {
            f2mini   = i2;
            f2minj   = j2;
            f2minval = currval;
          } else if (currval>f2maxval) {
            f2maxi   = i2;
            f2maxj   = j2;
            f2maxval = currval;
          }
        }
      }
      
      // f1 minimum
      for (int32_t i2=f1mini-n; i2<=min(f1mini+n,width-1-margin); i2++) {
        for (int32_t j2=f1minj-n; j2<=min(f1minj+n,height-1-margin); j2++) {
          currval = *(I_f1+getAddressOffsetImage(i2,j2,bpl));
          if (currval<f1minval && (i2<i || i2>i+n || j2<j || j2>j+n))
            goto failed_f1min;            
        }
      }
      if (f1minval<=-tau)
        maxima.push_back(Matcher::maximum(f1mini,f1minj,f1minval,0));
      failed_f1min:;

      // f1 maximum
      for (int32_t i2=f1maxi-n; i2<=min(f1maxi+n,width-1-margin); i2++) {
        for (int32_t j2=f1maxj-n; j2<=min(f1maxj+n,height-1-margin); j2++) {
          currval = *(I_f1+getAddressOffsetImage(i2,j2,bpl));
          if (currval>f1maxval && (i2<i || i2>i+n || j2<j || j2>j+n))
            goto failed_f1max;
        }
      }
      if (f1maxval>=tau)
        maxima.push_back(Matcher::maximum(f1maxi,f1maxj,f1maxval,1));
      failed_f1max:;
      
      // f2 minimum
      for (int32_t i2=f2mini-n; i2<=min(f2mini+n,width-1-margin); i2++) {
        for (int32_t j2=f2minj-n; j2<=min(f2minj+n,height-1-margin); j2++) {
          currval = *(I_f2+getAddressOffsetImage(i2,j2,bpl));
          if (currval<f2minval && (i2<i || i2>i+n || j2<j || j2>j+n))
            goto failed_f2min;
        }
      }
      if (f2minval<=-tau)
        maxima.push_back(Matcher::maximum(f2mini,f2minj,f2minval,2));
      failed_f2min:;

      // f2 maximum
      for (int32_t i2=f2maxi-n; i2<=min(f2maxi+n,width-1-margin); i2++) {
        for (int32_t j2=f2maxj-n; j2<=min(f2maxj+n,height-1-margin); j2++) {
          currval = *(I_f2+getAddressOffsetImage(i2,j2,bpl));
          if (currval>f2maxval && (i2<i || i2>i+n || j2<j || j2>j+n))
            goto failed_f2max;
        }
      }
      if (f2maxval>=tau)
        maxima.push_back(Matcher::maximum(f2maxi,f2maxj,f2maxval,3));
      failed_f2max:;
    }
  }
}

inline void Matcher::computeDescriptor (const uint8_t* I_du,const uint8_t* I_dv,const int32_t &bpl,const int32_t &u,const int32_t &v,uint8_t *desc_addr) {
  
    // get address indices
  int32_t addr_m1 = getAddressOffsetImage(u,v-1,bpl);
  int32_t addr_m3 = addr_m1-2*bpl;
  int32_t addr_m5 = addr_m3-2*bpl;
  int32_t addr_p1 = addr_m1+2*bpl;
  int32_t addr_p3 = addr_p1+2*bpl;
  int32_t addr_p5 = addr_p3+2*bpl;
  
  // compute descriptor
  uint32_t k = 0;
  desc_addr[k++] = I_du[addr_m1-3];
  desc_addr[k++] = I_dv[addr_m1-3];
  desc_addr[k++] = I_du[addr_p1-3];
  desc_addr[k++] = I_dv[addr_p1-3];
  desc_addr[k++] = I_du[addr_m1-1];
  desc_addr[k++] = I_dv[addr_m1-1];
  desc_addr[k++] = I_du[addr_p1-1];
  desc_addr[k++] = I_dv[addr_p1-1];
  desc_addr[k++] = I_du[addr_m1+3];
  desc_addr[k++] = I_dv[addr_m1+3];
  desc_addr[k++] = I_du[addr_p1+3];
  desc_addr[k++] = I_dv[addr_p1+3];
  desc_addr[k++] = I_du[addr_m1+1];
  desc_addr[k++] = I_dv[addr_m1+1];
  desc_addr[k++] = I_du[addr_p1+1];
  desc_addr[k++] = I_dv[addr_p1+1];
  desc_addr[k++] = I_du[addr_m5-1];
  desc_addr[k++] = I_dv[addr_m5-1];
  desc_addr[k++] = I_du[addr_p5-1];
  desc_addr[k++] = I_dv[addr_p5-1];
  desc_addr[k++] = I_du[addr_m5+1];
  desc_addr[k++] = I_dv[addr_m5+1];
  desc_addr[k++] = I_du[addr_p5+1];
  desc_addr[k++] = I_dv[addr_p5+1];
  desc_addr[k++] = I_du[addr_m3-5];
  desc_addr[k++] = I_dv[addr_m3-5];
  desc_addr[k++] = I_du[addr_p3-5];
  desc_addr[k++] = I_dv[addr_p3-5];
  desc_addr[k++] = I_du[addr_m3+5];
  desc_addr[k++] = I_dv[addr_m3+5];
  desc_addr[k++] = I_du[addr_p3+5];
  desc_addr[k++] = I_dv[addr_p3+5];
}

inline void Matcher::computeSmallDescriptor (const uint8_t* I_du,const uint8_t* I_dv,const int32_t &bpl,const int32_t &u,const int32_t &v,uint8_t *desc_addr) {
  
  // get address indices
  int32_t addr2 = getAddressOffsetImage(u,v,bpl);
  int32_t addr1 = addr2-bpl;
  int32_t addr0 = addr1-bpl;
  int32_t addr3 = addr2+bpl;
  int32_t addr4 = addr3+bpl;
  
  // compute ELAS-descriptor
  uint32_t k = 0;
  desc_addr[k++] = I_du[addr0];
  desc_addr[k++] = I_du[addr1-2];
  desc_addr[k++] = I_du[addr1];
  desc_addr[k++] = I_du[addr1+2];
  desc_addr[k++] = I_du[addr2-1];
  desc_addr[k++] = I_du[addr2];
  desc_addr[k++] = I_du[addr2];
  desc_addr[k++] = I_du[addr2+1];
  desc_addr[k++] = I_du[addr3-2];
  desc_addr[k++] = I_du[addr3];
  desc_addr[k++] = I_du[addr3+2];
  desc_addr[k++] = I_du[addr4];
  desc_addr[k++] = I_dv[addr1];
  desc_addr[k++] = I_dv[addr2-1];
  desc_addr[k++] = I_dv[addr2+1];
  desc_addr[k++] = I_dv[addr3];
}

void Matcher::computeDescriptors (uint8_t* I_du,uint8_t* I_dv,const int32_t bpl,std::vector<Matcher::maximum> &maxima) {
  
  // loop variables
  int32_t u,v;
  uint8_t *desc_addr;
  
  // for all maxima do
  for (vector<Matcher::maximum>::iterator it=maxima.begin(); it!=maxima.end(); it++) {
    u = (*it).u;
    v = (*it).v;
    desc_addr = (uint8_t*)(&((*it).d1));
    computeDescriptor(I_du,I_dv,bpl,u,v,desc_addr);    
  }
}

inline uint8_t Matcher::saturate (int16_t in) {
  if (in<0)   return 0;
  if (in>255) return 255;
  return in;
}

void Matcher::filterImageAll (uint8_t* I,uint8_t* I_du,uint8_t* I_dv,int16_t* I_f1,int16_t* I_f2,const int* dims) {
  
  // get bpl and height  
  const int32_t height = dims[1];
  const int32_t bpl    = dims[2];
  
  // init sobel pointers
  const uint8_t* p00 = I + 0;
  const uint8_t* p01 = I + 1;
  const uint8_t* p02 = I + 2;
  const uint8_t* p03 = I + 3;
  const uint8_t* p04 = I + 4;
  const uint8_t* p10 = I + 0 + bpl;
  const uint8_t* p11 = I + 1 + bpl;
  const uint8_t* p12 = I + 2 + bpl;
  const uint8_t* p13 = I + 3 + bpl;
  const uint8_t* p14 = I + 4 + bpl;
  const uint8_t* p20 = I + 0 + 2*bpl;
  const uint8_t* p21 = I + 1 + 2*bpl;
  const uint8_t* p22 = I + 2 + 2*bpl;
  const uint8_t* p23 = I + 3 + 2*bpl;
  const uint8_t* p24 = I + 4 + 2*bpl;
  const uint8_t* p30 = I + 0 + 3*bpl;
  const uint8_t* p31 = I + 1 + 3*bpl;
  const uint8_t* p32 = I + 2 + 3*bpl;
  const uint8_t* p33 = I + 3 + 3*bpl;
  const uint8_t* p34 = I + 4 + 3*bpl;
  const uint8_t* p40 = I + 0 + 4*bpl;
  const uint8_t* p41 = I + 1 + 4*bpl;
  const uint8_t* p42 = I + 2 + 4*bpl;
  const uint8_t* p43 = I + 3 + 4*bpl;
  const uint8_t* p44 = I + 4 + 4*bpl;

  // init output pointers
  uint8_t* result_du = I_du +   bpl + 1;
  uint8_t* result_dv = I_dv +   bpl + 1;
  int16_t* result_f1 = I_f1 + 2*bpl + 2;
  int16_t* result_f2 = I_f2 + 2*bpl + 2;

  // stop here
  const uint8_t* end_input = I + bpl*height;

  // compute filter responses (border pixels are invalid)
  for( ; p44 != end_input; p00++, p01++, p02++, p03++, p04++,
                           p10++, p11++, p12++, p13++, p14++,
                           p20++, p21++, p22++, p23++, p24++,
                           p30++, p31++, p32++, p33++, p34++,
                           p40++, p41++, p42++, p43++, p44++,
                           result_du++, result_dv++, result_f1++, result_f2++ ) {
    int16_t temp_du = - *p00 - 2 * *p10 - *p20 + *p02 + 2 * *p12 + *p22;
    int16_t temp_dv = - *p00 - 2 * *p01 - *p02 + *p20 + 2 * *p21 + *p22;
    *result_du = saturate( temp_du/4 + 128 );
    *result_dv = saturate( temp_dv/4 + 128 );
    *result_f1 = - *p00 - *p01 -     *p02 - *p03 - *p04
                 - *p10 + *p11 +     *p12 + *p13 - *p14
                 - *p20 + *p21 + 8 * *p22 + *p23 - *p24
                 - *p30 + *p31 +     *p32 + *p33 - *p34
                 - *p40 - *p41 -     *p42 - *p43 - *p44;
    *result_f2 = - *p00 - *p01 + *p03 + *p04
                 - *p10 - *p11 + *p13 + *p14
                 + *p30 + *p31 - *p33 - *p34
                 + *p40 + *p41 - *p43 - *p44;
  }
}

void Matcher::filterImageSobel (uint8_t* I,uint8_t* I_du,uint8_t* I_dv,const int* dims) {
  
  // get image width and height  
  const int32_t height = dims[1];
  const int32_t bpl    = dims[2];
  
  // init sobel pointers
  const uint8_t* p00 = I + 0;
  const uint8_t* p01 = I + 1;
  const uint8_t* p02 = I + 2;
  const uint8_t* p10 = I + 0 + bpl;
  const uint8_t* p11 = I + 1 + bpl;
  const uint8_t* p12 = I + 2 + bpl;
  const uint8_t* p20 = I + 0 + 2*bpl;
  const uint8_t* p21 = I + 1 + 2*bpl;
  const uint8_t* p22 = I + 2 + 2*bpl;

  // init output pointers
  uint8_t* result_du = I_du + bpl + 1;
  uint8_t* result_dv = I_dv + bpl + 1;

  // stop here
  const uint8_t* end_input = I + bpl*height;

  // compute filter responses (border pixels are invalid)
  for( ; p22 != end_input; p00++, p01++, p02++,
                           p10++, p11++, p12++,
                           p20++, p21++, p22++,
                           result_du++, result_dv++) {
    int16_t temp_du = - *p00 - 2 * *p10 - *p20 + *p02 + 2 * *p12 + *p22;
    int16_t temp_dv = - *p00 - 2 * *p01 - *p02 + *p20 + 2 * *p21 + *p22;
    *result_du = saturate( temp_du/4 + 128 );
    *result_dv = saturate( temp_dv/4 + 128 );
  }
}

void Matcher::getHalfResolutionDimensions(const int32_t *dims,int32_t *dims_half) {
  dims_half[0] = dims[0]/2;
  dims_half[1] = dims[1]/2;
  dims_half[2] = dims_half[0]+15-(dims_half[0]-1)%16;
}

uint8_t* Matcher::createHalfResolutionImage(uint8_t *I,const int32_t* dims) {
  int32_t dims_half[3];
  getHalfResolutionDimensions(dims,dims_half);
  uint8_t* I_half = (uint8_t*)_mm_malloc(dims_half[2]*dims_half[1]*sizeof(uint8_t),16);
  for (int32_t v=0; v<dims_half[1]; v++)
    for (int32_t u=0; u<dims_half[0]; u++)
      I_half[v*dims_half[2]+u] =  (uint8_t)(((int32_t)I[(v*2+0)*dims[2]+u*2+0]+
                                             (int32_t)I[(v*2+0)*dims[2]+u*2+1]+
                                             (int32_t)I[(v*2+1)*dims[2]+u*2+0]+
                                             (int32_t)I[(v*2+1)*dims[2]+u*2+1])/4);
  return I_half;
}

void Matcher::computeFeatures (uint8_t *I,const int32_t* dims,int32_t* &max1,int32_t &num1,int32_t* &max2,int32_t &num2,uint8_t* &I_du,uint8_t* &I_dv,uint8_t* &I_du_full,uint8_t* &I_dv_full) {
  
  int16_t *I_f1;
  int16_t *I_f2;
  
  int32_t dims_matching[3];
  memcpy(dims_matching,dims,3*sizeof(int32_t));
  
  // allocate memory for sobel images and filter images
  if (!param.half_resolution) {
    I_du = (uint8_t*)_mm_malloc(dims[2]*dims[1]*sizeof(uint8_t*),16);
    I_dv = (uint8_t*)_mm_malloc(dims[2]*dims[1]*sizeof(uint8_t*),16);
    I_f1 = (int16_t*)_mm_malloc(dims[2]*dims[1]*sizeof(int16_t),16);
    I_f2 = (int16_t*)_mm_malloc(dims[2]*dims[1]*sizeof(int16_t),16);
    filter::sobel5x5(I,I_du,I_dv,dims[2],dims[1]);
    filter::blob5x5(I,I_f1,dims[2],dims[1]);
    filter::checkerboard5x5(I,I_f2,dims[2],dims[1]);
  } else {
    uint8_t* I_matching = createHalfResolutionImage(I,dims);
    getHalfResolutionDimensions(dims,dims_matching);
    I_du      = (uint8_t*)_mm_malloc(dims_matching[2]*dims_matching[1]*sizeof(uint8_t*),16);
    I_dv      = (uint8_t*)_mm_malloc(dims_matching[2]*dims_matching[1]*sizeof(uint8_t*),16);
    I_f1      = (int16_t*)_mm_malloc(dims_matching[2]*dims_matching[1]*sizeof(int16_t),16);
    I_f2      = (int16_t*)_mm_malloc(dims_matching[2]*dims_matching[1]*sizeof(int16_t),16);
    I_du_full = (uint8_t*)_mm_malloc(dims[2]*dims[1]*sizeof(uint8_t*),16);
    I_dv_full = (uint8_t*)_mm_malloc(dims[2]*dims[1]*sizeof(uint8_t*),16);
    filter::sobel5x5(I_matching,I_du,I_dv,dims_matching[2],dims_matching[1]);
    filter::sobel5x5(I,I_du_full,I_dv_full,dims[2],dims[1]);
    filter::blob5x5(I_matching,I_f1,dims_matching[2],dims_matching[1]);
    filter::checkerboard5x5(I_matching,I_f2,dims_matching[2],dims_matching[1]);
    _mm_free(I_matching);
  }
  
  // extract sparse maxima (1st pass) via non-maximum suppression
  vector<Matcher::maximum> maxima1;
  if (param.multi_stage) {
    int32_t nms_n_sparse = param.nms_n*3;
    if (nms_n_sparse>10)
      nms_n_sparse = max(param.nms_n,10);
    nonMaximumSuppression(I_f1,I_f2,dims_matching,maxima1,nms_n_sparse);
    computeDescriptors(I_du,I_dv,dims_matching[2],maxima1);
  }
  
  // extract dense maxima (2nd pass) via non-maximum suppression
  vector<Matcher::maximum> maxima2;
  nonMaximumSuppression(I_f1,I_f2,dims_matching,maxima2,param.nms_n);
  computeDescriptors(I_du,I_dv,dims_matching[2],maxima2);

  // release filter images
  _mm_free(I_f1);
  _mm_free(I_f2);  
  
  // get number of interest points and init maxima pointer to NULL
  num1 = maxima1.size();
  num2 = maxima2.size();
  max1 = 0;
  max2 = 0;
  
  int32_t s = 1;
  if (param.half_resolution)
    s = 2;

  // return sparse maxima as 16-bytes aligned memory
  if (num1!=0) {
    max1 = (int32_t*)_mm_malloc(sizeof(Matcher::maximum)*num1,16);
    int32_t k=0;
    for (vector<Matcher::maximum>::iterator it=maxima1.begin(); it!=maxima1.end(); it++) {
      *(max1+k++) = it->u*s;  *(max1+k++) = it->v*s;  *(max1+k++) = 0;        *(max1+k++) = it->c;
      *(max1+k++) = it->d1;   *(max1+k++) = it->d2;   *(max1+k++) = it->d3;   *(max1+k++) = it->d4;
      *(max1+k++) = it->d5;   *(max1+k++) = it->d6;   *(max1+k++) = it->d7;   *(max1+k++) = it->d8;
    }
  }
  
  // return dense maxima as 16-bytes aligned memory
  if (num2!=0) {
    max2 = (int32_t*)_mm_malloc(sizeof(Matcher::maximum)*num2,16);
    int32_t k=0;
    for (vector<Matcher::maximum>::iterator it=maxima2.begin(); it!=maxima2.end(); it++) {
      *(max2+k++) = it->u*s;  *(max2+k++) = it->v*s;  *(max2+k++) = 0;        *(max2+k++) = it->c;
      *(max2+k++) = it->d1;   *(max2+k++) = it->d2;   *(max2+k++) = it->d3;   *(max2+k++) = it->d4;
      *(max2+k++) = it->d5;   *(max2+k++) = it->d6;   *(max2+k++) = it->d7;   *(max2+k++) = it->d8;
    }
  }
}

void Matcher::computePriorStatistics (vector<p_match> &p_matched,int32_t method) {
   
  // compute number of bins
  int32_t u_bin_num = (int32_t)ceil((float)dims_c[0]/(float)param.match_binsize);
  int32_t v_bin_num = (int32_t)ceil((float)dims_c[1]/(float)param.match_binsize);
  int32_t bin_num   = v_bin_num*u_bin_num;
  
  // number of matching stages
  int32_t num_stages = 2;
  if (method==2)
    num_stages = 4;
  
  // allocate bin accumulator memory
  vector<Matcher::delta> *delta_accu = new vector<Matcher::delta>[bin_num];
  
  // fill bin accumulator
  Matcher::delta delta_curr;
  for (vector<p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {

    // method flow: compute position delta
    if (method==0) {
      delta_curr.val[0] = it->u1p - it->u1c;
      delta_curr.val[1] = it->v1p - it->v1c;
      delta_curr.val[2] = it->u1c - it->u1p;
      delta_curr.val[3] = it->v1c - it->v1p;
      
    // method stereo: compute position delta
    } else if (method==1) {
      delta_curr.val[0] = it->u2c - it->u1c;
      delta_curr.val[1] = 0; 
      delta_curr.val[2] = it->u1c - it->u2c;
      delta_curr.val[3] = 0;     
      
    // method quad matching: compute position delta
    } else {
      /*
      delta_curr.val[0] = it->u1p - it->u1c;
      delta_curr.val[1] = it->v1p - it->v1c;
      delta_curr.val[2] = it->u2p - it->u1p;
      delta_curr.val[3] = 0;
      delta_curr.val[4] = it->u2c - it->u2p;
      delta_curr.val[5] = it->v2c - it->v2p;
      delta_curr.val[6] = it->u1c - it->u2c;
      delta_curr.val[7] = 0;*/
      delta_curr.val[0] = it->u2p - it->u1p;
      delta_curr.val[1] = 0;
      delta_curr.val[2] = it->u2c - it->u2p;
      delta_curr.val[3] = it->v2c - it->v2p;
      delta_curr.val[4] = it->u1c - it->u2c;
      delta_curr.val[5] = 0;
      delta_curr.val[6] = it->u1p - it->u1c;
      delta_curr.val[7] = it->v1p - it->v1c;
    }
    
    // compute row and column of bin to which this observation belongs
    int32_t u_bin_min,u_bin_max,v_bin_min,v_bin_max;

    // flow + stereo: use current left image as reference
    if (method<2) {
      u_bin_min = min(max((int32_t)floor(it->u1c/(float)param.match_binsize)-1,0),u_bin_num-1);
      u_bin_max = min(max((int32_t)floor(it->u1c/(float)param.match_binsize)+1,0),u_bin_num-1);
      v_bin_min = min(max((int32_t)floor(it->v1c/(float)param.match_binsize)-1,0),v_bin_num-1);
      v_bin_max = min(max((int32_t)floor(it->v1c/(float)param.match_binsize)+1,0),v_bin_num-1);
      
    // quad matching: use current previous image as reference
    } else {
      u_bin_min = min(max((int32_t)floor(it->u1p/(float)param.match_binsize)-1,0),u_bin_num-1);
      u_bin_max = min(max((int32_t)floor(it->u1p/(float)param.match_binsize)+1,0),u_bin_num-1);
      v_bin_min = min(max((int32_t)floor(it->v1p/(float)param.match_binsize)-1,0),v_bin_num-1);
      v_bin_max = min(max((int32_t)floor(it->v1p/(float)param.match_binsize)+1,0),v_bin_num-1);
    }
    
    // add to accumulator
    for (int32_t v_bin=v_bin_min; v_bin<=v_bin_max; v_bin++)
      for (int32_t u_bin=u_bin_min; u_bin<=u_bin_max; u_bin++)
        delta_accu[v_bin*u_bin_num+u_bin].push_back(delta_curr);
  }
  
  // clear ranges
  ranges.clear();
  
  // for all bins compute statistics
  for (int32_t v_bin=0; v_bin<v_bin_num; v_bin++) {
    for (int32_t u_bin=0; u_bin<u_bin_num; u_bin++) {
      
      // use full range in case there are no observations
      delta delta_min(-param.match_radius);
      delta delta_max(+param.match_radius);
      
      // otherwise determine delta min and delta max
      if (delta_accu[v_bin*u_bin_num+u_bin].size()>0) {
        
        // init displacements 'delta' to 'infinite'
        delta_min = delta(+1000000);
        delta_max = delta(-1000000);
        
        // find minimum and maximum displacements
        for (vector<Matcher::delta>::iterator it=delta_accu[v_bin*u_bin_num+u_bin].begin();
             it!=delta_accu[v_bin*u_bin_num+u_bin].end(); it++) {
          for (int32_t i=0; i<num_stages*2; i++) {
            if (it->val[i]<delta_min.val[i]) delta_min.val[i] = it->val[i];
            if (it->val[i]>delta_max.val[i]) delta_max.val[i] = it->val[i];
          }
        }
      }
      
      // set search range for this bin
      range r;
      for (int32_t i=0; i<num_stages; i++) {
        
        // bound minimum search range to 20x20
        float delta_u = delta_max.val[i*2+0]-delta_min.val[i*2+0];
        if (delta_u<20) {
          delta_min.val[i*2+0] -= ceil((20-delta_u)/2);
          delta_max.val[i*2+0] += ceil((20-delta_u)/2);
        }
        float delta_v = delta_max.val[i*2+1]-delta_min.val[i*2+1];
        if (delta_v<20) {
          delta_min.val[i*2+1] -= ceil((20-delta_v)/2);
          delta_max.val[i*2+1] += ceil((20-delta_v)/2);
        }
        
        // set range for this bin
        r.u_min[i] = delta_min.val[i*2+0];
        r.u_max[i] = delta_max.val[i*2+0];
        r.v_min[i] = delta_min.val[i*2+1];
        r.v_max[i] = delta_max.val[i*2+1];
      }
      ranges.push_back(r);      
    }
  }
  
  // free bin accumulator memory
  delete []delta_accu;
}

void Matcher::createIndexVector (int32_t* m,int32_t n,vector<int32_t> *k,const int32_t &u_bin_num,const int32_t &v_bin_num) {

  // descriptor step size
  int32_t step_size = sizeof(Matcher::maximum)/sizeof(int32_t);
  
  // for all points do
  for (int32_t i=0; i<n; i++) {
    
    // extract coordinates and class
    int32_t u = *(m+step_size*i+0); // u-coordinate
    int32_t v = *(m+step_size*i+1); // v-coordinate
    int32_t c = *(m+step_size*i+3); // class
    
    // compute row and column of bin to which this observation belongs
    int32_t u_bin = min((int32_t)floor((float)u/(float)param.match_binsize),u_bin_num-1);
    int32_t v_bin = min((int32_t)floor((float)v/(float)param.match_binsize),v_bin_num-1);
    
    // save index
    k[(c*v_bin_num+v_bin)*u_bin_num+u_bin].push_back(i);
  }
}

inline void Matcher::findMatch (int32_t* m1,const int32_t &i1,int32_t* m2,const int32_t &step_size,vector<int32_t> *k2,
                                const int32_t &u_bin_num,const int32_t &v_bin_num,const int32_t &stat_bin,
                                int32_t& min_ind,int32_t stage,bool flow,bool use_prior,double u_,double v_) {
  
  // init and load image coordinates + feature
  min_ind          = 0;
  double  min_cost = 10000000;
  int32_t u1       = *(m1+step_size*i1+0);
  int32_t v1       = *(m1+step_size*i1+1);
  int32_t c        = *(m1+step_size*i1+3);
  __m128i xmm1     = _mm_load_si128((__m128i*)(m1+step_size*i1+4));
  __m128i xmm2     = _mm_load_si128((__m128i*)(m1+step_size*i1+8));
  
  float u_min,u_max,v_min,v_max;
  
  // restrict search range with prior
  if (use_prior) {
    u_min = u1+ranges[stat_bin].u_min[stage];
    u_max = u1+ranges[stat_bin].u_max[stage];
    v_min = v1+ranges[stat_bin].v_min[stage];
    v_max = v1+ranges[stat_bin].v_max[stage];
    
  // otherwise: use full search space
  } else {
    u_min = u1-param.match_radius;
    u_max = u1+param.match_radius;
    v_min = v1-param.match_radius;
    v_max = v1+param.match_radius;
  }
  
  // if stereo search => constrain to 1d
  if (!flow) {
    v_min = v1-param.match_disp_tolerance;
    v_max = v1+param.match_disp_tolerance;
  }
  
  // bins of interest
  int32_t u_bin_min = min(max((int32_t)floor(u_min/(float)param.match_binsize),0),u_bin_num-1);
  int32_t u_bin_max = min(max((int32_t)floor(u_max/(float)param.match_binsize),0),u_bin_num-1);
  int32_t v_bin_min = min(max((int32_t)floor(v_min/(float)param.match_binsize),0),v_bin_num-1);
  int32_t v_bin_max = min(max((int32_t)floor(v_max/(float)param.match_binsize),0),v_bin_num-1);
  
  // for all bins of interest do
  for (int32_t u_bin=u_bin_min; u_bin<=u_bin_max; u_bin++) {
    for (int32_t v_bin=v_bin_min; v_bin<=v_bin_max; v_bin++) {
      int32_t k2_ind = (c*v_bin_num+v_bin)*u_bin_num+u_bin;
      for (vector<int32_t>::const_iterator i2_it=k2[k2_ind].begin(); i2_it!=k2[k2_ind].end(); i2_it++) {
        int32_t u2   = *(m2+step_size*(*i2_it)+0);
        int32_t v2   = *(m2+step_size*(*i2_it)+1);
        if (u2>=u_min && u2<=u_max && v2>=v_min && v2<=v_max) {
          __m128i xmm3 = _mm_load_si128((__m128i*)(m2+step_size*(*i2_it)+4));
          __m128i xmm4 = _mm_load_si128((__m128i*)(m2+step_size*(*i2_it)+8));                    
          xmm3 = _mm_sad_epu8 (xmm1,xmm3);
          xmm4 = _mm_sad_epu8 (xmm2,xmm4);
          xmm4 = _mm_add_epi16(xmm3,xmm4);
          double cost = (double)(_mm_extract_epi16(xmm4,0)+_mm_extract_epi16(xmm4,4));
          
          if (u_>=0 && v_>=0) {
            double du = (double)u2-u_;
            double dv = (double)v2-v_;
            double dist = sqrt(du*du+dv*dv);
            cost += 4*dist;
          }
          
          if (cost<min_cost) {
            min_ind  = *i2_it;
            min_cost = cost;
          }
        }
      }
    }
  }
}

void Matcher::matching (int32_t *m1p,int32_t *m2p,int32_t *m1c,int32_t *m2c,
                        int32_t n1p,int32_t n2p,int32_t n1c,int32_t n2c,
                        vector<p_match> &p_matched,int32_t method,bool use_prior,const Matrix *Tr_delta) {

  // descriptor step size (number of int32_t elements in struct)
  int32_t step_size = sizeof(Matcher::maximum)/sizeof(int32_t);
  
  // compute number of bins
  int32_t u_bin_num = (int32_t)ceil((float)dims_c[0]/(float)param.match_binsize);
  int32_t v_bin_num = (int32_t)ceil((float)dims_c[1]/(float)param.match_binsize);
  int32_t bin_num   = 4*v_bin_num*u_bin_num; // 4 classes
  
  // allocate memory for index vectors (needed for efficient search)
  vector<int32_t> *k1p = new vector<int32_t>[bin_num];
  vector<int32_t> *k2p = new vector<int32_t>[bin_num];
  vector<int32_t> *k1c = new vector<int32_t>[bin_num];
  vector<int32_t> *k2c = new vector<int32_t>[bin_num];
  
  // loop variables
  int32_t* M = (int32_t*)calloc(dims_c[0]*dims_c[1],sizeof(int32_t));
  int32_t i1p,i2p,i1c,i2c,i1c2,i1p2;
  int32_t u1p,v1p,u2p,v2p,u1c,v1c,u2c,v2c;
  
  double t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23;
  if (Tr_delta) {
    t00 = Tr_delta->val[0][0];
    t01 = Tr_delta->val[0][1];
    t02 = Tr_delta->val[0][2];
    t03 = Tr_delta->val[0][3];
    t10 = Tr_delta->val[1][0];
    t11 = Tr_delta->val[1][1];
    t12 = Tr_delta->val[1][2];
    t13 = Tr_delta->val[1][3];
    t20 = Tr_delta->val[2][0];
    t21 = Tr_delta->val[2][1];
    t22 = Tr_delta->val[2][2];
    t23 = Tr_delta->val[2][3];
  }

  /////////////////////////////////////////////////////
  // method: flow
  if (method==0) {
    
    // create position/class bin index vectors
    createIndexVector(m1p,n1p,k1p,u_bin_num,v_bin_num);
    createIndexVector(m1c,n1c,k1c,u_bin_num,v_bin_num);
    
    // for all points do
    for (i1c=0; i1c<n1c; i1c++) {

      // coordinates in previous left image
      u1c = *(m1c+step_size*i1c+0);
      v1c = *(m1c+step_size*i1c+1);

      // compute row and column of statistics bin to which this observation belongs
      int32_t u_bin = min((int32_t)floor((float)u1c/(float)param.match_binsize),u_bin_num-1);
      int32_t v_bin = min((int32_t)floor((float)v1c/(float)param.match_binsize),v_bin_num-1);
      int32_t stat_bin = v_bin*u_bin_num+u_bin;

      // match forward/backward
      findMatch(m1c,i1c,m1p,step_size,k1p,u_bin_num,v_bin_num,stat_bin,i1p, 0,true,use_prior);
      findMatch(m1p,i1p,m1c,step_size,k1c,u_bin_num,v_bin_num,stat_bin,i1c2,1,true,use_prior);

      // circle closure success?
      if (i1c2==i1c) {

        // extract coordinates
        u1p = *(m1p+step_size*i1p+0);
        v1p = *(m1p+step_size*i1p+1);

        // add match if this pixel isn't matched yet
        if (*(M+getAddressOffsetImage(u1c,v1c,dims_c[0]))==0) {
          p_matched.push_back(p_match(u1p,v1p,i1p,-1,-1,-1,u1c,v1c,i1c,-1,-1,-1));
          *(M+getAddressOffsetImage(u1c,v1c,dims_c[0])) = 1;
        }
      }
    }
    
  /////////////////////////////////////////////////////
  // method: stereo
  } else if (method==1) {
    
    // create position/class bin index vectors
    createIndexVector(m1c,n1c,k1c,u_bin_num,v_bin_num);
    createIndexVector(m2c,n2c,k2c,u_bin_num,v_bin_num);
    
    // for all points do
    for (i1c=0; i1c<n1c; i1c++) {

      // coordinates in previous left image
      u1c = *(m1c+step_size*i1c+0);
      v1c = *(m1c+step_size*i1c+1);

      // compute row and column of statistics bin to which this observation belongs
      int32_t u_bin = min((int32_t)floor((float)u1c/(float)param.match_binsize),u_bin_num-1);
      int32_t v_bin = min((int32_t)floor((float)v1c/(float)param.match_binsize),v_bin_num-1);
      int32_t stat_bin = v_bin*u_bin_num+u_bin;

      // match left/right
      findMatch(m1c,i1c,m2c,step_size,k2c,u_bin_num,v_bin_num,stat_bin,i2c, 0,false,use_prior);
      findMatch(m2c,i2c,m1c,step_size,k1c,u_bin_num,v_bin_num,stat_bin,i1c2,1,false,use_prior);

      // circle closure success?
      if (i1c2==i1c) {

        // extract coordinates
        u2c = *(m2c+step_size*i2c+0);
        v2c = *(m2c+step_size*i2c+1);

        // if disparity is positive
        if (u1c>=u2c) {

          // add match if this pixel isn't matched yet
          if (*(M+getAddressOffsetImage(u1c,v1c,dims_c[0]))==0) {
            p_matched.push_back(p_match(-1,-1,-1,-1,-1,-1,u1c,v1c,i1c,u2c,v2c,i2c));
            *(M+getAddressOffsetImage(u1c,v1c,dims_c[0])) = 1;
          }
        }
      }
    }
    
  /////////////////////////////////////////////////////
  // method: quad matching
  } else {
    
    // create position/class bin index vectors
    createIndexVector(m1p,n1p,k1p,u_bin_num,v_bin_num);
    createIndexVector(m2p,n2p,k2p,u_bin_num,v_bin_num);
    createIndexVector(m1c,n1c,k1c,u_bin_num,v_bin_num);
    createIndexVector(m2c,n2c,k2c,u_bin_num,v_bin_num);
    
 /*   static int mytemp=0;
    char filename[300];
    sprintf(filename, "/media/jianzhuhuai0108/Mag/tempk1pandk1c%d.txt", mytemp);
    ++mytemp;
    std::ofstream output(filename, std::ios::out);
    for (int jack=0; jack<n1p;++jack)
    {
        u1p = *(m1p+step_size*jack+0);
        v1p = *(m1p+step_size*jack+1);
        output<<u1p<<" "<<v1p<<endl;
    }
    output<<endl<<endl<<endl;
    for (int jack=0; jack<n1c;++jack){
        u1c = *(m1c+step_size*jack+0);
        v1c = *(m1c+step_size*jack+1);
        output<<u1c<<" "<<v1c<<endl;
    }
    output<<endl;
    output.close();*/

    // for all points do
    for (i1p=0; i1p<n1p; i1p++) {

      // coordinates
      u1p = *(m1p+step_size*i1p+0);
      v1p = *(m1p+step_size*i1p+1);

      // compute row and column of statistics bin to which this observation belongs
      int32_t u_bin = min((int32_t)floor((float)u1p/(float)param.match_binsize),u_bin_num-1);
      int32_t v_bin = min((int32_t)floor((float)v1p/(float)param.match_binsize),v_bin_num-1);
      int32_t stat_bin = v_bin*u_bin_num+u_bin;

      // match in circle
      findMatch(m1p,i1p,m2p,step_size,k2p,u_bin_num,v_bin_num,stat_bin,i2p, 0,false,use_prior);

      u2p = *(m2p+step_size*i2p+0);
      v2p = *(m2p+step_size*i2p+1);

      if (Tr_delta) {
      
        double d = max((double)u1p-(double)u2p,1.0);
        double x1p = ((double)u1p-param.cu)*param.base/d;
        double y1p = ((double)v1p-param.cv)*param.base/d;
        double z1p = param.f*param.base/d;

        double x2c = t00*x1p + t01*y1p + t02*z1p + t03 - param.base;
        double y2c = t10*x1p + t11*y1p + t12*z1p + t13;
        double z2c = t20*x1p + t21*y1p + t22*z1p + t23;

        double u2c_ = param.f*x2c/z2c+param.cu;
        double v2c_ = param.f*y2c/z2c+param.cv;

        findMatch(m2p,i2p,m2c,step_size,k2c,u_bin_num,v_bin_num,stat_bin,i2c, 1,true ,use_prior,u2c_,v2c_);
      } else {
        findMatch(m2p,i2p,m2c,step_size,k2c,u_bin_num,v_bin_num,stat_bin,i2c, 1,true ,use_prior);
      }
      findMatch(m2c,i2c,m1c,step_size,k1c,u_bin_num,v_bin_num,stat_bin,i1c, 2,false,use_prior);
      if (Tr_delta)
        findMatch(m1c,i1c,m1p,step_size,k1p,u_bin_num,v_bin_num,stat_bin,i1p2,3,true ,use_prior,u1p,v1p);
      else
        findMatch(m1c,i1c,m1p,step_size,k1p,u_bin_num,v_bin_num,stat_bin,i1p2,3,true ,use_prior);
      
      // circle closure success?
      if (i1p2==i1p) {

        // extract coordinates
        u2c = *(m2c+step_size*i2c+0); v2c = *(m2c+step_size*i2c+1);
        u1c = *(m1c+step_size*i1c+0); v1c = *(m1c+step_size*i1c+1);

        // if disparities are positive
        if (u1p>=u2p && u1c>=u2c) {
          
          // add match
          p_matched.push_back(p_match(u1p,v1p,i1p,u2p,v2p,i2p,
                                               u1c,v1c,i1c,u2c,v2c,i2c));
        }
      }
    }
    
    // old version:
    /*
    // for all points do
    for (i1c=0; i1c<n1c; i1c++) {

      // coordinates
      u1c = *(m1c+step_size*i1c+0);
      v1c = *(m1c+step_size*i1c+1);

      // compute row and column of statistics bin to which this observation belongs
      int32_t u_bin = min((int32_t)floor((float)u1c/(float)param.match_binsize),u_bin_num-1);
      int32_t v_bin = min((int32_t)floor((float)v1c/(float)param.match_binsize),v_bin_num-1);
      int32_t stat_bin = v_bin*u_bin_num+u_bin;

      // match in circle
      findMatch(m1c,i1c,m1p,step_size,k1p,u_bin_num,v_bin_num,stat_bin,i1p, 0,true ,use_prior,Tr_delta);
      findMatch(m1p,i1p,m2p,step_size,k2p,u_bin_num,v_bin_num,stat_bin,i2p, 1,false,use_prior,Tr_delta);
      findMatch(m2p,i2p,m2c,step_size,k2c,u_bin_num,v_bin_num,stat_bin,i2c, 2,true ,use_prior,Tr_delta);
      findMatch(m2c,i2c,m1c,step_size,k1c,u_bin_num,v_bin_num,stat_bin,i1c2,3,false,use_prior,Tr_delta);
      
      // circle closure success?
      if (i1c2==i1c) {

        // extract coordinates
        u1p = *(m1p+step_size*i1p+0); v1p = *(m1p+step_size*i1p+1);
        u2p = *(m2p+step_size*i2p+0); v2p = *(m2p+step_size*i2p+1);
        u2c = *(m2c+step_size*i2c+0); v2c = *(m2c+step_size*i2c+1);

        // if disparities are positive
        if (u1p>=u2p && u1c>=u2c) {
          
          // add match if this pixel isn't matched yet
          if (*(M+getAddressOffsetImage(u1c,v1c,dims_c[0]))==0) {
            p_matched.push_back(p_match(u1p,v1p,i1p,u2p,v2p,i2p,
                                                 u1c,v1c,i1c,u2c,v2c,i2c));
            *(M+getAddressOffsetImage(u1c,v1c,dims_c[0])) = 1;
          }
          
        }
      }
    }
    */
  }

  // free memory
  free(M);
  delete []k1p;
  delete []k2p;
  delete []k1c;
  delete []k2c;
}

void Matcher::removeOutliers (vector<p_match> &p_matched,int32_t method) {
  
  // do we have enough points for outlier removal?
  if (p_matched.size()<=3)
    return;

  // input/output structure for triangulation
  struct triangulateio in, out;

  // inputs
  in.numberofpoints = p_matched.size();
  in.pointlist = (float*)malloc(in.numberofpoints*2*sizeof(float));
  int32_t k=0;
  
  // create copy of p_matched, init vector with number of support points
  // and fill triangle point vector for delaunay triangulation
  vector<p_match> p_matched_copy;  
  vector<int32_t> num_support;
  for (vector<p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {
    p_matched_copy.push_back(*it);
    num_support.push_back(0);
    in.pointlist[k++] = it->u1c;
    in.pointlist[k++] = it->v1c;
  }
  
  // input parameters
  in.numberofpointattributes = 0;
  in.pointattributelist      = NULL;
  in.pointmarkerlist         = NULL;
  in.numberofsegments        = 0;
  in.numberofholes           = 0;
  in.numberofregions         = 0;
  in.regionlist              = NULL;
  
  // outputs
  out.pointlist              = NULL;
  out.pointattributelist     = NULL;
  out.pointmarkerlist        = NULL;
  out.trianglelist           = NULL;
  out.triangleattributelist  = NULL;
  out.neighborlist           = NULL;
  out.segmentlist            = NULL;
  out.segmentmarkerlist      = NULL;
  out.edgelist               = NULL;
  out.edgemarkerlist         = NULL;

  // do triangulation (z=zero-based, n=neighbors, Q=quiet, B=no boundary markers)
  // attention: not using the B switch or using the n switch creates a memory leak (=> use valgrind!)
  char parameters[] = "zQB";
  triangulate(parameters, &in, &out, NULL);
  
  // for all triangles do
  for (int32_t i=0; i<out.numberoftriangles; i++) {
    
    // extract triangle corner points
    int32_t p1 = out.trianglelist[i*3+0];
    int32_t p2 = out.trianglelist[i*3+1];
    int32_t p3 = out.trianglelist[i*3+2];
    
    // method: flow
    if (method==0) {
      
      // 1. corner disparity and flow
      float p1_flow_u = p_matched_copy[p1].u1c-p_matched_copy[p1].u1p;
      float p1_flow_v = p_matched_copy[p1].v1c-p_matched_copy[p1].v1p;

      // 2. corner disparity and flow
      float p2_flow_u = p_matched_copy[p2].u1c-p_matched_copy[p2].u1p;
      float p2_flow_v = p_matched_copy[p2].v1c-p_matched_copy[p2].v1p;

      // 3. corner disparity and flow
      float p3_flow_u = p_matched_copy[p3].u1c-p_matched_copy[p3].u1p;
      float p3_flow_v = p_matched_copy[p3].v1c-p_matched_copy[p3].v1p;

      // consistency of 1. edge
      if (fabs(p1_flow_u-p2_flow_u)+fabs(p1_flow_v-p2_flow_v)<param.outlier_flow_tolerance) {
        num_support[p1]++;
        num_support[p2]++;
      }

      // consistency of 2. edge
      if (fabs(p2_flow_u-p3_flow_u)+fabs(p2_flow_v-p3_flow_v)<param.outlier_flow_tolerance) {
        num_support[p2]++;
        num_support[p3]++;
      }

      // consistency of 3. edge
      if (fabs(p1_flow_u-p3_flow_u)+fabs(p1_flow_v-p3_flow_v)<param.outlier_flow_tolerance) {
        num_support[p1]++;
        num_support[p3]++;
      }
      
    // method: stereo
    } else if (method==1) {
      
      // 1. corner disparity and flow
      float p1_disp   = p_matched_copy[p1].u1c-p_matched_copy[p1].u2c;

      // 2. corner disparity and flow
      float p2_disp   = p_matched_copy[p2].u1c-p_matched_copy[p2].u2c;

      // 3. corner disparity and flow
      float p3_disp   = p_matched_copy[p3].u1c-p_matched_copy[p3].u2c;

      // consistency of 1. edge
      if (fabs(p1_disp-p2_disp)<param.outlier_disp_tolerance) {
        num_support[p1]++;
        num_support[p2]++;
      }

      // consistency of 2. edge
      if (fabs(p2_disp-p3_disp)<param.outlier_disp_tolerance) {
        num_support[p2]++;
        num_support[p3]++;
      }

      // consistency of 3. edge
      if (fabs(p1_disp-p3_disp)<param.outlier_disp_tolerance) {
        num_support[p1]++;
        num_support[p3]++;
      }
      
    // method: quad matching
    } else {
      
      // 1. corner disparity and flow
      float p1_flow_u = p_matched_copy[p1].u1c-p_matched_copy[p1].u1p;
      float p1_flow_v = p_matched_copy[p1].v1c-p_matched_copy[p1].v1p;
      float p1_disp   = p_matched_copy[p1].u1p-p_matched_copy[p1].u2p;

      // 2. corner disparity and flow
      float p2_flow_u = p_matched_copy[p2].u1c-p_matched_copy[p2].u1p;
      float p2_flow_v = p_matched_copy[p2].v1c-p_matched_copy[p2].v1p;
      float p2_disp   = p_matched_copy[p2].u1p-p_matched_copy[p2].u2p;

      // 3. corner disparity and flow
      float p3_flow_u = p_matched_copy[p3].u1c-p_matched_copy[p3].u1p;
      float p3_flow_v = p_matched_copy[p3].v1c-p_matched_copy[p3].v1p;
      float p3_disp   = p_matched_copy[p3].u1p-p_matched_copy[p3].u2p;

      // consistency of 1. edge
      if (fabs(p1_disp-p2_disp)<param.outlier_disp_tolerance && fabs(p1_flow_u-p2_flow_u)+fabs(p1_flow_v-p2_flow_v)<param.outlier_flow_tolerance) {
        num_support[p1]++;
        num_support[p2]++;
      }

      // consistency of 2. edge
      if (fabs(p2_disp-p3_disp)<param.outlier_disp_tolerance && fabs(p2_flow_u-p3_flow_u)+fabs(p2_flow_v-p3_flow_v)<param.outlier_flow_tolerance) {
        num_support[p2]++;
        num_support[p3]++;
      }

      // consistency of 3. edge
      if (fabs(p1_disp-p3_disp)<param.outlier_disp_tolerance && fabs(p1_flow_u-p3_flow_u)+fabs(p1_flow_v-p3_flow_v)<param.outlier_flow_tolerance) {
        num_support[p1]++;
        num_support[p3]++;
      }
    }
  }
  
  // refill p_matched
  p_matched.clear();
  for (int i=0; i<in.numberofpoints; i++)
    if (num_support[i]>=4)
      p_matched.push_back(p_matched_copy[i]);
  
  // free memory used for triangulation
  free(in.pointlist);
  free(out.pointlist);
  free(out.trianglelist);
}

bool Matcher::parabolicFitting(const uint8_t* I1_du,const uint8_t* I1_dv,const int32_t* dims1,
                               const uint8_t* I2_du,const uint8_t* I2_dv,const int32_t* dims2,
                               const float &u1,const float &v1,
                               float       &u2,float       &v2,
                               Matrix At,Matrix AtA,
                               uint8_t* desc_buffer) {

  // check if parabolic fitting is feasible (descriptors are within margin)
  if (u2-3<margin || u2+3>dims2[0]-1-margin || v2-3<margin || v2+3>dims2[1]-1-margin)
    return false;
  
  // compute reference descriptor
  __m128i xmm1,xmm2;
  computeSmallDescriptor(I1_du,I1_dv,dims1[2],(int32_t)u1,(int32_t)v1,desc_buffer);
  xmm1 = _mm_load_si128((__m128i*)(desc_buffer));
  
  // compute cost matrix
  int32_t cost[49];
  for (int32_t dv=0; dv<7; dv++) {
    for (int32_t du=0; du<7; du++) {
      computeSmallDescriptor(I2_du,I2_dv,dims2[2],(int32_t)u2+du-3,(int32_t)v2+dv-3,desc_buffer);
      xmm2 = _mm_load_si128((__m128i*)(desc_buffer));
      xmm2 = _mm_sad_epu8(xmm1,xmm2);
      cost[dv*7+du] = _mm_extract_epi16(xmm2,0)+_mm_extract_epi16(xmm2,4);
    }
  }
  
  // compute minimum
  int32_t min_ind  = 0;
  int32_t min_cost = cost[0];
  for (int32_t i=1; i<49; i++) {
    if (cost[i]<min_cost) {
      min_ind   = i;
      min_cost  = cost[i];
    }
  }
  
  // get indices
  int32_t du = min_ind%7;
  int32_t dv = min_ind/7;
  
  // if minimum is at borders => remove this match
  if (du==0 || du==6 || dv==0 || dv==6)
    return false;
  
  // solve least squares system
  Matrix c(9,1);
  for (int32_t i=-1; i<=+1; i++) {
    for (int32_t j=-1; j<=+1; j++) {
      int32_t cost_curr = cost[(dv+i)*7+(du+j)];
      // if (i!=0 && j!=0 && cost_curr<=min_cost+150)
        // return false;
      c.val[(i+1)*3+(j+1)][0] = cost_curr;
    }
  }
  Matrix b = At*c;
  if (!b.solve(AtA))
    return false;
  
  // extract relative coordinates
  float divisor = (b.val[2][0]*b.val[2][0]-4.0*b.val[0][0]*b.val[1][0]);
  if (fabs(divisor)<1e-8 || fabs(b.val[2][0])<1e-8)
    return false;
  float ddv = (2.0*b.val[0][0]*b.val[4][0]-b.val[2][0]*b.val[3][0])/divisor;
  float ddu = -(b.val[4][0]+2.0*b.val[1][0]*ddv)/b.val[2][0];
  if (fabs(ddu)>=1.0 || fabs(ddv)>=1.0)
    return false;
  
  // update target
  u2 += (float)du-3.0+ddu;
  v2 += (float)dv-3.0+ddv;
  
  // return true on success
  return true;
}

void Matcher::relocateMinimum(const uint8_t* I1_du,const uint8_t* I1_dv,const int32_t* dims1,
                              const uint8_t* I2_du,const uint8_t* I2_dv,const int32_t* dims2,
                              const float &u1,const float &v1,
                              float       &u2,float       &v2,
                              uint8_t* desc_buffer) {

  // check if parabolic fitting is feasible (descriptors are within margin)
  if (u2-2<margin || u2+2>dims2[0]-1-margin || v2-2<margin || v2+2>dims2[1]-1-margin)
    return;
  
  // compute reference descriptor
  __m128i xmm1,xmm2;
  computeSmallDescriptor(I1_du,I1_dv,dims1[2],(int32_t)u1,(int32_t)v1,desc_buffer);
  xmm1 = _mm_load_si128((__m128i*)(desc_buffer));
  
  // compute cost matrix
  int32_t cost[25];
  for (int32_t dv=0; dv<5; dv++) {
    for (int32_t du=0; du<5; du++) {
      computeSmallDescriptor(I2_du,I2_dv,dims2[2],(int32_t)u2+du-2,(int32_t)v2+dv-2,desc_buffer);
      xmm2 = _mm_load_si128((__m128i*)(desc_buffer));
      xmm2 = _mm_sad_epu8(xmm1,xmm2);
      cost[dv*5+du] = _mm_extract_epi16(xmm2,0)+_mm_extract_epi16(xmm2,4);
    }
  }
  
  // compute minimum
  int32_t min_ind  = 0;
  int32_t min_cost = cost[0];
  for (int32_t i=1; i<25; i++) {
    if (cost[i]<min_cost) {
      min_ind   = i;
      min_cost  = cost[i];
    }
  }
  
  // update target
  u2 += (float)(min_ind%5)-2.0;
  v2 += (float)(min_ind/5)-2.0;
}

void Matcher::refinement (vector<p_match> &p_matched,int32_t method) {
  
  // allocate aligned memory (32 bytes for 1 descriptors)
  uint8_t* desc_buffer = (uint8_t*)_mm_malloc(32*sizeof(uint8_t),16);
  
  // copy vector (for refill)
  vector<p_match> p_matched_copy = p_matched;
  p_matched.clear();
  
  // create matrices for least square fitting
  FLOAT A_data[9*6] = { 1, 1, 1,-1,-1, 1,
                        0, 1, 0, 0,-1, 1,
                        1, 1,-1, 1,-1, 1,
                        1, 0, 0,-1, 0, 1,
                        0, 0, 0, 0, 0, 1,
                        1, 0, 0, 1, 0, 1,
                        1, 1,-1,-1, 1, 1,
                        0, 1, 0, 0, 1, 1,
                        1, 1, 1, 1, 1, 1};
  Matrix A(9,6,A_data);
  Matrix At  = ~A;
  Matrix AtA = At*A;
  
  uint8_t* I1p_du_fit = I1p_du;
  uint8_t* I1p_dv_fit = I1p_dv;
  uint8_t* I2p_du_fit = I2p_du;
  uint8_t* I2p_dv_fit = I2p_dv;
  uint8_t* I1c_du_fit = I1c_du;
  uint8_t* I1c_dv_fit = I1c_dv;
  uint8_t* I2c_du_fit = I2c_du;
  uint8_t* I2c_dv_fit = I2c_dv;
  if (param.half_resolution) {
    I1p_du_fit = I1p_du_full;
    I1p_dv_fit = I1p_dv_full;
    I2p_du_fit = I2p_du_full;
    I2p_dv_fit = I2p_dv_full;
    I1c_du_fit = I1c_du_full;
    I1c_dv_fit = I1c_dv_full;
    I2c_du_fit = I2c_du_full;
    I2c_dv_fit = I2c_dv_full;
  }
  
  // for all matches do
  for (vector<p_match>::iterator it=p_matched_copy.begin(); it!=p_matched_copy.end(); it++) {
    
    // method: flow or quad matching
    if (method==0 || method==2) {
      if (param.refinement==2) {
        if (!parabolicFitting(I1c_du_fit,I1c_dv_fit,dims_c,I1p_du_fit,I1p_dv_fit,dims_p,
                              it->u1c,it->v1c,it->u1p,it->v1p,At,AtA,desc_buffer))
          continue;
      } else {
        relocateMinimum(I1c_du_fit,I1c_dv_fit,dims_c,I1p_du_fit,I1p_dv_fit,dims_p,
                        it->u1c,it->v1c,it->u1p,it->v1p,desc_buffer);
      }
    }
    
    // method: stereo or quad matching
    if (method==1 || method==2) {
      if (param.refinement==2) {
        if (!parabolicFitting(I1c_du_fit,I1c_dv_fit,dims_c,I2c_du_fit,I2c_dv_fit,dims_c,
                              it->u1c,it->v1c,it->u2c,it->v2c,At,AtA,desc_buffer))
          continue;
      } else {
        relocateMinimum(I1c_du_fit,I1c_dv_fit,dims_c,I2c_du_fit,I2c_dv_fit,dims_c,
                        it->u1c,it->v1c,it->u2c,it->v2c,desc_buffer);
      }
    }
    
    // method: quad matching
    if (method==2) {
      if (param.refinement==2) {
        if (!parabolicFitting(I1c_du_fit,I1c_dv_fit,dims_c,I2p_du_fit,I2p_dv_fit,dims_p,
                              it->u1c,it->v1c,it->u2p,it->v2p,At,AtA,desc_buffer))
          continue;
      } else {
        relocateMinimum(I1c_du_fit,I1c_dv_fit,dims_c,I2p_du_fit,I2p_dv_fit,dims_p,
                        it->u1c,it->v1c,it->u2p,it->v2p,desc_buffer);
      }
    }

    // add this match
    p_matched.push_back(*it);
  }
  
  // free memory
  _mm_free(desc_buffer);
}

float Matcher::mean(const uint8_t* I,const int32_t &bpl,const int32_t &u_min,const int32_t &u_max,const int32_t &v_min,const int32_t &v_max) {
  float mean = 0;
  for (int32_t v=v_min; v<=v_max; v++)
    for (int32_t u=u_min; u<=u_max; u++)
      mean += (float)*(I+getAddressOffsetImage(u,v,bpl));
  return
    mean /= (float)((u_max-u_min+1)*(v_max-v_min+1));
}
}
