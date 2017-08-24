/*
 * vision.cpp
 *
 *  Created on: May 14, 2013
 *      Author: cforster
 */

#include <vikit/vision.h>

#if __SSE2__
# include <emmintrin.h>
#elif __ARM_NEON__
# include <arm_neon.h>
#endif

namespace vk {

#ifdef __SSE2__
void halfSampleSSE2(const unsigned char* in, unsigned char* out, int w, int h)
{
  const unsigned long long mask[2] = {0x00FF00FF00FF00FFull, 0x00FF00FF00FF00FFull};
  const unsigned char* nextRow = in + w;
  __m128i m = _mm_loadu_si128((const __m128i*)mask);
  int sw = w >> 4;
  int sh = h >> 1;
  for (int i=0; i<sh; i++)
  {
    for (int j=0; j<sw; j++)
    {
      __m128i here = _mm_load_si128((const __m128i*)in);
      __m128i next = _mm_load_si128((const __m128i*)nextRow);
      here = _mm_avg_epu8(here,next);
      next = _mm_and_si128(_mm_srli_si128(here,1), m);
      here = _mm_and_si128(here,m);
      here = _mm_avg_epu16(here, next);
      _mm_storel_epi64((__m128i*)out, _mm_packus_epi16(here,here));
      in += 16;
      nextRow += 16;
      out += 8;
    }
    in += w;
    nextRow += w;
  }
}
#endif 

#ifdef __ARM_NEON__
void halfSampleNEON( const cv::Mat& in, cv::Mat& out )
{
  for( int y = 0; y < in.rows; y += 2)
  {
    const uint8_t * in_top = in.data + y*in.cols;
    const uint8_t * in_bottom = in.data + (y+1)*in.cols;
    uint8_t * out_data = out.data + (y >> 1)*out.cols;
    for( int x = in.cols; x > 0 ; x-=16, in_top += 16, in_bottom += 16, out_data += 8)
    {
      uint8x8x2_t top  = vld2_u8( (const uint8_t *)in_top );
      uint8x8x2_t bottom = vld2_u8( (const uint8_t *)in_bottom );
      uint16x8_t sum = vaddl_u8( top.val[0], top.val[1] );
      sum = vaddw_u8( sum, bottom.val[0] );
      sum = vaddw_u8( sum, bottom.val[1] );
      uint8x8_t final_sum = vshrn_n_u16(sum, 2);
      vst1_u8(out_data, final_sum);
    }
  }
}
#endif


void
halfSample(const cv::Mat& in, cv::Mat& out)
{
  assert( in.rows/2==out.rows && in.cols/2==out.cols);
  assert( in.type()==CV_8U && out.type()==CV_8U);

#ifdef __SSE2__
  if(aligned_mem::is_aligned16(in.data) && aligned_mem::is_aligned16(out.data) && ((in.cols % 16) == 0))
  {
    halfSampleSSE2(in.data, out.data, in.cols, in.rows);
    return;
  }
#endif 
#ifdef __ARM_NEON__ 
  if( (in.cols % 16) == 0 )
  {
    halfSampleNEON(in, out);
    return;
  }
#endif

  const int stride = in.step.p[0];
  uint8_t* top = (uint8_t*) in.data;
  uint8_t* bottom = top + stride;
  uint8_t* end = top + stride*in.rows;
  const int out_width = out.cols;
  uint8_t* p = (uint8_t*) out.data;
  while (bottom < end)
  {
    for (int j=0; j<out_width; j++)
    {
      *p = static_cast<uint8_t>( (uint16_t (top[0]) + top[1] + bottom[0] + bottom[1])/4 );
      p++;
      top += 2;
      bottom += 2;
    }
    top += stride;
    bottom += stride;
  }
}


float
shiTomasiScore(const cv::Mat& img, int u, int v)
{
  assert(img.type() == CV_8UC1);

  float dXX = 0.0;
  float dYY = 0.0;
  float dXY = 0.0;
  const int halfbox_size = 4;
  const int box_size = 2*halfbox_size;
  const int box_area = box_size*box_size;
  const int x_min = u-halfbox_size;
  const int x_max = u+halfbox_size;
  const int y_min = v-halfbox_size;
  const int y_max = v+halfbox_size;

  if(x_min < 1 || x_max >= img.cols-1 || y_min < 1 || y_max >= img.rows-1)
    return 0.0; // patch is too close to the boundary

  const int stride = img.step.p[0];
  for( int y=y_min; y<y_max; ++y )
  {
    const uint8_t* ptr_left   = img.data + stride*y + x_min - 1;
    const uint8_t* ptr_right  = img.data + stride*y + x_min + 1;
    const uint8_t* ptr_top    = img.data + stride*(y-1) + x_min;
    const uint8_t* ptr_bottom = img.data + stride*(y+1) + x_min;
    for(int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom)
    {
      float dx = *ptr_right - *ptr_left;
      float dy = *ptr_bottom - *ptr_top;
      dXX += dx*dx;
      dYY += dy*dy;
      dXY += dx*dy;
    }
  }

  // Find and return smaller eigenvalue:
  dXX = dXX / (2.0 * box_area);
  dYY = dYY / (2.0 * box_area);
  dXY = dXY / (2.0 * box_area);
  return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));
}


void
calcSharrDeriv(const cv::Mat& src, cv::Mat& dst)
{

  using namespace cv;
  typedef short deriv_type;

  int rows = src.rows, cols = src.cols, cn = src.channels(), colsn = cols*cn, depth = src.depth();
  CV_Assert(depth == CV_8U);
  dst.create(rows, cols, CV_MAKETYPE(DataType<deriv_type>::depth, cn*2));

  int x, y, delta = (int)alignSize((cols + 2)*cn, 16);
  AutoBuffer<deriv_type> _tempBuf(delta*2 + 64);
  deriv_type *trow0 = alignPtr(_tempBuf + cn, 16), *trow1 = alignPtr(trow0 + delta, 16);

#ifdef __SSE2__
  __m128i z = _mm_setzero_si128(), c3 = _mm_set1_epi16(3), c10 = _mm_set1_epi16(10);
#endif

  for( y = 0; y < rows; y++ )
  {
      const uchar* srow0 = src.ptr<uchar>(y > 0 ? y-1 : rows > 1 ? 1 : 0);
      const uchar* srow1 = src.ptr<uchar>(y);
      const uchar* srow2 = src.ptr<uchar>(y < rows-1 ? y+1 : rows > 1 ? rows-2 : 0);
      deriv_type* drow = dst.ptr<deriv_type>(y);

      // do vertical convolution
      x = 0;
#ifdef __SSE2__
      for( ; x <= colsn - 8; x += 8 )
      {
          __m128i s0 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i*)(srow0 + x)), z);
          __m128i s1 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i*)(srow1 + x)), z);
          __m128i s2 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i*)(srow2 + x)), z);
          __m128i t0 = _mm_add_epi16(_mm_mullo_epi16(_mm_add_epi16(s0, s2), c3), _mm_mullo_epi16(s1, c10));
          __m128i t1 = _mm_sub_epi16(s2, s0);
          _mm_store_si128((__m128i*)(trow0 + x), t0);
          _mm_store_si128((__m128i*)(trow1 + x), t1);
      }
#endif
      for( ; x < colsn; x++ )
      {
          int t0 = (srow0[x] + srow2[x])*3 + srow1[x]*10;
          int t1 = srow2[x] - srow0[x];
          trow0[x] = (deriv_type)t0;
          trow1[x] = (deriv_type)t1;
      }

      // make border
      int x0 = (cols > 1 ? 1 : 0)*cn, x1 = (cols > 1 ? cols-2 : 0)*cn;
      for( int k = 0; k < cn; k++ )
      {
          trow0[-cn + k] = trow0[x0 + k]; trow0[colsn + k] = trow0[x1 + k];
          trow1[-cn + k] = trow1[x0 + k]; trow1[colsn + k] = trow1[x1 + k];
      }

      // do horizontal convolution, interleave the results and store them to dst
      x = 0;
#ifdef __SSE2__
      for( ; x <= colsn - 8; x += 8 )
      {
          __m128i s0 = _mm_loadu_si128((const __m128i*)(trow0 + x - cn));
          __m128i s1 = _mm_loadu_si128((const __m128i*)(trow0 + x + cn));
          __m128i s2 = _mm_loadu_si128((const __m128i*)(trow1 + x - cn));
          __m128i s3 = _mm_load_si128((const __m128i*)(trow1 + x));
          __m128i s4 = _mm_loadu_si128((const __m128i*)(trow1 + x + cn));

          __m128i t0 = _mm_sub_epi16(s1, s0);
          __m128i t1 = _mm_add_epi16(_mm_mullo_epi16(_mm_add_epi16(s2, s4), c3), _mm_mullo_epi16(s3, c10));
          __m128i t2 = _mm_unpacklo_epi16(t0, t1);
          t0 = _mm_unpackhi_epi16(t0, t1);
          // this can probably be replaced with aligned stores if we aligned dst properly.
          _mm_storeu_si128((__m128i*)(drow + x*2), t2);
          _mm_storeu_si128((__m128i*)(drow + x*2 + 8), t0);
      }
#endif
      for( ; x < colsn; x++ )
      {
          deriv_type t0 = (deriv_type)(trow0[x+cn] - trow0[x-cn]);
          deriv_type t1 = (deriv_type)((trow1[x+cn] + trow1[x-cn])*3 + trow1[x]*10);
          drow[x*2] = t0; drow[x*2+1] = t1;
      }
  }

//  vector<cv::Mat> vec_mat;
//  cv::split(dst, vec_mat);
//  cv::namedWindow("deriv");
//  cv::imshow("deriv", vec_mat[0]);
//  cv::namedWindow("derivy");
//  cv::imshow("derivy", vec_mat[1]);
//  cv::waitKey(0);
}

#ifdef __SSE2__
void convertRawDepthImageSse_16u_to_32f(cv::Mat& depth_16u, cv::Mat& depth_32f, float scale)
{
  depth_32f.create(depth_16u.rows, depth_16u.cols, CV_32FC1);

  const unsigned short* input_ptr = depth_16u.ptr<unsigned short>();
  float* output_ptr = depth_32f.ptr<float>();

  __m128 _scale = _mm_set1_ps(scale);
  __m128 _zero  = _mm_setzero_ps();
  __m128 _nan   = _mm_set1_ps(std::numeric_limits<float>::quiet_NaN());

  for(int idx = 0; idx < depth_16u.size().area(); idx += 8, input_ptr += 8, output_ptr += 8)
  {
    __m128 _input, mask;
    __m128i _inputi = _mm_load_si128((__m128i*) input_ptr);

    // load low shorts and convert to float
    _input = _mm_cvtepi32_ps(_mm_unpacklo_epi16(_inputi, _mm_setzero_si128()));

    mask = _mm_cmpeq_ps(_input, _zero);

    // zero to nan
    _input = _mm_or_ps(_input, _mm_and_ps(mask, _nan));
    // scale
    _input = _mm_mul_ps(_input, _scale);
    // save
    _mm_store_ps(output_ptr + 0, _input);

    // load high shorts and convert to float
    _input = _mm_cvtepi32_ps(_mm_unpackhi_epi16(_inputi, _mm_setzero_si128()));

    mask = _mm_cmpeq_ps(_input, _zero);

    // zero to nan
    _input = _mm_or_ps(_input, _mm_and_ps(mask, _nan));
    // scale
    _input = _mm_mul_ps(_input, _scale);
    // save
    _mm_store_ps(output_ptr + 4, _input);
  }
}
#endif

}


