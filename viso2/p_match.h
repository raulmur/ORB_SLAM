#ifndef P_MATCH_H
#define P_MATCH_H
// define fixed-width datatypes for Visual Studio projects
#ifndef _MSC_VER
  #include <stdint.h>
#else
  typedef __int8            int8_t;
  typedef __int16           int16_t;
  typedef __int32           int32_t;
  typedef __int64           int64_t;
  typedef unsigned __int8   uint8_t;
  typedef unsigned __int16  uint16_t;
  typedef unsigned __int32  uint32_t;
  typedef unsigned __int64  uint64_t;
#endif
// structure for storing matches
struct p_match {
  float   u1p,v1p; // u,v-coordinates in previous left  image
  int32_t i1p;     // feature index (for tracking)
  float   u2p,v2p; // u,v-coordinates in previous right image
  int32_t i2p;     // feature index (for tracking)
  float   u1c,v1c; // u,v-coordinates in current  left  image
  int32_t i1c;     // feature index (for tracking)
  float   u2c,v2c; // u,v-coordinates in current  right image
  int32_t i2c;     // feature index (for tracking)
  p_match(){}
  p_match(float u1p,float v1p,int32_t i1p,float u2p,float v2p,int32_t i2p,
          float u1c,float v1c,int32_t i1c,float u2c,float v2c,int32_t i2c):
          u1p(u1p),v1p(v1p),i1p(i1p),u2p(u2p),v2p(v2p),i2p(i2p),
          u1c(u1c),v1c(v1c),i1c(i1c),u2c(u2c),v2c(v2c),i2c(i2c) {}
  p_match& operator =(const p_match& other)
  {
      if(this==&other)
          return *this;
      u1p=other.u1p;
      v1p=other.v1p;
      i1p=other.i1p;
      u2p=other.u2p;
      v2p=other.v2p;i2p=other.i2p;
      u1c=other.u1c;v1c=other.v1c;
      i1c=other.i1c;u2c=other.u2c;
      v2c=other.v2c;i2c=other.i2c;
      return *this;
  }
  p_match(const p_match & other):
      u1p(other.u1p),
      v1p(other.v1p),
      i1p(other.i1p),
      u2p(other.u2p),
      v2p(other.v2p),i2p(other.i2p),
      u1c(other.u1c),v1c(other.v1c),
      i1c(other.i1c),u2c(other.u2c),
      v2c(other.v2c),i2c(other.i2c)
  {
  }
};

#endif // P_MATCH_H
