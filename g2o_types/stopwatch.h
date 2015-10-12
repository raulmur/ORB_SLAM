// This file is part of VisionTools.
//
// Copyright 2011 Hauke Strasdat (Imperial College London)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights  to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#ifndef VISIONTOOLS_STOP_WATCH_H
#define VISIONTOOLS_STOP_WATCH_H

#include <cassert>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

#include <stdexcept>


namespace VisionTools
{

class   StopWatch
{
public:
  StopWatch()
  {
    running_ = false;
    time_ = 0;
  }

  void start()
  {
    assert(running_==false);
    gettimeofday(&start_time_, NULL);
    running_ = true;
  }

  void stop()
  {
    assert(running_);
    gettimeofday(&end_time_, NULL);
    long seconds  = end_time_.tv_sec  - start_time_.tv_sec;
    long useconds = end_time_.tv_usec - start_time_.tv_usec;
    time_ = ((seconds) + useconds*0.000001);
    running_ = false;
  }

  double read_current_time()
  {
    assert(running_);
    timeval cur_time;
    gettimeofday(&cur_time, NULL);
    long seconds  = cur_time.tv_sec  - start_time_.tv_sec;
    long useconds = cur_time.tv_usec - start_time_.tv_usec;
    return ((seconds) + useconds*0.000001);
  }

  double get_stopped_time()
  {
    assert(running_==false);
    return time_;
  }

  inline void reset()
  {
    time_ = 0;
  }

private:
  timeval start_time_;
  timeval end_time_;
  double time_;
  bool running_;
};
}

#endif
