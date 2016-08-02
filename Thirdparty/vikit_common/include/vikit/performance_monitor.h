/*
 * performance_monitor.h
 *
 *  Created on: Aug 26, 2011
 *      Author: Christian Forster
 */

#ifndef VIKIT_PERFORMANCE_MONITOR_H
#define VIKIT_PERFORMANCE_MONITOR_H

#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <vikit/timer.h>

namespace vk
{

struct LogItem
{
  double data;
  bool   set;
};

class PerformanceMonitor
{
public:
  PerformanceMonitor();
  ~PerformanceMonitor();
  void init(const std::string& trace_name, const std::string& trace_dir);
  void addTimer(const std::string& name);
  void addLog(const std::string& name);
  void writeToFile();
  void startTimer(const std::string& name);
  void stopTimer(const std::string& name);
  double getTime(const std::string& name) const;
  void log(const std::string& name, double data);

private:
  std::map<std::string, Timer>      timers_;
  std::map<std::string, LogItem>    logs_;
  std::string                       trace_name_;        //<! name of the thread that started the performance monitor
  std::string                       trace_dir_;         //<! directory where the logfiles are saved
  std::ofstream                     ofs_;

  void trace();
  void traceHeader();
};

} // namespace vk

#endif // VIKIT_PERFORMANCE_MONITOR_H
