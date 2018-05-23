#ifndef _FILE_HANDLER_HPP
#define _FILE_HANDLER_HPP

#include <time.h>
#include <string>

std::string getTimeStamp();


std::string getTimeStamp() {
  time_t now;
  struct tm *time_now;

  time(&now);
  time_now = localtime(&now);

  int year = time_now->tm_year + 1900;
  int month = time_now->tm_mon;
  int day  = time_now->tm_mday;
  int hour = time_now->tm_hour;
  int minute = time_now->tm_min;
  int second = time_now->tm_sec;

  char str[25];
  sprintf(str, "%d_%d_%d_%d_%d_%d", year, month, day, hour, minute, second);

  std::string time_stamp = str;

  return time_stamp;
}

#endif // ifndef _FILE_HANDLER_HPP
