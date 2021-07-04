// include guard instead of pragma once: workaround for g++ bug
#ifndef __ICFPC2021_STDAFX_H__
#define __ICFPC2021_STDAFX_H__

#ifdef _MSC_VER
#define CPPHTTPLIB_OPENSSL_SUPPORT
#endif 

#include <httplib.h>  // include before <windows.h>

#ifdef _MSC_VER
#  define NOMINMAX
#  include <windows.h>
#  undef ERROR
#else
#  include <time.h>
#  include <sys/time.h>
#endif

#include "glog/logging.h"

#endif // __ICFPC2021_STDAFX_H__
