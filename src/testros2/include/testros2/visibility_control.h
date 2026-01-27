#ifndef TESTROS2__VISIBILITY_CONTROL_H_
#define TESTROS2__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TESTROS2_EXPORT __attribute__ ((dllexport))
    #define TESTROS2_IMPORT __attribute__ ((dllimport))
  #else
    #define TESTROS2_EXPORT __declspec(dllexport)
    #define TESTROS2_IMPORT __declspec(dllimport)
  #endif
  #ifdef TESTROS2_BUILDING_LIBRARY
    #define TESTROS2_PUBLIC TESTROS2_EXPORT
  #else
    #define TESTROS2_PUBLIC TESTROS2_IMPORT
  #endif
  #define TESTROS2_PUBLIC_TYPE TESTROS2_PUBLIC
  #define TESTROS2_LOCAL
#else
  #define TESTROS2_EXPORT __attribute__ ((visibility("default")))
  #define TESTROS2_IMPORT
  #if __GNUC__ >= 4
    #define TESTROS2_PUBLIC __attribute__ ((visibility("default")))
    #define TESTROS2_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TESTROS2_PUBLIC
    #define TESTROS2_LOCAL
  #endif
  #define TESTROS2_PUBLIC_TYPE
#endif
#endif  // TESTROS2__VISIBILITY_CONTROL_H_
// Generated 27-Jan-2026 15:36:27
 