#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NAV_DRONE_EXPORT __attribute__ ((dllexport))
    #define NAV_DRONE_IMPORT __attribute__ ((dllimport))
  #else
    #define NAV_DRONE_EXPORT __declspec(dllexport)
    #define NAV_DRONE_IMPORT __declspec(dllimport)
  #endif
  #ifdef NAV_DRONE_BUILDING_DLL
    #define NAV_DRONE_PUBLIC NAV_DRONE_EXPORT
  #else
    #define NAV_DRONE_PUBLIC NAV_DRONE_IMPORT
  #endif
  #define NAV_DRONE_PUBLIC_TYPE NAV_DRONE_PUBLIC
  #define NAV_DRONE_LOCAL
#else
  #define NAV_DRONE_EXPORT __attribute__ ((visibility("default")))
  #define NAV_DRONE_IMPORT
  #if __GNUC__ >= 4
    #define NAV_DRONE_PUBLIC __attribute__ ((visibility("default")))
    #define NAV_DRONE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NAV_DRONE_PUBLIC
    #define NAV_DRONE_LOCAL
  #endif
  #define NAV_DRONE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif
