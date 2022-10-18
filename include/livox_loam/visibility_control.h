#ifndef LIVOX_LOAM__VISIBILITY_CONTROL_H_
#define LIVOX_LOAM__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LIVOX_LOAM_EXPORT __attribute__ ((dllexport))
    #define LIVOX_LOAM_IMPORT __attribute__ ((dllimport))
  #else
    #define LIVOX_LOAM_EXPORT __declspec(dllexport)
    #define LIVOX_LOAM_IMPORT __declspec(dllimport)
  #endif
  #ifdef LIVOX_LOAM_BUILDING_LIBRARY
    #define LIVOX_LOAM_PUBLIC LIVOX_LOAM_EXPORT
  #else
    #define LIVOX_LOAM_PUBLIC LIVOX_LOAM_IMPORT
  #endif
  #define LIVOX_LOAM_PUBLIC_TYPE LIVOX_LOAM_PUBLIC
  #define LIVOX_LOAM_LOCAL
#else
  #define LIVOX_LOAM_EXPORT __attribute__ ((visibility("default")))
  #define LIVOX_LOAM_IMPORT
  #if __GNUC__ >= 4
    #define LIVOX_LOAM_PUBLIC __attribute__ ((visibility("default")))
    #define LIVOX_LOAM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LIVOX_LOAM_PUBLIC
    #define LIVOX_LOAM_LOCAL
  #endif
  #define LIVOX_LOAM_PUBLIC_TYPE
#endif

#endif  // LIVOX_LOAM__VISIBILITY_CONTROL_H_
