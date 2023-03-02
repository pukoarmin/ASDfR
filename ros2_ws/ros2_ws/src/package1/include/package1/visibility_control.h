#ifndef IMAGE_TOOLS__VISIBILITY_CONTROL_H_
#define IMAGE_TOOLS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IMAGE_TOOLS_EXPORT __attribute__ ((dllexport))
    #define IMAGE_TOOLS_IMPORT __attribute__ ((dllimport))
  #else
    #define IMAGE_TOOLS_EXPORT __declspec(dllexport)
    #define IMAGE_TOOLS_IMPORT __declspec(dllimport)
  #endif
  #ifdef IMAGE_TOOLS_BUILDING_DLL
    #define IMAGE_TOOLS_PUBLIC IMAGE_TOOLS_EXPORT
  #else
    #define IMAGE_TOOLS_PUBLIC IMAGE_TOOLS_IMPORT
  #endif
  #define IMAGE_TOOLS_PUBLIC_TYPE IMAGE_TOOLS_PUBLIC
  #define IMAGE_TOOLS_LOCAL
#else
  #define IMAGE_TOOLS_EXPORT __attribute__ ((visibility("default")))
  #define IMAGE_TOOLS_IMPORT
  #if __GNUC__ >= 4
    #define IMAGE_TOOLS_PUBLIC __attribute__ ((visibility("default")))
    #define IMAGE_TOOLS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IMAGE_TOOLS_PUBLIC
    #define IMAGE_TOOLS_LOCAL
  #endif
  #define IMAGE_TOOLS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // IMAGE_TOOLS__VISIBILITY_CONTROL_H_