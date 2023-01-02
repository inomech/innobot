
#ifndef INNOBOT_CONTROL_HARDWARE__VISIBILITY_CONTROL_H_
#define INNOBOT_CONTROL_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define INNOBOT_CONTROL_HARDWARE_EXPORT __attribute__((dllexport))
#define INNOBOT_CONTROL_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define INNOBOT_CONTROL_HARDWARE_EXPORT __declspec(dllexport)
#define INNOBOT_CONTROL_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef INNOBOT_CONTROL_HARDWARE_BUILDING_DLL
#define INNOBOT_CONTROL_HARDWARE_PUBLIC INNOBOT_CONTROL_HARDWARE_EXPORT
#else
#define INNOBOT_CONTROL_HARDWARE_PUBLIC INNOBOT_CONTROL_HARDWARE_IMPORT
#endif
#define INNOBOT_CONTROL_HARDWARE_PUBLIC_TYPE INNOBOT_CONTROL_HARDWARE_PUBLIC
#define INNOBOT_CONTROL_HARDWARE_LOCAL
#else
#define INNOBOT_CONTROL_HARDWARE_EXPORT __attribute__((visibility("default")))
#define INNOBOT_CONTROL_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define INNOBOT_CONTROL_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define INNOBOT_CONTROL_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define INNOBOT_CONTROL_HARDWARE_PUBLIC
#define INNOBOT_CONTROL_HARDWARE_LOCAL
#endif
#define INNOBOT_CONTROL_HARDWARE_PUBLIC_TYPE
#endif

#endif  // INNOBOT_CONTROL_HARDWARE__VISIBILITY_CONTROL_H_