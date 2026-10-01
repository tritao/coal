// Stands in for the header coal's CMake (jrl-cmakemodules) generates.
#ifndef COAL_DEPRECATED_HH
#define COAL_DEPRECATED_HH

#define COAL_DEPRECATED [[deprecated]]
#define COAL_DEPRECATED_MESSAGE(...) [[deprecated(#__VA_ARGS__)]]

#endif  // COAL_DEPRECATED_HH
