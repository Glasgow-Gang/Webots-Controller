#pragma once

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b)                                                              \
  ({                                                                           \
    __typeof__(a) _a = (a);                                                    \
    __typeof__(b) _b = (b);                                                    \
    _a < _b ? _a : _b;                                                         \
  })
#endif

#ifndef DEF2STR
#define _XR_TO_STR(_arg) #_arg
#define DEF2STR(_arg) _XR_TO_STR(_arg)
#endif

#ifndef UNUSED
#define UNUSED(_x) ((void)(_x))
#endif

#ifndef OFFSET_OF
#define OFFSET_OF(type, member) ((size_t) & ((type *)0)->member)
#endif

#ifndef MEMBER_SIZE_OF
#define MEMBER_SIZE_OF(type, member) (sizeof(typeof(((type *)0)->member)))
#endif

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, member)                                        \
  ({                                                                           \
    const typeof(((type *)0)->member) *__mptr = (ptr);                         \
    (type *)((char *)__mptr - ms_offset_of(type, member));                     \
  })
#endif

enum class ErrorCode {
  OK = 0,
  FAILED = -1,
  INIT_ERR = -2,
  ARG_ERR = -3,
  STATE_ERR = -4,
  SIZE_ERR = -5,
  CHECK_ERR = -6,
  NOT_SUPPORT = -7,
  NOT_FOUND = -8,
  NO_REPONSE = -9,
  NO_MEM = -10,
  NO_BUFF = -11,
  TIMEOUT = -12,
  EMPTY = -13,
  FULL = -14,
  BUSY = -15,
  PTR_NULL = -16,
};

enum class SizeLimitMode { EQUAL = 0, LESS = 1, MORE = 2, NONE = 3 };

#ifdef ASSERT
#undef ASSERT
#endif

#ifdef LIBXR_DEBUG_BUILD
#define ASSERT(arg)                                                            \
  if (!(arg)) {                                                                \
    LibXR::Assert::FatalError(__FILE__, __LINE__);                             \
  }
#else
#define ASSERT(arg) (void)0;
#endif
