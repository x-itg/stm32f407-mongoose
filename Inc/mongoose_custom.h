#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>//����lwipʱ

#define MG_ARCH MG_ARCH_NEW_LIB
#define MG_ENABLE_TCPIP 1
#define MG_ENABLE_CUSTOM_MILLIS 1
#define MG_ENABLE_FILE 0
#define MG_ENABLE_PACKED_FS 1