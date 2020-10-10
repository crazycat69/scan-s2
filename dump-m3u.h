#ifndef __DUMP_M3U_H__
#define __DUMP_M3U_H__

#include <stdint.h>

#include "scan.h"

extern void m3u_dump_service_parameter_set (FILE *f, service_t *s, transponder_t *t, unsigned char *url);

#endif
