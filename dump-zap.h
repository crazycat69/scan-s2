#ifndef __DUMP_ZAP_H__
#define __DUMP_ZAP_H__

#include <stdint.h>

#include "scan.h"

extern void zap_dump_dvb_parameters(FILE *f, transponder_t *t, int sat_number);

extern void zap_dump_service_parameter_set (FILE *f, service_t *s, transponder_t *t, int sat_number);

#endif

