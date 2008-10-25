#ifndef __DUMP_VDR_H__
#define __DUMP_VDR_H__

#include <stdint.h>

#include "scan.h"

extern void vdr_dump_dvb_parameters (FILE *f, transponder_t *t, char *orbital_pos_override);

extern void vdr_dump_service_parameter_set (FILE *f, service_t *s, transponder_t *t, char *orbital_pos_override, int dump_channum, int dump_provider, int ca_select);

#endif

