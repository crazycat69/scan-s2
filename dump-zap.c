#include <stdio.h>
#include "dump-zap.h"

static const char *inv_name [] = {
	"INVERSION_OFF",
	"INVERSION_ON",
	"INVERSION_AUTO"
};

static const char *fec_name [] = {
	"FEC_NONE",
	"FEC_1_2",
	"FEC_2_3",
	"FEC_3_4",
	"FEC_4_5",
	"FEC_5_6",
	"FEC_6_7",
	"FEC_7_8",
	"FEC_8_9",
	"FEC_AUTO"
};

static const char *qam_name [] = {
	"QPSK",
	"QAM_16",
	"QAM_32",
	"QAM_64",
	"QAM_128",
	"QAM_256",
	"QAM_AUTO",
	"8VSB",
	"16VSB",
};

static const char *bw_name [] = {
	"BANDWIDTH_8_MHZ",
	"BANDWIDTH_7_MHZ",
	"BANDWIDTH_6_MHZ",
	"BANDWIDTH_AUTO"
};

static const char *mode_name [] = {
	"TRANSMISSION_MODE_2K",
	"TRANSMISSION_MODE_8K",
	"TRANSMISSION_MODE_AUTO"
};

static const char *guard_name [] = {
	"GUARD_INTERVAL_1_32",
	"GUARD_INTERVAL_1_16",
	"GUARD_INTERVAL_1_8",
	"GUARD_INTERVAL_1_4",
	"GUARD_INTERVAL_AUTO"
};

static const char *hierarchy_name [] = {
	"HIERARCHY_NONE",
	"HIERARCHY_1",
	"HIERARCHY_2",
	"HIERARCHY_4",
	"HIERARCHY_AUTO"
};

static char sat_polarisation(transponder_t *t)
{
	return t->polarisation == POLARISATION_VERTICAL ? 'v' : 'h';
}

void zap_dump_dvb_parameters (FILE *f, transponder_t *t, int sat_number)
{
	switch (t->delivery_system) {
		case SYS_DVBS:
		case SYS_DVBS2:
			fprintf (f, "%i:", t->frequency / 1000);	/* channels.conf wants MHz */
			fprintf (f, "%c:", sat_polarisation(t));
			fprintf (f, "%d:", sat_number);
			fprintf (f, "%i", t->symbol_rate / 1000); /* channels.conf wants kBaud */
			/*fprintf (f, "%s", fec_name[p->u.qpsk.fec_inner]);*/
			break;

		case SYS_DVBC_ANNEX_AC:
		case SYS_DVBC_ANNEX_B:
			fprintf (f, "%i:", t->frequency);
			fprintf (f, "%s:", inv_name[t->inversion]);
			fprintf (f, "%i:", t->symbol_rate);
			fprintf (f, "%s:", fec_name[t->fec]);
			fprintf (f, "%s", qam_name[t->modulation]);
			break;

		case SYS_DVBT:
			fprintf (f, "%i:", t->frequency);
			fprintf (f, "%s:", inv_name[t->inversion]);
			fprintf (f, "%s:", bw_name[t->bandwidth]);
			fprintf (f, "%s:", fec_name[t->fecHP]);
			fprintf (f, "%s:", fec_name[t->fecLP]);
			fprintf (f, "%s:", qam_name[t->modulation]);
			fprintf (f, "%s:", mode_name[t->transmission_mode]);
			fprintf (f, "%s:", guard_name[t->guard_interval]);
			fprintf (f, "%s", hierarchy_name[t->hierarchy]);
			break;

		case SYS_ATSC:
			fprintf (f, "%i:", t->frequency);
			fprintf (f, "%s", qam_name[t->modulation]);
			break;

		default:
			break;
	}
}

void zap_dump_service_parameter_set (FILE *f, service_t *s, transponder_t *t, int sat_number)
{
	fprintf (f, "%s:", s->service_name);
	zap_dump_dvb_parameters (f, t, sat_number);
	fprintf (f, ":%i:%i:%i", s->video_pid, s->audio_pid[0], s->service_id);
	fprintf (f, ":%i", t->delivery_system);
	fprintf (f, "\n");
}
