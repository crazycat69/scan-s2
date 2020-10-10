/*
For SATIP

#EXTM3U
#EXTINF:-1,01 ПЕРВЫЙ КАНАЛ
http://192.168.31.11:8080/?freq=546&bw=8&msys=dvbt2&pids=0,1010,1011,1012,1014
#EXTINF:-1,02 РОССИЯ-1
http://192.168.31.11:8080/?freq=546&bw=8&msys=dvbt2&plp=1&pids=0,1020,1021,1022,1024

For VLC

#EXTM3U
#EXTINF:-1,01 ПЕРВЫЙ КАНАЛ
#EXTVLCOPT:dvb-plp-id=0
#EXTVLCOPT:dvb-bandwidth=8
#EXTVLCOPT:dvb-ts-id=1
#EXTVLCOPT:dvb-transmission=32
#EXTVLCOPT:dvb-guard=1/16
#EXTVLCOPT:program=1010
dvb-t2://frequency=546000000
*/

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "dump-m3u.h"
#include "scan.h"

static int start_header_m3u = 0;

static const char *inv_name [] = {
	"0",
	"1",
	"999" /* INVERSION_AUTO */
};

static const char *bw_name [] = {
	"8",
	"7",
	"6",
	"999", /* BANDWIDTH_AUTO */
	"5",
	"10",
	"2" /* 1.712 in VLC not support */
};

static const char *rate_name [] = {
	"0",
	"1/2",
	"2/3",
	"3/4",
	"4/5",
	"5/6",
	"6/7",
	"7/8",
	"8/9",
	"999", /* FEC_AUTO */
	"3/5",
	"9/10",
	"2/5"
};

static const char *gi_name [] = {
	"1/32",
	"1/16",
	"1/8",
	"1/4",
	"999", /* GUARD_INTERVAL_AUTO */
	"1/128",
	"19/128",
	"19/256",
	"0",
	"0",
	"0"
};

static const char *transmission_name [] = {
	"2",
	"8",
	"999", /* TRANSMISSION_MODE_AUTO */
	"4",
	"1",
	"16",
	"32",
	"0",
	"0"
};

static const char *mod_name [] = {
	"QPSK",
	"16QAM",
	"32QAM",
	"64QAM",
	"128QAM",
	"256QAM",
	"QAM", /* QAM_AUTO */
	"8VSB",
	"16VSB",
	"8PSK",
	"16APSK",
	"32APSK",
	"DQPSK",
	"NONE" /* QAM_4_NR */
};

static const char *rolloff_name [] = {
	"35",
	"20",
	"25",
	"999" /* ROLLOFF_AUTO */
};

static const char *hierarchy_name [] = {
	"0",
	"1",
	"2",
	"4",
	"999" /* HIERARCHY_AUTO  */
};

#ifndef POLARIZATION_CIRCULAR_LEFT
#define POLARIZATION_CIRCULAR_LEFT 2
#endif

static char sat_polarisation(transponder_t *t)
{
	if(t->polarisation == POLARIZATION_CIRCULAR_LEFT)
		return 'h';
	return t->polarisation == POLARISATION_HORIZONTAL ? 'h' : 'v';
}

void m3u_dvb_parameters (FILE *f, transponder_t *t, int use_url)
{
	switch (t->delivery_system) {
		case SYS_DVBS:
		case SYS_DVBS2:
			if(use_url) {
				fprintf (f, "freq=%i&", t->frequency / 1000000);
				if(t->bandwidth != BANDWIDTH_AUTO)
					fprintf (f, "bw=%s&", bw_name[t->bandwidth]);
				fprintf (f, "sr=%i&", t->symbol_rate / 1000);
				fprintf (f, "pol=%c&", sat_polarisation(t));
				if ((t->delivery_system == SYS_DVBS2) && (t->stream_id != NO_STREAM_ID_FILTER))
					fprintf (f, "plp=%d&", ((t->pls_mode & 0x3) << 26) |
							((t->pls_code & 0x3ffff) << 8) |
							(t->stream_id & 0xff));
				fprintf (f, "msys=%s&", t->delivery_system == SYS_DVBS2 ? "dvbs2" : "dvbs");
			} else {
				fprintf (f, "#EXTVLCOPT:dvb-ts-id=%i\n", t->transport_stream_id);
				if ((t->delivery_system == SYS_DVBS2) && (t->stream_id != NO_STREAM_ID_FILTER))
					fprintf (f, "#EXTVLCOPT:dvb-plp-id=%d\n", ((t->pls_mode & 0x3) << 26) |
							((t->pls_code & 0x3ffff) << 8) |
							(t->stream_id & 0xff));
				if(t->symbol_rate > 0)
					fprintf (f, "#EXTVLCOPT:dvb-srate=%i\n", t->symbol_rate / 1000);
				fprintf (f, "#EXTVLCOPT:dvb-polarization=%c\n", (char)toupper((int)sat_polarisation(t)));
				if(t->bandwidth != BANDWIDTH_AUTO)
					fprintf (f, "#EXTVLCOPT:dvb-bandwidth=%s\n", bw_name[t->bandwidth]);
				if(t->transmission_mode != TRANSMISSION_MODE_AUTO)
					fprintf (f, "#EXTVLCOPT:dvb-transmission=%s\n", transmission_name[t->transmission_mode]);
				if(t->guard_interval != GUARD_INTERVAL_AUTO)
					fprintf (f, "#EXTVLCOPT:dvb-guard=%s\n", gi_name[t->guard_interval]);
				if (t->inversion != INVERSION_AUTO)
					fprintf (f, "#EXTVLCOPT:dvb-inversion=%s\n", inv_name[t->inversion]);
				if ((t->modulation != QAM_AUTO) && (t->modulation != QAM_4_NR))
					fprintf (f, "#EXTVLCOPT:dvb-modulation=%s\n", mod_name[t->modulation]);
				if (t->fec != FEC_AUTO)
					fprintf (f, "#EXTVLCOPT:dvb-fec=%s\n", rate_name[t->fec]);
				if (t->rolloff != ROLLOFF_AUTO)
					fprintf (f, "#EXTVLCOPT:dvb-rolloff=%s\n", rolloff_name[t->rolloff]);
				fprintf (f, "%s://frequency=%i\n", t->delivery_system == SYS_DVBS2 ? "dvb-s2" : "dvb-s", t->frequency);
			}
			break;

		case SYS_DVBC_ANNEX_AC:
		case SYS_DVBC_ANNEX_B:
			if(use_url) {
				fprintf (f, "freq=%i&", t->frequency / 1000000);
				fprintf (f, "sr=%i&", t->symbol_rate / 1000);
				if(t->bandwidth != BANDWIDTH_AUTO)
					fprintf (f, "bw=%s&", bw_name[t->bandwidth]);
				if (t->inversion != INVERSION_AUTO)
					fprintf (f, "specinv=%s&", inv_name[t->inversion]);
				fprintf (f, "msys=%s&", t->delivery_system == SYS_DVBC_ANNEX_B ? "dvbcb" : "dvbc");
			} else {
				fprintf (f, "#EXTVLCOPT:dvb-ts-id=%i\n", t->transport_stream_id);
				if(t->symbol_rate > 0)
					fprintf (f, "#EXTVLCOPT:dvb-srate=%i\n", t->symbol_rate / 1000);
				if (t->inversion != INVERSION_AUTO)
					fprintf (f, "#EXTVLCOPT:dvb-inversion=%s\n", inv_name[t->inversion]);
				if ((t->modulation != QAM_AUTO) && (t->modulation != QAM_4_NR))
					fprintf (f, "#EXTVLCOPT:dvb-modulation=%s\n", mod_name[t->modulation]);
				fprintf (f, "dvb-c://frequency=%i\n", t->frequency);
			}
			break;

		case SYS_DVBT:
		case SYS_DVBT2:
			if(use_url) {
				fprintf (f, "freq=%i&", t->frequency / 1000000);
				if(t->bandwidth != BANDWIDTH_AUTO)
					fprintf (f, "bw=%s&", bw_name[t->bandwidth]);
				if((t->delivery_system == SYS_DVBT2) && (t->stream_id > 0))
					fprintf (f, "plp=%d&", t->stream_id);
				fprintf (f, "msys=%s&", t->delivery_system == SYS_DVBT2 ? "dvbt2" : "dvbt");
			} else {
				fprintf (f, "#EXTVLCOPT:dvb-ts-id=%i\n", t->transport_stream_id);
				if ((t->delivery_system == SYS_DVBT) && (t->hierarchy != HIERARCHY_AUTO) && (t->hierarchy != HIERARCHY_NONE))
					fprintf (f, "#EXTVLCOPT:dvb-hierarchy=%s\n", hierarchy_name[t->modulation]);
				if ((t->delivery_system == SYS_DVBT) && (t->inversion != INVERSION_AUTO))
					fprintf (f, "#EXTVLCOPT:dvb-inversion=%s\n", inv_name[t->inversion]);
				if ((t->modulation != QAM_AUTO) && (t->modulation != QAM_4_NR))
					fprintf (f, "#EXTVLCOPT:dvb-modulation=%s\n", mod_name[t->modulation]);
				if((t->delivery_system == SYS_DVBT2) && (t->stream_id > 0))
					fprintf (f, "#EXTVLCOPT:dvb-plp-id=%d\n", t->stream_id);
				if(t->bandwidth != BANDWIDTH_AUTO)
					fprintf (f, "#EXTVLCOPT:dvb-bandwidth=%s\n", bw_name[t->bandwidth]);
				if(t->transmission_mode != TRANSMISSION_MODE_AUTO)
					fprintf (f, "#EXTVLCOPT:dvb-transmission=%s\n", transmission_name[t->transmission_mode]);
				if(t->guard_interval != GUARD_INTERVAL_AUTO)
					fprintf (f, "#EXTVLCOPT:dvb-guard=%s\n", gi_name[t->guard_interval]);
				if ((t->delivery_system == SYS_DVBT) && ((t->fecHP != FEC_AUTO) && (t->fecHP != FEC_NONE)))
					fprintf (f, "#EXTVLCOPT:dvb-code-rate-hp=%s\n", rate_name[t->fecHP]);
				if ((t->delivery_system == SYS_DVBT) && ((t->fecLP != FEC_AUTO) && (t->fecLP != FEC_NONE)))
					fprintf (f, "#EXTVLCOPT:dvb-code-rate-lp=%s\n", rate_name[t->fecLP]);
				fprintf (f, "%s://frequency=%i\n", t->delivery_system == SYS_DVBT2 ? "dvb-t2" : "dvb-t", t->frequency);
			}
			break;

		default:
			break;
	}
}

void m3u_dump_service_parameter_set (FILE *f, service_t *s, transponder_t *t, unsigned char *url)
{
	int i;

	if(!start_header_m3u)
	{
		fprintf (f, "#EXTM3U\n");
		start_header_m3u = 1;
	}

	fprintf (f, "#EXTINF:-1,%s\n", s->service_name);

	if(url && (strlen(url) > 10))
	{
		fprintf (f, "%s/?src=1&", url); /* "src" is DiSEqC=1(A) */

		m3u_dvb_parameters (f, t, 1);

		fprintf (f, "pids=0,%i,%i", s->service_id, s->audio_pid[0]);
		for (i = 1; i < s->audio_num; i++)
			fprintf (f, ",%i", s->audio_pid[i]);
		if(s->video_pid > 0)
			fprintf (f, ",%i", s->video_pid);
		if(s->teletext_pid > 0)
			fprintf (f, ",%i", s->teletext_pid);
		fprintf (f, "\n");
	} else {
		fprintf (f, "#EXTVLCOPT:program=%i\n", s->service_id);

		m3u_dvb_parameters (f, t, 0);
	}
}
