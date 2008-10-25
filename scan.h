#ifndef __SCAN_H__
#define __SCAN_H__

#include <stdio.h>
#include <errno.h>

#include <linux/dvb/frontend.h>
#include "list.h"


extern int verbosity;

#define dprintf(level, fmt...)		\
	do {							\
		if (level <= verbosity)		\
		fprintf(stderr, fmt);		\
	} while (0)

#define dpprintf(level, fmt, args...) \
	dprintf(level, "%s:%d: " fmt, __FUNCTION__, __LINE__ , ##args)

#define fatal(fmt, args...) do { dpprintf(-1, "FATAL: " fmt , ##args); exit(1); } while(0)
#define error(msg...) dprintf(0, "ERROR: " msg)
#define errorn(msg) dprintf(0, "%s:%d: ERROR: " msg ": %d %m\n", __FUNCTION__, __LINE__, errno)
#define warning(msg...) dprintf(1, "WARNING: " msg)
#define info(msg...) dprintf(2, msg)
#define verbose(msg...) dprintf(3, msg)
#define moreverbose(msg...) dprintf(4, msg)
#define debug(msg...) dpprintf(5, msg)
#define verbosedebug(msg...) dpprintf(6, msg)


enum running_mode {
	RM_NOT_RUNNING = 0x01,
	RM_STARTS_SOON = 0x02,
	RM_PAUSING     = 0x03,
	RM_RUNNING     = 0x04
};

enum polarisation {
	POLARISATION_HORIZONTAL     = 0x00,
	POLARISATION_VERTICAL       = 0x01,
	POLARISATION_CIRCULAR_LEFT  = 0x02,
	POLARISATION_CIRCULAR_RIGHT = 0x03
};



#define AUDIO_CHAN_MAX (32)
#define CA_SYSTEM_ID_MAX (16)

typedef struct service {
	struct list_head list;
	int transport_stream_id;
	int service_id;
	char *provider_name;
	char *service_name;
	uint16_t pmt_pid;
	uint16_t pcr_pid;
	uint16_t video_pid;
	uint16_t audio_pid[AUDIO_CHAN_MAX];
	char audio_lang[AUDIO_CHAN_MAX][4];
	int audio_num;
	uint16_t ca_id[CA_SYSTEM_ID_MAX];
	int ca_num;
	uint16_t teletext_pid;
	uint16_t subtitling_pid;
	uint16_t ac3_pid;
	unsigned int type         : 8;
	unsigned int scrambled	  : 1;
	enum running_mode running;
	void *priv;
	int channel_num;
} service_t;

typedef struct transponder {
	struct list_head list;
	struct list_head services;
	int network_id;
	int original_network_id;
	int transport_stream_id;
	uint32_t frequency;
	uint32_t symbol_rate;
	fe_spectral_inversion_t inversion;
	fe_code_rate_t fec;						/* DVB-S, DVB-C */
	fe_code_rate_t fecHP;					/* DVB-T */
	fe_code_rate_t fecLP;					/* DVB-T */
	fe_modulation_t modulation;
	fe_bandwidth_t bandwidth;				/* DVB-T */
	fe_hierarchy_t hierarchy;				/* DVB-T */
	fe_guard_interval_t guard_interval;		/* DVB-T */
	fe_transmit_mode_t transmission_mode;	/* DVB-T */
	enum polarisation polarisation;			/* only for DVB-S */
	int orbital_pos;						/* only for DVB-S */
	fe_delivery_system_t delivery_system;
	unsigned int we_flag		  : 1;		/* West/East Flag - only for DVB-S */
	unsigned int scan_done		  : 1;
	unsigned int last_tuning_failed	  : 1;
	unsigned int other_frequency_flag : 1;	/* DVB-T */
	unsigned int wrong_frequency	  : 1;	/* DVB-T with other_frequency_flag */
	int n_other_f;
	uint32_t *other_f;			/* DVB-T freqeuency-list descriptor */
} transponder_t;


#endif

