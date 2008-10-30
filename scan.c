/*
*  Simple MPEG parser to achieve network/service information.
*
*  refered standards:
*
*    ETSI EN 300 468
*    ETSI TR 101 211
*    ETSI ETR 211
*    ITU-T H.222.0
*
* 2005-05-10 - Basic ATSC PSIP parsing support added
*    ATSC Standard Revision B (A65/B)
*
* Thanks to Sean Device from Triveni for providing access to ATSC signals
*    and to Kevin Fowlks for his independent ATSC scanning tool.
*
* Please contribute: It is possible that some descriptors for ATSC are
*        not parsed yet and thus the result won't be complete.
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#include <assert.h>
#include <glob.h>
#include <ctype.h>

#include <linux/dvb/version.h>
#include <linux/dvb/frontend.h>
#include <linux/dvb/dmx.h>

#include "list.h"
#include "diseqc.h"
#include "dump-zap.h"
#include "dump-vdr.h"
#include "scan.h"
#include "lnb.h"

#include "atsc_psip_section.h"

#if DVB_API_VERSION != 5 || DVB_API_VERSION_MINOR != 0
#error scan-s2 requires Linux DVB driver API version 5.0!
#endif

enum table_type {
	PAT,
	PMT,
	SDT,
	NIT
};

enum format {
	OUTPUT_ZAP,
	OUTPUT_VDR,
};

static char demux_devname[80];

// Configuration parameters
int verbosity = 2;

static int scan_iterations = 10;
static int skip_count = 0;
static int long_timeout;
static int current_tp_only;
static int get_other_nits;
static int vdr_dump_provider;
static int vdr_dump_channum;
static int no_ATSC_PSIP;
static int ATSC_type=1;
static int ca_select = -1;
static int serv_select = 7;
static struct lnb_types_st lnb_type;
static int unique_anon_services;
static fe_spectral_inversion_t spectral_inversion = INVERSION_AUTO;
static int switch_pos = 0;
static int uncommitted_switch_pos = 0;
static char override_orbital_pos[16] = "";
static enum format output_format = OUTPUT_VDR;
static int output_format_set = 0;


struct section_buf {
	struct list_head list;
	const char *dmx_devname;
	unsigned int run_once  : 1;
	unsigned int segmented : 1;	/* segmented by table_id_ext */
	int fd;
	int pid;
	int table_id;
	int table_id_ext;
	int section_version_number;
	uint8_t section_done[32];
	int sectionfilter_done;
	unsigned char buf[1024];
	time_t timeout;
	time_t start_time;
	time_t running_time;
	struct section_buf *next_seg;	/* this is used to handle
									* segmented tables (like NIT-other)
									*/
	int skip_count;
};

static LIST_HEAD(scanned_transponders);
static LIST_HEAD(new_transponders);
static struct transponder *current_tp;


static void dump_dvb_parameters (FILE *f, struct transponder *p);

static void setup_filter (struct section_buf* s, const char *dmx_devname,
						  int pid, int tid, int tid_ext,
						  int run_once, int segmented, int timeout);
static void add_filter (struct section_buf *s);

static const char * fe_type2str(fe_type_t t);

/* According to the DVB standards, the combination of network_id and
* transport_stream_id should be unique, but in real life the satellite
* operators and broadcasters don't care enough to coordinate
* the numbering. Thus we identify TPs by frequency (dvbscan handles only
* one satellite at a time). Further complication: Different NITs on
* one satellite sometimes list the same TP with slightly different
* frequencies, so we have to search within some bandwidth.
*/
static struct transponder *alloc_transponder(uint32_t frequency)
{
	struct transponder *tp = calloc(1, sizeof(*tp));

	tp->frequency = frequency;

	INIT_LIST_HEAD(&tp->list);
	INIT_LIST_HEAD(&tp->services);
	list_add_tail(&tp->list, &new_transponders);
	return tp;
}

static int is_same_transponder(uint32_t f1, uint32_t f2)
{
	uint32_t diff;
	if (f1 == f2)
		return 1;
	diff = (f1 > f2) ? (f1 - f2) : (f2 - f1);
	//FIXME: use symbolrate etc. to estimate bandwidth
	if (diff < 2000) {
		debug("f1 = %u is same TP as f2 = %u\n", f1, f2);
		return 1;
	}
	return 0;
}

static struct transponder *find_transponder(uint32_t frequency)
{
	struct list_head *pos;
	struct transponder *tp;

	list_for_each(pos, &scanned_transponders) {
		tp = list_entry(pos, struct transponder, list);
		if (current_tp_only)
			return tp;

		if (is_same_transponder(tp->frequency, frequency))
			return tp;
	}

	list_for_each(pos, &new_transponders) {
		tp = list_entry(pos, struct transponder, list);

		if (is_same_transponder(tp->frequency, frequency))
			return tp;
	}

	return NULL;
}

static void copy_transponder(struct transponder *d, struct transponder *s)
{
	d->network_id = s->network_id;
	d->original_network_id = s->original_network_id;
	d->transport_stream_id = s->transport_stream_id;
	d->frequency = s->frequency;
	d->symbol_rate = s->symbol_rate;
	d->inversion = s->inversion;
	d->fec = s->fec;
	d->fecHP = s->fecHP;
	d->fecLP = s->fecLP;
	d->modulation = s->modulation;
	d->bandwidth = s->bandwidth;
	d->hierarchy = s->hierarchy;
	d->guard_interval = s->guard_interval;
	d->transmission_mode = s->transmission_mode;
	d->polarisation = s->polarisation;
	d->orbital_pos = s->orbital_pos;
	d->delivery_system = s->delivery_system;
	d->we_flag = s->we_flag;
	d->scan_done = s->scan_done;
	d->last_tuning_failed = s->last_tuning_failed;
	d->other_frequency_flag = s->other_frequency_flag;
	d->n_other_f = s->n_other_f;
	if (d->n_other_f) {
		d->other_f = calloc(d->n_other_f, sizeof(uint32_t));
		memcpy(d->other_f, s->other_f, d->n_other_f * sizeof(uint32_t));
	}
	else
		d->other_f = NULL;
}

/* service_ids are guaranteed to be unique within one TP
* (the DVB standards say theay should be unique within one
* network, but in real life...)
*/
static struct service *alloc_service(struct transponder *tp, int service_id)
{
	struct service *s = calloc(1, sizeof(*s));
	INIT_LIST_HEAD(&s->list);
	s->service_id = service_id;
	s->transport_stream_id = tp->transport_stream_id;
	list_add_tail(&s->list, &tp->services);
	return s;
}

static struct service *find_service(struct transponder *tp, int service_id)
{
	struct list_head *pos;
	struct service *s;
	list_for_each(pos, &tp->services) {
		s = list_entry(pos, struct service, list);

		if (s->service_id == service_id)
			return s;
	}

	return NULL;
}


static void parse_ca_identifier_descriptor (const unsigned char *buf, struct service *s)
{
	unsigned char len = buf [1];
	unsigned int i;

	buf += 2;

	if (len > sizeof(s->ca_id)) {
		len = sizeof(s->ca_id);
		warning("too many CA system ids\n");
	}

	s->ca_num = 0;
	memcpy(s->ca_id, buf, len);

	for (i = 0; i < len / sizeof(s->ca_id[0]); i++) {
		int id = ((s->ca_id[i] & 0x00FF) << 8) + ((s->ca_id[i] & 0xFF00) >> 8);
		s->ca_id[i] = id;
		info("  CA ID 0x%04X\n", s->ca_id[i]);

		s->ca_num++;
	}
}


static void parse_iso639_language_descriptor (const unsigned char *buf, struct service *s)
{
	unsigned char len = buf [1];

	buf += 2;

	if (len >= 4) {
		debug("    LANG=%.3s %d\n", buf, buf[3]);
		memcpy(s->audio_lang[s->audio_num], buf, 3);
#if 0
		/* seems like the audio_type is wrong all over the place */
		//if (buf[3] == 0) -> normal
		if (buf[3] == 1)
			s->audio_lang[s->audio_num][3] = '!'; /* clean effects (no language) */
		else if (buf[3] == 2)
			s->audio_lang[s->audio_num][3] = '?'; /* for the hearing impaired */
		else if (buf[3] == 3)
			s->audio_lang[s->audio_num][3] = '+'; /* visually impaired commentary */
#endif
	}
}

static void parse_network_name_descriptor (const unsigned char *buf, void *dummy)
{
	(void)dummy;

	unsigned char len = buf [1];

	info("Network Name '%.*s'\n", len, buf + 2);
}

static void parse_terrestrial_uk_channel_number (const unsigned char *buf, void *dummy)
{
	(void)dummy;

	int i, n, channel_num, service_id;
	struct list_head *p1, *p2;
	struct transponder *t;
	struct service *s;

	// 32 bits per record
	n = buf[1] / 4;
	if (n < 1)
		return;

	// desc id, desc len, (service id, service number)
	buf += 2;
	for (i = 0; i < n; i++) {
		service_id = (buf[0]<<8)|(buf[1]&0xff);
		channel_num = ((buf[2]&0x03)<<8)|(buf[3]&0xff);
		debug("Service ID 0x%X has channel number %d ", service_id, channel_num);
		list_for_each(p1, &scanned_transponders) {
			t = list_entry(p1, struct transponder, list);
			list_for_each(p2, &t->services) {
				s = list_entry(p2, struct service, list);
				if (s->service_id == service_id)
					s->channel_num = channel_num;
			}
		}
		buf += 4;
	}
}


static long bcd32_to_cpu (const int b0, const int b1, const int b2, const int b3)
{
	return ((b0 >> 4) & 0x0f) * 10000000 + (b0 & 0x0f) * 1000000 +
		((b1 >> 4) & 0x0f) * 100000   + (b1 & 0x0f) * 10000 +
		((b2 >> 4) & 0x0f) * 1000     + (b2 & 0x0f) * 100 +
		((b3 >> 4) & 0x0f) * 10       + (b3 & 0x0f);
}


static void parse_cable_delivery_system_descriptor (const unsigned char *buf, struct transponder *t)
{
	static const fe_code_rate_t fec_tab [8] = {
		FEC_AUTO, FEC_1_2, FEC_2_3, FEC_3_4,
		FEC_5_6, FEC_7_8, FEC_NONE, FEC_NONE
	};

	static const fe_modulation_t qam_tab [6] = {
		QAM_AUTO, QAM_16, QAM_32, QAM_64, QAM_128, QAM_256
	};

	if (!t) {
		warning("cable_delivery_system_descriptor outside transport stream definition (ignored)\n");
		return;
	}

	t->delivery_system = SYS_DVBC_ANNEX_AC;

	t->frequency = bcd32_to_cpu (buf[2], buf[3], buf[4], buf[5]);
	t->frequency *= 100;
	t->fec = fec_tab[buf[12] & 0x07];
	t->symbol_rate = 10 * bcd32_to_cpu (buf[9], buf[10], buf[11], buf[12] & 0xf0);
	if ((buf[8] & 0x0f) > 5)
		t->modulation = QAM_AUTO;
	else
		t->modulation = qam_tab[buf[8] & 0x0f];
	t->inversion = spectral_inversion;

	if (verbosity >= 5) {
		debug("%#04x/%#04x ", t->network_id, t->transport_stream_id);
		dump_dvb_parameters (stderr, t);
		if (t->scan_done)
			dprintf(5, " (done)");
		if (t->last_tuning_failed)
			dprintf(5, " (tuning failed)");
		dprintf(5, "\n");
	}
}

static void parse_s2_satellite_delivery_system_descriptor (const unsigned char *buf, struct transponder *t)
{
	if (!t) {
		warning("satellite_delivery_system_descriptor outside transport stream definition (ignored)\n");
		return;
	}

	t->delivery_system = SYS_DVBS2;
}

static void parse_satellite_delivery_system_descriptor (const unsigned char *buf, struct transponder *t)
{
	static const fe_code_rate_t fec_tab [8] = {
		FEC_AUTO, FEC_1_2, FEC_2_3, FEC_3_4,
		FEC_5_6, FEC_7_8, FEC_NONE, FEC_NONE
	};

	if (!t) {
		warning("satellite_delivery_system_descriptor outside transport stream definition (ignored)\n");
		return;
	}

	if(((buf[8] >> 1) & 0x01) == 0) {
		t->delivery_system = SYS_DVBS;
	}
	else {
		t->delivery_system = SYS_DVBS2;
	}

	t->frequency = 10 * bcd32_to_cpu (buf[2], buf[3], buf[4], buf[5]);
	t->fec = fec_tab[buf[12] & 0x07];
	t->symbol_rate = 10 * bcd32_to_cpu (buf[9], buf[10], buf[11], buf[12] & 0xf0);

	t->inversion = spectral_inversion;	

	t->polarisation = (buf[8] >> 5) & 0x03;
	t->orbital_pos = bcd32_to_cpu (0x00, 0x00, buf[6], buf[7]);
	t->we_flag = buf[8] >> 7;

	if (verbosity >= 5) {
		debug("%#04x/%#04x ", t->network_id, t->transport_stream_id);
		dump_dvb_parameters (stderr, t);
		if (t->scan_done)
			dprintf(5, " (done)");
		if (t->last_tuning_failed)
			dprintf(5, " (tuning failed)");
		dprintf(5, "\n");
	}
}


static void parse_terrestrial_delivery_system_descriptor (const unsigned char *buf, struct transponder *t)
{
	static const fe_modulation_t m_tab [] = { QPSK, QAM_16, QAM_64, QAM_AUTO };
	static const fe_code_rate_t ofec_tab [8] = { FEC_1_2, FEC_2_3, FEC_3_4, FEC_5_6, FEC_7_8 };

	if (!t) {
		warning("terrestrial_delivery_system_descriptor outside transport stream definition (ignored)\n");
		return;
	}

	t->delivery_system = SYS_DVBT;

	t->frequency = (buf[2] << 24) | (buf[3] << 16);
	t->frequency |= (buf[4] << 8) | buf[5];
	t->frequency *= 10;
	t->inversion = spectral_inversion;

	t->bandwidth = BANDWIDTH_8_MHZ + ((buf[6] >> 5) & 0x3);
	t->modulation = m_tab[(buf[7] >> 6) & 0x3];
	t->hierarchy = HIERARCHY_NONE + ((buf[7] >> 3) & 0x3);

	if ((buf[7] & 0x7) > 4)
		t->fecHP = FEC_AUTO;
	else
		t->fecHP = ofec_tab [buf[7] & 0x7];

	if (((buf[8] >> 5) & 0x7) > 4)
		t->fecLP = FEC_AUTO;
	else
		t->fecLP = ofec_tab [(buf[8] >> 5) & 0x7];

	t->guard_interval = GUARD_INTERVAL_1_32 + ((buf[8] >> 3) & 0x3);

	t->transmission_mode = (buf[8] & 0x2) ? TRANSMISSION_MODE_8K : TRANSMISSION_MODE_2K;

	t->other_frequency_flag = (buf[8] & 0x01);

	if (verbosity >= 5) {
		debug("%#04x/%#04x ", t->network_id, t->transport_stream_id);
		dump_dvb_parameters (stderr, t);
		if (t->scan_done)
			dprintf(5, " (done)");
		if (t->last_tuning_failed)
			dprintf(5, " (tuning failed)");
		dprintf(5, "\n");
	}
}

static void parse_frequency_list_descriptor (const unsigned char *buf, struct transponder *t)
{
	int n, i;
	typeof(*t->other_f) f;

	if (!t) {
		warning("frequency_list_descriptor outside transport stream definition (ignored)\n");
		return;
	}
	if (t->other_f)
		return;

	n = (buf[1] - 1) / 4;
	if (n < 1 || (buf[2] & 0x03) != 3)
		return;

	t->other_f = calloc(n, sizeof(*t->other_f));
	t->n_other_f = n;
	buf += 3;
	for (i = 0; i < n; i++) {
		f = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
		t->other_f[i] = f * 10;
		buf += 4;
	}
}

static void parse_service_descriptor (const unsigned char *buf, struct service *s)
{
	unsigned char len;
	unsigned char *src, *dest;

	//	s->type = buf[2];

	buf += 3;
	len = *buf;
	buf++;

	if (s->provider_name)
		free (s->provider_name);

	s->provider_name = malloc (len + 1);
	memcpy (s->provider_name, buf, len);
	s->provider_name[len] = '\0';

	/* remove control characters (FIXME: handle short/long name) */
	/* FIXME: handle character set correctly (e.g. via iconv)
	* c.f. EN 300 468 annex A */
	for (src = dest = (unsigned char *) s->provider_name; *src; src++)
		if (*src >= 0x20 && (*src < 0x80 || *src > 0x9f))
			*dest++ = *src;
	*dest = '\0';
	if (!s->provider_name[0]) {
		/* zap zero length names */
		free (s->provider_name);
		s->provider_name = 0;
	}

	if (s->service_name)
		free (s->service_name);

	buf += len;
	len = *buf;
	buf++;

	s->service_name = malloc (len + 1);
	memcpy (s->service_name, buf, len);
	s->service_name[len] = '\0';

	/* remove control characters (FIXME: handle short/long name) */
	/* FIXME: handle character set correctly (e.g. via iconv)
	* c.f. EN 300 468 annex A */
	for (src = dest = (unsigned char *) s->service_name; *src; src++)
		if (*src >= 0x20 && (*src < 0x80 || *src > 0x9f))
			*dest++ = *src;
	*dest = '\0';
	if (!s->service_name[0]) {
		/* zap zero length names */
		free (s->service_name);
		s->service_name = 0;
	}

	info("0x%04X 0x%04X: pmt_pid 0x%04X %s -- %s (%s%s)\n",
		s->transport_stream_id,
		s->service_id,
		s->pmt_pid,
		s->provider_name, s->service_name,
		s->running == RM_NOT_RUNNING ? "not running" :
		s->running == RM_STARTS_SOON ? "starts soon" :
		s->running == RM_PAUSING     ? "pausing" :
		s->running == RM_RUNNING     ? "running" : "???",
		s->scrambled ? ", scrambled" : "");
}

static void parse_ca_descriptor (const unsigned char *buf, struct service *s) 
{
	unsigned char descriptor_length = buf [1];
	int CA_system_ID;
	int found=0;
	int i;

	buf += 2;

	if (descriptor_length < 4) return;

	CA_system_ID = (buf[0] << 8) | buf[1];

	for (i=0; i<s->ca_num; i++)
		if (s->ca_id[i] == CA_system_ID)
			found++;

	if (!found) {
		if (s->ca_num + 1 >= CA_SYSTEM_ID_MAX)
			warning("TOO MANY CA SYSTEM IDs.\n");
		else {
			info("  CA ID     : PID 0x%04X\n", CA_system_ID);
			s->ca_id[s->ca_num]=CA_system_ID;
			s->ca_num++;
		}
	} 	
} 

static int find_descriptor(uint8_t tag, const unsigned char *buf,
						   int descriptors_loop_len,
						   const unsigned char **desc, int *desc_len)
{
	while (descriptors_loop_len > 0) {
		unsigned char descriptor_tag = buf[0];
		unsigned char descriptor_len = buf[1] + 2;

		if (!descriptor_len) {
			warning("descriptor_tag == 0x%02X, len is 0\n", descriptor_tag);
			break;
		}

		if (tag == descriptor_tag) {
			if (desc)
				*desc = buf;
			if (desc_len)
				*desc_len = descriptor_len;
			return 1;
		}

		buf += descriptor_len;
		descriptors_loop_len -= descriptor_len;
	}
	return 0;
}

static void parse_descriptors(enum table_type t, const unsigned char *buf,
							  int descriptors_loop_len, void *data)
{
	while (descriptors_loop_len > 0) {
		unsigned char descriptor_tag = buf[0];
		unsigned char descriptor_len = buf[1] + 2;

		if (!descriptor_len) {
			warning("descriptor_tag == 0x%02X, len is 0\n", descriptor_tag);
			break;
		}

		switch(descriptor_tag) 
		{
		case 0x09:		/* 0x09 ca_descriptor, caid patch 20080106 */
			if (t == PMT)
				parse_ca_descriptor (buf, data);	
			break;

		case 0x0a:
			if (t == PMT)
				parse_iso639_language_descriptor (buf, data);
			break;

		case 0x40:
			if (t == NIT)
				parse_network_name_descriptor (buf, data);
			break;

		case 0x43:
			if (t == NIT)
				parse_satellite_delivery_system_descriptor (buf, data);
			break;

		case 0x79:
			info("ERROR: S2_satellite_delivery_system_descriptor not parsed\n");
			if( t == NIT)
				parse_s2_satellite_delivery_system_descriptor (buf, data);
			break;

		case 0x44:
			if (t == NIT)
				parse_cable_delivery_system_descriptor (buf, data);
			break;

		case 0x48:
			if (t == SDT)
				parse_service_descriptor (buf, data);
			break;

		case 0x53:
			if (t == SDT)
				parse_ca_identifier_descriptor (buf, data);
			break;

		case 0x5a:
			if (t == NIT)
				parse_terrestrial_delivery_system_descriptor (buf, data);
			break;

		case 0x62:
			if (t == NIT)
				parse_frequency_list_descriptor (buf, data);
			break;

		case 0x83:
			/* 0x83 is in the privately defined range of descriptor tags,
			* so we parse this only if the user says so to avoid
			* problems when 0x83 is something entirely different... */
			if (t == NIT && vdr_dump_channum)
				parse_terrestrial_uk_channel_number (buf, data);
			break;

		default:
			verbosedebug("skip descriptor 0x%02X\n", descriptor_tag);
		};

		buf += descriptor_len;
		descriptors_loop_len -= descriptor_len;
	}
}


static void parse_pat(const unsigned char *buf, int section_length,
					  int transport_stream_id)
{
	(void)transport_stream_id;

	while (section_length > 0) {
		struct service *s;
		int service_id = (buf[0] << 8) | buf[1];

		info("service_id = 0x%X\n",service_id);
		if (service_id == 0)
			goto skip;	/* nit pid entry */

		/* SDT might have been parsed first... */
		s = find_service(current_tp, service_id);
		if (!s)
			s = alloc_service(current_tp, service_id);
		s->pmt_pid = ((buf[2] & 0x1f) << 8) | buf[3];
		info("pmt_pid = 0x%X\n",s->pmt_pid);
		if (!s->priv && s->pmt_pid) {
			s->priv = malloc(sizeof(struct section_buf));
			setup_filter(s->priv, demux_devname,
				s->pmt_pid, 0x02, s->service_id, 1, 0, 5);

			add_filter (s->priv);
		}

skip:
		buf += 4;
		section_length -= 4;
	};
}


static void parse_pmt (const unsigned char *buf, int section_length, int service_id)
{
	int program_info_len;
	struct service *s;
	char msg_buf[14 * AUDIO_CHAN_MAX + 1];
	char *tmp;
	int i;

	s = find_service (current_tp, service_id);
	if (!s) {
		error("PMT for service_id 0x%04X was not in PAT\n", service_id);
		return;
	}

	s->pcr_pid = ((buf[0] & 0x1f) << 8) | buf[1];

	program_info_len = ((buf[2] & 0x0f) << 8) | buf[3];

	// caid patch 20080106, search PMT program info for CA Ids
	buf +=4;
	section_length -= 4;

	while (program_info_len > 0) {
		int descriptor_length = ((int)buf[1]) + 2;
		parse_descriptors(PMT, buf, section_length, s);
		buf += descriptor_length;
		section_length   -= descriptor_length;
		program_info_len -= descriptor_length;
	}

	while (section_length >= 5) {
		int ES_info_len = ((buf[3] & 0x0f) << 8) | buf[4];
		int elementary_pid = ((buf[1] & 0x1f) << 8) | buf[2];

		switch (buf[0]) 
		{
		case 0x01:
		case 0x02:
		case 0x1b:
			info("  VIDEO     : PID 0x%04X\n", elementary_pid);
			if (s->video_pid == 0)
				s->video_pid = elementary_pid;
			break;

		case 0x03:
		case 0x81: /* Audio per ATSC A/53B [2] Annex B */
		case 0x04:
			info("  AUDIO     : PID 0x%04X\n", elementary_pid);
			if (s->audio_num < AUDIO_CHAN_MAX) {
				s->audio_pid[s->audio_num] = elementary_pid;
				parse_descriptors (PMT, buf + 5, ES_info_len, s);
				s->audio_num++;
			}
			else
				warning("more than %i audio channels, truncating\n",
				AUDIO_CHAN_MAX);
			break;

		case 0x06:
			if (find_descriptor(0x56, buf + 5, ES_info_len, NULL, NULL)) {
				info("  TELETEXT  : PID 0x%04X\n", elementary_pid);
				s->teletext_pid = elementary_pid;
				break;
			}
			else if (find_descriptor(0x59, buf + 5, ES_info_len, NULL, NULL)) {
				/* Note: The subtitling descriptor can also signal
				* teletext subtitling, but then the teletext descriptor
				* will also be present; so we can be quite confident
				* that we catch DVB subtitling streams only here, w/o
				* parsing the descriptor. */
				info("  SUBTITLING: PID 0x%04X\n", elementary_pid);
				s->subtitling_pid = elementary_pid;
				break;
			}
			else if (find_descriptor(0x6a, buf + 5, ES_info_len, NULL, NULL)) {
				info("  AC3       : PID 0x%04X\n", elementary_pid);
				s->ac3_pid = elementary_pid;
				break;
			}
			/* fall through */

		default:
			info("  OTHER     : PID 0x%04X TYPE 0x%02X\n", elementary_pid, buf[0]);
		};

		buf += ES_info_len + 5;
		section_length -= ES_info_len + 5;
	};

	tmp = msg_buf;
	tmp += sprintf(tmp, "0x%04X (%.4s)", s->audio_pid[0], s->audio_lang[0]);

	if (s->audio_num > AUDIO_CHAN_MAX) {
		warning("more than %i audio channels: %i, truncating to %i\n",
			AUDIO_CHAN_MAX, s->audio_num, AUDIO_CHAN_MAX);
		s->audio_num = AUDIO_CHAN_MAX;
	}

	for (i=1; i<s->audio_num; i++)
		tmp += sprintf(tmp, ", 0x%04X (%.4s)", s->audio_pid[i], s->audio_lang[i]);

	debug("0x%04X 0x%04X: %s -- %s, pmt_pid 0x%04X, vpid 0x%04X, apid %s\n",
		s->transport_stream_id,
		s->service_id,
		s->provider_name, s->service_name,
		s->pmt_pid, s->video_pid, msg_buf);
}


static void parse_nit (const unsigned char *buf, int section_length, int network_id)
{
	int descriptors_loop_len = ((buf[0] & 0x0f) << 8) | buf[1];

	if (section_length < descriptors_loop_len + 4)
	{
		warning("section too short: network_id == 0x%04X, section_length == %i, "
			"descriptors_loop_len == %i\n",
			network_id, section_length, descriptors_loop_len);
		return;
	}

	parse_descriptors (NIT, buf + 2, descriptors_loop_len, NULL);

	section_length -= descriptors_loop_len + 4;
	buf += descriptors_loop_len + 4;

	while (section_length > 6) {
		int transport_stream_id = (buf[0] << 8) | buf[1];
		struct transponder *t, tn;

		descriptors_loop_len = ((buf[4] & 0x0f) << 8) | buf[5];

		if (section_length < descriptors_loop_len + 4)
		{
			warning("section too short: transport_stream_id == 0x%04X, "
				"section_length == %i, descriptors_loop_len == %i\n",
				transport_stream_id, section_length,
				descriptors_loop_len);
			break;
		}

		debug("transport_stream_id 0x%04X\n", transport_stream_id);

		memset(&tn, 0, sizeof(tn));
		tn.network_id = network_id;
		tn.original_network_id = (buf[2] << 8) | buf[3];
		tn.transport_stream_id = transport_stream_id;
		tn.fec = FEC_AUTO;
		tn.inversion = spectral_inversion;
		tn.modulation = QAM_AUTO;
		tn.rolloff = ROLLOFF_AUTO;

		parse_descriptors (NIT, buf + 6, descriptors_loop_len, &tn);

		t = find_transponder(tn.frequency);
		
		if (t == NULL) {
			if(get_other_nits) {
				// New transponder
				t = alloc_transponder(tn.frequency);

				// For sattelites start with DVB-S, it will switch to DVB-S2 if DVB-S gives no results
				if(current_tp->delivery_system == SYS_DVBS || current_tp->delivery_system == SYS_DVBS2) {
					tn.delivery_system = SYS_DVBS;
				}
			}
		}
		else {
			// transponder exist, use its known delivery system
			tn.delivery_system = t->delivery_system;

			// Transponder already exist and its parameters are not set to AUTO, use the specified parameters
			if(t->fec != FEC_AUTO) {
				tn.fec = t->fec;
			}
		}

		if(t != NULL) {
			copy_transponder(t, &tn);
		}

		section_length -= descriptors_loop_len + 6;
		buf += descriptors_loop_len + 6;
	}
}


static void parse_sdt (const unsigned char *buf, int section_length,
					   int transport_stream_id)
{
	(void)transport_stream_id;

	buf += 3;	       /*  skip original network id + reserved field */

	while (section_length >= 5) {
		int service_id = (buf[0] << 8) | buf[1];
		int descriptors_loop_len = ((buf[3] & 0x0f) << 8) | buf[4];
		struct service *s;

		if (section_length < descriptors_loop_len || !descriptors_loop_len)
		{
			warning("section too short: service_id == 0x%02X, section_length == %i, "
				"descriptors_loop_len == %i\n",
				service_id, section_length,
				descriptors_loop_len);
			break;
		}

		s = find_service(current_tp, service_id);
		if (!s)
			/* maybe PAT has not yet been parsed... */
			s = alloc_service(current_tp, service_id);

		s->running = (buf[3] >> 5) & 0x7;
		s->scrambled = (buf[3] >> 4) & 1;

		parse_descriptors (SDT, buf + 5, descriptors_loop_len, s);

		section_length -= descriptors_loop_len + 5;
		buf += descriptors_loop_len + 5;
	};
}

/* ATSC PSIP VCT */
static void parse_atsc_service_loc_desc(struct service *s,const unsigned char *buf)
{
	struct ATSC_service_location_descriptor d = read_ATSC_service_location_descriptor(buf);
	int i;
	unsigned char *b = (unsigned char *) buf+5;

	s->pcr_pid = d.PCR_PID;
	for (i=0; i < d.number_elements; i++) {
		struct ATSC_service_location_element e = read_ATSC_service_location_element(b);
		switch (e.stream_type) 
		{
		case 0x02: /* video */
			s->video_pid = e.elementary_PID;
			info("  VIDEO     : PID 0x%04X\n", e.elementary_PID);
			break;

		case 0x81: /* ATSC audio */
			if (s->audio_num < AUDIO_CHAN_MAX) {
				s->audio_pid[s->audio_num] = e.elementary_PID;
				s->audio_lang[s->audio_num][0] = (e.ISO_639_language_code >> 16) & 0xff;
				s->audio_lang[s->audio_num][1] = (e.ISO_639_language_code >> 8)  & 0xff;
				s->audio_lang[s->audio_num][2] =  e.ISO_639_language_code        & 0xff;
				s->audio_num++;
			}
			info("  AUDIO     : PID 0x%04X lang: %s\n",e.elementary_PID,s->audio_lang[s->audio_num-1]);
			break;

		default:
			warning("unhandled stream_type: %X\n",e.stream_type);
			break;
		};

		b += 6;
	}
}

static void parse_atsc_ext_chan_name_desc(struct service *s,const unsigned char *buf)
{
	unsigned char *b = (unsigned char *) buf+2;
	int i,j;
	int num_str = b[0];

	b++;
	for (i = 0; i < num_str; i++) {
		int num_seg = b[3];
		b += 4; /* skip lang code */
		for (j = 0; j < num_seg; j++) {
			int comp_type = b[0],/* mode = b[1],*/ num_bytes = b[2];

			switch (comp_type) 
			{
			case 0x00:
				if (s->service_name)
					free(s->service_name);
				s->service_name = malloc(num_bytes * sizeof(char) + 1);
				memcpy(s->service_name,&b[3],num_bytes);
				s->service_name[num_bytes] = '\0';
				break;

			default:
				warning("compressed strings are not supported yet\n");
				break;
			}

			b += 3 + num_bytes;
		}
	}
}

static void parse_psip_descriptors(struct service *s,const unsigned char *buf,int len)
{
	unsigned char *b = (unsigned char *) buf;
	int desc_len;
	while (len > 0) {
		desc_len = b[1];
		switch (b[0]) 
		{
		case ATSC_SERVICE_LOCATION_DESCRIPTOR_ID:
			parse_atsc_service_loc_desc(s,b);
			break;

		case ATSC_EXTENDED_CHANNEL_NAME_DESCRIPTOR_ID:
			parse_atsc_ext_chan_name_desc(s,b);
			break;

		default:
			warning("unhandled psip descriptor: %02x\n",b[0]);
			break;
		}

		b += 2 + desc_len;
		len -= 2 + desc_len;
	}
}

static void parse_psip_vct (const unsigned char *buf, int section_length,
							int table_id, int transport_stream_id)
{
	(void)section_length;
	(void)table_id;
	(void)transport_stream_id;

	/*	int protocol_version = buf[0];*/
	int num_channels_in_section = buf[1];
	int i;
	int pseudo_id = 0xffff;
	unsigned char *b = (unsigned char *) buf + 2;

	for (i = 0; i < num_channels_in_section; i++) {
		struct service *s;
		struct tvct_channel ch = read_tvct_channel(b);

		switch (ch.service_type) 
		{
		case 0x01:
			info("analog channels won't be put info channels.conf\n");
			break;

		case 0x02: /* ATSC TV */
		case 0x03: /* ATSC Radio */
			break;

		case 0x04: /* ATSC Data */
		default:
			continue;
		}

		if (ch.program_number == 0)
			ch.program_number = --pseudo_id;

		s = find_service(current_tp, ch.program_number);
		if (!s)
			s = alloc_service(current_tp, ch.program_number);

		if (s->service_name)
			free(s->service_name);

		s->service_name = malloc(7*sizeof(unsigned char));
		/* TODO find a better solution to convert UTF-16 */
		s->service_name[0] = ch.short_name0;
		s->service_name[1] = ch.short_name1;
		s->service_name[2] = ch.short_name2;
		s->service_name[3] = ch.short_name3;
		s->service_name[4] = ch.short_name4;
		s->service_name[5] = ch.short_name5;
		s->service_name[6] = ch.short_name6;

		parse_psip_descriptors(s,&b[32],ch.descriptors_length);

		s->channel_num = ch.major_channel_number << 10 | ch.minor_channel_number;

		if (ch.hidden) {
			s->running = RM_NOT_RUNNING;
			info("service is not running, pseudo program_number.");
		} else {
			s->running = RM_RUNNING;
			info("service is running.");
		}

		info(" Channel number: %d:%d. Name: '%s'\n",
			ch.major_channel_number, ch.minor_channel_number,s->service_name);

		b += 32 + ch.descriptors_length;
	}
}

static int get_bit (uint8_t *bitfield, int bit)
{
	return (bitfield[bit/8] >> (bit % 8)) & 1;
}

static void set_bit (uint8_t *bitfield, int bit)
{
	bitfield[bit/8] |= 1 << (bit % 8);
}


/**
*   returns 0 when more sections are expected
*	   1 when all sections are read on this pid
*	   -1 on invalid table id
*/
static int parse_section (struct section_buf *s)
{
	const unsigned char *buf = s->buf;
	int table_id;
	int section_length;
	int table_id_ext;
	int section_version_number;
	int section_number;
	int last_section_number;
	int i;

	table_id = buf[0];

	if (s->table_id != table_id) {
		info(">>> s->table_id (%X) != table_id (%X)!\n", s->table_id, table_id);
		return -1;
	}

	section_length = ((buf[1] & 0x0f) << 8) | buf[2];

	table_id_ext = (buf[3] << 8) | buf[4];
	section_version_number = (buf[5] >> 1) & 0x1f;
	section_number = buf[6];
	last_section_number = buf[7];

	info(">>> parse_section, section number %d out of %d...!\n", section_number, last_section_number);

	if (s->segmented && s->table_id_ext != -1 && s->table_id_ext != table_id_ext) {
		/* find or allocate actual section_buf matching table_id_ext */
		while (s->next_seg) {
			s = s->next_seg;
			if (s->table_id_ext == table_id_ext)
				break;
		}
		if (s->table_id_ext != table_id_ext) {
			assert(s->next_seg == NULL);
			s->next_seg = calloc(1, sizeof(struct section_buf));
			s->next_seg->segmented = s->segmented;
			s->next_seg->run_once = s->run_once;
			s->next_seg->timeout = s->timeout;
			s = s->next_seg;
			s->table_id = table_id;
			s->table_id_ext = table_id_ext;
			s->section_version_number = section_version_number;
		}
	}

	if (s->section_version_number != section_version_number || s->table_id_ext != table_id_ext) {
		struct section_buf *next_seg = s->next_seg;

		if (s->section_version_number != -1 && s->table_id_ext != -1) {
			debug("section version_number or table_id_ext changed "
				"%d -> %d / %04x -> %04x\n",
				s->section_version_number, section_version_number,
				s->table_id_ext, table_id_ext);
		}

		s->table_id_ext = table_id_ext;
		s->section_version_number = section_version_number;
		s->sectionfilter_done = 0;
		memset (s->section_done, 0, sizeof(s->section_done));
		s->next_seg = next_seg;
	}

	buf += 8;			/* past generic table header */
	section_length -= 5 + 4;	/* header + crc */
	if (section_length < 0) {
		warning("truncated section (PID 0x%04X, lenght %d)",
			s->pid, section_length + 9);
		return 0;
	}

	if (!get_bit(s->section_done, section_number)) {
		set_bit (s->section_done, section_number);

		debug("pid 0x%02X tid 0x%02X table_id_ext 0x%04X, "
			"%i/%i (version %i)\n",
			s->pid, table_id, table_id_ext, section_number,
			last_section_number, section_version_number);

		switch (table_id) 
		{
		case 0x00:
			info("parse_pat......\n");
			verbose("PAT\n");
			parse_pat (buf, section_length, table_id_ext);
			break;

		case 0x02:
			info("parse_pmt......\n");
			verbose("PMT 0x%04X for service 0x%04X\n", s->pid, table_id_ext);
			parse_pmt (buf, section_length, table_id_ext);
			break;

		case 0x41:
			verbose("NIT (other TS)\n");
			if(get_other_nits) {
				info("parse_nit other......\n");
				parse_nit (buf, section_length, table_id_ext);
			}
			else {
				verbose("Ignoring, use -n switch to enable parsing\n");
			}
			break;

		case 0x40:
			info("parse_nit actual......\n");
			verbose("NIT (actual TS)\n");
			parse_nit (buf, section_length, table_id_ext);
			break;

		case 0x42:
		case 0x46:
			info("parse_sdt......\n");
			verbose("SDT (%s TS)\n", table_id == 0x42 ? "actual":"other");
			parse_sdt (buf, section_length, table_id_ext);
			break;

		case 0xc8:
		case 0xc9:
			verbose("ATSC VCT\n");
			parse_psip_vct(buf, section_length, table_id, table_id_ext);
			break;

		default:
			break;
		};

		for (i = 0; i <= last_section_number; i++)
			if (get_bit (s->section_done, i) == 0)
				break;

		if (i > last_section_number)
			s->sectionfilter_done = 1;
	}

	if (s->segmented) {
		info(">>> segmented!\n");
		/* always wait for timeout; this is because we don't now how
		* many segments there are
		*/
		return 0;
	}
	else if (s->sectionfilter_done)
		return 1;

	return 0;
}


static int read_sections (struct section_buf *s)
{
	int section_length, count;

	if (s->sectionfilter_done && !s->segmented)
		return 1;

	/* the section filter API guarantess that we get one full section
	* per read(), provided that the buffer is large enough (it is)
	*/
	if (((count = read (s->fd, s->buf, sizeof(s->buf))) < 0) && errno == EOVERFLOW)
		count = read (s->fd, s->buf, sizeof(s->buf));
	if (count < 0) {
		errorn("read_sections: read error");
		return -1;
	}

	s->skip_count--;
	if(s->skip_count > 0) {
		info("skipping section, table_id %X, pid %X\n", s->table_id, s->pid);
		return -1;
	}

	if (count < 4)
		return -1;

	section_length = ((s->buf[1] & 0x0f) << 8) | s->buf[2];

	if (count != section_length + 3)
		return -1;

	if (parse_section(s) == 1)
		return 1;

	return 0;
}


static LIST_HEAD(running_filters);
static LIST_HEAD(waiting_filters);
static int n_running;
#define MAX_RUNNING 128
static struct pollfd poll_fds[MAX_RUNNING];
static struct section_buf* poll_section_bufs[MAX_RUNNING];


static void setup_filter (struct section_buf* s, const char *dmx_devname,
						  int pid, int tid, int tid_ext,
						  int run_once, int segmented, int timeout)
{
	memset (s, 0, sizeof(struct section_buf));

	s->fd = -1;
	s->dmx_devname = dmx_devname;
	s->pid = pid;
	s->table_id = tid;

	s->skip_count = skip_count;
	s->run_once = run_once;
	s->segmented = segmented;

	if (long_timeout) {
		s->timeout = 5 * timeout;
	}
	else {
		s->timeout = timeout;
	}

	// multiplying timers since we skip first received messages
	if(s->skip_count > 0) {
		s->timeout *= 2;
	}

	s->table_id_ext = tid_ext;
	s->section_version_number = -1;

	INIT_LIST_HEAD (&s->list);
}

static void update_poll_fds(void)
{
	struct list_head *p;
	struct section_buf* s;
	int i;

	memset(poll_section_bufs, 0, sizeof(poll_section_bufs));
	for (i = 0; i < MAX_RUNNING; i++)
		poll_fds[i].fd = -1;
	i = 0;
	list_for_each (p, &running_filters) {
		if (i >= MAX_RUNNING)
			fatal("too many poll_fds\n");
		s = list_entry (p, struct section_buf, list);
		if (s->fd == -1)
			fatal("s->fd == -1 on running_filters\n");
		verbosedebug("poll fd %d\n", s->fd);
		poll_fds[i].fd = s->fd;
		poll_fds[i].events = POLLIN;
		poll_fds[i].revents = 0;
		poll_section_bufs[i] = s;
		i++;
	}
	if (i != n_running)
		fatal("n_running is hosed\n");
}

static int start_filter (struct section_buf* s)
{
	struct dmx_sct_filter_params f;

	if (n_running >= MAX_RUNNING)
		goto err0;
	if ((s->fd = open (s->dmx_devname, O_RDWR | O_NONBLOCK)) < 0)
		goto err0;

	verbosedebug("start filter pid 0x%04X table_id 0x%02X\n", s->pid, s->table_id);

	memset(&f, 0, sizeof(f));

	f.pid = (uint16_t) s->pid;

	if (s->table_id < 0x100 && s->table_id > 0) {
		f.filter.filter[0] = (uint8_t) s->table_id;
		f.filter.mask[0]   = 0xff;
	}
	if (s->table_id_ext < 0x10000 && s->table_id_ext > 0) {
		f.filter.filter[1] = (uint8_t) ((s->table_id_ext >> 8) & 0xff);
		f.filter.filter[2] = (uint8_t) (s->table_id_ext & 0xff);
		f.filter.mask[1] = 0xff;
		f.filter.mask[2] = 0xff;
	}

	f.timeout = 0;
	f.flags = DMX_IMMEDIATE_START | DMX_CHECK_CRC;

	if (ioctl(s->fd, DMX_SET_FILTER, &f) == -1) {
		errorn ("ioctl DMX_SET_FILTER failed");
		goto err1;
	}

	s->sectionfilter_done = 0;
	time(&s->start_time);

	list_del_init (&s->list);  /* might be in waiting filter list */
	list_add (&s->list, &running_filters);

	n_running++;
	update_poll_fds();

	return 0;

err1:
	ioctl (s->fd, DMX_STOP);
	close (s->fd);
err0:
	return -1;
}


static void stop_filter (struct section_buf *s)
{
	verbosedebug("stop filter pid 0x%04X\n", s->pid);
	ioctl (s->fd, DMX_STOP);
	close (s->fd);
	s->fd = -1;
	list_del (&s->list);
	s->running_time += time(NULL) - s->start_time;

	n_running--;
	update_poll_fds();
}


static void add_filter (struct section_buf *s)
{
	verbosedebug("add filter pid 0x%04X\n", s->pid);
	if (start_filter (s))
		list_add_tail (&s->list, &waiting_filters);
}


static void remove_filter (struct section_buf *s)
{
	verbosedebug("remove filter pid 0x%04X\n", s->pid);
	stop_filter (s);

	while (!list_empty(&waiting_filters)) {
		struct list_head *next = waiting_filters.next;
		s = list_entry (next, struct section_buf, list);
		if (start_filter (s))
			break;
	};
}


static void read_filters (void)
{
	struct section_buf *s;
	int i, n, done;

	n = poll(poll_fds, n_running, 1000);
	if (n == -1)
		errorn("poll");

	for (i = 0; i < n_running; i++) {
		s = poll_section_bufs[i];
		if (!s)
			fatal("poll_section_bufs[%d] is NULL\n", i);
		if (poll_fds[i].revents)
			done = read_sections (s) == 1;
		else
			done = 0; /* timeout */
		if (done || time(NULL) > s->start_time + s->timeout) {
			if (s->run_once) {
				if (done)
					verbosedebug("filter done pid 0x%04X\n", s->pid);
				else
					warning("filter timeout pid 0x%04X\n", s->pid);
				remove_filter (s);
			}
		}
	}
}

static int __tune_to_transponder (int frontend_fd, struct transponder *t)
{
	int rc;
	int i;
	fe_status_t s;
	uint32_t if_freq;
	current_tp = t;

	struct dtv_property p_clear[] = {
		{ .cmd = DTV_CLEAR },
	};

	struct dtv_properties cmdseq_clear = {
		.num = 1,
		.props = p_clear
	};

	if ((ioctl(frontend_fd, FE_SET_PROPERTY, &cmdseq_clear)) == -1) {
		perror("FE_SET_PROPERTY DTV_CLEAR failed");
		return;
	}

	if (verbosity >= 1) {
		dprintf(1, ">>> tune to: ");
		dump_dvb_parameters (stderr, t);
		if (t->last_tuning_failed)
			dprintf(1, " (tuning failed)");
		dprintf(1, "\n");
	}

	if (t->delivery_system == SYS_DVBS || t->delivery_system == SYS_DVBS2) {
		if (lnb_type.high_val) {
			if (lnb_type.switch_val) {
				/* Voltage-controlled switch */
				int hiband = 0;

				if (t->frequency >= lnb_type.switch_val)
					hiband = 1;

				setup_switch (frontend_fd,
					switch_pos,
					t->polarisation == POLARISATION_VERTICAL ? 0 : 1,
					hiband,
					uncommitted_switch_pos);

				usleep(50000);

				if (hiband)
					if_freq = abs(t->frequency - lnb_type.high_val);
				else
					if_freq = abs(t->frequency - lnb_type.low_val);
			} else {
				/* C-Band Multipoint LNBf */
				if_freq = abs(t->frequency - (t->polarisation == POLARISATION_VERTICAL ? 
					lnb_type.low_val: lnb_type.high_val));
			}
		} else	{
			/* Monopoint LNBf without switch */
			if_freq = abs(t->frequency - lnb_type.low_val);
		}
		if (verbosity >= 2)
			dprintf(1,"DVB-S IF freq is %d\n", if_freq);
	}


	struct dvb_frontend_event ev;
	struct dtv_property p_tune[] = {
		{ .cmd = DTV_DELIVERY_SYSTEM,	.u.data = t->delivery_system },
		{ .cmd = DTV_FREQUENCY,			.u.data = if_freq },
		{ .cmd = DTV_MODULATION,		.u.data = t->modulation },
		{ .cmd = DTV_SYMBOL_RATE,		.u.data = t->symbol_rate },
		{ .cmd = DTV_INNER_FEC,			.u.data = t->fec },
		{ .cmd = DTV_INVERSION,			.u.data = t->inversion },
		{ .cmd = DTV_ROLLOFF,			.u.data = t->rolloff },
		{ .cmd = DTV_PILOT,				.u.data = PILOT_AUTO },
		{ .cmd = DTV_TUNE },
	};
	struct dtv_properties cmdseq_tune = {
		.num = 9,
		.props = p_tune
	};

	/* discard stale QPSK events */
	while (1) {
		if (ioctl(frontend_fd, FE_GET_EVENT, &ev) == -1)
			break;
	}

	if ((ioctl(frontend_fd, FE_SET_PROPERTY, &cmdseq_tune)) == -1) {
		perror("FE_SET_PROPERTY TUNE failed");
		return;
	}

	for (i = 0; i < scan_iterations; i++) {
		usleep (200000);

		if (ioctl(frontend_fd, FE_READ_STATUS, &s) == -1) {
			errorn("FE_READ_STATUS failed");
			return -1;
		}

		verbose(">>> tuning status == 0x%02X\n", s);

		// Tuning succeed
		if (s & FE_HAS_LOCK) {
			t->last_tuning_failed = 0;

			struct dtv_property p[] = {
				{ .cmd = DTV_DELIVERY_SYSTEM },
				{ .cmd = DTV_MODULATION },
				{ .cmd = DTV_INNER_FEC },
				{ .cmd = DTV_INVERSION },
				{ .cmd = DTV_ROLLOFF },
			};

			struct dtv_properties cmdseq = {
				.num = 5,
				.props = p
			};

			// get the actual parameters from the driver for that channel
			if ((ioctl(frontend_fd, FE_GET_PROPERTY, &cmdseq)) == -1) {
				perror("FE_GET_PROPERTY failed");
				return;
			}

			t->delivery_system = p[0].u.data;
			t->modulation = p[1].u.data;
			t->fec = p[2].u.data;
			t->inversion = p[3].u.data;
			t->rolloff = p[4].u.data;

			return 0;
		}
	}

	warning(">>> tuning failed!!!\n");

	t->last_tuning_failed = 1;

	return -1;
}

static int tune_to_transponder (int frontend_fd, struct transponder *t)
{
	/* move TP from "new" to "scanned" list */
	list_del_init(&t->list);
	list_add_tail(&t->list, &scanned_transponders);
	t->scan_done = 1;

	switch(t->delivery_system) 
	{
	case SYS_DVBS:
		info("----------------------------------> Using DVB-S\n");
		break;

	case SYS_DSS:
		info("----------------------------------> Using DSS\n");
		break;

	case SYS_DVBS2:
		info("----------------------------------> Using DVB-S2\n");
		break;

	case SYS_DVBT:
		info("----------------------------------> Using DVB-T\n");
		break;

	case SYS_ATSC:
		info("----------------------------------> Using ATSC\n");
		break;

	case SYS_DVBC_ANNEX_AC:
		info("----------------------------------> Using DVB-C ANNEX_AC\n");
		break;

	case SYS_DVBC_ANNEX_B:
		info("----------------------------------> Using DVB-C ANNEX_B\n");
		break;

	default:
		info("Unsupported Delivery system (%d)!\n", t->delivery_system);
		t->last_tuning_failed = 1;
		return -1;
	}

	if (__tune_to_transponder (frontend_fd, t) == 0)
		return 0;

	return __tune_to_transponder (frontend_fd, t);
}

static int tune_to_next_transponder (int frontend_fd)
{
	struct list_head *pos, *tmp;
	struct transponder *t, *to;
	uint32_t freq;
	int rc;

	list_for_each_safe(pos, tmp, &new_transponders) {
		t = list_entry (pos, struct transponder, list);
retry:

		rc = tune_to_transponder(frontend_fd, t);

		// If scan failed and it's a DVB-S system, try DVB-S2 before giving up
		if (rc != 0 && t->delivery_system == SYS_DVBS) {
			t->delivery_system = SYS_DVBS2;
			rc = tune_to_transponder(frontend_fd, t);
		}

		if (rc == 0) {
			return 0;
		}

		if(rc == -2) {
			return -2;
		}
next:
		if (t->other_frequency_flag && t->other_f && t->n_other_f) {
			/* check if the alternate freqeuncy is really new to us */
			freq = t->other_f[t->n_other_f - 1];
			t->n_other_f--;

			if (find_transponder(freq))
				goto next;

			/* remember tuning to the old frequency failed */
			to = calloc(1, sizeof(*to));
			to->frequency = t->frequency;
			to->wrong_frequency = 1;
			INIT_LIST_HEAD(&to->list);
			INIT_LIST_HEAD(&to->services);
			list_add_tail(&to->list, &scanned_transponders);
			copy_transponder(to, t);

			t->frequency = freq;
			info("retrying with f=%d\n", t->frequency);

			goto retry;
		}
	}
	return -1;
}

struct strtab {
	const char *str;
	int val;
};

static int str2enum(const char *str, const struct strtab *tab, int deflt)
{
	while (tab->str) {
		if (!strcmp(tab->str, str))
			return tab->val;
		tab++;
	}
	error("invalid enum value '%s'\n", str);
	return deflt;
}

static const char * enum2str(int v, const struct strtab *tab, const char *deflt)
{
	while (tab->str) {
		if (v == tab->val)
			return tab->str;
		tab++;
	}
	error("invalid enum value '%d'\n", v);
	return deflt;
}

static enum fe_code_rate str2fec(const char *fec)
{
	struct strtab fectab[] = {
		{ "NONE", FEC_NONE },
		{ "1/2",  FEC_1_2 },
		{ "2/3",  FEC_2_3 },
		{ "3/4",  FEC_3_4 },
		{ "4/5",  FEC_4_5 },
		{ "5/6",  FEC_5_6 },
		{ "6/7",  FEC_6_7 },
		{ "7/8",  FEC_7_8 },
		{ "8/9",  FEC_8_9 },
		{ "AUTO", FEC_AUTO },
		{ NULL, 0 }
	};
	return str2enum(fec, fectab, FEC_AUTO);
}

static enum fe_modulation str2qam(const char *qam)
{
	struct strtab qamtab[] = {
		{ "QPSK",   QPSK },
		{ "QAM16",  QAM_16 },
		{ "QAM32",  QAM_32 },
		{ "QAM64",  QAM_64 },
		{ "QAM128", QAM_128 },
		{ "QAM256", QAM_256 },
		{ "AUTO",   QAM_AUTO },
		{ "8VSB",   VSB_8 },
		{ "16VSB",  VSB_16 },
		{ NULL, 0 }
	};
	return str2enum(qam, qamtab, QAM_AUTO);
}

static enum fe_bandwidth str2bandwidth(const char *bw)
{
	struct strtab bwtab[] = {
		{ "8MHz", BANDWIDTH_8_MHZ },
		{ "7MHz", BANDWIDTH_7_MHZ },
		{ "6MHz", BANDWIDTH_6_MHZ },
		{ "AUTO", BANDWIDTH_AUTO },
		{ NULL, 0 }
	};
	return str2enum(bw, bwtab, BANDWIDTH_AUTO);
}

static enum fe_transmit_mode str2mode(const char *mode)
{
	struct strtab modetab[] = {
		{ "2k",   TRANSMISSION_MODE_2K },
		{ "8k",   TRANSMISSION_MODE_8K },
		{ "AUTO", TRANSMISSION_MODE_AUTO },
		{ NULL, 0 }
	};
	return str2enum(mode, modetab, TRANSMISSION_MODE_AUTO);
}

static enum fe_guard_interval str2guard(const char *guard)
{
	struct strtab guardtab[] = {
		{ "1/32", GUARD_INTERVAL_1_32 },
		{ "1/16", GUARD_INTERVAL_1_16 },
		{ "1/8",  GUARD_INTERVAL_1_8 },
		{ "1/4",  GUARD_INTERVAL_1_4 },
		{ "AUTO", GUARD_INTERVAL_AUTO },
		{ NULL, 0 }
	};
	return str2enum(guard, guardtab, GUARD_INTERVAL_AUTO);
}

static enum fe_hierarchy str2hier(const char *hier)
{
	struct strtab hiertab[] = {
		{ "NONE", HIERARCHY_NONE },
		{ "1",    HIERARCHY_1 },
		{ "2",    HIERARCHY_2 },
		{ "4",    HIERARCHY_4 },
		{ "AUTO", HIERARCHY_AUTO },
		{ NULL, 0 }
	};
	return str2enum(hier, hiertab, HIERARCHY_AUTO);
}

static int tune_initial (int frontend_fd, const char *initial)
{
	FILE *inif;
	unsigned int f, sr;
	char buf[200];
	char pol[20], fec[20], qam[20], bw[20], fec2[20], mode[20], guard[20], hier[20];
	struct transponder *t;

	inif = fopen(initial, "r");
	if (!inif) {
		error("cannot open '%s': %d %m\n", initial, errno);
		return -1;
	}
	while (fgets(buf, sizeof(buf), inif)) {
		if (buf[0] == '#' || buf[0] == '\n')
			;
		else if (sscanf(buf, "S %u %1[HVLR] %u %4s\n", &f, pol, &sr, fec) == 4) {
			t = alloc_transponder(f);
			t->delivery_system = SYS_DVBS;
			t->modulation = QAM_AUTO;
			t->rolloff = ROLLOFF_AUTO;
			t->fec = FEC_AUTO;
			switch(pol[0]) 
			{
			case 'H':
			case 'L':
				t->polarisation = POLARISATION_HORIZONTAL;
				break;
			default:
				t->polarisation = POLARISATION_VERTICAL;;
				break;
			}
			t->inversion = spectral_inversion;
			t->symbol_rate = sr;
			t->fec = str2fec(fec);
			info("initial transponder %u %c %u %d\n",
				t->frequency,
				pol[0], sr,
				t->symbol_rate);
		}
		else if (sscanf(buf, "C %u %u %4s %6s\n", &f, &sr, fec, qam) == 4) {
			t = alloc_transponder(f);
			t->delivery_system = SYS_DVBC_ANNEX_AC;
			t->inversion = spectral_inversion;
			t->symbol_rate = sr;
			t->fec = str2fec(fec);
			t->modulation = str2qam(qam);
			info("initial transponder %u %u %d %d\n",
				t->frequency,
				sr,
				t->fec,
				t->modulation);
		}
		else if (sscanf(buf, "T %u %4s %4s %4s %7s %4s %4s %4s\n",
			&f, bw, fec, fec2, qam, mode, guard, hier) == 8) {
				t = alloc_transponder(f);
				t->delivery_system = SYS_DVBT;
				t->inversion = spectral_inversion;
				t->bandwidth = str2bandwidth(bw);
				t->fecHP = str2fec(fec);
				if (t->fecHP == FEC_NONE)
					t->fecHP = FEC_AUTO;
				t->fecLP = str2fec(fec2);
				if (t->fecLP == FEC_NONE)
					t->fecLP = FEC_AUTO;
				t->modulation = str2qam(qam);
				t->transmission_mode = str2mode(mode);
				t->guard_interval = str2guard(guard);
				t->hierarchy = str2hier(hier);
				info("initial transponder %u %d %d %d %d %d %d %d\n",
					t->frequency,
					t->bandwidth,
					t->fecHP,
					t->fecLP,
					t->modulation,
					t->transmission_mode,
					t->guard_interval,
					t->hierarchy);
		}
		else if (sscanf(buf, "A %u %7s\n",
			&f,qam) == 2) {
				t = alloc_transponder(f);
				t->delivery_system = SYS_ATSC;
				t->modulation = str2qam(qam);
		}
		else
			error("cannot parse'%s'\n", buf);
	}

	fclose(inif);

	return tune_to_next_transponder(frontend_fd);
}


static void scan_tp_atsc(void)
{
	struct section_buf s0,s1,s2;

	if (no_ATSC_PSIP) {
		setup_filter(&s0, demux_devname, 0x00, 0x00, -1, 1, 0, 5); /* PAT */
		add_filter(&s0);
	} else {
		if (ATSC_type & 0x1) {
			setup_filter(&s0, demux_devname, 0x1ffb, 0xc8, -1, 1, 0, 5); /* terrestrial VCT */
			add_filter(&s0);
		}
		if (ATSC_type & 0x2) {
			setup_filter(&s1, demux_devname, 0x1ffb, 0xc9, -1, 1, 0, 5); /* cable VCT */
			add_filter(&s1);
		}
		setup_filter(&s2, demux_devname, 0x00, 0x00, -1, 1, 0, 5); /* PAT */
		add_filter(&s2);
	}

	do {
		read_filters ();
	} while (!(list_empty(&running_filters) &&
		list_empty(&waiting_filters)));
}

static void scan_tp_dvb (void)
{
	struct section_buf s0;
	struct section_buf s1;
	struct section_buf s2;
	struct section_buf s3;

	/**
	*  filter timeouts > min repetition rates specified in ETR211
	*/
	setup_filter (&s0, demux_devname, 0x00, 0x00, -1, 1, 0, 5); /* PAT */
	setup_filter (&s1, demux_devname, 0x11, 0x42, -1, 1, 0, 5); /* SDT */

	add_filter (&s0);
	add_filter (&s1);

	if (!current_tp_only) {
		setup_filter (&s2, demux_devname, 0x10, 0x40, -1, 1, 0, 15); /* NIT */
		add_filter (&s2);
		if (get_other_nits) {
			/* get NIT-others
			* Note: There is more than one NIT-other: one per
			* network, separated by the network_id.
			*/
			setup_filter (&s3, demux_devname, 0x10, 0x41, -1, 1, 1, 15);
			add_filter (&s3);
		}
	}

	do {
		read_filters ();
	} while (!(list_empty(&running_filters) &&
		list_empty(&waiting_filters)));
}

static void scan_tp(int frontend_fd)
{
	struct dtv_property p[] = {
		{ .cmd = DTV_DELIVERY_SYSTEM }
	};

	struct dtv_properties cmdseq = {
		.num = 1,
		.props = p
	};

	if ((ioctl(frontend_fd, FE_GET_PROPERTY, &cmdseq)) == -1) {
		perror("FE_GET_PROPERTY failed");
		return;
	}

	switch(p[0].u.data) 
	{
	case SYS_DVBS:
	case SYS_DVBS2:
	case SYS_DVBC_ANNEX_AC:
	case SYS_DVBC_ANNEX_B:
	case SYS_DVBT:
	case SYS_DSS:
		scan_tp_dvb();
		break;

	case SYS_ATSC:
		scan_tp_atsc();
		break;

	default:
		break;
	}
}

static void scan_network (int frontend_fd, const char *initial)
{
	int rc;

	if (tune_initial (frontend_fd, initial) < 0) {
		error("initial tuning failed\n");
		return;
	}

	do {
		scan_tp(frontend_fd);
		do {
			rc = tune_to_next_transponder(frontend_fd);
		} while(rc == -2);
	} while (rc == 0);
}

static int sat_number (struct transponder *t)
{
	(void) t;

	return switch_pos + uncommitted_switch_pos*4;
}

static void dump_lists (void)
{
	struct list_head *p1, *p2;
	struct transponder *t;
	struct service *s;
	int n = 0, i;
	int cnt = 1;
	char sn[20];
	int anon_services = 0;

	list_for_each(p1, &scanned_transponders) {
		t = list_entry(p1, struct transponder, list);
		if (t->wrong_frequency)
			continue;
		list_for_each(p2, &t->services) {
			n++;
		}
	}
	info("dumping lists (%d services)\n", n);

	list_for_each(p1, &scanned_transponders) {
		t = list_entry(p1, struct transponder, list);
		if (t->wrong_frequency) {
			warning("wrong_frequency\n");
			continue;
		}
		list_for_each(p2, &t->services) {
			s = list_entry(p2, struct service, list);

			if (!s->service_name) {
				/* not in SDT */
				if (unique_anon_services)
					snprintf(sn, sizeof(sn), "[%03x-%04x]",
					anon_services, s->service_id);
				else
					snprintf(sn, sizeof(sn), "[%04x]",
					s->service_id);
				s->service_name = strdup(sn);
				anon_services++;
			}
			/* ':' is field separator in szap and vdr service lists */
			for (i = 0; s->service_name[i]; i++) {
				if (s->service_name[i] == ':')
					s->service_name[i] = ' ';
			}
			for (i = 0; s->provider_name && s->provider_name[i]; i++) {
				if (s->provider_name[i] == ':')
					s->provider_name[i] = ' ';
			}
			if (s->video_pid && !(serv_select & 1)) {
				warning("no TV services\n");
				continue; /* no TV services */
			}
			if (!s->video_pid && s->audio_num && !(serv_select & 2)) {
				warning("no radio services\n");
				continue; /* no radio services */
			}
			if (!s->video_pid && !s->audio_num && !(serv_select & 4)) {
				warning("no data/other services\n");
				continue; /* no data/other services */
			}

			if (s->scrambled && ca_select>0)
				continue; /* FTA only */

			switch (output_format)
			{
			case OUTPUT_VDR:
				if(s->audio_pid[0] == 0 && s->ac3_pid != 0)
					s->audio_pid[0] = s->ac3_pid;

				vdr_dump_service_parameter_set(stdout, s, t, override_orbital_pos, vdr_dump_channum, vdr_dump_provider, ca_select);
				break;

			case OUTPUT_ZAP:
				if(s->audio_pid[0] == 0 && s->ac3_pid != 0)
					s->audio_pid[0] = s->ac3_pid;

				zap_dump_service_parameter_set (stdout, s, t, sat_number(t));
				break;

			default:
				break;
			}
		}
	}
	info("Done.\n");
}

static void show_existing_tuning_data_files(void)
{
#ifndef DATADIR
#define DATADIR "/usr/local/share"
#endif
	static const char* prefixlist[] = { DATADIR "/dvb", "/etc/dvb",
		DATADIR "/doc/packages/dvb", 0 };
	unsigned int i;
	const char **prefix;
	fprintf(stderr, "initial tuning data files:\n");
	for (prefix = prefixlist; *prefix; prefix++) {
		glob_t globbuf;
		char* globspec = malloc (strlen(*prefix)+9);
		strcpy (globspec, *prefix); strcat (globspec, "/dvb-?/*");
		if (! glob (globspec, 0, 0, &globbuf)) {
			for (i=0; i < globbuf.gl_pathc; i++)
				fprintf(stderr, " file: %s\n", globbuf.gl_pathv[i]);
		}
		free (globspec);
		globfree (&globbuf);
	}
}

static void handle_sigint(int sig)
{
	(void)sig;
	error("interrupted by SIGINT, dumping partial result...\n");
	dump_lists();
	exit(2);
}

static const char *usage = "\n"
"usage: %s [options...] [-c | initial-tuning-data-file]\n"
"	atsc/dvbscan doesn't do frequency scans, hence it needs initial\n"
"	tuning data for at least one transponder/channel.\n"
"	-c	scan on currently tuned transponder only\n"
"	-v 	verbose (repeat for more)\n"
"	-q 	quiet (repeat for less)\n"
"	-a N	use DVB /dev/dvb/adapterN/\n"
"	-f N	use DVB /dev/dvb/adapter?/frontendN\n"
"	-d N	use DVB /dev/dvb/adapter?/demuxN\n"
"	-s N	use DiSEqC switch position N (DVB-S only)\n"
"	-S N    use DiSEqC uncommitted switch position N (DVB-S only)\n"
"	-i N	spectral inversion setting (0: off, 1: on, 2: auto [default])\n"
"	-n	evaluate NIT messages for full network scan (slow!)\n"
"	-5	multiply all filter timeouts by factor 5\n"
"		for non-DVB-compliant section repitition rates\n"
"	-O pos	Orbital position override 'S4W', 'S19.2E' - good for VDR output\n"
"	-k cnt	Skip count, will skip every first specified messages for every message type (default 0)\n"
"	-I cnt	Scan iterations count (default 10). Larger number will make scan longer on every channel\n"
"	-o fmt	output format: 'vdr' (default) or 'zap'\n"
"	-x N	Conditional Access, (default -1)\n"
"		N=-2  gets all channels (FTA and encrypted), output received CAID :CAID:\n"
"		N=-1  gets all channels (FTA and encrypted), output CA is set to :0:\n"
"		N=0   gets only FTA channels\n"
"		N=xxx sets ca field in vdr output to :xxx:\n"
"	-t N	Service select, Combined bitfield parameter.\n"
"		1 = TV, 2 = Radio, 4 = Other, (default 7)\n"
"	-p	for vdr output format: dump provider name\n"
"	-e N	VDR version, default 2 for VDR-1.2.x\n"
"		ANYTHING ELSE GIVES NONZERO NIT and TID\n"
"		Vdr version 1.3.x and up implies -p.\n"
"	-l lnb-type (DVB-S Only) (use -l help to print types) or \n"
"	-l low[,high[,switch]] in Mhz\n"
"	-u      UK DVB-T Freeview channel numbering for VDR\n\n"
"	-P do not use ATSC PSIP tables for scanning\n"
"	    (but only PAT and PMT) (applies for ATSC only)\n"
"	-A N	check for ATSC 1=Terrestrial [default], 2=Cable or 3=both\n"
"	-U	Uniquely name unknown services\n";

void bad_usage(char *pname, int problem)
{
	int i;
	struct lnb_types_st *lnbp;
	char **cp;

	switch (problem) 
	{
	default:
	case 0:
		fprintf (stderr, usage, pname);
		break;

	case 1:
		i = 0;
		fprintf(stderr, "-l <lnb-type> or -l low[,high[,switch]] in Mhz\n"
			"where <lnb-type> is:\n");
		while(NULL != (lnbp = lnb_enum(i))) {
			fprintf (stderr, "%s\n", lnbp->name);
			for (cp = lnbp->desc; *cp ; cp++) {
				fprintf (stderr, "   %s\n", *cp);
			}
			i++;
		}
		break;

	case 2:
		show_existing_tuning_data_files();
		fprintf (stderr, usage, pname);
		break;
	}
}

int main (int argc, char **argv)
{
	char frontend_devname [80];
	int adapter = 0, frontend = 0, demux = 0;
	int opt, i;
	int frontend_fd;
	int fe_open_mode;
	const char *initial = NULL;

	if (argc <= 1) {
		bad_usage(argv[0], 2);
		return -1;
	}

	info("API major %d, minor %d\n", DVB_API_VERSION, DVB_API_VERSION_MINOR);

	/* start with default lnb type */
	lnb_type = *lnb_enum(0);
	while ((opt = getopt(argc, argv, "5cnpa:f:d:O:k:I:S:s:o:x:t:i:l:vquPA:U")) != -1) {
		switch (opt) 
		{
		case 'a':
			adapter = strtoul(optarg, NULL, 0);
			break;

		case 'c':
			current_tp_only = 1;
			if (!output_format_set)
				output_format = OUTPUT_VDR;
			break;

		case 'n':
			get_other_nits = 1;
			break;

		case 'd':
			demux = strtoul(optarg, NULL, 0);
			break;

		case 'f':
			frontend = strtoul(optarg, NULL, 0);
			break;

		case 'k':
			skip_count = strtoul(optarg, NULL, 0);
			break;

		case 'I':
			scan_iterations = strtoul(optarg, NULL, 0);
			break;

		case 'p':
			vdr_dump_provider = 1;
			break;

		case 's':
			switch_pos = strtoul(optarg, NULL, 0);
			break;

		case 'S':
			uncommitted_switch_pos = strtoul(optarg, NULL, 0);
			break;

		case 'O':
			strncpy(override_orbital_pos, optarg, sizeof(override_orbital_pos)-1);
			break;

		case 'o':
			if      (strcmp(optarg, "zap") == 0) output_format = OUTPUT_ZAP;
			else if (strcmp(optarg, "vdr") == 0) output_format = OUTPUT_VDR;
			else {
				bad_usage(argv[0], 0);
				return -1;
			}
			output_format_set = 1;
			break;

		case '5':
			long_timeout = 1;
			break;

		case 'x':
			ca_select = strtoul(optarg, NULL, 0);
			break;

		case 't':
			serv_select = strtoul(optarg, NULL, 0);
			break;

		case 'i':
			spectral_inversion = strtoul(optarg, NULL, 0);
			break;

		case 'l':
			if (lnb_decode(optarg, &lnb_type) < 0) {
				bad_usage(argv[0], 1);
				return -1;
			}
			break;

		case 'v':
			verbosity++;
			break;

		case 'q':
			if (--verbosity < 0)
				verbosity = 0;
			break;

		case 'u':
			vdr_dump_channum = 1;
			break;

		case 'P':
			no_ATSC_PSIP = 1;
			break;

		case 'A':
			ATSC_type = strtoul(optarg,NULL,0);
			if (ATSC_type == 0 || ATSC_type > 3) {
				bad_usage(argv[0], 1);
				return -1;
			}
			break;

		case 'U':
			unique_anon_services = 1;
			break;

		default:
			bad_usage(argv[0], 0);
			return -1;
		};
	}

	if (optind < argc)
		initial = argv[optind];
	if ((!initial && !current_tp_only) || (initial && current_tp_only) ||
		(spectral_inversion > 2)) {
			bad_usage(argv[0], 0);
			return -1;
	}

	lnb_type.low_val *= 1000;	/* convert to kiloherz */
	lnb_type.high_val *= 1000;	/* convert to kiloherz */
	lnb_type.switch_val *= 1000;	/* convert to kiloherz */
	if (switch_pos >= 4) {
		fprintf (stderr, "switch position needs to be < 4!\n");
		return -1;
	}
	if (uncommitted_switch_pos >= 16) {
		fprintf (stderr, "uncommitted_switch position needs to be < 16!\n");
		return -1;
	}
	if (initial)
		info("scanning %s\n", initial);

	snprintf (frontend_devname, sizeof(frontend_devname),
		"/dev/dvb/adapter%i/frontend%i", adapter, frontend);

	snprintf (demux_devname, sizeof(demux_devname),
		"/dev/dvb/adapter%i/demux%i", adapter, demux);
	info("using '%s' and '%s'\n", frontend_devname, demux_devname);

	for (i = 0; i < MAX_RUNNING; i++)
		poll_fds[i].fd = -1;

	fe_open_mode = current_tp_only ? O_RDONLY : O_RDWR;
	if ((frontend_fd = open (frontend_devname, fe_open_mode | O_NONBLOCK)) < 0)
		fatal("failed to open '%s': %d %m\n", frontend_devname, errno);

	signal(SIGINT, handle_sigint);

	if (current_tp_only) {
		current_tp = alloc_transponder(0); /* dummy */
		/* move TP from "new" to "scanned" list */
		list_del_init(&current_tp->list);
		list_add_tail(&current_tp->list, &scanned_transponders);
		current_tp->scan_done = 1;

		scan_tp(frontend_fd);
	}
	else
		scan_network (frontend_fd, initial);

	close (frontend_fd);

	dump_lists ();

	return 0;
}

static void dump_dvb_parameters (FILE *f, struct transponder *t)
{
	switch (output_format) 
	{
	case OUTPUT_VDR:
		vdr_dump_dvb_parameters(f, t, override_orbital_pos);	
		break;

	case OUTPUT_ZAP:
		zap_dump_dvb_parameters(f, t, sat_number(t));
		break;

	default:
		break;
	}
}
