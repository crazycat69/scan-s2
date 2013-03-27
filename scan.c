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
#include <iconv.h>
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

#if DVB_API_VERSION < 5 || DVB_API_VERSION_MINOR < 2
#error scan-s2 requires Linux DVB driver API version 5.2 and newer!
#endif

#ifndef DTV_STREAM_ID
	#define DTV_STREAM_ID DTV_ISDBS_TS_ID
#endif

#ifndef NO_STREAM_ID_FILTER
	#define NO_STREAM_ID_FILTER	(~0U)
#endif


#define CRC_LEN		4

enum pid {
	PID_PAT			= 0x0000,
	PID_CAT			= 0x0001,
	PID_TSDT		= 0x0002,
	PID_NIT_ST		= 0x0010,
	PID_SDT_BAT_ST	= 0x0011,
	PID_EIT_STCIT	= 0x0012,
	PID_RST_ST		= 0x0013,
	PID_TDT_TOT_ST	= 0x0014,
	PID_NET_SYNC	= 0x0015,
	PID_RNT			= 0x0016,
	PID_INBAND_SIG	= 0x001C,
	PID_MEASUREMENT	= 0x001D,
	PID_DIT			= 0x001E,
	PID_SIT			= 0x001F,
};

enum table_id {
	TID_PAT			= 0x00,		// Program association table
	TID_CAT			= 0x01,		// Conditional access table
	TID_PMT			= 0x02,		// Program map table
	TID_SDT			= 0x03,		// Stream description table
	// 0x04 .. 0x3F - Reserved
	TID_NIT_ACTUAL	= 0x40,		// Network information table - actual network
	TID_NIT_OTHER	= 0x41,		// Network information table - other network
	TID_SDT_ACTUAL	= 0x42,		// Service description table - actual stream
	// 0x43 .. 0x45 - Reserved
	TID_SDT_OTHER	= 0x46,		// Service description table - other stream
	// 0x47 .. 0x49 - Reserved
	TID_BAT			= 0x4A,		// Bouquet association table
	// 0x4B .. 0x4D - Reserved
	TID_EIT_ACTUAL	= 0x4E,		// Event information table - actual stream - present/following
	TID_EIT_OTHER	= 0x4F,		// Event information table - other stream - present/following
	// 0x50 .. 0x5F					// Event information table - actual stream - schedule
	// 0x60 .. 0x6F					// Event information table - other stream - schedule
	TID_TDT			= 0x70,		// Time date table
	TID_RST			= 0x71,		// Running status table
	TID_ST			= 0x72,		// Stuffing table
	TID_TOT			= 0x73,		// Time offset table
	TID_AIT			= 0x74,		// Application information table
	TID_CT			= 0x75,		// Container table
	TID_RCT			= 0x76,		// Related content table
	TID_CIT			= 0x77,		// Content identifier table
	TID_MPE_FEC		= 0x78,		// MPE-FEC table
	TID_RNT			= 0x79,		// Resolution notification table
	// 0x7A .. 0x7D - Reserved
	TID_DIT			= 0x7E,		// Discountinuity information table
	TID_SIT			= 0x7F,		// Selection information table
	// 0x80 .. 0xFE - User defined
	TID_ATSC_CVT1	= 0xC8,
	TID_ATSC_CVT2	= 0xC9,
	// 0xFF - Reserved
};

enum table_type {
	PAT,
	PMT,
	SDT,
	NIT
};

static char demux_devname[80];

// Configuration parameters
int verbosity = 2;

static int scan_iterations = 10;
static int skip_count = 0;
static int long_timeout;
static int current_tp_only;
static int get_other_nits;
static int noauto=0;
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
static int rotor_pos = 0;
static int curr_rotor_pos = 0;
static char rotor_pos_name[16] = "";
static char override_orbital_pos[16] = "";
enum format output_format = OUTPUT_VDR;
static int output_format_set = 0;
static int disable_s1 = FALSE;
static int disable_s2 = FALSE;

static rotorslot_t rotor[49];

struct section_buf {
	struct list_head list;
	const char *dmx_devname;
	unsigned int run_once  : 1;
	unsigned int segmented : 1;	/* segmented by table_id_ext */
	int fd;
	enum pid pid;
	enum table_id table_id;
	int table_id_ext;
	int section_version_number;
	uint8_t section_done[32];
	int sectionfilter_done;
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
						  enum pid pid, enum table_id tid, int tid_ext,
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

	memset(tp, 0, sizeof(*tp));

	tp->frequency = frequency;

	INIT_LIST_HEAD(&tp->list);
	INIT_LIST_HEAD(&tp->services);
	list_add_tail(&tp->list, &new_transponders);
	return tp;
}

static int is_same_frequency(uint32_t f1, uint32_t f2)
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

static int is_same_transponder(struct transponder *t1, struct transponder *t2)
{
	if(is_same_frequency(t1->frequency, t2->frequency) && t1->polarisation == t2->polarisation && t1->stream_id == t2->stream_id) {
		return 1;
	}
	else {
		return 0;
	}
}

static struct transponder *find_transponder_by_freq(uint32_t frequency)
{
	struct list_head *pos;
	struct transponder *tp;

	list_for_each(pos, &scanned_transponders) {
		tp = list_entry(pos, struct transponder, list);
		if (current_tp_only)
			return tp;

		if (is_same_frequency(tp->frequency, frequency))
			return tp;
	}

	list_for_each(pos, &new_transponders) {
		tp = list_entry(pos, struct transponder, list);

		if (is_same_frequency(tp->frequency, frequency))
			return tp;
	}

	return NULL;
}

static struct transponder *find_transponder(uint32_t frequency, enum polarisation pol)
{
	struct list_head *pos;
	struct transponder *tp;

	list_for_each(pos, &scanned_transponders) {
		tp = list_entry(pos, struct transponder, list);
		if (current_tp_only)
			return tp;

		if (is_same_frequency(tp->frequency, frequency) && tp->polarisation == pol)
			return tp;
	}

	list_for_each(pos, &new_transponders) {
		tp = list_entry(pos, struct transponder, list);

		if (is_same_frequency(tp->frequency, frequency) && tp->polarisation == pol)
			return tp;
	}

	return NULL;
}

static void remove_duplicate_transponder(struct transponder *t)
{
	struct list_head *pos;
	struct transponder *tp;

	list_for_each(pos, &new_transponders) {
		tp = list_entry(pos, struct transponder, list);

		if (is_same_transponder(tp, t) && tp != t) {
			pos = tp->list.prev;
			list_del_init(&tp->list);
		}
	}
}

static void copy_transponder(struct transponder *d, struct transponder *s, int isOverride)
{
	d->network_id = s->network_id;
	d->original_network_id = s->original_network_id;
	d->transport_stream_id = s->transport_stream_id;
	d->frequency = s->frequency;
	d->symbol_rate = s->symbol_rate;
	d->inversion = s->inversion;
	if(isOverride || d->rolloff == ROLLOFF_AUTO) {
		d->rolloff = s->rolloff;
	}
	if(isOverride || d->fec == FEC_AUTO) {
		d->fec = s->fec;
	}
	if(isOverride || d->fecHP == FEC_AUTO) {
		d->fecHP = s->fecHP;
	}
	if(isOverride || d->fecLP == FEC_AUTO) {
		d->fecLP = s->fecLP;
	}
	if(isOverride || d->modulation == QAM_AUTO) {
		d->modulation = s->modulation;
	}
	if(isOverride || d->bandwidth == BANDWIDTH_AUTO) {
		d->bandwidth = s->bandwidth;
	}
	if(isOverride || d->hierarchy == HIERARCHY_AUTO) {
		d->hierarchy = s->hierarchy;
	}
	if(isOverride || d->guard_interval == GUARD_INTERVAL_AUTO) {
		d->guard_interval = s->guard_interval;
	}
	if(isOverride || d->transmission_mode == TRANSMISSION_MODE_AUTO) {
		d->transmission_mode = s->transmission_mode;
	}
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
	if (!t) {
		warning("satellite_delivery_system_descriptor outside transport stream definition (ignored)\n");
		return;
	}

	switch ( getBits(buf,69,1) ) 
	{
	case 0: t->delivery_system = SYS_DVBS; break;
	case 1: t->delivery_system = SYS_DVBS2; break;
	}

	if (t->delivery_system == SYS_DVBS2) 
	{
		switch ( getBits(buf,67,2) ) 
		{
		case 0 : t->rolloff = ROLLOFF_35; break;
		case 1 : t->rolloff = ROLLOFF_25; break;
		case 2 : t->rolloff = ROLLOFF_20; break;
		}
	} 
	else {
		if (noauto) t->rolloff = ROLLOFF_35;
	}

	t->frequency = 10 * bcd32_to_cpu (buf[2], buf[3], buf[4], buf[5]);

	switch ( getBits(buf,100,4) ) 
	{
	case 0 : t->fec = FEC_AUTO; break;
	case 1 : t->fec = FEC_1_2; break;
	case 2 : t->fec = FEC_2_3; break;
	case 3 : t->fec = FEC_3_4; break;
	case 4 : t->fec = FEC_5_6; break;
	case 5 : t->fec = FEC_7_8; break;
	case 6 : t->fec = FEC_8_9; break;
	case 7 : t->fec = FEC_3_5; break;
	case 8 : t->fec = FEC_4_5; break;
	case 9 : t->fec = FEC_9_10; break;
	case 15 : t->fec = FEC_NONE; break;
	}

	t->symbol_rate = 10 * bcd32_to_cpu (buf[9], buf[10], buf[11], buf[12] & 0xf0);

	t->inversion = spectral_inversion;	

	t->polarisation = (buf[8] >> 5) & 0x03;
	t->orbital_pos = bcd32_to_cpu (0x00, 0x00, buf[6], buf[7]);
	t->we_flag = buf[8] >> 7;

	switch ( getBits(buf,70,2) ) 
	{
	case 0 : t->modulation = QAM_AUTO; break;
	case 1 : t->modulation = QPSK; break;
	case 2 : t->modulation = PSK_8; break;
	case 3 : t->modulation = QAM_16; break;
	}

	if (verbosity >= 5) {
		debug("%#04x/%#04x ", t->network_id, t->transport_stream_id);
		dump_dvb_parameters (stderr, t);
		printf("\n");
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


char * dvbtext2utf8(char* dvbtext, int dvbtextlen)
{

	unsigned char *src, *dest;

	char *utf8buf;
	char *utf8out = utf8buf;
	char *utf8in;
	char *utf8res;
	size_t inlen, outlen, convertedlen;
	
	int old_style_conv=0;
	int skip_char=0;

	iconv_t code_desc;
	
	switch (dvbtext[0]) {
		case 0x01: /* ISO-8859-5 */
				skip_char = 1;
				code_desc = iconv_open("UTF-8","ISO8859-5");
			break;
		case 0x02: /* ISO-8859-6 */
				skip_char = 1;
				code_desc = iconv_open("UTF-8","ISO8859-6");
			break;
		case 0x03: /* ISO-8859-7 */
				skip_char = 1;
				code_desc = iconv_open("UTF-8","ISO8859-7");
			break;
		case 0x04: /* ISO-8859-8 */
				skip_char = 1;
				code_desc = iconv_open("UTF-8","ISO8859-8");
			break;
		case 0x05: /* ISO-8859-9 */
				skip_char = 1;
				code_desc = iconv_open("UTF-8","ISO8859-9");
			break;
		case 0x06: /* ISO-8859-10 */
				skip_char = 1;
				code_desc = iconv_open("UTF-8","ISO8859-10");
			break;
		case 0x07: /* ISO-8859-11 */
				skip_char = 1;
				code_desc = iconv_open("UTF-8","ISO8859-11");
			break;
		case 0x08: /* ISO-8859-12 */
				skip_char = 1;
				code_desc = iconv_open("UTF-8","ISO8859-12");
			break;
		case 0x09: /* ISO-8859-13 */
				skip_char = 1;
				code_desc = iconv_open("UTF-8","ISO8859-13");
			break;
		case 0x0a: /* ISO-8859-14 */
				skip_char = 1;
				code_desc = iconv_open("UTF-8","ISO8859-14");
			break;
		case 0x0b: /* ISO-8859-15 */
				skip_char = 1;
				code_desc = iconv_open("UTF-8","ISO8859-15");
			break;
		case 0x0c: /* 0x0C - 0x0F - reserverd for future use */
		case 0x0d: 
		case 0x0e:
		case 0x0f:
				skip_char = 1;
				code_desc = iconv_open("UTF-8","LATIN1");
				break;
		case 0x10:
				skip_char = 3;
				if ( dvbtext[1] != 0x00 ) {
					old_style_conv = 1;
				} else {
					switch (dvbtext[2]) {
							case 0x01: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-1");
								break;
							case 0x02: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-2");
								break;
							case 0x03: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-3");
								break;
							case 0x04: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-4");
								break;
							case 0x05: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-5");
								break;
							case 0x06: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-6");
								break;
							case 0x07: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-7");
								break;
							case 0x08: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-8");
								break;
							case 0x09: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-9");
								break;
							case 0x0a: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-10");
								break;
							case 0x0b: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-11");
								break;
							case 0x0c: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-12");
								break;
							case 0x0d: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-13");
								break;
							case 0x0e: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-14");
								break;
							case 0x0f: /* ISO-8859-1 */
								code_desc = iconv_open("UTF-8","ISO8859-15");
								break;
							default:
								skip_char = 3;
								old_style_conv = 1;
								break;
					}
				}
				break;
		case 0x11: /* ISO/IEC 10646 Basic Multilingual Plane (BMP) */
				skip_char = 1;
				old_style_conv = 1;
				break;
		case 0x12: /* KSX1001-2004 Korean Character Set */
				skip_char = 1;
				old_style_conv = 1;
				break;
		case 0x13: /* GB-2312-1980 Simplified Chinese Character */
				skip_char = 1;
				old_style_conv = 1;
				break;
		case 0x14: /* Big5 subset of ISO/IEC 10646 Traditional Chinese */
				skip_char = 1;
				old_style_conv = 1;
				break;
		case 0x15: /* UTF-8 encoding of ISO/IEC 10646  Basic Multilingual Plane (BMP) */
				skip_char = 1;
				old_style_conv = 1;
				break;
		case 0x16: /* 0x16 - 0x1E - reserverd for future use */
		case 0x17: 
		case 0x18:
		case 0x19:
		case 0x1a:
		case 0x1b:
		case 0x1c:
		case 0x1d:
		case 0x1e:
				skip_char = 1;
				old_style_conv = 1;
				break;
		case 0x1F: /* described by encoding_type_id. TBD */
				skip_char = 1;
				old_style_conv = 1;
				break;
		default:
				skip_char = 0;
				old_style_conv = 1;
			break;
	}
	
	if ( skip_char > 0 )
	{
		memmove(dvbtext,&dvbtext[skip_char],dvbtextlen-skip_char);
		dvbtext[dvbtextlen-skip_char]='\0';
	}

	if ( old_style_conv == 1) 
	{
		
		utf8buf = malloc(dvbtextlen);
		memset( utf8buf, 0, dvbtextlen );
		memcpy ( utf8buf, dvbtext, dvbtextlen );

		for (src = dest = (unsigned char *) utf8buf; *src; src++)
			if (*src >= 0x20 && (*src < 0x80 || *src > 0x9f))
				*dest++ = *src;
		*dest = '\0';
		

		if (!utf8buf[0]) {
			/* zap zero length names */
			if ( utf8buf )
				free(utf8buf);
			utf8res = strdup("\0");
		}	else {
			code_desc = iconv_open("UTF-8","LATIN1");
			if ( code_desc != (iconv_t)(-1) )
			{
				utf8in = strdup(utf8buf);
				inlen = strlen(utf8buf);
				if ( utf8buf )
					free(utf8buf);

					outlen = inlen*2;
				utf8buf = malloc(outlen);
				utf8out = utf8buf;
				memset( utf8buf, 0, outlen);
				errno = 0;
				utf8in = dvbtext;
				convertedlen = iconv( code_desc, &utf8in, &inlen, &utf8out, &outlen);
				utf8res = strdup(utf8buf);
				
				if ( utf8buf )
					free(utf8buf);

				iconv_close(code_desc);

			}
		}
	} 
	else
	{
		if ( code_desc != (iconv_t)(-1) )
		{
			inlen = dvbtextlen-skip_char;
			outlen = inlen*2;
			utf8buf = malloc(outlen);
			utf8out = utf8buf;
			memset( utf8buf, 0, outlen);
			errno = 0;
			utf8in = dvbtext;
			convertedlen = iconv( code_desc, &utf8in, &inlen, &utf8out, &outlen);
			utf8res = strdup(utf8buf);
			if ( utf8buf )
				free(utf8buf);

			iconv_close(code_desc);
		}

	}
	return utf8res;
}

static void parse_service_descriptor (const unsigned char *buf, struct service *s)
{
	unsigned char len;
	unsigned char *src, *dest;
	char* dvbtext;

	//	s->type = buf[2];

	buf += 3;
	len = *buf;
	buf++;

	if (s->provider_name == NULL)
		free (s->provider_name);

	dvbtext = malloc (len + 1);
	memcpy (dvbtext, buf, len);
	dvbtext[len]='\0';
	s->provider_name = dvbtext2utf8(dvbtext,len + 1);

	if (dvbtext == NULL) 
		free(dvbtext);

	if (s->service_name)
		free (s->service_name);

	buf += len;
	len = *buf;
	buf++;

	dvbtext = malloc (len + 1);
	memcpy (dvbtext, buf, len);
	dvbtext[len]='\0';
	s->service_name = dvbtext2utf8(dvbtext,len + 1);

	if (dvbtext == NULL) 
		free(dvbtext);

	info("0x%04X 0x%04X: pmt_pid 0x%04X %s -- %s (%s%s)\n",
		current_tp->transport_stream_id,
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
		unsigned char descriptor_tag = getBits(buf, 0, 8);
		unsigned char descriptor_len = getBits(buf, 8, 8) + 2;

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


static void parse_pat(struct section_buf *sb, const unsigned char *buf, int section_length,
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
				s->pmt_pid, TID_PMT, s->service_id, 1, 0, 5);

			add_filter (s->priv);
		}

skip:
		buf += 4;
		section_length -= 4;
	};
}


static void parse_pmt (struct section_buf *sb, const unsigned char *buf, int section_length, int service_id)
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
		current_tp->transport_stream_id,
		s->service_id,
		s->provider_name, s->service_name,
		s->pmt_pid, s->video_pid, msg_buf);
}


static void parse_nit (struct section_buf *sb, const unsigned char *buf, int section_length, int network_id)
{
	// Update known parameters for current transponder
	if(sb->table_id == TID_NIT_ACTUAL) {
		current_tp->network_id = network_id;
	}

	// Buffer doesn't include all common fields up to last_section_number
	int descriptors_loop_len = getBits(buf, 4, 12);

	// section length doesn't have all common fields and CRC
	if (section_length < descriptors_loop_len + 2)
	{
		warning("section too short: network_id == 0x%04X, section_length == %i, "
			"descriptors_loop_len == %i\n",
			network_id, section_length, descriptors_loop_len);
		return;
	}

	parse_descriptors (NIT, buf + 2, descriptors_loop_len, NULL);

	// advance buffer to skip descriptors
	section_length -= (descriptors_loop_len + 2);
	buf += (descriptors_loop_len + 2);

	int streams_loop_len = getBits(buf, 4, 12);
	if (section_length < streams_loop_len + 2)
	{
		warning("section too short: network_id == 0x%04X, section_length == %i, "
			"steams_loop_len == %i\n",
			network_id, section_length, streams_loop_len);
		return;
	}

	// section_length doesn't include all the descriptors and header
	// advance to skip the loop length
	buf += 2;

	while (streams_loop_len > 6) {
		int transport_stream_id = getBits(buf, 0, 16);

		struct transponder *t, tn;

		descriptors_loop_len = getBits(buf, 36, 12);

		// sream_loop_length include also
		// transport_stream_id		: 16 bit
		// original_network_id		: 16 bit
		// reserved					: 4 bit
		// transport_descriptors_len: 12 bit
		//-----------------------------------
		// total					: 48 bit = 6 byte
		if (streams_loop_len < descriptors_loop_len + 6)
		{
			warning("section too short: transport_stream_id == 0x%04X, "
				"stream_loop_len == %i, streams descriptors_loop_len == %i\n",
				transport_stream_id, streams_loop_len,
				descriptors_loop_len);
			break;
		}

		memset(&tn, 0, sizeof(tn));
		tn.network_id = network_id;
		tn.original_network_id = getBits(buf, 16, 16);
		tn.transport_stream_id = transport_stream_id;
		tn.fec = FEC_AUTO;
		tn.inversion = spectral_inversion;
		tn.modulation = QAM_AUTO;
		tn.rolloff = ROLLOFF_AUTO;

		parse_descriptors (NIT, buf + 6, descriptors_loop_len, &tn);

		t = find_transponder(tn.frequency, tn.polarisation);

		if (t == NULL) {
			if(get_other_nits) {
				// New transponder
				t = alloc_transponder(tn.frequency);

				// For satellites add both DVB-S and DVB-S2 transponders since we don't know what should be used
				if(current_tp->delivery_system == SYS_DVBS || current_tp->delivery_system == SYS_DVBS2) {
					tn.delivery_system = SYS_DVBS;
					copy_transponder(t, &tn, TRUE);

					t = alloc_transponder(tn.frequency);
					tn.delivery_system = SYS_DVBS2;
					copy_transponder(t, &tn, TRUE);
				}
				else {
					copy_transponder(t, &tn, TRUE);
				}
			}
		}
		else {
			// Trasponder exist, update transponder info, don't override known values
			copy_transponder(t, &tn, FALSE);
		}

		streams_loop_len -= (descriptors_loop_len + 6);
		buf += (descriptors_loop_len + 6);
	}
}


static void parse_sdt (struct section_buf *sb, const unsigned char *buf, int section_length,
					   int transport_stream_id)
{
	// buf doesn't include all common fields up to last_section_number
	// section length doesn't have all common fields and CRC

	if(sb->table_id == TID_SDT_ACTUAL) {
		// update current transporter
		current_tp->transport_stream_id = transport_stream_id;
		current_tp->original_network_id = getBits(buf, 0, 16);
	}
	
	buf += 3;	       /*  skip original network id + reserved field */

	while (section_length >= 5) {
		int service_id = getBits(buf, 0, 16);
		int descriptors_loop_len = getBits(buf, 28, 12);
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

		s->running = getBits(buf, 24, 3);
		s->scrambled = getBits(buf, 27, 1);

		parse_descriptors (SDT, buf + 5, descriptors_loop_len, s);

		section_length -= (descriptors_loop_len + 5);
		buf += (descriptors_loop_len + 5);
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

static void parse_psip_vct (struct section_buf *sb, const unsigned char *buf, int section_length,
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
static int parse_section(struct section_buf *sb, unsigned char *buf)
{
	int table_id;
	int section_length;
	int table_id_ext;
	int section_version_number;
	int section_number;
	int last_section_number;
	int i;

	table_id = getBits(buf, 0, 8);

	if (sb->table_id != table_id) {
		info(">>> sb->table_id (%X) != table_id (%X)!\n", sb->table_id, table_id);
		return -1;
	}

	section_length = getBits(buf, 12, 12);

	table_id_ext = getBits(buf, 24, 16);
	section_version_number = getBits(buf, 42, 5);
	section_number = getBits(buf, 48, 8);
	last_section_number = getBits(buf, 56, 8);

	info(">>> parse_section, section number %d out of %d...!\n", section_number, last_section_number);

	if (sb->segmented && sb->table_id_ext != -1 && sb->table_id_ext != table_id_ext) {
		/* find or allocate actual section_buf matching table_id_ext */
		while (sb->next_seg) {
			sb = sb->next_seg;
			if (sb->table_id_ext == table_id_ext)
				break;
		}
		if (sb->table_id_ext != table_id_ext) {
			assert(sb->next_seg == NULL);
			sb->next_seg = calloc(1, sizeof(struct section_buf));
			sb->next_seg->segmented = sb->segmented;
			sb->next_seg->run_once = sb->run_once;
			sb->next_seg->timeout = sb->timeout;
			sb = sb->next_seg;
			sb->table_id = table_id;
			sb->table_id_ext = table_id_ext;
			sb->section_version_number = section_version_number;
		}
	}

	if (sb->section_version_number != section_version_number || sb->table_id_ext != table_id_ext) {
		struct section_buf *next_seg = sb->next_seg;

		if (sb->section_version_number != -1 && sb->table_id_ext != -1) {
			debug("section version_number or table_id_ext changed "
				"%d -> %d / %04x -> %04x\n",
				sb->section_version_number, section_version_number,
				sb->table_id_ext, table_id_ext);
		}

		sb->table_id_ext = table_id_ext;
		sb->section_version_number = section_version_number;
		sb->sectionfilter_done = 0;
		memset (sb->section_done, 0, sizeof(sb->section_done));
		sb->next_seg = next_seg;
	}

	buf += 8;			/* past generic table header */
	section_length -= CRC_LEN;	// Reduce CRC
	section_length -= 5;		// Reduce common part of the messages up to last_section_number
	if (section_length < 0) {
		warning("truncated section (PID 0x%04X, lenght %d)",
			sb->pid, section_length + CRC_LEN);
		return 0;
	}

	if (!get_bit(sb->section_done, section_number)) {
		set_bit (sb->section_done, section_number);

		debug("pid 0x%02X tid 0x%02X table_id_ext 0x%04X, "
			"%i/%i (version %i)\n",
			sb->pid, table_id, table_id_ext, section_number,
			last_section_number, section_version_number);

		switch (table_id) 
		{
		case TID_PAT:
			verbose("PAT\n");
			parse_pat (sb, buf, section_length, table_id_ext);
			break;

		case TID_PMT:
			verbose("PMT 0x%04X for service 0x%04X\n", sb->pid, table_id_ext);
			parse_pmt (sb, buf, section_length, table_id_ext);
			break;

		case TID_NIT_OTHER:
			verbose("NIT (other TS)\n");
			parse_nit (sb, buf, section_length, table_id_ext);
			break;

		case TID_NIT_ACTUAL:
			verbose("NIT (actual TS)\n");
			parse_nit (sb, buf, section_length, table_id_ext);
			break;

		case TID_SDT_ACTUAL:
		case TID_SDT_OTHER:
			verbose("SDT (%s TS)\n", table_id == 0x42 ? "actual":"other");
			parse_sdt (sb, buf, section_length, table_id_ext);
			break;

		case TID_ATSC_CVT1:
		case TID_ATSC_CVT2:
			verbose("ATSC VCT\n");
			parse_psip_vct(sb, buf, section_length, table_id, table_id_ext);
			break;

		default:
			break;
		};

		for (i = 0; i <= last_section_number; i++)
			if (get_bit (sb->section_done, i) == 0)
				break;

		if (i > last_section_number)
			sb->sectionfilter_done = 1;
	}

	if (sb->segmented) {
		/* always wait for timeout; this is because we don't now how
		* many segments there are
		*/
		return 0;
	}
	else if (sb->sectionfilter_done)
		return 1;

	return 0;
}

static int read_sections (struct section_buf *sb)
{
	unsigned char buffer[4096];	// CAT can be up to 4K, the rest are 1K max
	int section_length, count;

	if (sb->sectionfilter_done && !sb->segmented)
		return 1;

	memset(buffer, 0, sizeof(buffer));

	/* the section filter API guarantess that we get one full section
	* per read(), provided that the buffer is large enough (it is)
	*/
	if (((count = read (sb->fd, buffer, sizeof(buffer))) < 0) && errno == EOVERFLOW)
		count = read (sb->fd, buffer, sizeof(buffer));
	if (count < 0) {
		errorn("read_sections: read error");
		return -1;
	}

	if(sb->skip_count > 0) {
		info("skipping section, table_id %X, pid %X\n", sb->table_id, sb->pid);
		sb->skip_count--;
		return -1;
	}

	if (count < 4)
		return -1;

	section_length = getBits(buffer, 12, 12);

	if (count != section_length + 3) {
		error("Ignoring section, read %d, while section length + 3 = %d\n", count, section_length + 3);
		return -1;
	}

	int i;
	for(i=0; i<count; i++) {
		debug("0x%02X ", buffer[i]);
		if((i+1)%10 == 0) {
			verbosedebug("\n");
		}
	}
	debug("\n");

	if (parse_section(sb, buffer) == 1)
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
						  enum pid pid, enum table_id tid, int tid_ext,
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
	struct section_buf *sb;
	int i, n, done;

	n = poll(poll_fds, n_running, 1000);
	if (n == -1)
		errorn("poll");

	for (i = 0; i < n_running; i++) {
		sb = poll_section_bufs[i];
		if (!sb)
			fatal("poll_section_bufs[%d] is NULL\n", i);
		if (poll_fds[i].revents)
			done = read_sections (sb) == 1;
		else
			done = 0; /* timeout */
		if (done || time(NULL) > sb->start_time + sb->timeout) {
			if (sb->run_once) {
				if (done)
					verbosedebug("filter done pid 0x%04X\n", sb->pid);
				else
					warning("filter timeout pid 0x%04X\n", sb->pid);
				remove_filter (sb);
			}
		}
	}
}

static int __tune_to_transponder (int frontend_fd, struct transponder *t)
{
	int rc;
	int i;
	fe_status_t s;
	uint32_t if_freq = 0;
	uint32_t bandwidth_hz = 0;
	current_tp = t;
	int hiband = 0;

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

	switch(t->delivery_system) 
	{
	case SYS_DVBS:
	case SYS_DVBS2:
		if (lnb_type.high_val) {
			if (lnb_type.switch_val) {
				/* Voltage-controlled switch */
				hiband = 0;

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
		if (verbosity >= 2) {
			dprintf(1,"DVB-S IF freq is %d\n", if_freq);
		}

		if (rotor_pos != 0 ) {
			/* Rotate DiSEqC 1.2 rotor to correct orbital position */
			if (t->orbital_pos!=0) rotor_pos = rotor_nn(t->orbital_pos, t->we_flag);
			int err;
			err = rotate_rotor(	frontend_fd,
						curr_rotor_pos, 
						rotor_pos,
						t->polarisation == POLARISATION_VERTICAL ? 0 : 1,
						hiband);
			if (err)
				error("Error in rotate_rotor err=%i\n",err); 
			else
				curr_rotor_pos = rotor_pos;
		}
		break;

	case SYS_DVBT:
		if_freq = t->frequency;

		switch(t->bandwidth) 
		{
		case BANDWIDTH_6_MHZ:	bandwidth_hz = 6000000; break;
		case BANDWIDTH_7_MHZ:	bandwidth_hz = 7000000; break;
		case BANDWIDTH_8_MHZ:	bandwidth_hz = 8000000; break;
		case BANDWIDTH_AUTO:	bandwidth_hz = 0; break;
		default:				bandwidth_hz = 0; break;
		}

		if (verbosity >= 2){
			dprintf(1,"DVB-T frequency is %d\n", if_freq);
			dprintf(1,"DVB-T bandwidth is %d\n", bandwidth_hz);
		}
		break;

	case SYS_DVBT2:
		if_freq = t->frequency;

		switch(t->bandwidth) 
		{
		case BANDWIDTH_5_MHZ:	bandwidth_hz = 5000000; break;
		case BANDWIDTH_6_MHZ:	bandwidth_hz = 6000000; break;
		case BANDWIDTH_7_MHZ:	bandwidth_hz = 7000000; break;
		case BANDWIDTH_8_MHZ:	bandwidth_hz = 8000000; break;
		case BANDWIDTH_10_MHZ:	bandwidth_hz = 10000000; break;
		case BANDWIDTH_AUTO:	bandwidth_hz = 0; break;
		default:				bandwidth_hz = 0; break;
		}

		if (verbosity >= 2){
			dprintf(1,"DVB-T2 frequency is %d\n", if_freq);
			dprintf(1,"DVB-T2 bandwidth is %d\n", bandwidth_hz);
		}
		break;

	case SYS_DVBC_ANNEX_B:
	case SYS_DVBC_ANNEX_AC:
		if_freq = t->frequency;

		if (verbosity >= 2){
			dprintf(1,"DVB-C frequency is %d\n", if_freq);
		}
		break;
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
		{ .cmd = DTV_BANDWIDTH_HZ,		.u.data = bandwidth_hz },
		{ .cmd = DTV_PILOT,			.u.data = PILOT_AUTO },
		{ .cmd = DTV_STREAM_ID,		.u.data = t->stream_id },
		{ .cmd = DTV_TUNE },
	};
	struct dtv_properties cmdseq_tune = {
		.num = sizeof(p_tune)/sizeof(p_tune[0]),
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

	// wait for zero status indicating start of tunning
	do {
		ioctl(frontend_fd, FE_GET_EVENT, &ev);
	}
	while(ev.status != 0);

	// Wait for tunning
	for (i = 0; i < scan_iterations; i++) {
		usleep (200000);

		if (ioctl(frontend_fd, FE_GET_EVENT, &ev) == -1) {
			// no answer, consider it as not locked situation
			ev.status = 0;
		}

		verbose(">>> tuning status == 0x%02X\n", ev.status);

		// Tuning succeed
		if(ev.status & FE_HAS_LOCK) {
			t->last_tuning_failed = 0;

			/* Remove duplicate entries for the same frequency that were created for other delivery systems */
			remove_duplicate_transponder(t);

#ifdef READ_PARAMS
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
#endif

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

	case SYS_DVBT2:
		info("----------------------------------> Using DVB-T2\n");
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

			if (find_transponder_by_freq(freq))
				goto next;

			/* remember tuning to the old frequency failed */
			to = calloc(1, sizeof(*to));
			to->frequency = t->frequency;
			to->wrong_frequency = 1;
			INIT_LIST_HEAD(&to->list);
			INIT_LIST_HEAD(&to->services);
			list_add_tail(&to->list, &scanned_transponders);
			copy_transponder(to, t, FALSE);

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

struct strtab fectab[] = {
	{ "NONE",	FEC_NONE },
	{ "1/2",	FEC_1_2 },
	{ "2/3",	FEC_2_3 },
	{ "3/4",	FEC_3_4 },
	{ "3/5",	FEC_3_5 },
	{ "4/5",	FEC_4_5 },
	{ "5/6",	FEC_5_6 },
	{ "6/7",	FEC_6_7 },
	{ "7/8",	FEC_7_8 },
	{ "8/9",	FEC_8_9 },
	{ "9/10",	FEC_9_10 },
	{ "AUTO",	FEC_AUTO },
	{ NULL, 0 }
};

static enum fe_code_rate str2fec(const char *fec)
{
	return str2enum(fec, fectab, FEC_AUTO);
}

static const char* fec2str(enum fe_code_rate fec)
{
	return enum2str(fec, fectab, "???");
}

struct strtab rollofftab[] = {
	{ "20",  ROLLOFF_20 },
	{ "25",  ROLLOFF_25 },
	{ "35",  ROLLOFF_35 },
	{ "AUTO", ROLLOFF_AUTO },
	{ NULL, 0 }
};

static enum fe_rolloff str2rolloff(const char *rolloff)
{
	return str2enum(rolloff, rollofftab, ROLLOFF_AUTO);
}

static const char* rolloff2str(enum fe_rolloff rolloff)
{
	return enum2str(rolloff, rollofftab, "???");
}

struct strtab qamtab[] = {
	{ "AUTO",	QAM_AUTO },
	{ "QAM16",	QAM_16 },
	{ "QAM32",	QAM_32 },
	{ "QAM64",	QAM_64 },
	{ "QAM128",	QAM_128 },
	{ "QAM256",	QAM_256 },
	{ "8VSB",	VSB_8 },
	{ "16VSB",	VSB_16 },
	{ "QPSK",	QPSK },
	{ "8PSK",	PSK_8 },
	{ "16APSK",	APSK_16 },
	{ "32APSK",	APSK_32 },
	{ NULL, 0 }
};

static enum fe_modulation str2qam(const char *qam)
{
	return str2enum(qam, qamtab, QAM_AUTO);
}

static const char* qam2str(enum fe_modulation qam)
{
	return enum2str(qam, qamtab, "???");
}

struct strtab bwtab[] = {
	{ "8MHz", BANDWIDTH_8_MHZ },
	{ "7MHz", BANDWIDTH_7_MHZ },
	{ "6MHz", BANDWIDTH_6_MHZ },
	{ "5MHz", BANDWIDTH_5_MHZ },
	{ "10MHz", BANDWIDTH_10_MHZ },
	{ "AUTO", BANDWIDTH_AUTO },
	{ NULL, 0 }
};

static enum fe_bandwidth str2bandwidth(const char *bw)
{
	return str2enum(bw, bwtab, BANDWIDTH_AUTO);
}

static const char* bandwidth2str(enum fe_bandwidth bw)
{
	return enum2str(bw, bwtab, "???");
}

struct strtab modetab[] = {
	{ "1k",   TRANSMISSION_MODE_1K },
	{ "2k",   TRANSMISSION_MODE_2K },
	{ "4k",   TRANSMISSION_MODE_4K },
	{ "8k",   TRANSMISSION_MODE_8K },
	{ "16k",  TRANSMISSION_MODE_16K },
	{ "32k",  TRANSMISSION_MODE_32K },
	{ "AUTO", TRANSMISSION_MODE_AUTO },
	{ NULL, 0 }
};

static enum fe_transmit_mode str2mode(const char *mode)
{
	return str2enum(mode, modetab, TRANSMISSION_MODE_AUTO);
}

static const char* mode2str(enum fe_transmit_mode mode)
{
	return enum2str(mode, modetab, "???");
}

struct strtab guardtab[] = {
	{ "1/32", GUARD_INTERVAL_1_32 },
	{ "1/16", GUARD_INTERVAL_1_16 },
	{ "1/8",  GUARD_INTERVAL_1_8 },
	{ "1/4",  GUARD_INTERVAL_1_4 },
	{ "1/128",  GUARD_INTERVAL_1_128 },
	{ "19/128",  GUARD_INTERVAL_19_128 },
	{ "19/256",  GUARD_INTERVAL_19_256 },
	{ "AUTO", GUARD_INTERVAL_AUTO },
	{ NULL, 0 }
};

static enum fe_guard_interval str2guard(const char *guard)
{
	return str2enum(guard, guardtab, GUARD_INTERVAL_AUTO);
}

static const char* guard2str(enum fe_guard_interval guard)
{
	return enum2str(guard, guardtab, "???");
}

struct strtab hiertab[] = {
	{ "NONE", HIERARCHY_NONE },
	{ "1",    HIERARCHY_1 },
	{ "2",    HIERARCHY_2 },
	{ "4",    HIERARCHY_4 },
	{ "AUTO", HIERARCHY_AUTO },
	{ NULL, 0 }
};

static enum fe_hierarchy str2hier(const char *hier)
{
	return str2enum(hier, hiertab, HIERARCHY_AUTO);
}

static const char* hier2str(enum fe_hierarchy hier)
{
	return enum2str(hier, hiertab, "???");
}

static int read_rotor_conf(const char *rotor_conf)
{
	FILE *rotor_conf_fd;
	unsigned int nn;
	char buf[200], angle_we[20], angle[20], we[2];
	int i = -1;
	rotor_conf_fd = fopen (rotor_conf, "r");
	if (!rotor_conf_fd){
		error("Cannot open rotor configuration file '%s'.\n", rotor_conf);
		return errno;
	}
	while (fgets(buf, sizeof(buf), rotor_conf_fd)) {
		if (buf[0] != '#' && buf[0] != '\n') {
			if (sscanf(buf, "%u %s\n", &nn, angle_we)==2) {
				i++;
				rotor[i].nn = nn;
				strcpy(rotor[i].angle_we,angle_we);
				strncpy(angle,angle_we,strlen(angle_we)-1);
				rotor[i].orbital_pos = atof(angle) * 10;
				strncpy(we,angle_we+strlen(angle_we)-1,1);
				we[1]='\0';
				rotor[i].we_flag = (strcmp(we,"W")==0 || strcmp(we,"w")==0) ? 0 : 1;
				//info("rotor: i=%i, nn=%i, orbital_pos=%i we_flag=%i\n", 
				//	i, rotor[i].nn, rotor[i].orbital_pos, rotor[i].we_flag);
			}
		}
	}
	fclose(rotor_conf_fd);
	return 0;
}

int rotor_nn(int orbital_pos, int we_flag){
	/*given say 192,1 return the position number*/
	int i;
	for (i=0; i<49; i++){
		if (rotor[i].orbital_pos == orbital_pos && rotor[i].we_flag == we_flag) {
			return rotor[i].nn;
		}
	}
	error("rotor_nn: orbital_pos=%i, we_flag=%i not found.\n", orbital_pos, we_flag);
	return 0;
}

int rotor_name2nn(char *angle_we){
	/*given say '19.2E' return the position number*/
	int i;
	for (i=0; i<49; i++){
		if (strcmp(rotor[i].angle_we, angle_we) == 0) {
			return rotor[i].nn;
		}
	}
	error("rotor_name2nn: '%s' not found.\n", angle_we);
	return 0;
}

float rotor_angle(int nn) {
	/*given nn, return the angle in 0.0-359.9 range (1=1.0E, 359=1.0W) */
	int i;
	float angle;
	for (i=0; i<49; i++){
		if (rotor[i].nn == nn) {
			if(rotor[i].we_flag == 0) //west
				angle = 360.00 - rotor[i].orbital_pos / 10;
			else //east
				angle = rotor[i].orbital_pos / 10;
			return angle;
		}
	}
	error("rotor_angle: nn=%i not found",nn);
	return -999;
}

static int tune_initial (int frontend_fd, const char *initial)
{
	FILE *inif;
	unsigned int f, sr;
	char buf[1024];
	char pol[20], fec[20], qam[20], bw[20], fec2[20], mode[20], guard[20], hier[20], rolloff[20], scan_mode;
	struct transponder *t;
	struct transponder *t2;
	int scan_mode1 = FALSE;
	int scan_mode2 = FALSE;
	int stream_id;

	inif = fopen(initial, "r");
	if (!inif) {
		error("cannot open '%s': %d %m\n", initial, errno);
		return -1;
	}
	while (fgets(buf, sizeof(buf), inif)) {
		scan_mode = 0;
		memset(pol, 0, sizeof(pol));
		memset(fec, 0, sizeof(fec));
		memset(qam, 0, sizeof(qam));
		memset(bw, 0, sizeof(bw));
		memset(fec2, 0, sizeof(fec2));
		memset(mode, 0, sizeof(mode));
		memset(guard, 0, sizeof(guard));
		memset(hier, 0, sizeof(hier));
		memset(rolloff, 0, sizeof(rolloff));
		stream_id = NO_STREAM_ID_FILTER;

		if (buf[0] == '#' || buf[0] == '\n')
			;
		else if (sscanf(buf, "S%c %u %1[HVLR] %u %4s %4s %6s %i\n", &scan_mode, &f, pol, &sr, fec, rolloff, qam, &stream_id) >= 3) {
			scan_mode1 = FALSE;
			scan_mode2 = FALSE;
			switch(scan_mode)
			{
			case '1':
				/* Enable only DVB-S mode */
				if (!disable_s1) scan_mode1 = TRUE;
				break;

			case '2':
				/* Enable only DVB-S2 mode */
				if (!disable_s2) scan_mode2 = TRUE;
				break;

			default:
				/* Enable both DVB-S and DVB-S2 scan modes */
				if (!disable_s1) scan_mode1 = TRUE;
				if (!disable_s2) scan_mode2 = TRUE;
				break;
			}

			/*Generate a list of transponders, explicitly enumerating the AUTOs if they
			are disabled with the -X parameter.*/

			int nmod,nfec,ndel,nrol;
			int imod,ifec,idel,irol;

			/* set up list of delivery systems*/
			fe_delivery_system_t delset[]={SYS_DVBS,SYS_DVBS2};
			ndel=2;
			if (!scan_mode1 && !scan_mode2) continue;
			if (scan_mode1 && !scan_mode2) {delset[0]=SYS_DVBS ; ndel=1;}
			if (!scan_mode1 && scan_mode2) {delset[0]=SYS_DVBS2; ndel=1;}

			/* set up list of modulations*/
			fe_modulation_t modset[2]={ QPSK, PSK_8 };
			nmod=2;
			if (strlen(qam)>0) {
				modset[0]=str2qam(qam); nmod=1;
			} else if (noauto) { 
				if (scan_mode1 && !scan_mode2 ) nmod=1;
			} else {
				modset[0]=QAM_AUTO; nmod=1;
			}
			
			/* Check MIS 0-255 */
			if (stream_id<0 || stream_id>255)
				stream_id = NO_STREAM_ID_FILTER;

			/* set up list of rollofs*/			
			fe_rolloff_t rolset[3]={ROLLOFF_35,ROLLOFF_25,ROLLOFF_20};
			nrol=3;
			if (strlen(rolloff)>0) {
				rolset[0]=str2rolloff(rolloff); nrol=1;
			} else if (noauto) { 
				if (scan_mode1 && ! scan_mode2) nrol=1;
			} else {
				rolset[0]=ROLLOFF_AUTO; nrol=1;
			}

			/* set up list of FECs*/
			fe_code_rate_t fecset[9]={FEC_1_2,FEC_2_3,FEC_3_4,FEC_5_6,FEC_7_8,FEC_8_9,FEC_3_5,FEC_4_5,FEC_9_10};
			if (strlen(fec)>0) {
				fecset[0]=str2fec(fec); nfec=1;
			} else if (noauto) { 
				if (scan_mode1) nfec=6;
				if (scan_mode2) nfec=9;
			} else {
				fecset[0]=FEC_AUTO; nfec=1;
			}

			for (idel=0;idel<ndel;idel++){
				for (ifec=0;ifec<nfec;ifec++){
					for (irol=0;irol<nrol;irol++){
						for (imod=0;imod<nmod;imod++){
							/*skip impossible settings*/
							if ((rolset[irol]==ROLLOFF_25||rolset[irol]==ROLLOFF_20) && delset[idel]!=SYS_DVBS2) continue;
							if (ifec > 5 && delset[idel]!=SYS_DVBS2) continue;
							if (modset[imod] == PSK_8 && delset[idel] != SYS_DVBS2) continue;

							t = alloc_transponder(f);

							t->delivery_system = delset[idel];
							t->modulation = modset[imod];
							t->rolloff = rolset[irol];
							t->fec = fecset[ifec];
							t->stream_id = stream_id;

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

							info("initial transponder DVB-S%s %u %c %d %s %s %s %i\n",
								t->delivery_system==SYS_DVBS?" ":"2",
								t->frequency,
								pol[0], t->symbol_rate, fec2str(t->fec), rolloff2str(t->rolloff), qam2str(t->modulation), stream_id);
						}
					}
				}
			}
		}
		else if (sscanf(buf, "C %u %u %4s %6s\n", &f, &sr, fec, qam) >= 2) {
			t = alloc_transponder(f);
			t->delivery_system = SYS_DVBC_ANNEX_AC;
			t->inversion = spectral_inversion;
			t->symbol_rate = sr;
			t->fec = FEC_AUTO;
			t->modulation = QAM_AUTO;

			// parse optional parameters
			if(strlen(fec) > 0) {
				t->fec = str2fec(fec);
			}

			if(strlen(qam) > 0) {
				t->modulation = str2qam(qam);
			}

			info("initial transponder %u %u %s %s\n",
				t->frequency,
				sr,
				fec2str(t->fec),
				qam2str(t->modulation));
		}
		else if (sscanf(buf, "T%c %u %4s %4s %4s %7s %4s %4s %4s %i\n",
			&scan_mode, &f, bw, fec, fec2, qam, mode, guard, hier, &stream_id) >= 2) {
				t = alloc_transponder(f);
				t->delivery_system = scan_mode == '2' ? SYS_DVBT2 : SYS_DVBT;
				t->inversion = spectral_inversion;
				t->bandwidth = BANDWIDTH_AUTO;
				t->fecHP = FEC_AUTO;
				t->fecLP = FEC_AUTO;
				t->modulation = QAM_AUTO;
				t->transmission_mode = TRANSMISSION_MODE_AUTO;
				t->guard_interval = GUARD_INTERVAL_AUTO;
				t->hierarchy = HIERARCHY_AUTO;

				// parse optional parameters
				if(strlen(bw) > 0) {
					t->bandwidth = str2bandwidth(bw);
				}

				if(strlen(fec) > 0) {
					t->fecHP = str2fec(fec);
					if (t->fecHP == FEC_NONE)
						t->fecHP = FEC_AUTO;
				}

				if(strlen(fec2) > 0) {
					t->fecLP = str2fec(fec2);
					if (t->fecLP == FEC_NONE)
						t->fecLP = FEC_AUTO;
				}

				if(strlen(qam) > 0) {
					t->modulation = str2qam(qam);
				}

				if(strlen(mode) > 0) {
					t->transmission_mode = str2mode(mode);
				}

				if(strlen(guard) > 0) {
					t->guard_interval = str2guard(guard);
				}

				if(strlen(hier) > 0) {
					t->hierarchy = str2hier(hier);
				}

				/* Check PLP 0-255 */
				if (stream_id<0 || stream_id>255)
					stream_id = NO_STREAM_ID_FILTER;

				t->stream_id = stream_id;

				info("initial transponder %u %s %s %s %s %s %s %s %i\n",
					t->frequency,
					bandwidth2str(t->bandwidth),
					fec2str(t->fecHP),
					fec2str(t->fecLP),
					qam2str(t->modulation),
					mode2str(t->transmission_mode),
					guard2str(t->guard_interval),
					hier2str(t->hierarchy),
					t->stream_id);
		}
		else if (sscanf(buf, "A %u %7s\n", &f, qam) >= 1) {
			t = alloc_transponder(f);
			t->delivery_system = SYS_ATSC;
			t->modulation = QAM_AUTO;

			// parse optional parameters
			if(strlen(qam) > 0) {
				t->modulation = str2qam(qam);
			}
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
		setup_filter(&s0, demux_devname, PID_PAT, TID_PAT, -1, 1, 0, 5); /* PAT */
		add_filter(&s0);
	} else {
		if (ATSC_type & 0x1) {
			setup_filter(&s0, demux_devname, 0x1ffb, TID_ATSC_CVT1, -1, 1, 0, 5); /* terrestrial VCT */
			add_filter(&s0);
		}
		if (ATSC_type & 0x2) {
			setup_filter(&s1, demux_devname, 0x1ffb, TID_ATSC_CVT2, -1, 1, 0, 5); /* cable VCT */
			add_filter(&s1);
		}
		setup_filter(&s2, demux_devname, PID_PAT, TID_PAT, -1, 1, 0, 5); /* PAT */
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
	setup_filter (&s0, demux_devname, PID_PAT, TID_PAT, -1, 1, 0, 5); /* PAT */
	setup_filter (&s1, demux_devname, PID_SDT_BAT_ST, TID_SDT_ACTUAL, -1, 1, 0, 5); /* SDT */

	add_filter (&s0);
	add_filter (&s1);

	if (!current_tp_only) {
		setup_filter (&s2, demux_devname, PID_NIT_ST, TID_NIT_ACTUAL, -1, 1, 0, 15); /* NIT */
		add_filter (&s2);
		if (get_other_nits) {
			/* get NIT-others
			* Note: There is more than one NIT-other: one per
			* network, separated by the network_id.
			*/
			setup_filter (&s3, demux_devname, PID_NIT_ST, TID_NIT_OTHER, -1, 1, 1, 15);
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

			if (s->scrambled && ca_select==0)
				continue; /* FTA only */

			if(s->audio_pid[0] == 0 && s->ac3_pid != 0)
				s->audio_pid[0] = s->ac3_pid;

			switch (output_format)
			{
			case OUTPUT_VDR:
				vdr_dump_service_parameter_set(stdout, s, t, override_orbital_pos, vdr_dump_channum, vdr_dump_provider, ca_select);
				break;

			case OUTPUT_VDR_16x:
				if(t->delivery_system != SYS_DVBS2) {
					vdr_dump_service_parameter_set(stdout, s, t, override_orbital_pos, vdr_dump_channum, vdr_dump_provider, ca_select);
				}
				break;

			case OUTPUT_ZAP:
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
"	-r sat  move DiSEqC rotor to satellite location, e.g. '13.0E' or '1.0W'\n"
"	-R N    move DiSEqC rotor to position number N\n"
"	-i N	spectral inversion setting (0: off, 1: on, 2: auto [default])\n"
"	-n	evaluate NIT messages for full network scan (slow!)\n"
"	-5	multiply all filter timeouts by factor 5\n"
"		for non-DVB-compliant section repitition rates\n"
"	-O pos	Orbital position override 'S4W', 'S19.2E' - good for VDR output\n"
"	-k cnt	Skip count: skip the first cnt \n"
"		messages of each message type (default 0)\n"
"	-I cnt	Scan iterations count (default 10).\n"
"		Larger number will make scan longer on every channel\n"
"	-o fmt	output format: 'vdr' (default), 'vdr16x' for VDR version 1.6.x or 'zap'\n"
"	-x N	Conditional Access, (default -1)\n"
"		N=-2  gets all channels (FTA and encrypted),\n"
"		      output received CAID :CAID:\n"
"		N=-1  gets all channels (FTA and encrypted),\n"
"		      output CA is set to :0:\n"
"		N=0   gets only FTA channels\n"
"		N=xxx  sets ca field in vdr output to :xxx:\n"
"	-t N  Service select, Combined bitfield parameter.\n"
"		1 = TV, 2 = Radio, 4 = Other, (default 7)\n"
"	-p	for vdr output format: dump provider name\n"
"	-e N  VDR version, default 2 for VDR-1.2.x\n"
"		ANYTHING ELSE GIVES NONZERO NIT and TID\n"
"		Vdr version 1.3.x and up implies -p.\n"
"	-l lnb-type (DVB-S Only) (use -l help to print types) or \n"
"	-l low[,high[,switch]] in Mhz\n"
"	-u UK DVB-T Freeview channel numbering for VDR\n\n"
"	-P do not use ATSC PSIP tables for scanning\n"
"	    (but only PAT and PMT) (applies for ATSC only)\n"
"	-A N	check for ATSC 1=Terrestrial [default], 2=Cable or 3=both\n"
"	-U	Uniquely name unknown services\n"
"	-D s	Disable specified scan mode (by default all modes are enabled)\n"
"		s=S1  Disable DVB-S scan\n"
"		s=S2  Disable DVB-S2 scan (good for owners of cards that do not\n"
"		      support DVB-S2 systems)\n"
"	-X	Disable AUTOs for initial transponders (esp. for hardware which\n"
"		not support it). Instead try each value of any free parameters.\n";


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
	while ((opt = getopt(argc, argv, "5cnXpa:f:d:O:k:I:S:s:r:R:o:D:x:t:i:l:vquPA:U")) != -1) {
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

		case 'X':
			noauto = 1;
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

		case 'r':
			strncpy(rotor_pos_name,optarg,sizeof(rotor_pos_name)-1);
			break;

		case 'R':
			rotor_pos = strtoul(optarg, NULL, 0);
			break;

		case 'O':
			strncpy(override_orbital_pos, optarg, sizeof(override_orbital_pos)-1);
			break;

		case 'o':
			if      (strcmp(optarg, "zap") == 0) output_format = OUTPUT_ZAP;
			else if (strcmp(optarg, "vdr") == 0) output_format = OUTPUT_VDR;
			else if (strcmp(optarg, "vdr16x") == 0) output_format = OUTPUT_VDR_16x;
			else {
				bad_usage(argv[0], 0);
				return -1;
			}
			output_format_set = 1;
			break;

		case 'D':
			if      (strcmp(optarg, "S1") == 0) disable_s1 = TRUE;
			else if (strcmp(optarg, "S2") == 0) disable_s2 = TRUE;
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

	if(read_rotor_conf("rotor.conf") == 0) {
		if (strlen(rotor_pos_name)>0){
			rotor_pos=rotor_name2nn(rotor_pos_name);
			if (rotor_pos == 0){
				fprintf(stderr,"Rotor position '%s' not found. Check config.",rotor_pos_name);
				return -1;
			}
		}
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
		struct dtv_property p[] = {
			{ .cmd = DTV_FREQUENCY },
			{ .cmd = DTV_DELIVERY_SYSTEM },
			{ .cmd = DTV_MODULATION },
			{ .cmd = DTV_SYMBOL_RATE },
			{ .cmd = DTV_INNER_FEC },
			{ .cmd = DTV_INVERSION },
			{ .cmd = DTV_ROLLOFF },
			{ .cmd = DTV_BANDWIDTH_HZ },
			{ .cmd = DTV_STREAM_ID },
		};

		struct dtv_properties cmdseq = {
			.num = sizeof(p)/sizeof(p[0]),
			.props = p
		};

		/* query for currently tuned parameters */
		if ((ioctl(frontend_fd, FE_GET_PROPERTY, &cmdseq)) == -1) {
			perror("FE_GET_PROPERTY failed");
			return;
		}
		current_tp = alloc_transponder(p[0].u.data);
		current_tp->delivery_system = p[1].u.data;
		current_tp->modulation = p[2].u.data;
		current_tp->symbol_rate = p[3].u.data;
		current_tp->fec = p[4].u.data;
		current_tp->inversion = p[5].u.data;
		current_tp->rolloff = p[6].u.data;
		current_tp->stream_id = p[8].u.data;

		switch(p[6].u.data) 
		{
		case 6000000: current_tp->bandwidth = BANDWIDTH_6_MHZ; break;
		case 7000000: current_tp->bandwidth = BANDWIDTH_7_MHZ; break;
		case 8000000: current_tp->bandwidth = BANDWIDTH_8_MHZ; break;
		default:
		case 0:	current_tp->bandwidth = BANDWIDTH_AUTO; break;
		}

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
	case OUTPUT_VDR_16x:
		vdr_dump_dvb_parameters(f, t, override_orbital_pos);	
		break;

	case OUTPUT_ZAP:
		zap_dump_dvb_parameters(f, t, sat_number(t));
		break;

	default:
		break;
	}
}
