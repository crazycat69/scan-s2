#include <stdio.h>
#include <string.h>
#include "dump-vdr.h"
#include "scan.h"

static const char *inv_name [] = {
	"0",
	"1",
	"999"
};

static const char *fec_name [] = {
	"0",
	"12",
	"23",
	"34",
	"45",
	"56",
	"67",
	"78",
	"89",
	"999",
	"35",
	"910"
};

static const char *qam_name [] = {
	"0",
	"16",
	"32",
	"64",
	"128",
	"256",
	"999"
};

static const char *bw_name [] = {
	"8",
	"7",
	"6",
	"999",
	"5",
	"10"
};

static const char *mode_name [] = {
	"2",
	"8",
	"999",
	"4",
	"1",
	"16",
	"32"
};

static const char *guard_name [] = {
	"32",
	"16",
	"8",
	"4",
	"999",
	"1128",
	"19128",
	"19256"
};

static const char *hierarchy_name [] = {
	"0",
	"1",
	"2",
	"4",
	"999"
};

static const char *west_east_flag_name [] = {
	"W",
	"E"
};

static char sat_polarisation(transponder_t *t)
{
	return t->polarisation == POLARISATION_VERTICAL ? 'V' : 'H';
}

extern enum format output_format;

void vdr_dump_dvb_parameters (FILE *f, transponder_t *t, char *orbital_pos_override)
{
	switch (t->delivery_system) {
		case SYS_DVBS:
		case SYS_DVBS2:
			fprintf (f, "%i:", t->frequency / 1000);
			fprintf (f, "%c", sat_polarisation(t));

			switch(t->fec) 
			{
			case FEC_1_2: fprintf (f, "C12"); break;
			case FEC_2_3: fprintf (f, "C23"); break;
			case FEC_3_4: fprintf (f, "C34"); break;
			case FEC_3_5: fprintf (f, "C35"); break;
			case FEC_4_5: fprintf (f, "C45"); break;
			case FEC_5_6: fprintf (f, "C56"); break;
			case FEC_6_7: fprintf (f, "C67"); break;
			case FEC_7_8: fprintf (f, "C78"); break;
			case FEC_8_9: fprintf (f, "C89"); break;
			case FEC_9_10: fprintf (f, "C910"); break;
			}

			if(output_format != OUTPUT_VDR_16x) {
				switch(t->modulation)
				{
				case QPSK: fprintf(f, "M2"); break;
				case QAM_16: fprintf(f, "M16"); break;
				case QAM_32: fprintf(f, "M32"); break;
				case QAM_64: fprintf(f, "M64"); break;
				case QAM_128: fprintf(f, "M128"); break;
				case QAM_256: fprintf(f, "M256"); break;
				case VSB_8: fprintf(f, "M10"); break;
				case VSB_16: fprintf(f, "M11"); break;
				case PSK_8: fprintf(f, "M5"); break;
				case APSK_16: fprintf(f, "M6"); break;
				case APSK_32: fprintf(f, "M7"); break;
				//case DQPSK: ???
				}

				switch(t->rolloff) 
				{
				case ROLLOFF_20: fprintf(f, "O20"); break;
				case ROLLOFF_25: fprintf(f, "O25"); break;
				case ROLLOFF_35: fprintf(f, "O35"); break;
				case ROLLOFF_AUTO:
				default:
				    break;
				}

				if(t->delivery_system == SYS_DVBS2) {
					fprintf (f, "S1");
					if (t->stream_id != NO_STREAM_ID_FILTER)
						fprintf (f, "P%i", ((t->pls_mode & 0x3) << 26) |
								   ((t->pls_code & 0x3ffff) << 8) |
								   (t->stream_id & 0xff));
				}
			}
		
			fprintf(f, ":");

			if(strlen(orbital_pos_override) > 0) {
				fprintf (f, "%s:", orbital_pos_override);
			}
			else {
				fprintf (f, "S%i.%i%s:", t->orbital_pos/10,
					t->orbital_pos % 10, west_east_flag_name[t->we_flag]);
			}

			fprintf (f, "%i:", t->symbol_rate / 1000);
			break;

		case SYS_DVBC_ANNEX_AC:
		case SYS_DVBC_ANNEX_B:
			fprintf (f, "%i:", t->frequency / 1000000);
			fprintf (f, "M%s:C:", qam_name[t->modulation]);
			fprintf (f, "%i:", t->symbol_rate / 1000);
			break;

		case SYS_DVBT:
			fprintf (f, "%i:", t->frequency / 1000);
			fprintf (f, "I%s", inv_name[t->inversion]);
			fprintf (f, "B%s", bw_name[t->bandwidth]);
			fprintf (f, "C%s", fec_name[t->fecHP]);
			fprintf (f, "D%s", fec_name[t->fecLP]);
			fprintf (f, "M%s", qam_name[t->modulation]);
			fprintf (f, "T%s", mode_name[t->transmission_mode]);
			fprintf (f, "G%s", guard_name[t->guard_interval]);
			fprintf (f, "Y%s", hierarchy_name[t->hierarchy]);
			fprintf (f, ":T:27500:");
			break;

		case SYS_DVBT2:
			fprintf (f, "%i:", t->frequency / 1000);
			fprintf (f, "B%s", bw_name[t->bandwidth]);
			fprintf (f, "C%s", fec_name[t->fecHP]);
			fprintf (f, "M%s", qam_name[t->modulation]);
			fprintf (f, "T%s", mode_name[t->transmission_mode]);
			fprintf (f, "G%s", guard_name[t->guard_interval]);
			if(t->delivery_system == SYS_DVBT2)
			{
				fprintf (f, "S1");
				fprintf (f, "P%i", t->stream_id);
			}
			fprintf (f, ":T:27500:");
			break;

		case SYS_ATSC:
			fprintf (f, "%i:", t->frequency / 1000);
			fprintf (f, "M%s", qam_name[t->modulation]);
			fprintf (f, ":A:");
			break;

		default:
			break;
	}
}

void vdr_dump_service_parameter_set (FILE *f, service_t *s, transponder_t *t, char *orbital_pos_override, int dump_channum, int dump_provider, int ca_select)
{
	int i;

	if ((s->video_pid || s->audio_pid[0]) && ((ca_select == -1) || (ca_select < 0) || (ca_select > 0) || ((ca_select == 0) && (s->scrambled == 0)))) {
		if ((dump_channum == 1) && (s->channel_num > 0))
			fprintf(f, ":@%i\n", s->channel_num);

		if (dump_provider == 1) {
			fprintf (f, "%s - ", s->provider_name);
		}
		fprintf (f, "%s;%s:", s->service_name, s->provider_name?s->provider_name:"");

		vdr_dump_dvb_parameters (f, t, orbital_pos_override);

		if ((s->pcr_pid != s->video_pid) && (s->video_pid > 0))
			fprintf (f, "%i+%i:", s->video_pid, s->pcr_pid);
		else
			fprintf (f, "%i:", s->video_pid);

		fprintf (f, "%i", s->audio_pid[0]);

		if (s->audio_lang && s->audio_lang[0][0])
			fprintf (f, "=%.4s", s->audio_lang[0]);

		for (i = 1; i < s->audio_num; i++)
		{
			fprintf (f, ",%i", s->audio_pid[i]);
			if (s->audio_lang && s->audio_lang[i][0])
				fprintf (f, "=%.4s", s->audio_lang[i]);
		}

		if (s->ac3_pid)
		{
			fprintf (f, ";%i", s->ac3_pid);
			if (s->audio_lang && s->audio_lang[0][0])
				fprintf (f, "=%.4s", s->audio_lang[0]);
		}

		fprintf (f, ":%d:", s->teletext_pid);

		/* 0 = FTA only (filtered by first IF in that function), set to 0; -1 = All, but output 0 */
		if(ca_select == -1 || ca_select == 0) { 
			fprintf (f, "0");
		}
		/* -2 = All, output real CAID */
		else if(ca_select == -2) {
			fprintf (f, "%X", s->ca_id[0]);
			for (i = 1; i < s->ca_num; i++) {
				if (s->ca_id[i] == 0) continue;
				fprintf (f, ",%X", s->ca_id[i]);
			}
		}
		/* Other = override CAID with specified value */
		else {
			fprintf (f, "%d", ca_select);
		}

		fprintf (f, ":%d:%d:%d:0", s->service_id, t->original_network_id, t->transport_stream_id);

		fprintf (f, "\n");
	}
}

