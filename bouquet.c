#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <fnmatch.h>

#include "list.h"
#include "scan.h"
#include "section.h"
#include "htable.h"

extern char * dvbtext2utf8(char* dvbtext, int dvbtextlen);

// only video/audio streams for VDR output
#define SERVICE_CHECK(s)		(((((s)->video_pid != 0) | (((s)->audio_pid[0] != 0) << 1)) & ctx->serv_select) && !(ctx->ca_select == 0 && (s)->scrambled))

struct bouquet {
	struct htable_entry hash;
	unsigned char hash_key[2];
	int bouquet_id;
	char *bouquet_name;
	char *bouquet_ml_name;
	struct htable services;
};

struct bouquet_service {
	struct htable_entry hash;
	unsigned char hash_key[6];
	struct transponder *tp;
	struct service *sp;
};

#define	STAGE_PRE_SCAN		1
#define	STAGE_POST_SCAN		2
#define	STAGE_PRE_OUTPUT	4

struct bouquet_ctx;

struct bouquet_option_params {
	const char *option_name;
	int min_args;
	int max_args;
	int priority;
	int stage;
	void (*action)(struct bouquet_ctx *ctx, int argc, char **argv);
};

struct bouquet_option {
	char **argv;
	int argc;
	struct bouquet_option_params *params;
};

struct bouquet_config {
	const char **languages;
	struct bouquet_option **options;
	// option strings are allocated here
	char *opt_buf;
};

struct bouquet_ctx {
	struct bouquet_config cfg;
	struct htable bouquets;
	struct list_head *scanned_transponders;
	int serv_select;
	int ca_select;
};

#define ARRAY(var, type, size_incr)	struct {type (*buf); int len; int _size; int _incr;} var = {NULL, 0, 0, (size_incr)}
#define ARRAY_RESET(arr)	(arr).len = 0
#define ARRAY_APPEND(arr, val)			\
	if ((arr).len == (arr)._size) {		\
		(arr)._size += (arr)._incr;		\
		(arr).buf = realloc((arr).buf, (arr)._size * sizeof(*((arr).buf)));	\
	}									\
	(arr).buf[(arr).len++] = val
#define ARRAY_FOREACH(arr, var, action) 			\
do {												\
	int __idx;										\
	for (__idx = 0; __idx < (arr).len; __idx++) {	\
		var = (arr).buf[__idx];						\
		{action}									\
	}												\
} while(0)
#define ARRAY_CLEAN(arr)		free((arr).buf); (arr).buf = NULL; (arr)._size = 0; (arr).len = 0

static void bouquet_init_key(struct bouquet *bp, int id)
{
	bp->hash_key[0] = id & 0xff;
	bp->hash_key[1] = id>>8 & 0xff;
	htable_entry_init(&bp->hash, bp->hash_key, sizeof(bp->hash_key));
}

static void init_service(struct bouquet_service *s, int nid, int tid, int sid)
{
	s->hash_key[0] = sid & 0xff;
	s->hash_key[1] = sid>>8 & 0xff;
	s->hash_key[2] = tid & 0xff;
	s->hash_key[3] = tid>>8 & 0xff;
	s->hash_key[4] = nid & 0xff;
	s->hash_key[5] = nid>>8 & 0xff;
	htable_entry_init(&s->hash, s->hash_key, sizeof(s->hash_key));
	s->tp = NULL;
	s->sp = NULL;
}

static struct bouquet *bouquet_entry_create(int id, const char *name, const char *ml_name)
{
	struct bouquet *bp;

	bp = malloc(sizeof(struct bouquet));
	bp->bouquet_id = id;
	bp->bouquet_name = (name)? strdup(name) : NULL;
	bp->bouquet_ml_name = (ml_name)? strdup(ml_name) : NULL;
	bouquet_init_key(bp, id);
	htable_init(&bp->services, 256, 0);
	return bp;
}

static void bouquet_entry_free(struct bouquet *bp)
{
	struct bouquet_service *sp;

	if (bp->bouquet_name) free(bp->bouquet_name);
	if (bp->bouquet_ml_name) free(bp->bouquet_ml_name);
	HTABLE_FOREACH(&bp->services, sp, struct bouquet_service, hash,
		free(sp);
	);
	htable_free(&bp->services);
	free(bp);
}

// strip leading and trailing space
static char *strip_space(char *s)
{
	char *e = s + strlen(s) - 1;
	while (s <= e && (*s == ' ' || *s == '\t'))
		s++;
	while (e > s && (*e == ' ' || *e == '\t'))
		e--;
	*(e + 1) = '\0';
	return s;
}

static int pattern_match(const char *pattern, const char *string)
{
	return fnmatch(pattern, string, 0);
}

// lang=string,string,...
static void opt_lang(struct bouquet_ctx *ctx, int argc, char **argv)
{
	int i;

	ctx->cfg.languages = malloc(argc * sizeof(char *));
	for (i = 1; i < argc; i++)
		ctx->cfg.languages[i - 1] = argv[i];
	ctx->cfg.languages[argc - 1] = NULL;
}

// merge=target,pattern,pattern,...
static void opt_merge(struct bouquet_ctx *ctx, int argc, char **argv)
{
	char *target_name = argv[1];
	struct bouquet *bp, *target_ptr = NULL;
	struct bouquet_service *sp;
	int i;

	HTABLE_FOREACH(&ctx->bouquets, bp, struct bouquet, hash,
		for (i = 2; i < argc; i++) {
			if (pattern_match(argv[i], bp->bouquet_name) == 0)
				break;
		}
		if (i < argc) {
			// matched
			if (target_ptr == NULL) {
				// becomes a new target
				target_ptr = bp;
				free(bp->bouquet_name);
				bp->bouquet_name = strdup(target_name);
			} else {
				HTABLE_FOREACH(&bp->services, sp, struct bouquet_service, hash,
					htable_remove_noresize(&bp->services, &sp->hash);
					if (htable_lookup(&target_ptr->services, &sp->hash) == NULL)
						htable_insert(&target_ptr->services, &sp->hash);
					else
						free(sp);
				);
				htable_remove_noresize(&ctx->bouquets, &bp->hash);
				bouquet_entry_free(bp);
			}
		}
	);
}

// rename=old,new
static void opt_rename(struct bouquet_ctx *ctx, int argc, char **argv)
{
	struct bouquet *bp;

	HTABLE_FOREACH(&ctx->bouquets, bp, struct bouquet, hash,
		if (strcmp(argv[1], bp->bouquet_name) == 0) {
			free(bp->bouquet_name);
			bp->bouquet_name = strdup(argv[2]);
			break;
		}
	);
}

//  add=bouquet,pattern,pattern,...
// move=bouquet,pattern,pattern,...
static void opt_add(struct bouquet_ctx *ctx, int argc, char **argv)
{
	struct list_head *p1, *p2;
	struct transponder *tp;
	struct service *sp;
	struct bouquet *bp, *bp2;
	struct bouquet_service *bsp, *bsp2;
	int i, found;

	found = 0;
	HTABLE_FOREACH(&ctx->bouquets, bp, struct bouquet, hash,
		if (strcmp(argv[1], bp->bouquet_name) == 0) {
			found = 1;
			break;
		}
	);
	if (!found) {
		bp = bouquet_entry_create(-1, argv[1], NULL);
		htable_insert(&ctx->bouquets, &bp->hash);
	}
	list_for_each(p1, ctx->scanned_transponders) {
		tp = list_entry(p1, struct transponder, list);
		list_for_each(p2, &tp->services) {
			sp = list_entry(p2, struct service, list);
			if (!SERVICE_CHECK(sp))
				continue;
			for (i = 2; i < argc; i++) {
				if (pattern_match(argv[i], sp->service_name) == 0)
					break;
			}
			if (i < argc) {
				// matched
				struct htable_entry *e;
				bsp = malloc(sizeof(struct bouquet_service));
				init_service(bsp, tp->original_network_id, tp->transport_stream_id, sp->service_id);
				if ((e = htable_lookup(&bp->services, &bsp->hash)) != NULL) {
					free(bsp);
					bsp = container_of(e, struct bouquet_service, hash);
				} else {
					htable_insert(&bp->services, &bsp->hash);
				}
				bsp->tp = tp;
				bsp->sp = sp;
				if (strcmp(argv[0], "move") == 0) {
					// remove from other bouquets
					HTABLE_FOREACH(&ctx->bouquets, bp2, struct bouquet, hash,
						if (bp2 == bp)
							continue;
						e = htable_remove(&bp2->services, &bsp->hash);
						if (e) {
							bsp2 = container_of(e, struct bouquet_service, hash);
							free(bsp2);
						}
					);
				}
			}
		}
	}
}

struct substring_pair {
	char *buf;
	int len;
};

static char *replace(char *src, char *sub, char *repl)
{
	char *start, *end, *pos;
	int newlen, sublen, replen, modified, i;
	struct substring_pair p;
	ARRAY(chunks, struct substring_pair, 16);

	start = src;
	end = start + strlen(start);
	sublen = strlen(sub);
	replen = strlen(repl);
	if (sublen == 0)
		return NULL;
	modified = 0;
	while (start < end) {
		if ((pos = strstr(start, sub)) != NULL) {
			modified = 1;
			p.buf = start; p.len = pos - start; ARRAY_APPEND(chunks, p);
			p.buf = repl; p.len = replen; ARRAY_APPEND(chunks, p);
			start = pos + sublen;
		} else
			break;
	}
	if (!modified)
		return NULL;
	p.buf = start; p.len = end - start; ARRAY_APPEND(chunks, p);
	for (newlen = 0, i = 0; i < chunks.len; i++)
		newlen += chunks.buf[i].len;
	start = malloc(newlen + 1);
	for (pos = start, i = 0; i < chunks.len; i++)
		if (chunks.buf[i].len) {
			memcpy(pos, chunks.buf[i].buf, chunks.buf[i].len);
			pos += chunks.buf[i].len;
		}
	start[newlen] = '\0';
	ARRAY_CLEAN(chunks);
	return start;
}

// s=substring,substitution
static void opt_s(struct bouquet_ctx *ctx, int argc, char **argv)
{
	struct list_head *p1, *p2;
	struct transponder *tp;
	struct service *sp;
	char *p;

	list_for_each(p1, ctx->scanned_transponders) {
		tp = list_entry(p1, struct transponder, list);
		list_for_each(p2, &tp->services) {
			sp = list_entry(p2, struct service, list);
			if (!SERVICE_CHECK(sp))
				continue;
			if ((p = replace(sp->service_name, argv[1], argv[2])) != NULL) {
				free(sp->service_name);
				sp->service_name = p;
			}
		}
	}
}

// remove=pattern,pattern,...
// ignore=pattern,pattern,...
static void opt_remove(struct bouquet_ctx *ctx, int argc, char **argv)
{
	struct bouquet *bp;
	int i;

	HTABLE_FOREACH(&ctx->bouquets, bp, struct bouquet, hash,
		for (i = 1; i < argc; i++) {
			if (pattern_match(argv[i], bp->bouquet_name) == 0) {
				htable_remove_noresize(&ctx->bouquets, &bp->hash);
				bouquet_entry_free(bp);
			}
		}
	);
}

static struct bouquet_option_params option_params[] = {
	{"lang", 1, -1, 10, STAGE_PRE_SCAN, opt_lang},
	{"merge", 2, -1, 10, STAGE_POST_SCAN, opt_merge},
	{"rename", 2, 2, 10, STAGE_POST_SCAN, opt_rename},
	{"add", 2, -1, 20, STAGE_POST_SCAN, opt_add},
	{"move", 2, -1, 20, STAGE_POST_SCAN, opt_add},
	{"s", 2, 2, 30, STAGE_POST_SCAN, opt_s},
	{"ignore", 1, -1, 10, STAGE_POST_SCAN, opt_remove},
	{"remove", 1, -1, 10, STAGE_PRE_OUTPUT, opt_remove},
};

static int cmp_option(const void *a, const void *b){
	return (*((struct bouquet_option **)b))->params->priority - (*((struct bouquet_option **)a))->params->priority;
}

static int bouquet_parse_options(struct bouquet_config *cfg, const char *optstring)
{
	struct bouquet_option *opt;
	char *s1, *s2, *tok1, *tok2, *save1, *save2;
	struct bouquet_option_params *params;
	int i, ret;
	ARRAY(args, char *, 16);
	ARRAY(opts, struct bouquet_option *, 16);

	cfg->opt_buf = strdup(optstring);
	for (s1 = cfg->opt_buf; (tok1 = strtok_r(s1, ";", &save1)) != NULL; s1 = NULL) {
		ARRAY_RESET(args);
		s2 = strchr(tok1, '=');
		if (s2 != NULL)
			*s2++ = '\0';
		else
			s2 = tok1 + strlen(tok1);
		ARRAY_APPEND(args, tok1);
		for (; (tok2 = strtok_r(s2, ",", &save2)) != NULL; s2 = NULL) {
			ARRAY_APPEND(args, tok2);
		}
		params = NULL;
		for (i = 0; i < sizeof(option_params)/sizeof(struct bouquet_option_params); i++) {
			if (strcmp(args.buf[0], option_params[i].option_name) == 0) {
				params = &option_params[i];
				break;
			}
		}
		if (params) {
			if (
				(params->min_args >= 0 && args.len - 1 < params->min_args) ||
				(params->max_args >= 0 && args.len - 1 > params->max_args)
			) {
				error("Invalid number of arguments for bouquet option '%s'\n", args.buf[0]);
				goto error;
			}
		} else {
			error("Invalid bouquet option: '%s'\n", args.buf[0]);
			goto error;
		}
		opt = malloc(sizeof(struct bouquet_option));
		opt->argc = args.len;
		opt->argv = malloc((opt->argc + 1) * sizeof(char *));
		for (i = 0; i < opt->argc; i++)
			opt->argv[i] = args.buf[i];
		opt->argv[opt->argc] = NULL;
		opt->params = params;
		ARRAY_APPEND(opts, opt);
	}
	qsort(opts.buf, opts.len, sizeof(struct bouquet_option *), cmp_option);
	ARRAY_APPEND(opts, NULL);
	cfg->options = opts.buf;
	ret = 0;
	goto cleanup;
error:
	ret = -1;
	for (i = 0; i < opts.len; i++)
		free(opts.buf[i]->argv);
	ARRAY_CLEAN(opts);
cleanup:
	ARRAY_CLEAN(args);
	return ret;
}

static void bouquet_process_options(struct bouquet_ctx *ctx, int stage)
{
	struct bouquet_option **opt;

	for (opt = ctx->cfg.options; *opt != NULL; opt++) {
		if ((*opt)->params->stage & stage)
			(*opt)->params->action(ctx, (*opt)->argc, (*opt)->argv);
	}
}

void bouquet_free(struct bouquet_ctx *ctx)
{
	struct bouquet_option **opt;

	if (ctx->cfg.languages) free(ctx->cfg.languages);
	if (ctx->cfg.opt_buf) free(ctx->cfg.opt_buf);
	if (ctx->cfg.options) {
		for (opt = ctx->cfg.options; *opt != NULL; opt++)
			free((*opt)->argv);
		free(ctx->cfg.options);
	}
	htable_free(&ctx->bouquets);
	free(ctx);
}

struct bouquet_ctx *bouquet_create(const char *optstring)
{
	struct bouquet_ctx *ctx;

	ctx = malloc(sizeof(struct bouquet_ctx));

	ctx->cfg.languages = NULL;
	ctx->cfg.options = NULL;
	ctx->cfg.opt_buf = NULL;

	htable_init(&ctx->bouquets, 32, 0);

	if (bouquet_parse_options(&ctx->cfg, optstring) != 0) {
		bouquet_free(ctx);
		return NULL;
	}

	bouquet_process_options(ctx, STAGE_PRE_SCAN);

	return ctx;
}

const char *bouquet_help_msg()
{
	static const char *msg = "\n"
		"Options for organizing and fixing channel groups (aka bouquets).\n"
		"Format: OPTION=VALUE,VALUE,...;OPTION=VALUE,VALUE,...;...\n"
		"\n"
		"Each option can be used more than once. Patterns used for name matching are \n"
		"shell path patterns, see fnmatch(3).\n"
		"\n"
		"Options:\n"
		"\n"
		"merge=TARGET,PATTERN,PATTERN,...\n"
		"    Merge channel groups matching PATTERN into a new group TARGET.\n"
		"\n"
		"remove=PATTERN,PATTERN,...\n"
		"ignore=PATTERN,PATTERN,...\n"
		"   Remove channel groups matching PATTERN. The only difference between 'ignore'\n"
		"   and 'remove' is that with 'ignore' channels which belongs to a group to \n"
		"   remove and don't belong to any other group are added to the 'UNSORTED' group.\n"
		"\n"
		"rename=OLD,NEW\n"
		"    Rename a channel group OLD to NEW.\n"
		"\n"
		"add=GROUP,PATTERN,PATTERN,...\n"
		"    Add channels matching PATTERN to GROUP. If this group doesn't exist\n"
		"    a new group is created.\n"
		"\n"
		"move=GROUP,PATTERN,PATTERN,...\n"
		"   Same as 'add' but remove matching channels from their original groups.\n"
		"\n"
		"s=SUBSTRING,REPLACEMENT\n"
		"   Replace all occurrences of SUBSTRING in channel names. No patterns here.\n"
		"\n"
		"lang=LANG,...\n"
		"    Set languages for multilingual bouquet names. LANG is a three character\n"
		"    language code as defined by ISO 639-2, e.g. lang=deu,ger. You should\n"
		"    set this option if your provider broadcasts bouquet names in your native\n"
		"    language as multilingual.\n"
		"\n"
		"\n";
	return msg;
}

static char *name_utf8(const unsigned char *desc_buf, int desc_len)
{
	char *r;
	char *dvbtext = malloc(desc_len + 1);
	memcpy(dvbtext, desc_buf, desc_len);
	dvbtext[desc_len] = '\0';
	r = dvbtext2utf8(dvbtext, desc_len + 1);
	free(dvbtext);
	return r;
}

static void add_service(struct bouquet *bp, int nid, int tid, int sid, int stype)
{
	struct bouquet_service s, *sp;

	init_service(&s, nid, tid, sid);
	if (htable_lookup(&bp->services, &s.hash) == NULL) {
		sp = malloc(sizeof(struct bouquet_service));
		init_service(sp, nid, tid, sid);
		htable_insert(&bp->services, &sp->hash);
		debug("== bouquet(%d):%s<--tid=%d,sid=%d,nid=%d,type=%d\n", bp->bouquet_id, (bp->bouquet_name)? bp->bouquet_name : "NULL", tid, sid, nid, stype);
	}
}

#define DESCRIPTORS_VARS \
const unsigned char *desc_buf;	\
int desc_tag, desc_len, descriptors_loop_len;

// input: buf, descriptors_loop_len
// sets (desc_tag, desc_buf, desc_len) for every descriptor
// action can have 'break'
// advances buf pointer
#define DESCRIPTORS_LOOP(action)		\
do {									\
while (descriptors_loop_len > 0) {		\
	desc_tag = (unsigned)buf[0];		\
	desc_len = (unsigned)buf[1];		\
	desc_buf = buf + 2;					\
	descriptors_loop_len -= desc_len + 2;	\
	buf += desc_len + 2;				\
	{ action }							\
}										\
buf += descriptors_loop_len;			\
} while(0)

void bouquet_parse_bat(struct bouquet_ctx *ctx, const unsigned char *buf, 
		int section_length, int bouquet_id, int version_number)
{
	DESCRIPTORS_VARS

	const unsigned char *transports_end;
	int tid, sid, nid, service_type, name_found;
	int transports_loop_len, ml_name_len;
	char lang[4];
	const char **lp;

	struct bouquet b, *bp;
	struct htable_entry *p;

	bouquet_init_key(&b, bouquet_id);
	if ((p = htable_lookup(&ctx->bouquets, &b.hash)) == NULL) {
		bp = bouquet_entry_create(bouquet_id, NULL, NULL);
		htable_insert(&ctx->bouquets, &bp->hash);
	} else {
		bp = container_of(p, struct bouquet, hash);
	}

	// bouquet descriptors - look for a bouquet name
	descriptors_loop_len = getBits(buf, 4, 12);
	buf += 2;

	name_found = 0;
	DESCRIPTORS_LOOP(
		switch (desc_tag) {
			// bouquet name
			case 0x47:
				name_found = 1;
				if (!bp->bouquet_name)
					bp->bouquet_name = name_utf8(desc_buf, desc_len);
				break;
			// multilingual bouquet name
			case 0x5c:
				name_found = 1;
				if (!bp->bouquet_ml_name) {
					while (desc_len > 0) {
						memcpy(lang, desc_buf, 3);
						lang[3] = '\0';
						ml_name_len = getBits(desc_buf + 3, 0, 8);
						desc_buf += 4;
						for (lp = ctx->cfg.languages; lp != NULL; lp++) {
							if (pattern_match(*lp, lang) == 0) {
								bp->bouquet_ml_name = name_utf8(desc_buf, ml_name_len);
								break;
							}
						}
						info("  Multilingual bouquet name (%s)\n", lang);
						desc_buf += ml_name_len;
						desc_len -= ml_name_len + 4;
					}
				}
				break;
			// Some providers put their service list into a user-defined
			// descriptor in the bouquet descriptors section and set
			// transport_stream_loop_length to 0. The internal format of
			// this user-defined descriptor is provider specific.

			// Tricolor TV service list. (tid_16, nid_16, sid_16)
			case 0x86:
				for (;desc_len >= 6; desc_len -= 6, desc_buf += 6) {
					tid = getBits(desc_buf + 0, 0, 16);
					nid = getBits(desc_buf + 2, 0, 16);
					sid = getBits(desc_buf + 4, 0, 16);
					add_service(bp, nid, tid, sid, -1);
				}
				break;
		}
	);
	if (!name_found)
		warning("parse_bat(): bouquet name descriptor not found, bouquet_id: %d\n", bouquet_id);

	transports_loop_len = getBits(buf, 4, 12);
	buf += 2;
	transports_end = buf + transports_loop_len;

	// transport stream loop
	while (buf < transports_end) {
		tid = getBits(buf, 0, 16);
		nid = getBits(buf + 2, 0, 16);
		descriptors_loop_len = getBits(buf + 4, 4, 12);
		buf += 6;
		// service list descriptors loop (actually only one descriptor)
		DESCRIPTORS_LOOP(
			if (desc_tag == 0x41) {
				// service list
				for (;desc_len >= 3; desc_len -= 3, desc_buf += 3) {
					sid = getBits(desc_buf, 0, 16);
					service_type = getBits(desc_buf + 2, 0, 8);
					add_service(bp, nid, tid, sid, service_type);
				}
			}
		);
	}
}

struct bouquet_service_pair {
	struct transponder *tp;
	struct service *sp;
};

static int cmp_bouquet_name(const void *a, const void *b)
{
	//return (*(struct bouquet **)a)->bouquet_id - (*(struct bouquet **)b)->bouquet_id;
	return strcmp(
		(*(struct bouquet **)a)->bouquet_name,
		(*(struct bouquet **)b)->bouquet_name
	);
}

static int cmp_bouquet_service(const void *a, const void *b)
{
//	return ((struct bouquet_service_pair *)a)->sp->service_id - ((struct bouquet_service_pair *)b)->sp->service_id;
	return strcmp(
		((struct bouquet_service_pair *)a)->sp->service_name,
		((struct bouquet_service_pair *)b)->sp->service_name
	);
}

#define STRIP_SPACE(var, tmpvar) 	\
	if ((var)) {					\
		tmpvar = strip_space(var);	\
		if ((tmpvar) != (var)) {	\
			free(var);				\
			var = strdup(tmpvar);	\
		}							\
	} else {						\
		var = strdup("");			\
	}


void bouquet_dump(struct bouquet_ctx *ctx, struct list_head *scanned_transponders,
		int ca_select, int serv_select,
		void (*dump_service_cb)(struct transponder *, struct service *))
{
	struct list_head *p1, *p2;
	struct transponder *tp;
	struct service *sp;
	struct bouquet *bp;
	struct bouquet_service bs, *bsp;
	struct bouquet_service_pair pair;
	int n_bouquets, n_mapped, n_unmapped;
	char *str;
	struct htable uniq_channels;

	ARRAY(barr, struct bouquet *, 32);
	ARRAY(sarr, struct bouquet_service_pair, 1000);
	ARRAY(uarr, struct bouquet_service_pair, 1000);

	// set bouquet name, strip leading and trailing spaces
	HTABLE_FOREACH(&ctx->bouquets, bp, struct bouquet, hash,
		if (bp->bouquet_ml_name) {
			if (bp->bouquet_name)
				free(bp->bouquet_name);
			bp->bouquet_name = bp->bouquet_ml_name;
			bp->bouquet_ml_name = NULL;
		}
		STRIP_SPACE(bp->bouquet_name, str);
	);

	// strip leading and trailing spaces
	list_for_each(p1, scanned_transponders) {
		tp = list_entry(p1, struct transponder, list);
		list_for_each(p2, &tp->services) {
			sp = list_entry(p2, struct service, list);
			STRIP_SPACE(sp->service_name, str);
		}
	}

	ctx->scanned_transponders = scanned_transponders;
	ctx->serv_select = serv_select;
	ctx->ca_select = ca_select;
	bouquet_process_options(ctx, STAGE_POST_SCAN);

	n_bouquets = 0;
	n_mapped = 0;
	n_unmapped = 0;

	// map to bouquet services
	list_for_each(p1, scanned_transponders) {
		tp = list_entry(p1, struct transponder, list);
		list_for_each(p2, &tp->services) {
			struct htable_entry *entry;
			int mapped;

			sp = list_entry(p2, struct service, list);
			if (!SERVICE_CHECK(sp))
				continue;
			init_service(&bs, tp->original_network_id, tp->transport_stream_id, sp->service_id);
			mapped = 0;
			HTABLE_FOREACH(&ctx->bouquets, bp, struct bouquet, hash,
				if ((entry = htable_lookup(&bp->services, &bs.hash)) != NULL) {
					bsp = container_of(entry, struct bouquet_service, hash);
					bsp->tp = tp;
					bsp->sp = sp;
					mapped = 1;
					//debug("-- MAP:%s->%s=\n", sp->service_name, bp->bouquet_name);
				}
			);
			if (!mapped) {
				struct bouquet_service_pair tmp = {tp, sp};
				ARRAY_APPEND(uarr, tmp);
				n_unmapped++;
				//debug("-- UNSORTED:%s\n", sp->service_name);
			} else
				n_mapped++;
		}
	}

	bouquet_process_options(ctx, STAGE_PRE_OUTPUT);

	HTABLE_FOREACH(&ctx->bouquets, bp, struct bouquet, hash,
		ARRAY_APPEND(barr, bp);
		n_bouquets++;
	);
	qsort(barr.buf, barr.len, sizeof(struct bouquet *), cmp_bouquet_name);

	htable_init(&uniq_channels, 256, 0);

	// output per bouquet
	ARRAY_FOREACH(barr, bp,
		ARRAY_RESET(sarr);
		HTABLE_FOREACH(&bp->services, bsp, struct bouquet_service, hash,
			struct bouquet_service_pair tmp;
			if (bsp->tp == NULL || bsp->sp == NULL)
				continue;
			tmp.tp = bsp->tp;
			tmp.sp = bsp->sp;
			ARRAY_APPEND(sarr, tmp);
			if (!htable_lookup(&uniq_channels, &bsp->hash))
				htable_insert(&uniq_channels, &bsp->hash);
		);
		if (sarr.len) {
			qsort(sarr.buf, sarr.len, sizeof(struct bouquet_service_pair), cmp_bouquet_service);
			fprintf(stdout, ":%s\n", bp->bouquet_name);
			ARRAY_FOREACH(sarr, pair,
				dump_service_cb(pair.tp, pair.sp);
			);
		}
	);

	// unsorted
	if (uarr.len) {
		qsort(uarr.buf, uarr.len, sizeof(struct bouquet_service_pair), cmp_bouquet_service);
		fprintf(stdout, ":==UNSORTED==\n");
		ARRAY_FOREACH(uarr, pair,
			dump_service_cb(pair.tp, pair.sp);
		);
	}

	info("\
==============================\n\
Bouquets:           %d\n\
Mapped services:    %d\n\
Unmapped services:  %d\n\
Channels:           %d\n\
==============================\n\
",
		n_bouquets, n_mapped, n_unmapped, htable_len(&uniq_channels));

	ARRAY_CLEAN(barr);
	ARRAY_CLEAN(sarr);
	ARRAY_CLEAN(uarr);
	htable_free(&uniq_channels);
}

