#ifndef __BOUQUET_H__
#define __BOUQUET_H__

struct bouquet_ctx;

extern struct bouquet_ctx *bouquet_create(const char *optstring);
extern void bouquet_free(struct bouquet_ctx *ctx);

extern void bouquet_parse_bat(struct bouquet_ctx *ctx, const unsigned char *buf, 
		int section_length, int bouquet_id, int version_number);

extern void bouquet_dump(struct bouquet_ctx *ctx, struct list_head *scanned_transponders,
		int ca_select, int serv_select,
		void (*dump_service_cb)(struct transponder *, struct service *));

extern const char *bouquet_help_msg();

#endif

