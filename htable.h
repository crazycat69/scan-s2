#ifndef __HTABLE_H__
#define __HTABLE_H__

#include <stdint.h>
#include <stddef.h>

typedef uint32_t hash_value_t;

struct htable_entry {
	void *key;
	int key_len;
	hash_value_t hash;
	struct htable_entry *next;
};

struct htable {
	struct htable_entry **arr;
	int size;
	int count;
	int resize;
};

#ifndef container_of
#define container_of(ptr, type, member) ({ \
				const typeof( ((type *)0)->member ) *__mptr = (ptr); \
				(type *)( (char *)__mptr - offsetof(type,member) );})
#endif

// no inner loop - allows break and continue from the action code
#define HTABLE_FOREACH(htable_ptr, var, type, member, action) \
do {														\
	int __idx; struct htable_entry *__cur;					\
	__idx = -1;												\
	__cur = NULL;											\
	while (1) {												\
		if (__cur == NULL) {								\
			__idx++;										\
			if (__idx >= (htable_ptr)->size)				\
				break;										\
			__cur = (htable_ptr)->arr[__idx];				\
			continue;										\
		}													\
		var = container_of(__cur, type, member);			\
		__cur = __cur->next;								\
		{action}											\
	}														\
} while (0)

extern void htable_entry_init(struct htable_entry *e, void *key, int len);

// size must be power of 2
extern void htable_init(struct htable *h, int size, int resize);
extern void htable_free(struct htable *h);
// returns entry found or NULL
extern struct htable_entry *htable_lookup(struct htable *h, struct htable_entry *e);
// only unique entries, call htable_lookup() first
extern void htable_insert(struct htable *h, struct htable_entry *e);
// returns the pointer to a removed entry or NULL if not found
extern struct htable_entry *htable_remove(struct htable *h, struct htable_entry *e);
// for removals inside HTABLE_FOREACH()
extern struct htable_entry *htable_remove_noresize(struct htable *h, struct htable_entry *e);
extern int htable_len(struct htable *h);


#endif

