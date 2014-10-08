#include <stdlib.h>
#include <string.h>

#include "htable.h"

#define	HASH_MINSIZE	2

typedef uint32_t Fnv32_t;
#define FNV_32_PRIME ((Fnv32_t)0x01000193)
static Fnv32_t fnv_32a_buf(void *buf, size_t len)
{
	Fnv32_t hval = (Fnv32_t)0x811c9dc5;
	unsigned char *bp = (unsigned char *)buf;
	unsigned char *be = bp + len;
	while (bp < be) {
		hval ^= (Fnv32_t)*bp++;
		hval *= FNV_32_PRIME;
	}
	return hval;
}

static hash_value_t hash_simple(void *key, int len)
{
	unsigned char *b = (unsigned char *)key;
	return (unsigned int)(b[1]<<8) | b[0];
}

#define hash_function	fnv_32a_buf
//#define hash_function	hash_simple

// size must be power of 2
void htable_init(struct htable *h, int size, int resize)
{
	if (size < HASH_MINSIZE)
		size = HASH_MINSIZE;
	h->size = size;
	h->count = 0;
	h->resize = resize;
	h->arr = malloc(size * sizeof(struct htable_entry *));
	memset(h->arr, '\0', size * sizeof(struct htable_entry *));
}

void htable_free(struct htable *h)
{
	if (h->arr) free(h->arr);
	h->arr = NULL;
}

struct htable_entry *htable_lookup(struct htable *h, struct htable_entry *e)
{
	int i;
	struct htable_entry *p;
	i = e->hash & (h->size - 1);
	for(p = h->arr[i];
		p != NULL && (p->key_len != e->key_len || memcmp(p->key, e->key, e->key_len) != 0);
		p = p->next
	);
	return p;
}

static void htable_resize(struct htable *h, int newsize)
{
	int i, old_size, old_count, idx;
	struct htable_entry **old_arr, *p;

	if (newsize < HASH_MINSIZE)
		newsize = HASH_MINSIZE;
	old_arr = h->arr;
	old_size = h->size;
	old_count = h->count;
	if (old_size == newsize)
		return;
	htable_init(h, newsize, h->resize);
	for (i = 0; i < old_size; i++) {
		while (old_arr[i] != NULL) {
			p = old_arr[i];
			idx = p->hash & (h->size - 1);
			old_arr[i] = p->next;
			p->next = h->arr[idx];
			h->arr[idx] = p;
		}
	}
	h->count = old_count;
	free(old_arr);
}

void htable_insert(struct htable *h, struct htable_entry *e)
{
	int i;

	if (h->resize && h->count >= h->size) {
		htable_resize(h, h->size * 2);
	}
	i = e->hash & (h->size - 1);
	e->next = h->arr[i];
	h->arr[i] = e;
	h->count++;
}

static struct htable_entry *htable_remove_int(struct htable *h, struct htable_entry *e, int noresize)
{
	int i;
	struct htable_entry *p, **link;

	i = e->hash & (h->size - 1);
	link = &(h->arr[i]);
	for(p = h->arr[i]; p != NULL && (p->key_len != e->key_len || memcmp(p->key, e->key, e->key_len)); p = p->next)
		link = &(p->next);
	if (p != NULL) {
		*link = p->next;
		h->count--;
		if (h->resize && noresize == 0 && h->count <= h->size / 2)
			htable_resize(h, h->size / 2);
	}
	return p;
}

struct htable_entry *htable_remove_noresize(struct htable *h, struct htable_entry *e)
{
	return htable_remove_int(h, e, 1);
}

struct htable_entry *htable_remove(struct htable *h, struct htable_entry *e)
{
	return htable_remove_int(h, e, 0);
}

void htable_entry_init(struct htable_entry *e, void *key, int len)
{
	e->key = key;
	e->key_len = len;
	e->hash = hash_function(key, len);
	e->next = NULL;
}

int htable_len(struct htable *h)
{
	return h->count;
}

#ifdef HASH_TEST
#include <stdio.h>

int main (int argc, char **argv)
{
	struct entry {
		unsigned char key[6];
		struct htable_entry hash;
	};
	int nid, tid, sid, i, count, max_probes, n_probes, n_buckets, n_dup = 0;
	struct entry e, *ep;
	struct htable ht;
	struct htable_entry *p;

	htable_init(&ht, atoi(argv[1]), 0);
	while (fscanf(stdin, "%d:%d:%d", &sid, &nid, &tid) == 3) {
		e.key[0] = sid & 0xff;
		e.key[1] = sid>>8 & 0xff;
		e.key[2] = tid & 0xff;
		e.key[3] = tid>>8 & 0xff;
		e.key[4] = nid & 0xff;
		e.key[5] = nid>>8 & 0xff;
		htable_entry_init(&e.hash, e.key, 6);
		if (htable_lookup(&ht, &e.hash) == NULL) {
			ep = malloc(sizeof(struct entry));
			memcpy(ep, &e, sizeof(e));
			htable_entry_init(&ep->hash, ep->key, 6);
			htable_insert(&ht, &ep->hash);
		} else {
			n_dup++;
		}
	}

	max_probes = 0; n_probes = 0; n_buckets = 0;
	for (i = 0; i < ht.size; i++) {
		count = 0;
		for (p = ht.arr[i]; p != NULL; p = p->next)
			count++;
		if (count != 0)
			n_buckets++;
		n_probes += count;
		if (count > max_probes)
			max_probes = count;
		printf("%d %d\n", i, count);
	}
	fprintf(stderr, "dup: %d\nmax: %d\navg: %.2f\n", n_dup, max_probes, (double)n_probes / n_buckets);

	return 0;
}
#endif
