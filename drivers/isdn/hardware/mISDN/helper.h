/* $Id$
 *
 *   Basic declarations, defines and prototypes
 *
 * This file is (c) under GNU PUBLIC LICENSE
 *
 */
#ifndef _mISDN_HELPER_H
#define	_mISDN_HELPER_H
#include <linux/kernel.h>
#ifdef MEMDBG
#include "memdbg.h"
#endif

#define int_error() \
        printk(KERN_ERR "mISDN: INTERNAL ERROR in %s:%d\n", \
                       __FILE__, __LINE__)
                       
#define int_errtxt(fmt, arg...) \
        printk(KERN_ERR "mISDN: INTERNAL ERROR in %s:%d " fmt "\n", \
                       __FILE__, __LINE__, ## arg)
                       
#define APPEND_TO_LIST(item,base) \
	if (item->prev || item->next) \
		int_errtxt("APPEND not clean %p<-%p->%p", \
			item->prev, item, item->next); \
	item->next = NULL; \
	item->prev = base; \
	while (item->prev && item->prev->next) \
		item->prev = item->prev->next; \
	if (item->prev == item) { \
		int_errtxt("APPEND DUP %p", item); \
	} else \
		if (base) { \
			item->prev->next = item; \
		} else \
			base = item

#define INSERT_INTO_LIST(newi,nexti,base) \
	newi->next = nexti; \
	newi->prev = nexti->prev; \
	if (newi->prev) \
		newi->prev->next = newi; \
	nexti->prev = newi; \
	if (base == nexti) \
		base = newi

#define REMOVE_FROM_LIST(item) \
	if (item->prev) \
		item->prev->next = item->next; \
	if (item->next) \
		item->next->prev = item->prev

#define REMOVE_FROM_LISTBASE(item,base) \
	REMOVE_FROM_LIST(item); \
	if (item == base) \
		base = item->next

static inline int
discard_queue(struct sk_buff_head *q)
{
	struct sk_buff *skb;
	int ret=0;

	while ((skb = skb_dequeue(q))) {
		dev_kfree_skb(skb);
		ret++;
	}
	return(ret);
}

static inline struct sk_buff *
alloc_uplinkD_skb(size_t size)
{
	struct sk_buff *skb;

	if (!(skb = alloc_skb(size + L3_EXTRA_SIZE, GFP_ATOMIC)))
		printk(KERN_WARNING "%s(%d): no skb size\n", __FUNCTION__,
			size);
	else
		skb_reserve(skb, L3_EXTRA_SIZE);
	return(skb);
}

extern void	set_dchannel_pid(mISDN_pid_t *, int, int);
extern int	get_lowlayer(int);
extern int	get_up_layer(int);
extern int	get_down_layer(int);
extern int	layermask2layer(int);
extern int	get_protocol(mISDNstack_t *, int);
extern int	HasProtocol(mISDNobject_t *, int);
extern int	SetHandledPID(mISDNobject_t *, mISDN_pid_t *);
extern void	RemoveUsedPID(mISDN_pid_t *, mISDN_pid_t *);

static inline int HasProtocolP(mISDNobject_t *obj, int *PP)
{
	if (!PP) {
		int_error();
		return(-EINVAL);
	}
	return(HasProtocol(obj, *PP));
}

extern __inline__ void mISDN_sethead(u_int prim, int dinfo, struct sk_buff *skb)
{
	mISDN_head_t *hh = mISDN_HEAD_P(skb);

	hh->prim = prim;
	hh->dinfo = dinfo;
}

extern __inline__ int if_newhead(mISDNif_t *i, u_int prim, int dinfo,
	struct sk_buff *skb)
{
	if (!i->func || !skb)
		return(-ENXIO);
	mISDN_sethead(prim, dinfo, skb);
	return(i->func(i, skb));
}


extern __inline__ struct sk_buff *create_link_skb(u_int prim, int dinfo,
	int len, void *arg, int reserve)
{
	struct sk_buff	*skb;

	if (!(skb = alloc_skb(len + reserve, GFP_ATOMIC))) {
		printk(KERN_WARNING "%s: no skb size %d+%d\n",
			__FUNCTION__, len, reserve);
		return(NULL);
	} else
		skb_reserve(skb, reserve);
	if (len)
		memcpy(skb_put(skb, len), arg, len);
	mISDN_sethead(prim, dinfo, skb);
	return(skb);
}

extern __inline__ int if_link(mISDNif_t *i, u_int prim, int dinfo, int len,
	void *arg, int reserve)
{
	struct sk_buff	*skb;
	int		err;

	if (!(skb = create_link_skb(prim, dinfo, len, arg, reserve)))
		return(-ENOMEM);
	if (!i)
		err = -ENXIO;
	else
		err = i->func(i, skb);
	if (err)
		kfree_skb(skb);
	return(err);
}

/* L3 data struct helper functions */

extern	signed int	l3_ie2pos(u_char);
extern	unsigned char	l3_pos2ie(int);
extern	void		initQ931_info(Q931_info_t *);
extern	struct sk_buff 	*alloc_l3msg(int, u_char);
extern	void		AddvarIE(struct sk_buff *, u_char *);
extern	void		AddIE(struct sk_buff *, u_char, u_char *);
extern	void		LogL3Msg(struct sk_buff *);

#endif /* _mISDN_HELPER_H */
