/* $Id$
 *
 * Author       Karsten Keil (keil@isdn4linux.de)
 *
 * This file is (c) under GNU PUBLIC LICENSE
 *
 */

#define __NO_VERSION__
#include <linux/mISDNif.h>
#include "layer1.h"
#include "helper.h"
#include "dchannel.h"

static void
dchannel_bh(dchannel_t *dch)
{
	struct sk_buff	*skb;
	int		err;

	if (!dch)
		return;
	if (dch->debug)
		printk(KERN_DEBUG "%s: event %lx\n", __FUNCTION__, dch->event);
#if 0
	if (test_and_clear_bit(D_CLEARBUSY, &dch->event)) {
		if (dch->debug)
			debugprint(&dch->inst, "D-Channel Busy cleared");
		stptr = dch->stlist;
		while (stptr != NULL) {
			stptr->l1.l1l2(stptr, PH_PAUSE | CONFIRM, NULL);
			stptr = stptr->next;
		}
	}
#endif
	if (test_and_clear_bit(D_XMTBUFREADY, &dch->event)) {
		if ((skb = dch->next_skb)) {
			dch->next_skb = NULL;
			skb_trim(skb, 0);
			if (if_newhead(&dch->inst.up, PH_DATA_CNF, DINFO_SKB, skb))
				dev_kfree_skb(skb);
		}
	}

	if (test_and_clear_bit(D_RCVBUFREADY, &dch->event)) {
		while ((skb = skb_dequeue(&dch->rqueue))) {
			err = if_newhead(&dch->inst.up, PH_DATA_IND, DINFO_SKB, skb);
			if (err < 0) {
				printk(KERN_WARNING "%s: deliver err %d\n", __FUNCTION__, err);
				dev_kfree_skb(skb);
			}
		}
	}

	if (dch->hw_bh)
		dch->hw_bh(dch);
}

int
init_dchannel(dchannel_t *dch) {
	if (!(dch->dlog = kmalloc(MAX_DLOG_SPACE, GFP_ATOMIC))) {
		printk(KERN_WARNING
			"mISDN: No memory for dlog\n");
		return(-ENOMEM);
	}
	if (!(dch->tx_buf = kmalloc(MAX_DFRAME_LEN_L1, GFP_ATOMIC))) {
		printk(KERN_WARNING
			"mISDN: No memory for dchannel tx_buf\n");
		kfree(dch->dlog);
		dch->dlog = NULL;
		return(-ENOMEM);
	}
	dch->hw = NULL;
	dch->rx_skb = NULL;
	dch->tx_idx = 0;
	dch->next_skb = NULL;
	dch->event = 0;
	dch->tqueue.data = dch;
	dch->tqueue.routine = (void *) (void *) dchannel_bh;
	dch->hw_bh = NULL;
	skb_queue_head_init(&dch->rqueue);
	return(0);
}

int
free_dchannel(dchannel_t *dch) {

	if (dch->tqueue.sync)
		printk(KERN_ERR"free_dchannel tqueue.sync\n");
	discard_queue(&dch->rqueue);
	if (dch->rx_skb) {
		dev_kfree_skb(dch->rx_skb);
		dch->rx_skb = NULL;
	}
	if (dch->tx_buf) {
		kfree(dch->tx_buf);
		dch->tx_buf = NULL;
	}
	if (dch->next_skb) {
		dev_kfree_skb(dch->next_skb);
		dch->next_skb = NULL;
	}
	if (dch->dlog) {
		kfree(dch->dlog);
		dch->dlog = NULL;
	}
	return(0);
}

