/* $Id$
 *
 * Author       Karsten Keil (keil@isdn4linux.de)
 *
 * This file is (c) under GNU PUBLIC LICENSE
 *
 */

#define __NO_VERSION__
#include <linux/mISDNif.h>
#include "mISDNl1.h"
#include "mISDN_bch.h"
#include "helper.h"

static void
bchannel_bh(bchannel_t *bch)
{
	struct sk_buff	*skb;
	u_int 		pr;
	int		ret;
	mISDN_head_t	*hh;
	mISDNif_t	*hif;

	if (!bch)
		return;
	if (!bch->inst.up.func) {
		printk(KERN_WARNING "%s: without up.func\n", __FUNCTION__);
		return;
	}
#if 0
	printk(KERN_DEBUG "%s: event %x\n", __FUNCTION__, bch->event);
	if (bch->dev)
		printk(KERN_DEBUG "%s: rpflg(%x) wpflg(%x)\n", __FUNCTION__,
			bch->dev->rport.Flag, bch->dev->wport.Flag);
#endif
	if (test_and_clear_bit(B_XMTBUFREADY, &bch->event)) {
		skb = bch->next_skb;
		if (skb) {
			hh = mISDN_HEAD_P(skb);
			bch->next_skb = NULL;
			if (bch->inst.pid.protocol[2] == ISDN_PID_L2_B_TRANS)
				pr = DL_DATA | CONFIRM;
			else
				pr = PH_DATA | CONFIRM;
			if ((bch->inst.pid.protocol[2] == ISDN_PID_L2_B_RAWDEV)
				&& bch->dev)
				hif = &bch->dev->rport.pif;
			else
				hif = &bch->inst.up;
			if (if_newhead(hif, pr, hh->dinfo, skb))
				dev_kfree_skb(skb);
		}
	}
	if (test_and_clear_bit(B_RCVBUFREADY, &bch->event)) {
		if ((bch->inst.pid.protocol[2] == ISDN_PID_L2_B_RAWDEV)
			&& bch->dev)
			hif = &bch->dev->rport.pif;
		else
			hif = &bch->inst.up;
		while ((skb = skb_dequeue(&bch->rqueue))) {
			if (bch->inst.pid.protocol[2] == ISDN_PID_L2_B_TRANS)
				pr = DL_DATA | INDICATION;
			else
				pr = PH_DATA | INDICATION;
			ret = if_newhead(hif, pr, DINFO_SKB, skb);
			if (ret < 0) {
				printk(KERN_WARNING "%s: deliver err %d\n",
					__FUNCTION__, ret);
				dev_kfree_skb(skb);
			}
		}
	}
	if (bch->hw_bh)
		bch->hw_bh(bch);
}

int
init_bchannel(bchannel_t *bch) {
	int	devtyp = mISDN_RAW_DEVICE;

	if (!(bch->blog = kmalloc(MAX_BLOG_SPACE, GFP_ATOMIC))) {
		printk(KERN_WARNING
			"mISDN: No memory for blog\n");
		return(-ENOMEM);
	}
	if (!(bch->rx_buf = kmalloc(MAX_DATA_MEM, GFP_ATOMIC))) {
		printk(KERN_WARNING
			"mISDN: No memory for bchannel rx_buf\n");
		kfree(bch->blog);
		bch->blog = NULL;
		return (-ENOMEM);
	}
	if (!(bch->tx_buf = kmalloc(MAX_DATA_MEM, GFP_ATOMIC))) {
		printk(KERN_WARNING
			"mISDN: No memory for bchannel tx_buf\n");
		kfree(bch->blog);
		bch->blog = NULL;
		kfree(bch->rx_buf);
		bch->rx_buf = NULL;
		return (-ENOMEM);
	}
	skb_queue_head_init(&bch->rqueue);
	bch->next_skb = NULL;
	bch->Flag = 0;
	bch->event = 0;
	bch->rx_idx = 0;
	bch->tx_len = 0;
	bch->tx_idx = 0;
	bch->tqueue.data = bch;
	bch->tqueue.routine = (void *) (void *) bchannel_bh;
	bch->hw_bh = NULL;
	if (!bch->dev) {
		if (bch->inst.obj->ctrl(&bch->dev, MGR_GETDEVICE | REQUEST,
			&devtyp)) {
			printk(KERN_WARNING
				"mISDN: no raw device for bchannel\n");
		}
	}
	return(0);
}

int
free_bchannel(bchannel_t *bch) {

	if (bch->tqueue.sync)
		printk(KERN_ERR"free_bchannel tqueue.sync\n");
	discard_queue(&bch->rqueue);
	if (bch->blog) {
		kfree(bch->blog);
		bch->blog = NULL;
	}
	if (bch->rx_buf) {
		kfree(bch->rx_buf);
		bch->rx_buf = NULL;
	}
	if (bch->tx_buf) {
		kfree(bch->tx_buf);
		bch->tx_buf = NULL;
	}
	if (bch->next_skb) {
		dev_kfree_skb(bch->next_skb);
		bch->next_skb = NULL;
	}
	if (bch->inst.obj->ctrl(bch->dev, MGR_DELDEVICE | REQUEST, NULL)) {
		printk(KERN_WARNING
			"mISDN: del raw device error\n");
	} else
		bch->dev = NULL;
	return(0);
}
