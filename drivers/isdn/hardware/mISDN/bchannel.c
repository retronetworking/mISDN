/* $Id$
 *
 * Author       Karsten Keil (keil@isdn4linux.de)
 *
 * This file is (c) under GNU PUBLIC LICENSE
 *
 */

#include <linux/module.h>
#include <linux/mISDNif.h>
#include "layer1.h"
#include "bchannel.h"
#include "helper.h"

#ifdef OBSOLATE
static void
bchannel_bh(bchannel_t *bch)
{
	struct sk_buff	*skb;
	u_int 		pr;
	int		ret;
	mISDN_head_t	*hh;

	if (!bch)
		return;
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
#ifdef FIXME
			if ((bch->inst.pid.protocol[2] == ISDN_PID_L2_B_RAWDEV)
				&& bch->dev)
				hif = &bch->dev->rport.pif;
			else
				hif = &bch->inst.up;
#endif
			if (mISDN_queueup_newhead(&bch->inst, 0, pr, hh->dinfo, skb))
				dev_kfree_skb(skb);
		}
	}
	if (test_and_clear_bit(B_RCVBUFREADY, &bch->event)) {
#ifdef FIXME
		if ((bch->inst.pid.protocol[2] == ISDN_PID_L2_B_RAWDEV)
			&& bch->dev)
			hif = &bch->dev->rport.pif;
		else
			hif = &bch->inst.up;
#endif
		while ((skb = skb_dequeue(&bch->rqueue))) {
			if (bch->inst.pid.protocol[2] == ISDN_PID_L2_B_TRANS)
				pr = DL_DATA | INDICATION;
			else
				pr = PH_DATA | INDICATION;
			ret = mISDN_queueup_newhead(&bch->inst, 0, pr, MISDN_ID_ANY, skb);
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
#endif

int
mISDN_init_bch(bchannel_t *bch) {
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
#ifdef OBSOLATE
	skb_queue_head_init(&bch->rqueue);
	bch->event = 0;
	INIT_WORK(&bch->work, (void *)(void *)bchannel_bh, bch);
#endif
	bch->next_skb = NULL;
	bch->Flag = 0;
	bch->rx_idx = 0;
	bch->tx_len = 0;
	bch->tx_idx = 0;
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
mISDN_free_bch(bchannel_t *bch) {
#ifdef OBSOLATE
#ifdef HAS_WORKQUEUE
	if (bch->work.pending)
		printk(KERN_ERR "mISDN_free_bch work:(%lx)\n", bch->work.pending);
#else
	if (bch->work.sync)
		printk(KERN_ERR "mISDN_free_bch work:(%lx)\n", bch->work.sync);
#endif
	discard_queue(&bch->rqueue);
#endif
	kfree(bch->blog);
	bch->blog = NULL;
	kfree(bch->rx_buf);
	bch->rx_buf = NULL;
	kfree(bch->tx_buf);
	bch->tx_buf = NULL;
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

EXPORT_SYMBOL(mISDN_init_bch);
EXPORT_SYMBOL(mISDN_free_bch);
