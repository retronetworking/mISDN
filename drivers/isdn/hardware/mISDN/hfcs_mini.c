/* $Id$
 *
 * mISDN driver for Colognechip HFC-S mini Evaluation Card
 *
 * Authors : Martin Bachem, Joerg Ciesielski
 * Contact : info@colognechip.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *******************************************************************************
 *
 * MODULE PARAMETERS:
 * (NOTE: layermask and protocol must be given for all ports,
 *  not for the number of cards.)
 *
 * - protocol=<p1>[,p2,p3...]
 *   Values:
 *      <bit  3 -  0>  D-channel protocol id
 *      <bit  4 -  4>  Flags for special features
 *      <bit 31 -  5>  Spare (set to 0)
 *
 *        D-channel protocol ids
 *        - 1       1TR6 (not released yet)
 *        - 2       DSS1
 *
 *        Feature Flags
 *        <bit 4>   0x0010  Net side stack (NT mode)
 *        <bit 5>   0x0020  PCI mode (0=master, 1=slave)
 *        <bit 6>   0x0040  not in use
 *        <bit 7>   0x0080  B channel loop (for layer1 tests)
 *
 * - layermask=<l1>[,l2,l3...] (32bit):
 *        mask of layers to be used for D-channel stack
 *
 * - debug:
 *        enable debugging (see hfcs_mini.h for debug options)
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/timex.h>
#include "layer1.h"
#include "debug.h"
#include "hfcs_mini.h"
#include "hfcsmcc.h"

#if HFCBRIDGE == BRIDGE_HFCPCI
#include <linux/pci.h>
#endif

static const char hfcsmini_rev[] = "$Revision$";

#define MAX_CARDS	8
static int card_cnt;
static u_int protocol[MAX_CARDS];
static int layermask[MAX_CARDS];

#ifdef MODULE
MODULE_LICENSE("GPL");
#define MODULE_PARM_T	"1-8i"
MODULE_PARM(debug, "1i");
MODULE_PARM(protocol, MODULE_PARM_T);
MODULE_PARM(layermask, MODULE_PARM_T);
#endif

static mISDNobject_t hw_mISDNObj;
static int debug = 0;

static inline void
hfcsmini_sel_reg(hfcsmini_hw * hw, __u8 reg_addr)
{
	outb(6, hw->iobase + 3); /* A0 = 1, reset = 1 */
	outb(reg_addr, hw->iobase + 1); /* write register number */
	outb(4, hw->iobase + 3); /* A0 = 0, reset = 1 */
}


static inline __u8
read_hfcsmini(hfcsmini_hw * hw, __u8 reg_addr)
{
	register u_char ret;
	
#ifdef SPIN_LOCK_hfcsmini_REGISTER
	spin_lock_irq(&hw->rlock);
#endif
	hfcsmini_sel_reg(hw, reg_addr);
	ret = inb(hw->iobase + 1);
#ifdef SPIN_LOCK_hfcsmini_REGISTER	
	spin_unlock_irq(&hw->rlock);
#endif	
	return(ret);
}


/* read register in already spin-locked irq context */
static inline __u8
read_hfcsmini_irq(hfcsmini_hw * hw, __u8 reg_addr)
{
	register u_char ret;
	hfcsmini_sel_reg(hw, reg_addr);
	ret = inb(hw->iobase + 1);
	return(ret);
}


static inline __u8  
read_hfcsmini_stable(hfcsmini_hw * hw, __u8 reg_addr)
{
	register u_char in1, in2; 

#ifdef SPIN_LOCK_hfcsmini_REGISTER
	spin_lock_irq(&hw->rlock);
#endif
	hfcsmini_sel_reg(hw, reg_addr);

	in1 = inb(hw->iobase + 1);
	// loop until 2 equal accesses
	while((in2=inb(hw->iobase + 1))!=in1) in1=in2;
	
#ifdef SPIN_LOCK_hfcsmini_REGISTER	
	spin_unlock_irq(&hw->rlock);
#endif	
	return(in1);
}


static inline void
write_hfcsmini(hfcsmini_hw * hw, __u8 reg_addr, __u8 value)
{
#ifdef SPIN_LOCK_hfcsmini_REGISTER
	spin_lock_irq(&hw->rlock);
#endif
	hfcsmini_sel_reg(hw, reg_addr);
	outb(value, hw->iobase + 1);
#ifdef SPIN_LOCK_hfcsmini_REGISTER	
	spin_unlock_irq(&hw->rlock);
#endif	
}

static void
hfcsmini_ph_command(dchannel_t * dch, u_char command)
{
	hfcsmini_hw *hw = dch->hw;
	
	if (dch->debug)
		mISDN_debugprint(&dch->inst,
				 "%s command(%i) channel(%i)",
				 __FUNCTION__, command, dch->channel);

	switch (command) {
		case HFC_L1_ACTIVATE_TE:
			if ((dch->debug) & (debug & DEBUG_HFC_S0_STATES)) {
				mISDN_debugprint(&dch->inst,
						 "HFC_L1_ACTIVATE_TE channel(%i) command(%i)",
						 dch->channel, command);
			}

			write_hfcsmini(hw, R_ST_WR_STA, (M_ST_LD_STA | (M1_ST_SET_STA*4)));
			udelay(125); /* to be sure INFO1 signals are sent */
			write_hfcsmini(hw, R_ST_WR_STA, (M1_ST_SET_STA * 4));
			break;
			
		case HFC_L1_FORCE_DEACTIVATE_TE:
			write_hfcsmini(hw, R_ST_WR_STA, (M_ST_LD_STA | (M1_ST_SET_STA*3)));
			udelay(7); /* wait at least 5,21 us */
			write_hfcsmini(hw, R_ST_WR_STA, (M1_ST_SET_STA*3));
			break;
			

		case HFC_L1_ACTIVATE_NT:
			if ((dch->debug) & (debug & DEBUG_HFC_S0_STATES))
				mISDN_debugprint(&dch->inst,
					 "HFC_L1_ACTIVATE_NT channel(%i)");

			write_hfcsmini(hw, R_ST_WR_STA, (M1_ST_ACT | M_SET_G2_G3));
			break;

		case HFC_L1_DEACTIVATE_NT:
			if ((dch->debug) & (debug & DEBUG_HFC_S0_STATES))
				mISDN_debugprint(&dch->inst,
						 "HFC_L1_DEACTIVATE_NT channel(%i)");

			write_hfcsmini(hw, R_ST_WR_STA, (M1_ST_ACT * 2));
			break;
			
		case HFC_L1_TESTLOOP_B1:
			break;
			
		case HFC_L1_TESTLOOP_B2:
			break;

	}
}


/*********************************/
/* S0 state change event handler */
/*********************************/
static void
s0_new_state(dchannel_t * dch)
{
	u_int prim = PH_SIGNAL | INDICATION;
	u_int para = 0;
	hfcsmini_hw *hw = dch->hw;

	if (hw->portmode & PORT_MODE_TE) {
		if ((dch->debug) & (debug & DEBUG_HFC_S0_STATES))
			mISDN_debugprint(&dch->inst,
					 "%s: TE %d",
					 __FUNCTION__, dch->ph_state);

		switch (dch->ph_state) {
			case (0):
				prim = PH_CONTROL | INDICATION;
				para = HW_RESET;
				break;
			case (3):
				prim = PH_CONTROL | INDICATION;
				para = HW_DEACTIVATE;
				break;
			case (6):
				para = INFO2;
				break;
			case (7):
				para = INFO4_P8;
				break;
			case (5):
			case (8):
				para = ANYSIGNAL;
				break;
			default:
				return;
		}
	} // PORT_MODE_TE

	if (hw->portmode & PORT_MODE_NT) {
		if ((dch->debug) & (debug & DEBUG_HFC_S0_STATES))
			mISDN_debugprint(&dch->inst,
					 "%s: NT %d",
					 __FUNCTION__, dch->ph_state);

		switch (dch->ph_state) {
			case (1):
				hw->nt_timer = 0;
				hw->portmode &= ~NT_TIMER;
				prim = PH_DEACTIVATE | INDICATION;
				para = 0;
				break;
			case (2):
				if (hw->nt_timer < 0) {
					hw->nt_timer = 0;
					hw->portmode &= ~NT_TIMER;
					hfcsmini_ph_command(dch,
							HFC_L1_DEACTIVATE_NT);
				} else {
					hw->nt_timer = NT_T1_COUNT;
					hw->portmode |= NT_TIMER;
					write_hfcsmini(hw, R_ST_WR_STA, M_SET_G2_G3);
				}
				return;
			case (3):
				hw->nt_timer = 0;
				hw->portmode &= ~NT_TIMER;
				prim = PH_ACTIVATE | INDICATION;
				para = 0;
				break;
			case (4):
				hw->nt_timer = 0;
				hw->portmode &= ~NT_TIMER;
				return;
			default:
				break;
		}
	} // PORT_MODE_NT
	
	mISDN_queue_data(&dch->inst, FLG_MSG_UP, prim, para, 0, NULL, 0);
}


/*************************************/
/* Layer 1 D-channel hardware access */
/*************************************/
static int
hfcsmini_l1hwD(mISDNinstance_t *inst, struct sk_buff *skb)
{
	dchannel_t	*dch = container_of(inst, dchannel_t, inst);
	int		ret = 0;
	mISDN_head_t	*hh = mISDN_HEAD_P(skb);
	hfcsmini_hw	*hw = inst->privat;
	u_long		flags;
	__u16		i;
		
	if (hh->prim == PH_DATA_REQ) {
		/* check oversize */
		if (skb->len <= 0) {
			printk(KERN_WARNING "%s: skb too small\n", __FUNCTION__);
			return(-EINVAL);
		}
		if (skb->len > MAX_DFRAME_LEN_L1) {
			printk(KERN_WARNING "%s: skb too large\n", __FUNCTION__);
			return(-EINVAL);
		}
		
		/* check for pending next_skb */
		spin_lock_irqsave(inst->hwlock, flags);
		if (dch->next_skb) {
			test_and_set_bit(FLG_TX_NEXT, &dch->DFlags);
			dch->next_skb = skb;
			spin_unlock_irqrestore(inst->hwlock, flags);
			mISDN_debugprint(&dch->inst, "pending dch->next_skb!\n");
			
			return (0);
		}
		
		if (test_and_set_bit(FLG_TX_BUSY, &dch->DFlags)) {
			if ((dch->debug) && (debug & DEBUG_HFC_DTRACE)) {
				mISDN_debugprint(&dch->inst, "channel(%i) busy, attaching dch->next_skb!\n",
						 dch->channel,
						 skb->len);
			}
			test_and_set_bit(FLG_TX_NEXT, &dch->DFlags);
			dch->next_skb = skb;
			spin_unlock_irqrestore(inst->hwlock, flags);
			return (0);
		} else {
			dch->tx_len = skb->len;
			memcpy(dch->tx_buf, skb->data, dch->tx_len);
			dch->tx_idx = 0;
			if ((dch->debug) && (debug & DEBUG_HFC_DTRACE)) {
				mISDN_debugprint(&dch->inst,
						 "channel(%i) new D-TX len(%i): ",
						 dch->channel,
						 dch->tx_len);
				i = 0;
				printk("  ");
				while (i < dch->tx_len)
					printk("%02x ", skb->data[i++]);
				printk("\n");
			}
			spin_unlock_irqrestore(inst->hwlock, flags);

			/* schedule bottom half instead of */
			/* calling hfcsmini_write_fifo */
			tasklet_schedule(&hw->tasklet);

			skb_trim(skb, 0);

			return(mISDN_queueup_newhead(inst, 0, PH_DATA_CNF,
				hh->dinfo, skb));
		}
	} else if (hh->prim == (PH_SIGNAL | REQUEST)) {
		ret = -EINVAL;
	} else if (hh->prim == (PH_CONTROL | REQUEST)) {
		spin_lock_irqsave(inst->hwlock, flags);
		if (hh->dinfo == HW_RESET) {
			if (dch->ph_state != 0)
				hfcsmini_ph_command(dch, HFC_L1_ACTIVATE_TE);
			spin_unlock_irqrestore(inst->hwlock, flags);
			skb_trim(skb, 0);
			return(mISDN_queueup_newhead(inst, 0, PH_CONTROL | INDICATION,HW_POWERUP, skb));
		} else if (hh->dinfo == HW_DEACTIVATE) {
			if (dch->next_skb) {
				dev_kfree_skb(dch->next_skb);
				dch->next_skb = NULL;
			}
			test_and_clear_bit(FLG_TX_NEXT, &dch->DFlags);
			test_and_clear_bit(FLG_TX_BUSY, &dch->DFlags);
#ifdef FIXME
			if (test_and_clear_bit(FLG_L1_DBUSY, &dch->DFlags))
				dchannel_sched_event(dch, D_CLEARBUSY);
#endif
		} else if ((hh->dinfo & HW_TESTLOOP) == HW_TESTLOOP) {
			if (1 & hh->dinfo)
				hfcsmini_ph_command(dch, HFC_L1_TESTLOOP_B1);
				
			if (2 & hh->dinfo)
				hfcsmini_ph_command(dch, HFC_L1_TESTLOOP_B2);
				
		} else if (hh->dinfo == HW_POWERUP) {
			hfcsmini_ph_command(dch, HFC_L1_FORCE_DEACTIVATE_TE);
		} else {
			if (dch->debug & L1_DEB_WARN)
				mISDN_debugprint(&dch->inst,
						 "hfcsmini_l1hw unknown ctrl %x",
						 hh->dinfo);
			ret = -EINVAL;
		}
		spin_unlock_irqrestore(inst->hwlock, flags);
	} else if (hh->prim == (PH_ACTIVATE | REQUEST)) {
		spin_lock_irqsave(inst->hwlock, flags);
		if (hw->portmode & PORT_MODE_NT) {
			hfcsmini_ph_command(dch, HFC_L1_ACTIVATE_NT);
		} else {
			if (dch->debug & L1_DEB_WARN)
				mISDN_debugprint(&dch->inst,
						 "%s: PH_ACTIVATE none NT mode",
						 __FUNCTION__);
			ret = -EINVAL;
		}
		spin_unlock_irqrestore(inst->hwlock, flags);
	} else if (hh->prim == (PH_DEACTIVATE | REQUEST)) {
		spin_lock_irqsave(inst->hwlock, flags);
		if (hw->portmode & PORT_MODE_NT) {
			hfcsmini_ph_command(dch, HFC_L1_DEACTIVATE_NT);
		} else {
			if (dch->debug & L1_DEB_WARN)
				mISDN_debugprint(&dch->inst,
						 "%s: PH_DEACTIVATE none NT mode",
						 __FUNCTION__);
			ret = -EINVAL;
		}
		spin_unlock_irqrestore(inst->hwlock, flags);
	} else {
		if (dch->debug & L1_DEB_WARN)
			mISDN_debugprint(&dch->inst,
					 "hfcsmini_l1hw unknown prim %x",
					 hh->prim);
		ret = -EINVAL;
	}
	if (!ret)
		dev_kfree_skb(skb);
	return (ret);
}


/******************************/
/* Layer2 -> Layer 1 Transfer */
/******************************/
static int
hfcsmini_l2l1B(mISDNinstance_t *inst, struct sk_buff *skb)
{
	bchannel_t	*bch = container_of(inst, bchannel_t, inst);
	hfcsmini_hw 	*hw = bch->hw;
	int		ret = 0;
	mISDN_head_t	*hh = mISDN_HEAD_P(skb);
	u_long		flags;
	
	if ((hh->prim == PH_DATA_REQ) || (hh->prim == (DL_DATA | REQUEST))) {
		if (skb->len <= 0) {
			printk(KERN_WARNING "%s: skb too small\n", __FUNCTION__);
			return(-EINVAL);
		}
		if (skb->len > MAX_DATA_MEM) {
			printk(KERN_WARNING "%s: skb too large\n", __FUNCTION__);
			return(-EINVAL);
		}
		spin_lock_irqsave(inst->hwlock, flags);
		
		/* check for pending next_skb */
		if (bch->next_skb) {
			printk(KERN_WARNING "%s: next_skb exist ERROR\n",
			       __FUNCTION__);
			spin_unlock_irqrestore(inst->hwlock, flags);
			return (-EBUSY);
		}
		
		
		if (test_and_set_bit(BC_FLG_TX_BUSY, &bch->Flag)) {
			test_and_set_bit(BC_FLG_TX_NEXT, &bch->Flag);
			bch->next_skb = skb;
			spin_unlock_irqrestore(inst->hwlock, flags);
			return (0);
		} else {
			bch->tx_len = skb->len;
			memcpy(bch->tx_buf, skb->data, bch->tx_len);
			bch->tx_idx = 0;
			spin_unlock_irqrestore(inst->hwlock, flags);
			
			if ((bch->debug) && (debug & DEBUG_HFC_DTRACE)) {
				mISDN_debugprint(&bch->inst,
						 "%s channel(%i) "
						 "new TX-B (%i)",
						 __FUNCTION__,
						 bch->channel,
						 bch->tx_len);
			}

			/* schedule bottom half instead of */
			/* calling hfcsmini_write_fifo */
			tasklet_schedule(&hw->tasklet);

#ifdef FIXME
			if ((bch->inst.pid.protocol[2] == ISDN_PID_L2_B_RAWDEV)
				&& bch->dev)
				hif = &bch->dev->rport.pif;
			else
				hif = &bch->inst.up;
#endif
			skb_trim(skb, 0);
			return(mISDN_queueup_newhead(inst, 0, hh->prim | CONFIRM,
				hh->dinfo, skb));
		}
	} else if ((hh->prim == (PH_ACTIVATE | REQUEST)) ||
		   (hh->prim == (DL_ESTABLISH | REQUEST))) {
		if (!test_and_set_bit(BC_FLG_ACTIV, &bch->Flag)) {
			spin_lock_irqsave(inst->hwlock, flags);
			ret =
			    setup_channel(hw, bch->channel,
					  bch->inst.pid.protocol[1]);
			spin_unlock_irqrestore(inst->hwlock, flags);
		}
#ifdef FIXME
		if (bch->inst.pid.protocol[2] == ISDN_PID_L2_B_RAWDEV)
			if (bch->dev)
				if_link(&bch->dev->rport.pif,
					hh->prim | CONFIRM, 0, 0, NULL, 0);
#endif
		skb_trim(skb, 0);
		return(mISDN_queueup_newhead(inst, 0, hh->prim | CONFIRM, ret, skb));
		
	} else if ((hh->prim == (PH_DEACTIVATE | REQUEST)) ||
		   (hh->prim == (DL_RELEASE | REQUEST)) ||
		   ((hh->prim == (PH_CONTROL | REQUEST) && (hh->dinfo == HW_DEACTIVATE)))) {
		   	
		spin_lock_irqsave(inst->hwlock, flags);
		if (test_and_clear_bit(BC_FLG_TX_NEXT, &bch->Flag)) {
			dev_kfree_skb(bch->next_skb);
			bch->next_skb = NULL;
		}
		test_and_clear_bit(BC_FLG_TX_BUSY, &bch->Flag);
		setup_channel(hw, bch->channel, ISDN_PID_NONE);
		test_and_clear_bit(BC_FLG_ACTIV, &bch->Flag);
		spin_unlock_irqrestore(inst->hwlock, flags);
		skb_trim(skb, 0);
		if (hh->prim != (PH_CONTROL | REQUEST)) {
#ifdef FIXME
			if (bch->inst.pid.protocol[2] == ISDN_PID_L2_B_RAWDEV)
				if (bch->dev)
					if_link(&bch->dev->rport.pif,
						hh->prim | CONFIRM, 0, 0, NULL, 0);
#endif
			if (!mISDN_queueup_newhead(inst, 0, hh->prim | CONFIRM, 0, skb))
				return(0);
		}

	} else if (hh->prim == (PH_CONTROL | REQUEST)) {
		// do not handle PH_CONTROL | REQUEST ??
	} else {
		printk(KERN_WARNING "%s %s: unknown prim(%x)\n",
		       hw->card_name, __FUNCTION__, hh->prim);
	}
	if (!ret)
		dev_kfree_skb(skb);
	return (ret);
}


static int
hfcsmini_manager(void *data, u_int prim, void *arg)
{
	hfcsmini_hw *hw = NULL;
	mISDNinstance_t *inst = data;
	struct sk_buff *skb;
	int channel = -1;
	int i;
	dchannel_t *dch = NULL;
	bchannel_t *bch = NULL;
	u_long flags;

	if (!data) {
		MGR_HASPROTOCOL_HANDLER(prim, arg, &hw_mISDNObj)
		    printk(KERN_ERR "%s %s: no data prim %x arg %p\n",
			   hw->card_name, __FUNCTION__, prim, arg);
		return (-EINVAL);
	}
	
	spin_lock_irqsave(&hw_mISDNObj.lock, flags);

	/* find channel and card */
	list_for_each_entry(hw, &hw_mISDNObj.ilist, list) {
		i = 0;
		while (i < MAX_CHAN) {
			if (hw->chan[i].dch)
				if (&hw->chan[i].dch->inst == inst) {
					channel = i;
					dch = hw->chan[i].dch;
					break;
				}
			if (hw->chan[i].bch)
				if (&hw->chan[i].bch->inst == inst) {
					channel = i;
					bch = hw->chan[i].bch;
					break;
				}
			i++;
		}
		if (channel >= 0)
			break;
	}
	spin_unlock_irqrestore(&hw_mISDNObj.lock, flags);
	
	if (channel < 0) {
		printk(KERN_ERR
		       "%s: no card/channel found  data %p prim %x arg %p\n",
		       __FUNCTION__, data, prim, arg);
		return (-EINVAL);
	}

	switch (prim) {
		case MGR_REGLAYER | CONFIRM:
			if (dch)
				dch_set_para(dch, &inst->st->para);
			if (bch)
				bch_set_para(bch, &inst->st->para);
			break;
		case MGR_UNREGLAYER | REQUEST:
			if ((skb = create_link_skb(PH_CONTROL | REQUEST,
			     HW_DEACTIVATE, 0, NULL, 0))) {
				if (channel == 2) {
					if (hfcsmini_l1hwD(inst, skb))
						dev_kfree_skb(skb);
				} else {
					if (hfcsmini_l2l1B(inst, skb))
						dev_kfree_skb(skb);
				}
			} else
				printk(KERN_WARNING "no SKB in %s MGR_UNREGLAYER | REQUEST\n", __FUNCTION__);
			hw_mISDNObj.ctrl(inst, MGR_UNREGLAYER | REQUEST, NULL);
			break;
		case MGR_CLRSTPARA | INDICATION:
			arg = NULL;
		case MGR_ADDSTPARA | INDICATION:
			if (dch)
				dch_set_para(dch, arg);
			if (bch)
				bch_set_para(bch, arg);
			break;
		case MGR_RELEASE | INDICATION:
			if (channel == 2) {
				release_card(hw);
			} else {
				hw_mISDNObj.refcnt--;
			}
			break;
		case MGR_SETSTACK | INDICATION:
			if ((channel != 2) && (inst->pid.global == 2)) {
				if ((skb = create_link_skb(PH_ACTIVATE | REQUEST,
					0, 0, NULL, 0))) {
					if (hfcsmini_l2l1B(inst, skb))
						dev_kfree_skb(skb);
				}
				if (inst->pid.protocol[2] == ISDN_PID_L2_B_TRANS)
					mISDN_queue_data(inst, FLG_MSG_UP, DL_ESTABLISH | INDICATION,
						0, 0, NULL, 0);
				else
					mISDN_queue_data(inst, FLG_MSG_UP, PH_ACTIVATE | INDICATION,
						0, 0, NULL, 0);
			}
			break;
		case MGR_GLOBALOPT | REQUEST:
			if (arg) {
				// FIXME: detect cards with HEADSET
				u_int *gopt = arg;
				*gopt = GLOBALOPT_INTERNAL_CTRL |
				    GLOBALOPT_EXTERNAL_EQUIPMENT |
				    GLOBALOPT_HANDSET;
			} else
				return (-EINVAL);
			break;
		case MGR_SELCHANNEL | REQUEST:
			// no special procedure
			return (-EINVAL);
			PRIM_NOT_HANDLED(MGR_CTRLREADY | INDICATION);
		default:
			printk(KERN_WARNING "%s %s: prim %x not handled\n",
			       hw->card_name, __FUNCTION__, prim);
			return (-EINVAL);
	}
	return (0);
}


/************************************/
/* check if new buffer for DChannel */
/* is waitinng is transmitt queue   */
/************************************/
int
next_d_tx_frame(hfcsmini_hw * hw, __u8 channel)
{
	dchannel_t *dch = hw->chan[channel].dch;

	if (test_and_clear_bit(FLG_TX_NEXT, &dch->DFlags)) {
		struct sk_buff	*skb = dch->next_skb;
		
		if (skb) {
			mISDN_head_t *hh = mISDN_HEAD_P(skb);
			dch->next_skb = NULL;
			test_and_clear_bit(FLG_TX_NEXT, &dch->DFlags);
			
			dch->tx_len = skb->len;
			memcpy(dch->tx_buf, skb->data, dch->tx_len);
			dch->tx_idx = 0;
			skb_trim(skb, 0);
			if (mISDN_queueup_newhead(&dch->inst, 0, PH_DATA_CNF, hh->dinfo, skb))
				dev_kfree_skb(skb);
			return (1);
		} else {
			printk(KERN_WARNING
			       "%s channel(%i) TX_NEXT without skb\n",
			       hw->card_name, channel);
			test_and_clear_bit(FLG_TX_NEXT, &dch->DFlags);
			test_and_clear_bit(FLG_TX_BUSY, &dch->DFlags);
		}
	} else {
		test_and_clear_bit(FLG_TX_BUSY, &dch->DFlags);
	}
	return (0);
}


/************************************/
/* check if new buffer for BChannel */
/* is waitinng is transmitt queue   */
/************************************/
int
next_b_tx_frame(hfcsmini_hw * hw, __u8 channel)
{
	bchannel_t *bch = hw->chan[channel].bch;

	if (test_and_clear_bit(BC_FLG_TX_NEXT, &bch->Flag)) {
		struct sk_buff	*skb = bch->next_skb;
		
		if (skb) {
			mISDN_head_t *hh = mISDN_HEAD_P(skb);
			bch->next_skb = NULL;
			test_and_clear_bit(BC_FLG_TX_NEXT, &bch->Flag);
			bch->tx_idx = 0;
			bch->tx_len = skb->len;
			memcpy(bch->tx_buf, skb->data, bch->tx_len);
			skb_trim(skb, 0);
			queue_bch_frame(bch, CONFIRM, hh->dinfo, skb);
			return (1);
		} else {
			printk(KERN_WARNING
			       "%s channel(%i) TX_NEXT without skb\n",
			       hw->card_name, channel);
			test_and_clear_bit(BC_FLG_TX_NEXT, &bch->Flag);
		}
	}
	return (0);
}


static inline void
hfcsmini_waitbusy(hfcsmini_hw * hw)
{
	while (read_hfcsmini(hw, R_STATUS) & M_BUSY);
}


static inline void
hfcsmini_selfifo(hfcsmini_hw * hw, __u8 fifo)
{
	write_hfcsmini(hw, R_FIFO, fifo);
	hfcsmini_waitbusy(hw);
}


static inline void
hfcsmini_inc_f(hfcsmini_hw * hw)
{
	write_hfcsmini(hw, A_INC_RES_FIFO, M_INC_F);
	hfcsmini_waitbusy(hw);
}


static inline void
hfcsmini_resetfifo(hfcsmini_hw * hw)
{
	write_hfcsmini(hw, A_INC_RES_FIFO, M_RES_FIFO);
	hfcsmini_waitbusy(hw);
}


/**************************/
/* fill fifo with TX data */
/**************************/
void
hfcsmini_write_fifo(hfcsmini_hw * hw, __u8 channel)
{
	__u8 *buf = NULL;
	int *len = NULL, *idx = NULL;
	mISDNinstance_t *inst = NULL;
	__u8 hdlc = 0;
	__u8 fcnt, tcnt, i;
	__u8 free;
	__u8 f1, f2;
	__u8 fstat;
	__u8 *data;

	dchannel_t *dch = hw->chan[channel].dch;
	bchannel_t *bch = hw->chan[channel].bch;

	/* get skb, fifo & mode */
	if (dch) {
		inst = &dch->inst;
		buf = dch->tx_buf;
		len = &dch->tx_len;
		idx = &dch->tx_idx;
		hdlc = 1;
	}
	if (bch) {
		inst = &bch->inst;
		buf = bch->tx_buf;
		len = &bch->tx_len;
		idx = &bch->tx_idx;
		if (bch->protocol == ISDN_PID_L1_B_64HDLC)
			hdlc = 1;
	}

	if (!(inst)) {
		printk(KERN_INFO
		       "%s %s no instance found for channel(%i) !\n ",
		       hw->card_name, __FUNCTION__, channel);
		return;
	}
	
      send_buffer:
      
	if (*len) {
		hfcsmini_selfifo(hw, (channel * 2));

		free = (hw->max_z - (read_hfcsmini_stable(hw, A_USAGE)));
		tcnt = ((free >= (*len - *idx)) ? (*len - *idx) : free);
		fstat = read_hfcsmini(hw, R_ST_RD_STA);

		f1 = read_hfcsmini_stable(hw, A_F1);
		f2 = read_hfcsmini(hw, A_F2);
		fcnt = 0x07 - ((f1 - f2) & 0x07);	/* free frame count in tx fifo */

		if (debug & DEBUG_HFC_FIFO) {
			mISDN_debugprint(inst,
					 "%s channel(%i) len(%i) idx(%i) f1(%i) "
					 "f2(%i) fcnt(%i) tcnt(%i) free(%i) fstat(%i)",
					 __FUNCTION__, channel, *len, *idx,
					 f1, f2, fcnt, tcnt, free, fstat);
		}

		if (free && fcnt && tcnt) {
			data = buf + *idx;
			*idx += tcnt;

			if (debug & DEBUG_HFC_FIFO) {
				printk("%s channel(%i) writing: ",
				       hw->card_name, channel);
			}
			i = tcnt;
			
			/* write data to Fifo */
			while (i--) {
				if (debug & DEBUG_HFC_FIFO)
					printk("%02x ", *data);
				write_hfcsmini(hw, A_FIFO_DATA, *data++);
			}
			if (debug & DEBUG_HFC_FIFO)
				printk("\n");
			
			if (*idx == *len) {
				if (hdlc) {
					/* terminate frame */
					hfcsmini_inc_f(hw);
				} else {
					hfcsmini_selfifo(hw, (channel * 2));
				}
				*len = 0;

				if (dch) {
					if (debug & DEBUG_HFC_DTRACE)
						mISDN_debugprint(&dch->
								 inst,
								 "TX frame channel(%i) completed",
								 channel);

					if (next_d_tx_frame(hw, channel)) {
						if (debug &
						    DEBUG_HFC_DTRACE)
							mISDN_debugprint
							    (&dch->inst,
							     "channel(%i) has next_d_tx_frame",
							     channel);
					}
				}

				if (bch) {
					if (debug & DEBUG_HFC_BTRACE)
						mISDN_debugprint(&bch->
								 inst,
								 "TX frame channel(%i) completed",
								 channel);

					if (next_b_tx_frame(hw, channel)) {
						if (debug &
						    DEBUG_HFC_BTRACE)
							mISDN_debugprint
							    (&bch->inst,
							     "channel(%i) has next_b_tx_frame",
							     channel);

						if ((free - tcnt) > 8) {
							if (debug &
							    DEBUG_HFC_BTRACE)
								mISDN_debugprint
								    (&bch->
								     inst,
								     "channel(%i) continue B-TX immediatetly",
								     channel, channel);
							goto send_buffer;
						}
					}
					test_and_clear_bit(BC_FLG_TX_BUSY, &bch->Flag);
				}
								
			} else {
				/* tx buffer not complete, but fifo filled to maximum */
				hfcsmini_selfifo(hw, (channel * 2));
			}
				
						
		}
		
	}
}


/****************************/
/* read RX data out of fifo */
/****************************/
void
hfcsmini_read_fifo(hfcsmini_hw * hw, __u8 channel)
{
	__u8 f1 = 0, f2 = 0, z1, z2;
	__u8 fstat = 0;
	__u8 hdlc = 0;
	int rcnt;		/* read rcnt bytes out of fifo */
	__u8 *buf = NULL;
	int *idx = NULL;
	int max = 0;
	mISDNinstance_t *inst = NULL;
	__u8 *data;		/* new data pointer */
	struct sk_buff *skb;	/* data buffer for upper layer */
	__u16 i;

	dchannel_t *dch = hw->chan[channel].dch;
	bchannel_t *bch = hw->chan[channel].bch;

	if (dch) {
		inst = &dch->inst;
		buf = hw->chan[channel].rx_buf;
		idx = &hw->chan[channel].rx_idx;
		max = MAX_DFRAME_LEN_L1;
		hdlc = 1;
	}

	if (bch) {
		inst = &bch->inst;
		buf = bch->rx_buf;
		idx = &bch->rx_idx;
		max = MAX_DATA_MEM;
		hdlc = (bch->protocol == ISDN_PID_L1_B_64HDLC);
	}

	if (!(inst)) {
		printk(KERN_INFO
		       "%s %s no instance found for channel(%i)!\n ",
		       hw->card_name, __FUNCTION__, channel);
		return;
	}


      receive_buffer:

	hfcsmini_selfifo(hw, (channel * 2) + 1);

	if (hdlc) {
		/* hdlc rcnt */
		f1 = read_hfcsmini_stable(hw, A_F1);
		f2 = read_hfcsmini(hw, A_F2);
		z1 = read_hfcsmini_stable(hw, A_Z1);
		z2 = read_hfcsmini(hw, A_Z2);
		fstat = read_hfcsmini(hw, R_ST_RD_STA);

		rcnt = (z1 - z2) & hw->max_z;
		if (f1 != f2)
			rcnt++;

	} else {
		/* transparent rcnt */
		rcnt = read_hfcsmini_stable(hw, A_USAGE) - 1;
		f1=f2=z1=z2=0;
	}

	if (debug & DEBUG_HFC_FIFO)
		mISDN_debugprint(inst, "reading %i bytes channel(%i) "
				 "irq_cnt(%i) fstat(%i) idx(%i) f1(%i) f2(%i) z1(%i) z2(%i)",
				 rcnt, channel, hw->irq_cnt, fstat,
				 *idx, f1, f2, z1, z2);

	if (rcnt > 0) {
		data = buf + *idx;
		*idx += rcnt;
		
		/* read data from FIFO*/
		while (rcnt--)
			*data++ = read_hfcsmini(hw, A_FIFO_DATA);
		
	} else
		goto read_exit;


	if (hdlc) {
		if (f1 != f2) {
			hfcsmini_inc_f(hw);

			/* check minimum frame size */
			if (*idx < 4) {
				if (debug & DEBUG_HFC_FIFO_ERR)
					mISDN_debugprint(inst,
							 "%s: frame in channel(%i) < minimum size",
							 __FUNCTION__,
							 channel);
				goto read_exit;
			}
			
			/* check crc */
			if (buf[(*idx) - 1]) {
				if (debug & DEBUG_HFC_FIFO_ERR)
					mISDN_debugprint(inst,
							 "%s: channel(%i) CRC-error",
							 __FUNCTION__,
							 channel);
				goto read_exit;	
			}
			
			/* alloc skb and copy receive buffer to skb */
			if (!(skb = alloc_stack_skb((*idx) - 3,
						    (bch) ? hw->
						    chan[channel].bch->
						    up_headerlen : hw->
						    chan[channel].dch->
						    up_headerlen))) {
				mISDN_debugprint(inst,
						 "%s: No mem for skb",
						 __FUNCTION__);
				return;
			}
			memcpy(skb_put(skb, (*idx) - 3), buf, (*idx) - 3);


			if (dch) {
				if ((dch->debug)
				    && (debug & DEBUG_HFC_DTRACE)) {
					mISDN_debugprint(inst,
							 "channel(%i) new D-RX len(%i): ",
							 channel,
							 (*idx) - 3);
					i = 0;
					printk("  ");
					while (i < (*idx) - 3)
						printk("%02x ",
						       skb->data[i++]);
					printk("\n");
				}
				mISDN_queueup_newhead(&dch->inst, 0, PH_DATA_IND, MISDN_ID_ANY, skb);
			}
			if (bch) {
				if ((bch->debug)
				    && (debug & DEBUG_HFC_BTRACE)) {
					mISDN_debugprint(inst,
							 "channel(%i) new B-RX len(%i) ",
							 channel,
							 (*idx) - 3);
					i = 0;
					printk("  ");
					while (i < (*idx) - 3)
						printk("%02x ",
						       skb->data[i++]);
					printk("\n");
				}
				queue_bch_frame(bch, INDICATION, MISDN_ID_ANY, skb);
			}

		      read_exit:
			*idx = 0;
			if (read_hfcsmini_stable(hw, A_USAGE) > 8) {
				if (debug & DEBUG_HFC_FIFO)
					mISDN_debugprint(inst,
							 "%s: channel(%i) continue hfcsmini_read_fifo",
							 __FUNCTION__,
							 channel);
				goto receive_buffer;
			}
			return;


		} else {
			hfcsmini_selfifo(hw, (channel * 2) + 1);
		}
	} else {
		if (bch) {
			hfcsmini_selfifo(hw, (channel * 2) + 1);
			if (*idx >= 128) {
				/* deliver transparent data to layer2 */
				if (!(skb = alloc_stack_skb(*idx,
							    hw->
							    chan[channel].
							    bch->
							    up_headerlen)))
				{
					mISDN_debugprint(inst,
							 "%s: No mem for skb",
							 __FUNCTION__);
					return;
				}
				memcpy(skb_put(skb, *idx), buf, *idx);
				*idx = 0;

				queue_bch_frame(bch, INDICATION, MISDN_ID_ANY, skb);
			}
		}
	}
}


/*************************************/
/* bottom half handler for interrupt */
/*************************************/
static void
hfcsmini_bh_handler(unsigned long ul_hw)
{
	hfcsmini_hw *hw = (hfcsmini_hw *) ul_hw;
	reg_r_st_rd_sta state;
	dchannel_t *dch;
	
	int i;
	
	if (hw->misc_irq.bit.v_ti_irq) {	/* Timer Int */
		hw->misc_irq.bit.v_ti_irq = 0;
		
		for (i = 0; i < hw->max_fifo; i++) {
			/* add Fifo-Fill info into int_s1 bitfield */
			hw->fifo_irq.reg |= ((read_hfcsmini(hw, R_FILL) ^ FIFO_MASK_TX) & hw->fifomask);
		
			/* Handle TX Fifos */	
			if ((1 << (i * 2)) & (hw->fifo_irq.reg)) {
				hw->fifo_irq.reg &= ~(1 << (i * 2));
				
				if (hw->chan[i].dch)
					if (test_bit
					    (FLG_TX_BUSY,
					     &hw->chan[i].dch->DFlags)) {
						hfcsmini_write_fifo(hw, i);
					}

				if (hw->chan[i].bch)
					if (test_bit
					    (BC_FLG_TX_BUSY,
					     &hw->chan[i].bch->Flag)
					    && (hw->chan[i].bch->protocol)) {
						hfcsmini_write_fifo(hw, i);
					}
			}
		}
	}
	
	/* Handle RX Fifos */
	for (i = 0; i < hw->max_fifo; i++) {
		if ((1 << (i * 2 + 1)) & (hw->fifo_irq.reg)) {
			hw->fifo_irq.reg &= ~(1 << (i * 2 + 1));
			hfcsmini_read_fifo(hw, i);
		}
	}
	
		
	if (hw->misc_irq.bit.v_st_irq) {	/* state machine IRQ */
		hw->misc_irq.bit.v_st_irq = 0;
		
		state.reg = read_hfcsmini(hw, R_ST_RD_STA);
		dch = hw->chan[2].dch;
		
		if (dch) {
			/*
			mISDN_debugprint(&dch->inst,
				 "new_l1_state(0x%02x)", state.bit.v_st_sta);
			*/
			if (state.bit.v_st_sta != dch->ph_state) {
				dch->ph_state = state.bit.v_st_sta;
				s0_new_state(dch);
			}
		}		
	}
	

	return;
}


/*********************/
/* Interrupt handler */
/*********************/
static irqreturn_t
hfcsmini_interrupt(int intno, void *dev_id, struct pt_regs *regs)
{
	__u8 fifo_irq, misc_irq;
	hfcsmini_hw *hw = dev_id;

	spin_lock(&hw->rlock);
	
	if (!(hw->misc_irqmsk.bit.v_irq_en)) {
		if (!(hw->testirq))
			printk(KERN_INFO
			       "%s %s GLOBAL INTERRUPT DISABLED\n",
			       hw->card_name, __FUNCTION__);		
		spin_unlock(&hw->rlock);
		return IRQ_NONE;
	} 

	fifo_irq = read_hfcsmini_irq(hw, R_FIFO_IRQ) & hw->fifo_irqmsk.reg;
	misc_irq = read_hfcsmini_irq(hw, R_MISC_IRQ) & hw->misc_irqmsk.reg;
	
	if (!fifo_irq && !misc_irq) {
		spin_unlock(&hw->rlock);
		return IRQ_NONE; /* other hardware interrupted */
	}
	
	hw->irq_cnt++;
	
	hw->fifo_irq.reg |= fifo_irq;
	hw->misc_irq.reg |= misc_irq;
	
	/* queue bottom half */
	if (!(hw->testirq)) {
		tasklet_schedule(&hw->tasklet);
	}
	
	spin_unlock(&hw->rlock);
	
	return IRQ_HANDLED;
}


/*************************************/
/* free memory for all used channels */
/*************************************/
void
release_channels(hfcsmini_hw * hw)
{
	int i;

	i = 0;
	while (i < MAX_CHAN) {
		if (hw->chan[i].dch) {
			if (debug & DEBUG_HFC_INIT)
				printk(KERN_DEBUG
				       "%s %s: free D-channel %d\n",
				       hw->card_name, __FUNCTION__, i);
			mISDN_free_dch(hw->chan[i].dch);
			hw_mISDNObj.ctrl(&hw->chan[i].dch->inst, MGR_UNREGLAYER | REQUEST, NULL);
			kfree(hw->chan[i].dch);
			hw->chan[i].dch = NULL;
		}
		if (hw->chan[i].rx_buf) {
			kfree(hw->chan[i].rx_buf);
			hw->chan[i].rx_buf = NULL;
		}
		if (hw->chan[i].bch) {
			if (debug & DEBUG_HFC_INIT)
				printk(KERN_DEBUG
				       "%s %s: free B-channel %d\n",
				       hw->card_name, __FUNCTION__, i);
			mISDN_free_bch(hw->chan[i].bch);
			kfree(hw->chan[i].bch);
			hw->chan[i].bch = NULL;
		}
		i++;
	}
}


/******************************************/
/* Setup Fifo using HDLC_PAR and CON_HDLC */
/******************************************/
void setup_fifo(hfcsmini_hw * hw, int fifo, __u8 hdlcreg, __u8 con_reg, __u8 irq_enable, __u8 enable)
{       
	
	if (enable)
		/* mark fifo to be 'in use' */
		hw->fifomask |= (1 << fifo);
	else
		hw->fifomask &= ~(1 << fifo);

	if (irq_enable)
		hw->fifo_irqmsk.reg |= (1 << fifo);
	else
		hw->fifo_irqmsk.reg &= ~(1 << fifo);
		
	write_hfcsmini(hw, R_FIFO_IRQMSK, hw->fifo_irqmsk.reg);
	
	hfcsmini_selfifo(hw, fifo);
	write_hfcsmini(hw, A_HDLC_PAR, hdlcreg); 
	write_hfcsmini(hw, A_CON_HDLC, con_reg); 
	hfcsmini_resetfifo(hw);
}


/*************************************************/
/* Setup ST interface, enable/disable B-Channels */
/*************************************************/
void
setup_st(hfcsmini_hw * hw, __u8 bc, __u8 enable)
{
	if (!((bc == 0) || (bc == 1))) {
		printk(KERN_INFO "%s %s: ERROR: bc(%i) unvalid!\n",
		       hw->card_name, __FUNCTION__, bc);
		return;
	}

	if (bc) {
		hw->st_ctrl0.bit.v_b2_en = (enable?1:0);
		hw->st_ctrl2.bit.v_b2_rx_en = (enable?1:0);
	} else {
		hw->st_ctrl0.bit.v_b1_en = (enable?1:0);
		hw->st_ctrl2.bit.v_b1_rx_en = (enable?1:0);
	}

	write_hfcsmini(hw, R_ST_CTRL0, hw->st_ctrl0.reg);
	write_hfcsmini(hw, R_ST_CTRL2, hw->st_ctrl2.reg);
	
	if (debug & DEBUG_HFC_MODE) {
		printk(KERN_INFO
		       "%s %s: bc(%i) %s, R_ST_CTRL0(0x%02x) R_ST_CTRL2(0x%02x)\n",
		       hw->card_name, __FUNCTION__, bc, enable?"enable":"disable",
		       hw->st_ctrl0.reg, hw->st_ctrl2.reg);
	}
}


/*********************************************/
/* (dis-) connect D/B-Channel using protocol */
/*********************************************/
int
setup_channel(hfcsmini_hw * hw, __u8 channel, int protocol)
{

	if ((hw->chan[channel].dch) && (hw->chan[channel].bch)) {
		printk(KERN_INFO
		       "%s %s ERROR: channel(%i) is B and D !!!\n",
		       hw->card_name, __FUNCTION__, channel);
		return (-1);
	}


	if (hw->chan[channel].bch) {
		if (debug & DEBUG_HFC_MODE)
			mISDN_debugprint(&hw->chan[channel].bch->inst,
					 "channel(%i) protocol %x-->%x",
					 channel,
					 hw->chan[channel].bch->protocol,
					 protocol);

		switch (protocol) {
			case (-1):	/* used for init */
				hw->chan[channel].bch->protocol = -1;
				hw->chan[channel].bch->channel = channel;
				/* fall trough */
			case (ISDN_PID_NONE):
				if (debug & DEBUG_HFC_MODE)
					mISDN_debugprint(&hw->
							 chan[channel].
							 bch->inst,
							 "ISDN_PID_NONE");
				if (hw->chan[channel].bch->protocol ==
				    ISDN_PID_NONE)
					return (0);	/* already in idle state */
				hw->chan[channel].bch->protocol =
				    ISDN_PID_NONE;
				    
				setup_fifo(hw,			/* B-TX */
			                   (channel << 1),
			                   0,
			                   0,
			                   FIFO_IRQ_OFF,
			                   FIFO_DISABLE); 
				                   
				setup_fifo(hw,			/* B-RX */
			                   (channel << 1) + 1,
			                   0,
			                   0,
			                   FIFO_IRQ_OFF,
			                   FIFO_DISABLE);

				setup_st(hw, channel, 0);

				break;

			case (ISDN_PID_L1_B_64TRANS):
				if (debug & DEBUG_HFC_MODE)
					mISDN_debugprint(&hw->
							 chan[channel].
							 bch->inst,
							 "ISDN_PID_L1_B_64TRANS");

				setup_fifo(hw,		/* B-TX */
			                   (channel << 1),
			                   HDLC_PAR_BCH,
			                   CON_HDLC_B_TRANS,
			                   FIFO_IRQ_OFF,
			                   FIFO_ENABLE); 
				                   
				setup_fifo(hw,		/* B-RX */
			                   (channel << 1) + 1,
			                   HDLC_PAR_BCH,
			                   CON_HDLC_B_TRANS,
			                   FIFO_IRQ_OFF,
			                   FIFO_ENABLE);
				                   							 
				setup_st(hw, channel, 1);

				hw->chan[channel].bch->protocol =
				    ISDN_PID_L1_B_64TRANS;

				break;


			case (ISDN_PID_L1_B_64HDLC):
				if (debug & DEBUG_HFC_MODE)
					mISDN_debugprint(&hw->
							 chan[channel].
							 bch->inst,
							 "ISDN_PID_L1_B_64HDLC");
							 
				setup_fifo(hw,			/* B-TX */
			                   (channel << 1),
			                   HDLC_PAR_BCH,
			                   CON_HDLC_B_HDLC,
			                   FIFO_IRQ_OFF,
			                   FIFO_ENABLE); 
				                   
				setup_fifo(hw,			/* B-RX */
			                   (channel << 1) + 1,
			                   HDLC_PAR_BCH,
			                   CON_HDLC_B_HDLC,
			                   FIFO_IRQ_ON,
			                   FIFO_ENABLE);

				setup_st(hw, channel, 1);

				hw->chan[channel].bch->protocol =
				    ISDN_PID_L1_B_64HDLC;

				break;
			default:
				mISDN_debugprint(&hw->chan[channel].bch->
						 inst, "prot not known %x",
						 protocol);
				return (-ENOPROTOOPT);
		}
		return (0);
	}

	if (hw->chan[channel].dch) {
		if (debug & DEBUG_HFC_MODE)
			mISDN_debugprint(&hw->chan[channel].dch->inst,
					 "D channel(%i) protocol(%i)",
					 channel, protocol);
		
		/* init the D-channel fifos */
		setup_fifo(hw,			/* D-TX */
	                   (channel << 1),
	                   HDLC_PAR_DCH,
	                   CON_HDLC_D_HDLC,
	                   FIFO_IRQ_OFF,
	                   FIFO_ENABLE); 

		setup_fifo(hw,			/* D-RX */
	                   (channel << 1) + 1,
	                   HDLC_PAR_DCH,
	                   CON_HDLC_D_HDLC,
	                   FIFO_IRQ_ON,
	                   FIFO_DISABLE);

		return (0);
	}

	printk(KERN_INFO
	       "%s %s ERROR: channel(%i) is NEITHER B nor D !!!\n",
	       hw->card_name, __FUNCTION__, channel);

	return (-1);
}


/*****************************************************/
/* register ISDN stack for one HFC-S mini instance   */
/*   - register all ports and channels               */
/*   - set param_idx                                 */
/*                                                   */
/*  channel mapping in mISDN in hw->chan[]           */
/*    0=B1,  1=B2,  2=D,  3=PCM                      */
/*****************************************************/
int
init_mISDN_channels(hfcsmini_hw * hw)
{
	int err;
	int ch;
	int b;
	dchannel_t *dch;
	bchannel_t *bch;
	mISDN_pid_t pid;
	u_long flags;

	/* init D channels */
	ch = 2;
	if (debug & DEBUG_HFC_INIT)
		printk(KERN_INFO
		       "%s %s: Registering D-channel, card(%d) protocol(%x)\n",
		       hw->card_name, __FUNCTION__, hw->cardnum,
		       hw->dpid);

	dch = kmalloc(sizeof(dchannel_t), GFP_ATOMIC);
	if (!dch) {
		err = -ENOMEM;
		goto free_channels;
	}
	memset(dch, 0, sizeof(dchannel_t));
	dch->channel = ch;
	dch->debug = debug;
	dch->inst.obj = &hw_mISDNObj;
	dch->inst.hwlock = &hw->mlock;
	dch->inst.class_dev.dev = &hw->pdev->dev;
	mISDN_init_instance(&dch->inst, &hw_mISDNObj, hw, hfcsmini_l1hwD);
	
	dch->inst.pid.layermask = ISDN_LAYER(0);
	sprintf(dch->inst.name, "%s", hw->card_name);

	if (!
	    (hw->chan[ch].rx_buf =
	     kmalloc(MAX_DFRAME_LEN_L1, GFP_ATOMIC))) {
		err = -ENOMEM;
		goto free_channels;
	}

	if (mISDN_init_dch(dch)) {
		err = -ENOMEM;
		goto free_channels;
	}
	dch->hw = hw;
	hw->chan[ch].dch = dch;

	/* init B channels */
	for (b = 0; b < 2; b++) {
		ch = b;
		if (debug & DEBUG_HFC_INIT)
			printk(KERN_DEBUG
			       "%s %s: Registering B-channel, card(%d) "
			       "ch(%d)\n", hw->card_name,
			       __FUNCTION__, hw->cardnum, ch);

		bch = kmalloc(sizeof(bchannel_t), GFP_ATOMIC);
		if (!bch) {
			err = -ENOMEM;
			goto free_channels;
		}
		memset(bch, 0, sizeof(bchannel_t));
		bch->channel = ch;
		bch->debug = debug;
		mISDN_init_instance(&bch->inst, &hw_mISDNObj, hw, hfcsmini_l2l1B);
		bch->inst.pid.layermask = ISDN_LAYER(0);
		bch->inst.hwlock = &hw->mlock;
		bch->inst.class_dev.dev = &hw->pdev->dev;
		
		sprintf(bch->inst.name, "%s B%d",
			dch->inst.name, b + 1);
		if (mISDN_init_bch(bch)) {
			kfree(bch);
			err = -ENOMEM;
			goto free_channels;
		}
		bch->hw = hw;
		hw->chan[ch].bch = bch;
#ifdef FIXME
		if (bch->dev) {
			bch->dev->wport.pif.func = hfcsmini_l2l1B;
			bch->dev->wport.pif.fdata = bch;
		}
#endif
	}

	mISDN_set_dchannel_pid(&pid, hw->dpid,
			       layermask[hw->param_idx]);

	/* set protocol for NT/TE */
	if (hw->portmode & PORT_MODE_NT) {
		/* NT-mode */
		hw->portmode |= NT_TIMER;
		hw->nt_timer = 0;

		dch->inst.pid.protocol[0] = ISDN_PID_L0_NT_S0;
		dch->inst.pid.protocol[1] = ISDN_PID_L1_NT_S0;
		pid.protocol[0] = ISDN_PID_L0_NT_S0;
		pid.protocol[1] = ISDN_PID_L1_NT_S0;
		dch->inst.pid.layermask |= ISDN_LAYER(1);
		pid.layermask |= ISDN_LAYER(1);
		if (layermask[hw->param_idx] & ISDN_LAYER(2))
			pid.protocol[2] = ISDN_PID_L2_LAPD_NET;
	} else {
		/* TE-mode */
		hw->portmode |= PORT_MODE_TE;
		dch->inst.pid.protocol[0] = ISDN_PID_L0_TE_S0;
		pid.protocol[0] = ISDN_PID_L0_TE_S0;
	}

	if (debug & DEBUG_HFC_INIT)
		printk(KERN_INFO
		       "%s %s: registering Stack\n",
		       hw->card_name, __FUNCTION__);

	/* register stack */
	dch = hw->chan[2].dch;
	err =
	    hw_mISDNObj.ctrl(NULL, MGR_NEWSTACK | REQUEST,
			     &dch->inst);
	if (err) {
		printk(KERN_ERR
		       "%s %s: MGR_NEWSTACK | REQUEST  err(%d)\n",
		       hw->card_name, __FUNCTION__, err);
		goto free_channels;
	}

	dch->hw_bh = s0_new_state;
	dch->ph_state = 0;

	for (b = 0; b < 2; b++) {
		bch = hw->chan[b].bch;
		
		err =
		    hw_mISDNObj.ctrl(dch->inst.st,
				     MGR_NEWSTACK | REQUEST,
				     &bch->inst);
		if (err) {
			printk(KERN_ERR
			       "%s %s: MGR_ADDSTACK bchan error %d\n",
			       hw->card_name, __FUNCTION__, err);
			goto free_stack;
		}
		bch->st = bch->inst.st;
	}

	err =
	    hw_mISDNObj.ctrl(dch->inst.st, MGR_SETSTACK | REQUEST,
			     &pid);

	if (err) {
		printk(KERN_ERR
		       "%s %s: MGR_SETSTACK REQUEST dch err(%d)\n",
		       hw->card_name, __FUNCTION__, err);
		hw_mISDNObj.ctrl(dch->inst.st,
				 MGR_DELSTACK | REQUEST, NULL);
		goto free_stack;
	}

	setup_channel(hw, dch->channel, -1);
	for (b = 0; b < 2; b++) {
		setup_channel(hw, b, -1);
	}

	/* delay some time */
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout((100 * HZ) / 1000);	/* Timeout 100ms */

	hw_mISDNObj.ctrl(dch->inst.st, MGR_CTRLREADY | INDICATION,
			 NULL);

	return (0);

      free_stack:
	hw_mISDNObj.ctrl(dch->inst.st, MGR_DELSTACK | REQUEST, NULL);
      free_channels:
      	spin_lock_irqsave(&hw_mISDNObj.lock, flags);
	release_channels(hw);
	list_del(&hw->list);
	spin_unlock_irqrestore(&hw_mISDNObj.lock, flags);

	return (err);
}


/********************************/
/* parse module paramaters like */
/* NE/TE and S0/Up port mode    */
/********************************/
void
parse_module_params(hfcsmini_hw * hw)
{

	/* D-Channel protocol: (2=DSS1) */
	hw->dpid = (protocol[hw->param_idx] & 0x0F);
	if (hw->dpid == 0) {
		printk(KERN_INFO
		       "%s %s: WARNING: wrong value for protocol[%i], "
		       "assuming 0x02 (DSS1)...\n",
		       hw->card_name, __FUNCTION__,
		       hw->param_idx);
		hw->dpid = 0x02;
	}

	/* Line Interface TE or NT */
	if (protocol[hw->param_idx] & 0x10)
		hw->portmode |= PORT_MODE_NT;
	else
		hw->portmode |= PORT_MODE_TE;

	/* Line Interface in S0 or Up mode */
	if (!(protocol[hw->param_idx] & 0x40))
		hw->portmode |= PORT_MODE_BUS_MASTER;

		
	/* link B-channel loop */
	if (protocol[hw->param_idx] & 0x80)
		hw->portmode |= PORT_MODE_LOOP;
	

	if (debug & DEBUG_HFC_INIT)
		printk ("%s %s: protocol[%i]=0x%02x, dpid=%d,%s bus-mode:%s %s\n",
		        hw->card_name, __FUNCTION__, hw->param_idx,
		        protocol[hw->param_idx],
		        hw->dpid,
		        (hw->portmode & PORT_MODE_TE)?"TE":"NT",
		        (hw->portmode & PORT_MODE_BUS_MASTER)?"MASTER":"SLAVE",
		        (hw->portmode & PORT_MODE_LOOP)?"B-LOOP":""
		        );
}


/*****************************************/
/* initialise the HFC-S mini ISDN Chip   */
/* return 0 on success.                  */
/*****************************************/
int
init_hfcsmini(hfcsmini_hw * hw)
{
	int err = 0;
	reg_r_fifo_thres threshold;

#if HFCBRIDGE == BRIDGE_HFCPCI
	err = init_pci_bridge(hw);
	if (err)
		return(-ENODEV);
#endif

	hw->chip_id.reg = read_hfcsmini(hw, R_CHIP_ID);

	if (debug & DEBUG_HFC_INIT)
		printk(KERN_INFO "%s %s ChipID: 0x%x\n", hw->card_name,
		       __FUNCTION__, hw->chip_id.bit.v_chip_id);

	switch (hw->chip_id.bit.v_chip_id) {
		case CHIP_ID_HFCSMINI:
			hw->max_fifo = 4;
			hw->ti.reg   = 5; /* 8 ms timer interval */
			hw->max_z    = 0x7F;
			break;
		default:
			err = -ENODEV;
	}

	if (err) {
		if (debug & DEBUG_HFC_INIT)
			printk(KERN_ERR "%s %s: unkown Chip ID 0x%x\n",
			       hw->card_name, __FUNCTION__, hw->chip_id.bit.v_chip_id);
		return (err);
	}
	
	/* reset card */
	write_hfcsmini(hw, R_CIRM, M_SRES); /* Reset On */
	udelay(10);
	write_hfcsmini(hw, R_CIRM, 0); /* Reset Off */
	
	/* wait until fifo controller init sequence is finished */
	hfcsmini_waitbusy(hw);

	/* reset D-Channel S/T controller */
	write_hfcsmini(hw, R_ST_CTRL1, M_D_RES);

	if (hw->portmode & PORT_MODE_TE) {
		/* TE mode */
		hw->st_ctrl0.reg = 0;
		write_hfcsmini(hw, R_ST_CLK_DLY, (M1_ST_CLK_DLY* 0xF));
		write_hfcsmini(hw, R_ST_CTRL1, 0);
	} else {
		/* NT mode */
		hw->st_ctrl0.reg = 4;
		write_hfcsmini(hw, R_ST_CLK_DLY, ((M1_ST_SMPL * 0x6) | (M1_ST_CLK_DLY*0xC)));
		write_hfcsmini(hw, R_ST_CTRL1, M_E_IGNO);
	}
	
	hw->st_ctrl2.reg = 0;
	write_hfcsmini(hw, R_ST_CTRL0, hw->st_ctrl0.reg);
	write_hfcsmini(hw, R_ST_CTRL2, hw->st_ctrl2.reg);

	/* HFC Master/Slave Mode */
	if (hw->portmode & PORT_MODE_BUS_MASTER)
		hw->pcm_md0.bit.v_pcm_md = 1;
	else
		hw->pcm_md0.bit.v_pcm_md = 0;

	write_hfcsmini(hw, R_PCM_MD0, hw->pcm_md0.reg);
	write_hfcsmini(hw, R_PCM_MD1, 0);
	write_hfcsmini(hw, R_PCM_MD2, 0);

	/* setup threshold register */
	threshold.bit.v_thres_tx = (HFCSMINI_TX_THRESHOLD / 8);
	threshold.bit.v_thres_rx = (HFCSMINI_RX_THRESHOLD / 8);
	write_hfcsmini(hw, R_FIFO_THRES, threshold.reg);

	/* test timer irq */
	enable_interrupts(hw);
	mdelay(((1 << hw->ti.reg)+1)*2);
	hw->testirq = 0;
	
	if (hw->irq_cnt) {
		printk(KERN_INFO
		       "%s %s: test IRQ OK, irq_cnt %i\n",
		       hw->card_name, __FUNCTION__, hw->irq_cnt);
		disable_interrupts(hw);
		return (0);
	} else {
		if (debug & DEBUG_HFC_INIT)
			printk(KERN_INFO
			       "%s %s: ERROR getting IRQ (irq_cnt %i)\n",
			       hw->card_name, __FUNCTION__, hw->irq_cnt);
		disable_interrupts(hw);
		free_irq(hw->irq, hw);
		return (-EIO);
	}
}


/*****************************************************/
/* disable all interrupts by disabling M_GLOB_IRQ_EN */
/*****************************************************/
void
disable_interrupts(hfcsmini_hw * hw)
{
	u_long flags;
	if (debug & DEBUG_HFC_IRQ)
		printk(KERN_INFO "%s %s\n", hw->card_name, __FUNCTION__);

	spin_lock_irqsave(&hw->mlock, flags);
	hw->fifo_irqmsk.reg = 0;
	hw->misc_irqmsk.reg = 0;	
	write_hfcsmini(hw, R_FIFO_IRQMSK, hw->fifo_irqmsk.reg);
	write_hfcsmini(hw, R_MISC_IRQMSK, hw->misc_irqmsk.reg);
	spin_unlock_irqrestore(&hw->mlock, flags);
}


/******************************************/
/* start interrupt and set interrupt mask */
/******************************************/
void
enable_interrupts(hfcsmini_hw * hw)
{
	u_long flags;
	
	if (debug & DEBUG_HFC_IRQ)
		printk(KERN_INFO "%s %s\n", hw->card_name, __FUNCTION__);

	spin_lock_irqsave(&hw->mlock, flags);
	
	hw->fifo_irq.reg = 0;
	hw->misc_irq.reg = 0;
	
	write_hfcsmini(hw, R_TI, hw->ti.reg); 

	/* D-RX and D-TX interrupts enable */
	hw->fifo_irqmsk.bit.v_fifo2_tx_irqmsk = 1;
	hw->fifo_irqmsk.bit.v_fifo2_rx_irqmsk = 1;	

	/* clear pending ints */
	if (read_hfcsmini(hw, R_FIFO_IRQ));
	if (read_hfcsmini(hw, R_MISC_IRQ));
	
	/* Finally enable IRQ output */
	hw->misc_irqmsk.bit.v_st_irqmsk = 1; /* enable L1-state change irq */
	hw->misc_irqmsk.bit.v_ti_irqmsk = 1; /* enable timer irq */
	hw->misc_irqmsk.bit.v_irq_en    = 1; /* IRQ global enable */
	
	write_hfcsmini(hw, R_MISC_IRQMSK, hw->misc_irqmsk.reg);
	
	spin_unlock_irqrestore(&hw->mlock, flags);
	
	return;
}


/**************************************/
/* initialise the HFC-S mini hardware */
/* return 0 on success.               */
/**************************************/
static int __devinit
setup_instance(hfcsmini_hw * hw)
{
	int		err;
	hfcsmini_hw *previous_hw;
	u_long		flags;
	

	if (debug & DEBUG_HFC_INIT)
		printk(KERN_WARNING "%s %s\n",
		       hw->card_name, __FUNCTION__);

	spin_lock_init(&hw->mlock);
	spin_lock_init(&hw->rlock);
	tasklet_init(&hw->tasklet, hfcsmini_bh_handler, (unsigned long) hw);
	
	/* search previous instances to index protocol[] array */
	list_for_each_entry(previous_hw, &hw_mISDNObj.ilist, list)
		hw->param_idx++;

	/* add this instance to hardware list */
	spin_lock_irqsave(&hw_mISDNObj.lock, flags);
	list_add_tail(&hw->list, &hw_mISDNObj.ilist);
	spin_unlock_irqrestore(&hw_mISDNObj.lock, flags);

	/* init interrupt engine */
	hw->testirq = 1;
	if (debug & DEBUG_HFC_INIT)
		printk(KERN_WARNING "%s %s: requesting IRQ %d\n",
		       hw->card_name, __FUNCTION__, hw->irq);
		       
	if (request_irq(hw->irq, hfcsmini_interrupt, SA_SHIRQ, "HFC-S mini", hw)) {
		printk(KERN_WARNING "%s %s: couldn't get interrupt %d\n",
		       hw->card_name, __FUNCTION__, hw->irq);
		       
		hw->irq = 0;
		err = -EIO;
		goto out;
	}

	parse_module_params(hw);
	
	err = init_hfcsmini(hw);
	if (err)
		goto out;

	/* register all channels at ISDN procol stack */
	err = init_mISDN_channels(hw);
	if (err)
		goto out;
		
	/* delay some time to have mISDN initialazed complete */
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout((100 * HZ) / 1000);	/* Timeout 100ms */
	
	/* Clear already pending ints */
	if (read_hfcsmini(hw, R_FIFO_IRQ));
	
	enable_interrupts(hw);
	
	/* enable state machine */
	write_hfcsmini(hw, R_ST_RD_STA, 0x0);
	
	return(0);

      out:
	return (err);
}


#if HFCBRIDGE == BRIDGE_HFCPCI

/***********************/
/* PCI Bridge ID List  */
/***********************/
static struct pci_device_id hfcsmini_ids[] = {
	{.vendor = PCI_VENDOR_ID_CCD,
	 .device = 0xA001,
	 .subvendor = PCI_VENDOR_ID_CCD,
	 .subdevice = 0xFFFF,
	 .driver_data =
	 (unsigned long) &((hfcsmini_param) {0xFF, "HFC-S mini Evaluation Board"}),
	 },
	{}
};

/******************************/
/* initialise the PCI Bridge  */
/* return 0 on success.       */
/******************************/
int
init_pci_bridge(hfcsmini_hw * hw)
{
	outb(0x58, hw->iobase + 4); /* ID-register of bridge */
	if ((inb(hw->iobase) & 0xf0) != 0x30) {
		printk(KERN_INFO "%s %s: chip ID for PCI bridge invalid\n",
		       hw->card_name, __FUNCTION__);
		release_region(hw->iobase, 8);
		return(-EIO);
	}

	outb(0x60, hw->iobase + 4); /* CIRM register of bridge */
	outb(0x07, hw->iobase); /* 15 PCI clocks aux access */

	/* reset sequence */
	outb(2, hw->iobase + 3); /* A0 = 1, reset = 0 (active) */
	udelay(10);
	outb(6, hw->iobase + 3); /* A0 = 1, reset = 1 (inactive) */
	outb(0, hw->iobase + 1); /* write dummy register number */

	/* wait until reset sequence finished, can be redefined after schematic review */
	mdelay(300);

	return (0);
}

/************************/
/* release single card  */
/************************/
static void
release_card(hfcsmini_hw * hw)
{
	u_long	flags;

	disable_interrupts(hw);
	free_irq(hw->irq, hw);

	/* wait for pending tasklet to finish */
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout((100 * HZ) / 1000);	/* Timeout 100ms */
	
	spin_lock_irqsave(&hw_mISDNObj.lock, flags);
	release_channels(hw);
	list_del(&hw->list);
	spin_unlock_irqrestore(&hw_mISDNObj.lock, flags);
	
	kfree(hw);
}

/*****************************************/
/* PCI hotplug interface: probe new card */
/*****************************************/
static int __devinit
hfcsmini_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	hfcsmini_param *driver_data = (hfcsmini_param *) ent->driver_data;
	hfcsmini_hw *hw;

	int err = -ENOMEM;

	if (!(hw = kmalloc(sizeof(hfcsmini_hw), GFP_ATOMIC))) {
		printk(KERN_ERR "%s %s: No kmem for HFC-S mini card\n",
		       hw->card_name, __FUNCTION__);
		return (err);
	}
	memset(hw, 0, sizeof(hfcsmini_hw));

	hw->pdev = pdev;
	err = pci_enable_device(pdev);

	if (err)
		goto out;

	hw->cardnum = card_cnt;
	sprintf(hw->card_name, "%s_%d", DRIVER_NAME, hw->cardnum);
	printk(KERN_INFO "%s %s: adapter '%s' found on PCI bus %02x dev %02x\n",
	       hw->card_name, __FUNCTION__, driver_data->device_name,
	       pdev->bus->number, pdev->devfn);

	hw->driver_data = *driver_data;
	hw->irq = pdev->irq;
	
 	hw->iobase = (u_int) get_pcibase(pdev, 0);
	if (!hw->iobase) {
		printk(KERN_WARNING "%s no IO for PCI card found\n",
		       hw->card_name);
		return(-EIO);
	}
	
	if (!request_region(hw->iobase, 8, "hfcmulti")) {
		printk(KERN_WARNING "%s failed to request "
		                    "address space at 0x%04x\n",
		                    hw->card_name,
		                    hw->iobase);
	}
	
        printk(KERN_INFO "%s defined at IOBASE 0x%#x IRQ %d HZ %d\n",
	       hw->card_name, 
	       (u_int) hw->iobase,
	       hw->irq,
	       HZ);

	/* enable IO */
	pci_write_config_word(pdev, PCI_COMMAND, 0x01);
	
	pci_set_drvdata(pdev, hw);
	err = setup_instance(hw);
	
	if (!err) {
		card_cnt++;
		return (0);
	} else {
		goto out;
	}

      out:
	kfree(hw);
	return (err);
};


/**************************************/
/* PCI hotplug interface: remove card */
/**************************************/
static void __devexit
hfcsmini_pci_remove(struct pci_dev *pdev)
{
	hfcsmini_hw *hw = pci_get_drvdata(pdev);
	printk(KERN_INFO "%s %s: removing card\n", hw->card_name,
	       __FUNCTION__);
	release_card(hw);
	card_cnt--;
	pci_disable_device(pdev);
	return;
};


/*****************************/
/* Module PCI driver exports */
/*****************************/
static struct pci_driver hfcsmini_driver = {
	name:DRIVER_NAME,
	probe:hfcsmini_pci_probe,
	remove:__devexit_p(hfcsmini_pci_remove),
	id_table:hfcsmini_ids,
};

MODULE_DEVICE_TABLE(pci, hfcsmini_ids);

#endif

/***************/
/* Module init */
/***************/
static int __init
hfcsmini_init(void)
{
	int err;

	printk(KERN_INFO "HFC-S mini: %s driver Rev. %s (debug=%i)\n",
	       __FUNCTION__, mISDN_getrev(hfcsmini_rev), debug);

#ifdef MODULE
	hw_mISDNObj.owner = THIS_MODULE;
#endif

	INIT_LIST_HEAD(&hw_mISDNObj.ilist);
	spin_lock_init(&hw_mISDNObj.lock);
	hw_mISDNObj.name = DRIVER_NAME;
	hw_mISDNObj.own_ctrl = hfcsmini_manager;

	hw_mISDNObj.DPROTO.protocol[0] = ISDN_PID_L0_TE_S0 |
	    ISDN_PID_L0_NT_S0;
	hw_mISDNObj.DPROTO.protocol[1] = ISDN_PID_L1_NT_S0;
	hw_mISDNObj.BPROTO.protocol[1] = ISDN_PID_L1_B_64TRANS |
	    ISDN_PID_L1_B_64HDLC;
	hw_mISDNObj.BPROTO.protocol[2] = ISDN_PID_L2_B_TRANS |
	    ISDN_PID_L2_B_RAWDEV;

	card_cnt = 0;

	if ((err = mISDN_register(&hw_mISDNObj))) {
		printk(KERN_ERR "HFC-S mini: can't register HFC-S mini, error(%d)\n",
		       err);
		goto out;
	}

#if HFCBRIDGE == BRIDGE_HFCPCI
	err = pci_register_driver(&hfcsmini_driver);
	if (err < 0) {
		goto out;
	}
#if !defined(CONFIG_HOTPLUG)
	if (err == 0) {
		err = -ENODEV;
		pci_unregister_driver(&hfcsmini_driver);
		goto out;
	}
#endif
#endif

	printk(KERN_INFO "HFC-S mini: %d cards installed\n", card_cnt);
	return 0;

      out:
	return (err);
}


static void __exit
hfcsmini_cleanup(void)
{
	int err;

#if HFCBRIDGE == BRIDGE_HFCPCI
	pci_unregister_driver(&hfcsmini_driver);
#endif

	if ((err = mISDN_unregister(&hw_mISDNObj))) {
		printk(KERN_ERR "HFC-S mini: can't unregister HFC-S mini, error(%d)\n",
		       err);
	}
	printk(KERN_INFO "%s: driver removed\n", __FUNCTION__);
}

module_init(hfcsmini_init);
module_exit(hfcsmini_cleanup);
