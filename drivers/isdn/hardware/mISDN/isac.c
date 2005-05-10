/* $Id$
 *
 * isac.c   ISAC specific routines
 *
 * Author       Karsten Keil (keil@isdn4linux.de)
 *
 *		This file is (c) under GNU PUBLIC LICENSE
 */

#include <linux/module.h>
#include "dchannel.h"
#include "isac.h"
#include "arcofi.h"
#include "layer1.h"
#include "helper.h"
#include "debug.h"
#ifdef CONFIG_KMOD
#include <linux/kmod.h>
#endif


#define DBUSY_TIMER_VALUE 80
#define ARCOFI_USE 1

const char *isac_revision = "$Revision$";

#ifdef MODULE
MODULE_AUTHOR("Karsten Keil");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
EXPORT_SYMBOL(mISDN_isac_init);
EXPORT_SYMBOL(mISDN_isac_free);
EXPORT_SYMBOL(mISDN_isac_interrupt);
EXPORT_SYMBOL(mISDN_clear_isac);
EXPORT_SYMBOL(mISDN_ISAC_l1hw);
#endif

static inline void
ph_command(dchannel_t *dch, unsigned int command)
{
	if (dch->debug & L1_DEB_ISAC)
		mISDN_debugprint(&dch->inst, "ph_command %x", command);
	if (dch->type & ISAC_TYPE_ISACSX)
		dch->write_reg(dch->inst.privat, ISACSX_CIX0, (command << 4) | 0xE);
	else
		dch->write_reg(dch->inst.privat, ISAC_CIX0, (command << 2) | 3);
}

static void
isac_ph_state_change(dchannel_t *dch)
{
	u_int		prim = PH_SIGNAL | INDICATION;
	u_int		para = 0;

	switch (dch->ph_state) {
		case (ISAC_IND_RS):
		case (ISAC_IND_EI):
			ph_command(dch, ISAC_CMD_DUI);
			prim = PH_CONTROL | INDICATION;
			para = HW_RESET;
			break;
		case (ISAC_IND_DID):
			prim = PH_CONTROL | CONFIRM;
			para = HW_DEACTIVATE;
			break;
		case (ISAC_IND_DR):
			prim = PH_CONTROL | INDICATION;
			para = HW_DEACTIVATE;
			break;
		case (ISAC_IND_PU):
			prim = PH_CONTROL | INDICATION;
			para = HW_POWERUP;
			break;
		case (ISAC_IND_RSY):
			para = ANYSIGNAL;
			break;
		case (ISAC_IND_ARD):
			para = INFO2;
			break;
		case (ISAC_IND_AI8):
			para = INFO4_P8;
			break;
		case (ISAC_IND_AI10):
			para = INFO4_P10;
			break;
		default:
			return;
	}
	mISDN_queue_data(&dch->inst, FLG_MSG_UP, prim, para, 0, NULL, 0);
}

#ifdef OBSOLATE
static void
isac_hwbh(dchannel_t *dch)
{
	if (dch->debug)
		printk(KERN_DEBUG "%s: event %lx\n", __FUNCTION__, dch->event);
#if 0
	if (test_and_clear_bit(D_CLEARBUSY, &dch->event)) {
		if (dch->debug)
			mISDN_debugprint(&dch->inst, "D-Channel Busy cleared");
		stptr = dch->stlist;
		while (stptr != NULL) {
			stptr->l1.l1l2(stptr, PH_PAUSE | CONFIRM, NULL);
			stptr = stptr->next;
		}
	}
#endif
	if (test_and_clear_bit(D_L1STATECHANGE, &dch->event))
		isac_new_ph(dch);		
#if ARCOFI_USE
	if (!(ISAC_TYPE_ARCOFI & dch->type))
		return;
	if (test_and_clear_bit(D_RX_MON1, &dch->event))
		arcofi_fsm(dch, ARCOFI_RX_END, NULL);
	if (test_and_clear_bit(D_TX_MON1, &dch->event))
		arcofi_fsm(dch, ARCOFI_TX_END, NULL);
#endif
}
#endif

void
isac_empty_fifo(dchannel_t *dch, int count)
{
	u_char *ptr;

	if ((dch->debug & L1_DEB_ISAC) && !(dch->debug & L1_DEB_ISAC_FIFO))
		mISDN_debugprint(&dch->inst, "isac_empty_fifo");

	if (!dch->rx_skb) {
		if (!(dch->rx_skb = alloc_stack_skb(MAX_DFRAME_LEN_L1, dch->up_headerlen))) {
			printk(KERN_WARNING "mISDN: D receive out of memory\n");
			dch->write_reg(dch->inst.privat, ISAC_CMDR, 0x80);
			return;
		}
	}
	if ((dch->rx_skb->len + count) >= MAX_DFRAME_LEN_L1) {
		if (dch->debug & L1_DEB_WARN)
			mISDN_debugprint(&dch->inst, "isac_empty_fifo overrun %d",
				dch->rx_skb->len + count);
		dch->write_reg(dch->inst.privat, ISAC_CMDR, 0x80);
		return;
	}
	ptr = skb_put(dch->rx_skb, count);
	dch->read_fifo(dch->inst.privat, ptr, count);
	dch->write_reg(dch->inst.privat, ISAC_CMDR, 0x80);
	if (dch->debug & L1_DEB_ISAC_FIFO) {
		char *t = dch->dlog;

		t += sprintf(t, "isac_empty_fifo cnt %d", count);
		mISDN_QuickHex(t, ptr, count);
		mISDN_debugprint(&dch->inst, dch->dlog);
	}
}

static void
isac_fill_fifo(dchannel_t *dch)
{
	int count, more;
	u_char *ptr;

	if ((dch->debug & L1_DEB_ISAC) && !(dch->debug & L1_DEB_ISAC_FIFO))
		mISDN_debugprint(&dch->inst, "isac_fill_fifo");

	count = dch->tx_len - dch->tx_idx;
	if (count <= 0)
		return;

	more = 0;
	if (count > 32) {
		more = !0;
		count = 32;
	}
	ptr = dch->tx_buf + dch->tx_idx;
	dch->tx_idx += count;
	dch->write_fifo(dch->inst.privat, ptr, count);
	dch->write_reg(dch->inst.privat, ISAC_CMDR, more ? 0x8 : 0xa);
	if (test_and_set_bit(FLG_DBUSY_TIMER, &dch->DFlags)) {
		mISDN_debugprint(&dch->inst, "isac_fill_fifo dbusytimer running");
		del_timer(&dch->dbusytimer);
	}
	init_timer(&dch->dbusytimer);
	dch->dbusytimer.expires = jiffies + ((DBUSY_TIMER_VALUE * HZ)/1000);
	add_timer(&dch->dbusytimer);
	if (dch->debug & L1_DEB_ISAC_FIFO) {
		char *t = dch->dlog;

		t += sprintf(t, "isac_fill_fifo cnt %d", count);
		mISDN_QuickHex(t, ptr, count);
		mISDN_debugprint(&dch->inst, dch->dlog);
	}
}

static void
isac_rme_irq(dchannel_t *dch)
{
	u_char	val;
	u_int	count;

	val = dch->read_reg(dch->inst.privat, ISAC_RSTA);
	if ((val & 0x70) != 0x20) {
		if (val & 0x40) {
			if (dch->debug & L1_DEB_WARN)
				mISDN_debugprint(&dch->inst, "ISAC RDO");
#ifdef ERROR_STATISTIC
			dch->err_rx++;
#endif
		}
		if (!(val & 0x20)) {
			if (dch->debug & L1_DEB_WARN)
				mISDN_debugprint(&dch->inst, "ISAC CRC error");
#ifdef ERROR_STATISTIC
			dch->err_crc++;
#endif
		}
		dch->write_reg(dch->inst.privat, ISAC_CMDR, 0x80);
		if (dch->rx_skb)
			dev_kfree_skb(dch->rx_skb);
	} else {
		count = dch->read_reg(dch->inst.privat, ISAC_RBCL) & 0x1f;
		if (count == 0)
			count = 32;
		isac_empty_fifo(dch, count);
		if (dch->rx_skb)
			if (unlikely(mISDN_queueup_newhead(&dch->inst, 0, PH_DATA_IND, MISDN_ID_ANY, dch->rx_skb))) {
				int_error();
				dev_kfree_skb(dch->rx_skb);
			}
	}
	dch->rx_skb = NULL;
}

static void
isac_xpr_irq(dchannel_t *dch)
{
	if (test_and_clear_bit(FLG_DBUSY_TIMER, &dch->DFlags))
		del_timer(&dch->dbusytimer);
#if 0
	if (test_and_clear_bit(FLG_L1_DBUSY, &dch->DFlags))
		dchannel_sched_event(dch, D_CLEARBUSY);
#endif
	if (dch->tx_idx < dch->tx_len) {
		isac_fill_fifo(dch);
	} else {
		if (test_and_clear_bit(FLG_TX_NEXT, &dch->DFlags)) {
			struct sk_buff	*skb;
			mISDN_head_t	*hh;
			if ((skb = dch->next_skb)) {
				dch->next_skb = NULL;
				dch->tx_len = skb->len;
				memcpy(dch->tx_buf, skb->data, dch->tx_len);
				dch->tx_idx = 0;
				isac_fill_fifo(dch);
				hh = mISDN_HEAD_P(skb);
				skb_trim(skb, 0);
				if (unlikely(mISDN_queueup_newhead(&dch->inst, 0, PH_DATA_CNF, hh->dinfo, skb))) {
					int_error();
					dev_kfree_skb(skb);
				}
			} else {
				printk(KERN_WARNING "isac tx irq TX_NEXT without skb\n");
				test_and_clear_bit(FLG_TX_BUSY, &dch->DFlags);
			}
		} else
			test_and_clear_bit(FLG_TX_BUSY, &dch->DFlags);
	}
}

static void
isac_retransmit(dchannel_t *dch)
{
	if (test_and_clear_bit(FLG_DBUSY_TIMER, &dch->DFlags))
		del_timer(&dch->dbusytimer);
#if 0
	if (test_and_clear_bit(FLG_L1_DBUSY, &dch->DFlags))
		dchannel_sched_event(dch, D_CLEARBUSY);
#endif
	if (test_bit(FLG_TX_BUSY, &dch->DFlags)) {
		/* Restart frame */
		dch->tx_idx = 0;
		isac_fill_fifo(dch);
	} else {
		printk(KERN_WARNING "mISDN: ISAC XDU no TX_BUSY\n");
		mISDN_debugprint(&dch->inst, "ISAC XDU no TX_BUSY");
		if (test_and_clear_bit(FLG_TX_NEXT, &dch->DFlags)) {
			struct sk_buff	*skb;
			mISDN_head_t	*hh;
			if ((skb = dch->next_skb)) {
				dch->next_skb = NULL;
				dch->tx_len = skb->len;
				memcpy(dch->tx_buf, skb->data, dch->tx_len);
				dch->tx_idx = 0;
				isac_fill_fifo(dch);
				hh = mISDN_HEAD_P(skb);
				skb_trim(skb, 0);
				if (unlikely(mISDN_queueup_newhead(&dch->inst, 0, PH_DATA_CNF, hh->dinfo, skb))) {
					int_error();
					dev_kfree_skb(skb);
				}
			} else {
				printk(KERN_WARNING "isac xdu irq TX_NEXT without skb\n");
			}
		}
	}
}

static void
isac_mos_irq(dchannel_t *dch)
{
	u_char		val;
	isac_chip_t	*isac = dch->hw;

	val = dch->read_reg(dch->inst.privat, ISAC_MOSR);
	if (dch->debug & L1_DEB_MONITOR)
		mISDN_debugprint(&dch->inst, "ISAC MOSR %02x", val);
#if ARCOFI_USE
	if (val & 0x08) {
		if (!isac->mon_rx) {
			if (!(isac->mon_rx = kmalloc(MAX_MON_FRAME, GFP_ATOMIC))) {
				if (dch->debug & L1_DEB_WARN)
					mISDN_debugprint(&dch->inst, "ISAC MON RX out of memory!");
				isac->mocr &= 0xf0;
				isac->mocr |= 0x0a;
				dch->write_reg(dch->inst.privat, ISAC_MOCR, isac->mocr);
				goto afterMONR0;
			} else
				isac->mon_rxp = 0;
		}
		if (isac->mon_rxp >= MAX_MON_FRAME) {
			isac->mocr &= 0xf0;
			isac->mocr |= 0x0a;
			dch->write_reg(dch->inst.privat, ISAC_MOCR, isac->mocr);
			isac->mon_rxp = 0;
			if (dch->debug & L1_DEB_WARN)
				mISDN_debugprint(&dch->inst, "ISAC MON RX overflow!");
			goto afterMONR0;
		}
		isac->mon_rx[isac->mon_rxp++] = dch->read_reg(dch->inst.privat, ISAC_MOR0);
		if (dch->debug & L1_DEB_MONITOR)
			mISDN_debugprint(&dch->inst, "ISAC MOR0 %02x", isac->mon_rx[isac->mon_rxp -1]);
		if (isac->mon_rxp == 1) {
			isac->mocr |= 0x04;
			dch->write_reg(dch->inst.privat, ISAC_MOCR, isac->mocr);
		}
	}
afterMONR0:
	if (val & 0x80) {
		if (!isac->mon_rx) {
			if (!(isac->mon_rx = kmalloc(MAX_MON_FRAME, GFP_ATOMIC))) {
				if (dch->debug & L1_DEB_WARN)
					mISDN_debugprint(&dch->inst, "ISAC MON RX out of memory!");
				isac->mocr &= 0x0f;
				isac->mocr |= 0xa0;
				dch->write_reg(dch->inst.privat, ISAC_MOCR, isac->mocr);
				goto afterMONR1;
			} else
				isac->mon_rxp = 0;
		}
		if (isac->mon_rxp >= MAX_MON_FRAME) {
			isac->mocr &= 0x0f;
			isac->mocr |= 0xa0;
			dch->write_reg(dch->inst.privat, ISAC_MOCR, isac->mocr);
			isac->mon_rxp = 0;
			if (dch->debug & L1_DEB_WARN)
				mISDN_debugprint(&dch->inst, "ISAC MON RX overflow!");
			goto afterMONR1;
		}
		isac->mon_rx[isac->mon_rxp++] = dch->read_reg(dch->inst.privat, ISAC_MOR1);
		if (dch->debug & L1_DEB_MONITOR)
			mISDN_debugprint(&dch->inst, "ISAC MOR1 %02x", isac->mon_rx[isac->mon_rxp -1]);
		isac->mocr |= 0x40;
		dch->write_reg(dch->inst.privat, ISAC_MOCR, isac->mocr);
	}
afterMONR1:
	if (val & 0x04) {
		isac->mocr &= 0xf0;
		dch->write_reg(dch->inst.privat, ISAC_MOCR, isac->mocr);
		isac->mocr |= 0x0a;
		dch->write_reg(dch->inst.privat, ISAC_MOCR, isac->mocr);
		mISDN_queue_data(&dch->inst, 0, PH_SIGNAL | INDICATION, D_RX_MON0,
			isac->mon_rxp, isac->mon_rx, 0);
	}
	if (val & 0x40) {
		isac->mocr &= 0x0f;
		dch->write_reg(dch->inst.privat, ISAC_MOCR, isac->mocr);
		isac->mocr |= 0xa0;
		dch->write_reg(dch->inst.privat, ISAC_MOCR, isac->mocr);
		mISDN_queue_data(&dch->inst, 0, PH_SIGNAL | INDICATION, D_RX_MON1,
			isac->mon_rxp, isac->mon_rx, 0);
	}
	if (val & 0x02) {
		if ((!isac->mon_tx) || (isac->mon_txc && 
			(isac->mon_txp >= isac->mon_txc) && !(val & 0x08))) {
			isac->mocr &= 0xf0;
			dch->write_reg(dch->inst.privat, ISAC_MOCR, isac->mocr);
			isac->mocr |= 0x0a;
			dch->write_reg(dch->inst.privat, ISAC_MOCR, isac->mocr);
			if (isac->mon_txc && (isac->mon_txp >= isac->mon_txc))
				mISDN_queue_data(&dch->inst, 0, PH_SIGNAL | INDICATION,
					D_TX_MON0, 0, NULL, 0);
			goto AfterMOX0;
		}
		if (isac->mon_txc && (isac->mon_txp >= isac->mon_txc)) {
			mISDN_queue_data(&dch->inst, 0, PH_SIGNAL | INDICATION,
				D_TX_MON0, 0, NULL, 0);
			goto AfterMOX0;
		}
		dch->write_reg(dch->inst.privat, ISAC_MOX0,
		isac->mon_tx[isac->mon_txp++]);
		if (dch->debug & L1_DEB_MONITOR)
			mISDN_debugprint(&dch->inst, "ISAC %02x -> MOX0", isac->mon_tx[isac->mon_txp -1]);
	}
AfterMOX0:
	if (val & 0x20) {
		if ((!isac->mon_tx) || (isac->mon_txc && 
			(isac->mon_txp >= isac->mon_txc) && !(val & 0x80))) {
			isac->mocr &= 0x0f;
			dch->write_reg(dch->inst.privat, ISAC_MOCR, isac->mocr);
			isac->mocr |= 0xa0;
			dch->write_reg(dch->inst.privat, ISAC_MOCR, isac->mocr);
			if (isac->mon_txc && (isac->mon_txp >= isac->mon_txc))
				mISDN_queue_data(&dch->inst, 0, PH_SIGNAL | INDICATION,
					D_TX_MON1, 0, NULL, 0);
			goto AfterMOX1;
		}
		if (isac->mon_txc && (isac->mon_txp >= isac->mon_txc)) {
			mISDN_queue_data(&dch->inst, 0, PH_SIGNAL | INDICATION,
				D_TX_MON1, 0, NULL, 0);
			goto AfterMOX1;
		}
		dch->write_reg(dch->inst.privat, ISAC_MOX1,
		isac->mon_tx[isac->mon_txp++]);
		if (dch->debug & L1_DEB_MONITOR)
			mISDN_debugprint(&dch->inst, "ISAC %02x -> MOX1", isac->mon_tx[isac->mon_txp -1]);
	}
AfterMOX1:
	val = 0; /* dummy to avoid warning */
#endif
}

static void
isac_cisq_irq(dchannel_t *dch) {
	unsigned char val;

	val = dch->read_reg(dch->inst.privat, ISAC_CIR0);
	if (dch->debug & L1_DEB_ISAC)
		mISDN_debugprint(&dch->inst, "ISAC CIR0 %02X", val);
	if (val & 2) {
		if (dch->debug & L1_DEB_ISAC)
			mISDN_debugprint(&dch->inst, "ph_state change %x->%x",
				dch->ph_state, (val >> 2) & 0xf);
		dch->ph_state = (val >> 2) & 0xf;
		isac_ph_state_change(dch);
	}
	if (val & 1) {
		val = dch->read_reg(dch->inst.privat, ISAC_CIR1);
		if (dch->debug & L1_DEB_ISAC)
			mISDN_debugprint(&dch->inst, "ISAC CIR1 %02X", val );
	}
}

static void
isacsx_cic_irq(dchannel_t *dch)
{
	unsigned char val;

	val = dch->read_reg(dch->inst.privat, ISACSX_CIR0);
	if (dch->debug & L1_DEB_ISAC)
		mISDN_debugprint(&dch->inst, "ISACSX CIR0 %02X", val);
	if (val & ISACSX_CIR0_CIC0) {
		if (dch->debug & L1_DEB_ISAC)
			mISDN_debugprint(&dch->inst, "ph_state change %x->%x",
				dch->ph_state, val >> 4);
		dch->ph_state = val >> 4;
		isac_ph_state_change(dch);
	}
}

static void
isacsx_rme_irq(dchannel_t *dch)
{
	int count;
	unsigned char val;

	val = dch->read_reg(dch->inst.privat, ISACSX_RSTAD);
	if ((val & (ISACSX_RSTAD_VFR | 
		    ISACSX_RSTAD_RDO | 
		    ISACSX_RSTAD_CRC | 
		    ISACSX_RSTAD_RAB)) 
	    != (ISACSX_RSTAD_VFR | ISACSX_RSTAD_CRC)) {
	    	if (dch->debug & L1_DEB_WARN)
	    		mISDN_debugprint(&dch->inst, "RSTAD %#x, dropped", val);
#ifdef ERROR_STATISTIC
		if (val & ISACSX_RSTAD_CRC)
			dch->err_rx++;
		else
			dch->err_crc++;
#endif
	    	dch->write_reg(dch->inst.privat, ISACSX_CMDRD, ISACSX_CMDRD_RMC);
		if (dch->rx_skb)
			dev_kfree_skb(dch->rx_skb);
	} else {
		count = dch->read_reg(dch->inst.privat, ISACSX_RBCLD) & 0x1f;
		if (count == 0)
			count = 32;
		isac_empty_fifo(dch, count);
		if (dch->rx_skb) {
			skb_trim(dch->rx_skb, dch->rx_skb->len - 1);
			if (unlikely(mISDN_queueup_newhead(&dch->inst, 0, PH_DATA_IND, MISDN_ID_ANY, dch->rx_skb))) {
				int_error();
				dev_kfree_skb(dch->rx_skb);
			}
		}
	}
	dch->rx_skb = NULL;
}

void
mISDN_isac_interrupt(dchannel_t *dch, u_char val)
{
	if (dch->debug & L1_DEB_ISAC)
		mISDN_debugprint(&dch->inst, "ISAC interrupt %02x", val);
	if (dch->type & ISAC_TYPE_ISACSX) {
		if (val & ISACSX_ISTA_CIC)
			isacsx_cic_irq(dch);
		if (val & ISACSX_ISTA_ICD) {
			val = dch->read_reg(dch->inst.privat, ISACSX_ISTAD);
			if (dch->debug & L1_DEB_ISAC)
				mISDN_debugprint(&dch->inst, "ISTAD %02x", val);
			if (val & ISACSX_ISTAD_XDU) {
				if (dch->debug & L1_DEB_WARN)
					mISDN_debugprint(&dch->inst, "ISAC XDU");
#ifdef ERROR_STATISTIC
				dch->err_tx++;
#endif
				isac_retransmit(dch);
			}
			if (val & ISACSX_ISTAD_XMR) {
				if (dch->debug & L1_DEB_WARN)
					mISDN_debugprint(&dch->inst, "ISAC XMR");
#ifdef ERROR_STATISTIC
				dch->err_tx++;
#endif
				isac_retransmit(dch);
			}
			if (val & ISACSX_ISTAD_XPR)
				isac_xpr_irq(dch);
			if (val & ISACSX_ISTAD_RFO) {
				if (dch->debug & L1_DEB_WARN)
					mISDN_debugprint(&dch->inst, "ISAC RFO");
				dch->write_reg(dch->inst.privat, ISACSX_CMDRD, ISACSX_CMDRD_RMC);
			}
			if (val & ISACSX_ISTAD_RME)
				isacsx_rme_irq(dch);
			if (val & ISACSX_ISTAD_RPF)
				isac_empty_fifo(dch, 0x20);
		}
	} else {
		if (val & 0x80)	/* RME */
			isac_rme_irq(dch);
		if (val & 0x40)	/* RPF */
			isac_empty_fifo(dch, 32);
		if (val & 0x10)	/* XPR */
			isac_xpr_irq(dch);
		if (val & 0x04)	/* CISQ */
			isac_cisq_irq(dch);
		if (val & 0x20)	/* RSC - never */
			if (dch->debug & L1_DEB_WARN)
				mISDN_debugprint(&dch->inst, "ISAC RSC interrupt");
		if (val & 0x02)	/* SIN - never */
			if (dch->debug & L1_DEB_WARN)
				mISDN_debugprint(&dch->inst, "ISAC SIN interrupt");
		if (val & 0x01) {	/* EXI */
			val = dch->read_reg(dch->inst.privat, ISAC_EXIR);
			if (dch->debug & L1_DEB_WARN)
				mISDN_debugprint(&dch->inst, "ISAC EXIR %02x", val);
			if (val & 0x80)	/* XMR */
				mISDN_debugprint(&dch->inst, "ISAC XMR");
			if (val & 0x40) { /* XDU */
				if (dch->debug & L1_DEB_WARN)
					mISDN_debugprint(&dch->inst, "ISAC XDU");
#ifdef ERROR_STATISTIC
				dch->err_tx++;
#endif
				isac_retransmit(dch);
			}
			if (val & 0x04)	/* MOS */
				isac_mos_irq(dch);
		}
	}
}

int
mISDN_ISAC_l1hw(mISDNinstance_t *inst, struct sk_buff *skb)
{
	dchannel_t	*dch;
	int		ret = -EINVAL;
	mISDN_head_t	*hh;

	hh = mISDN_HEAD_P(skb);
	dch = container_of(inst, dchannel_t, inst);
	ret = 0;
	if (hh->prim == PH_DATA_REQ) {
		if (dch->next_skb) {
			mISDN_debugprint(&dch->inst, " l2l1 next_skb exist this shouldn't happen");
			return(-EBUSY);
		}
		dch->inst.lock(dch->inst.privat,0);
		if (test_and_set_bit(FLG_TX_BUSY, &dch->DFlags)) {
			test_and_set_bit(FLG_TX_NEXT, &dch->DFlags);
			dch->next_skb = skb;
			dch->inst.unlock(dch->inst.privat);
			return(0);
		} else {
			dch->tx_len = skb->len;
			memcpy(dch->tx_buf, skb->data, dch->tx_len);
			dch->tx_idx = 0;
			isac_fill_fifo(dch);
			dch->inst.unlock(dch->inst.privat);
			skb_trim(skb, 0);
			return(mISDN_queueup_newhead(inst, 0, PH_DATA_CNF,
				hh->dinfo, skb));
		}
	} else if (hh->prim == (PH_SIGNAL | REQUEST)) {
		dch->inst.lock(dch->inst.privat,0);
		if (hh->dinfo == INFO3_P8)
			ph_command(dch, ISAC_CMD_AR8);
		else if (hh->dinfo == INFO3_P10)
			ph_command(dch, ISAC_CMD_AR10);
		else
			ret = -EINVAL;
		dch->inst.unlock(dch->inst.privat);
	} else if (hh->prim == (PH_CONTROL | REQUEST)) {
		dch->inst.lock(dch->inst.privat,0);
		if (hh->dinfo == HW_RESET) {
			if ((dch->ph_state == ISAC_IND_EI) ||
				(dch->ph_state == ISAC_IND_DR) ||
				(dch->ph_state == ISAC_IND_RS))
			        ph_command(dch, ISAC_CMD_TIM);
			else
				ph_command(dch, ISAC_CMD_RS);
		} else if (hh->dinfo == HW_POWERUP) {
			ph_command(dch, ISAC_CMD_TIM);
		} else if (hh->dinfo == HW_DEACTIVATE) {
			if (dch->next_skb) {
				dev_kfree_skb(dch->next_skb);
				dch->next_skb = NULL;
			}
			test_and_clear_bit(FLG_TX_NEXT, &dch->DFlags);
			test_and_clear_bit(FLG_TX_BUSY, &dch->DFlags);
			if (test_and_clear_bit(FLG_DBUSY_TIMER, &dch->DFlags))
				del_timer(&dch->dbusytimer);
#if 0
			if (test_and_clear_bit(FLG_L1_DBUSY, &dch->DFlags))
				dchannel_sched_event(dch, D_CLEARBUSY);
#endif
		} else if ((hh->dinfo & HW_TESTLOOP) == HW_TESTLOOP) {
			u_char		tl;
			if (dch->type & ISAC_TYPE_ISACSX) {
			/* TODO */
			} else {
				tl = 0;
				if (1 & hh->dinfo)
					tl |= 0x0c;
				if (2 & hh->dinfo)
					tl |= 0x3;
				if (ISAC_TYPE_IOM1 & dch->type) {
					/* IOM 1 Mode */
					if (!tl) {
						dch->write_reg(dch->inst.privat, ISAC_SPCR, 0xa);
						dch->write_reg(dch->inst.privat, ISAC_ADF1, 0x2);
					} else {
						dch->write_reg(dch->inst.privat, ISAC_SPCR, tl);
						dch->write_reg(dch->inst.privat, ISAC_ADF1, 0xa);
					}
				} else {
					/* IOM 2 Mode */
					dch->write_reg(dch->inst.privat, ISAC_SPCR, tl);
					if (tl)
						dch->write_reg(dch->inst.privat, ISAC_ADF1, 0x8);
					else
						dch->write_reg(dch->inst.privat, ISAC_ADF1, 0x0);
				}
			}
		} else {
			if (dch->debug & L1_DEB_WARN)
				mISDN_debugprint(&dch->inst, "isac_l1hw unknown ctrl %x",
					hh->dinfo);
			ret = -EINVAL;
		}
		dch->inst.unlock(dch->inst.privat);
	} else if (hh->prim == (PH_SIGNAL | INDICATION)) {
#if ARCOFI_USE
		if ((ISAC_TYPE_ARCOFI & dch->type)) {
			if (hh->dinfo == D_RX_MON1) {
				arcofi_fsm(dch, ARCOFI_RX_END, skb);
			} else if (hh->dinfo == D_TX_MON1) {
				arcofi_fsm(dch, ARCOFI_TX_END, NULL);
			}
		}
#endif
		ret = 0;
	} else {
		if (dch->debug & L1_DEB_WARN)
			mISDN_debugprint(&dch->inst, "isac_l1hw unknown prim %x",
				hh->prim);
		ret = -EINVAL;
	}
	if (!ret)
		dev_kfree_skb(skb);
	return(ret);
}

void 
mISDN_isac_free(dchannel_t *dch) {
	isac_chip_t     *isac = dch->hw;

	if (dch->dbusytimer.function != NULL) {
		del_timer(&dch->dbusytimer);
		dch->dbusytimer.function = NULL;
	}
	if (!isac)
		return;
	if (isac->mon_rx) {
		kfree(isac->mon_rx);
		isac->mon_rx = NULL;
	}
	if (isac->mon_tx) {
		kfree(isac->mon_tx);
		isac->mon_tx = NULL;
	}
}

static void
dbusy_timer_handler(dchannel_t *dch)
{
	int	rbch, star;

	if (test_bit(FLG_DBUSY_TIMER, &dch->DFlags)) {
		if (dch->inst.lock(dch->inst.privat, 1)) {
			dch->dbusytimer.expires = jiffies + 1;
			add_timer(&dch->dbusytimer);
			return;
		}
		rbch = dch->read_reg(dch->inst.privat, ISAC_RBCH);
		star = dch->read_reg(dch->inst.privat, ISAC_STAR);
		if (dch->debug) 
			mISDN_debugprint(&dch->inst, "D-Channel Busy RBCH %02x STAR %02x",
				rbch, star);
		if (rbch & ISAC_RBCH_XAC) { /* D-Channel Busy */
			test_and_set_bit(FLG_L1_DBUSY, &dch->DFlags);
#if 0
			stptr = dch->stlist;
			while (stptr != NULL) {
				stptr->l1.l1l2(stptr, PH_PAUSE | INDICATION, NULL);
				stptr = stptr->next;
			}
#endif
		} else {
			/* discard frame; reset transceiver */
			test_and_clear_bit(FLG_DBUSY_TIMER, &dch->DFlags);
			if (dch->tx_idx) {
				dch->tx_idx = 0;
			} else {
				printk(KERN_WARNING "mISDN: ISAC D-Channel Busy no tx_idx\n");
				mISDN_debugprint(&dch->inst, "D-Channel Busy no tx_idx");
			}
			/* Transmitter reset */
			dch->write_reg(dch->inst.privat, ISAC_CMDR, 0x01);
		}
		dch->inst.unlock(dch->inst.privat);
	}
}

static char *ISACVer[] =
{"2086/2186 V1.1", "2085 B1", "2085 B2",
 "2085 V2.3"};

int
mISDN_isac_init(dchannel_t *dch)
{
	isac_chip_t	*isac = dch->hw;
	u_char		val;


  	if (!isac)
  		return(-EINVAL);
	dch->hw_bh = NULL;
	isac->mon_tx = NULL;
	isac->mon_rx = NULL;
	dch->dbusytimer.function = (void *) dbusy_timer_handler;
	dch->dbusytimer.data = (long) dch;
	init_timer(&dch->dbusytimer);
  	isac->mocr = 0xaa;
	if (dch->type & ISAC_TYPE_ISACSX) {
		// clear LDD
		dch->write_reg(dch->inst.privat, ISACSX_TR_CONF0, 0x00);
		// enable transmitter
		dch->write_reg(dch->inst.privat, ISACSX_TR_CONF2, 0x00);
		// transparent mode 0, RAC, stop/go
		dch->write_reg(dch->inst.privat, ISACSX_MODED, 0xc9);
		// all HDLC IRQ unmasked
		dch->write_reg(dch->inst.privat, ISACSX_MASKD, 0x03);
		// unmask ICD, CID IRQs
		dch->write_reg(dch->inst.privat, ISACSX_MASK, ~(ISACSX_ISTA_ICD | ISACSX_ISTA_CIC));
		printk(KERN_INFO "mISDN_isac_init: ISACSX\n");
		isac_ph_state_change(dch);		
		ph_command(dch, ISAC_CMD_RS);
	} else { /* old isac */
	  	dch->write_reg(dch->inst.privat, ISAC_MASK, 0xff);
		val = dch->read_reg(dch->inst.privat, ISAC_RBCH);
		printk(KERN_INFO "mISDN_isac_init: ISAC version (%x): %s\n", val, ISACVer[(val >> 5) & 3]);
		dch->type |= ((val >> 5) & 3);
		if (ISAC_TYPE_IOM1 & dch->type) {
			/* IOM 1 Mode */
			dch->write_reg(dch->inst.privat, ISAC_ADF2, 0x0);
			dch->write_reg(dch->inst.privat, ISAC_SPCR, 0xa);
			dch->write_reg(dch->inst.privat, ISAC_ADF1, 0x2);
			dch->write_reg(dch->inst.privat, ISAC_STCR, 0x70);
			dch->write_reg(dch->inst.privat, ISAC_MODE, 0xc9);
		} else {
			/* IOM 2 Mode */
			if (!isac->adf2)
				isac->adf2 = 0x80;
			dch->write_reg(dch->inst.privat, ISAC_ADF2, isac->adf2);
			dch->write_reg(dch->inst.privat, ISAC_SQXR, 0x2f);
			dch->write_reg(dch->inst.privat, ISAC_SPCR, 0x00);
			dch->write_reg(dch->inst.privat, ISAC_STCR, 0x70);
			dch->write_reg(dch->inst.privat, ISAC_MODE, 0xc9);
			dch->write_reg(dch->inst.privat, ISAC_TIMR, 0x00);
			dch->write_reg(dch->inst.privat, ISAC_ADF1, 0x00);
		}
		isac_ph_state_change(dch);
		ph_command(dch, ISAC_CMD_RS);
		dch->write_reg(dch->inst.privat, ISAC_MASK, 0x0);
	}
	return 0;
}

void
mISDN_clear_isac(dchannel_t *dch)
{
	isac_chip_t	*isac = dch->hw;
	u_int		val, eval;

	if (!isac)
		return;
	/* Disable all IRQ */
	dch->write_reg(dch->inst.privat, ISAC_MASK, 0xFF);
	val = dch->read_reg(dch->inst.privat, ISAC_STAR);
	mISDN_debugprint(&dch->inst, "ISAC STAR %x", val);
	val = dch->read_reg(dch->inst.privat, ISAC_MODE);
	mISDN_debugprint(&dch->inst, "ISAC MODE %x", val);
	val = dch->read_reg(dch->inst.privat, ISAC_ADF2);
	mISDN_debugprint(&dch->inst, "ISAC ADF2 %x", val);
	val = dch->read_reg(dch->inst.privat, ISAC_ISTA);
	mISDN_debugprint(&dch->inst, "ISAC ISTA %x", val);
	if (val & 0x01) {
		eval = dch->read_reg(dch->inst.privat, ISAC_EXIR);
		mISDN_debugprint(&dch->inst, "ISAC EXIR %x", eval);
	}
	val = dch->read_reg(dch->inst.privat, ISAC_CIR0);
	mISDN_debugprint(&dch->inst, "ISAC CIR0 %x", val);
	dch->ph_state = (val >> 2) & 0xf;
}

#ifdef MODULE
static int isac_mod_init(void)
{
	printk(KERN_INFO "ISAC module %s\n", isac_revision);
	return(0);
}

static void isac_mod_cleanup(void)
{
	printk(KERN_INFO "ISAC module unloaded\n");
}
module_init(isac_mod_init);
module_exit(isac_mod_cleanup);
#endif
