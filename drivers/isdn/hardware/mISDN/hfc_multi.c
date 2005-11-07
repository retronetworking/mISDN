/*

 * hfc_multi.c  low level driver for hfc-4s/hfc-8s/hfc-e1 based cards
 *
 * Author	Andreas Eversberg (jolly@jolly.de)
 * ported to mqueue mechanism:
 * 		Peter Sprenger (sprengermoving-bytes.de)
 *
 * inspired by existing hfc-pci driver:
 * Copyright 1999  by Werner Cornelius (werner@isdn-development.de)
 * Copyright 2001  by Karsten Keil (keil@isdn4linux.de)
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
 *
 * Thanx to Cologne Chip AG for this great controller!
 */

/* module parameters:
 * type:
	Value 1	= HFC-E1 (1 port) 0x01
	Value 4	= HFC-4S (4 ports) 0x04
	Value 8	= HFC-8S (8 ports) 0x08
	Bit 8	= uLaw (instead of aLaw)
	Bit 9	= Enable DTMF detection on all B-channels
	Bit 10	= spare
	Bit 11	= Set PCM bus into slave mode.
	Bit 12	= Ignore missing frame clock on PCM bus.
	Bit 13	= Use direct RX clock for PCM sync rather than PLL. (E1 only)
	Bit 14	= Use external ram (128K)
	Bit 15	= Use external ram (512K)
	Bit 16	= Use 64 timeslots instead of 32
	Bit 17	= Use 128 timeslots instead of anything else

 * protocol:
	NOTE: Must be given for all ports, not for the number of cards.
	HFC-4S/HFC-8S/HFC-E1 bits:
 	Bit 0-3 = protocol
	Bit 4	= NT-Mode
	Bit 5	= PTP (instead of multipoint)

	HFC-4S/HFC-8S only bits:
	Bit 16	= Use master clock for this S/T interface (ony once per chip).
	Bit 17	= transmitter line setup (non capacitive mode) DONT CARE!
	Bit 18	= Disable E-channel. (No E-channel processing)
	Bit 19	= Register E-channel as D-stack (TE-mode only)

	HFC-E1 only bits:
	Bit 16	= interface: 0=copper, 1=optical
	Bit 17	= reserved (later for 32 B-channels transparent mode)
	Bit 18	= Report LOS
	Bit 19	= Report AIS
	Bit 20	= Report SLIP
	Bit 21-22 = elastic jitter buffer (1-3), Use 0 for default.
(all other bits are reserved and shall be 0)

 * layermask:
	NOTE: Must be given for all ports, not for the number of cards.
	mask of layers to be used for D-channel stack

 * debug:
	NOTE: only one debug value must be given for all cards
	enable debugging (see hfc_multi.h for debug options)

 * poll:
	NOTE: only one poll value must be given for all cards
	Give the number of samples for each fifo process.
	By default 128 is used. Decrease to reduce delay, increase to
	reduce cpu load. If unsure, don't mess with it!
	Valid is 8, 16, 32, 64, 128, 256.

 * pcm:
	NOTE: only one pcm value must be given for all cards
	Give the id of the PCM bus. All PCM busses with the same ID
	are expected to be connected and have equal slots.
	Only one chip of the PCM bus must be master, the others slave.
	-1 means no support of PCM bus.
 */

/* debug using register map (never use this, it will flood your system log) */
//#define HFC_REGISTER_MAP

#include <linux/config.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include "dchannel.h"
#include "bchannel.h"
#include "layer1.h"
#include "dsp.h"
#include "debug.h"
#include <linux/isdn_compat.h>

#warning

//#define IRQCOUNT_DEBUG
#include "hfc_multi.h"

#warning
#define bugtest {}
#if 0
#define bugtest \
	if (hc->irq) free_irq(hc->irq, hc); \
	hc->irq = 0; \
	if (request_irq(hc->pci_dev->irq, hfcmulti_interrupt, SA_SHIRQ, "HFC-multi", hc)) { \
		printk(KERN_WARNING "mISDN: Could not get interrupt %d.\n", hc->pci_dev->irq); \
	hc->irq = hc->pci_dev->irq; }
#endif
		
static void ph_state_change(dchannel_t *dch);

extern const char *CardType[];

static const char *hfcmulti_revision = "$Revision$";

static int HFC_cnt, HFC_idx;

static mISDNobject_t	HFCM_obj;

static char HFCName[] = "HFC_multi";

/* table entry in the PCI devices list */
typedef struct {
	int vendor_id;
	int vendor_sub;
	int device_id;
	int device_sub;
	char *vendor_name;
	char *card_name;
	int type;
	int clock2;
	int leds;
} PCI_ENTRY;

static int poll_timer = 6;	/* default = 128 samples = 16ms */
/* number of POLL_TIMER interrupts for G2 timeout (min 120ms) */
static int nt_t1_count[] = { 480, 240, 120, 60, 30, 15, 8, 4 };
#define CLKDEL_TE	0x0f	/* CLKDEL in TE mode */
#define CLKDEL_NT	0x0c	/* CLKDEL in NT mode (0x60 MUST not be included!) */
static u_char silence =	0xff;	/* silence by LAW */

/* enable 32 bit fifo access (PC usage) */
//#define FIFO_32BIT_ACCESS

#define VENDOR_CCD "Cologne Chip AG"
#define CCAG_VID 0x1397      // Cologne Chip Vendor ID
#define HFC4S_ID 0x08B4
#define HFC8S_ID 0x16B8
#define HFCE1_ID 0x30B1

static const PCI_ENTRY id_list[] =
{
	{CCAG_VID, 0xffffffff, HFC4S_ID, 0xffffffff, VENDOR_CCD,
	 "HFC-4S CCAG Eval", 4, 1, 2},
	{CCAG_VID, 0xffffffff, HFC8S_ID, 0xffffffff, VENDOR_CCD,
	 "HFC-8S CCAG Eval", 8, 1, 0},
	{CCAG_VID, 0xffffffff, HFCE1_ID, 0xffffffff, VENDOR_CCD,
	 "HFC-E1 CCAG Eval", 1, 0, 1}, /* E1 only supports single clock */
	{CCAG_VID, CCAG_VID, HFC4S_ID, 0x08B4, VENDOR_CCD,
	 "HFC-4S CCAG Eval (old)", 4, 0, 0},
	{CCAG_VID, CCAG_VID, HFC8S_ID, 0x16B8, VENDOR_CCD,
	 "HFC-8S CCAG Eval (old)", 8, 0, 0},
	{CCAG_VID, CCAG_VID, HFCE1_ID, 0x30B1, VENDOR_CCD,
	 "HFC-E1 CCAG Eval (old)", 1, 0, 0},
	{CCAG_VID, CCAG_VID, HFC4S_ID, 0xB520, VENDOR_CCD,
	 "HFC-4S IOB4ST", 4, 1, 2},
	{CCAG_VID, CCAG_VID, HFC4S_ID, 0xB620, VENDOR_CCD,
	 "HFC-4S", 4, 1, 2},
	{CCAG_VID, CCAG_VID, HFC4S_ID, 0xB560, VENDOR_CCD,
	 "HFC-4S Beronet Card", 4, 1, 2},
	{CCAG_VID, CCAG_VID, HFC8S_ID, 0xB521, VENDOR_CCD,
	 "HFC-8S IOB4ST Recording", 8, 1, 0},
	{CCAG_VID, CCAG_VID, HFC8S_ID, 0xB522, VENDOR_CCD,
	 "HFC-8S IOB8ST", 8, 1, 0},
	{CCAG_VID, CCAG_VID, HFC8S_ID, 0xB622, VENDOR_CCD,
	 "HFC-8S", 8, 1, 0},
	{CCAG_VID, CCAG_VID, HFC8S_ID, 0xB562, VENDOR_CCD,
	 "HFC-8S Beronet Card", 8, 1, 0},
	{CCAG_VID, CCAG_VID, HFCE1_ID, 0xB523, VENDOR_CCD,
	 "HFC-E1 IOB1E1", 1, 0, 1}, /* E1 only supports single clock */
	{CCAG_VID, CCAG_VID, HFCE1_ID, 0xC523, VENDOR_CCD,
	 "HFC-E1", 1, 0, 1}, /* E1 only supports single clock */
	{CCAG_VID, CCAG_VID, HFCE1_ID, 0xB563, VENDOR_CCD,
	 "HFC-E1 Beronet Card", 1, 0, 1}, /* E1 only supports single clock */
	{0x10B5, CCAG_VID, 0x9030, 0x3136, VENDOR_CCD,
	 "HFC-4S PCIBridgeEval", 4, 0, 0},      // PLX PCI-Bridge
	{0, 0, 0, 0, NULL, NULL, 0, 0, 0},
};


/****************/
/* module stuff */
/****************/

/* NOTE: MAX_PORTS must be 8*MAX_CARDS */
#define MAX_CARDS	16
#define MAX_PORTS	128
#define MODULE_CARDS_T	"1-16i"
#define MODULE_PORTS_T	"1-128i" /* 16 cards can have 128 ports */
static u_int type[MAX_CARDS];
static BYTE allocated[MAX_CARDS];  // remember if card is found
static int pcm[MAX_PORTS];
static u_int protocol[MAX_PORTS];
static int layermask[MAX_PORTS];
static int debug;
static int poll;

#ifdef MODULE
MODULE_AUTHOR("Andreas Eversberg");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
MODULE_PARM(debug, "1i");
MODULE_PARM(poll, "1i");
MODULE_PARM(type, MODULE_CARDS_T);
MODULE_PARM(pcm, MODULE_CARDS_T);
MODULE_PARM(protocol, MODULE_PORTS_T);
MODULE_PARM(layermask, MODULE_PORTS_T);
#endif

/******************************************/
/* free hardware resources used by driver */
/******************************************/

static void
release_io_hfcmulti(hfc_multi_t *hc)
{
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: entered\n", __FUNCTION__);

	/* irq off */
	HFC_outb(hc, R_IRQ_CTRL, 0);

	/* soft reset */
	hc->hw.r_cirm |= V_SRES;
	HFC_outb(hc, R_CIRM, hc->hw.r_cirm);
	udelay(1000);
	hc->hw.r_cirm &= ~V_SRES;
	HFC_outb(hc, R_CIRM, hc->hw.r_cirm);
	udelay(1000); /* instead of 'wait' that may cause locking */

	/* disable memory mapped ports / io ports */
	pci_write_config_word(hc->pci_dev, PCI_COMMAND, 0);
#ifdef CONFIG_HFCMULTI_PCIMEM
	if (hc->pci_membase) iounmap((void *)hc->pci_membase);
	if (hc->plx_membase) iounmap((void *)hc->plx_membase);
#else
	if (hc->pci_iobase)
		release_region(hc->pci_iobase, 8);
#endif

	if (hc->pci_dev) {
		pci_disable_device(hc->pci_dev);
		pci_set_drvdata(hc->pci_dev, NULL);
	}

	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: done\n", __FUNCTION__);
}

/****************************************************************************/
/* function called to reset the HFC chip. A complete software reset of chip */
/* and fifos is done. All configuration of the chip is done.                */
/****************************************************************************/

static int
init_chip(hfc_multi_t *hc)
{
	u_long 	flags, val, val2 = 0, rev;
	int	cnt = 0;
	int	i, err = 0;
	u_char	r_conf_en, rval;

	spin_lock_irqsave(&hc->lock, flags);
	/* reset all registers */
	memset(&hc->hw, 0, sizeof(hfcmulti_hw_t));

	/* revision check */
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: entered\n", __FUNCTION__);
	val = HFC_inb(hc, R_CHIP_ID)>>4;
	if(val!=0x8 && val!=0xc && val!=0xe)
	{
		printk(KERN_INFO "HFC_multi: unknown CHIP_ID:%x\n",(u_int)val);
		err = -EIO;
		goto out;
	}
	rev = HFC_inb(hc, R_CHIP_RV);
	printk(KERN_INFO "HFC_multi: resetting HFC with chip ID=0x%lx revision=%ld%s\n", val, rev, (rev==0)?" (old FIFO handling)":"");
	if (rev == 0) {
		test_and_set_bit(HFC_CHIP_REVISION0, &hc->chip);
		printk(KERN_WARNING "HFC_multi: NOTE: Your chip is revision 0, ask Cologne Chip for update. Newer chips have a better FIFO handling. Old chips still work but may have slightly lower HDLC transmit performance.\n");
	}
	if (rev > 1) {
		printk(KERN_WARNING "HFC_multi: WARNING: This driver doesn't consider chip revision = %ld. The chip / bridge may not work.\n", rev);
	}

	/* set s-ram size */
	hc->Flen = 0x10;
	hc->Zmin = 0x80;
	hc->Zlen = 384;
	hc->DTMFbase = 0x1000;
	if (test_bit(HFC_CHIP_EXRAM_128, &hc->chip)) {
		if (debug & DEBUG_HFCMULTI_INIT)
			printk(KERN_DEBUG "%s: changing to 128K extenal RAM\n", __FUNCTION__);
		hc->hw.r_ctrl |= V_EXT_RAM;
		hc->hw.r_ram_sz = 1;
		hc->Flen = 0x20;
		hc->Zmin = 0xc0;
		hc->Zlen = 1856;
		hc->DTMFbase = 0x2000;
	}
	if (test_bit(HFC_CHIP_EXRAM_512, &hc->chip)) {
		if (debug & DEBUG_HFCMULTI_INIT)
			printk(KERN_DEBUG "%s: changing to 512K extenal RAM\n", __FUNCTION__);
		hc->hw.r_ctrl |= V_EXT_RAM;
		hc->hw.r_ram_sz = 2;
		hc->Flen = 0x20;
		hc->Zmin = 0xc0;
		hc->Zlen = 8000;
		hc->DTMFbase = 0x2000;
	}

	/* we only want the real Z2 read-pointer for revision > 0 */
	if (!test_bit(HFC_CHIP_REVISION0, &hc->chip))
		hc->hw.r_ram_sz |= V_FZ_MD;

	/* soft reset */
	HFC_outb(hc, R_CTRL, hc->hw.r_ctrl);
	HFC_outb(hc, R_RAM_SZ, hc->hw.r_ram_sz);
	HFC_outb(hc, R_FIFO_MD, 0);
	hc->hw.r_cirm = V_SRES | V_HFCRES | V_PCMRES | V_STRES | V_RLD_EPR;
	HFC_outb(hc, R_CIRM, hc->hw.r_cirm);
	udelay(100);
	hc->hw.r_cirm = 0;
	HFC_outb(hc, R_CIRM, hc->hw.r_cirm);
	udelay(100);
	HFC_outb(hc, R_RAM_SZ, hc->hw.r_ram_sz);


	/* set pcm mode & reset */
	if (test_bit(HFC_CHIP_PCM_SLAVE, &hc->chip)) {
		if (debug & DEBUG_HFCMULTI_INIT) printk(KERN_DEBUG "%s: setting PCM into slave mode\n", __FUNCTION__);
	} else {
		if (debug & DEBUG_HFCMULTI_INIT) printk(KERN_DEBUG "%s: setting PCM into master mode\n", __FUNCTION__);
		hc->hw.r_pcm_md0 |= V_PCM_MD;
	}

	// RAM access test
	HFC_outb(hc, R_RAM_ADDR0, 0);
	HFC_outb(hc, R_RAM_ADDR1, 0);
	HFC_outb(hc, R_RAM_ADDR2, 0);

	for(i=0;i<256;i++) {
		HFC_outb(hc, R_RAM_ADDR0,i);
		HFC_outb(hc, R_RAM_DATA,((i*3)&0xff));
		//udelay(5);
		//HFC_outb(hc, R_RAM_DATA,((i*3)&0xff));
	}

	for(i=0;i<256;i++) {
		HFC_outb(hc, R_RAM_ADDR0,i);
		HFC_inb(hc, R_RAM_DATA);
		rval=HFC_inb(hc, R_INT_DATA);
		if(rval!=((i*3)&0xff))
		{
			printk(KERN_DEBUG "addr:%x val:%x should:%x\n",i,rval,(i*3)&0xff);
			err++;
		}
	}

	if (err) {
		printk(KERN_DEBUG "aborting.1 - %d RAM access errors\n",err);
		err = -EIO;
		goto out;
	}

	i = 0;
	HFC_outb(hc, R_PCM_MD0, hc->hw.r_pcm_md0 | 0x90);
	if (hc->slots == 32)
		HFC_outb(hc, R_PCM_MD1, 0x00);
	if (hc->slots == 64)
		HFC_outb(hc, R_PCM_MD1, 0x10);
	if (hc->slots == 128)
		HFC_outb(hc, R_PCM_MD1, 0x20);
	HFC_outb(hc, R_PCM_MD0, hc->hw.r_pcm_md0 | 0xa0);
	HFC_outb(hc, R_PCM_MD2, 0x00);
	HFC_outb(hc, R_PCM_MD0, hc->hw.r_pcm_md0 | 0x00);
	while (i < 256) {
		HFC_outb_(hc, R_SLOT, i);
		HFC_outb_(hc, A_SL_CFG, 0);
		HFC_outb_(hc, A_CONF, 0);
		hc->slot_owner[i] = -1;
		i++;
	}

	/* set clock speed */
	if (test_bit(HFC_CHIP_CLOCK2, &hc->chip)) {
		if (debug & DEBUG_HFCMULTI_INIT)
			printk(KERN_DEBUG "%s: setting double clock\n", __FUNCTION__);
		HFC_outb(hc, R_BRG_PCM_CFG, V_PCM_CLK);
	}

	/* check if R_F0_CNT counts */
	val = HFC_inb(hc, R_F0_CNTL);
	val += HFC_inb(hc, R_F0_CNTH) << 8;
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "HFC_multi F0_CNT %ld after status ok\n", val);
	spin_unlock_irqrestore(&hc->lock, flags);
	set_current_state(TASK_UNINTERRUPTIBLE);
	while (cnt < 50) { /* max 50 ms */
		schedule_timeout((HZ*10)/1000); /* Timeout 10ms */
		cnt+=10;
		val2 = HFC_inb(hc, R_F0_CNTL);
		val2 += HFC_inb(hc, R_F0_CNTH) << 8;
		if (val2 >= val+4) /* wait 4 pulses */
			break;
	}
	spin_lock_irqsave(&hc->lock, flags);
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "HFC_multi F0_CNT %ld after %dms\n", val2, cnt);
	if (val2 < val+4) {
		printk(KERN_ERR "HFC_multi ERROR 125us pulse not counting.\n");
		if (test_bit(HFC_CHIP_PCM_SLAVE, &hc->chip)) {
			printk(KERN_ERR "HFC_multi This happens in PCM slave mode without connected master.\n");
		}
		if (!test_bit(HFC_CHIP_CLOCK_IGNORE, &hc->chip)) {
			err = -EIO;
			goto out;
		}
	}

	/* set up timer */
	HFC_outb(hc, R_TI_WD, poll_timer);
	hc->hw.r_irqmsk_misc |= V_TI_IRQMSK;

	/* set E1 state machine IRQ */
	if (hc->type == 1)
		hc->hw.r_irqmsk_misc |= V_STA_IRQMSK;

	/* set DTMF detection */
	if (test_bit(HFC_CHIP_DTMF, &hc->chip)) {
		if (debug & DEBUG_HFCMULTI_INIT)
			printk(KERN_DEBUG "%s: enabling DTMF detection for all B-channel\n", __FUNCTION__);
		hc->hw.r_dtmf = V_DTMF_EN | V_DTMF_STOP;
		if (test_bit(HFC_CHIP_ULAW, &hc->chip))
			hc->hw.r_dtmf |= V_ULAW_SEL;
		HFC_outb(hc, R_DTMF_N, 102-1);
		hc->hw.r_irqmsk_misc |= V_DTMF_IRQMSK;
	}

	/* conference engine */
	if (test_bit(HFC_CHIP_ULAW, &hc->chip))
		r_conf_en = V_CONF_EN | V_ULAW;
	else
		r_conf_en = V_CONF_EN;
	HFC_outb(hc, R_CONF_EN, r_conf_en);

	/* setting leds */
	switch(hc->leds) {
		case 1: /* HFC-E1 OEM */
		HFC_outb(hc, R_GPIO_SEL, 0x30);
		HFC_outb(hc, R_GPIO_EN1, 0x0f);
		HFC_outb(hc, R_GPIO_OUT1, 0x00);
		break;

		case 2: /* HFC-4S OEM */
		HFC_outb(hc, R_GPIO_SEL, 0xf0);
		HFC_outb(hc, R_GPIO_EN1, 0xff);
		HFC_outb(hc, R_GPIO_OUT1, 0x00);
		break;
	}

	/* set master clock */
	if (hc->masterclk >= 0) {
		if (debug & DEBUG_HFCMULTI_INIT)
			printk(KERN_DEBUG "%s: setting ST master clock to port %d (0..%d)\n", __FUNCTION__, hc->masterclk, hc->type-1);
		hc->hw.r_st_sync = hc->masterclk | V_AUTO_SYNC;
		HFC_outb(hc, R_ST_SYNC, hc->hw.r_st_sync);
	}

	/* setting misc irq */
	HFC_outb(hc, R_IRQMSK_MISC, hc->hw.r_irqmsk_misc);
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "r_irqmsk_misc.2: 0x%x\n", hc->hw.r_irqmsk_misc);

	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: done\n", __FUNCTION__);
out:
	spin_unlock_irqrestore(&hc->lock, flags);
	return(err);
}


/***************/
/* output leds */
/***************/
static void
hfcmulti_leds(hfc_multi_t *hc)
{
	int i, state, active;
	dchannel_t *dch;
	int led[4];

	hc->ledcount += poll;
	if (hc->ledcount > 4096)
		hc->ledcount -= 4096;

	switch(hc->leds) {
		case 1: /* HFC-E1 OEM */
		/* 2 red blinking: LOS
		   1 red: AIS
		   left green: PH_ACTIVATE
		   right green flashing: FIFO activity
		*/
		i = HFC_inb(hc, R_GPI_IN0) & 0xf0;
		if (!(i & 0x40)) { /* LOS */
			if (hc->e1_switch != i) {
				hc->e1_switch = i;
				hc->hw.r_tx0 &= ~V_OUT_EN;
				HFC_outb(hc, R_TX0, hc->hw.r_tx0);
			}
			if (hc->ledcount & 512)
				led[0] = led[1] = 1;
			else
				led[0] = led[1] = 0;
			led[2] = led[3] = 0;
		} else
		if (!(i & 0x80)) { /* AIS */
			if (hc->e1_switch != i) {
				hc->e1_switch = i;
				hc->hw.r_tx0 |= V_OUT_EN;
				hc->hw.r_tx1 |= V_AIS_OUT;
				HFC_outb(hc, R_TX0, hc->hw.r_tx0);
				HFC_outb(hc, R_TX1, hc->hw.r_tx1);
			}
			if (hc->ledcount & 512)
				led[2] = led[3] = 1;
			else
				led[2] = led[3] = 0;
			led[0] = led[1] = 0;
		} else {
			if (hc->e1_switch != i) {
				/* reset LOS/AIS */
				hc->e1_switch = i;
				hc->hw.r_tx0 |= V_OUT_EN;
				hc->hw.r_tx1 &= ~V_AIS_OUT;
				HFC_outb(hc, R_TX0, hc->hw.r_tx0);
				HFC_outb(hc, R_TX1, hc->hw.r_tx1);
			}
			if (HFC_inb_(hc, R_RX_STA0) & V_SIG_LOS) {
				if (hc->ledcount>>11)
					led[0] = led[1] = 1; /* both red blinking */
				else
					led[0] = led[1] = 0;
			} else
			if (HFC_inb_(hc, R_RX_STA0) & V_AIS) {
				led[0] = led[1] = 1; /* both red */
			} else {
				led[0] = led[1] = 0; /* no red */
			}
			state = 0;
			active = 1;
			if ((dch = hc->chan[16].dch)) {
				state = dch->ph_state;
			}
			if (state == active) {
				led[2] = 1; /* left green */
				if (hc->activity[0]) {
					led[3] = 1; /* right green */
					hc->activity[0] = 0;
				} else
					led[3] = 0; /* no right green */

			} else
				led[2] = led[3] = 0; /* no green */
		}
		HFC_outb(hc, R_GPIO_OUT1,
			(led[0] | (led[1]<<2) | (led[2]<<1) | (led[3]<<3))^0xf); /* leds are inverted */

		break;

		case 2: /* HFC-4S OEM */
		/* red blinking = PH_DEACTIVATE
		   red steady = PH_ACTIVATE
		   green flashing = fifo activity
		*/
		i = 0;
		while(i < 4) {
			state = 0;
			active = -1;
			if ((dch = hc->chan[(i<<2)|2].dch)) {
				state = dch->ph_state;
				active = test_bit(HFC_CFG_NTMODE, &hc->chan[dch->channel].cfg)?3:7;
			}
			if (state) {
				if (state==active) {
					if (hc->activity[i]) {
						led[i] = 1; /* led green */
						hc->activity[i] = 0;
					} else
						led[i] = 2; /* led red */
				} else if (hc->ledcount>>11)
					led[i] = 2; /* led red */
				else
					led[i] = 0; /* led off */
			} else
				led[i] = 0; /* led off */
			i++;
		}
//printk("leds %d %d %d %d\n", led[0], led[1], led[2], led[3]);
		HFC_outb(hc, R_GPIO_EN1,
			((led[0]>0)<<0) | ((led[1]>0)<<1) |
			((led[2]>0)<<2) | ((led[3]>0)<<3));
		HFC_outb(hc, R_GPIO_OUT1,
			((led[0]&1)<<0) | ((led[1]&1)<<1) |
			((led[2]&1)<<2) | ((led[3]&1)<<3));
		break;
	}
}
/**************************/
/* read dtmf coefficients */
/**************************/

static void
hfcmulti_dtmf(hfc_multi_t *hc)
{
	signed long coeff[16];
	DWORD mantissa;
	int co, ch;
	bchannel_t *bch = NULL;
	BYTE exponent;
	int dtmf = 0;
	int addr;
	WORD w_float;
	struct sk_buff *skb;

	if (debug & DEBUG_HFCMULTI_DTMF)
		printk(KERN_DEBUG "%s: dtmf detection irq\n", __FUNCTION__);
	ch = 0;
	while(ch < 32) {
		// only process enabled B-channels
		if (!(bch = hc->chan[ch].bch)) {
			ch++;
			continue;
		}
		if (!hc->created[hc->chan[ch].port]) {
			ch++;
			continue;
		}
		if (bch->protocol != ISDN_PID_L1_B_64TRANS) {
			ch++;
			continue;
		}
		if (debug & DEBUG_HFCMULTI_DTMF)
			printk(KERN_DEBUG "%s: dtmf channel %d:", __FUNCTION__, ch);
		dtmf = 1;
		co = 0;
		while(co < 8) {
			// read W(n-1) coefficient
			addr = hc->DTMFbase + ((co<<7) | (ch<<2));
			HFC_outb_(hc, R_RAM_ADDR0, addr);
			HFC_outb_(hc, R_RAM_ADDR1, addr>>8);
			HFC_outb_(hc, R_RAM_ADDR2, (addr>>16) | V_ADDR_INC);
			w_float = HFC_inb_(hc, R_RAM_DATA);
#ifdef CONFIG_HFCMULTI_PCIMEM
			w_float |= (HFC_inb_(hc, R_RAM_DATA) << 8);
#else
			w_float |= (HFC_getb(hc) << 8);
#endif
			if (debug & DEBUG_HFCMULTI_DTMF)
				printk(" %04x", w_float);

			// decode float (see chip doc)
			mantissa = w_float & 0x0fff;
			if (w_float & 0x8000)
				mantissa |= 0xfffff000;
			exponent = (w_float>>12) & 0x7;
			if (exponent) {
				mantissa ^= 0x1000;
				mantissa <<= (exponent-1);
			}

			// store coefficient
			coeff[co<<1] = mantissa;

			// read W(n) coefficient
			w_float = HFC_inb_(hc, R_RAM_DATA);
#ifdef CONFIG_HFCMULTI_PCIMEM
			w_float |= (HFC_inb_(hc, R_RAM_DATA) << 8);
#else
			w_float |= (HFC_getb(hc) << 8);
#endif
			if (debug & DEBUG_HFCMULTI_DTMF)
				printk(" %04x", w_float);

			// decode float (see chip doc)
			mantissa = w_float & 0x0fff;
			if (w_float & 0x8000)
				mantissa |= 0xfffff000;
			exponent = (w_float>>12) & 0x7;
			if (exponent) {
				mantissa ^= 0x1000;
				mantissa <<= (exponent-1);
			}

			// store coefficient
			coeff[(co<<1)|1] = mantissa;
			co++;
		}
		skb = create_link_skb(PH_CONTROL | INDICATION, HW_HFC_COEFF, sizeof(coeff), coeff, 0);
		if (!skb) {
			printk(KERN_WARNING "%s: No memory for skb\n", __FUNCTION__);
			ch++;
			continue;
		}
		if (debug & DEBUG_HFCMULTI_DTMF) {
			printk("\n");
			printk("%s: DTMF ready %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx len=%d\n", __FUNCTION__,
			coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6], coeff[7], skb->len);
		}

#ifdef FIXME   // TODO changed
				if ((bch->inst.pid.protocol[2] == ISDN_PID_L2_B_RAWDEV) && bch->dev)
					hif = &bch->dev->rport.pif;
				else
					hif = &bch->inst.up;
#endif
		if (mISDN_queue_up(&bch->inst, 0, skb))
			dev_kfree_skb(skb);

		ch++;
	}

	// restart DTMF processing
	hc->dtmf = dtmf;
	if (dtmf)
		HFC_outb_(hc, R_DTMF, hc->hw.r_dtmf | V_RST_DTMF);
}

#ifdef CONFIG_HFCMULTI_PCIMEM
/*******************/
/* write fifo data */
/*******************/
void write_fifo_data(hfc_multi_t *hc,BYTE *dest,int len)
{
	int i,remain;

	remain=len;

#ifdef FIFO_32BIT_ACCESS
		for(i=0;i<remain/4;i++)
		{
			HFC_outl_(hc, A_FIFO_DATA0, *((DWORD *)dest));
			remain-=4;
			dest+=4;
		}
#endif

#ifdef FIFO_16BIT_ACCESS
		for(i=0;i<remain/2;i++)
		{
			HFC_outw_(hc, A_FIFO_DATA0, *((WORD *)dest));
			remain-=2;
			dest+=2;
		}
#endif

		for(i=0;i<remain;i++) HFC_outb_(hc, A_FIFO_DATA0, *dest++);

}
#endif

#ifndef CONFIG_HFCMULTI_PCIMEM
/*******************/
/* write fifo data */
/*******************/
void write_fifo_data(hfc_multi_t *hc,BYTE *dest,int len)
{
	int i,remain;

	remain=len;
	HFC_set(hc, A_FIFO_DATA0);

#ifdef FIFO_32BIT_ACCESS
		for(i=0;i<remain/4;i++)
		{
			HFC_putl(hc, *((DWORD *)dest));
			remain-=4;
			dest+=4;
		}
#endif

#ifdef FIFO_16BIT_ACCESS
		for(i=0;i<remain/2;i++)
		{
			HFC_putw(hc, *((WORD *)dest));
			remain-=2;
			dest+=2;
		}
#endif

		for(i=0;i<remain;i++) HFC_putb(hc, *dest++);
}
#endif

/*********************************/
/* fill fifo as much as possible */
/*********************************/

static void
hfcmulti_tx(hfc_multi_t *hc, int ch, dchannel_t *dch, bchannel_t *bch)
{
	int i, ii,temp;
	int Zspace, z1, z2;
	int Fspace, f1, f2;
	BYTE *d, *buf = NULL;
	int *len = NULL, *idx = NULL; /* = NULL, to make GCC happy */
	int txpending, slot_tx;
	int hdlc = 0;

	f1 = HFC_inb_(hc, A_F1);
	f2 = HFC_inb_(hc, A_F2);
	//printk(KERN_DEBUG "txf12:%x %x\n",f1,f2);
	/* get skb, fifo & mode */
	if (dch) {
		buf = dch->tx_buf;
		len = &dch->tx_len;
		idx = &dch->tx_idx;
		hdlc = 1;
	}
	if (bch) {
		buf = bch->tx_buf;
		len = &bch->tx_len;
		idx = &bch->tx_idx;
		if (bch->protocol == ISDN_PID_L1_B_64HDLC)
			hdlc = 1;
	}
	txpending = hc->chan[ch].txpending;
	slot_tx = hc->chan[ch].slot_tx;
	if ((!(*len)) && txpending!=1)
		return; /* no data */

//printk("debug: data: len = %d, txpending = %d!!!!\n", *len, txpending);
	/* lets see how much data we will have left in buffer */
	HFC_outb_(hc, R_FIFO, ch<<1);
	HFC_wait_(hc);
	if (txpending == 2) {
		/* reset fifo */
		HFC_outb_(hc, R_INC_RES_FIFO, V_RES_F);
		HFC_wait_(hc);
		HFC_outb(hc, A_SUBCH_CFG, 0);
		txpending = 1;
	}
next_frame:
	if (hdlc) {
		f1 = HFC_inb_(hc, A_F1);
		f2 = HFC_inb_(hc, A_F2);
		while (f2 != (temp=HFC_inb_(hc, A_F2))) {
			if (debug & DEBUG_HFCMULTI_FIFO)
				printk(KERN_DEBUG "%s: reread f2 because %d!=%d\n", __FUNCTION__, temp, f2);
			f2 = temp; /* repeat until F2 is equal */
		}
		Fspace = f2-f1-1;
		if (Fspace < 0)
			Fspace += hc->Flen;
		/* Old FIFO handling doesn't give us the current Z2 read
		 * pointer, so we cannot send the next frame before the fifo
		 * is empty. It makes no difference except for a slightly
		 * lower performance.
		 */
		if (test_bit(HFC_CHIP_REVISION0, &hc->chip)) {
			if (f1 != f2)
				Fspace = 0;
			else
				Fspace = 1;
		}
		/* one frame only for ST D-channels, to allow resending */
		if (hc->type!=1 && dch) {
			if (f1 != f2)
				Fspace = 0;
		}
		/* F-counter full condition */
		if (Fspace == 0)
			return;
	}
	z1 = HFC_inw_(hc, A_Z1) - hc->Zmin;
	z2 = HFC_inw_(hc, A_Z2) - hc->Zmin;
	while(z2 != (temp=(HFC_inw_(hc, A_Z2) - hc->Zmin))) {
		if (debug & DEBUG_HFCMULTI_FIFO)
			printk(KERN_DEBUG "%s: reread z2 because %d!=%d\n", __FUNCTION__, temp, z2);
		z2 = temp; /* repeat unti Z2 is equal */
	}
	Zspace = z2-z1-1;
	if (Zspace < 0)
		Zspace += hc->Zlen;
	/* buffer too full, there must be at least one more byte for 0-volume */
	if (Zspace < 4) /* just to be safe */
		return;

	/* if no data */
	if (!(*len)) {
		if (z1 == z2) { /* empty */
			/* if done with FIFO audio data during PCM connection */
			if (!hdlc && txpending && slot_tx>=0) {
				if (debug & DEBUG_HFCMULTI_MODE)
					printk(KERN_DEBUG "%s: reconnecting PCM due to no more FIFO data: channel %d slot_tx %d\n", __FUNCTION__, ch, slot_tx);

				/* connect slot */
				HFC_outb(hc, A_CON_HDLC, 0xc0 | 0x00 | V_HDLC_TRP | V_IFF);
				HFC_outb_(hc, R_FIFO, ch<<1 | 1);
				HFC_wait_(hc);
				HFC_outb(hc, A_CON_HDLC, 0xc0 | 0x00 | V_HDLC_TRP | V_IFF);
				HFC_outb_(hc, R_FIFO, ch<<1);
				HFC_wait_(hc);
			}
			txpending = hc->chan[ch].txpending = 0;
		}
		return; /* no data */
	}

	/* if audio data */
	if (!hdlc && !txpending && slot_tx>=0) {
		if (debug & DEBUG_HFCMULTI_MODE)
			printk(KERN_DEBUG "%s: disconnecting PCM due to FIFO data: channel %d slot_tx %d\n", __FUNCTION__, ch, slot_tx);
		/* disconnect slot */
		HFC_outb(hc, A_CON_HDLC, 0x80 | 0x00 | V_HDLC_TRP | V_IFF);
		HFC_outb_(hc, R_FIFO, ch<<1 | 1);
		HFC_wait_(hc);
		HFC_outb(hc, A_CON_HDLC, 0x80 | 0x00 | V_HDLC_TRP | V_IFF);
		HFC_outb_(hc, R_FIFO, ch<<1);
		HFC_wait_(hc);
	}
	txpending = hc->chan[ch].txpending = 1;

	/* show activity */
	hc->activity[hc->chan[ch].port] = 1;

	/* fill fifo to what we have left */
	i = *idx;
	ii = *len;
	d = buf + i;
	if (ii-i > Zspace)
		ii = Zspace+i;
	if (debug & DEBUG_HFCMULTI_FIFO) {
		printk(KERN_DEBUG "%s: fifo(%d) has %d bytes space left (z1=%04x, z2=%04x) sending %d of %d bytes %s\n",
			__FUNCTION__, ch, Zspace, z1, z2, ii-i, (*len)-i, hdlc?"HDLC":"TRANS");
	}

	write_fifo_data(hc,d,ii-i);
	*idx = ii;

	/* if not all data has been written */
	if (ii != *len) {
		/* NOTE: fifo is started by the calling function */
		return;
	}

	/* if all data has been written */
	if (hdlc) {
		/* increment f-counter */
		HFC_outb_(hc, R_INC_RES_FIFO, V_INC_F);
		HFC_wait_(hc);
	}
	if (dch) {
		// check for next frame
		/*if (test_and_clear_bit(FLG_TX_NEXT, &dch->DFlags)) {
			if (dch->next_skb) {
				dch->tx_idx = 0;
				dch->tx_len = dch->next_skb->len;
				memcpy(dch->tx_buf, dch->next_skb->data, dch->tx_len);
				dchannel_sched_event(dch, D_XMTBUFREADY);
				goto next_frame;
			} else
				printk(KERN_WARNING "%s: tx irq TX_NEXT without skb (dch ch=%d)\n", __FUNCTION__, ch);
		} */

		// check for next frame
		if (test_bit(FLG_TX_NEXT, &dch->DFlags)) {
			struct sk_buff	*skb = dch->next_skb;
			mISDN_head_t	*hh;
			if (skb) {
				dch->next_skb = NULL;
				test_and_clear_bit(FLG_TX_NEXT, &dch->DFlags);
				hh = mISDN_HEAD_P(skb);
				dch->tx_idx = 0;
				dch->tx_len = skb->len;
				memcpy(dch->tx_buf, skb->data, dch->tx_len);
				skb_trim(skb, 0);
				if (mISDN_queueup_newhead(&dch->inst, 0, PH_DATA_CNF, hh->dinfo, skb)) dev_kfree_skb(skb);
				goto next_frame;
			} else {
				test_and_clear_bit(FLG_TX_NEXT, &dch->DFlags);
				printk(KERN_WARNING "%s: tx irq TX_NEXT without skb (dch ch=%d)\n", __FUNCTION__, ch);
			}
		}

		test_and_clear_bit(FLG_TX_BUSY, &dch->DFlags);
		dch->tx_idx = dch->tx_len = 0;
	}
	if (bch) {
		// check for next frame
		if (test_bit(BC_FLG_TX_NEXT, &bch->Flag)) {
			struct sk_buff	*skb = bch->next_skb;
			mISDN_head_t	*hh;
			if (skb) {
				bch->next_skb = NULL;
				test_and_clear_bit(BC_FLG_TX_NEXT, &bch->Flag);
				hh = mISDN_HEAD_P(skb);
				bch->tx_idx = 0;
				bch->tx_len = skb->len;
				memcpy(bch->tx_buf, skb->data, bch->tx_len);
				skb_trim(skb, 0);
				queue_bch_frame(bch, CONFIRM, hh->dinfo, skb);
				goto next_frame;
			} else {
				test_and_clear_bit(BC_FLG_TX_NEXT, &bch->Flag);
				printk(KERN_WARNING "hfcB tx irq TX_NEXT without skb\n");
			}
		}

		test_and_clear_bit(BC_FLG_TX_BUSY, &bch->Flag);
		bch->tx_idx = bch->tx_len = 0;
	}
	/* now we have no more data, so in case of transparent,
	 * we set the last byte in fifo to 'silence' in case we will get
	 * no more data at all. this prevents sending an undefined value.
	 */
	if (!hdlc)
		HFC_outb_(hc, A_FIFO_DATA0_NOINC, silence);
}


#ifdef CONFIG_HFCMULTI_PCIMEM
/******************/
/* read fifo data */
/******************/
void read_fifo_data(hfc_multi_t *hc,BYTE *dest,int len)
{
	int i,remain;

	remain=len;

#ifdef FIFO_32BIT_ACCESS
		for(i=0;i<remain/4;i++)
		{
			*((DWORD *)dest) = HFC_inl_(hc, A_FIFO_DATA0);
			remain-=4;
			dest+=4;
		}
#endif

#ifdef FIFO_16BIT_ACCESS
		for(i=0;i<remain/2;i++)
		{
			*((WORD *)dest) = HFC_inw_(hc, A_FIFO_DATA0);
			remain-=2;
			dest+=2;
		}
#endif

		for(i=0;i<remain;i++) *dest++ = HFC_inb_(hc, A_FIFO_DATA0);

}
#endif

#ifndef CONFIG_HFCMULTI_PCIMEM
/******************/
/* read fifo data */
/******************/
void read_fifo_data(hfc_multi_t *hc,BYTE *dest,int len)
{
	int i,remain;

	remain=len;
	HFC_set(hc, A_FIFO_DATA0);

#ifdef FIFO_32BIT_ACCESS
		for(i=0;i<remain/4;i++)
		{
			*((DWORD *)dest) = HFC_getl(hc);
			remain-=4;
			dest+=4;
		}
#endif

#ifdef FIFO_16BIT_ACCESS
		for(i=0;i<remain/2;i++)
		{
			*((WORD *)dest) = HFC_getw(hc);
			remain-=2;
			dest+=2;
		}
#endif

		for(i=0;i<remain;i++) *dest++ = HFC_getb(hc);
}
#endif


static void
hfcmulti_rx(hfc_multi_t *hc, int ch, dchannel_t *dch, bchannel_t *bch)
{
	int temp;
	int Zsize, z1, z2 = 0; /* = 0, to make GCC happy */
	int f1 = 0, f2 = 0; /* = 0, to make GCC happy */
	BYTE *d, *buf = NULL;
	int *idx = NULL, max = 0; /* = 0, to make GCC happy */
	int hdlc = 0;
	struct sk_buff *skb;

	/* get skb, fifo & mode */
	if (dch) {
		buf = hc->chan[ch].rx_buf;
		idx = &hc->chan[ch].rx_idx;
		max = MAX_DFRAME_LEN_L1;
		hdlc = 1;
	}
	if (bch) {
		buf = bch->rx_buf;
		idx = &bch->rx_idx;
		max = MAX_DATA_MEM;
		if (bch->protocol == ISDN_PID_L1_B_64HDLC)
			hdlc = 1;
	}

	/* lets see how much data we received */
	HFC_outb_(hc, R_FIFO, (ch<<1)|1);
	HFC_wait_(hc);
next_frame:
	if (hdlc) {
		f1 = HFC_inb_(hc, A_F1);
		while (f1 != (temp=HFC_inb_(hc, A_F1))) {
			if (debug & DEBUG_HFCMULTI_FIFO)
				printk(KERN_DEBUG "%s: reread f1 because %d!=%d\n", __FUNCTION__, temp, f1);
			f1 = temp; /* repeat until F1 is equal */
		}
		f2 = HFC_inb_(hc, A_F2);
	}
	//if(f1!=f2) printk(KERN_DEBUG "got a chan:%d framef12:%x %x!!!!\n",ch,f1,f2);
	//printk(KERN_DEBUG "got a chan:%d framef12:%x %x!!!!\n",ch,f1,f2);
	z1 = HFC_inw_(hc, A_Z1) - hc->Zmin;
	while(z1 != (temp=(HFC_inw_(hc, A_Z1) - hc->Zmin))) {
		if (debug & DEBUG_HFCMULTI_FIFO)
			printk(KERN_DEBUG "%s: reread z2 because %d!=%d\n", __FUNCTION__, temp, z2);
		z1 = temp; /* repeat until Z1 is equal */
	}
	z2 = HFC_inw_(hc, A_Z2) - hc->Zmin;
	Zsize = z1-z2;
	if (hdlc && f1!=f2) /* complete hdlc frame */
		Zsize++;
	if (Zsize < 0)
		Zsize += hc->Zlen;
	/* if buffer is empty */
	if (Zsize <= 0)
		return;

	/* show activity */
	hc->activity[hc->chan[ch].port] = 1;

	/* empty fifo with what we have */
	if (hdlc) {
		if (debug & DEBUG_HFCMULTI_FIFO)
			printk(KERN_DEBUG "%s: fifo(%d) reading %d bytes (z1=%04x, z2=%04x) HDLC %s (f1=%d, f2=%d) got=%d\n",
				__FUNCTION__, ch, Zsize, z1, z2, (f1==f2)?"fragment":"COMPLETE", f1, f2, Zsize+*idx);
		/* HDLC */
		if ((Zsize + *idx) > max) {
			if (debug & DEBUG_HFCMULTI_FIFO)
				printk(KERN_DEBUG "%s: hdlc-frame too large.\n", __FUNCTION__);
			*idx = 0;
			HFC_outb_(hc, R_INC_RES_FIFO, V_RES_F);
			HFC_wait_(hc);
			return;
		}

		read_fifo_data(hc,buf+ *idx,Zsize);
		*idx += Zsize;

		if (f1 != f2) {
			/* increment Z2,F2-counter */
			HFC_outb_(hc, R_INC_RES_FIFO, V_INC_F);
			HFC_wait_(hc);
			/* check size */
			if (*idx < 4) {
				if (debug & DEBUG_HFCMULTI_FIFO)
					printk(KERN_DEBUG "%s: Frame below minimum size\n", __FUNCTION__);
				*idx = 0;
				goto next_frame;
			}
			/* there is at least one complete frame, check crc */
			if (buf[(*idx)-1]) {
				if (debug & DEBUG_HFCMULTI_CRC)
					printk(KERN_DEBUG "%s: CRC-error\n", __FUNCTION__);
				*idx = 0;
				goto next_frame;
			}
			/* only send dchannel if in active state */
			if (dch && hc->type==1 && hc->chan[ch].e1_state!=1)
			{
				*idx = 0;
				goto next_frame;
			}
			if (!(skb = alloc_stack_skb((*idx)-3, (bch)?bch->up_headerlen:dch->up_headerlen))) {
				printk(KERN_DEBUG "%s: No mem for skb\n", __FUNCTION__);
				*idx = 0;
				goto next_frame;
			}
			memcpy(skb_put(skb, (*idx)-3), buf, (*idx)-3);
			if (debug & DEBUG_HFCMULTI_FIFO) {
				temp = 0;
				while(temp < (*idx)-3)
					printk("%02x ", skb->data[temp++]);
				printk("\n");
			}
			if (dch) {
				if (debug & DEBUG_HFCMULTI_FIFO)
					printk(KERN_DEBUG "%s: sending D-channel frame to user space.\n", __FUNCTION__);
				// schedule D-channel event
				mISDN_queueup_newhead(&dch->inst, 0, PH_DATA_IND, MISDN_ID_ANY, skb);
			}
			if (bch)
				queue_bch_frame(bch, INDICATION, MISDN_ID_ANY, skb);
			*idx = 0;
			goto next_frame;
		}
		/* there is an incomplete frame */
	} else {
		/* transparent */
		if (Zsize > MAX_DATA_MEM) Zsize = MAX_DATA_MEM;
		if (!(skb = alloc_stack_skb(Zsize, bch->up_headerlen))) {
			printk(KERN_DEBUG "%s: No mem for skb\n", __FUNCTION__);
			HFC_outb_(hc, R_INC_RES_FIFO, V_RES_F);
			HFC_wait_(hc);
			return;
		}

		if (debug & DEBUG_HFCMULTI_FIFO)
			printk(KERN_DEBUG "%s: fifo(%d) reading %d bytes (z1=%04x, z2=%04x) TRANS\n",
				__FUNCTION__, ch, Zsize, z1, z2);
		d = skb_put(skb, Zsize);


		read_fifo_data(hc,d,Zsize);

		if (dch) {
			// schedule D-channel event
			//skb_queue_tail(&dch->rqueue, skb);
			//dchannel_sched_event(dch, D_RCVBUFREADY);
			mISDN_queueup_newhead(&dch->inst, 0, PH_DATA_IND, MISDN_ID_ANY, skb);
		}
		if (bch)
			queue_bch_frame(bch, INDICATION, MISDN_ID_ANY, skb);
	}
}


/*********************/
/* Interrupt handler */
/*********************/

static void signal_state_up(dchannel_t *dch, int dinfo, char *msg)
{
	struct sk_buff *skb;

	if (debug & DEBUG_HFCMULTI_STATE)
		 printk(KERN_DEBUG "%s: %s\n", __FUNCTION__,msg);

	skb = create_link_skb(PH_CONTROL | INDICATION, dinfo, 0, NULL, 0);
	if(!skb) return;

	if (mISDN_queue_up(&dch->inst, 0, skb))
		dev_kfree_skb(skb);
}

static irqreturn_t
hfcmulti_interrupt(int intno, void *dev_id, struct pt_regs *regs)
{
#ifdef IRQCOUNT_DEBUG
	static int iq1=0,iq2=0,iq3=0,iq4=0,iq5=0,iq6=0,iqcnt=0;
#endif
	hfc_multi_t *hc = dev_id;
	bchannel_t *bch;
	dchannel_t *dch;
	u_char r_irq_statech, status, r_irq_misc, r_irq_oview, r_irq_fifo_bl;
	//u_char bl1,bl2;
#ifdef CONFIG_PLX_PCI_BRIDGE
	u_short *plx_acc,wval;
#endif
	int ch,i,j,temp;

#ifdef CONFIG_PLX_PCI_BRIDGE
	plx_acc=(u_short*)(hc->plx_membase+0x4c);
	wval=*plx_acc;
	if(!(wval&0x04))
	{
		if(wval&0x20)
		{
			//printk(KERN_WARNING "NO irq  LINTI1:%x\n",wval);
			printk(KERN_WARNING "got irq  LINTI2\n");
		}
		return(IRQ_NONE);
	}
#endif

	spin_lock(&hc->lock);

	if (!hc) {
		printk(KERN_WARNING "HFC-multi: Spurious interrupt!\n");
		irq_notforus:

#ifdef CONFIG_PLX_PCI_BRIDGE
	//plx_acc=(u_short*)(hc->plx_membase+0x4c);
	//*plx_acc=0xc00;  // clear LINTI1 & LINTI2
	//*plx_acc=0xc41;
#endif
		spin_unlock(&hc->lock);
		return(IRQ_NONE);
	}
	status = HFC_inb_(hc, R_STATUS);
	r_irq_statech = HFC_inb_(hc, R_IRQ_STATECH);
#ifdef IRQCOUNT_DEBUG
	if(r_irq_statech) iq1++;
	if(status&V_DTMF_STA) iq2++;
	if(status&V_LOST_STA) iq3++;
	if(status&V_EXT_IRQSTA) iq4++;
	if(status&V_MISC_IRQSTA) iq5++;
	if(status&V_FR_IRQSTA) iq6++;
	if(iqcnt++>5000)
	{
		printk(KERN_ERR "iq1:%x iq2:%x iq3:%x iq4:%x iq5:%x iq6:%x\n",iq1,iq2,iq3,iq4,iq5,iq6);
		iqcnt=0;

	}
#endif
	if (!r_irq_statech && !(status & (V_DTMF_STA | V_LOST_STA | V_EXT_IRQSTA | V_MISC_IRQSTA | V_FR_IRQSTA))) {
		/* irq is not for us */
		//if(status) printk(KERN_WARNING "nofus:%x\n",status);
		goto irq_notforus;
	}
	hc->irqcnt++;
	if (r_irq_statech) {
		if (hc->type != 1) {
			/* state machine */
			ch = 0;
			while(ch < 32) {
				if ((dch = hc->chan[ch].dch)) {
					if (r_irq_statech & 1) {
						HFC_outb_(hc, R_ST_SEL, hc->chan[ch].port);
						dch->ph_state = HFC_inb(hc, A_ST_RD_STATE) & 0x0f;
						if (dch->ph_state == (test_bit(HFC_CFG_NTMODE, &hc->chan[dch->channel].cfg)?3:7)) {
							HFC_outb_(hc, R_FIFO, (ch<<1)|1);
							HFC_wait_(hc);
							HFC_outb_(hc, R_INC_RES_FIFO, V_RES_F);
							HFC_wait_(hc);
							dch->tx_idx=0;
						}

						ph_state_change(dch);
						if (debug & DEBUG_HFCMULTI_STATE)
							printk(KERN_DEBUG "%s: S/T newstate %x port %d\n", __FUNCTION__, dch->ph_state, hc->chan[ch].port);
					}
					r_irq_statech >>= 1;
				}
				ch++;
			}
		}
	}
	if (status & V_EXT_IRQSTA) {
		/* external IRQ */
	}
	if (status & V_LOST_STA) {
		/* LOST IRQ */
		HFC_outb(hc, R_INC_RES_FIFO, V_RES_LOST); /* clear irq! */
	}
	if (status & V_MISC_IRQSTA) {
		/* misc IRQ */
		r_irq_misc = HFC_inb_(hc, R_IRQ_MISC);
		if (r_irq_misc & V_STA_IRQ) {
			if (hc->type == 1) {
				/* state machine */
				dch = hc->chan[16].dch;
				dch->ph_state = HFC_inb_(hc, R_E1_RD_STA) & 0x7;
				ph_state_change(dch);
				if (debug & DEBUG_HFCMULTI_STATE)
					printk(KERN_DEBUG "%s: E1 newstate %x\n", __FUNCTION__, dch->ph_state);
			}
		}
		if (r_irq_misc & V_TI_IRQ) {
			/* -> timer IRQ */
			ch = 0;
			while(ch < 32) {
				if ((dch = hc->chan[ch].dch))
				if (hc->created[hc->chan[ch].port]) {
					hfcmulti_tx(hc, ch, dch, NULL);
					/* fifo is started when switching to rx-fifo */
					hfcmulti_rx(hc, ch, dch, NULL);
					if (hc->chan[ch].nt_timer > -1) {
						if (!(--hc->chan[ch].nt_timer)) {
							ph_state_change(dch);
							if (debug & DEBUG_HFCMULTI_STATE)
								printk(KERN_DEBUG "%s: nt_timer at state %x\n", __FUNCTION__, dch->ph_state);
						}
					}
				}
				if ((bch = hc->chan[ch].bch))
				if (hc->created[hc->chan[ch].port]) {
					if (bch->protocol != ISDN_PID_NONE) {
						hfcmulti_tx(hc, ch, NULL, bch);
						/* fifo is started when switching to rx-fifo */
						hfcmulti_rx(hc, ch, NULL, bch);
					}
				}
				ch++;
			}
			if (hc->type == 1)
			if (hc->created[0]) {
				if (test_bit(HFC_CFG_REPORT_LOS, &hc->chan[16].cfg)) {
					if (debug & DEBUG_HFCMULTI_SYNC)
						printk(KERN_DEBUG "%s: (id=%d) E1 got LOS\n", __FUNCTION__, hc->id);
					/* LOS */
					temp = HFC_inb_(hc, R_RX_STA0) & V_SIG_LOS;
					if (!temp && hc->chan[16].los) signal_state_up(dch,HW_LOS , "LOS detected");
					if (temp && !hc->chan[16].los) signal_state_up(dch,HW_LOS_OFF , "LOS gone");
					hc->chan[16].los = temp;
				}
				if (test_bit(HFC_CFG_REPORT_AIS, &hc->chan[16].cfg)) {
					if (debug & DEBUG_HFCMULTI_SYNC)
						printk(KERN_DEBUG "%s: (id=%d) E1 got AIS\n", __FUNCTION__, hc->id);
					/* AIS */
					temp = HFC_inb_(hc, R_RX_STA0) & V_AIS;
					if (!temp && hc->chan[16].ais) signal_state_up(dch,HW_AIS , "AIS detected");
					if (!temp && hc->chan[16].ais) signal_state_up(dch,HW_AIS_OFF , "AIS gone");
					hc->chan[16].ais = temp;
				}
				if (test_bit(HFC_CFG_REPORT_SLIP, &hc->chan[16].cfg)) {
					if (debug & DEBUG_HFCMULTI_SYNC)
						printk(KERN_DEBUG "%s: (id=%d) E1 got SLIP (RX)\n", __FUNCTION__, hc->id);
					/* SLIP */
					temp = HFC_inb_(hc, R_SLIP) & V_FOSLIP_RX;
					if (!temp && hc->chan[16].slip_rx) signal_state_up(dch,HW_SLIP_RX , " bit SLIP detected RX");
					hc->chan[16].slip_rx = temp;
					temp = HFC_inb_(hc, R_SLIP) & V_FOSLIP_TX;
					if (!temp && hc->chan[16].slip_tx) signal_state_up(dch,HW_SLIP_TX , " bit SLIP detected TX");
					hc->chan[16].slip_tx = temp;
				}
				temp = HFC_inb_(hc, R_JATT_DIR);
				switch(hc->chan[16].sync) {
					case 0:
					if ((temp&0x60) == 0x60) {
						if (debug & DEBUG_HFCMULTI_SYNC)
							printk(KERN_DEBUG "%s: (id=%d) E1 now in clock sync\n", __FUNCTION__, hc->id);
						HFC_outb(hc, R_RX_OFF, hc->chan[16].jitter | V_RX_INIT);
						HFC_outb(hc, R_TX_OFF, hc->chan[16].jitter | V_RX_INIT);
						hc->chan[16].sync = 1;
						goto check_framesync;
					}
					break;

					case 1:
					if ((temp&0x60) != 0x60) {
						if (debug & DEBUG_HFCMULTI_SYNC)
							printk(KERN_DEBUG "%s: (id=%d) E1 lost clock sync\n", __FUNCTION__, hc->id);
						hc->chan[16].sync = 0;
						break;
					}
					check_framesync:
					temp = HFC_inb_(hc, R_RX_STA0);
					if (temp == 0x27) {
						if (debug & DEBUG_HFCMULTI_SYNC)
							printk(KERN_DEBUG "%s: (id=%d) E1 now in frame sync\n", __FUNCTION__, hc->id);
						hc->chan[16].sync = 2;
					}
					break;

					case 2:
					if ((temp&0x60) != 0x60) {
						if (debug & DEBUG_HFCMULTI_SYNC)
							printk(KERN_DEBUG "%s: (id=%d) E1 lost clock & frame sync\n", __FUNCTION__, hc->id);
						hc->chan[16].sync = 0;
						break;
					}
					temp = HFC_inb_(hc, R_RX_STA0);
					if (temp != 0x27) {
						if (debug & DEBUG_HFCMULTI_SYNC)
							printk(KERN_DEBUG "%s: (id=%d) E1 lost frame sync\n", __FUNCTION__, hc->id);
						hc->chan[16].sync = 1;
					}
					break;
				}
			}
			if (hc->leds)
				hfcmulti_leds(hc);
		}

		if (r_irq_misc & V_DTMF_IRQ) {
			/* -> DTMF IRQ */
			hfcmulti_dtmf(hc);
		}
	}
	if (status & V_FR_IRQSTA) {
		/* FIFO IRQ */
		r_irq_oview = HFC_inb_(hc, R_IRQ_OVIEW);
		//if(r_irq_oview) printk(KERN_DEBUG "OV:%x\n",r_irq_oview);
		i = 0;
		while(i < 8) {
			if (r_irq_oview & (1 << i)) {
				r_irq_fifo_bl = HFC_inb_(hc, R_IRQ_FIFO_BL0 + i);
				//r_irq_fifo_bl = HFC_inb_(hc, R_INT_DATA);
				//if(r_irq_fifo_bl) printk(KERN_DEBUG "BL%d:%x\n",i,r_irq_fifo_bl);
				//bl1 = HFC_inb_(hc, R_IRQ_FIFO_BL0);
				//bl2 = HFC_inb_(hc, R_IRQ_FIFO_BL0);
				//printk(KERN_DEBUG "zero:%x :%x\n",bl1,bl2);
				r_irq_fifo_bl = HFC_inb_(hc, R_IRQ_FIFO_BL0 + i);
				j = 0;
				while(j < 8) {
					ch = (i<<2) + (j>>1);
					if (r_irq_fifo_bl & (1 << j)) {
						if ((dch = hc->chan[ch].dch))
						if (hc->created[hc->chan[ch].port]) {
							//printk(KERN_DEBUG "txchan:%d\n",ch);
							hfcmulti_tx(hc, ch, dch, NULL);
							/* start fifo */
							HFC_outb_(hc, R_FIFO, 0);
							HFC_wait_(hc);
						}
						if ((bch = hc->chan[ch].bch))
						if (hc->created[hc->chan[ch].port]) {
							if (bch->protocol != ISDN_PID_NONE) {
								hfcmulti_tx(hc, ch, NULL, bch);
								//printk(KERN_DEBUG "txchan:%d\n",ch);
								/* start fifo */
								HFC_outb_(hc, R_FIFO, 0);
								HFC_wait_(hc);
							}
						}
					}
					j++;
					if (r_irq_fifo_bl & (1 << j)) {
						if ((dch = hc->chan[ch].dch))
						if (hc->created[hc->chan[ch].port]) {
							hfcmulti_rx(hc, ch, dch, NULL);
							//printk(KERN_DEBUG "rxchan:%d\n",ch);
						}
						if ((bch = hc->chan[ch].bch))
						if (hc->created[hc->chan[ch].port]) {
							if (bch->protocol != ISDN_PID_NONE) {
								hfcmulti_rx(hc, ch, NULL, bch);
								//printk(KERN_DEBUG "rxchan:%d\n",ch);
							}
						}
					}
					j++;
				}
			}
			i++;
		}
	}

#ifdef CONFIG_PLX_PCI_BRIDGE
	//plx_acc=(u_short*)(hc->plx_membase+0x4c);
	//*plx_acc=0xc00;  // clear LINTI1 & LINTI2
	//*plx_acc=0xc41;
#endif
	spin_unlock(&hc->lock);
	return(IRQ_HANDLED);
}


/********************************************************************/
/* timer callback for D-chan busy resolution. Currently no function */
/********************************************************************/

static void
hfcmulti_dbusy_timer(hfc_multi_t *hc)
{
}


/***************************************************************/
/* activate/deactivate hardware for selected channels and mode */
/***************************************************************/

/* configure B-channel with the given protocol
 * ch eqals to the HFC-channel (0-31)
 * ch is the number of channel (0-4,4-7,8-11,12-15,16-19,20-23,24-27,28-31 for S/T, 1-31 for E1)
 * the hdlc interrupts will be set/unset
 *
 */
static int
mode_hfcmulti(hfc_multi_t *hc, int ch, int protocol, int slot_tx, int bank_tx, int slot_rx, int bank_rx)
{
	int flow_tx = 0, flow_rx = 0, routing = 0;
	int oslot_tx = hc->chan[ch].slot_tx;
	int oslot_rx = hc->chan[ch].slot_rx;
	int conf = hc->chan[ch].conf;

	if (debug & DEBUG_HFCMULTI_MODE)
		printk(KERN_DEBUG "%s: channel %d protocol %x slot %d bank %d (TX) slot %d bank %d (RX)\n", __FUNCTION__, ch, protocol, slot_tx, bank_tx, slot_rx, bank_rx);

	if (oslot_tx>=0 && slot_tx!=oslot_tx) {
		/* remove from slot */
		if (debug & DEBUG_HFCMULTI_MODE)
			printk(KERN_DEBUG "%s: remove from slot %d (TX)\n", __FUNCTION__, oslot_tx);
		if (hc->slot_owner[oslot_tx<<1] == ch) {
			HFC_outb(hc, R_SLOT, oslot_tx<<1);
			HFC_outb(hc, A_SL_CFG, 0);
			HFC_outb(hc, A_CONF, 0);
			hc->slot_owner[oslot_tx<<1] = -1;
		} else {
			if (debug & DEBUG_HFCMULTI_MODE)
				printk(KERN_DEBUG "%s: we are not owner of this slot anymore, channel %d is.\n", __FUNCTION__, hc->slot_owner[oslot_tx<<1]);
		}
	}

	if (oslot_rx>=0 && slot_rx!=oslot_rx) {
		/* remove from slot */
		if (debug & DEBUG_HFCMULTI_MODE)
			printk(KERN_DEBUG "%s: remove from slot %d (RX)\n", __FUNCTION__, oslot_rx);
		if (hc->slot_owner[(oslot_rx<<1)|1] == ch) {
			HFC_outb(hc, R_SLOT, (oslot_rx<<1) | V_SL_DIR);
			HFC_outb(hc, A_SL_CFG, 0);
			hc->slot_owner[(oslot_rx<<1)|1] = -1;
		} else {
			if (debug & DEBUG_HFCMULTI_MODE)
				printk(KERN_DEBUG "%s: we are not owner of this slot anymore, channel %d is.\n", __FUNCTION__, hc->slot_owner[(oslot_rx<<1)|1]);
		}
	}

	if (slot_tx < 0) {
		flow_tx = 0x80; /* FIFO->ST */
		/* disable pcm slot */
		hc->chan[ch].slot_tx = -1;
		hc->chan[ch].bank_tx = 0;
	} else {
		/* set pcm slot */
		if (hc->chan[ch].txpending)
			flow_tx = 0x80; /* FIFO->ST */
		else
			flow_tx = 0xc0; /* PCM->ST */
		/* put on slot */
		routing = bank_tx?0xc0:0x80;
		if (conf>=0 || bank_tx>1)
			routing = 0x40; /* loop */
		if (debug & DEBUG_HFCMULTI_MODE)
			printk(KERN_DEBUG "%s: put to slot %d bank %d flow %02x routing %02x conf %d (TX)\n", __FUNCTION__, slot_tx, bank_tx, flow_tx, routing, conf);
		HFC_outb(hc, R_SLOT, slot_tx<<1);
		HFC_outb(hc, A_SL_CFG, (ch<<1) | routing);
		HFC_outb(hc, A_CONF, (conf<0)?0:(conf|V_CONF_SL));
		hc->slot_owner[slot_tx<<1] = ch;
		hc->chan[ch].slot_tx = slot_tx;
		hc->chan[ch].bank_tx = bank_tx;
	}
	if (slot_rx < 0) {
		/* disable pcm slot */
		flow_rx = 0x80; /* ST->FIFO */
		hc->chan[ch].slot_rx = -1;
		hc->chan[ch].bank_rx = 0;
	} else {
		/* set pcm slot */
		if (hc->chan[ch].txpending)
			flow_rx = 0x80; /* ST->FIFO */
		else
			flow_rx = 0xc0; /* ST->(FIFO,PCM) */
		/* put on slot */
		routing = bank_rx?0x80:0xc0; /* reversed */
		if (conf>=0 || bank_rx>1)
			routing = 0x40; /* loop */
		if (debug & DEBUG_HFCMULTI_MODE)
			printk(KERN_DEBUG "%s: put to slot %d bank %d flow %02x routing %02x conf %d (RX)\n", __FUNCTION__, slot_rx, bank_rx, flow_rx, routing, conf);
		HFC_outb(hc, R_SLOT, (slot_rx<<1) | V_SL_DIR);
		HFC_outb(hc, A_SL_CFG, (ch<<1) | V_CH_DIR | routing);
		hc->slot_owner[(slot_rx<<1)|1] = ch;
		hc->chan[ch].slot_rx = slot_rx;
		hc->chan[ch].bank_rx = bank_rx;
	}

	switch (protocol) {
		case (ISDN_PID_NONE):
		/* disable TX fifo */
		HFC_outb(hc, R_FIFO, ch<<1);
		HFC_wait(hc);
		HFC_outb(hc, A_CON_HDLC, flow_tx | 0x00 | V_IFF);
		HFC_outb(hc, A_SUBCH_CFG, 0);
		HFC_outb(hc, A_IRQ_MSK, 0);
		HFC_outb(hc, R_INC_RES_FIFO, V_RES_F);
		HFC_wait(hc);
		/* disable RX fifo */
		HFC_outb(hc, R_FIFO, (ch<<1)|1);
		HFC_wait(hc);
		HFC_outb(hc, A_CON_HDLC, flow_rx | 0x00);
		HFC_outb(hc, A_SUBCH_CFG, 0);
		HFC_outb(hc, A_IRQ_MSK, 0);
		HFC_outb(hc, R_INC_RES_FIFO, V_RES_F);
		HFC_wait(hc);
		if (hc->type == 1) {
		} else if ((ch&0x3)<2) {
			hc->hw.a_st_ctrl0[hc->chan[ch].port] &= ((ch&0x3)==0)?~V_B1_EN:~V_B2_EN;
			HFC_outb(hc, R_ST_SEL, hc->chan[ch].port);
			HFC_outb(hc, A_ST_CTRL0,  hc->hw.a_st_ctrl0[hc->chan[ch].port]);
		}
		break;

		case (ISDN_PID_L1_B_64TRANS): /* B-channel */
		/* enable TX fifo */
		HFC_outb(hc, R_FIFO, ch<<1);
		HFC_wait(hc);
		HFC_outb(hc, A_CON_HDLC, flow_tx | 0x00 | V_HDLC_TRP | V_IFF);
		HFC_outb(hc, A_SUBCH_CFG, 0);
		HFC_outb(hc, A_IRQ_MSK, 0);
		HFC_outb(hc, R_INC_RES_FIFO, V_RES_F);
		HFC_wait(hc);
		HFC_outb_(hc, A_FIFO_DATA0_NOINC, silence); /* tx silence */
		/* enable RX fifo */
		HFC_outb(hc, R_FIFO, (ch<<1)|1);
		HFC_wait(hc);
		HFC_outb(hc, A_CON_HDLC, flow_rx | 0x00 | V_HDLC_TRP);
		HFC_outb(hc, A_SUBCH_CFG, 0);
		HFC_outb(hc, A_IRQ_MSK, 0);
		HFC_outb(hc, R_INC_RES_FIFO, V_RES_F);
		HFC_wait(hc);
		if (hc->type != 1) {
			hc->hw.a_st_ctrl0[hc->chan[ch].port] |= ((ch&0x3)==0)?V_B1_EN:V_B2_EN;
			HFC_outb(hc, R_ST_SEL, hc->chan[ch].port);
			HFC_outb(hc, A_ST_CTRL0,  hc->hw.a_st_ctrl0[hc->chan[ch].port]);
		}
		break;

		case (ISDN_PID_L1_B_64HDLC): /* B-channel */
		case (ISDN_PID_L1_TE_E1): /* D-channel E1 */
		case (ISDN_PID_L1_NT_E1):
		/* enable TX fifo */
		HFC_outb(hc, R_FIFO, ch<<1);
		HFC_wait(hc);
		HFC_outb(hc, A_CON_HDLC, flow_tx | 0x04);
		HFC_outb(hc, A_SUBCH_CFG, 0);
		HFC_outb(hc, A_IRQ_MSK, V_IRQ);
		HFC_outb(hc, R_INC_RES_FIFO, V_RES_F);
		HFC_wait(hc);
		/* enable RX fifo */
		HFC_outb(hc, R_FIFO, (ch<<1)|1);
		HFC_wait(hc);
		HFC_outb(hc, A_CON_HDLC, flow_rx | 0x04);
		HFC_outb(hc, A_SUBCH_CFG, 0);
		HFC_outb(hc, A_IRQ_MSK, V_IRQ);
		HFC_outb(hc, R_INC_RES_FIFO, V_RES_F);
		HFC_wait(hc);
		if (hc->type == 1) {
		} else {
			hc->hw.a_st_ctrl0[hc->chan[ch].port] |= ((ch&0x3)==0)?V_B1_EN:V_B2_EN;
			HFC_outb(hc, R_ST_SEL, hc->chan[ch].port);
			HFC_outb(hc, A_ST_CTRL0,  hc->hw.a_st_ctrl0[hc->chan[ch].port]);
		}
		break;

		case (ISDN_PID_L1_TE_S0): /* D-channel S0 */
		case (ISDN_PID_L1_NT_S0):
		/* enable TX fifo */
		HFC_outb(hc, R_FIFO, ch<<1);
		HFC_wait(hc);
		HFC_outb(hc, A_CON_HDLC, flow_tx | 0x04 | V_IFF);
		HFC_outb(hc, A_SUBCH_CFG, 2);
		HFC_outb(hc, A_IRQ_MSK, V_IRQ);
		HFC_outb(hc, R_INC_RES_FIFO, V_RES_F);
		HFC_wait(hc);
		/* enable RX fifo */
		HFC_outb(hc, R_FIFO, (ch<<1)|1);
		HFC_wait(hc);
		HFC_outb(hc, A_CON_HDLC, flow_rx | 0x04);
		HFC_outb(hc, A_SUBCH_CFG, 2);
		HFC_outb(hc, A_IRQ_MSK, V_IRQ);
		HFC_outb(hc, R_INC_RES_FIFO, V_RES_F);
		HFC_wait(hc);
		break;

		default:
		printk(KERN_DEBUG "%s: protocol not known %x\n", __FUNCTION__, protocol);
		hc->chan[ch].protocol = ISDN_PID_NONE;
		return(-ENOPROTOOPT);
	}
	hc->chan[ch].protocol = protocol;
	return(0);
}


/**************************/
/* connect/disconnect PCM */
/**************************/

static void
hfcmulti_pcm(hfc_multi_t *hc, int ch, int slot_tx, int bank_tx, int slot_rx, int bank_rx)
{
	if (slot_rx<0 || slot_rx<0 || bank_tx<0 || bank_rx<0) {
		/* disable PCM */
		mode_hfcmulti(hc, ch, hc->chan[ch].protocol, -1, 0, -1, 0);
		return;
	}

	/* enable pcm */
	mode_hfcmulti(hc, ch, hc->chan[ch].protocol, slot_tx, bank_tx, slot_rx, bank_rx);
}


/**************************/
/* set/disable conference */
/**************************/

static void
hfcmulti_conf(hfc_multi_t *hc, int ch, int num)
{
	if (num>=0 && num<=7)
		hc->chan[ch].conf = num;
	else
		hc->chan[ch].conf = -1;
	mode_hfcmulti(hc, ch, hc->chan[ch].protocol, hc->chan[ch].slot_tx, hc->chan[ch].bank_tx, hc->chan[ch].slot_rx, hc->chan[ch].bank_rx);
}


/***************************/
/* set/disable sample loop */
/***************************/
// NOTE: this function is experimental and therefore disabled
static void
hfcmulti_splloop(hfc_multi_t *hc, int ch, u_char *data, int len)
{
	bchannel_t *bch = hc->chan[ch].bch;

	/* flush pending TX data */
	if (bch->next_skb) {
		test_and_clear_bit(BC_FLG_TX_NEXT, &bch->Flag);
		dev_kfree_skb(bch->next_skb);
		bch->next_skb = NULL;
	}
	bch->tx_idx = bch->tx_len = bch->rx_idx = 0;

	/* prevent overflow */
	if (len > hc->Zlen-1)
		len = hc->Zlen-1;

	/* select fifo */
	HFC_outb_(hc, R_FIFO, ch<<1);
	HFC_wait_(hc);

	/* reset fifo */
	HFC_outb(hc, A_SUBCH_CFG, 0);
	udelay(500);
	HFC_outb_(hc, R_INC_RES_FIFO, V_RES_F);
	HFC_wait_(hc);
	udelay(500);

	/* if off */
	if (len <= 0) {
		HFC_outb_(hc, A_FIFO_DATA0_NOINC, silence);
		if (hc->chan[ch].slot_tx>=0) {
			if (debug & DEBUG_HFCMULTI_MODE)
				printk(KERN_DEBUG "%s: connecting PCM due to no more TONE: channel %d slot_tx %d\n", __FUNCTION__, ch, hc->chan[ch].slot_tx);
			/* connect slot */
			HFC_outb(hc, A_CON_HDLC, 0xc0 | 0x00 | V_HDLC_TRP | V_IFF);
			HFC_outb(hc, R_FIFO, ch<<1 | 1);
			HFC_wait(hc);
			HFC_outb(hc, A_CON_HDLC, 0xc0 | 0x00 | V_HDLC_TRP | V_IFF);
		}
		hc->chan[ch].txpending = 0;
		return;
	}

	/* loop fifo */

	/* set mode */
	hc->chan[ch].txpending = 2;

//printk("len=%d %02x %02x %02x\n", len, data[0], data[1], data[2]);
	/* write loop data */
	write_fifo_data(hc,data,len);

	udelay(500);
	HFC_outb(hc, A_SUBCH_CFG, V_LOOP_FIFO);
	udelay(500);

	/* disconnect slot */
	if (hc->chan[ch].slot_tx>=0) {
		if (debug & DEBUG_HFCMULTI_MODE)
			printk(KERN_DEBUG "%s: disconnecting PCM due to TONE: channel %d slot_tx %d\n", __FUNCTION__, ch, hc->chan[ch].slot_tx);
		HFC_outb(hc, A_CON_HDLC, 0x80 | 0x00 | V_HDLC_TRP | V_IFF);
		HFC_outb(hc, R_FIFO, ch<<1 | 1);
		HFC_wait(hc);
		HFC_outb(hc, A_CON_HDLC, 0x80 | 0x00 | V_HDLC_TRP | V_IFF);
		HFC_outb(hc, R_FIFO, ch<<1);
		HFC_wait(hc);
	} else {
		/* change fifo */
		HFC_outb(hc, R_FIFO, ch<<1);
		HFC_wait(hc);
	}

//udelay(300);
}

/*************************************/
/* Layer 1 D-channel hardware access */
/*************************************/

/* message transfer from layer 1 to hardware.
 */
static int hfcmulti_l1hw(mISDNinstance_t *inst, struct sk_buff *skb)
{
	dchannel_t	*dch = container_of(inst, dchannel_t, inst);
	int		slot_tx, slot_rx, bank_tx, bank_rx;
	hfc_multi_t	*hc;
	int		ret = 0;
	mISDN_head_t	*hh;
	u_long		flags;

	hh = mISDN_HEAD_P(skb);
	hc = dch->inst.privat;
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
			printk(KERN_WARNING "%s: next_skb exist ERROR (skb->len=%d next_skb->len=%d)\n",
				__FUNCTION__, skb->len, dch->next_skb->len);
			spin_unlock_irqrestore(inst->hwlock, flags);
			return(-EBUSY);
		}
		if (test_and_set_bit(FLG_TX_BUSY, &dch->DFlags)) {
			test_and_set_bit(FLG_TX_NEXT, &dch->DFlags);
			dch->next_skb = skb;
			spin_unlock_irqrestore(inst->hwlock, flags);
			return(0);
		}
		/* write to fifo */
		dch->tx_len = skb->len;
		memcpy(dch->tx_buf, skb->data, dch->tx_len);
//		if (debug & DEBUG_HFCMULTI_FIFO) {
//			printk(KERN_DEBUG "%s:from user space:\n", __FUNCTION__);
//			i = 0;
//			while(i < dch->tx_len)
//				printk(" %02x", dch->tx_buf[i++]);
//			printk("\n");
//		}
		dch->tx_idx = 0;
		hfcmulti_tx(hc, dch->channel, dch, NULL);
		/* start fifo */
		HFC_outb(hc, R_FIFO, 0);
		HFC_wait(hc);
		spin_unlock_irqrestore(inst->hwlock, flags);
		skb_trim(skb, 0);
		return(mISDN_queueup_newhead(inst, 0, PH_DATA_CNF,hh->dinfo, skb));
	} else
	if (hh->prim == (PH_SIGNAL | REQUEST)) {
		spin_lock_irqsave(inst->hwlock, flags);
		switch (hh->dinfo) {
			case INFO3_P8:
			case INFO3_P10:
			break;
			default:
			printk(KERN_DEBUG "%s: unknown PH_SIGNAL info %x\n", __FUNCTION__, hh->dinfo);
			ret = -EINVAL;
		}
		spin_unlock_irqrestore(inst->hwlock, flags);
	} else if (hh->prim == (PH_CONTROL | REQUEST)) {
		spin_lock_irqsave(inst->hwlock, flags);
		switch (hh->dinfo) {
			case HW_RESET:
			/* start activation */
			if (hc->type == 1) {
				HFC_outb(hc, R_E1_WR_STA, V_E1_LD_STA | 0);
				udelay(6); /* wait at least 5,21us */
				HFC_outb(hc, R_E1_WR_STA, 0);
			} else {
				HFC_outb(hc, R_ST_SEL, hc->chan[dch->channel].port);
				HFC_outb(hc, A_ST_WR_STATE, V_ST_LD_STA | 3); /* G1 */
				udelay(6); /* wait at least 5,21us */
				HFC_outb(hc, A_ST_WR_STATE, 3);
				HFC_outb(hc, A_ST_WR_STATE, 3 | (V_ST_ACT*3)); /* activate */
			}
			spin_unlock_irqrestore(inst->hwlock, flags);
			skb_trim(skb, 0);
			return(mISDN_queueup_newhead(inst, 0, PH_CONTROL | INDICATION,HW_POWERUP, skb));
			break;

			case HW_DEACTIVATE:
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: HW_DEACTIVATE\n", __FUNCTION__);
			goto hw_deactivate; /* after lock */

			/* connect interface to pcm timeslot (0..N) */
			case HW_PCM_CONN:
			if (skb->len < 4*sizeof(u_long)) {
				printk(KERN_WARNING "%s: HW_PCM_CONN lacks parameters\n", __FUNCTION__);
				break;
			}
			slot_tx = ((int *)skb->data)[0];
			bank_tx = ((int *)skb->data)[1];
			slot_rx = ((int *)skb->data)[2];
			bank_rx = ((int *)skb->data)[3];
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: HW_PCM_CONN slot %d bank %d (TX) slot %d bank %d (RX)\n", __FUNCTION__, slot_tx, bank_tx, slot_rx, bank_rx);
			if (slot_tx<=hc->slots && bank_tx<=2 && slot_rx<=hc->slots && bank_rx<=2)
				hfcmulti_pcm(hc, dch->channel, slot_tx, bank_tx, slot_rx, bank_rx);
			else
				printk(KERN_WARNING "%s: HW_PCM_CONN slot %d bank %d (TX) slot %d bank %d (RX) out of range\n", __FUNCTION__, slot_tx, bank_tx, slot_rx, bank_rx);
			break;

			/* release interface from pcm timeslot */
			case HW_PCM_DISC:
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: HW_PCM_DISC\n", __FUNCTION__);
			hfcmulti_pcm(hc, dch->channel, -1, -1, -1, -1);
			break;
			case HW_POWERUP:
				printk(KERN_INFO "HW_POWERUP\n");
				HFC_outb(hc, R_ST_SEL, hc->chan[dch->channel].port);
				HFC_outb(hc, A_ST_WR_STATE, 3 | 0x10); /* activate */
				udelay(6); /* wait at least 5,21us */
				HFC_outb(hc, A_ST_WR_STATE, 3); /* activate */
			break;

			default:
			printk(KERN_DEBUG "%s: unknown PH_CONTROL info %x\n", __FUNCTION__, hh->dinfo);
			ret = -EINVAL;
		}
		spin_unlock_irqrestore(inst->hwlock, flags);
	} else if (hh->prim == (PH_ACTIVATE | REQUEST)) {
		if (test_bit(HFC_CFG_NTMODE, &hc->chan[dch->channel].cfg)) {
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: PH_ACTIVATE port %d (0..%d)\n", __FUNCTION__, hc->chan[dch->channel].port, hc->type-1);
			spin_lock_irqsave(inst->hwlock, flags);
			/* start activation */
			if (hc->type == 1) {
				//dchannel_sched_event(dch, D_L1STATECHANGE);
				ph_state_change(dch);
				if (debug & DEBUG_HFCMULTI_STATE)
					printk(KERN_DEBUG "%s: E1 report state %x \n", __FUNCTION__, dch->ph_state);
			} else {
				HFC_outb(hc, R_ST_SEL, hc->chan[dch->channel].port);
				HFC_outb(hc, A_ST_WR_STATE, V_ST_LD_STA | 1); /* G1 */
				udelay(6); /* wait at least 5,21us */
				HFC_outb(hc, A_ST_WR_STATE, 1);
				HFC_outb(hc, A_ST_WR_STATE, 1 | (V_ST_ACT*3)); /* activate */
				dch->ph_state = 1;
			}
			spin_unlock_irqrestore(inst->hwlock, flags);
		} else {
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: PH_ACTIVATE no NT-mode port %d (0..%d)\n", __FUNCTION__, hc->chan[dch->channel].port, hc->type-1);
			ret = -EINVAL;
		}
	} else if (hh->prim == (PH_DEACTIVATE | REQUEST)) {
		if (test_bit(HFC_CFG_NTMODE, &hc->chan[dch->channel].cfg)) {
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: PH_DEACTIVATE port %d (0..%d)\n", __FUNCTION__, hc->chan[dch->channel].port, hc->type-1);
			spin_lock_irqsave(inst->hwlock, flags);
			hw_deactivate: /* after lock */
			dch->ph_state = 0;
			/* start deactivation */
			if (hc->type == 1) {
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: PH_DEACTIVATE no BRI\n", __FUNCTION__);
			} else {
				HFC_outb(hc, R_ST_SEL, hc->chan[dch->channel].port);
				HFC_outb(hc, A_ST_WR_STATE, V_ST_ACT*2); /* deactivate */
			}
			if (dch->next_skb) {
				dev_kfree_skb(dch->next_skb);
				dch->next_skb = NULL;
			}
			dch->tx_idx = dch->tx_len = hc->chan[dch->channel].rx_idx = 0;
			test_and_clear_bit(FLG_TX_NEXT, &dch->DFlags);
			test_and_clear_bit(FLG_TX_BUSY, &dch->DFlags);
			if (test_and_clear_bit(FLG_DBUSY_TIMER, &dch->DFlags))
				del_timer(&dch->dbusytimer);
			spin_unlock_irqrestore(inst->hwlock, flags);
		} else {
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: PH_DEACTIVATE no NT-mode port %d (0..%d)\n", __FUNCTION__, hc->chan[dch->channel].port, hc->type-1);
			ret = -EINVAL;
		}
	} else {
		if (debug & DEBUG_HFCMULTI_MSG)
			printk(KERN_DEBUG "%s: unknown prim %x\n", __FUNCTION__, hh->prim);
		ret = -EINVAL;
	}
	if (!ret) {
//		printk("1\n");
		dev_kfree_skb(skb);
//		printk("2\n");
	}
	return(ret);
}


/******************************/
/* Layer2 -> Layer 1 Transfer */
/******************************/

/* messages from layer 2 to layer 1 are processed here.
 */
static int
hfcmulti_l2l1(mISDNinstance_t *inst, struct sk_buff *skb)
{
	u_long		flags, num;
	int		slot_tx, slot_rx, bank_tx, bank_rx;
	bchannel_t	*bch = container_of(inst, bchannel_t, inst);
	int		ret = -EINVAL;
	mISDN_head_t	*hh;
	hfc_multi_t	*hc;
	struct		dsp_features *features;

	hh = mISDN_HEAD_P(skb);
	hc = bch->inst.privat;

	if ((hh->prim == PH_DATA_REQ)
	 || (hh->prim == (DL_DATA | REQUEST))) {
		if (skb->len <= 0) {
			printk(KERN_WARNING "%s: skb too small\n", __FUNCTION__);
			return(-EINVAL);
		}
		if (skb->len > MAX_DATA_MEM) {
			printk(KERN_WARNING "%s: skb too large\n", __FUNCTION__);
			return(-EINVAL);
		}
		/* check for pending next_skb */
		spin_lock_irqsave(inst->hwlock, flags);
		if (bch->next_skb) {
			printk(KERN_WARNING "%s: next_skb exist ERROR (skb->len=%d next_skb->len=%d)\n", __FUNCTION__, skb->len, bch->next_skb->len);
			spin_unlock_irqrestore(inst->hwlock, flags);
			return(-EBUSY);
		}
		/* if we have currently a pending tx skb */
		if (test_and_set_bit(BC_FLG_TX_BUSY, &bch->Flag)) {
			test_and_set_bit(BC_FLG_TX_NEXT, &bch->Flag);
			bch->next_skb = skb;
			spin_unlock_irqrestore(inst->hwlock, flags);
			return(0);
		}
		/* write to fifo */
		bch->tx_len = skb->len;
		memcpy(bch->tx_buf, skb->data, bch->tx_len);
		bch->tx_idx = 0;
		hfcmulti_tx(hc, bch->channel, NULL, bch);
		/* start fifo */
		HFC_outb_(hc, R_FIFO, 0);
		HFC_wait_(hc);
		spin_unlock_irqrestore(inst->hwlock, flags);
#ifdef FIXME   // TODO changed
		if ((bch->inst.pid.protocol[2] == ISDN_PID_L2_B_RAWDEV)
			&& bch->dev)
			hif = &bch->dev->rport.pif;
		else
			hif = &bch->inst.up;
#endif
		skb_trim(skb, 0);
		return(mISDN_queueup_newhead(inst, 0, hh->prim | CONFIRM, hh->dinfo, skb));
	} else if ((hh->prim == (PH_ACTIVATE | REQUEST))
	 || (hh->prim == (DL_ESTABLISH  | REQUEST))) {
		/* activate B-channel if not already activated */
		if (debug & DEBUG_HFCMULTI_MSG)
			printk(KERN_DEBUG "%s: PH_ACTIVATE ch %d (0..32)\n", __FUNCTION__, bch->channel);
		if (test_and_set_bit(BC_FLG_ACTIV, &bch->Flag))
			ret = 0;
		else {
			spin_lock_irqsave(inst->hwlock, flags);
			ret = mode_hfcmulti(hc, bch->channel, bch->inst.pid.protocol[1], hc->chan[bch->channel].slot_tx, hc->chan[bch->channel].bank_tx, hc->chan[bch->channel].slot_rx, hc->chan[bch->channel].bank_rx);
			if (!ret) {
				bch->protocol = bch->inst.pid.protocol[1];
				if (bch->protocol==ISDN_PID_L1_B_64TRANS && !hc->dtmf) {
					/* start decoder */
					hc->dtmf = 1;
					if (debug & DEBUG_HFCMULTI_DTMF)
						printk(KERN_DEBUG "%s: start dtmf decoder\n", __FUNCTION__);
					HFC_outb(hc, R_DTMF, hc->hw.r_dtmf | V_RST_DTMF);
				}
			}
			spin_unlock_irqrestore(inst->hwlock, flags);
		}
#ifdef FIXME  // TODO changed
		if (bch->inst.pid.protocol[2] == ISDN_PID_L2_B_RAWDEV)
			if (bch->dev)
				if_link(&bch->dev->rport.pif, hh->prim | CONFIRM, 0, 0, NULL, 0);
#endif
		skb_trim(skb, 0);
		return(mISDN_queueup_newhead(inst, 0, hh->prim | CONFIRM, ret, skb));
	} else if ((hh->prim == (PH_DEACTIVATE | REQUEST))
	 || (hh->prim == (DL_RELEASE | REQUEST))
	 || ((hh->prim == (PH_CONTROL | REQUEST) && (hh->dinfo == HW_DEACTIVATE)))) {
		if (debug & DEBUG_HFCMULTI_MSG)
			printk(KERN_DEBUG "%s: PH_DEACTIVATE ch %d (0..32)\n", __FUNCTION__, bch->channel);
		/* deactivate B-channel if not already deactivated */
		spin_lock_irqsave(inst->hwlock, flags);
		if (bch->next_skb) {
			test_and_clear_bit(BC_FLG_TX_NEXT, &bch->Flag);
			dev_kfree_skb(bch->next_skb);
			bch->next_skb = NULL;
		}
		bch->tx_idx = bch->tx_len = bch->rx_idx = 0;
		test_and_clear_bit(BC_FLG_TX_BUSY, &bch->Flag);
		hc->chan[bch->channel].slot_tx = -1;
		hc->chan[bch->channel].slot_rx = -1;
		hc->chan[bch->channel].conf = -1;
		mode_hfcmulti(hc, bch->channel, ISDN_PID_NONE, hc->chan[bch->channel].slot_tx, hc->chan[bch->channel].bank_tx, hc->chan[bch->channel].slot_rx, hc->chan[bch->channel].bank_rx);
		bch->protocol = ISDN_PID_NONE;
		test_and_clear_bit(BC_FLG_ACTIV, &bch->Flag);
		spin_unlock_irqrestore(inst->hwlock, flags);
		skb_trim(skb, 0);
//printk("5\n");
		if (hh->prim != (PH_CONTROL | REQUEST)) {
#ifdef FIXME  // TODO changed
			if (bch->inst.pid.protocol[2] == ISDN_PID_L2_B_RAWDEV)
				if (bch->dev)
					if_link(&bch->dev->rport.pif, hh->prim | CONFIRM, 0, 0, NULL, 0);
#endif
			return(mISDN_queueup_newhead(inst, 0, hh->prim | CONFIRM, ret, skb));
//printk("6\n");
		}
//printk("7\n");
		ret = 0;
	} else
	if (hh->prim == (PH_CONTROL | REQUEST)) {
		spin_lock_irqsave(inst->hwlock, flags);
		switch (hh->dinfo) {
			/* fill features structure */
			case HW_FEATURES:
			if (skb->len != sizeof(void *)) {
				printk(KERN_WARNING "%s: HW_FEATURES lacks parameters\n", __FUNCTION__);
				break;
			}
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: HW_FEATURE request\n", __FUNCTION__);
			features = *((struct dsp_features **)skb->data);
			features->hfc_id = hc->id;
			if (test_bit(HFC_CHIP_DTMF, &hc->chip))
				features->hfc_dtmf = 1;
			features->hfc_loops = 0;
			features->pcm_id = hc->pcm;
			features->pcm_slots = hc->slots;
			features->pcm_banks = 2;
			ret = 0;
			break;

			/* connect interface to pcm timeslot (0..N) */
			case HW_PCM_CONN:
			if (skb->len < 4*sizeof(u_long)) {
				printk(KERN_WARNING "%s: HW_PCM_CONN lacks parameters\n", __FUNCTION__);
				break;
			}
			slot_tx = ((int *)skb->data)[0];
			bank_tx = ((int *)skb->data)[1];
			slot_rx = ((int *)skb->data)[2];
			bank_rx = ((int *)skb->data)[3];
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: HW_PCM_CONN slot %d bank %d (TX) slot %d bank %d (RX)\n", __FUNCTION__, slot_tx, bank_tx, slot_rx, bank_rx);
			if (slot_tx<=hc->slots && bank_tx<=2 && slot_rx<=hc->slots && bank_rx<=2)
				hfcmulti_pcm(hc, bch->channel, slot_tx, bank_tx, slot_rx, bank_rx);
			else
				printk(KERN_WARNING "%s: HW_PCM_CONN slot %d bank %d (TX) slot %d bank %d (RX) out of range\n", __FUNCTION__, slot_tx, bank_tx, slot_rx, bank_rx);
			ret = 0;
			break;

			/* release interface from pcm timeslot */
			case HW_PCM_DISC:
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: HW_PCM_DISC\n", __FUNCTION__);
			hfcmulti_pcm(hc, bch->channel, -1, -1, -1, -1);
			ret = 0;
			break;

			/* join conference (0..7) */
			case HW_CONF_JOIN:
			if (skb->len < sizeof(u_long)) {
				printk(KERN_WARNING "%s: HW_CONF_JOIN lacks parameters\n", __FUNCTION__);
				break;
			}
			num = ((u_long *)skb->data)[0];
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: HW_CONF_JOIN conf %ld\n", __FUNCTION__, num);
			if (num <= 7) {
				hfcmulti_conf(hc, bch->channel, num);
				ret = 0;
			} else
				printk(KERN_WARNING "%s: HW_CONF_JOIN conf %ld out of range\n", __FUNCTION__, num);
			break;

			/* split conference */
			case HW_CONF_SPLIT:
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: HW_CONF_SPLIT\n", __FUNCTION__);
			hfcmulti_conf(hc, bch->channel, -1);
			ret = 0;
			break;

			/* set sample loop */
			case HW_SPL_LOOP_ON:
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: HW_SPL_LOOP_ON (len = %d)\n", __FUNCTION__, skb->len);
			hfcmulti_splloop(hc, bch->channel, skb->data, skb->len);
			ret = 0;
			break;

			/* set silence */
			case HW_SPL_LOOP_OFF:
			if (debug & DEBUG_HFCMULTI_MSG)
				printk(KERN_DEBUG "%s: HW_SPL_LOOP_OFF\n", __FUNCTION__);
			hfcmulti_splloop(hc, bch->channel, NULL, 0);
			ret = 0;
			break;

			default:
			printk(KERN_DEBUG "%s: unknown PH_CONTROL info %x\n", __FUNCTION__, hh->dinfo);
			ret = -EINVAL;
		}
		spin_unlock_irqrestore(inst->hwlock, flags);
	} else {
		printk(KERN_WARNING "%s: unknown prim(%x)\n", __FUNCTION__, hh->prim);
		ret = -EINVAL;
	}
	if (!ret) {
//		printk("3\n");
		dev_kfree_skb(skb);
//		printk("4\n");
	}
	return(ret);
}


/***************************/
/* handle D-channel events */
/***************************/

/* handle state change event
 */
static void ph_state_change(dchannel_t *dch)
{
	hfc_multi_t *hc = dch->inst.privat;
	u_int prim = PH_SIGNAL | INDICATION;
	u_int para = 0;
	int ch;

	if (!dch) {
		printk(KERN_WARNING "%s: ERROR given dch is NULL\n", __FUNCTION__);
		return;
	}
	ch = dch->channel;

	if (hc->type == 1) {
		if (!test_bit(HFC_CFG_NTMODE, &hc->chan[ch].cfg)) {
			if (debug & DEBUG_HFCMULTI_STATE)
				printk(KERN_DEBUG "%s: E1 TE newstate %x\n", __FUNCTION__, dch->ph_state);
		} else {
			if (debug & DEBUG_HFCMULTI_STATE)
				printk(KERN_DEBUG "%s: E1 NT newstate %x\n", __FUNCTION__, dch->ph_state);
		}
		switch (dch->ph_state) {
			case (1):
			prim = PH_ACTIVATE | INDICATION;
			para = 0;
			break;

			default:
			if (hc->chan[ch].e1_state != 1)
				return;
			prim = PH_DEACTIVATE | INDICATION;
			para = 0;
		}
		hc->chan[ch].e1_state = dch->ph_state;
	} else {
		if (!test_bit(HFC_CFG_NTMODE, &hc->chan[ch].cfg)) {
			if (debug & DEBUG_HFCMULTI_STATE)
				printk(KERN_DEBUG "%s: S/T TE newstate %x\n", __FUNCTION__, dch->ph_state);
			switch (dch->ph_state) {
				case (0):
				prim = PH_CONTROL | INDICATION;
				para = HW_RESET;
				break;

				case (3):
				prim = PH_CONTROL | INDICATION;
				para = HW_DEACTIVATE;
				break;

				case (5):
				case (8):
				para = ANYSIGNAL;
				break;

				case (6):
				para = INFO2;
				break;

				case (7):
				para = INFO4_P8;
				break;

				default:
				return;
			}
		} else {
			if (debug & DEBUG_HFCMULTI_STATE)
				printk(KERN_DEBUG "%s: S/T NT newstate %x\n", __FUNCTION__, dch->ph_state);
			switch (dch->ph_state) {
				case (2):
				if (hc->chan[ch].nt_timer == 0) {
					hc->chan[ch].nt_timer = -1;
					HFC_outb(hc, R_ST_SEL, hc->chan[ch].port);
					HFC_outb(hc, A_ST_WR_STATE, 4 | V_ST_LD_STA); /* G4 */
					udelay(6); /* wait at least 5,21us */
					HFC_outb(hc, A_ST_WR_STATE, 4);
					dch->ph_state = 4;
				} else {
					/* one extra count for the next event */
					hc->chan[ch].nt_timer = nt_t1_count[poll_timer] + 1;
					HFC_outb(hc, R_ST_SEL, hc->chan[ch].port);
					HFC_outb(hc, A_ST_WR_STATE, 2 | V_SET_G2_G3); /* allow G2 -> G3 transition */
				}
				return;

				case (1):
				prim = PH_DEACTIVATE | INDICATION;
				para = 0;
				hc->chan[ch].nt_timer = -1;
				break;

				case (4):
				hc->chan[ch].nt_timer = -1;
				return;

				case (3):
				prim = PH_ACTIVATE | INDICATION;
				para = 0;
				hc->chan[ch].nt_timer = -1;
				break;

				default:
				break;
			}
		}
	}

	mISDN_queue_data(&dch->inst, FLG_MSG_UP, prim, para, 0, NULL, 0);
}

/*************************************/
/* called for card mode init message */
/*************************************/

static void
hfcmulti_initmode(hfc_multi_t *hc)
{
	int		nt_mode;
	BYTE		r_sci_msk, a_st_wr_state, r_e1_wr_sta;
	int		i, port;
	dchannel_t	*dch;
	u_long		flags;

	if (debug & DEBUG_HFCMULTI_INIT)
		printk("%s: entered\n", __FUNCTION__);

	spin_lock_irqsave(&hc->lock, flags);
	if (hc->type == 1) {
		nt_mode = test_bit(HFC_CFG_NTMODE, &hc->chan[16].cfg);
		hc->chan[16].slot_tx = -1;
		hc->chan[16].slot_rx = -1;
		hc->chan[16].conf = -1;
		mode_hfcmulti(hc, 16, nt_mode?ISDN_PID_L1_NT_E1:ISDN_PID_L1_TE_E1, -1, 0, -1, 0);
		hc->chan[16].dch->dbusytimer.function = (void *) hfcmulti_dbusy_timer;
		hc->chan[16].dch->dbusytimer.data = (long) &hc->chan[16].dch;
		init_timer(&hc->chan[16].dch->dbusytimer);

		i = 0;
		while (i < 30) {
			hc->chan[i+1+(i>=15)].slot_tx = -1;
			hc->chan[i+1+(i>=15)].slot_rx = -1;
			hc->chan[i+1+(i>=15)].conf = -1;
			mode_hfcmulti(hc, i+1+(i>=15), ISDN_PID_NONE, -1, 0, -1, 0);
			hc->chan[i+1+(i>=15)].bch->protocol = ISDN_PID_NONE;
			i++;
		}
	} else {
		i = 0;
		while (i < hc->type) {
			nt_mode = test_bit(HFC_CFG_NTMODE, &hc->chan[(i<<2)+2].cfg);
			hc->chan[(i<<2)+2].slot_tx = -1;
			hc->chan[(i<<2)+2].slot_rx = -1;
			hc->chan[(i<<2)+2].conf = -1;
			mode_hfcmulti(hc, (i<<2)+2, nt_mode?ISDN_PID_L1_NT_S0:ISDN_PID_L1_TE_S0, -1, 0, -1, 0);
			hc->chan[(i<<2)+2].dch->dbusytimer.function = (void *) hfcmulti_dbusy_timer;
			hc->chan[(i<<2)+2].dch->dbusytimer.data = (long) &hc->chan[(i<<2)+2].dch;
			init_timer(&hc->chan[(i<<2)+2].dch->dbusytimer);

			hc->chan[i<<2].slot_tx = -1;
			hc->chan[i<<2].slot_rx = -1;
			hc->chan[i<<2].conf = -1;
			mode_hfcmulti(hc, i<<2, ISDN_PID_NONE, -1, 0, -1, 0);
			hc->chan[i<<2].bch->protocol = ISDN_PID_NONE;
			hc->chan[(i<<2)+1].slot_tx = -1;
			hc->chan[(i<<2)+1].slot_rx = -1;
			hc->chan[(i<<2)+1].conf = -1;
			mode_hfcmulti(hc, (i<<2)+1, ISDN_PID_NONE, -1, 0, -1, 0);
			hc->chan[(i<<2)+1].bch->protocol = ISDN_PID_NONE;
			i++;
		}
	}

	/* set up interface */
	if (hc->type != 1) {
		/* ST */
		r_sci_msk = 0;
		i = 0;
		while(i < 32) {
			if (!(dch = hc->chan[i].dch)) {
				i++;
				continue;
			}
			port = hc->chan[i].port;
			/* select interface */
			HFC_outb(hc, R_ST_SEL, port);
			if (test_bit(HFC_CFG_NTMODE, &hc->chan[i].cfg)) {
				if (debug & DEBUG_HFCMULTI_INIT)
					printk(KERN_DEBUG "%s: ST port %d is NT-mode\n", __FUNCTION__, port);
				/* clock delay */
				HFC_outb(hc, A_ST_CLK_DLY, CLKDEL_NT | 0x60);
				a_st_wr_state = 1; /* G1 */
				hc->hw.a_st_ctrl0[port] = V_ST_MD;
			} else {
				if (debug & DEBUG_HFCMULTI_INIT)
					printk(KERN_DEBUG "%s: ST port %d is TE-mode\n", __FUNCTION__, port);
				/* clock delay */
				HFC_outb(hc, A_ST_CLK_DLY, CLKDEL_TE);
				a_st_wr_state = 2; /* F2 */
				hc->hw.a_st_ctrl0[port] = 0;
			}
			if (!test_bit(HFC_CFG_NONCAP_TX, &hc->chan[i].cfg)) {
				hc->hw.a_st_ctrl0[port] |= V_TX_LI;
			}
			/* line setup */
			HFC_outb(hc, A_ST_CTRL0,  hc->hw.a_st_ctrl0[port]);
			/* disable E-channel */
			if (test_bit(HFC_CFG_NTMODE, &hc->chan[i].cfg)
			 || test_bit(HFC_CFG_DIS_ECHANNEL, &hc->chan[i].cfg))
				HFC_outb(hc, A_ST_CTRL1, V_E_IGNO);
			/* enable B-channel receive */
			HFC_outb(hc, A_ST_CTRL2,  V_B1_RX_EN | V_B2_RX_EN);
			/* state machine setup */
			HFC_outb(hc, A_ST_WR_STATE, a_st_wr_state | V_ST_LD_STA);
			udelay(6); /* wait at least 5,21us */
			HFC_outb(hc, A_ST_WR_STATE, a_st_wr_state);
			r_sci_msk |= 1 << port;
			i++;
		}
		/* state machine interrupts */
		HFC_outb(hc, R_SCI_MSK, r_sci_msk);
	} else {
		/* E1 */
		if (test_bit(HFC_CFG_REPORT_LOS, &hc->chan[16].cfg)) {
			HFC_outb(hc, R_LOS0, 255); /* 2 ms */
			HFC_outb(hc, R_LOS1, 255); /* 512 ms */
		}
		if (test_bit(HFC_CFG_OPTICAL, &hc->chan[16].cfg)) {
			HFC_outb(hc, R_RX0, 0);
			hc->hw.r_tx0 = 0 | V_OUT_EN;
		} else {
			HFC_outb(hc, R_RX0, 1);
			hc->hw.r_tx0 = 1 | V_OUT_EN;
		}
		hc->hw.r_tx1 = V_ATX | V_NTRI;
		HFC_outb(hc, R_TX0, hc->hw.r_tx0);
		HFC_outb(hc, R_TX1, hc->hw.r_tx1);
		HFC_outb(hc, R_TX_FR0, 0x00);
		HFC_outb(hc, R_TX_FR1, 0xf8);
		HFC_outb(hc, R_TX_FR2, V_TX_MF | V_TX_E | V_NEG_E);
		HFC_outb(hc, R_RX_FR0, V_AUTO_RESYNC | V_AUTO_RECO | 0);
		HFC_outb(hc, R_RX_FR1, V_RX_MF | V_RX_MF_SYNC);
		if (test_bit(HFC_CHIP_PCM_SLAVE, &hc->chip)) {
			/* SLAVE (clock master) */
			if (debug & DEBUG_HFCMULTI_INIT)
				printk(KERN_DEBUG "%s: E1 port is clock master\n", __FUNCTION__);
//			HFC_outb(hc, R_SYNC_CTRL, V_SYNC_OFFS | V_PCM_SYNC);
			HFC_outb(hc, R_SYNC_CTRL, V_EXT_CLK_SYNC | V_PCM_SYNC);
		} else {
			/* MASTER (clock slave) */
			if (debug & DEBUG_HFCMULTI_INIT)
				printk(KERN_DEBUG "%s: E1 port is clock slave\n", __FUNCTION__);
			HFC_outb(hc, R_SYNC_CTRL, V_SYNC_OFFS);
		}
		if (test_bit(HFC_CFG_NTMODE, &hc->chan[(i<<2)+2].cfg)) {
			if (debug & DEBUG_HFCMULTI_INIT)
				printk(KERN_DEBUG "%s: E1 port is NT-mode\n", __FUNCTION__);
			r_e1_wr_sta = 0; /* G0 */
		}else {
			if (debug & DEBUG_HFCMULTI_INIT)
				printk(KERN_DEBUG "%s: E1 port is TE-mode\n", __FUNCTION__);
			r_e1_wr_sta = 0; /* F0 */
		}
		HFC_outb(hc, R_JATT_ATT, 0x9c); /* undoc register */
		if (test_bit(HFC_CHIP_RX_SYNC, &hc->chip)) {
			HFC_outb(hc, R_SYNC_OUT, V_SYNC_E1_RX | V_IPATS0 | V_IPATS1 | V_IPATS2);
		} else {
			HFC_outb(hc, R_SYNC_OUT, V_IPATS0 | V_IPATS1 | V_IPATS2);
		}
		HFC_outb(hc, R_PWM_MD, V_PWM0_MD);
		HFC_outb(hc, R_PWM0, 0x50);
		HFC_outb(hc, R_PWM1, 0xff);
		/* state machine setup */
		HFC_outb(hc, R_E1_WR_STA, r_e1_wr_sta | V_E1_LD_STA);
		udelay(6); /* wait at least 5,21us */
		HFC_outb(hc, R_E1_WR_STA, r_e1_wr_sta);

	}

	/* set interrupts & global interrupt */
	hc->hw.r_irq_ctrl = V_FIFO_IRQ | V_GLOB_IRQ_EN;
	spin_unlock_irqrestore(&hc->lock, flags);
	if (debug & DEBUG_HFCMULTI_INIT)
		printk("%s: done\n", __FUNCTION__);
}


/***********************/
/* initialize the card */
/***********************/

/* start timer irq, wait some time and check if we have interrupts.
 * if not, reset chip and try again.
 */
static int
init_card(hfc_multi_t *hc)
{
	int 	cnt = 1; /* as long as there is no trouble */
	int 	err = -EIO;
#ifdef CONFIG_PLX_PCI_BRIDGE
	u_short	*plx_acc;
#endif

	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: entered\n", __FUNCTION__);

	if (request_irq(hc->pci_dev->irq, hfcmulti_interrupt, SA_SHIRQ, "HFC-multi", hc)) {
		printk(KERN_WARNING "mISDN: Could not get interrupt %d.\n", hc->pci_dev->irq);
		return(-EIO);
	}
	hc->irq = hc->pci_dev->irq;
#ifdef CONFIG_PLX_PCI_BRIDGE
	plx_acc=(u_short*)(hc->plx_membase+0x4c);
	*plx_acc=0x41;  // enable PCI & LINT1 irq
#endif

	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: IRQ %d count %d\n", __FUNCTION__, hc->irq, hc->irqcnt);
	while (cnt) {
		if ((err = init_chip(hc))) {
			goto error;
		}
		/* Finally enable IRQ output
		 * this is only allowed, if an IRQ routine is allready
		 * established for this HFC, so don't do that earlier
		 */
		HFC_outb(hc, R_IRQ_CTRL, V_GLOB_IRQ_EN);
		//printk(KERN_DEBUG "no master irq set!!!\n");
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout((100*HZ)/1000); /* Timeout 100ms */
		/* turn IRQ off until chip is completely initialized */
		HFC_outb(hc, R_IRQ_CTRL, 0);
		if (debug & DEBUG_HFCMULTI_INIT)
			printk(KERN_DEBUG "%s: IRQ %d count %d\n", __FUNCTION__, hc->irq, hc->irqcnt);
		if (hc->irqcnt) {
			if (debug & DEBUG_HFCMULTI_INIT)
				printk(KERN_DEBUG "%s: done\n", __FUNCTION__);
			return(0);
		}
		printk(KERN_WARNING "HFC PCI: IRQ(%d) getting no interrupts during init (try %d)\n", hc->irq, cnt);
#ifdef CONFIG_PLX_PCI_BRIDGE
		plx_acc=(u_short*)(hc->plx_membase+0x4c);
		*plx_acc=0x00;  // disable PCI & LINT1 irq
#endif
		cnt--;
		err = -EIO;
	}

	error:
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_WARNING "%s: free irq %d\n", __FUNCTION__, hc->irq);
	if (hc->irq) {
		free_irq(hc->irq, hc);
		hc->irq = 0;
	}

	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: done (err=%d)\n", __FUNCTION__, err);
	return(err);
}


/*********************************************************/
/* select free channel and return OK(0), -EBUSY, -EINVAL */
/*********************************************************/

static int
SelFreeBChannel(hfc_multi_t *hc, int ch, channel_info_t *ci)
{
	bchannel_t		*bch;
	hfc_multi_t		*hfc;
	mISDNstack_t		*bst;
	struct list_head	*head;
	int			cnr;
	int			port = hc->chan[ch].port;

	if (port < 0 || port>=hc->type) {
		printk(KERN_WARNING "%s: port(%d) out of range", __FUNCTION__, port);
		return(-EINVAL);
	}

	if (!ci)
		return(-EINVAL);
	ci->st.p = NULL;
	cnr=0;
	bst = hc->chan[ch].dch->inst.st;
	if (list_empty(&bst->childlist)) {
		if ((bst->id & FLG_CLONE_STACK) &&
			(bst->childlist.prev != &bst->childlist)) {
			head = bst->childlist.prev;
		} else {
			printk(KERN_ERR "%s: invalid empty childlist (no clone) stid(%x) childlist(%p<-%p->%p)\n",
				__FUNCTION__, bst->id, bst->childlist.prev, &bst->childlist, bst->childlist.next);
			return(-EINVAL);
		}
	} else
		head = &bst->childlist;
	list_for_each_entry(bst, head, list) {
		if (cnr == ((hc->type==1)?30:2)) /* 30 or 2 B-channels */ {
			printk(KERN_WARNING "%s: fatal error: more b-stacks than ports", __FUNCTION__);
			return(-EINVAL);
		}
		if(!bst->mgr) {
			int_errtxt("no mgr st(%p)", bst);
			return(-EINVAL);
		}
		hfc = bst->mgr->privat;
		if (!hfc) {
			int_errtxt("no mgr->data st(%p)", bst);
			return(-EINVAL);
		}
		if (hc->type==1)
			bch = hc->chan[cnr + 1 + (cnr>=15)].bch;
		else
			bch = hc->chan[(port<<2) + cnr].bch;
		if (!(ci->channel & (~CHANNEL_NUMBER))) {
			/* only number is set */
			if ((ci->channel & 0x3) == (cnr + 1)) {
				if (bch->protocol != ISDN_PID_NONE)
					return(-EBUSY);
				ci->st.p = bst;
				return(0);
			}
		}
		if ((ci->channel & (~CHANNEL_NUMBER)) == 0x00a18300) {
			if (bch->protocol == ISDN_PID_NONE) {
				ci->st.p = bst;
				return(0);
			}
		}
		cnr++;
	}
	return(-EBUSY);
}


/*********************************/
/* find pci device and set it up */
/*********************************/

static int
setup_pci(hfc_multi_t *hc, struct pci_dev *pdev,int id_idx)
{
	int i;
	//struct pci_dev *tmp_dev = NULL;
	//hfc_multi_t *hc_tmp, *next;

	printk(KERN_INFO "HFC-multi: card manufacturer: '%s' card name: '%s' clock: %s\n", id_list[id_idx].vendor_name, id_list[id_idx].card_name, (id_list[id_idx].clock2)?"double":"normal");

	/* go into 0-state (we might already be due to zero-filled-object */
	for(i=0;i<32;i++) {
		if (hc->chan[i].dch)
			hc->chan[i].dch->ph_state = 0;
	}

	hc->pci_dev = pdev;
	if (id_list[id_idx].clock2)
		test_and_set_bit(HFC_CHIP_CLOCK2, &hc->chip);
	if (hc->pci_dev->irq <= 0) {
		printk(KERN_WARNING "HFC-multi: No IRQ for PCI card found.\n");
		return (-EIO);
	}
	if (pci_enable_device(hc->pci_dev)) {
		printk(KERN_WARNING "HFC-multi: Error enabling PCI card.\n");
		return (-EIO);
	}
	hc->leds = id_list[id_idx].leds;
#ifdef CONFIG_HFCMULTI_PCIMEM
	hc->pci_membase = NULL;
	hc->plx_membase = NULL;

#ifdef CONFIG_PLX_PCI_BRIDGE
	hc->plx_origmembase =  get_pcibase(hc->pci_dev, 0);  // MEMBASE 1 is PLX PCI Bridge

	if (!hc->plx_origmembase) {
		printk(KERN_WARNING "HFC-multi: No IO-Memory for PCI PLX bridge found\n");
		pci_disable_device(hc->pci_dev);
		return (-EIO);
	}

	if (!(hc->plx_membase = ioremap(hc->plx_origmembase, 128))) {
		printk(KERN_WARNING "HFC-multi: failed to remap plx address space. (internal error)\n");
		hc->plx_membase = NULL;
		pci_disable_device(hc->pci_dev);
		return (-EIO);
	}
	printk(KERN_WARNING "HFC-multi: plx_membase:%#x plx_origmembase:%#x\n",(u_int) hc->plx_membase, (u_int)hc->plx_origmembase);

	hc->pci_origmembase =  get_pcibase(hc->pci_dev, 2);  // MEMBASE 1 is PLX PCI Bridge
	if (!hc->pci_origmembase) {
		printk(KERN_WARNING "HFC-multi: No IO-Memory for PCI card found\n");
		pci_disable_device(hc->pci_dev);
		return (-EIO);
	}

	if (!(hc->pci_membase = ioremap(hc->pci_origmembase, 0x400))) {
		printk(KERN_WARNING "HFC-multi: failed to remap io address space. (internal error)\n");
		hc->pci_membase = NULL;
		pci_disable_device(hc->pci_dev);
		return (-EIO);
	}

	printk(KERN_INFO "%s: defined at MEMBASE %#x (%#x) IRQ %d HZ %d leds-type %d\n", hc->name, (u_int) hc->pci_membase, (u_int) hc->pci_origmembase, hc->pci_dev->irq, HZ, hc->leds);
	pci_write_config_word(hc->pci_dev, PCI_COMMAND, PCI_ENA_MEMIO);
#else // CONFIG_PLX_PCI_BRIDGE
	hc->pci_origmembase = get_pcibase(hc->pci_dev, 1);
	if (!hc->pci_origmembase) {
		printk(KERN_WARNING "HFC-multi: No IO-Memory for PCI card found\n");
		pci_disable_device(hc->pci_dev);
		return (-EIO);
	}

	if (!(hc->pci_membase = ioremap(hc->pci_origmembase, 256))) {
		printk(KERN_WARNING "HFC-multi: failed to remap io address space. (internal error)\n");
		hc->pci_membase = NULL;
		pci_disable_device(hc->pci_dev);
		return (-EIO);
	}
	printk(KERN_INFO "%s: defined at MEMBASE %#x (%#x) IRQ %d HZ %d leds-type %d\n", hc->name, (u_int) hc->pci_membase, (u_int) hc->pci_origmembase, hc->pci_dev->irq, HZ, hc->leds);
	pci_write_config_word(hc->pci_dev, PCI_COMMAND, PCI_ENA_MEMIO);
#endif // CONFIG_PLX_PCI_BRIDGE
#else
	hc->pci_iobase = (u_int) get_pcibase(hc->pci_dev, 0);
	if (!hc->pci_iobase) {
		printk(KERN_WARNING "HFC-multi: No IO for PCI card found\n");
		pci_disable_device(hc->pci_dev);
		return (-EIO);
	}
	if (!request_region(hc->pci_iobase, 8, "hfcmulti")) {
		printk(KERN_WARNING "HFC-multi: failed to rquest address space at 0x%04x (internal error)\n", hc->pci_iobase);
		hc->pci_iobase = 0;
		pci_disable_device(hc->pci_dev);
		return (-EIO);
	}

	printk(KERN_INFO "%s: defined at IOBASE %#x IRQ %d HZ %d leds-type %d\n", hc->name, (u_int) hc->pci_iobase, hc->pci_dev->irq, HZ, hc->leds);
	pci_write_config_word(hc->pci_dev, PCI_COMMAND, PCI_ENA_REGIO);
#endif

	pci_set_drvdata(hc->pci_dev, hc);
	
	/* At this point the needed PCI config is done */
	/* fifos are still not enabled */
	return (0);
}


/*******************************************
 * remove port or complete card from stack *
 *******************************************/

static void
release_port(hfc_multi_t *hc, int port)
{
	int	i = 0;
	int	all = 1, any = 0;
	u_long	flags;

	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: entered\n", __FUNCTION__);

	if (port >= hc->type) {
		printk(KERN_WARNING "%s: ERROR port out of range (%d).\n", __FUNCTION__, port);
		return;
	}

//	if (debug & DEBUG_HFCMULTI_INIT)
//		printk(KERN_DEBUG "%s: before lock_dev\n", __FUNCTION__);
	spin_lock_irqsave(&hc->lock, flags);
//	if (debug & DEBUG_HFCMULTI_INIT)
//		printk(KERN_DEBUG "%s: after lock_dev\n", __FUNCTION__);

	if (port > -1) {
		i = 0;
		while(i < hc->type) {
			if (hc->created[i] && i!=port)
				all = 0;
			if (hc->created[i])
				any = 1;
			i++;
		}
		if (!any) {
			printk(KERN_WARNING "%s: ERROR card has no used stacks anymore.\n", __FUNCTION__);
			spin_unlock_irqrestore(&hc->lock, flags);
			return;
		}
	}
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: releasing port=%d all=%d any=%d\n", __FUNCTION__, port, all, any);

	if (port>-1 && !hc->created[port]) {
		printk(KERN_WARNING "%s: ERROR given stack is not used by card (port=%d).\n", __FUNCTION__, port);
		spin_unlock_irqrestore(&hc->lock, flags);
		return;
	}

	/* disable D-channels & B-channels */
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: disable all channels (d and b)\n", __FUNCTION__);
	if (hc->type == 1) {
		hc->chan[16].slot_tx = -1;
		hc->chan[16].slot_rx = -1;
		hc->chan[16].conf = -1;
		mode_hfcmulti(hc, 16, ISDN_PID_NONE, -1, 0, -1, 0);//d
		i = 0;
		while(i < 30) {
			hc->chan[i+1+(i>=15)].slot_tx = -1;
			hc->chan[i+1+(i>=15)].slot_rx = -1;
			hc->chan[i+1+(i>=15)].conf = -1;
			mode_hfcmulti(hc, i+1+(i>=15), ISDN_PID_NONE, -1, 0, -1, 0); //b
			i++;
		}
	} else {
		i = 0;
		while(i < hc->type) {
			if (all || port==i)
			if (hc->created[i]) {
				hc->chan[(i<<2)+2].slot_tx = -1;
				hc->chan[(i<<2)+2].slot_rx = -1;
				hc->chan[(i<<2)+2].conf = -1;
				mode_hfcmulti(hc, (i<<2)+2, ISDN_PID_NONE, -1, 0, -1, 0); //d
				hc->chan[i<<2].slot_tx = -1;
				hc->chan[i<<2].slot_rx = -1;
				hc->chan[i<<2].conf = -1;
				mode_hfcmulti(hc, i<<2, ISDN_PID_NONE, -1, 0, -1, 0); //b1
				hc->chan[(i<<2)+1].slot_tx = -1;
				hc->chan[(i<<2)+1].slot_rx = -1;
				hc->chan[(i<<2)+1].conf = -1;
				mode_hfcmulti(hc, (i<<2)+1, ISDN_PID_NONE, -1, 0, -1, 0); //b2
			}
			i++;
		}
	}

	i = 0;
	while(i < 32) {
		if (hc->chan[i].dch)
		if (hc->created[hc->chan[i].port])
		if (hc->chan[i].dch->dbusytimer.function != NULL && (all || port==i)) {
			del_timer(&hc->chan[i].dch->dbusytimer);
			hc->chan[i].dch->dbusytimer.function = NULL;
		}
		i++;
	}

	/* free channels */
	i = 0;
	while(i < 32) {
		if (hc->created[hc->chan[i].port])
		if (hc->chan[i].port==port || all) {
			if (hc->chan[i].dch) {
				if (debug & DEBUG_HFCMULTI_INIT)
					printk(KERN_DEBUG "%s: free port %d D-channel %d (1..32)\n", __FUNCTION__, hc->chan[i].port, i);
				mISDN_free_dch(hc->chan[i].dch);
				HFCM_obj.ctrl(&hc->chan[i].dch->inst, MGR_UNREGLAYER | REQUEST, NULL);
				kfree(hc->chan[i].dch);
				hc->chan[i].dch = NULL;
			}
			if (hc->chan[i].rx_buf) {
				kfree(hc->chan[i].rx_buf);
				hc->chan[i].rx_buf = NULL;
			}
			if (hc->chan[i].bch) {
				if (debug & DEBUG_HFCMULTI_INIT)
					printk(KERN_DEBUG "%s: free port %d B-channel %d (1..32)\n", __FUNCTION__, hc->chan[i].port, i);
				mISDN_free_bch(hc->chan[i].bch);
				kfree(hc->chan[i].bch);
				hc->chan[i].bch = NULL;
			}
		}
		i++;
	}
	i = 0;
	while(i < 8) {
		if (i==port || all)
			hc->created[i] = 0;
		i++;
	}

	/* dimm leds */
	if (hc->leds)
		hfcmulti_leds(hc);

	/* release IO & remove card */
	if (all) {
		/* release hardware */
		if (debug & DEBUG_HFCMULTI_INIT)
			printk(KERN_WARNING "%s: card has no more used stacks, so we release hardware.\n", __FUNCTION__);
		release_io_hfcmulti(hc);
		spin_unlock_irqrestore(&hc->lock, flags);
		/* release irq */
		if (hc->irq) {
			if (debug & DEBUG_HFCMULTI_INIT)
				printk(KERN_WARNING "%s: free irq %d\n", __FUNCTION__, hc->irq);
			free_irq(hc->irq, hc);
			hc->irq = 0;
		}

		/* remove us from list and delete */
		if (debug & DEBUG_HFCMULTI_INIT)
			printk(KERN_WARNING "%s: remove instance from list\n", __FUNCTION__);
		spin_lock_irqsave(&HFCM_obj.lock, flags);
		list_del(&hc->list);
		spin_unlock_irqrestore(&HFCM_obj.lock, flags);
		if (debug & DEBUG_HFCMULTI_INIT)
			printk(KERN_WARNING "%s: delete instance\n", __FUNCTION__);
//#warning
//		kfree(hc->davor);
//		kfree(hc->danach);
		kfree(hc);
		HFC_cnt--;
		if (debug & DEBUG_HFCMULTI_INIT)
			printk(KERN_WARNING "%s: card successfully removed\n", __FUNCTION__);
	} else
		spin_unlock_irqrestore(&hc->lock, flags);
}

static int
HFC_manager(void *data, u_int prim, void *arg)
{
	hfc_multi_t	*hc;
	mISDNinstance_t	*inst = data;
	struct sk_buff	*skb;
	dchannel_t	*dch = NULL;
	bchannel_t	*bch = NULL;
	int		ch = -1;
	int		i;
	u_long		flags;

	if (!data) {
		MGR_HASPROTOCOL_HANDLER(prim,arg,&HFCM_obj)
		printk(KERN_ERR "%s: no data prim %x arg %p\n", __FUNCTION__, prim, arg);
		return(-EINVAL);
	}

	/* find channel and card */
	spin_lock_irqsave(&HFCM_obj.lock, flags);
	list_for_each_entry(hc, &HFCM_obj.ilist, list) {
		i = 0;
		while(i < 32) {
//printk(KERN_DEBUG "comparing (D-channel) card=%08x inst=%08x with inst=%08x\n", hc, &hc->dch[i].inst, inst);
			if (hc->chan[i].dch)
			if (&hc->chan[i].dch->inst == inst) {
				ch = i;
				dch = hc->chan[i].dch;
				break;
			}
			if (hc->chan[i].bch)
			if (&hc->chan[i].bch->inst == inst) {
				ch = i;
				bch = hc->chan[i].bch;
				break;
			}
			i++;
		}
		if (ch >= 0)
			break;
	}
	spin_unlock_irqrestore(&HFCM_obj.lock, flags);
	if (ch < 0) {
		printk(KERN_ERR "%s: no card/channel found  data %p prim %x arg %p\n", __FUNCTION__, data, prim, arg);
		return(-EINVAL);
	}
	if (debug & DEBUG_HFCMULTI_MGR)
		printk(KERN_DEBUG "%s: channel %d (0..31)  data %p prim %x arg %p\n", __FUNCTION__, ch, data, prim, arg);

	switch(prim) {
		case MGR_REGLAYER | CONFIRM:
bugtest
		if (debug & DEBUG_HFCMULTI_MGR)
			printk(KERN_DEBUG "%s: MGR_REGLAYER\n", __FUNCTION__);
		if (dch)
			dch_set_para(dch, &inst->st->para);
		if (bch)
			bch_set_para(bch, &inst->st->para);
bugtest
		break;

		case MGR_UNREGLAYER | REQUEST:
bugtest
		if (debug & DEBUG_HFCMULTI_MGR)
			printk(KERN_DEBUG "%s: MGR_UNREGLAYER\n", __FUNCTION__);
		if (dch) {
			if ((skb = create_link_skb(PH_CONTROL | REQUEST, HW_DEACTIVATE, 0, NULL, 0))) {
				if (hfcmulti_l1hw(inst, skb)) dev_kfree_skb(skb);
			}
		} else
		if (bch) {
			if ((skb = create_link_skb(PH_CONTROL | REQUEST, 0, 0, NULL, 0))) {
				if (hfcmulti_l2l1(inst, skb)) dev_kfree_skb(skb);
			}
		}
		HFCM_obj.ctrl(inst, MGR_UNREGLAYER | REQUEST, NULL);
bugtest
		break;

		case MGR_CLRSTPARA | INDICATION:
		arg = NULL;
		// fall through
		case MGR_ADDSTPARA | INDICATION:
		if (debug & DEBUG_HFCMULTI_MGR)
			printk(KERN_DEBUG "%s: MGR_***STPARA\n", __FUNCTION__);
		if (dch)
			dch_set_para(dch, arg);
		if (bch)
			bch_set_para(bch, arg);
		break;

		case MGR_RELEASE | INDICATION:
		if (debug & DEBUG_HFCMULTI_MGR)
			printk(KERN_DEBUG "%s: MGR_RELEASE = remove port from mISDN\n", __FUNCTION__);
		if (dch) {
			release_port(hc, hc->chan[ch].port); /* hc is free */
		}
		break;
#ifdef FIXME
		case MGR_CONNECT | REQUEST:
		if (debug & DEBUG_HFCMULTI_MGR)
			printk(KERN_DEBUG "%s: MGR_CONNECT\n", __FUNCTION__);
		return(mISDN_ConnectIF(inst, arg));

		case MGR_SETIF | REQUEST:
		case MGR_SETIF | INDICATION:
		if (debug & DEBUG_HFCMULTI_MGR)
			printk(KERN_DEBUG "%s: MGR_SETIF\n", __FUNCTION__);
		if (dch)
			return(mISDN_SetIF(inst, arg, prim, hfcmulti_l1hw, NULL, dch));
		if (bch)
			return(mISDN_SetIF(inst, arg, prim, hfcmulti_l2l1, NULL, bch));
		break;

		case MGR_DISCONNECT | REQUEST:
		case MGR_DISCONNECT | INDICATION:
		if (debug & DEBUG_HFCMULTI_MGR)
			printk(KERN_DEBUG "%s: MGR_DISCONNECT\n", __FUNCTION__);
		return(mISDN_DisConnectIF(inst, arg));
#endif
		case MGR_SELCHANNEL | REQUEST:
		if (debug & DEBUG_HFCMULTI_MGR)
			printk(KERN_DEBUG "%s: MGR_SELCHANNEL\n", __FUNCTION__);
		if (!dch) {
			printk(KERN_WARNING "%s(MGR_SELCHANNEL|REQUEST): selchannel not dinst\n", __FUNCTION__);
			return(-EINVAL);
		}
		return(SelFreeBChannel(hc, ch, arg));

		case MGR_SETSTACK | INDICATION:
bugtest
		if (debug & DEBUG_HFCMULTI_MGR)
			printk(KERN_DEBUG "%s: MGR_SETSTACK\n", __FUNCTION__);
		if (bch && inst->pid.global==2) {
			if ((skb = create_link_skb(PH_ACTIVATE | REQUEST, 0, 0, NULL, 0))) {
				if (hfcmulti_l2l1(inst, skb)) dev_kfree_skb(skb);
			}
			if (inst->pid.protocol[2] == ISDN_PID_L2_B_TRANS)
			mISDN_queue_data(inst, FLG_MSG_UP, DL_ESTABLISH | INDICATION, 0, 0, NULL, 0);
		else mISDN_queue_data(inst, FLG_MSG_UP, PH_ACTIVATE | INDICATION, 0, 0, NULL, 0);
		}
bugtest
		break;

		PRIM_NOT_HANDLED(MGR_CTRLREADY | INDICATION);
		PRIM_NOT_HANDLED(MGR_GLOBALOPT | REQUEST);
		default:
		printk(KERN_WARNING "%s: prim %x not handled\n", __FUNCTION__, prim);
		return(-EINVAL);
	}
	return(0);
}

static void find_type_entry(int hfc_type, int *card, int *port)
{
	int i, j = 0;

	for(i=0;i<MAX_CARDS;i++)
	{
		if((type[i]&0xff)==hfc_type && !allocated[i])
		{
			*card = i;
			*port = j;
			return;
		}
		j = j + (type[i]&0xff);
	}
	*card = -1;
}

static int find_idlist_entry(int vendor,int subvendor, int device, int subdevice)
{
	int cnt;

	cnt=0;
	while(id_list[cnt].vendor_id)
	{
		if(id_list[cnt].vendor_id==vendor && id_list[cnt].vendor_sub==subvendor
		   && id_list[cnt].device_id==device && id_list[cnt].device_sub==subdevice) return(cnt);
		cnt++;
	}

	return(-1);
}

static int __devinit hfcpci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int		i,ret_err=0,port_idx;
	int		bchperport, pt;
	int		ch, ch2;
	int		id_idx;        // index to id_list
	int		hfc_type;      // chip type
	hfc_multi_t	*hc;
	mISDN_pid_t	pid, pids[8];
	mISDNstack_t	*dst = NULL; /* make gcc happy */
	dchannel_t	*dch;
	bchannel_t	*bch;
	u_long		flags;

	id_idx=find_idlist_entry(ent->vendor,ent->subvendor,ent->device,ent->subdevice);
	if(id_idx== -1)
	{
		if (ent->vendor == CCAG_VID)
			if (ent->device == HFC4S_ID
			 || ent->device == HFC8S_ID
			 || ent->device == HFCE1_ID)
				printk( KERN_ERR "unknown HFC multiport controller (vendor:%x device:%x subvendor:%x subdevice:%x) Please contact the driver maintainer for support.\n",
					ent->vendor,ent->device,ent->subvendor,ent->subdevice);
		return (-ENODEV);
	}

	hfc_type=id_list[id_idx].type;
	find_type_entry(hfc_type, &HFC_idx, &port_idx);
	if(HFC_idx == -1)
	{
		printk( KERN_ERR "HFC-MULTI: Card '%s' found, but not given by module's options, ignoring...\n",
			id_list[id_idx].card_name);
		return (-ENODEV);
	}

	if (debug & DEBUG_HFCMULTI_INIT) printk(KERN_DEBUG "%s: Registering chip type %d (0x%x)\n", __FUNCTION__, type[HFC_idx] & 0xff, type[HFC_idx]);

	/* check card type */
	switch (type[HFC_idx] & 0xff) {
		case 1:
		bchperport = 30;
		break;

		case 4:
		bchperport = 2;
		break;

		case 8:
		bchperport = 2;
		break;

		default:
		printk(KERN_ERR "Card type(%d) not supported.\n", type[HFC_idx] & 0xff);
		ret_err = -EINVAL;
		goto free_object;
	}


	/* allocate card+fifo structure */
//#warning
//void *davor=kmalloc(8, GFP_ATOMIC);
	if (!(hc = kmalloc(sizeof(hfc_multi_t), GFP_ATOMIC))) {
		printk(KERN_ERR "No kmem for HFC-Multi card\n");
		ret_err = -ENOMEM;
		goto free_object;
	}
//void *danach=kmalloc(8, GFP_ATOMIC);
	memset(hc, 0, sizeof(hfc_multi_t));
//hc->davor=davor;
//hc->danach=danach;
	hc->idx = HFC_idx;
	hc->id = HFC_idx + 1;
	hc->pcm = pcm[HFC_idx];

	/* set chip specific features */
	hc->masterclk = -1;
	hc->type = type[HFC_idx] & 0xff;
	if (type[HFC_idx] & 0x100) {
		test_and_set_bit(HFC_CHIP_ULAW, &hc->chip);
		silence = 0xff; /* ulaw silence */
	} else
		silence = 0x2a; /* alaw silence */
	if (type[HFC_idx] & 0x200)
		test_and_set_bit(HFC_CHIP_DTMF, &hc->chip);
//		if ((type[HFC_idx]&0x400) && hc->type==4)
//			test_and_set_bit(HFC_CHIP_LEDS, &hc->chip);
	if (type[HFC_idx] & 0x800)
		test_and_set_bit(HFC_CHIP_PCM_SLAVE, &hc->chip);
	if (type[HFC_idx] & 0x1000)
		test_and_set_bit(HFC_CHIP_CLOCK_IGNORE, &hc->chip);
	if (type[HFC_idx] & 0x2000)
		test_and_set_bit(HFC_CHIP_RX_SYNC, &hc->chip);
	if (type[HFC_idx] & 0x4000)
		test_and_set_bit(HFC_CHIP_EXRAM_128, &hc->chip);
	if (type[HFC_idx] & 0x8000)
		test_and_set_bit(HFC_CHIP_EXRAM_512, &hc->chip);
	hc->slots = 32;
	if (type[HFC_idx] & 0x10000)
		hc->slots = 64;
	if (type[HFC_idx] & 0x20000)
		hc->slots = 128;
	if (hc->type == 1)
		sprintf(hc->name, "HFC-E1#%d", HFC_idx+1);
	else
		sprintf(hc->name, "HFC-%dS#%d", hc->type, HFC_idx+1);

	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: (after APPEND_TO_LIST)\n", __FUNCTION__);
	
	spin_lock_irqsave(&HFCM_obj.lock, flags);
	list_add_tail(&hc->list, &HFCM_obj.ilist);
	spin_unlock_irqrestore(&HFCM_obj.lock, flags);
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: (after APPEND_TO_LIST)\n", __FUNCTION__);

	spin_lock_init(&hc->lock);

	pt = 0;
	while (pt < hc->type) {
		if (port_idx >= MAX_PORTS) {
			printk(KERN_ERR "Invalid HFC type.\n");
			ret_err = -EINVAL;
			goto free_channels;
		}
		if (protocol[port_idx] == 0) {
			printk(KERN_ERR "Not enough 'protocol' values given.\n");
			ret_err = -EINVAL;
			goto free_channels;
		}
		if (hc->type == 1)
			ch = 16;
		else
			ch = (pt<<2)+2;
		if (debug & DEBUG_HFCMULTI_INIT)
			printk(KERN_DEBUG "%s: Registering D-channel, card(%d) ch(%d) port(%d) protocol(%x)\n", __FUNCTION__, HFC_idx+1, ch, pt+1, protocol[port_idx]);
		hc->chan[ch].port = pt;
		hc->chan[ch].nt_timer = -1;
		dch = kmalloc(sizeof(dchannel_t), GFP_ATOMIC);
		if (!dch) {
			ret_err = -ENOMEM;
			goto free_channels;
		}
		memset(dch, 0, sizeof(dchannel_t));
		dch->channel = ch;
		//dch->debug = debug;
		dch->inst.obj = &HFCM_obj;
		dch->inst.hwlock = &hc->lock;
		dch->inst.class_dev.dev = &pdev->dev;
		mISDN_init_instance(&dch->inst, &HFCM_obj, hc, hfcmulti_l1hw);
		dch->inst.pid.layermask = ISDN_LAYER(0);
		sprintf(dch->inst.name, "HFCm%d/%d", HFC_idx+1, pt+1);
		if (!(hc->chan[ch].rx_buf = kmalloc(MAX_DFRAME_LEN_L1, GFP_ATOMIC))) {
			ret_err = -ENOMEM;
			goto free_channels;
		}
		if (mISDN_init_dch(dch)) {
			ret_err = -ENOMEM;
			goto free_channels;
		}
		hc->chan[ch].dch = dch;

		i=0;
		while(i < bchperport) {
			if (hc->type == 1)
				ch2 = i + 1 + (i>=15);
			else
				ch2 = (pt<<2)+i;
			if (debug & DEBUG_HFCMULTI_INIT)
				printk(KERN_DEBUG "%s: Registering B-channel, card(%d) ch(%d) port(%d) channel(%d)\n", __FUNCTION__, HFC_idx+1, ch2, pt+1, i);
			hc->chan[ch2].port = pt;
			bch = kmalloc(sizeof(bchannel_t), GFP_ATOMIC);
			if (!bch) {
				ret_err = -ENOMEM;
				goto free_channels;
			}
			memset(bch, 0, sizeof(bchannel_t));
			bch->channel = ch2;
			mISDN_init_instance(&bch->inst, &HFCM_obj, hc, hfcmulti_l2l1);
			bch->inst.pid.layermask = ISDN_LAYER(0);
			bch->inst.hwlock = &hc->lock;
			bch->inst.class_dev.dev = &pdev->dev;
			//bch->debug = debug;
			sprintf(bch->inst.name, "%s B%d",
				dch->inst.name, i+1);
			if (mISDN_init_bch(bch)) {
				kfree(bch);
				ret_err = -ENOMEM;
				goto free_channels;
			}
			hc->chan[ch2].bch = bch;
#ifdef FIXME  // TODO
			if (bch->dev) {
				bch->dev->wport.pif.func = hfcmulti_l2l1;
				bch->dev->wport.pif.fdata = bch;
			}
#endif
			i++;
		}

		/* set D-channel */
		mISDN_set_dchannel_pid(&pid, protocol[port_idx], layermask[port_idx]);

		/* set PRI */
		if (hc->type == 1) {
			if (layermask[port_idx] & ISDN_LAYER(2)) {
				pid.protocol[2] |= ISDN_PID_L2_DF_PTP;
			}
			if (layermask[port_idx] & ISDN_LAYER(3)) {
				pid.protocol[3] |= ISDN_PID_L3_DF_PTP;
				pid.protocol[3] |= ISDN_PID_L3_DF_EXTCID;
				pid.protocol[3] |= ISDN_PID_L3_DF_CRLEN2;
			}
		}

		/* set protocol type */
		if (protocol[port_idx] & 0x10) {
			/* NT-mode */
			dch->inst.pid.protocol[0] = (hc->type==1)?ISDN_PID_L0_NT_E1:ISDN_PID_L0_NT_S0;
			dch->inst.pid.protocol[1] = (hc->type==1)?ISDN_PID_L1_NT_E1:ISDN_PID_L1_NT_S0;
			pid.protocol[0] = (hc->type==1)?ISDN_PID_L0_NT_E1:ISDN_PID_L0_NT_S0;
			pid.protocol[1] = (hc->type==1)?ISDN_PID_L1_NT_E1:ISDN_PID_L1_NT_S0;
			dch->inst.pid.layermask |= ISDN_LAYER(1);
			pid.layermask |= ISDN_LAYER(1);
			if (layermask[port_idx] & ISDN_LAYER(2))
				pid.protocol[2] = ISDN_PID_L2_LAPD_NET;
			test_and_set_bit(HFC_CFG_NTMODE, &hc->chan[ch].cfg);
		} else {
			/* TE-mode */
			dch->inst.pid.protocol[0] = (hc->type==1)?ISDN_PID_L0_TE_E1:ISDN_PID_L0_TE_S0;
			pid.protocol[0] = (hc->type==1)?ISDN_PID_L0_TE_E1:ISDN_PID_L0_TE_S0;
			if (hc->type == 1) {
				/* own E1 for E1 */
				dch->inst.pid.protocol[1] = ISDN_PID_L1_TE_E1;
				pid.protocol[1] = ISDN_PID_L1_TE_E1;
				dch->inst.pid.layermask |= ISDN_LAYER(1);
				pid.layermask |= ISDN_LAYER(1);
			}
		}


		if (hc->type != 1) {
			/* S/T */
			/* set master clock */
			if (protocol[port_idx] & 0x10000) {
				if (debug & DEBUG_HFCMULTI_INIT)
					printk(KERN_DEBUG "%s: PROTOCOL set master clock: card(%d) port(%d)\n", __FUNCTION__, HFC_idx+1, pt);
				if (test_bit(HFC_CFG_NTMODE, &hc->chan[ch].cfg)) {
					printk(KERN_ERR "Error: Master clock for port(%d) of card(%d) is only possible with TE-mode\n", pt+1, HFC_idx+1);
					ret_err = -EINVAL;
					goto free_channels;
				}
				if (hc->masterclk >= 0) {
					printk(KERN_ERR "Error: Master clock for port(%d) of card(%d) already defined for port(%d)\n", pt+1, HFC_idx+1, hc->masterclk+1);
					ret_err = -EINVAL;
					goto free_channels;
				}
				hc->masterclk = pt;
			}

			/* set transmitter line to non capacitive */
			if (protocol[port_idx] & 0x20000) {
				if (debug & DEBUG_HFCMULTI_INIT)
					printk(KERN_DEBUG "%s: PROTOCOL set non capacitive transmitter: card(%d) port(%d)\n", __FUNCTION__, HFC_idx+1, pt);
				test_and_set_bit(HFC_CFG_NONCAP_TX, &hc->chan[ch].cfg);
			}

			/* disable E-channel */
			if (protocol[port_idx] & 0x40000) {
				if (debug & DEBUG_HFCMULTI_INIT)
					printk(KERN_DEBUG "%s: PROTOCOL disable E-channel: card(%d) port(%d)\n", __FUNCTION__, HFC_idx+1, pt);
				test_and_set_bit(HFC_CFG_DIS_ECHANNEL, &hc->chan[ch].cfg);
			}
			/* register E-channel */
			if (protocol[port_idx] & 0x80000) {
				if (debug & DEBUG_HFCMULTI_INIT)
					printk(KERN_DEBUG "%s: PROTOCOL register E-channel: card(%d) port(%d)\n", __FUNCTION__, HFC_idx+1, pt);
				test_and_set_bit(HFC_CFG_REG_ECHANNEL, &hc->chan[ch].cfg);
			}
		} else {
			/* E1 */
			/* set optical line type */
			if (protocol[port_idx] & 0x10000) {
				if (debug & DEBUG_HFCMULTI_INIT)
					printk(KERN_DEBUG "%s: PROTOCOL set optical interfacs: card(%d) port(%d)\n", __FUNCTION__, HFC_idx+1, pt);
				test_and_set_bit(HFC_CFG_OPTICAL, &hc->chan[ch].cfg);
			}

			/* set LOS report */
			if (protocol[port_idx] & 0x40000) {
				if (debug & DEBUG_HFCMULTI_INIT)
					printk(KERN_DEBUG "%s: PROTOCOL set LOS report: card(%d) port(%d)\n", __FUNCTION__, HFC_idx+1, pt);
				test_and_set_bit(HFC_CFG_REPORT_LOS, &hc->chan[ch].cfg);
			}

			/* set AIS report */
			if (protocol[port_idx] & 0x80000) {
				if (debug & DEBUG_HFCMULTI_INIT)
					printk(KERN_DEBUG "%s: PROTOCOL set AIS report: card(%d) port(%d)\n", __FUNCTION__, HFC_idx+1, pt);
				test_and_set_bit(HFC_CFG_REPORT_AIS, &hc->chan[ch].cfg);
			}

			/* set SLIP report */
			if (protocol[port_idx] & 0x100000) {
				if (debug & DEBUG_HFCMULTI_INIT)
					printk(KERN_DEBUG "%s: PROTOCOL set SLIP report: card(%d) port(%d)\n", __FUNCTION__, HFC_idx+1, pt);
				test_and_set_bit(HFC_CFG_REPORT_SLIP, &hc->chan[ch].cfg);
			}

			/* set elastic jitter buffer */
			if (protocol[port_idx] & 0x600000) {
				if (debug & DEBUG_HFCMULTI_INIT)
					printk(KERN_DEBUG "%s: PROTOCOL set elastic buffer to %d: card(%d) port(%d)\n", __FUNCTION__, hc->chan[ch].jitter, HFC_idx+1, pt);
				hc->chan[ch].jitter = (protocol[port_idx]>>21) & 0x3;
			} else
				hc->chan[ch].jitter = 2; /* default */
		}

		memcpy(&pids[pt], &pid, sizeof(pid));

		pt++;
		port_idx++;
	}

	/* run card setup */
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: Setting up card(%d)\n", __FUNCTION__, HFC_idx+1);
	if ((ret_err = setup_pci(hc,pdev,id_idx))) {
		goto free_channels;
	}
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: Initializing card(%d)\n", __FUNCTION__, HFC_idx+1);
	if ((ret_err = init_card(hc))) {
		if (debug & DEBUG_HFCMULTI_INIT) {
			printk(KERN_DEBUG "%s: do release_io_hfcmulti\n", __FUNCTION__);
			release_io_hfcmulti(hc);
		}
		goto free_channels;
	}

	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: Init modes card(%d)\n", __FUNCTION__, HFC_idx+1);
	hfcmulti_initmode(hc);

	/* add stacks */
	pt = 0;
	while(pt < hc->type) {
		if (debug & DEBUG_HFCMULTI_INIT)
			printk(KERN_DEBUG "%s: Adding d-stack: card(%d) port(%d)\n", __FUNCTION__, HFC_idx+1, pt+1);
		if (hc->type == 1)
			dch = hc->chan[16].dch;
		else
			dch = hc->chan[(pt<<2)+2].dch;
		if ((ret_err = HFCM_obj.ctrl(NULL, MGR_NEWSTACK | REQUEST, &dch->inst))) {
			printk(KERN_ERR  "MGR_ADDSTACK REQUEST dch err(%d)\n", ret_err);
			free_release:
			release_port(hc, -1); /* all ports, hc is free */
			goto free_object;
		}
		/* indicate that this stack is created */
		hc->created[pt] = 1;

		dst = dch->inst.st;

		i = 0;
		while(i < bchperport) {
			if (debug & DEBUG_HFCMULTI_INIT)
				printk(KERN_DEBUG "%s: Adding b-stack: card(%d) port(%d) B-channel(%d)\n", __FUNCTION__, HFC_idx+1, pt+1, i+1);
			if (hc->type == 1)
				bch = hc->chan[i + 1 + (i>=15)].bch;
			else
				bch = hc->chan[(pt<<2) + i].bch;
			if ((ret_err = HFCM_obj.ctrl(dst, MGR_NEWSTACK | REQUEST, &bch->inst))) {
				printk(KERN_ERR "MGR_ADDSTACK bchan error %d\n", ret_err);
				free_delstack:
				HFCM_obj.ctrl(dst, MGR_DELSTACK | REQUEST, NULL);
				goto free_release;
			}
			bch->st = bch->inst.st;
			i++;
		}
		if (debug & DEBUG_HFCMULTI_INIT)
			printk(KERN_DEBUG "%s: (before MGR_SETSTACK REQUEST) layermask=0x%x\n", __FUNCTION__, pids[pt].layermask);

		if ((ret_err = HFCM_obj.ctrl(dst, MGR_SETSTACK | REQUEST, &pids[pt]))) {
			printk(KERN_ERR "MGR_SETSTACK REQUEST dch err(%d)\n", ret_err);
			goto free_delstack;
		}
		if (debug & DEBUG_HFCMULTI_INIT)
			printk(KERN_DEBUG "%s: (after MGR_SETSTACK REQUEST)\n", __FUNCTION__);

		/* delay some time */
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout((100*HZ)/1000); /* Timeout 100ms */

		/* tell stack, that we are ready */
		HFCM_obj.ctrl(dst, MGR_CTRLREADY | INDICATION, NULL);

		pt++;
	}

	/* now turning on irq */
#warning
	HFC_outb(hc, R_IRQ_CTRL, hc->hw.r_irq_ctrl);

	/* we are on air! */
	allocated[HFC_idx] = 1;
	HFC_cnt++;
	return(0);

	/* if an error ocurred */
	free_channels:
	i = 0;
	while(i < 32) {
		if (hc->chan[i].dch) {
			if (debug & DEBUG_HFCMULTI_INIT)
				printk(KERN_DEBUG "%s: free D-channel %d (1..32)\n", __FUNCTION__, i);
			mISDN_free_dch(hc->chan[i].dch);
			kfree(hc->chan[i].dch);
			hc->chan[i].dch = NULL;
		}
		if (hc->chan[i].rx_buf) {
			kfree(hc->chan[i].rx_buf);
			hc->chan[i].rx_buf = NULL;
		}
		if (hc->chan[i].bch) {
			if (debug & DEBUG_HFCMULTI_INIT)
				printk(KERN_DEBUG "%s: free B-channel %d (1..32)\n", __FUNCTION__, i);
			mISDN_free_bch(hc->chan[i].bch);
			kfree(hc->chan[i].bch);
			hc->chan[i].bch = NULL;
		}
		i++;
	}
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: before REMOVE_FROM_LIST (refcnt = %d)\n", __FUNCTION__, HFCM_obj.refcnt);
	spin_lock_irqsave(&HFCM_obj.lock, flags);
	list_del(&hc->list);
	spin_unlock_irqrestore(&HFCM_obj.lock, flags);
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: after REMOVE_FROM_LIST (refcnt = %d)\n", __FUNCTION__, HFCM_obj.refcnt);
	kfree(hc);

	free_object:
	return(ret_err);
}

static void __devexit hfc_remove_pci(struct pci_dev *pdev)
{
	int i,ch;
	hfc_multi_t	*card = pci_get_drvdata(pdev);

	printk( KERN_INFO "removing hfc_multi card vendor:%x device:%x subvendor:%x subdevice:%x\n",
			pdev->vendor,pdev->device,pdev->subsystem_vendor,pdev->subsystem_device);
	if (card)
	{
		for(i=0;i<card->type;i++)  // type is also number of d-channel
		{
			if(card->created[i])
			{
				if (card->type == 1) ch = 16;
				else ch = (i*4)+2;
				// if created elete stack
				if(card->chan[ch].dch)  HFCM_obj.ctrl(card->chan[ch].dch->inst.st, MGR_DELSTACK | REQUEST, NULL);
			}
		}
		// relase all ports
		allocated[card->idx] = 0;
		release_port(card, -1); // card is free ... */
		printk(KERN_DEBUG "test\n");
	}
	else printk(KERN_WARNING "%s: drvdata allready removed\n", __FUNCTION__);
}

static struct pci_device_id hfmultipci_ids[] __devinitdata = {
	{ CCAG_VID, 0x08B4   , PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ CCAG_VID, 0x16B8   , PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ CCAG_VID, 0x30B1   , PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0x10B5,   0x9030   , CCAG_VID,   0x3136 ,    0, 0, 0 },  // PLX PCI Bridge
	{ 0x10B5,   0x9030   , PCI_ANY_ID,   PCI_ANY_ID ,  0, 0, 0 },  // PLX PCI Bridge
	{0, }
};
MODULE_DEVICE_TABLE(pci, hfmultipci_ids);

static struct pci_driver hfcmultipci_driver = {
	name:     "hfc_multi",
	probe:    hfcpci_probe,
	remove:   __devexit_p(hfc_remove_pci),
	id_table: hfmultipci_ids,
};

static void __exit
HFCmulti_cleanup(void)
{
	hfc_multi_t *hc,*next;
	int err;

	/* unregister mISDN object */
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: entered (refcnt = %d HFC_cnt = %d)\n", __FUNCTION__, HFCM_obj.refcnt, HFC_cnt);
	if ((err = mISDN_unregister(&HFCM_obj))) {
		printk(KERN_ERR "Can't unregister HFC-Multi cards error(%d)\n", err);
	}

	/* remove remaining devices, but this should never happen */
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: now checking ilist (refcnt = %d)\n", __FUNCTION__, HFCM_obj.refcnt);

	list_for_each_entry_safe(hc, next, &HFCM_obj.ilist, list) {
		printk(KERN_ERR "HFC PCI card struct not empty refs %d\n", HFCM_obj.refcnt);
		release_port(hc, -1); /* all ports, hc is free */
	}
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: done (refcnt = %d HFC_cnt = %d)\n", __FUNCTION__, HFCM_obj.refcnt, HFC_cnt);

	/* get rid of all devices of this driver */
	pci_unregister_driver(&hfcmultipci_driver);

}

static int __init
HFCmulti_init(void)
{
	int err, i;
	char tmpstr[64];

#if !defined(CONFIG_HOTPLUG) || !defined(MODULE)
#error	"CONFIG_HOTPLUG and MODULE are not defined."
#endif
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: init entered\n", __FUNCTION__);

#ifdef __BIG_ENDIAN
#error "not running on big endian machines now"
#endif
	strcpy(tmpstr, hfcmulti_revision);
	printk(KERN_INFO "mISDN: HFC-multi driver Rev. %s\n", mISDN_getrev(tmpstr));

	switch(poll) {
		case 0:
		poll_timer = 6;
		poll = 128;
		break; /* wenn dieses break nochmal verschwindet, gibt es heisse ohren :-) */
		case 8:
		poll_timer = 2;
		break;
		case 16:
		poll_timer = 3;
		break;
		case 32:
		poll_timer = 4;
		break;
		case 64:
		poll_timer = 5;
		break;
		case 128:
		poll_timer = 6;
		break;
		case 256:
		poll_timer = 7;
		break;
		default:
		printk(KERN_ERR "%s: Wrong poll value (%d).\n", __FUNCTION__, poll);
		err = -EINVAL;
		return(err);

	}

	memset(&HFCM_obj, 0, sizeof(HFCM_obj));
#ifdef MODULE
	HFCM_obj.owner = THIS_MODULE;
#endif
	spin_lock_init(&HFCM_obj.lock);
	INIT_LIST_HEAD(&HFCM_obj.ilist);
	HFCM_obj.name = HFCName;
	HFCM_obj.own_ctrl = HFC_manager;
	HFCM_obj.DPROTO.protocol[0] = ISDN_PID_L0_TE_S0 | ISDN_PID_L0_NT_S0
				| ISDN_PID_L0_TE_E1 | ISDN_PID_L0_NT_E1;
	HFCM_obj.DPROTO.protocol[1] = ISDN_PID_L1_NT_S0
				| ISDN_PID_L1_TE_E1 | ISDN_PID_L1_NT_E1;
	HFCM_obj.BPROTO.protocol[1] = ISDN_PID_L1_B_64TRANS | ISDN_PID_L1_B_64HDLC;
	HFCM_obj.BPROTO.protocol[2] = ISDN_PID_L2_B_TRANS | ISDN_PID_L2_B_RAWDEV;

	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: registering HFCM_obj\n", __FUNCTION__);
	if ((err = mISDN_register(&HFCM_obj))) {
		printk(KERN_ERR "Can't register HFC-Multi cards error(%d)\n", err);
		return(err);
	}
	if (debug & DEBUG_HFCMULTI_INIT)
		printk(KERN_DEBUG "%s: new mISDN object (refcnt = %d)\n", __FUNCTION__, HFCM_obj.refcnt);

	for(i=0;i<MAX_CARDS;i++) allocated[i]=0;
	HFC_cnt = HFC_idx = 0;

	err = pci_register_driver(&hfcmultipci_driver);
	if (err < 0)
	{
		printk(KERN_ERR "error registering pci driver:%x\n",err);
		HFCmulti_cleanup();
		return(err);
	}
	printk(KERN_INFO "%d devices registered\n", HFC_cnt);

	return(0);
}


#ifdef MODULE
module_init(HFCmulti_init);
module_exit(HFCmulti_cleanup);
#endif

