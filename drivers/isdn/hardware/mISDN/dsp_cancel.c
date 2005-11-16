/*
 *
 * Simple but fast Echo cancellation for mISDN_dsp.
 *
 * Copyright Chrisian Richter
 *
 * This software may be used and distributed according to the terms
 * of the GNU General Public License, incorporated herein by reference.
 *
 */

#include "layer1.h"
#include "helper.h"
#include "debug.h"
#include "dsp.h"


/*
 * how this works:
 *
 * 
 * 
 */
void bchdev_echocancel_chunk(dsp_t* dev, uint8_t *rxchunk, uint8_t *txchunk, uint16_t size);
int bchdev_echocancel_activate(dsp_t* dev, int deftaps, int train);
void bchdev_echocancel_deactivate(dsp_t* dev);


void
dsp_cancel_tx(dsp_t *dsp, u8 *data, int len)
{
	if (!dsp ) return ;
	if (!data) return;
	
	if (dsp->txbuflen + len < ECHOCAN_BUFLEN) {
		memcpy(&dsp->txbuf[dsp->txbuflen],data,len);
		dsp->txbuflen+=len;
	} else {
		printk("ECHOCAN: TXBUF Overflow len:%d newlen:%d\n",dsp->txbuflen,len);
		dsp->txbuflen=0;
	}
	
}

void
dsp_cancel_rx(dsp_t *dsp, u8 *data, int len)
{
	if (!dsp ) return ;
	if (!data) return;
	
	if (len <= dsp->txbuflen) {
		char tmp[ECHOCAN_BUFLEN];
		
		int delta=dsp->txbuflen-len;
		
		memcpy(tmp,&dsp->txbuf[len],delta);
		
		bchdev_echocancel_chunk(dsp, data, dsp->txbuf, len);
		
		memcpy(dsp->txbuf,tmp,delta);
		dsp->txbuflen=delta;
		//dsp->txbuflen=0;
		
		//bchdev_echocancel_chunk(dsp,  dsp->txbuf, data, len);
	} else {
		printk("ECHOCAN: TXBUF Underrun len:%d newlen:%d\n",dsp->txbuflen,len);
	}
	
}

int
dsp_cancel_init(dsp_t *dsp, int deftaps, int training, int delay)
{
	
	if (!dsp) return -1;
	
	printk("DSP_CANCEL_INIT called\n");
	
	if (delay < 0)
	{
		printk("Disabling EC\n");
		dsp->cancel_enable = 0;
		
		dsp->txbuflen=0;
		
		bchdev_echocancel_deactivate(dsp);
		
		return(0);
	}
	
	dsp->txbuflen=0;
	dsp->rxbuflen=0;
	
	
	bchdev_echocancel_activate(dsp,deftaps, training);
	
	printk("Enabling EC\n");
	dsp->cancel_enable = 1;
	return(0);
}





/*****************************************************/
#define __ECHO_STATE_MUTE                       (1 << 8)
#define ECHO_STATE_IDLE                         (0)
#define ECHO_STATE_PRETRAINING          (1 | (__ECHO_STATE_MUTE))
#define ECHO_STATE_STARTTRAINING        (2 | (__ECHO_STATE_MUTE))
#define ECHO_STATE_AWAITINGECHO         (3 | (__ECHO_STATE_MUTE))
#define ECHO_STATE_TRAINING                     (4 | (__ECHO_STATE_MUTE))
#define ECHO_STATE_ACTIVE                       (5)

#define AMI_MASK 0x55



/** @return string of given echo cancellation state */
char* bchdev_echocancel_statestr(uint16_t state)
{
  switch(state) {
  case ECHO_STATE_IDLE:
    return "idle";
    break;
  case ECHO_STATE_PRETRAINING:
    return "pre-training";
    break;
  case ECHO_STATE_STARTTRAINING:
    return "transmit impulse";
    break;
  case ECHO_STATE_AWAITINGECHO:
    return "awaiting echo";
    break;
  case ECHO_STATE_TRAINING:
    return "training start";
    break;
  case ECHO_STATE_ACTIVE:
    return "training finished";
    break;
  default:
    return "unknown";
  }
}

/** Changes state of echo cancellation to given state */
void bchdev_echocancel_setstate(dsp_t* dev, uint16_t state)
{
  char* statestr = bchdev_echocancel_statestr(state);
  
  printk("bchdev: echo cancel state %d (%s)\n", state & 0xff, statestr);
  if (state == ECHO_STATE_ACTIVE)
	  printk("bchdev: %d taps trained\n", dev->echolastupdate);
  dev->echostate = state;
}

static int buf_size=0;
static int ec_timer=2000;
//static int ec_timer=1000;


/** Activates echo cancellation for the given bch_dev, device must have been locked before! */
int bchdev_echocancel_activate(dsp_t* dev, int deftaps, int training)
{
  int taps;
  
  if (! dev) return -EINVAL;
  
  if (dev->ec && dev->ecdis_rd && dev->ecdis_wr) {
	  // already active
    return 0;
  }
  
  if (deftaps>0) {
	  taps=deftaps;
  } else {
	  taps=128;
  }
  
  
  switch (buf_size) {
  case  0: taps +=    0; break;
  case  1: taps +=  256-128; break;
  case  2: taps +=  512-128; break;
  default: taps += 1024-128;
  }
  
  if (!dev->ec) dev->ec = echo_can_create(taps, 0);
  if (!dev->ec) {
	  return -ENOMEM;
  }
  
  dev->echolastupdate = 0;

  if (!training) {
	  dev->echotimer=0;
	  bchdev_echocancel_setstate(dev, ECHO_STATE_IDLE);
  } else {
	  if (training<10) 
		  training= ec_timer;
	  
	  dev->echotimer      = training;
	  bchdev_echocancel_setstate(dev, ECHO_STATE_PRETRAINING);

  }
  
  if (!dev->ecdis_rd) dev->ecdis_rd = kmalloc(sizeof(echo_can_disable_detector_state_t), GFP_KERNEL);
  if (!dev->ecdis_rd) {
	  kfree(dev->ec); dev->ec = NULL;
	  return -ENOMEM;
  }
  echo_can_disable_detector_init(dev->ecdis_rd);
  
  if (!dev->ecdis_wr) dev->ecdis_wr = kmalloc(sizeof(echo_can_disable_detector_state_t), GFP_KERNEL);
  if (!dev->ecdis_wr) {
	  kfree(dev->ec); dev->ec = NULL;
	  kfree(dev->ecdis_rd); dev->ecdis_rd = NULL;
    return -ENOMEM;
  }
  echo_can_disable_detector_init(dev->ecdis_wr);

  return 0;
}

/** Deactivates echo cancellation for the given bch_dev, device must have been locked before! */
void bchdev_echocancel_deactivate(dsp_t* dev)
{
  if (! dev) return;

  //chan_misdn_log("bchdev: deactivating echo cancellation on port=%04x, chan=%02x\n", dev->stack->port, dev->channel);
  
  if (dev->ec) echo_can_free(dev->ec);
  dev->ec = NULL;
  
  dev->echolastupdate = 0;
  dev->echotimer      = 0;
  bchdev_echocancel_setstate(dev, ECHO_STATE_IDLE);

  if (dev->ecdis_rd) kfree(dev->ecdis_rd);
  dev->ecdis_rd = NULL;
  
  if (dev->ecdis_wr) kfree(dev->ecdis_wr);
  dev->ecdis_wr = NULL;
}

/** Processes one TX- and one RX-packet with echocancellation */
void bchdev_echocancel_chunk(dsp_t* dev, uint8_t *rxchunk, uint8_t *txchunk, uint16_t size)
{
  int16_t rxlin, txlin;
  uint16_t pos;

  /* Perform echo cancellation on a chunk if requested */
  if (dev->ec) {
    if (dev->echostate & __ECHO_STATE_MUTE) {
      if (dev->echostate == ECHO_STATE_STARTTRAINING) {
	// Transmit impulse now
	txchunk[0] = dsp_audio_s16_to_law[16384 & 0xffff];
	memset(txchunk+1, 0, size-1);
	bchdev_echocancel_setstate(dev, ECHO_STATE_TRAINING); //AWAITINGECHO);
      } else {
	// train the echo cancellation
	for (pos = 0; pos < size; pos++) {
	  rxlin = dsp_audio_law_to_s32[rxchunk[pos]];
	  txlin = dsp_audio_law_to_s32[txchunk[pos]];
	  if (dev->echostate == ECHO_STATE_PRETRAINING) {
	    if (dev->echotimer <= 0) {
	      dev->echotimer = 0;
	      bchdev_echocancel_setstate(dev, ECHO_STATE_STARTTRAINING);
	    } else {
	      dev->echotimer--;
	    }
	  }
	  if ((dev->echostate == ECHO_STATE_AWAITINGECHO) && (txlin > 8000)) {
	    dev->echolastupdate = 0;
	    bchdev_echocancel_setstate(dev, ECHO_STATE_TRAINING);
	  }
	  if (dev->echostate == ECHO_STATE_TRAINING) {
	    if (echo_can_traintap(dev->ec, dev->echolastupdate++, rxlin)) {
	      bchdev_echocancel_setstate(dev, ECHO_STATE_ACTIVE);
	    }
	  }

	  rxchunk[pos] = dsp_silence;
	  txchunk[pos] = dsp_silence;
	}
      }
    } else {
      for (pos = 0; pos < size; pos++) {
	rxlin = dsp_audio_law_to_s32[rxchunk[pos]];
	txlin = dsp_audio_law_to_s32[txchunk[pos]];

	if (echo_can_disable_detector_update(dev->ecdis_rd, rxlin) || 
	    echo_can_disable_detector_update(dev->ecdis_wr, txlin)) {
	  bchdev_echocancel_deactivate(dev);
	  printk("EC: Disable tone detected\n");
	  return ;
	} else	{
	  rxlin = echo_can_update(dev->ec, txlin, rxlin);
	  rxchunk[pos] = dsp_audio_s16_to_law[rxlin & 0xffff];
	}
      }
    }
  }
}

/******************************************************/
