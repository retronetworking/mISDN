/* $Id$
 *
 * Linux ISDN subsystem, DTMF tone module
 *
 * Author	Karsten Keil (kkeil@suse.de)
 *
 * based on I4L isdn_audio code
 * Copyright 2003 by Karsten Keil (kkeil@suse.de)
 * Copyright 1994-1999 by Fritz Elfert (fritz@isdn4linux.de)
 * DTMF code (c) 1996 by Christian Mock (cm@kukuruz.ping.at)
 * Silence detection (c) 1998 by Armin Schindler (mac@gismo.telekom.de)
 *
 * This software may be used and distributed according to the terms
 * of the GNU General Public License, incorporated herein by reference.
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include "layer1.h"
#include "helper.h"
#include "debug.h"
#include "ctrl.h"

#define DTMF_NPOINTS 205        /* Number of samples for DTMF recognition */

typedef struct _dtmf {
	struct list_head	list;
	u_long 			Flags;
	int			debug;
	char			last;
	int			idx;
	int			buf[DTMF_NPOINTS];
	mISDNinstance_t		inst;
} dtmf_t;

#define	FLG_DTMF_ULAW	1
#define FLG_DTMF_ACTIV	2

static u_int debug = 0;

#define DEBUG_DTMF_MGR		0x001
#define DEBUG_DTMF_TONE		0x010
#define DEBUG_DTMF_CTRL		0x020
#define DEBUG_DTMF_DETECT	0x100
#define DEBUG_DTMF_KOEFF	0x200

static mISDNobject_t dtmf_obj;

static char *mISDN_dtmf_revision = "$Revision$";

/*
 * Misc. lookup-tables.
 */

/* ulaw -> signed 16-bit */
static short isdn_audio_ulaw_to_s16[] =
{
	0x8284, 0x8684, 0x8a84, 0x8e84, 0x9284, 0x9684, 0x9a84, 0x9e84,
	0xa284, 0xa684, 0xaa84, 0xae84, 0xb284, 0xb684, 0xba84, 0xbe84,
	0xc184, 0xc384, 0xc584, 0xc784, 0xc984, 0xcb84, 0xcd84, 0xcf84,
	0xd184, 0xd384, 0xd584, 0xd784, 0xd984, 0xdb84, 0xdd84, 0xdf84,
	0xe104, 0xe204, 0xe304, 0xe404, 0xe504, 0xe604, 0xe704, 0xe804,
	0xe904, 0xea04, 0xeb04, 0xec04, 0xed04, 0xee04, 0xef04, 0xf004,
	0xf0c4, 0xf144, 0xf1c4, 0xf244, 0xf2c4, 0xf344, 0xf3c4, 0xf444,
	0xf4c4, 0xf544, 0xf5c4, 0xf644, 0xf6c4, 0xf744, 0xf7c4, 0xf844,
	0xf8a4, 0xf8e4, 0xf924, 0xf964, 0xf9a4, 0xf9e4, 0xfa24, 0xfa64,
	0xfaa4, 0xfae4, 0xfb24, 0xfb64, 0xfba4, 0xfbe4, 0xfc24, 0xfc64,
	0xfc94, 0xfcb4, 0xfcd4, 0xfcf4, 0xfd14, 0xfd34, 0xfd54, 0xfd74,
	0xfd94, 0xfdb4, 0xfdd4, 0xfdf4, 0xfe14, 0xfe34, 0xfe54, 0xfe74,
	0xfe8c, 0xfe9c, 0xfeac, 0xfebc, 0xfecc, 0xfedc, 0xfeec, 0xfefc,
	0xff0c, 0xff1c, 0xff2c, 0xff3c, 0xff4c, 0xff5c, 0xff6c, 0xff7c,
	0xff88, 0xff90, 0xff98, 0xffa0, 0xffa8, 0xffb0, 0xffb8, 0xffc0,
	0xffc8, 0xffd0, 0xffd8, 0xffe0, 0xffe8, 0xfff0, 0xfff8, 0x0000,
	0x7d7c, 0x797c, 0x757c, 0x717c, 0x6d7c, 0x697c, 0x657c, 0x617c,
	0x5d7c, 0x597c, 0x557c, 0x517c, 0x4d7c, 0x497c, 0x457c, 0x417c,
	0x3e7c, 0x3c7c, 0x3a7c, 0x387c, 0x367c, 0x347c, 0x327c, 0x307c,
	0x2e7c, 0x2c7c, 0x2a7c, 0x287c, 0x267c, 0x247c, 0x227c, 0x207c,
	0x1efc, 0x1dfc, 0x1cfc, 0x1bfc, 0x1afc, 0x19fc, 0x18fc, 0x17fc,
	0x16fc, 0x15fc, 0x14fc, 0x13fc, 0x12fc, 0x11fc, 0x10fc, 0x0ffc,
	0x0f3c, 0x0ebc, 0x0e3c, 0x0dbc, 0x0d3c, 0x0cbc, 0x0c3c, 0x0bbc,
	0x0b3c, 0x0abc, 0x0a3c, 0x09bc, 0x093c, 0x08bc, 0x083c, 0x07bc,
	0x075c, 0x071c, 0x06dc, 0x069c, 0x065c, 0x061c, 0x05dc, 0x059c,
	0x055c, 0x051c, 0x04dc, 0x049c, 0x045c, 0x041c, 0x03dc, 0x039c,
	0x036c, 0x034c, 0x032c, 0x030c, 0x02ec, 0x02cc, 0x02ac, 0x028c,
	0x026c, 0x024c, 0x022c, 0x020c, 0x01ec, 0x01cc, 0x01ac, 0x018c,
	0x0174, 0x0164, 0x0154, 0x0144, 0x0134, 0x0124, 0x0114, 0x0104,
	0x00f4, 0x00e4, 0x00d4, 0x00c4, 0x00b4, 0x00a4, 0x0094, 0x0084,
	0x0078, 0x0070, 0x0068, 0x0060, 0x0058, 0x0050, 0x0048, 0x0040,
	0x0038, 0x0030, 0x0028, 0x0020, 0x0018, 0x0010, 0x0008, 0x0000
};

/* alaw -> signed 16-bit */
static short isdn_audio_alaw_to_s16[] =
{
	0x13fc, 0xec04, 0x0144, 0xfebc, 0x517c, 0xae84, 0x051c, 0xfae4,
	0x0a3c, 0xf5c4, 0x0048, 0xffb8, 0x287c, 0xd784, 0x028c, 0xfd74,
	0x1bfc, 0xe404, 0x01cc, 0xfe34, 0x717c, 0x8e84, 0x071c, 0xf8e4,
	0x0e3c, 0xf1c4, 0x00c4, 0xff3c, 0x387c, 0xc784, 0x039c, 0xfc64,
	0x0ffc, 0xf004, 0x0104, 0xfefc, 0x417c, 0xbe84, 0x041c, 0xfbe4,
	0x083c, 0xf7c4, 0x0008, 0xfff8, 0x207c, 0xdf84, 0x020c, 0xfdf4,
	0x17fc, 0xe804, 0x018c, 0xfe74, 0x617c, 0x9e84, 0x061c, 0xf9e4,
	0x0c3c, 0xf3c4, 0x0084, 0xff7c, 0x307c, 0xcf84, 0x030c, 0xfcf4,
	0x15fc, 0xea04, 0x0164, 0xfe9c, 0x597c, 0xa684, 0x059c, 0xfa64,
	0x0b3c, 0xf4c4, 0x0068, 0xff98, 0x2c7c, 0xd384, 0x02cc, 0xfd34,
	0x1dfc, 0xe204, 0x01ec, 0xfe14, 0x797c, 0x8684, 0x07bc, 0xf844,
	0x0f3c, 0xf0c4, 0x00e4, 0xff1c, 0x3c7c, 0xc384, 0x03dc, 0xfc24,
	0x11fc, 0xee04, 0x0124, 0xfedc, 0x497c, 0xb684, 0x049c, 0xfb64,
	0x093c, 0xf6c4, 0x0028, 0xffd8, 0x247c, 0xdb84, 0x024c, 0xfdb4,
	0x19fc, 0xe604, 0x01ac, 0xfe54, 0x697c, 0x9684, 0x069c, 0xf964,
	0x0d3c, 0xf2c4, 0x00a4, 0xff5c, 0x347c, 0xcb84, 0x034c, 0xfcb4,
	0x12fc, 0xed04, 0x0134, 0xfecc, 0x4d7c, 0xb284, 0x04dc, 0xfb24,
	0x09bc, 0xf644, 0x0038, 0xffc8, 0x267c, 0xd984, 0x026c, 0xfd94,
	0x1afc, 0xe504, 0x01ac, 0xfe54, 0x6d7c, 0x9284, 0x06dc, 0xf924,
	0x0dbc, 0xf244, 0x00b4, 0xff4c, 0x367c, 0xc984, 0x036c, 0xfc94,
	0x0f3c, 0xf0c4, 0x00f4, 0xff0c, 0x3e7c, 0xc184, 0x03dc, 0xfc24,
	0x07bc, 0xf844, 0x0008, 0xfff8, 0x1efc, 0xe104, 0x01ec, 0xfe14,
	0x16fc, 0xe904, 0x0174, 0xfe8c, 0x5d7c, 0xa284, 0x05dc, 0xfa24,
	0x0bbc, 0xf444, 0x0078, 0xff88, 0x2e7c, 0xd184, 0x02ec, 0xfd14,
	0x14fc, 0xeb04, 0x0154, 0xfeac, 0x557c, 0xaa84, 0x055c, 0xfaa4,
	0x0abc, 0xf544, 0x0058, 0xffa8, 0x2a7c, 0xd584, 0x02ac, 0xfd54,
	0x1cfc, 0xe304, 0x01cc, 0xfe34, 0x757c, 0x8a84, 0x075c, 0xf8a4,
	0x0ebc, 0xf144, 0x00d4, 0xff2c, 0x3a7c, 0xc584, 0x039c, 0xfc64,
	0x10fc, 0xef04, 0x0114, 0xfeec, 0x457c, 0xba84, 0x045c, 0xfba4,
	0x08bc, 0xf744, 0x0018, 0xffe8, 0x227c, 0xdd84, 0x022c, 0xfdd4,
	0x18fc, 0xe704, 0x018c, 0xfe74, 0x657c, 0x9a84, 0x065c, 0xf9a4,
	0x0cbc, 0xf344, 0x0094, 0xff6c, 0x327c, 0xcd84, 0x032c, 0xfcd4
};

/* alaw -> ulaw */
static char isdn_audio_alaw_to_ulaw[] =
{
	0xab, 0x2b, 0xe3, 0x63, 0x8b, 0x0b, 0xc9, 0x49,
	0xba, 0x3a, 0xf6, 0x76, 0x9b, 0x1b, 0xd7, 0x57,
	0xa3, 0x23, 0xdd, 0x5d, 0x83, 0x03, 0xc1, 0x41,
	0xb2, 0x32, 0xeb, 0x6b, 0x93, 0x13, 0xcf, 0x4f,
	0xaf, 0x2f, 0xe7, 0x67, 0x8f, 0x0f, 0xcd, 0x4d,
	0xbe, 0x3e, 0xfe, 0x7e, 0x9f, 0x1f, 0xdb, 0x5b,
	0xa7, 0x27, 0xdf, 0x5f, 0x87, 0x07, 0xc5, 0x45,
	0xb6, 0x36, 0xef, 0x6f, 0x97, 0x17, 0xd3, 0x53,
	0xa9, 0x29, 0xe1, 0x61, 0x89, 0x09, 0xc7, 0x47,
	0xb8, 0x38, 0xf2, 0x72, 0x99, 0x19, 0xd5, 0x55,
	0xa1, 0x21, 0xdc, 0x5c, 0x81, 0x01, 0xbf, 0x3f,
	0xb0, 0x30, 0xe9, 0x69, 0x91, 0x11, 0xce, 0x4e,
	0xad, 0x2d, 0xe5, 0x65, 0x8d, 0x0d, 0xcb, 0x4b,
	0xbc, 0x3c, 0xfa, 0x7a, 0x9d, 0x1d, 0xd9, 0x59,
	0xa5, 0x25, 0xde, 0x5e, 0x85, 0x05, 0xc3, 0x43,
	0xb4, 0x34, 0xed, 0x6d, 0x95, 0x15, 0xd1, 0x51,
	0xac, 0x2c, 0xe4, 0x64, 0x8c, 0x0c, 0xca, 0x4a,
	0xbb, 0x3b, 0xf8, 0x78, 0x9c, 0x1c, 0xd8, 0x58,
	0xa4, 0x24, 0xde, 0x5e, 0x84, 0x04, 0xc2, 0x42,
	0xb3, 0x33, 0xec, 0x6c, 0x94, 0x14, 0xd0, 0x50,
	0xb0, 0x30, 0xe8, 0x68, 0x90, 0x10, 0xce, 0x4e,
	0xbf, 0x3f, 0xfe, 0x7e, 0xa0, 0x20, 0xdc, 0x5c,
	0xa8, 0x28, 0xe0, 0x60, 0x88, 0x08, 0xc6, 0x46,
	0xb7, 0x37, 0xf0, 0x70, 0x98, 0x18, 0xd4, 0x54,
	0xaa, 0x2a, 0xe2, 0x62, 0x8a, 0x0a, 0xc8, 0x48,
	0xb9, 0x39, 0xf4, 0x74, 0x9a, 0x1a, 0xd6, 0x56,
	0xa2, 0x22, 0xdd, 0x5d, 0x82, 0x02, 0xc0, 0x40,
	0xb1, 0x31, 0xea, 0x6a, 0x92, 0x12, 0xcf, 0x4f,
	0xae, 0x2e, 0xe6, 0x66, 0x8e, 0x0e, 0xcc, 0x4c,
	0xbd, 0x3d, 0xfc, 0x7c, 0x9e, 0x1e, 0xda, 0x5a,
	0xa6, 0x26, 0xdf, 0x5f, 0x86, 0x06, 0xc4, 0x44,
	0xb5, 0x35, 0xee, 0x6e, 0x96, 0x16, 0xd2, 0x52
};

/* ulaw -> alaw */
static char isdn_audio_ulaw_to_alaw[] =
{
	0xab, 0x55, 0xd5, 0x15, 0x95, 0x75, 0xf5, 0x35,
	0xb5, 0x45, 0xc5, 0x05, 0x85, 0x65, 0xe5, 0x25,
	0xa5, 0x5d, 0xdd, 0x1d, 0x9d, 0x7d, 0xfd, 0x3d,
	0xbd, 0x4d, 0xcd, 0x0d, 0x8d, 0x6d, 0xed, 0x2d,
	0xad, 0x51, 0xd1, 0x11, 0x91, 0x71, 0xf1, 0x31,
	0xb1, 0x41, 0xc1, 0x01, 0x81, 0x61, 0xe1, 0x21,
	0x59, 0xd9, 0x19, 0x99, 0x79, 0xf9, 0x39, 0xb9,
	0x49, 0xc9, 0x09, 0x89, 0x69, 0xe9, 0x29, 0xa9,
	0xd7, 0x17, 0x97, 0x77, 0xf7, 0x37, 0xb7, 0x47,
	0xc7, 0x07, 0x87, 0x67, 0xe7, 0x27, 0xa7, 0xdf,
	0x9f, 0x7f, 0xff, 0x3f, 0xbf, 0x4f, 0xcf, 0x0f,
	0x8f, 0x6f, 0xef, 0x2f, 0x53, 0x13, 0x73, 0x33,
	0xb3, 0x43, 0xc3, 0x03, 0x83, 0x63, 0xe3, 0x23,
	0xa3, 0x5b, 0xdb, 0x1b, 0x9b, 0x7b, 0xfb, 0x3b,
	0xbb, 0xbb, 0x4b, 0x4b, 0xcb, 0xcb, 0x0b, 0x0b,
	0x8b, 0x8b, 0x6b, 0x6b, 0xeb, 0xeb, 0x2b, 0x2b,
	0xab, 0x54, 0xd4, 0x14, 0x94, 0x74, 0xf4, 0x34,
	0xb4, 0x44, 0xc4, 0x04, 0x84, 0x64, 0xe4, 0x24,
	0xa4, 0x5c, 0xdc, 0x1c, 0x9c, 0x7c, 0xfc, 0x3c,
	0xbc, 0x4c, 0xcc, 0x0c, 0x8c, 0x6c, 0xec, 0x2c,
	0xac, 0x50, 0xd0, 0x10, 0x90, 0x70, 0xf0, 0x30,
	0xb0, 0x40, 0xc0, 0x00, 0x80, 0x60, 0xe0, 0x20,
	0x58, 0xd8, 0x18, 0x98, 0x78, 0xf8, 0x38, 0xb8,
	0x48, 0xc8, 0x08, 0x88, 0x68, 0xe8, 0x28, 0xa8,
	0xd6, 0x16, 0x96, 0x76, 0xf6, 0x36, 0xb6, 0x46,
	0xc6, 0x06, 0x86, 0x66, 0xe6, 0x26, 0xa6, 0xde,
	0x9e, 0x7e, 0xfe, 0x3e, 0xbe, 0x4e, 0xce, 0x0e,
	0x8e, 0x6e, 0xee, 0x2e, 0x52, 0x12, 0x72, 0x32,
	0xb2, 0x42, 0xc2, 0x02, 0x82, 0x62, 0xe2, 0x22,
	0xa2, 0x5a, 0xda, 0x1a, 0x9a, 0x7a, 0xfa, 0x3a,
	0xba, 0xba, 0x4a, 0x4a, 0xca, 0xca, 0x0a, 0x0a,
	0x8a, 0x8a, 0x6a, 0x6a, 0xea, 0xea, 0x2a, 0x2a
};

#define NCOEFF            8     /* number of frequencies to be analyzed       */
#define DTMF_TRESH     4000     /* above this is dtmf                         */
#define SILENCE_TRESH   200     /* below this is silence                      */
#define AMP_BITS          9     /* bits per sample, reduced to avoid overflow */
#define LOGRP             0
#define HIGRP             1

/* For DTMF recognition:
 * 2 * cos(2 * PI * k / N) precalculated for all k
 */
static int cos2pik[NCOEFF] =
{
	55813, 53604, 51193, 48591, 38114, 33057, 25889, 18332
};

static char dtmf_matrix[4][4] =
{
	{'1', '2', '3', 'A'},
	{'4', '5', '6', 'B'},
	{'7', '8', '9', 'C'},
	{'*', '0', '#', 'D'}
};

static inline void
isdn_audio_tlookup(const u_char *table, u_char *buff, unsigned long n)
{
#ifdef __i386__
	unsigned long d0, d1, d2, d3;
	__asm__ __volatile__(
		"cld\n"
		"1:\tlodsb\n\t"
		"xlatb\n\t"
		"stosb\n\t"
		"loop 1b\n\t"
	:	"=&b"(d0), "=&c"(d1), "=&D"(d2), "=&S"(d3)
	:	"0"((long) table), "1"(n), "2"((long) buff), "3"((long) buff)
	:	"memory", "ax");
#else
	while (n--)
		*buff = table[*(unsigned char *)buff], buff++;
#endif
}

void
isdn_audio_ulaw2alaw(unsigned char *buff, unsigned long len)
{
	isdn_audio_tlookup(isdn_audio_ulaw_to_alaw, buff, len);
}

void
isdn_audio_alaw2ulaw(unsigned char *buff, unsigned long len)
{
	isdn_audio_tlookup(isdn_audio_alaw_to_ulaw, buff, len);
}

/*
 * Goertzel algorithm.
 * See http://ptolemy.eecs.berkeley.edu/~pino/Ptolemy/papers/96/dtmf_ict/
 * for more info.
 */

static void
isdn_audio_goertzel(dtmf_t *dtmf)
{
	int		sk[NCOEFF], sk1[NCOEFF], sk2[NCOEFF];
	register int	sample;	
	int		k, n;
	int		thresh, silence;
	int		lgrp,hgrp;
	char		what;

	memset(sk, 0, NCOEFF*sizeof(int));
	memset(sk1, 0, NCOEFF*sizeof(int));
	memset(sk2, 0, NCOEFF*sizeof(int));
	for (n = 0; n < DTMF_NPOINTS; n++) {
		sample = dtmf->buf[n];
		for (k = 0; k < NCOEFF; k++)
			sk[k] = sample + ((cos2pik[k] * sk1[k]) >> 15) - sk2[k];
		memcpy(sk2, sk1, NCOEFF*sizeof(int));
		memcpy(sk1, sk, NCOEFF*sizeof(int));
	}	
	thresh = silence = 0;
	lgrp = hgrp = -1;
	for (k = 0; k < NCOEFF; k++) {
		sk[k] >>= 1;
		sk2[k] >>= 1;
		/* compute |X(k)|**2 */
		/* report overflows. This should not happen. */
		/* Comment this out if desired */
		if (sk[k] < -32768 || sk[k] > 32767)
			printk(KERN_DEBUG
				"dtmf goertzel overflow, sk[%d]=%d\n", k, sk[k]);
		if (sk2[k] < -32768 || sk2[k] > 32767)
			printk(KERN_DEBUG
				"isdn_audio: dtmf goertzel overflow, sk2[%d]=%d\n", k, sk2[k]);
		sk1[k] = ((sk[k] * sk[k]) >> AMP_BITS) -
			((((cos2pik[k] * sk[k]) >> 15) * sk2[k]) >> AMP_BITS) +
			((sk2[k] * sk2[k]) >> AMP_BITS);
		if (sk1[k] > DTMF_TRESH) {
			if (sk1[k] > thresh)
				thresh = sk1[k];
		} else if (sk1[k] < SILENCE_TRESH)
			silence++;
	}
	if (dtmf->debug & DEBUG_DTMF_KOEFF)
		printk(KERN_DEBUG "DTMF koeff(%d,%d,%d,%d,%d,%d,%d,%d) range(%d-%d)\n",
			sk1[0], sk1[1], sk1[2], sk1[3], sk1[4], sk1[5],
			sk1[6], sk1[7], SILENCE_TRESH, DTMF_TRESH);
	if (silence == NCOEFF)
		what = ' ';
	else {
		if (thresh > 0)	{
			thresh = thresh >> 4;  /* touchtones must match within 12 dB */
			for (k = 0; k < NCOEFF; k++) {
				if (sk1[k] < thresh)
					continue;  /* ignore */
				/* good level found. This is allowed only one time per group */
				if (k < NCOEFF / 2) {
					/* lowgroup*/
					if (lgrp >= 0) {
						// Bad. Another tone found. */
						lgrp = -1;
						break;
					} else
						lgrp = k;
				} else { /* higroup */
					if (hgrp >= 0) {
						// Bad. Another tone found. */
						hgrp = -1;
						break;
					} else
						hgrp = k - NCOEFF/2;
				}
			}
			if ((lgrp >= 0) && (hgrp >= 0)) {
				what = dtmf_matrix[lgrp][hgrp];
				if (dtmf->last != ' ' && dtmf->last != '.')
					dtmf->last = what;	/* min. 1 non-DTMF between DTMF */
			} else
					what = '.';
		} else
			what = '.';
	}
	if (dtmf->debug & DEBUG_DTMF_DETECT)
		printk(KERN_DEBUG "DTMF: last(%c) what(%c)\n",
			dtmf->last, what);
	if ((what != dtmf->last) && (what != ' ') && (what != '.')) {
		if (dtmf->debug & DEBUG_DTMF_TONE)
			printk(KERN_DEBUG "DTMF: tone='%c'\n", what);
		k = what | DTMF_TONE_VAL;
		mISDN_queue_data(&dtmf->inst, FLG_MSG_UP, PH_CONTROL | INDICATION,
			0, sizeof(int), &k, 0);
	}
	dtmf->last = what;
}

/*
 * Decode audio stream into signed u16 
 * start detection if enough data was sampled
 */
static void
isdn_audio_calc_dtmf(dtmf_t *dtmf, struct sk_buff *skb)
{
	int len = skb->len;
	u_char	*p = skb->data;
	int i;
	int c;

	while (len) {
		c = DTMF_NPOINTS - dtmf->idx;
		if (c > len)
			c = len;
		if (c <= 0)
			break;
		for (i = 0; i < c; i++) {
			if (test_bit(FLG_DTMF_ULAW, &dtmf->Flags))
				dtmf->buf[dtmf->idx++] =
					isdn_audio_ulaw_to_s16[*p++] >> (15 - AMP_BITS);
			else
				dtmf->buf[dtmf->idx++] =
					isdn_audio_alaw_to_s16[*p++] >> (15 - AMP_BITS);
		}
		if (dtmf->idx == DTMF_NPOINTS) {
			isdn_audio_goertzel(dtmf);
			dtmf->idx = 0;
		}
		len -= c;
	}
}

static void
dtmf_reset(dtmf_t *dtmf)
{
	dtmf->last = ' ';
	dtmf->idx = 0;
}

#ifdef OBSOLETE
static int
dtmf_from_up(mISDNinstance_t *inst, struct sk_buff *skb)
{
	dtmf_t		*dtmf;
	mISDN_head_t	*hh;
	int		*data;
	int		err = 0;

	dtmf = inst->privat;
	hh = mISDN_HEAD_P(skb);
	switch(hh->prim) {
		case (PH_CONTROL | REQUEST):
			if ((hh->dinfo == 0) && (skb->len >= sizeof(int))) {
				data = (int *)skb->data;
				if (dtmf->debug & DEBUG_DTMF_CTRL)
					printk(KERN_DEBUG "DTMF: PH_CONTROL REQ data %04x\n",
						*data);
				if (*data == DTMF_TONE_START) {
					test_and_set_bit(FLG_DTMF_ACTIV, &dtmf->Flags);
					dtmf_reset(dtmf);
					break;
				} else if (*data == DTMF_TONE_STOP) {
					test_and_clear_bit(FLG_DTMF_ACTIV, &dtmf->Flags);
					dtmf_reset(dtmf);
					break;
				}
			}
			/* Fall trough in case of not handled function */
		default:
			return(mISDN_queue_down(inst, 0, skb));
	}
	if (!err)
		dev_kfree_skb(skb);
	return(err);
}
#endif

static int
dtmf_function(mISDNinstance_t *inst,  struct sk_buff *skb)
{
	dtmf_t		*dtmf;
	mISDN_head_t	*hh;

	dtmf = inst->privat;
	hh = mISDN_HEAD_P(skb);
	switch(hh->prim) {
		case (PH_DATA | CONFIRM):
			hh->prim = DL_DATA | CONFIRM;
			break;
		case (DL_DATA_IND):
		case (PH_DATA_IND):
			if (test_bit(FLG_DTMF_ACTIV, &dtmf->Flags))
				isdn_audio_calc_dtmf(dtmf, skb);
			hh->prim = DL_DATA_IND;
			break;
		case (PH_CONTROL | REQUEST):
			if ((hh->dinfo == 0) && (skb->len >= sizeof(int))) {
				int *data = (int *)skb->data;
				if (dtmf->debug & DEBUG_DTMF_CTRL)
					printk(KERN_DEBUG "DTMF: PH_CONTROL REQ data %04x\n",
						*data);
				if (*data == DTMF_TONE_START) {
					test_and_set_bit(FLG_DTMF_ACTIV, &dtmf->Flags);
					dtmf_reset(dtmf);
					dev_kfree_skb(skb);
					return(0);
				} else if (*data == DTMF_TONE_STOP) {
					test_and_clear_bit(FLG_DTMF_ACTIV, &dtmf->Flags);
					dtmf_reset(dtmf);
					dev_kfree_skb(skb);
					return(0);
				}
			}
			break;
		case (PH_ACTIVATE | CONFIRM):
			hh->prim = DL_ESTABLISH | CONFIRM;
			break;
		case (PH_ACTIVATE | INDICATION):
			hh->prim = DL_ESTABLISH | INDICATION;
			break;
		case (PH_DEACTIVATE | CONFIRM):
			hh->prim = DL_RELEASE | CONFIRM;
			break;
		case (PH_DEACTIVATE | INDICATION):
			hh->prim = DL_RELEASE | INDICATION;
			break;
	}
	return(mISDN_queue_message(inst, hh->addr & MSG_DIR_MASK, skb));
}

static void
release_dtmf(dtmf_t *dtmf) {
	mISDNinstance_t	*inst = &dtmf->inst;
	u_long		flags;

	spin_lock_irqsave(&dtmf_obj.lock, flags);
	list_del(&dtmf->list);
	spin_unlock_irqrestore(&dtmf_obj.lock, flags);
	mISDN_ctrl(inst, MGR_UNREGLAYER | REQUEST, NULL);
	kfree(dtmf);
}

static int
new_dtmf(mISDNstack_t *st, mISDN_pid_t *pid) {
	dtmf_t	*n_dtmf;
	int	err;
	u_long	flags;

	if (!st || !pid)
		return(-EINVAL);
	if (!(n_dtmf = kmalloc(sizeof(dtmf_t), GFP_ATOMIC))) {
		printk(KERN_ERR "kmalloc dtmf_t failed\n");
		return(-ENOMEM);
	}
	memset(n_dtmf, 0, sizeof(dtmf_t));
	memcpy(&n_dtmf->inst.pid, pid, sizeof(mISDN_pid_t));
	mISDN_init_instance(&n_dtmf->inst, &dtmf_obj, n_dtmf, dtmf_function);
	if (!mISDN_SetHandledPID(&dtmf_obj, &n_dtmf->inst.pid)) {
		int_error();
		kfree(n_dtmf);
		return(-ENOPROTOOPT);
	}
	n_dtmf->debug = debug;
	spin_lock_irqsave(&dtmf_obj.lock, flags);
	list_add_tail(&n_dtmf->list, &dtmf_obj.ilist);
	spin_unlock_irqrestore(&dtmf_obj.lock, flags);
	err = mISDN_ctrl(st, MGR_REGLAYER | INDICATION, &n_dtmf->inst);
	if (err) {
		list_del(&n_dtmf->list);
		kfree(n_dtmf);
	}
	return(err);
}

#if 0
static int
dtmf_status(dtmf_t *dtmf, status_info_dtmf_t *si)
{

	if (!si)
		return(-EINVAL);
	memset(si, 0, sizeof(status_info_dtmf_t));
	si->len = sizeof(status_info_dtmf_t) - 2*sizeof(int);
	si->typ = STATUS_INFO_L1;
	si->protocol = dtmf->inst.pid.protocol[1];
	if (test_bit(FLG_L1_ACTIVATED, &dtmf->Flags))
		si->status = 1;
	si->state = dtmf->dtmfm.state;
	si->Flags = dtmf->Flags;
	si->T3 = TIMER3_VALUE;
	si->debug = dtmf->delay;
	si->debug = dtmf->debug;
	return(0);
}

#endif

static char MName[] = "DTMF";

#ifdef MODULE
MODULE_AUTHOR("Karsten Keil");
module_param (debug, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC (debug, "dtmf debug mask");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
#endif

static int
dtmf_manager(void *data, u_int prim, void *arg) {
	mISDNinstance_t	*inst = data;
	dtmf_t		*dtmf_l;
	int		ret = -EINVAL;
	u_long		flags;

	if (debug & DEBUG_DTMF_MGR)
		printk(KERN_DEBUG "dtmf_manager data:%p prim:%x arg:%p\n", data, prim, arg);
	if (!data)
		return(ret);
	spin_lock_irqsave(&dtmf_obj.lock, flags);
	list_for_each_entry(dtmf_l, &dtmf_obj.ilist, list) {
		if (&dtmf_l->inst == inst) {
			ret = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&dtmf_obj.lock, flags);
	if (prim == (MGR_NEWLAYER | REQUEST))
		return(new_dtmf(data, arg));
	if (ret) {
		printk(KERN_WARNING "dtmf_manager prim(%x) no instance\n", prim);
		return(ret);
	}
	switch(prim) {
	    case MGR_CLRSTPARA | INDICATION:
		break;
#ifdef OBSOLETE
	    case MGR_CLONELAYER | REQUEST:
		break;
	    case MGR_CONNECT | REQUEST:
		return(mISDN_ConnectIF(inst, arg));
	    case MGR_SETIF | REQUEST:
	    case MGR_SETIF | INDICATION:
		return(mISDN_SetIF(inst, arg, prim, dtmf_from_up, dtmf_from_down, dtmf_l));
	    case MGR_DISCONNECT | REQUEST:
	    case MGR_DISCONNECT | INDICATION:
		return(mISDN_DisConnectIF(inst, arg));
#endif
	    case MGR_UNREGLAYER | REQUEST:
	    case MGR_RELEASE | INDICATION:
		if (debug & DEBUG_DTMF_MGR)
			printk(KERN_DEBUG "release_dtmf id %x\n", dtmf_l->inst.st->id);
		release_dtmf(dtmf_l);
		break;
//	    case MGR_STATUS | REQUEST:
//		return(dtmf_status(dtmf_l, arg));
	    default:
		if (debug & DEBUG_DTMF_MGR)
			printk(KERN_WARNING "dtmf_manager prim %x not handled\n", prim);
		return(-EINVAL);
	}
	return(0);
}

static int dtmf_init(void)
{
	int err;

	printk(KERN_INFO "DTMF modul version %s\n", mISDN_getrev(mISDN_dtmf_revision));
#ifdef MODULE
	dtmf_obj.owner = THIS_MODULE;
#endif
	dtmf_obj.name = MName;
	dtmf_obj.BPROTO.protocol[2] = ISDN_PID_L2_B_TRANSDTMF;
	dtmf_obj.own_ctrl = dtmf_manager;
	spin_lock_init(&dtmf_obj.lock);
	INIT_LIST_HEAD(&dtmf_obj.ilist);
	if ((err = mISDN_register(&dtmf_obj))) {
		printk(KERN_ERR "Can't register %s error(%d)\n", MName, err);
	}
	return(err);
}

static void dtmf_cleanup(void)
{
	int	err;
	dtmf_t	*dtmf, *nd;

	if ((err = mISDN_unregister(&dtmf_obj))) {
		printk(KERN_ERR "Can't unregister DTMF error(%d)\n", err);
	}
	if (!list_empty(&dtmf_obj.ilist)) {
		printk(KERN_WARNING "dtmf inst list not empty\n");
		list_for_each_entry_safe(dtmf, nd, &dtmf_obj.ilist, list)
			release_dtmf(dtmf);
	}
}

module_init(dtmf_init);
module_exit(dtmf_cleanup);
