/* $Id$
 *
 * Author       Karsten Keil (keil@isdn4linux.de)
 *
 * This file is (c) under GNU PUBLIC LICENSE
 *
 */

extern void vmISDNdebug(int id, char *head, char *fmt, va_list args);
extern void mISDNdebug(int id, char *head, char *fmt, ...);
extern char * mISDN_getrev(const char *revision);
extern void debugprint(mISDNinstance_t *inst, char *fmt, ...);
extern int QuickHex(char *, u_char *, int);
