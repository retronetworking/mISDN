/* $Id$
 *
 * Author       Karsten Keil (keil@isdn4linux.de)
 *
 * This file is (c) under GNU PUBLIC LICENSE
 *
 */

extern void vmISDN_debug(int id, char *head, char *fmt, va_list args);
extern void mISDN_debug(int id, char *head, char *fmt, ...);
extern char * mISDN_getrev(const char *revision);
extern void mISDN_debugprint(mISDNinstance_t *inst, char *fmt, ...);
extern int mISDN_QuickHex(char *, u_char *, int);
