/* $Id$
 *
 * Author       Karsten Keil (keil@isdn4linux.de)
 *
 * This file is (c) under GNU PUBLIC LICENSE
 *
 */

extern void vhisaxdebug(int id, char *head, char *fmt, va_list args);
extern void hisaxdebug(int id, char *head, char *fmt, ...);
extern char * HiSax_getrev(const char *revision);
extern void debugprint(hisaxinstance_t *inst, char *fmt, ...);
extern int QuickHex(char *, u_char *, int);
