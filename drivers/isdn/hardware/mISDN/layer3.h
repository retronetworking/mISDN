/* $Id$
 *
 * This file is (c) under GNU PUBLIC LICENSE
 *
 */

#define SBIT(state) (1<<state)
#define ALL_STATES  0x03ffffff

#define PROTO_DIS_EURO	0x08

#define L3_DEB_WARN	0x01
#define L3_DEB_PROTERR	0x02
#define L3_DEB_STATE	0x04
#define L3_DEB_CHARGE	0x08
#define L3_DEB_CHECK	0x10
#define L3_DEB_SI	0x20

#define FLG_L2BLOCK     1

#define MAX_NR_LEN		32

typedef struct _cause {
	u_char	len __attribute__ ((packed));
	u_char	loc __attribute__ ((packed));
	u_char	rec __attribute__ ((packed));
	u_char	val __attribute__ ((packed));
	u_char	diag[28] __attribute__ ((packed));
} cause_t;

typedef struct _channel {
	u_char	len __attribute__ ((packed));
	u_char	chan __attribute__ ((packed));
	u_char	spare[6] __attribute__ ((packed));
} channel_t;

typedef struct _setup {
	u_char	sending_cmpl __attribute__ ((packed));
	channel_t channel __attribute__ ((packed));
	u_char	bc[16] __attribute__ ((packed));
	u_char	calling_nr[MAX_NR_LEN] __attribute__ ((packed));
	u_char	calling_sub[MAX_NR_LEN] __attribute__ ((packed));
	u_char	called_nr[MAX_NR_LEN] __attribute__ ((packed));
	u_char	called_sub[MAX_NR_LEN] __attribute__ ((packed));
	u_char	llc[16] __attribute__ ((packed));
	u_char  hlc[8] __attribute__ ((packed));
} setup_t;

typedef struct _L3Timer {
	struct _l3_process	*pc;
	struct timer_list	tl;
	int			event;
} L3Timer_t;

typedef struct _l3_process {
	struct _l3_process	*prev;
	struct _l3_process	*next;
	struct _layer3		*l3;
	int			callref;
	int			state;
	L3Timer_t		timer;
	int			N303;
	int			debug;
	union {
		setup_t		setup;
		channel_t	channel;
		cause_t		cause;
	} para;
} l3_process_t;

typedef struct _layer3 {
	struct _layer3	*prev;
	struct _layer3	*next;
	struct FsmInst	l3m;
	struct FsmTimer	l3m_timer;
	l3_process_t	*proc;
	l3_process_t	*global;
	int		(*p_down)(struct _layer3 *, int, l3_process_t *);
	int		N303;
	int		debug;
	u_int		Flag;
	u_int		msgnr;
	u_int		last_nr;
	u_int		ph_nr;
	hisaxinstance_t	inst;
	struct sk_buff	*ph_skb;
	struct sk_buff_head squeue;
} layer3_t;

struct stateentry {
	int state;
	int primitive;
	void (*rout) (l3_process_t *, u_char, void *);
};

extern void l3_msg(layer3_t *, int, void *);
extern struct sk_buff *l3_alloc_skb(int);
extern void newl3state(l3_process_t *, int);
extern void L3InitTimer(l3_process_t *, L3Timer_t *);
extern void L3DelTimer(L3Timer_t *);
extern int L3AddTimer(L3Timer_t *, int, int);
extern void StopAllL3Timer(l3_process_t *);
extern void release_l3_process(l3_process_t *);
extern l3_process_t *new_l3_process(layer3_t *, int);
extern l3_process_t *getl3proc(layer3_t *, int);
extern u_char *findie(u_char *, int, u_char, int);
extern int hisax_l3up(l3_process_t *, u_int, void *);
extern int getcallref(u_char * p);
extern int newcallref(void);

/*
#define l3_debug(st, fmt, args...) HiSax_putstatus(st->l1.hardware, "l3 ", fmt, ## args)
*/
