/* $Id$
 *
 */

#include "hisax_capi.h"
#include "helper.h"
#include "debug.h"
#include "dss1.h"

// --------------------------------------------------------------------
// NCCI state machine

enum {
	ST_NCCI_N_0,
	ST_NCCI_N_0_1,
	ST_NCCI_N_1,
	ST_NCCI_N_2,
	ST_NCCI_N_ACT,
	ST_NCCI_N_3,
	ST_NCCI_N_4,
	ST_NCCI_N_5,
}

const ST_NCCI_COUNT = ST_NCCI_N_5 + 1;

static char *str_st_ncci[] = {
	"ST_NCCI_N_0",
	"ST_NCCI_N_0_1",
	"ST_NCCI_N_1",
	"ST_NCCI_N_2",
	"ST_NCCI_N_ACT",
	"ST_NCCI_N_3",
	"ST_NCCI_N_4",
	"ST_NCCI_N_5",
}; 

enum {
	EV_NCCI_CONNECT_B3_REQ,
	EV_NCCI_CONNECT_B3_CONF,
	EV_NCCI_CONNECT_B3_IND,
	EV_NCCI_CONNECT_B3_RESP,
	EV_NCCI_CONNECT_B3_ACTIVE_IND,
	EV_NCCI_CONNECT_B3_ACTIVE_RESP,
	EV_NCCI_RESET_B3_REQ,
	EV_NCCI_RESET_B3_IND,
	EV_NCCI_CONNECT_B3_T90_ACTIVE_IND,
	EV_NCCI_DISCONNECT_B3_REQ,
	EV_NCCI_DISCONNECT_B3_IND,
	EV_NCCI_DISCONNECT_B3_CONF,
	EV_NCCI_DISCONNECT_B3_RESP,
	EV_NCCI_SELECT_B_PROTOCOL,
	EV_NCCI_DL_ESTABLISH_IND,
	EV_NCCI_DL_ESTABLISH_CONF,
	EV_NCCI_DL_RELEASE_IND,
	EV_NCCI_DL_RELEASE_CONF,
	EV_NCCI_DL_DOWN_IND,
}

const EV_NCCI_COUNT = EV_NCCI_DL_DOWN_IND + 1;

static char* str_ev_ncci[] = {
	"EV_NCCI_CONNECT_B3_REQ",
	"EV_NCCI_CONNECT_B3_CONF",
	"EV_NCCI_CONNECT_B3_IND",
	"EV_NCCI_CONNECT_B3_RESP",
	"EV_NCCI_CONNECT_B3_ACTIVE_IND",
	"EV_NCCI_CONNECT_B3_ACTIVE_RESP",
	"EV_NCCI_RESET_B3_REQ",
	"EV_NCCI_RESET_B3_IND",
	"EV_NCCI_CONNECT_B3_T90_ACTIVE_IND",
	"EV_NCCI_DISCONNECT_B3_REQ",
	"EV_NCCI_DISCONNECT_B3_IND",
	"EV_NCCI_DISCONNECT_B3_CONF",
	"EV_NCCI_DISCONNECT_B3_RESP",
	"EV_NCCI_SELECT_B_PROTOCOL",
	"EV_NCCI_DL_ESTABLISH_IND",
	"EV_NCCI_DL_ESTABLISH_CONF",
	"EV_NCCI_DL_RELEASE_IND",
	"EV_NCCI_DL_RELEASE_CONF",
	"EV_NCCI_DL_DOWN_IND",
};

static struct Fsm ncci_fsm =
{ 0, 0, 0, 0, 0 };

static void ncci_debug(struct FsmInst *fi, char *fmt, ...)
{
	char tmp[128];
	char *p = tmp;
	va_list args;
	Ncci_t *ncci = fi->userdata;
	
	va_start(args, fmt);
	p += sprintf(p, "NCCI 0x%x: ", ncci->adrNCCI);
	p += vsprintf(p, fmt, args);
	*p++ = '\n';
	*p = 0;
	printk(KERN_DEBUG "%s", tmp);
	va_end(args);
}

inline void ncciRecvCmsg(Ncci_t *ncci, _cmsg *cmsg)
{
	contrRecvCmsg(ncci->contr, cmsg);
}

inline void ncciCmsgHeader(Ncci_t *ncci, _cmsg *cmsg, __u8 cmd, __u8 subcmd)
{
	capi_cmsg_header(cmsg, ncci->appl->ApplId, cmd, subcmd, 
			 ncci->appl->MsgId++, ncci->adrNCCI);
}

static void ncci_connect_b3_req(struct FsmInst *fi, int event, void *arg)
{
	Ncci_t *ncci = fi->userdata;
	_cmsg *cmsg = arg;

	FsmChangeState(fi, ST_NCCI_N_0_1);
	capi_cmsg_answer(cmsg);
	cmsg->adr.adrNCCI = ncci->adrNCCI;

	FsmEvent(fi, EV_NCCI_CONNECT_B3_CONF, cmsg);
	ncciRecvCmsg(ncci, cmsg);

	if (cmsg->Info < 0x1000) 
		L4L3(ncci, DL_ESTABLISH | REQUEST, 0);
}

static void ncci_connect_b3_ind(struct FsmInst *fi, int event, void *arg)
{
	FsmChangeState(fi, ST_NCCI_N_1);
}

static void ncci_connect_b3_resp(struct FsmInst *fi, int event, void *arg)
{
	Ncci_t *ncci = fi->userdata;
	_cmsg *cmsg = arg;
  
	if (cmsg->Info == 0) {
		FsmChangeState(fi, ST_NCCI_N_2);
		ncciCmsgHeader(ncci, cmsg, CAPI_CONNECT_B3_ACTIVE, CAPI_IND);
		FsmEvent(&ncci->ncci_m, EV_NCCI_CONNECT_B3_ACTIVE_IND, cmsg);
		ncciRecvCmsg(ncci, cmsg);
	} else {
		FsmChangeState(fi, ST_NCCI_N_4);
		ncciCmsgHeader(ncci, cmsg, CAPI_DISCONNECT_B3, CAPI_IND);
		FsmEvent(&ncci->ncci_m, EV_NCCI_DISCONNECT_B3_IND, cmsg);
		ncciRecvCmsg(ncci, cmsg);
	}
}

static void ncci_connect_b3_conf(struct FsmInst *fi, int event, void *arg)
{
	_cmsg *cmsg = arg;
  
	if (cmsg->Info == 0) {
		FsmChangeState(fi, ST_NCCI_N_2);
	} else {
		FsmChangeState(fi, ST_NCCI_N_0);
	}
}

static void ncci_disconnect_b3_req(struct FsmInst *fi, int event, void *arg)
{
	Ncci_t *ncci = fi->userdata;
	_cmsg *cmsg = arg;
	__u16 Info = 0;
	int saved_state = fi->state;
	
	FsmChangeState(fi, ST_NCCI_N_4);

	capi_cmsg_answer(cmsg);
	cmsg->Info = Info;
	FsmEvent(fi, EV_NCCI_DISCONNECT_B3_CONF, cmsg);
	if (cmsg->Info != 0) {
		FsmChangeState(fi, saved_state);
		ncciRecvCmsg(ncci, cmsg);
	} else {
		ncciRecvCmsg(ncci, cmsg);
		L4L3(ncci, DL_RELEASE | REQUEST, 0);
	}
}

static void ncci_connect_b3_active_ind(struct FsmInst *fi, int event, void *arg)
{
	Ncci_t *ncci = fi->userdata;
	int i;

	FsmChangeState(fi, ST_NCCI_N_ACT);
	for (i = 0; i < CAPI_MAXDATAWINDOW; i++) {
		ncci->xmit_skb_handles[i].skb = 0;
		ncci->recv_skb_handles[i] = 0;
	}
}

static void ncci_connect_b3_active_resp(struct FsmInst *fi, int event, void *arg)
{
}

static void ncci_disconnect_b3_conf(struct FsmInst *fi, int event, void *arg)
{
}

static void ncci_disconnect_b3_ind(struct FsmInst *fi, int event, void *arg)
{
	FsmChangeState(fi, ST_NCCI_N_5);
}

static void ncci_disconnect_b3_resp(struct FsmInst *fi, int event, void *arg)
{
	FsmChangeState(fi, ST_NCCI_N_0);
}

static void ncci_n0_dl_establish_ind_conf(struct FsmInst *fi, int event, void *arg)
{
	Ncci_t *ncci = fi->userdata;
	_cmsg cmsg;

	ncciCmsgHeader(ncci, &cmsg, CAPI_CONNECT_B3, CAPI_IND);
	FsmEvent(&ncci->ncci_m, EV_NCCI_CONNECT_B3_IND, &cmsg);
	ncciRecvCmsg(ncci, &cmsg);
}

static void ncci_dl_establish_conf(struct FsmInst *fi, int event, void *arg)
{
	Ncci_t *ncci = fi->userdata;
	_cmsg cmsg;

	ncciCmsgHeader(ncci, &cmsg, CAPI_CONNECT_B3_ACTIVE, CAPI_IND);
	FsmEvent(&ncci->ncci_m, EV_NCCI_CONNECT_B3_ACTIVE_IND, &cmsg);
	ncciRecvCmsg(ncci, &cmsg);
}

static void ncci_dl_release_ind_conf(struct FsmInst *fi, int event, void *arg)
{
	Ncci_t *ncci = fi->userdata;
	_cmsg cmsg;

	ncciCmsgHeader(ncci, &cmsg, CAPI_DISCONNECT_B3, CAPI_IND);
	FsmEvent(&ncci->ncci_m, EV_NCCI_DISCONNECT_B3_IND, &cmsg);
	ncciRecvCmsg(ncci, &cmsg);
}

static void ncci_dl_down_ind(struct FsmInst *fi, int event, void *arg)
{
	Ncci_t *ncci = fi->userdata;
	_cmsg cmsg;

	ncciCmsgHeader(ncci, &cmsg, CAPI_DISCONNECT_B3, CAPI_IND);
	cmsg.Reason_B3 = CapiProtocolErrorLayer1;
	FsmEvent(&ncci->ncci_m, EV_NCCI_DISCONNECT_B3_IND, &cmsg);
	ncciRecvCmsg(ncci, &cmsg);
}

static void ncci_select_b_protocol(struct FsmInst *fi, int event, void *arg)
{
	Ncci_t *ncci = fi->userdata;

//	if (ncci->l4.st)
//		ncciReleaseSt(ncci);
	ncciInitSt(ncci);
}

static struct FsmNode fn_ncci_list[] =
{
  {ST_NCCI_N_0,       EV_NCCI_CONNECT_B3_REQ,            ncci_connect_b3_req},
  {ST_NCCI_N_0,       EV_NCCI_CONNECT_B3_IND,            ncci_connect_b3_ind},
  {ST_NCCI_N_0,       EV_NCCI_SELECT_B_PROTOCOL,         ncci_select_b_protocol},
  {ST_NCCI_N_0,       EV_NCCI_DL_ESTABLISH_CONF,         ncci_n0_dl_establish_ind_conf},
  {ST_NCCI_N_0,       EV_NCCI_DL_ESTABLISH_IND,          ncci_n0_dl_establish_ind_conf},

  {ST_NCCI_N_0_1,     EV_NCCI_CONNECT_B3_CONF,           ncci_connect_b3_conf},

  {ST_NCCI_N_1,       EV_NCCI_CONNECT_B3_RESP,           ncci_connect_b3_resp},
  {ST_NCCI_N_1,       EV_NCCI_DISCONNECT_B3_REQ,         ncci_disconnect_b3_req},
  {ST_NCCI_N_1,       EV_NCCI_DISCONNECT_B3_IND,         ncci_disconnect_b3_ind},

  {ST_NCCI_N_2,       EV_NCCI_CONNECT_B3_ACTIVE_IND,     ncci_connect_b3_active_ind},
  {ST_NCCI_N_2,       EV_NCCI_DISCONNECT_B3_REQ,         ncci_disconnect_b3_req},
  {ST_NCCI_N_2,       EV_NCCI_DISCONNECT_B3_IND,         ncci_disconnect_b3_ind},
  {ST_NCCI_N_2,       EV_NCCI_DL_ESTABLISH_CONF,         ncci_dl_establish_conf},
  {ST_NCCI_N_2,       EV_NCCI_DL_RELEASE_IND,            ncci_dl_release_ind_conf},
     
  {ST_NCCI_N_ACT,     EV_NCCI_CONNECT_B3_ACTIVE_RESP,    ncci_connect_b3_active_resp},
  {ST_NCCI_N_ACT,     EV_NCCI_DISCONNECT_B3_REQ,         ncci_disconnect_b3_req},
  {ST_NCCI_N_ACT,     EV_NCCI_DISCONNECT_B3_IND,         ncci_disconnect_b3_ind},
  {ST_NCCI_N_ACT,     EV_NCCI_DL_RELEASE_IND,            ncci_dl_release_ind_conf},
  {ST_NCCI_N_ACT,     EV_NCCI_DL_DOWN_IND,               ncci_dl_down_ind},

  {ST_NCCI_N_4,       EV_NCCI_DISCONNECT_B3_CONF,        ncci_disconnect_b3_conf},
  {ST_NCCI_N_4,       EV_NCCI_DISCONNECT_B3_IND,         ncci_disconnect_b3_ind},
  {ST_NCCI_N_4,       EV_NCCI_DL_RELEASE_CONF,           ncci_dl_release_ind_conf},
  {ST_NCCI_N_4,       EV_NCCI_DL_DOWN_IND,               ncci_dl_down_ind},

  {ST_NCCI_N_5,       EV_NCCI_DISCONNECT_B3_RESP,        ncci_disconnect_b3_resp},

#if 0
  {ST_NCCI_N_ACT,     EV_NCCI_RESET_B3_REQ,              ncci_reset_b3_req},
  {ST_NCCI_N_ACT,     EV_NCCI_RESET_B3_IND,              ncci_reset_b3_ind},
  {ST_NCCI_N_ACT,     EV_NCCI_CONNECT_B3_T90_ACTIVE_IND, ncci_connect_b3_t90_active_ind},

  {ST_NCCI_N_3,       EV_NCCI_RESET_B3_IND,              ncci_reset_b3_ind},
  {ST_NCCI_N_3,       EV_NCCI_DISCONNECT_B3_REQ,         ncci_disconnect_b3_req},
  {ST_NCCI_N_3,       EV_NCCI_DISCONNECT_B3_IND,         ncci_disconnect_b3_ind},
#endif
};

const int FN_NCCI_COUNT = sizeof(fn_ncci_list)/sizeof(struct FsmNode);

static int ncci_l3l4(hisaxif_t *hif, u_int prim, u_int nr, int dtyp, void *arg);

void ncciConstr(Ncci_t *ncci, Cplci_t *cplci)
{
	memset(ncci, 0, sizeof(Ncci_t));

	ncci->ncci_m.fsm        = &ncci_fsm;
	ncci->ncci_m.state      = ST_NCCI_N_0;
	ncci->ncci_m.debug      = 1;
	ncci->ncci_m.userdata   = ncci;
	ncci->ncci_m.printdebug = ncci_debug;

//	ncci->l4.l3l4 = ncci_l3l4st;
	ncci->adrNCCI = 0x10000 | cplci->adrPLCI;
	ncci->cplci = cplci;
	ncci->contr = cplci->contr;
	ncci->appl = cplci->appl;
	ncci->window = cplci->appl->rp.datablkcnt;
	if (ncci->window > CAPI_MAXDATAWINDOW) {
		ncci->window = CAPI_MAXDATAWINDOW;
	}


}

void ncciInitSt(Ncci_t *ncci)
{
#if 0
	struct StackParams sp;
	int bchannel, retval;
	Cplci_t *cplci = ncci->cplci;

	ncci->l4.st = kmalloc(sizeof(struct PStack), GFP_ATOMIC);
	if (!ncci->l4.st) {
		int_error();
		return;
	}
	memset(ncci->l4.st, 0, sizeof(struct PStack));
	bchannel = cplci->plci->l4_pc.l3pc->para.bchannel - 1;
	sp.b1_mode = cplci->Bprotocol.B1protocol;
	sp.b2_mode = cplci->Bprotocol.B2protocol;
	sp.b3_mode = cplci->Bprotocol.B3protocol;
	sp.headroom = 22; // reserve space for DATA_B3 IND message in skb's
	retval = init_st(&ncci->l4, cplci->contr->cs, &sp, bchannel);
	if (retval) {
		int_error();
		return;
	}
	if (sp.b2_mode != B2_MODE_TRANS || !test_bit(PLCI_FLAG_OUTGOING, &cplci->plci->flags)) {
		// listen for e.g. SABME
		L4L3(&ncci->l4, PH_ACTIVATE | REQUEST, 0);
	}
#endif
}

void ncciReleaseSt(Ncci_t *ncci)
{
#if 0
 	if (!ncci->l4.st) {
 		int_error();
		return;
	}
	release_st(ncci->l4.st);
	kfree(ncci->l4.st);
	ncci->l4.st = 0;
#endif
}

void ncciLinkUp(Ncci_t *ncci)
{
	ncci->contr->ctrl->new_ncci(ncci->contr->ctrl, ncci->appl->ApplId, 
				    ncci->adrNCCI, ncci->window);
	ncciInitSt(ncci);
}

void ncciLinkDown(Ncci_t *ncci)
{
#if 0
	if (ncci->l4.st)
		ncciReleaseSt(ncci);
	FsmEvent(&ncci->ncci_m, EV_NCCI_DL_DOWN_IND, 0);
#endif	
}

__u16 ncciSelectBprotocol(Ncci_t *ncci)
{
	int retval;
	retval = FsmEvent(&ncci->ncci_m, EV_NCCI_SELECT_B_PROTOCOL, 0);
	if (retval)
		return CapiMessageNotSupportedInCurrentState;
	return CapiSuccess;
}

void ncciDestr(Ncci_t *ncci)
{
#if 0
	if (ncci->l4.st)
		ncciReleaseSt(ncci);
#endif
	if (ncci->appl)
		ncci->contr->ctrl->free_ncci(ncci->contr->ctrl, 
					     ncci->appl->ApplId, ncci->adrNCCI);
}

void ncciDataInd(Ncci_t *ncci, int pr, void *arg)
{
	struct sk_buff *skb, *nskb;
	int i;

	skb = arg;

	for (i = 0; i < CAPI_MAXDATAWINDOW; i++) {
		if (ncci->recv_skb_handles[i] == 0)
			break;
	}

	if (i == CAPI_MAXDATAWINDOW) {
		// FIXME: trigger flow control if supported by L2 protocol
		printk(KERN_INFO "HiSax: frame dropped\n");
		dev_kfree_skb(skb);
		return;
	}

	if (skb_headroom(skb) < 22) {
		printk(KERN_DEBUG "HiSax: only %d bytes headroom, need %d\n",
		       skb_headroom(skb), 22);
		nskb = skb_realloc_headroom(skb, 22);
		dev_kfree_skb(skb);
		if (!nskb) {
			int_error();
			return;
		}
      	} else { 
		nskb = skb;
	}

	ncci->recv_skb_handles[i] = nskb;
	
	skb_push(nskb, 22);
	*((__u16*) nskb->data) = 22;
	*((__u16*)(nskb->data+2)) = ncci->appl->ApplId;
	*((__u8*) (nskb->data+4)) = CAPI_DATA_B3;
	*((__u8*) (nskb->data+5)) = CAPI_IND;
	*((__u16*)(nskb->data+6)) = ncci->appl->MsgId++;
	*((__u32*)(nskb->data+8)) = ncci->adrNCCI;
	*((__u32*)(nskb->data+12)) = (__u32)(nskb->data + 22);
	*((__u16*)(nskb->data+16)) = nskb->len - 22;
	*((__u16*)(nskb->data+18)) = i;
	*((__u16*)(nskb->data+20)) = 0;
	ncci->contr->ctrl->handle_capimsg(ncci->contr->ctrl, 
					  ncci->appl->ApplId, nskb);
}

void ncciDataReq(Ncci_t *ncci, struct sk_buff *skb)
{
	int i;
	
	if (CAPIMSG_LEN(skb->data) != 22 && CAPIMSG_LEN(skb->data) != 30) {
		int_error();
		goto fail;
	}
	for (i = 0; i < ncci->window; i++) {
		if (ncci->xmit_skb_handles[i].skb == 0)
			break;
	}
	if (i == ncci->window) {
		goto fail;
	}

	ncci->xmit_skb_handles[i].skb = skb;
	ncci->xmit_skb_handles[i].DataHandle = CAPIMSG_REQ_DATAHANDLE(skb->data);
	ncci->xmit_skb_handles[i].MsgId = CAPIMSG_MSGID(skb->data);

	skb_pull(skb, CAPIMSG_LEN(skb->data));
	L4L3(ncci, DL_DATA | REQUEST, skb);
	return;

 fail:
	dev_kfree_skb(skb);
}

void ncciDataConf(Ncci_t *ncci, int pr, void *arg)
{
	struct sk_buff *skb = arg;
	_cmsg cmsg;
	int i;

	for (i = 0; i < ncci->window; i++) {
		if (ncci->xmit_skb_handles[i].skb == skb)
			break;
	}
	if (i == ncci->window) {
		int_error();
		return;
	}
	ncci->xmit_skb_handles[i].skb = 0;
	capi_cmsg_header(&cmsg, ncci->cplci->appl->ApplId, CAPI_DATA_B3, CAPI_CONF, 
			 ncci->xmit_skb_handles[i].MsgId, ncci->adrNCCI);
	cmsg.DataHandle = ncci->xmit_skb_handles[i].DataHandle;
	cmsg.Info = 0;
	ncciRecvCmsg(ncci, &cmsg);
	ncci->xmit_skb_handles[i].skb = 0;
}	
	
void ncciDataResp(Ncci_t *ncci, struct sk_buff *skb)
{
	// FIXME: incoming flow control doesn't work yet

	int i;

	i = CAPIMSG_RESP_DATAHANDLE(skb->data);
	if (i < 0 || i > CAPI_MAXDATAWINDOW) {
		int_error();
		return;
	}
	if (!ncci->recv_skb_handles[i]) {
		int_error();
		return;
	}
	ncci->recv_skb_handles[i] = 0;

	dev_kfree_skb(skb);
}

void ncciSendMessage(Ncci_t *ncci, struct sk_buff *skb)
{
	int retval = 0;
	_cmsg cmsg;

	// we're not using the Fsm for DATA_B3 for performance reasons
	switch (CAPICMD(CAPIMSG_COMMAND(skb->data), CAPIMSG_SUBCOMMAND(skb->data))) {
	case CAPI_DATA_B3_REQ:
		if (ncci->ncci_m.state == ST_NCCI_N_ACT) {
			ncciDataReq(ncci, skb);
		} else {
			contrAnswerMessage(ncci->cplci->contr, skb, 
					   CapiMessageNotSupportedInCurrentState);
			dev_kfree_skb(skb);
		}
		goto out;
	case CAPI_DATA_B3_RESP:
		ncciDataResp(ncci, skb);
		goto out;
	}

	capi_message2cmsg(&cmsg, skb->data);
	switch (CMSGCMD(&cmsg)) {
	case CAPI_CONNECT_B3_REQ:
		retval = FsmEvent(&ncci->ncci_m, EV_NCCI_CONNECT_B3_REQ, &cmsg);
		break;
	case CAPI_CONNECT_B3_RESP:
		retval = FsmEvent(&ncci->ncci_m, EV_NCCI_CONNECT_B3_RESP, &cmsg);
		break;
	case CAPI_CONNECT_B3_ACTIVE_RESP:
		retval = FsmEvent(&ncci->ncci_m, EV_NCCI_CONNECT_B3_ACTIVE_RESP, &cmsg);
		break;
	case CAPI_DISCONNECT_B3_REQ:
		retval = FsmEvent(&ncci->ncci_m, EV_NCCI_DISCONNECT_B3_REQ, &cmsg);
		break;
	case CAPI_DISCONNECT_B3_RESP:
		retval = FsmEvent(&ncci->ncci_m, EV_NCCI_DISCONNECT_B3_RESP, &cmsg);
		break;
	default:
		int_error();
		retval = -1;
	}
	if (retval) { 
		if (CAPIMSG_SUBCOMMAND(skb->data) == CAPI_REQ) {
			contrAnswerMessage(ncci->cplci->contr, skb, 
					   CapiMessageNotSupportedInCurrentState);
		}
	}
	dev_kfree_skb(skb);
 out:
}

#if 0
void ncci_l3l4st(struct PStack *st, int pr, void *arg)
{
	Ncci_t *ncci = (Ncci_t *)st->l4;

	ncci_l3l4(ncci, pr, arg);
}
#endif

static int ncci_l3l4(hisaxif_t *hif, u_int prim, u_int nr, int dtyp, void *arg)
{
	Ncci_t *ncci;
	struct sk_buff *skb = arg;

	if (!hif || !hif->fdata)
		return(-EINVAL);
	ncci = hif->fdata;

	switch (prim) {
		// we're not using the Fsm for DL_DATA for performance reasons
		case DL_DATA | INDICATION: 
			if (ncci->ncci_m.state == ST_NCCI_N_ACT) {
				ncciDataInd(ncci, prim, arg);
			} else {
				dev_kfree_skb(skb);
			}
			break;
		case DL_DATA | CONFIRM: 
			if (ncci->ncci_m.state == ST_NCCI_N_ACT) {
				ncciDataConf(ncci, prim, arg);
			}
			break;
		case DL_ESTABLISH | INDICATION:
			FsmEvent(&ncci->ncci_m, EV_NCCI_DL_ESTABLISH_IND, arg);
			break;
		case DL_ESTABLISH | CONFIRM:
			FsmEvent(&ncci->ncci_m, EV_NCCI_DL_ESTABLISH_CONF, arg);
			break;
		case DL_RELEASE | INDICATION:
			FsmEvent(&ncci->ncci_m, EV_NCCI_DL_RELEASE_IND, arg);
			break;
		case DL_RELEASE | CONFIRM:
			FsmEvent(&ncci->ncci_m, EV_NCCI_DL_RELEASE_CONF, arg);
			break;
		default:
			printk("pr %#x\n", prim);
			int_error();
	}
	return(0);
}

void init_ncci(void)
{
	ncci_fsm.state_count = ST_NCCI_COUNT;
	ncci_fsm.event_count = EV_NCCI_COUNT;
	ncci_fsm.strEvent = str_ev_ncci;
	ncci_fsm.strState = str_st_ncci;
	
	FsmNew(&ncci_fsm, fn_ncci_list, FN_NCCI_COUNT);
}

