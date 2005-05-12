/* $Id$
 *
 * Author       Karsten Keil (keil@isdn4linux.de)
 *
 * This file is (c) under GNU PUBLIC LICENSE
 *
 */

#include "core.h"

LIST_HEAD(mISDN_stacklist);
LIST_HEAD(mISDN_instlist);

int
get_stack_cnt(void)
{
	int cnt = 0;
	mISDNstack_t *st;

	list_for_each_entry(st, &mISDN_stacklist, list)
		cnt++;
	return(cnt);
}

void
get_stack_info(struct sk_buff *skb)
{
	mISDN_head_t	*hp;
	mISDNstack_t	*cst, *st;
	stack_info_t	*si;
	int		i;

	hp = mISDN_HEAD_P(skb);
	st = get_stack4id(hp->addr);
	if (!st)
		hp->len = 0;
	else {
		si = (stack_info_t *)skb->data;
		memset(si, 0, sizeof(stack_info_t));
		si->id = st->id;
		si->extentions = st->extentions;
		if (st->mgr)
			si->mgr = st->mgr->id;
		else
			si->mgr = 0;
		memcpy(&si->pid, &st->pid, sizeof(mISDN_pid_t));
		memcpy(&si->para, &st->para, sizeof(mISDN_stPara_t));
		si->instcnt = 0;
		for (i = 0; i <= MAX_LAYER_NR; i++) {
			if (st->i_array[i]) {
				si->inst[si->instcnt] = st->i_array[i]->id;
				si->instcnt++;
			}
		}
		si->childcnt = 0;
		list_for_each_entry(cst, &st->childlist, list) {
			si->child[si->childcnt] = cst->id;
			si->childcnt++;
		}
		hp->len = sizeof(stack_info_t);
		if (si->childcnt>2)
			hp->len += (si->childcnt-2)*sizeof(int);
	}
	skb_put(skb, hp->len);
}

static int
get_free_stackid(mISDNstack_t *mst, int flag)
{
	u_int		id = 0, found;
	mISDNstack_t	*st;

	if (!mst) {
		while(id < STACK_ID_MAX) {
			found = 0;
			id += STACK_ID_INC;
			list_for_each_entry(st, &mISDN_stacklist, list) {
				if (st->id == id) {
					found++;
					break;
				}
			}
			if (!found)
				return(id);
		}
	} else if (flag & FLG_CLONE_STACK) {
		id = mst->id | FLG_CLONE_STACK;
		while(id < CLONE_ID_MAX) {
			found = 0;
			id += CLONE_ID_INC;
			list_for_each_entry(st, &mISDN_stacklist, list) {
				if (st->id == id) {
					found++;
					break;
				}
			}
			if (!found)
				return(id);
		}
	} else if (flag & FLG_CHILD_STACK) {
		id = mst->id | FLG_CHILD_STACK;
		while(id < CHILD_ID_MAX) {
			id += CHILD_ID_INC;
			found = 0;
			list_for_each_entry(st, &mst->childlist, list) {
				if (st->id == id) {
					found++;
					break;
				}
			}
			if (!found)
				return(id);
		}
	}
	return(0);
}

mISDNstack_t *
get_stack4id(u_int id)
{
	mISDNstack_t *cst, *st;

	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "get_stack4id(%x)\n", id);
	if (!id) /* 0 isn't a valid id */
		return(NULL);
	list_for_each_entry(st, &mISDN_stacklist, list) {	
		if (id == st->id)
			return(st);
		list_for_each_entry(cst, &st->childlist, list) {
			if (cst->id == id)
				return(cst);
		}
	}
	return(NULL);
}

mISDNinstance_t *
getlayer4lay(mISDNstack_t *st, int layermask)
{
	int	i;

	if (!st) {
		int_error();
		return(NULL);
	}
	for (i = 0; i <= MAX_LAYER_NR; i++) {
		if (st->i_array[i] && (st->i_array[i]->pid.layermask & layermask))
			return(st->i_array[i]);
	}
	return(NULL);
}

static mISDNinstance_t *
get_nextlayer(mISDNstack_t *st, u_int addr)
{
	mISDNinstance_t	*inst=NULL;
	int		layer = addr & LAYER_ID_MASK;

	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "%s: st(%08x) addr(%08x)\n", __FUNCTION__, st->id, addr);

	if (!(addr & FLG_MSG_TARGET)) {
		switch(addr & MSG_DIR_MASK) {
			case FLG_MSG_DOWN:
				if (addr & FLG_MSG_CLONED) {
					
				} else
					layer -= LAYER_ID_INC;
				break;
			case FLG_MSG_UP:
				layer += LAYER_ID_INC;
				break;
			case MSG_TO_OWNER:
				break;
			default: /* broadcast */
				int_errtxt("st(%08x) addr(%08x) wrong address", st->id, addr);
				return(NULL);
		}
	}	
	if ((layer < 0) || (layer > MAX_LAYER_NR)) {
		int_errtxt("st(%08x) addr(%08x) layer %d out of range", st->id, addr, layer);
		return(NULL);
	}
	inst = st->i_array[layer];
	/* maybe more checks */
	return(inst);
}

mISDNinstance_t *
get_instance(mISDNstack_t *st, int layer_nr, int protocol)
{
	mISDNinstance_t	*inst=NULL;
	int		i;

	if (!st) {
		int_error();
		return(NULL);
	}
	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "get_instance st(%08x) lnr(%d) prot(%x)\n",
			st->id, layer_nr, protocol);
	if ((layer_nr < 0) || (layer_nr > MAX_LAYER_NR)) {
		int_errtxt("lnr %d", layer_nr);
		return(NULL);
	}
	list_for_each_entry(inst, &st->prereg, list) {
		if (core_debug & DEBUG_CORE_FUNC)
			printk(KERN_DEBUG "get_instance prereg(%p, %x) lm %x/%x prot %x/%x\n",
				inst, inst->id, inst->pid.layermask, ISDN_LAYER(layer_nr),
				inst->pid.protocol[layer_nr], protocol);
		if ((inst->pid.layermask & ISDN_LAYER(layer_nr)) &&
			(inst->pid.protocol[layer_nr] == protocol)) {
			list_del_init(&inst->list);
			i = register_layer(st, inst);
			if (i) {
				int_errtxt("error(%d) register preregistered inst(%08x) on st(%08x)", i, inst->id, st->id);
				return(NULL);
			}
			return(inst);
		}
	}
	for (i = 0; i <= MAX_LAYER_NR; i++) {
		if ((inst = st->i_array[i])) {
			if (core_debug & DEBUG_CORE_FUNC)
				printk(KERN_DEBUG "get_instance inst%d(%p, %x) lm %x/%x prot %x/%x\n",
					i,inst, inst->id, inst->pid.layermask, ISDN_LAYER(layer_nr),
					inst->pid.protocol[layer_nr], protocol);
			if ((inst->pid.layermask & ISDN_LAYER(layer_nr)) &&
				(inst->pid.protocol[layer_nr] == protocol))
				return(inst);
		}
	}
	return(NULL);
}

mISDNinstance_t *
get_instance4id(u_int id)
{
	mISDNinstance_t *inst;

	list_for_each_entry(inst, &mISDN_instlist, list)
		if (inst->id == id)
			return(inst);
	return(NULL);
}

#ifdef OBSOLATE
int
get_layermask(mISDNlayer_t *layer)
{
	int mask = 0;

	if (layer->inst)
		mask |= layer->inst->pid.layermask;
	return(mask);
}

int
insertlayer(mISDNstack_t *st, mISDNlayer_t *layer, int layermask)
{
	mISDNlayer_t *item;
	
	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "%s(%p, %p, %x)\n",
			__FUNCTION__, st, layer, layermask);  
	if (!st || !layer) {
		int_error();
		return(-EINVAL);
	}
	if (list_empty(&st->layerlist)) {
		list_add(&layer->list, &st->layerlist);
	} else {
		list_for_each_entry(item, &st->layerlist, list) {
			if (layermask < get_layermask(item)) {
				list_add_tail(&layer->list, &item->list); 
				return(0);
			}
		}
		list_add_tail(&layer->list, &st->layerlist);
	}
	return(0);
}
#endif

int
mISDN_queue_message(mISDNinstance_t *inst, u_int aflag, struct sk_buff *skb)
{
	mISDN_head_t	*hh = mISDN_HEAD_P(skb);
	mISDNstack_t	*st = inst->st;
	u_int		id;

	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "%s(%08x, %x, prim(%x))\n", __FUNCTION__,
			inst->id, aflag, hh->prim);
	if (aflag & FLG_MSG_TARGET) {
		id = aflag;
	} else {
		id = (inst->id & INST_ID_MASK) | aflag;
	}
	if ((aflag & MSG_DIR_MASK) == FLG_MSG_DOWN) {
		if (inst->parent) {
			st = inst->parent->st;
			id = id | FLG_MSG_CLONED;
		}
	}
	if (!st)
		return(-EINVAL);
	if (st->id == 0 || test_bit(mISDN_STACK_ABORT, &st->status))
		return(-EBUSY);
	if (test_bit(mISDN_STACK_KILLED, &st->status))
		return(-EBUSY);
	if (test_bit(mISDN_STACK_STOPPED, &st->status))
		return(-EBUSY);
	if ((st->id & STACK_ID_MASK) != (id & STACK_ID_MASK)) {
		int_errtxt("stack id not match st(%08x) id(%08x)", st->id, id);
	}
	hh->addr = id;
	skb_queue_tail(&st->msgq, skb);
	wake_up_interruptible(&st->workq);
	return(0);
}

static void
do_broadcast(mISDNstack_t *st, struct sk_buff *skb)
{
	mISDN_head_t	*hh = mISDN_HEAD_P(skb);

	/* not implemented yet */
	int_errtxt("%s: st(%08x) addr(%08x) prim(%x) dinfo(%x)", __FUNCTION__, st->id, hh->addr, hh->prim, hh->dinfo);
	dev_kfree_skb(skb);
}

static int
mISDNStackd(void *data)
{
	mISDNstack_t	*st = data;
	int		err = 0;

#ifdef CONFIG_SMP
	lock_kernel();
#endif
	MAKEDAEMON("mISDNStackd");
	sigfillset(&current->blocked);
	st->thread = current;
#ifdef CONFIG_SMP
	unlock_kernel();
#endif
	printk(KERN_DEBUG "mISDNStackd started for id(%08x)\n", st->id);
	test_and_clear_bit(mISDN_STACK_INIT, &st->status);
	for (;;) {
		struct sk_buff	*skb;
		mISDN_head_t	*hh;

		while ((skb = skb_dequeue(&st->msgq))) {
			mISDNinstance_t	*inst;

			hh = mISDN_HEAD_P(skb);
			if ((hh->addr & MSG_DIR_MASK) == MSG_BROADCAST) {
				do_broadcast(st, skb);
				continue;
			}
			inst = get_nextlayer(st, hh->addr);
			if (!inst) {
				int_errtxt("%s: st(%08x) no instance for addr(%08x) prim(%x) dinfo(%x)", __FUNCTION__,
					st->id, hh->addr, hh->prim, hh->dinfo);
				dev_kfree_skb(skb);
				continue;
			}
			if (!inst->function) {
				int_errtxt("%s: instance(%08x) no function", __FUNCTION__, inst->id);
				dev_kfree_skb(skb);
				continue;
			}
			err = inst->function(inst, skb);
			if (err) {
				int_errtxt("%s: instance(%08x)->function return(%d)", __FUNCTION__, inst->id, err);
				dev_kfree_skb(skb);
				continue;
			}
		}
		if (test_bit(mISDN_STACK_ABORT, &st->status))
			break;
		if (st->notify != NULL)
			up(st->notify);
		interruptible_sleep_on(&st->workq);
	}
	printk(KERN_DEBUG "mISDNStackd daemon for id(%08x) killed now\n", st->id);
	test_and_set_bit(mISDN_STACK_KILLED, &st->status);
	discard_queue(&st->msgq);
	st->thread = NULL;
	if (st->notify != NULL)
		up(st->notify);
	return(0);
}

mISDNstack_t *
new_stack(mISDNstack_t *master, mISDNinstance_t *inst)
{
	mISDNstack_t *newst;

	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "create %s stack inst(%p)\n",
			master ? "child" : "master", inst);
	if (!(newst = kmalloc(sizeof(mISDNstack_t), GFP_ATOMIC))) {
		printk(KERN_ERR "kmalloc mISDN_stack failed\n");
		return(NULL);
	}
	memset(newst, 0, sizeof(mISDNstack_t));
	INIT_LIST_HEAD(&newst->childlist);
	INIT_LIST_HEAD(&newst->prereg);
	init_waitqueue_head(&newst->workq);
	skb_queue_head_init(&newst->msgq);
	test_and_set_bit(mISDN_STACK_INIT, &newst->status);
	if (!master) {
		if (inst && inst->st) {
			newst->id = get_free_stackid(inst->st, FLG_CLONE_STACK);
		} else {
			newst->id = get_free_stackid(NULL, 0);
		}
	} else {
		newst->id = get_free_stackid(master, FLG_CHILD_STACK);
	}
	newst->mgr = inst;
	if (master) {
		list_add_tail(&newst->list, &master->childlist);
	} else {
		list_add_tail(&newst->list, &mISDN_stacklist);
	}
	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "Stack id %x added\n", newst->id);
	if (inst)
		inst->st = newst;
	kernel_thread(mISDNStackd, (void *)newst, 0);	
	return(newst);
}


static int
release_layers(mISDNstack_t *st, u_int prim)
{
	int	i;

	for (i = 0; i <= MAX_LAYER_NR; i++) {
		if (st->i_array[i]) {
			if (core_debug & DEBUG_CORE_FUNC)
				printk(KERN_DEBUG  "%s: st(%p) inst%d(%p):%x %s lm(%x)\n",
					__FUNCTION__, st, i, st->i_array[i], st->i_array[i]->id,
					st->i_array[i]->name, st->i_array[i]->pid.layermask);
			st->i_array[i]->obj->own_ctrl(st->i_array[i], prim, NULL);
		}
	}
	return(0);
}

int
do_for_all_layers(void *data, u_int prim, void *arg)
{
	mISDNstack_t	*st = data;
	int		i;

	if (!st) {
		int_error();
		return(-EINVAL);
	}
	for (i = 0; i <= MAX_LAYER_NR; i++) {
		if (st->i_array[i]) {
			if (core_debug & DEBUG_CORE_FUNC)
				printk(KERN_DEBUG  "%s: st(%p) inst%d(%p):%x %s prim(%x) arg(%p)\n",
					__FUNCTION__, st, i, st->i_array[i], st->i_array[i]->id,
					st->i_array[i]->name, prim, arg);
			st->i_array[i]->obj->own_ctrl(st->i_array[i], prim, arg);
		}
	}
	return(0);
}

int
change_stack_para(mISDNstack_t *st, u_int prim, mISDN_stPara_t *stpara)
{
	int	changed = 0;
	if (!st) {
		int_error();
		return(-EINVAL);
	}
	if (prim == (MGR_ADDSTPARA | REQUEST)) {
		if (!stpara) {
			int_error();
			return(-EINVAL);
		}
		prim = MGR_ADDSTPARA | INDICATION;
		if (stpara->maxdatalen > 0 && stpara->maxdatalen < st->para.maxdatalen) {
			changed++;
			st->para.maxdatalen = stpara->maxdatalen;
		}
		if (stpara->up_headerlen > st->para.up_headerlen) {
			changed++;
			st->para.up_headerlen = stpara->up_headerlen;
		}
		if (stpara->down_headerlen > st->para.down_headerlen) {
			changed++;
			st->para.down_headerlen = stpara->down_headerlen;
		}
		if (!changed)
			return(0);
		stpara = &st->para;
	} else if (prim == (MGR_CLRSTPARA | REQUEST)) {
		prim = MGR_CLRSTPARA | INDICATION;
		memset(&st->para, 0, sizeof(mISDN_stPara_t));
		stpara = NULL;
	}
	return(do_for_all_layers(st, prim, stpara));
}

static int
delete_stack(mISDNstack_t *st)
{
	DECLARE_MUTEX_LOCKED(sem);
	int	err;

	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "%s: st(%p:%08x)\n", __FUNCTION__, st, st->id);
	if (!list_empty(&st->prereg)) {
		mISDNinstance_t	*inst, *ni;

		int_errtxt("st(%08x)->prereg not empty\n", st->id);
		list_for_each_entry_safe(inst, ni, &st->prereg, list) {
			int_errtxt("inst(%p:%08x) preregistered", inst, inst->id);
			list_del(&inst->list);
		}
	}
	if (st->thread) {
		st->notify = &sem;
		test_and_set_bit(mISDN_STACK_ABORT, &st->status);
		wake_up_interruptible(&st->workq);
		down(&sem);
		st->notify = NULL;
	}
	if ((err = release_layers(st, MGR_RELEASE | INDICATION))) {
		printk(KERN_WARNING "%s: err(%d)\n", __FUNCTION__, err);
		return(err);
	}
	list_del(&st->list);
	kfree(st);
	return(0);
}

int
release_stack(mISDNstack_t *st) {
	int err;
	mISDNstack_t *cst, *nst;

	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "%s: st(%p)\n", __FUNCTION__, st);

	list_for_each_entry_safe(cst, nst, &st->childlist, list) {
		if ((err = delete_stack(cst))) {
			return(err);
		}
	}

	if ((err = delete_stack(st)))
		return(err);

	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "%s: mISDN_stacklist(%p<-%p->%p)\n", __FUNCTION__,
			mISDN_stacklist.prev, &mISDN_stacklist, mISDN_stacklist.next);
	return(0);
}

void
release_stacks(mISDNobject_t *obj)
{
	mISDNstack_t	*st, *tmp;
	int		rel, i;

	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "%s: obj(%p) %s\n",
			__FUNCTION__, obj, obj->name);
	list_for_each_entry_safe(st, tmp, &mISDN_stacklist, list) {
		rel = 0;
		if (core_debug & DEBUG_CORE_FUNC)
			printk(KERN_DEBUG "%s: st(%p)\n",
				__FUNCTION__, st);
		for (i = 0; i <= MAX_LAYER_NR; i++) {
			if (!st->i_array[i])
				continue;
			if (core_debug & DEBUG_CORE_FUNC)
				printk(KERN_DEBUG "%s: inst%d(%p)\n",
					__FUNCTION__, i, st->i_array[i]);
			if (st->i_array[i]->obj == obj)
				rel++;
		}		
		if (rel)
			release_stack(st);
	}
	if (obj->refcnt)
		printk(KERN_WARNING "release_stacks obj %s refcnt is %d\n",
			obj->name, obj->refcnt);
}

#ifdef OBSOLATE
static void
get_free_instid(mISDNstack_t *st, mISDNinstance_t *inst) {
	mISDNinstance_t *il;

	inst->id = mISDN_get_lowlayer(inst->pid.layermask)<<20;
	inst->id |= FLG_INSTANCE;
	if (st) {
		inst->id |= st->id;
	} else {
		list_for_each_entry(il, &mISDN_instlist, list) {
			if (il->id == inst->id) {
				if ((inst->id & IF_INSTMASK) >= INST_ID_MAX) {
					inst->id = 0;
					return;
				}
				inst->id += LAYER_ID_INC;
				il = list_entry(mISDN_instlist.next, mISDNinstance_t, list);
			}
		}
	}
}
#endif

int
register_layer(mISDNstack_t *st, mISDNinstance_t *inst)
{
	int		idx;
	mISDNinstance_t	*dup;

	if (!inst || !st)
		return(-EINVAL);
	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "%s:st(%p) inst(%p/%p) lmask(%x) id(%x)\n",
			__FUNCTION__, st, inst, inst->obj,
			inst->pid.layermask, inst->id);
	if (inst->id) { /* already registered */
		// if (inst->st || !st) {
			int_errtxt("register duplicate %08x %p %p",
				inst->id, inst->st, st);
			return(-EBUSY);
		//}
	}
	/*
	 * To simplify registration we assume that our stacks are 
	 * always build with monoton increasing layernumbers from
	 * bottom (HW,L0) to highest number
	 */
//	if (st) {
		for (idx = 0; idx <= MAX_LAYER_NR; idx++)
			if (!st->i_array[idx])
				break;
		if (idx > MAX_LAYER_NR) {
			int_errtxt("stack %08x overflow", st->id);
			return(-EXFULL);
		}
		inst->regcnt++;
		st->i_array[idx] = inst;
		inst->id = st->id | FLG_INSTANCE | idx;
		dup = get_instance4id(inst->id);
		if (dup) {
			int_errtxt("register duplicate %08x i1(%p) i2(%p) i1->st(%p) i2->st(%p) st(%p)",
				inst->id, inst, dup, inst->st, dup->st, st);
			inst->regcnt--;
			st->i_array[idx] = NULL;
			inst->id = 0;
			return(-EBUSY);
		}
//	}
	
	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "%s: inst(%p/%p) id(%x)\n", __FUNCTION__,
			inst, inst->obj, inst->id);
	inst->st = st;
	list_add_tail(&inst->list, &mISDN_instlist);
	return(0);
}

int
preregister_layer(mISDNstack_t *st, mISDNinstance_t *inst)
{
	if (!inst || !st)
		return(-EINVAL);
	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "%s:st(%08x) inst(%p:%08x) lmask(%x)\n",
			__FUNCTION__, st->id, inst, inst->id, inst->pid.layermask);
	if (inst->id) {
		/* already registered */
		int_errtxt("register duplicate %08x %p %p",
			inst->id, inst->st, st);
		return(-EBUSY);
	}
	inst->st = st;
	list_add_tail(&inst->list, &st->prereg);
	return(0);
}

int
unregister_instance(mISDNinstance_t *inst) {
	int	i;

	if (!inst)
		return(-EINVAL);
	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "%s: st(%p) inst(%p):%x lay(%x)\n",
			__FUNCTION__, inst->st, inst, inst->id, inst->pid.layermask);
	if (inst->st && inst->id) {
		i = inst->id & LAYER_ID_MASK;
		if (i > MAX_LAYER_NR) {
			int_errtxt("unregister %08x  st(%08x) wrong layer", inst->id, inst->st->id);
			return(-EINVAL);
		}
		if (inst->st->i_array[i] == inst) {
			inst->regcnt--;
			inst->st->i_array[i] = NULL;
		} else if (inst->st->i_array[i]) {
			int_errtxt("unregister %08x  st(%08x) wrong instance %08x",
				inst->id, inst->st->id, inst->st->i_array[i]->id);
			return(-EINVAL);
		} else
			printk(KERN_WARNING "unregister %08x  st(%08x) not in stack",
				inst->id, inst->st->id);
		if (inst->st && (inst->st->mgr != inst))
			inst->st = NULL;
	}
	list_del_init(&inst->list);
	inst->id = 0;
	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "%s: mISDN_instlist(%p<-%p->%p)\n", __FUNCTION__,
			mISDN_instlist.prev, &mISDN_instlist, mISDN_instlist.next);
	return(0);
}

int
copy_pid(mISDN_pid_t *dpid, mISDN_pid_t *spid, u_char *pbuf)
{
	u_int	i, off;

	memcpy(dpid, spid, sizeof(mISDN_pid_t));
	if (spid->pbuf) {
		if (!pbuf) {
			int_error();
			return(-ENOMEM);
		}
		dpid->pbuf = pbuf;
		memcpy(dpid->pbuf, spid->pbuf, spid->maxplen);
		for (i = 0; i <= MAX_LAYER_NR; i++) {
			if (spid->param[i]) {
				off = (u_int)(spid->param[i] - spid->pbuf);
				dpid->param[i] = dpid->pbuf + off;
			}
		}
	}
	return(0);
}

int
set_stack(mISDNstack_t *st, mISDN_pid_t *pid)
{
	int 		err, i;
	u_char		*pbuf = NULL;
	mISDNinstance_t	*inst;
//	mISDNlayer_t	*hl, *hln;

	if (!st || !pid) {
		int_error();
		return(-EINVAL);
	}
	if (!st->mgr || !st->mgr->obj || !st->mgr->obj->ctrl) {
		int_error();
		return(-EINVAL);
	}
	if (pid->pbuf)
		pbuf = kmalloc(pid->maxplen, GFP_ATOMIC);
	err = copy_pid(&st->pid, pid, pbuf);
	if (err)
		return(err);
	memcpy(&st->mgr->pid, &st->pid, sizeof(mISDN_pid_t));
	if (!mISDN_SetHandledPID(st->mgr->obj, &st->mgr->pid)) {
		int_error();
		return(-ENOPROTOOPT);
	} else {
		mISDN_RemoveUsedPID(pid, &st->mgr->pid);
	}
	err = st->mgr->obj->ctrl(st, MGR_REGLAYER | REQUEST, st->mgr);
	if (err) {
		int_error();
		return(err);
	}
	while (pid->layermask) {
		inst = get_next_instance(st, pid);
		if (!inst) {
			int_error();
			st->mgr->obj->ctrl(st, MGR_CLEARSTACK| REQUEST, NULL);
			return(-ENOPROTOOPT);
		}
		mISDN_RemoveUsedPID(pid, &inst->pid);
	}
	if (!list_empty(&st->prereg))
		int_errtxt("st(%08x)->prereg not empty\n", st->id);

	for (i = 0; i <= MAX_LAYER_NR; i++) {
		inst = st->i_array[i];
		if (!inst)
			break;
		if (!inst->obj) {
			int_error();
			continue;
		}
		if (!inst->obj->own_ctrl) {
			int_error();
			continue;
		}
		inst->obj->own_ctrl(inst, MGR_SETSTACK |CONFIRM, NULL);
	}
	return(0);
}

int
clear_stack(mISDNstack_t *st) {

	if (!st)
		return(-EINVAL);
	if (core_debug & DEBUG_CORE_FUNC)
		printk(KERN_DEBUG "%s: st(%p)\n", __FUNCTION__, st);
	if (st->pid.pbuf)
		kfree(st->pid.pbuf);
	memset(&st->pid, 0, sizeof(mISDN_pid_t));
	memset(&st->para, 0, sizeof(mISDN_stPara_t));
	return(release_layers(st, MGR_UNREGLAYER | REQUEST));
}

static int
test_stack_protocol(mISDNstack_t *st, u_int l1prot, u_int l2prot, u_int l3prot)
{
	int		cnt = MAX_LAYER_NR + 1, ret = 1;
	mISDN_pid_t	pid;
	mISDNinstance_t	*inst;
	
	clear_stack(st);
	memset(&pid, 0, sizeof(mISDN_pid_t));
	pid.layermask = ISDN_LAYER(1);
	if (!(((l2prot == 2) || (l2prot == 0x40)) && (l3prot == 1)))
		pid.layermask |= ISDN_LAYER(2);
	if (!(l3prot == 1))
		pid.layermask |= ISDN_LAYER(3);
	
	pid.protocol[1] = l1prot | ISDN_PID_LAYER(1) | ISDN_PID_BCHANNEL_BIT;
	if (pid.layermask & ISDN_LAYER(2))
		pid.protocol[2] = l2prot | ISDN_PID_LAYER(2) | ISDN_PID_BCHANNEL_BIT;
	if (pid.layermask & ISDN_LAYER(3))
		pid.protocol[3] = l3prot | ISDN_PID_LAYER(3) | ISDN_PID_BCHANNEL_BIT;
	copy_pid(&st->pid, &pid, NULL);
	memcpy(&st->mgr->pid, &pid, sizeof(mISDN_pid_t));
	if (!mISDN_SetHandledPID(st->mgr->obj, &st->mgr->pid)) {
		memset(&st->pid, 0, sizeof(mISDN_pid_t));
		return(-ENOPROTOOPT);
	} else {
		mISDN_RemoveUsedPID(&pid, &st->mgr->pid);
	}
	if (!pid.layermask) {
		memset(&st->pid, 0, sizeof(mISDN_pid_t));
		return(0);
	}
	ret = st->mgr->obj->ctrl(st, MGR_REGLAYER | REQUEST, st->mgr);
	if (ret) {
		clear_stack(st);
		return(ret);
	}
	while (pid.layermask && cnt--) {
		inst = get_next_instance(st, &pid);
		if (!inst) {
			st->mgr->obj->ctrl(st, MGR_CLEARSTACK| REQUEST, NULL);
			return(-ENOPROTOOPT);
		}
		mISDN_RemoveUsedPID(&pid, &inst->pid);
	}
	if (!cnt)
		ret = -ENOPROTOOPT;
	clear_stack(st);
	return(ret);
}

static u_int	validL1pid4L2[ISDN_PID_IDX_MAX + 1] = {
			0x022d,
			0x03ff,
			0x0000,
			0x0000,
			0x0010,
			0x022d,
			0x03ff,
			0x0380,
			0x022d,
			0x022d,
			0x022d,
			0x01c6,
			0x0000,
};

static u_int	validL2pid4L3[ISDN_PID_IDX_MAX + 1] = {
			0x1fff,
			0x0000,
			0x0101,
			0x0101,
			0x0010,
			0x0010,
			0x0000,
			0x00c0,
			0x0000,
};

int
evaluate_stack_pids(mISDNstack_t *st, mISDN_pid_t *pid)
{
	int 		err;
	mISDN_pid_t	pidmask;
	u_int		l1bitm, l2bitm, l3bitm;
	u_int		l1idx, l2idx, l3idx;

	if (!st || !pid) {
		int_error();
		return(-EINVAL);
	}
	if (!st->mgr || !st->mgr->obj || !st->mgr->obj->ctrl) {
		int_error();
		return(-EINVAL);
	}
	copy_pid(&pidmask, pid, NULL);
	memset(pid, 0, sizeof(mISDN_pid_t));
	for (l1idx=0; l1idx <= ISDN_PID_IDX_MAX; l1idx++) {
		l1bitm = 1 << l1idx;
		if (!(pidmask.protocol[1] & l1bitm))
			continue;
		for (l2idx=0; l2idx <= ISDN_PID_IDX_MAX; l2idx++) {
			l2bitm = 1 << l2idx;
			if (!(pidmask.protocol[2] & l2bitm))
				continue;
			if (!(validL1pid4L2[l2idx] & l1bitm))
				continue;
			for (l3idx=0; l3idx <= ISDN_PID_IDX_MAX; l3idx++) {
				err = 1;
				l3bitm = 1 << l3idx;
				if (!(pidmask.protocol[3] & l3bitm))
					continue;
				if (!(validL2pid4L3[l3idx] & l2bitm))
					continue;
				err = test_stack_protocol(st, l1bitm, l2bitm, l3bitm);
				if (!err) {
					pid->protocol[3] |= l3bitm;
					pid->protocol[2] |= l2bitm;
					pid->protocol[1] |= l1bitm;
				}
			}
		}
	}
	return(0);
}

EXPORT_SYMBOL(mISDN_queue_message);
