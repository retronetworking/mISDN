/* $Id$
 *
 * Author       Karsten Keil (keil@isdn4linux.de)
 *
 * mISDN sysfs stack stuff
 *
 * This file is (c) under GNU PUBLIC LICENSE
 *
 */
#include "core.h"
#include "sysfs.h"

#define to_mISDNstack(d) container_of(d, mISDNstack_t, class_dev)

static ssize_t show_st_id(struct class_device *class_dev, char *buf)
{
	mISDNstack_t	*st = to_mISDNstack(class_dev);
	return sprintf(buf, "%08x\n", st->id);
}
static CLASS_DEVICE_ATTR(id, S_IRUGO, show_st_id, NULL);

static ssize_t show_st_status(struct class_device *class_dev, char *buf)
{
	mISDNstack_t	*st = to_mISDNstack(class_dev);
	return sprintf(buf, "0x%08lx\n", st->status);
}

static ssize_t store_st_status(struct class_device *class_dev, const char *buf, size_t count)
{
	mISDNstack_t	*st = to_mISDNstack(class_dev);
	ulong	status;
	int	err;

	status = simple_strtol(buf, NULL, 0);
	printk(KERN_DEBUG "%s: status %08lx\n", __FUNCTION__, status);
	if (status == (1<<mISDN_STACK_INIT)) {
		/* we want to make st->new_pid activ */
		err = clear_stack(st, 1);
		if (err) {
			int_errtxt("clear_stack:%d", err);
			return(err);
		}
		err = set_stack(st ,&st->new_pid);
		if (err) {
			int_errtxt("set_stack:%d", err);
			return(err);
		}
		return(count);
	} else if (status == (1<<mISDN_STACK_THREADSTART)) {
		/* we want to start a new process after abort */
		err = mISDN_start_stack_thread(st);
		if (err) {
			int_errtxt("start_stack_thread:%d", err);
			return(err);
		}
		return(count);
	}
	st->status = status;
	wake_up_interruptible(&st->workq);
	return(count);
}
static CLASS_DEVICE_ATTR(status, S_IRUGO | S_IWUSR, show_st_status, store_st_status);

static ssize_t store_st_protocol(struct class_device *class_dev, const char *buf, size_t count)
{
	mISDNstack_t	*st = to_mISDNstack(class_dev);
	ulong	tmp;
	char	*p = (char *)buf;
	u_int	i;

	memset(&st->new_pid.protocol, 0, (MAX_LAYER_NR + 1)*sizeof(st->new_pid.protocol[0]));
	for (i=0; i<=MAX_LAYER_NR; i++) {
		if (!*p)
			break;
		tmp = simple_strtol(p, &p, 0);
		st->new_pid.protocol[i] = tmp;
		if (*p)
			p++;
	}
	if (*p)
		int_errtxt("overflow");
	return(count);
}

static ssize_t store_st_layermask(struct class_device *class_dev, const char *buf, size_t count)
{
	mISDNstack_t	*st = to_mISDNstack(class_dev);
	ulong		mask = (1<<(MAX_LAYER_NR + 1)) -1;

	st->new_pid.layermask = simple_strtol(buf, NULL, 0);
	if (st->new_pid.layermask > mask) {
		int_errtxt("overflow");
		st->new_pid.layermask &= mask;
	}
	return(count);
}

static ssize_t store_st_parameter(struct class_device *class_dev, const char *buf, size_t count)
{
	mISDNstack_t	*st = to_mISDNstack(class_dev);
	ulong	tmp;
	char	*d, *p = (char *)buf;
	u_int	i, j, l;

	memset(&st->new_pid.param, 0, (MAX_LAYER_NR + 1)*sizeof(st->new_pid.param[0]));
	kfree(st->new_pid.pbuf);
	l = 0;
	for (i=0; i<=MAX_LAYER_NR; i++) {
		if (!*p)
			break;
		tmp = simple_strtol(p, &p, 0);
		if (*p)
			p++;
		if (tmp) {
			j = tmp;
			l += j+1;
			while(j--) {
				if (!*p)
					break;
				tmp = simple_strtol(p, &p, 0);
				if (*p)
					p++;
				else
					break;
			}
		}
	}
	if (*p)
		int_errtxt("overflow");
	if (l == 0) {
		st->new_pid.maxplen = 0;
		return(count);
	}
	st->new_pid.pbuf = kmalloc(l, GFP_ATOMIC);
	if (!st->new_pid.pbuf)
		return(-ENOMEM);
	st->new_pid.maxplen = l;
	d = st->new_pid.pbuf;
	memset(d, 0, l);
	p = (char *)buf;
	for (i=0; i<=MAX_LAYER_NR; i++) {
		if (!*p)
			break;
		tmp = simple_strtol(p, &p, 0);
		if (*p)
			p++;
		if (tmp) {
			j = tmp;
			st->new_pid.param[i] = d;
			*d++ = tmp & 0xff;
			while(j--) {
				if (!*p)
					break;
				tmp = simple_strtol(p, &p, 0);
				*d++ = tmp & 0xff;
				if (*p)
					p++;
				else
					break;
			}
		}
	}
	return(count);
}

MISDN_PROTO(mISDNstack, pid, S_IRUGO);
MISDN_PROTO(mISDNstack, new_pid, S_IRUGO);

static ssize_t show_st_qlen(struct class_device *class_dev, char *buf)
{
	mISDNstack_t	*st = to_mISDNstack(class_dev);
	return sprintf(buf, "%d\n", skb_queue_len(&st->msgq));
}
static CLASS_DEVICE_ATTR(qlen, S_IRUGO, show_st_qlen, NULL);

static void release_mISDN_stack(struct class_device *dev)
{
	mISDNstack_t	*st = to_mISDNstack(dev);
	char		name[12];

	sysfs_remove_group(&st->class_dev.kobj, &pid_group);
	sysfs_remove_group(&st->class_dev.kobj, &new_pid_group);
	printk(KERN_INFO "release stack class dev %s\n", dev->class_id);
	if (st->parent) {
		sysfs_remove_link(&dev->kobj, "parent");
		snprintf(name, 12, "child%d", (CHILD_ID_MASK & st->id) >> 16);
		sysfs_remove_link(&st->parent->class_dev.kobj, name);
	}
	if (st->master) {
		sysfs_remove_link(&dev->kobj, "master");
		snprintf(name, 12, "clone%d", (CLONE_ID_MASK & st->id) >> 16);
		sysfs_remove_link(&st->master->class_dev.kobj, name);
	}
}

static struct class stack_dev_class = {
	.name		= "mISDN-stacks",
	.owner		= THIS_MODULE,
	.release	= &release_mISDN_stack,
};

int
mISDN_register_sysfs_stack(mISDNstack_t *st)
{
	int	err;
	char	name[12];

	st->class_dev.class = &stack_dev_class;
	if (st->id & FLG_CHILD_STACK)
		snprintf(st->class_dev.class_id, BUS_ID_SIZE, "chst-%08x", st->id);
	else if (st->id & FLG_CLONE_STACK)
		snprintf(st->class_dev.class_id, BUS_ID_SIZE, "clst-%08x", st->id);
	else
		snprintf(st->class_dev.class_id, BUS_ID_SIZE, "st-%08x", st->id);
	if (st->mgr)
		st->class_dev.dev = st->mgr->class_dev.dev;
	err = class_device_register(&st->class_dev);
	if (err)
		return(err);
	err = sysfs_create_group(&st->class_dev.kobj, &pid_group);
	if (err)
		goto out_unreg;
	mISDNstack_attr_protocol_new_pid.attr.mode |= S_IWUSR;
	mISDNstack_attr_protocol_new_pid.store = store_st_protocol;
	mISDNstack_attr_parameter_new_pid.attr.mode |= S_IWUSR;
	mISDNstack_attr_parameter_new_pid.store = store_st_parameter;
	mISDNstack_attr_layermask_new_pid.attr.mode |= S_IWUSR;
	mISDNstack_attr_layermask_new_pid.store = store_st_layermask;
	err = sysfs_create_group(&st->class_dev.kobj, &new_pid_group);
	if (err)
		goto out_unreg;
	class_device_create_file(&st->class_dev, &class_device_attr_id);
	class_device_create_file(&st->class_dev, &class_device_attr_qlen);
	class_device_create_file(&st->class_dev, &class_device_attr_status);
	if (st->parent) {
		sysfs_create_link(&st->class_dev.kobj, &st->parent->class_dev.kobj, "parent");
		snprintf(name, 12, "child%d", (CHILD_ID_MASK & st->id) >> 16);
		sysfs_create_link(&st->parent->class_dev.kobj, &st->class_dev.kobj, name);
	}
	if (st->master) {
		sysfs_create_link(&st->class_dev.kobj, &st->master->class_dev.kobj, "master");
		snprintf(name, 12, "clone%d", (CLONE_ID_MASK & st->id) >> 16);
		sysfs_create_link(&st->master->class_dev.kobj, &st->class_dev.kobj, name);
	}
	return(err);

out_unreg:
	class_device_unregister(&st->class_dev);
	return(err);
}

void
mISDN_unregister_sysfs_st(mISDNstack_t *st)
{
	class_device_unregister(&st->class_dev);
}

int
mISDN_sysfs_st_init(void)
{
	return(class_register(&stack_dev_class));
}

void
mISDN_sysfs_st_cleanup(void)
{
	class_unregister(&stack_dev_class);
}
