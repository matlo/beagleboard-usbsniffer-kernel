/*
 *  linux/arch/arm/kernel/sysfs.c
 *
 *  Copyright (C) 2008 Mans Rullgard
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/cpu.h>
#include <linux/sysdev.h>
#include <linux/fs.h>

#define SETBITS(val, bits, new)			\
	do {					\
		val &= ~bits;			\
		val |= new & bits;		\
	} while (0)

#define SHOW_REG(name, opc1, crn, crm, opc2)				\
static ssize_t name##_show(struct sys_device *dev,			\
			   struct sysdev_attribute *attr,		\
			   char *buf)					\
{									\
	unsigned val;							\
	asm ("mrc p15,"#opc1", %0,"#crn","#crm","#opc2 : "=r"(val));	\
	return snprintf(buf, PAGE_SIZE, "%08x\n", val);			\
}

#define STORE_REG(name, opc1, crn, crm, opc2, bits)			\
static ssize_t name##_store(struct sys_device *dev,			\
			    struct sysdev_attribute *attr,		\
			    const char *buf, size_t size)		\
{									\
	char *end;							\
	unsigned new = simple_strtoul(buf, &end, 0);			\
	unsigned val;							\
									\
	if (end == buf)							\
		return -EINVAL;						\
									\
	asm ("mrc p15,"#opc1", %0,"#crn","#crm","#opc2 : "=r"(val));	\
	SETBITS(val, bits, new);					\
	asm ("mcr p15,"#opc1", %0,"#crn","#crm","#opc2 :: "r"(val));	\
									\
	return end - buf;						\
}

#define RD_REG(name, opc1, crn, crm, opc2)				\
	SHOW_REG(name, opc1, crn, crm, opc2)				\
	static SYSDEV_ATTR(name, S_IRUGO|S_IWUSR, name##_show, NULL)

#define RDWR_REG(name, opc1, crn, crm, opc2, bits)			\
	SHOW_REG(name, opc1, crn, crm, opc2)				\
	STORE_REG(name, opc1, crn, crm, opc2, bits)			\
	static SYSDEV_ATTR(name, S_IRUGO|S_IWUSR, name##_show, name##_store)

RDWR_REG(control, 0, c1, c0, 0, 0x802);

SHOW_REG(aux_ctl, 0, c1, c0, 1)

#ifdef CONFIG_ARCH_OMAP34XX
static ssize_t aux_ctl_store(struct sys_device *dev,
			     struct sysdev_attribute *attr,
			     const char *buf, size_t size)
{
	char *end;
	unsigned new = simple_strtoul(buf, &end, 0);
	unsigned val;

	if (end == buf)
		return -EINVAL;

	asm ("mrc p15, 0, %0, c1, c0, 1" : "=r"(val));
	SETBITS(val, 0xff8, new);
	val &= ~2;
	asm ("mov r0,  %0	\n\t"
	     "mov r12, #3	\n\t"
	     "smc #0		\n\t"
	     :: "r"(val) : "r0", "r12");

	return end - buf;
}
#define AUX_WR S_IWUSR
#else
#define aux_ctl_store NULL
#define AUX_WR 0
#endif

static SYSDEV_ATTR(aux_control, S_IRUGO|AUX_WR, aux_ctl_show, aux_ctl_store);

SHOW_REG(l2_aux_ctl, 1, c9, c0, 2)

#ifdef CONFIG_ARCH_OMAP34XX
static ssize_t l2_aux_ctl_store(struct sys_device *dev,
				struct sysdev_attribute *attr,
				const char *buf, size_t size)
{
	char *end;
	unsigned new = simple_strtoul(buf, &end, 0);
	unsigned val;

	if (end == buf)
		return -EINVAL;

	asm ("mrc p15, 1, %0, c9, c0, 2" : "=r"(val));
	SETBITS(val, 0xbc00000, new);
	asm ("mov r0,  %0	\n\t"
	     "mov r12, #2	\n\t"
	     "smc #0		\n\t"
	     :: "r"(val) : "r0", "r12");

	return end - buf;
}
#define L2AUX_WR S_IWUSR
#else
#define l2_aux_ctl_store NULL
#define L2AUX_WR 0
#endif

static SYSDEV_ATTR(l2_aux_control, S_IRUGO|L2AUX_WR,
		   l2_aux_ctl_show, l2_aux_ctl_store);

RDWR_REG(pmon_pmnc,   0, c9, c12, 0, 0x3f)
RDWR_REG(pmon_cntens, 0, c9, c12, 1, 0x8000000f)
RDWR_REG(pmon_cntenc, 0, c9, c12, 2, 0x8000000f)
RDWR_REG(pmon_ccnt,   0, c9, c13, 0, 0xffffffff)
RDWR_REG(pmon_useren, 0, c9, c14, 0, 1)

#define REG_ATTR(sysdev, name)						\
	do {								\
		int err = sysfs_create_file(&sysdev->kobj, &name.attr); \
		WARN_ON(err != 0);					\
	} while (0)

static int __init cpu_sysfs_init(void)
{
	struct sys_device *sysdev;
	int cpu;

	for_each_possible_cpu(cpu) {
		sysdev = get_cpu_sysdev(cpu);
		REG_ATTR(sysdev, attr_control);
		REG_ATTR(sysdev, attr_aux_control);
		REG_ATTR(sysdev, attr_l2_aux_control);
		REG_ATTR(sysdev, attr_pmon_pmnc);
		REG_ATTR(sysdev, attr_pmon_cntens);
		REG_ATTR(sysdev, attr_pmon_cntenc);
		REG_ATTR(sysdev, attr_pmon_ccnt);
		REG_ATTR(sysdev, attr_pmon_useren);
	}

	return 0;
}
device_initcall(cpu_sysfs_init);
