# Makefile for OMAP3 ISP driver

ifdef CONFIG_ARCH_OMAP3410
isp-mod-objs += \
	isp.o ispccdc.o
else
isp-mod-objs += \
	isp.o ispqueue.o ispvideo.o \
	ispcsiphy.o ispccp2.o ispcsi2.o \
	ispccdc.o isppreview.o ispresizer.o \
	ispstat.o isph3a_aewb.o isph3a_af.o isphist.o
endif

obj-$(CONFIG_VIDEO_OMAP3) += isp-mod.o
