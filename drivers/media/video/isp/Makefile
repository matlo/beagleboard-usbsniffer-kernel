# Makefile for OMAP3 ISP driver

isp-mod-objs += \
	isp.o ispccdc.o \
	isppreview.o ispresizer.o isph3a.o isphist.o isp_af.o ispcsi2.o \
	ispstat.o

obj-$(CONFIG_VIDEO_OMAP3_ISP) += isp-mod.o

obj-$(CONFIG_VIDEO_OMAP34XX_ISP_PREVIEWER) += \
	omap_previewer.o

obj-$(CONFIG_VIDEO_OMAP34XX_ISP_RESIZER) += \
	omap_resizer.o

