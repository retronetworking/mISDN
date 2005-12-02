BASEDIR=$(shell pwd)

#PATH to linux source/headers
#LINUX=/usr/src/linux
LINUX=/lib/modules/$(shell uname -r)/build

MISDNDIR=$(BASEDIR)
MISDN_SRC=$(MISDNDIR)/drivers/isdn/hardware/mISDN

########################################
# USER CONFIGS END
########################################

CONFIGS+=CONFIG_MISDN_DRV=m CONFIG_MISDN_DSP=m 
CONFIGS+=CONFIG_MISDN_HFCMULTI=m 
CONFIGS+=CONFIG_MISDN_HFCPCI=m
CONFIGS+=CONFIG_MISDN_HFCUSB=m
CONFIGS+=CONFIG_MISDN_AVM_FRITZ=m


all: 
	@echo
	@echo "Makeing mISDN"
	@echo "============="
	@echo
	@if ! diff $(MISDNDIR)/include/linux/mISDNif.h /usr/include/linux/mISDNif.h > /dev/null 2>/dev/null ; then cp $(MISDNDIR)/include/linux/mISDNif.h /usr/include/linux/mISDNif.h ; fi
	@if ! diff $(MISDNDIR)/include/linux/mISDNif.h $(LINUX)/include/linux/mISDNif.h > /dev/null 2>/dev/null ; then cp $(MISDNDIR)/include/linux/mISDNif.h $(LINUX)/include/linux/mISDNif.h ; fi
	@if ! diff $(MISDNDIR)/include/linux/isdn_compat.h /usr/include/linux/isdn_compat.h >/dev/null 2>/dev/null ; then cp $(MISDNDIR)/include/linux/isdn_compat.h /usr/include/linux/isdn_compat.h ; fi
	@if ! diff $(MISDNDIR)/include/linux/isdn_compat.h $(LINUX)/include/linux/isdn_compat.h >/dev/null 2>/dev/null ; then cp $(MISDNDIR)/include/linux/isdn_compat.h $(LINUX)/include/linux/isdn_compat.h ; fi
	cp $(MISDNDIR)/drivers/isdn/hardware/mISDN/Makefile.v2.6 $(MISDNDIR)/drivers/isdn/hardware/mISDN/Makefile

	cd $(LINUX) ; make SUBDIRS=$(MISDN_SRC) modules $(CONFIGS) 



install: all
	cd $(LINUX) ; make SUBDIRS=$(MISDN_SRC) modules_install 
	depmod

.PHONY: install all clean 

clean:
	rm -rf drivers/isdn/hardware/mISDN/*.o
	rm -rf drivers/isdn/hardware/mISDN/*.ko
	rm -rf *~

