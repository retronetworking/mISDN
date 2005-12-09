BASEDIR=$(shell pwd)


INSTALL_PREFIX := /
export INSTALL_PREFIX

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
#CONFIGS+=CONFIG_MISDN_HFCUSB=m
CONFIGS+=CONFIG_MISDN_AVM_FRITZ=m


MINCLUDES+=-I$(MISDNDIR)/include

all: 
	@echo
	@echo "Makeing mISDN"
	@echo "============="
	@echo
	cp $(MISDNDIR)/drivers/isdn/hardware/mISDN/Makefile.v2.6 $(MISDNDIR)/drivers/isdn/hardware/mISDN/Makefile

	cd $(LINUX) ; make SUBDIRS=$(MISDN_SRC) modules $(CONFIGS) LINUXINCLUDE="$(MINCLUDES) -I$(LINUX)/include"



install: all
	cd $(LINUX) ; make SUBDIRS=$(MISDN_SRC) modules_install 
	cp $(MISDNDIR)/include/linux/*.h $(INSTALL_PREFIX)/usr/include/linux/
	depmod

.PHONY: install all clean 

clean:
	rm -rf drivers/isdn/hardware/mISDN/*.o
	rm -rf drivers/isdn/hardware/mISDN/*.ko
	rm -rf *~

