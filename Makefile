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
CONFIGS+=CONFIG_MISDN_HFCUSB=m
CONFIGS+=CONFIG_MISDN_AVM_FRITZ=m


MINCLUDES+=-I$(MISDNDIR)/include

all: test_old_misdn
	@echo
	@echo "Makeing mISDN"
	@echo "============="
	@echo
	cp $(MISDNDIR)/drivers/isdn/hardware/mISDN/Makefile.v2.6 $(MISDNDIR)/drivers/isdn/hardware/mISDN/Makefile

#	cd $(LINUX) ; make SUBDIRS=$(MISDN_SRC) modules $(CONFIGS) LINUXINCLUDE="$(MINCLUDES) -I$(LINUX)/include"
	 make -C $(LINUX) SUBDIRS=$(MISDN_SRC) modules $(CONFIGS) 


install: all
	cd $(LINUX) ; make SUBDIRS=$(MISDN_SRC) modules_install 
	cp $(MISDNDIR)/include/linux/*.h $(INSTALL_PREFIX)/usr/include/linux/
	install -m755 misdn-init /etc/init.d/
	depmod


test_old_misdn:
	@if echo -ne "#include <linux/mISDNif.h>" | gcc -C -E - 2>/dev/null 1>/dev/null  ; then \
		if ! echo -ne "#include <linux/mISDNif.h>\n#ifndef FLG_MSG_DOWN\n#error old mISDNif.h\n#endif\n" | gcc -C -E - 2>/dev/null 1>/dev/null ; then \
			echo -ne "\n!!You should remove the following files:\n\n$(LINUX)/include/linux/mISDNif.h\n$(LINUX)/include/linux/isdn_compat.h\n/usr/include/linux/mISDNif.h\n/usr/include/linux/isdn_compat.h\n\nIn order to upgrade to the mqueue branch\n\n"; \
			exit 1; \
		fi ;\
	fi


.PHONY: install all clean 

clean:
	rm -rf drivers/isdn/hardware/mISDN/*.o
	rm -rf drivers/isdn/hardware/mISDN/*.ko
	rm -rf *~

