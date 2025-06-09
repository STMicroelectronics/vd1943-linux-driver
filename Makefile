ifneq ($(KERNELRELEASE),)
include Kbuild

else
KVERSION ?= `uname -r`
KDIR ?= /lib/modules/${KVERSION}/build
default:
	CONFIG_DRIVER_VD1943=m $(MAKE) -C $(KDIR) M=$$PWD

clean:
	CONFIG_DRIVER_VD1943=m $(MAKE) -C $(KDIR) M=$$PWD clean

endif
