# SPDX-License-Identifier: GPL-2.0-only
#
# Out-of-tree build:   make
# Install:             make install   (as root)
# Engine replay test:  make replay && tools/engine/replay <stream.bin>
# In-tree:             copy nwfermi_*.[ch] to drivers/input/touchscreen/ and
#                      merge Kconfig and the two kbuild lines below.

ifneq ($(KERNELRELEASE),)

obj-m += nwfermi.o
nwfermi-y := nwfermi_usb.o nwfermi_engine.o

else

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

install:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules_install
	depmod -a

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
	rm -f tools/engine/replay

checkpatch:
	$(KDIR)/scripts/checkpatch.pl --no-tree --strict -f nwfermi_usb.c nwfermi_engine.c nwfermi_engine.h

replay: tools/engine/replay

tools/engine/replay: tools/engine/replay.c nwfermi_engine.c nwfermi_engine.h tools/engine/kshim.h
	$(CC) -O2 -Wall -Wextra -Wno-unused-parameter -include tools/engine/kshim.h -I. -o $@ tools/engine/replay.c nwfermi_engine.c

.PHONY: all install clean checkpatch replay

endif
