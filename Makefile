obj-m += rmi_bus.o
obj-m += rmi_i2c.o
obj-m += rmi_spi.o
obj-m += rmi_generic.o
rmi_generic-objs := rmi_driver.o rmi_f01.o
obj-m += rmi_f11.o
obj-m += rmi_f19.o
obj-m += rmi_f34.o

KDIR ?= /lib/modules/$(shell uname -r)/build
EXTRA_CFLAGS += -I$(PWD)

all:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

clean:
	rm -f Module.symvers modules.order *.o *.mod.* *.ko .*.cmd .*.d
	rm -f test/*.o test/*.mod.* test/*.ko test/.*.cmd test/.*.d
	rm -fr .tmp_versions




