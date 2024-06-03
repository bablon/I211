obj-m += i211.o
obj-m += i211-phy.o

KSRC = /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KSRC) M=$(PWD) modules

.PHONY: clean

clean:
	$(MAKE) -C $(KSRC) M=$(PWD) clean

insmod:
	@sudo insmod i211-phy.ko
	@sudo insmod i211.ko

rmmod:
	@sudo rmmod i211
	@sudo rmmod i211-phy
