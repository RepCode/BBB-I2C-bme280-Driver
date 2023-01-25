obj-m := i2cDriver.o
KERNEL_DIR = /home/rep/Imagen_BBB/bb-kernel/KERNEL
all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C $(KERNEL_DIR) M=$(PWD) modules
clean:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C $(KERNEL_DIR) M=$(PWD) clean