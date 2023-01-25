#include <linux/module.h>   
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>

#include "i2cDriver.h"

MODULE_AUTHOR("Martín Pereira Nuñez Machado");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("TD3 I2C Driver");

enum TransmissionStatus { T_WAIT_START, T_STARTED, T_WRITE, T_READ, T_STOPPED};
static enum TransmissionStatus i2cTransmissionStatus;
struct TransmissionInfo{
    u_int8_t* buffer;
    unsigned int index;
};

struct i2c_device {
    struct TransmissionInfo CurrentTransmissionInfo;
    enum TransmissionStatus i2cTransmissionStatus;
    char* DriverName;
    dev_t dev;
    struct cdev *i2c_cdev;
    struct class *i2c_class;
    struct device* i2c_device;
    int irqNumber;
    volatile void *i2c_registers_address;
    volatile void *i2c_clk_module;
    volatile void *i2c_control_module;
    wait_queue_head_t waitQ;
    spinlock_t lock;
    unsigned long irqFlags;
    uint32_t cmPerOff;
    uint32_t sdaCtrl;
    uint32_t sclCtrl;
    uint32_t moduleClk;
    uint32_t transferClk;
};

struct i2c_device Device = {
    .DriverName = "td3,omap4-i2c"
};

static struct of_device_id device_match_table[] =
    {
        {
            .compatible = "td3,omap4-i2c",
        },
};

static irqreturn_t IrqHandler(int irq, void *d)
{
    unsigned int auxReg, statusReg;

    #ifdef PRINT_DEBUG
    printk(KERN_INFO "\nTD3 I2C Handler called\n");
    #endif

    statusReg = ioread32(Device.i2c_registers_address + I2C_IRQSTATUS);
    #ifdef PRINT_DEBUG
    printk(KERN_INFO "IRQ Flags: %d\n", statusReg);
    #endif

    if(statusReg & XRDY){
        #ifdef PRINT_DEBUG
        printk(KERN_INFO "Transmiting: %d\n", (int)Device.CurrentTransmissionInfo.buffer[Device.CurrentTransmissionInfo.index]);
        #endif
        iowrite32((unsigned int)Device.CurrentTransmissionInfo.buffer[Device.CurrentTransmissionInfo.index], Device.i2c_registers_address + I2C_DATA);
        Device.CurrentTransmissionInfo.index++;
        Device.i2cTransmissionStatus = T_WRITE;
    }
    if(statusReg & RRDY){
        auxReg = ioread32(Device.i2c_registers_address + I2C_DATA);
        Device.CurrentTransmissionInfo.buffer[Device.CurrentTransmissionInfo.index] = (u_int8_t)auxReg;
        #ifdef PRINT_DEBUG
        printk(KERN_INFO "Received: %d\n", (int)Device.CurrentTransmissionInfo.buffer[Device.CurrentTransmissionInfo.index]);
        #endif
        Device.CurrentTransmissionInfo.index++;
        Device.i2cTransmissionStatus = T_READ;
    }
    if(statusReg & BF){ // stop
        Device.i2cTransmissionStatus = T_STOPPED;
        #ifdef PRINT_DEBUG
        printk(KERN_INFO "Transmission Stopped\n");
        #endif
    }

    iowrite32(statusReg, Device.i2c_registers_address + I2C_IRQSTATUS);

    wake_up(&(Device.waitQ));
    return IRQ_HANDLED;
}

static int onProbe(struct platform_device *pdev) {
    int ret;
    struct resource *res;

    #ifdef PRINT_DEBUG
    printk(KERN_INFO "TD3 I2C Probe starting\n");
    #endif

    Device.irqNumber = platform_get_irq(pdev, 0);
    if (Device.irqNumber < 0) {
		printk(KERN_ERR "\n Error during platform_get_irq() \n");
		return Device.irqNumber;
	}
    ret = request_irq(Device.irqNumber, IrqHandler, 0, "i2c-td3", NULL);
    if (ret < 0) 
    {
        free_irq(Device.irqNumber, NULL);
        printk(KERN_ERR "KERN_ERR Error during request_irq() \n");
        return ret;
    }

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res)
    {
        printk(KERN_ERR "Error during platform_get_resource() \n");
        return -EINVAL;
    }
    Device.i2c_registers_address = ioremap(res->start, res->end - res->start + 1);
    Device.i2c_clk_module = ioremap(CM_PER_START, CM_PER_SIZE);
    Device.i2c_control_module = ioremap(CTRL_MODULE_START, CTRL_MODULE_SIZE);

    of_property_read_u32(pdev->dev.of_node, CM_PER_OFFSET, &Device.cmPerOff);
    of_property_read_u32(pdev->dev.of_node, SCL_CTRL, &Device.sclCtrl);
    of_property_read_u32(pdev->dev.of_node, SDA_CTRL, &Device.sdaCtrl);
    of_property_read_u32(pdev->dev.of_node, MODULE_CLK, &Device.moduleClk);
    of_property_read_u32(pdev->dev.of_node, TRANSFER_CLK, &Device.transferClk);

    #ifdef PRINT_DEBUG
    printk(KERN_INFO "CM_PER: %d\n", Device.cmPerOff);
    printk(KERN_INFO "SCL Pin Offset: %d\n", Device.sclCtrl);
    printk(KERN_INFO "SDA Pin Offset: %d\n", Device.sdaCtrl);
    printk(KERN_INFO "Module Clk: %d\n", Device.moduleClk);
    printk(KERN_INFO "Transfer Clk: %d\n", Device.transferClk);
    #endif

    iowrite32(CM_PER_ON, Device.i2c_clk_module + Device.cmPerOff);

    init_waitqueue_head(&(Device.waitQ));

    #ifdef PRINT_DEBUG
    printk(KERN_INFO "TD3 I2C Probe end\n");
    #endif

    return 0;
}

static int onRemove(struct platform_device *pdev)
{
    #ifdef PRINT_DEBUG
    printk(KERN_INFO "TD3 I2C Remove start\n");
    #endif
    
    free_irq(Device.irqNumber, NULL);

    iounmap(Device.i2c_registers_address);
    iounmap(Device.i2c_clk_module);
    iounmap(Device.i2c_control_module);

    #ifdef PRINT_DEBUG
    printk(KERN_INFO "TD3 I2C Remove end\n");
    #endif

	return 0;
}

static struct platform_driver i2c_bme280_driver = {
	.driver		= {
		.name	= "i2c-bme280-Driver",
		.owner	= THIS_MODULE,
        .of_match_table = device_match_table
	},
	.probe		= onProbe,
	.remove		= onRemove,
};

int transmit_locking(u_int8_t *buf, size_t count) {
    if(count <= 0) {
        return 0;
    }
    if(i2cTransmissionStatus != T_WAIT_START) {
        wait_event_interruptible(Device.waitQ, Device.i2cTransmissionStatus == T_WAIT_START);
    }

    spin_lock_irqsave(&Device.lock, Device.irqFlags);
    Device.i2cTransmissionStatus = T_STARTED;
    spin_unlock_irqrestore(&Device.lock, Device.irqFlags);

    iowrite32(BMP_ID, Device.i2c_registers_address + I2C_SA);
    iowrite32(count, Device.i2c_registers_address + I2C_CNT);
    iowrite32(CON_ENABLE | CON_MASTER | CON_TRANSMITER | CON_STP | CON_START, Device.i2c_registers_address + I2C_CON);

    Device.CurrentTransmissionInfo.buffer = buf;
    Device.CurrentTransmissionInfo.index = 0;

    iowrite32(BF | XRDY, Device.i2c_registers_address + I2C_IRQENABLE_SET);

    wait_event_interruptible(Device.waitQ, Device.i2cTransmissionStatus == T_STOPPED);

    Device.i2cTransmissionStatus = T_WAIT_START;
    iowrite32(ALL_IRQ, Device.i2c_registers_address + I2C_IRQENABLE_CLR);

    return 0;
}

int receive_locking(u_int8_t *buf, size_t count){
    if(count <= 0) {
        return 0;
    }
    if(Device.i2cTransmissionStatus != T_WAIT_START) {
        wait_event_interruptible(Device.waitQ, Device.i2cTransmissionStatus == T_WAIT_START);
    }

    spin_lock_irqsave(&Device.lock, Device.irqFlags);
    Device.i2cTransmissionStatus = T_STARTED;
    spin_unlock_irqrestore(&Device.lock, Device.irqFlags);

    iowrite32(BMP_ID, Device.i2c_registers_address + I2C_SA);
    iowrite32(count, Device.i2c_registers_address + I2C_CNT);
    iowrite32(CON_ENABLE | CON_MASTER | CON_STP | CON_START, Device.i2c_registers_address + I2C_CON);

    Device.CurrentTransmissionInfo.buffer = buf;
    Device.CurrentTransmissionInfo.index = 0;

    iowrite32(BF | RRDY, Device.i2c_registers_address + I2C_IRQENABLE_SET);

    wait_event_interruptible(Device.waitQ, Device.i2cTransmissionStatus == T_STOPPED);

    Device.i2cTransmissionStatus = T_WAIT_START;
    iowrite32(ALL_IRQ, Device.i2c_registers_address + I2C_IRQENABLE_CLR);

    return 0;
}

ssize_t onRead (struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    int32_t result = 0;
    u_int8_t config[2] = { BMP_R_CTRL_MEAS, BMP_MODE_NORMAL | BMP_TEMP_HIGH_RES }; // mejor resolucion posible para T, desactivo P, y lo paso a modo normal de funcionamiento
    u_int8_t askRead[1] = { BMP_R_TEMP_MSB };
    u_int8_t read[3] = {0};
    int readAmount = count * sizeof(char)/sizeof(u_int8_t)/4;
    int i;
    #ifdef PRINT_DEBUG
    printk(KERN_INFO "TD3 I2C onRead start\n\n");
    #endif

    transmit_locking(config, 2);

    for(i=0; i<readAmount; i++) {
        transmit_locking(askRead, 1);
        receive_locking(read, 3);

        result = 0;
        result = read[0];
        result <<= 8;
        result |= read[1];
        result <<= 8;
        result |= read[2];
        result >>= 4;

        #ifdef PRINT_DEBUG
        printk(KERN_INFO "T Raw: %d\n", result);
        #endif

        if(copy_to_user(((u_int8_t*)buf)+i*4, (u_int8_t*)&result, 4)) {
            printk(KERN_ERR "Error on copy to user\n\n");
        }
    }

    #ifdef PRINT_DEBUG
    printk(KERN_INFO "TD3 I2C onRead end\n\n");
    #endif

	return readAmount*4/sizeof(char);
}

loff_t onLlseek (struct file *filp, loff_t off, int whence)
{
    printk(KERN_INFO "TD3 I2C onLlseek\n");
	return -EPERM;
}

ssize_t onWrite (struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    u_int8_t kbuf[sizeof(char)/sizeof(u_int8_t)*count + 1];
    #ifdef PRINT_DEBUG
    printk(KERN_INFO "TD3 I2C onWrite start\n");
    #endif

    if(copy_from_user(kbuf, buf, sizeof(char)/sizeof(u_int8_t)*count)){
        printk(KERN_ERR "Error on copy from user\n\n");
    }
    transmit_locking(kbuf, count);

    #ifdef PRINT_DEBUG
    printk(KERN_INFO "TD3 I2C onWrite end\n");
    #endif

	return count;
}

int GetId(u_int8_t* dest) {
    u_int8_t askId[1] = { BMP_R_ID };
    u_int8_t id[1] = {0};
    transmit_locking(askId, 1);
    receive_locking(id, 1);

    if(copy_to_user(dest, id, sizeof(u_int8_t))) {
        printk(KERN_ERR "Error copy to user\n");
        return -1;
    }

    return 0;
}

int CalibrateSensor(struct BmpCalibrationValues* dest) {
    u_int8_t askCalibration[1] = { BMP_R_CALIB_00 };
    u_int8_t calibration[6] = {0};
    struct BmpCalibrationValues aux;

    transmit_locking(askCalibration, 1);
    receive_locking(calibration, 6);

    aux.Cal1 = calibration[1];
    aux.Cal1 <<= 8;
    aux.Cal1 |= calibration[0];
    aux.Cal2 = calibration[3];
    aux.Cal2 <<= 8;
    aux.Cal2 |= calibration[2];
    aux.Cal3 = calibration[5];
    aux.Cal3 <<= 8;
    aux.Cal3 |= calibration[4];

    if(copy_to_user(dest, &aux, sizeof(struct BmpCalibrationValues))) {
        printk(KERN_ERR "Error copy to user\n");
        return -1;
    }

    return 0;
}

long onIoctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
    printk(KERN_INFO "TD3 I2C onIoctl\n");
    switch(cmd){
        case GET_CALIBRATION:
            #ifdef PRINT_DEBUG
            printk(KERN_INFO "GET_CALIBRATION called\n");
            #endif
            return CalibrateSensor((struct BmpCalibrationValues*)arg);
            break;
        case GET_ID:
            #ifdef PRINT_DEBUG
            printk(KERN_INFO "GET_ID called\n");
            #endif
            return GetId((u_int8_t*) arg);
        default:
            return -ENOTTY;
    }
	return -EPERM;
}

int onOpen (struct inode *inode, struct file *filp)
{
    unsigned int auxReg;
    #ifdef PRINT_DEBUG
    printk(KERN_INFO "TD3 I2C onOpen starting\n");
    #endif

    auxReg = ioread32(Device.i2c_control_module + Device.sdaCtrl);
    auxReg = PULL_UP;
    iowrite32(auxReg, Device.i2c_control_module + Device.sdaCtrl);

    auxReg = ioread32(Device.i2c_control_module + Device.sclCtrl);
    auxReg = PULL_UP;
    iowrite32(auxReg, Device.i2c_control_module + Device.sclCtrl);

    iowrite32(0x00, Device.i2c_registers_address + I2C_CON);
    iowrite32((Device.moduleClk/FINAL_CLK)-1, Device.i2c_registers_address + I2C_PSC);
    iowrite32(Device.moduleClk/Device.transferClk-7, Device.i2c_registers_address + I2C_SCLL);
    iowrite32(Device.moduleClk/Device.transferClk-5, Device.i2c_registers_address + I2C_SCLH);
    iowrite32(OWN_ADDRESS, Device.i2c_registers_address + I2C_OA);
    iowrite32(SYSC_VALUE, Device.i2c_registers_address + I2C_SYSC);
    iowrite32(CON_ENABLE, Device.i2c_registers_address + I2C_CON);

    iowrite32(0x6FFF, Device.i2c_registers_address + I2C_IRQENABLE_CLR);

    Device.i2cTransmissionStatus = T_WAIT_START;

    #ifdef PRINT_DEBUG
    printk(KERN_INFO "TD3 I2C onOpen end\n");
    #endif

	return 0;
}

int onRelease (struct inode *inode, struct file *filp)
{
    #ifdef PRINT_DEBUG
    printk(KERN_INFO "TD3 I2C onRelease\n");
    #endif
	return 0;
}

static const struct file_operations device_file_ops = {
    .owner = THIS_MODULE,
	.llseek = onLlseek,
	.read = onRead,
	.write = onWrite,
	.unlocked_ioctl = onIoctl,
	.open = onOpen,
	.release = onRelease
};

 static int SetDriverPermission(struct device *dev, struct kobj_uevent_env *env){
    add_uevent_var(env, "DEVMODE=%#o", 0666);
    return 0;
}

static int __init i2c_module_init(void)
{
    int result;
    #ifdef PRINT_DEBUG
	printk(KERN_INFO "TD3 I2C Init starting\n");
    #endif

    result = alloc_chrdev_region(&(Device.dev), 0, DEVICE_COUNT, Device.DriverName);
    if(result < 0){
        printk(KERN_ERR "TD3 I2C Error during alloc__chrdev_region \n");
        return -1;
    }

    #ifdef PRINT_DEBUG
    printk(KERN_INFO "Major No: %d Minor_no : %d \n", MAJOR(Device.dev), MINOR(Device.dev));
    #endif

    Device.i2c_cdev = cdev_alloc();
    if(Device.i2c_cdev == NULL) {
        printk(KERN_ERR "Error during cdev_alloc()\n");
        unregister_chrdev_region(Device.dev, DEVICE_COUNT);
        return -1;
    }
    Device.i2c_cdev->owner = THIS_MODULE;
    Device.i2c_cdev->ops = &device_file_ops;

    if(cdev_add(Device.i2c_cdev, Device.dev, 1) < 0) {
        printk(KERN_ERR "Error during cdev_add()\n");
        cdev_del(Device.i2c_cdev);
        unregister_chrdev_region(Device.dev, DEVICE_COUNT);
        return -1;
    }

    Device.i2c_class = class_create(THIS_MODULE, "i2c-td3");
    if (IS_ERR(Device.i2c_class)){
        printk(KERN_ERR "Error during class_create()\n");
        cdev_del(Device.i2c_cdev);
        unregister_chrdev_region(Device.dev, DEVICE_COUNT);
        return -1;
    }

    Device.i2c_class->dev_uevent = SetDriverPermission;

    Device.i2c_device = device_create(Device.i2c_class, NULL, Device.dev, NULL, "i2c-td3-device");
    if (IS_ERR(Device.i2c_device)){
        printk(KERN_ERR "Error during device_create()\n");
        class_destroy(Device.i2c_class);
        cdev_del(Device.i2c_cdev);
        unregister_chrdev_region(Device.dev, DEVICE_COUNT);
        return -1;
    }

    #ifdef PRINT_DEBUG
    printk(KERN_INFO "TD3 I2C Init end\n");
    #endif

	return platform_driver_register(&i2c_bme280_driver);
}

static void __exit i2c_module_exit(void)
{
    #ifdef PRINT_DEBUG
	printk(KERN_INFO "TD3 I2C Exit starting\n");
    #endif

    platform_driver_unregister(&i2c_bme280_driver);

    device_destroy(Device.i2c_class, Device.dev);
    class_destroy(Device.i2c_class);
    cdev_del(Device.i2c_cdev);
    unregister_chrdev_region(Device.dev, DEVICE_COUNT);


    #ifdef PRINT_DEBUG
    printk(KERN_INFO "TD3 I2C Exit end\n");
    #endif
}

module_init(i2c_module_init);
module_exit(i2c_module_exit);