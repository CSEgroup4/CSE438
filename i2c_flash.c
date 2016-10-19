// EEPROM driver

#include <linux/module.h>  // Module Defines and Macros (THIS_MODULE)
#include <linux/kernel.h>  //
#include <linux/fs.h>	   // Inode and File types
#include <linux/cdev.h>    // Character Device Types and functions.
#include <linux/types.h>
#include <linux/slab.h>	   // Kmalloc/Kfree
#include <asm/uaccess.h>   // Copy to/from user space
#include <linux/string.h>
#include <linux/device.h>  // Device Creation / Destruction functions
#include <linux/init.h>
#include <linux/delay.h>

#include <linux/gpio.h>
#include <asm/gpio.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <linux/workqueue.h>
#include <linux/semaphore.h>

#define DEVICE_NAME "i2c_flash"
#define LED_PIN 40 
#define I2C_ADDR 0x50 
#define I2C_PIN 29
#define I2C_MUX_NAME "i2c_mux"
#define byte uint8_t
#define EEPROM_BUSY 1
#define EEPROM_FREE 0

struct eeprom_dev {
    struct cdev cdev;
    struct i2c_adapter *i2c_adapter;
    struct i2c_client *i2c_client;
    uint16_t current_addr;
    struct workqueue_struct *wq;
    uint8_t *read_buffer;
};

struct eeprom_work_struct {
    struct work_struct my_work;
    struct eeprom_dev *devp;
    uint8_t *data;
    size_t count; // number of bytes in data
};

enum ioctl_id {
    FLASHGETS = 1001,
    FLASHGETP = 1002,
    FLASHSETP = 1003,
    FLASHERASE = 1004
};

static dev_t eeprom_dev_number;
struct class *eeprom_dev_class;
static struct device *eeprom_dev_device;
struct eeprom_dev *eeprom_devp;
static uint16_t current_addr;
int eeprom_status; // 0 - busy, 1 - free
static DEFINE_SEMAPHORE(eeprom_mutex);
int read_data_status; // 0 - no data, 1 - has data
static DEFINE_SEMAPHORE(data_mutex);

void increment_address(struct eeprom_dev *devp)
{
    devp->current_addr += 64;
    if (devp->current_addr > 32704) {
        // set current_addr to 0x00
        devp->current_addr = 0;
    }
}

// stores the 2 MSB from 16-bit address to byteArray[0]
// stores the 2 LSB from 16-bit address to byteArray[1]
void parse_address(uint16_t address, uint8_t *byteArray)
{
    byteArray[0] = (address >> 8) & 0xFF;
    byteArray[1] = address & 0xFF;
}

// checks eeprom status bool
// returns 1 if eeprom is busy
// returns 0 if eeprom is not busy
int get_eeprom_status()
{
    int status;
    down(&eeprom_mutex);
    status = eeprom_status;
    up(&eeprom_mutex);
    return status;
}

// sets eeprom status bool
// sets to 1 if eeprom is busy
// sets to 0 if eeprom is not busy
void set_eeprom_status(int status)
{
    down(&eeprom_mutex);
    eeprom_status = status;
    up(&eeprom_mutex);
}

// checks if global buffer has data
// returns 0 if it does not have data
// returns 1 if it has data
int get_data_status()
{
    int status;
    down(&data_mutex);
    status = read_data_status;
    up(&data_mutex);
    return status;
}

// sets global buffer status
// sets to 0 if it does not have data
// sets to 1 if it has data
void set_data_status(int status)
{
    down(&data_mutex);
    read_data_status = status;
    up(&data_mutex);
}

int eeprom_open(struct inode *inode, struct file *file)
{
    struct eeprom_dev *eeprom_devp;
    //printk(KERN_DEBUG "eeprom open.\n");
    
    eeprom_devp = (struct eeprom_dev *)container_of(inode->i_cdev, struct eeprom_dev, cdev);
    file->private_data = eeprom_devp;
    return 0;
}

int eeprom_release(struct inode *inode, struct file *file)
{
    //printk(KERN_DEBUG "eeprom release.\n");
    return 0;
}

// reads a single byte and moves the internal address counter to the next byte
// assumes caller function sets current_address
void go_to_address(uint16_t address)
{
    // byte data;
    uint8_t data;
    struct i2c_msg msg[2];
    uint8_t addr[2];
    
    

    if (address == 0)
    {
        // read byte from 0x7FFF
        address = 0x7FFF;
    }
    else
    {
        // read byte from data - 1
        address = address - 1;
    }

    parse_address(address, addr);


    msg[0].addr = I2C_ADDR;
    msg[0].flags = 0;
    msg[0].len = 2;
    msg[0].buf = addr;
    
    msg[1].addr = I2C_ADDR;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = &data;
    if (i2c_transfer(eeprom_devp->i2c_adapter, msg, 2) < 0)
    {
        //printk(KERN_DEBUG "Error: failed to read single byte.\n");
    }
}

int flash_erase(struct eeprom_dev *devp)
{
    uint16_t addr = 0;
    int i;
    uint8_t buffer[66];
    struct i2c_msg msg[1];
    uint8_t data[64];
    int res = 1;

    if (get_eeprom_status() == EEPROM_BUSY)
        return -EBUSY;

    set_eeprom_status(EEPROM_BUSY);

    for (i = 0; i < 64; i++) {
        data[i] = 0xFF;
    }

    buffer[0] = 0x00;
    buffer[1] = 0x00;

    for (i = 0; i < 64; i++) {
        buffer[i + 2] = data[i];
    }

    // send to device
    //printk(KERN_DEBUG "Erasing eeprom\n");

    gpio_set_value_cansleep(LED_PIN, 1);
    devp->current_addr = addr;
    for (addr = 0; addr <= 32704; addr += 64)
    {
        parse_address(addr, buffer);
        
        msg[0].addr = I2C_ADDR;
        msg[0].flags = 0;
        msg[0].len = 66;
        msg[0].buf = buffer;
        // send to device

        if (i2c_transfer(devp->i2c_adapter, msg, 1) < 0) {
            //printk(KERN_DEBUG "Failed to erase eeprom.\n");
            res = 0;
            break;
        }
        increment_address(devp);
        msleep(5);
    }

    set_eeprom_status(EEPROM_FREE);
    gpio_set_value_cansleep(LED_PIN, 0);
    if (res == 1)
        //printk(KERN_DEBUG "Eeprom erased sucessfully\n");
    return 0;
}


// FLASHSETP:   Sets current page position
// FLASHGETP:   Returns current page position
// FLASHERASE:  Erases flash by storing 0xFF in each byte
int eeprom_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    int data;
    void __user *argp = (void __user *)arg;
    struct eeprom_dev *devp = (struct eeprom_dev *)file->private_data;
    
    switch (cmd) {
        case FLASHERASE:
            //printk(KERN_DEBUG "Flash erase...\n");
            flash_erase(devp);
            break;
            
        case FLASHGETP:
            //printk(KERN_DEBUG "Flash get current page position...\n");
            ret = devp->current_addr;
            break;
            
        case FLASHSETP:
            //printk(KERN_DEBUG "Setting current page position...\n");
            copy_from_user((void*)&data, (void*)arg, sizeof(int));
            devp->current_addr = (uint16_t) data;
            //printk(KERN_DEBUG "Page position set to %u\n", devp->current_addr);
            go_to_address(devp->current_addr);

            break;
            
        case FLASHGETS:
            //printk(KERN_DEBUG "Flash get status...\n");
            ret = get_eeprom_status();
            break;
            
        default:
            //printk(KERN_DEBUG "Error: Invalid argument %u.\n", cmd);
            break;
    }
    return ret;
}

void workqueue_write(struct work_struct *work)
{
    int pageNum;
    int data_start;
    int success = 1;
    int i;
    uint8_t buffer[66];
    struct i2c_msg msg[1];

    struct eeprom_work_struct *my_work;
    my_work = (struct eeprom_work_struct *) work;
    gpio_set_value_cansleep(LED_PIN, 1);

    for (pageNum = 0; pageNum < my_work->count; pageNum++)
    {
        parse_address(my_work->devp->current_addr, buffer);

        // store data in buffer
        data_start = pageNum * 64;
        for (i = 0; i < 64; i++)
        {
            buffer[i + 2] = my_work->data[data_start + i];
        }

        msg[0].addr = I2C_ADDR;
        msg[0].flags = 0;
        msg[0].len = 66;
        msg[0].buf = buffer;

        if (i2c_transfer(my_work->devp->i2c_adapter, msg, 1) < 0)
        {
            //printk(KERN_DEBUG "Error: failed to write messages.\n");
            success = 0;
        }
        else
        {
            increment_address(my_work->devp);
        }
        msleep(5);
    }
    kfree(my_work->data);
    kfree(work);
    set_eeprom_status(EEPROM_FREE);
    gpio_set_value_cansleep(LED_PIN, 0);
    return;
}

ssize_t eeprom_write(struct file *file, uint8_t *buf, size_t count, loff_t *loff)
{
    struct eeprom_dev *devp;
    int status;
    struct eeprom_work_struct *work;

    //printk(KERN_DEBUG "eeprom write.\n");
    devp = (struct eeprom_dev *)file->private_data;
    status = get_eeprom_status();
    if (status == EEPROM_FREE) {
        set_eeprom_status(EEPROM_BUSY);
        work = (struct eeprom_work_struct *) kmalloc(sizeof(struct eeprom_work_struct), GFP_KERNEL);
        INIT_WORK((struct work_struct *)work, workqueue_write);
        work->data = (uint8_t *) kmalloc(64 * count, GFP_KERNEL);
        copy_from_user((void *)work->data, (void *)buf, 64 * count);
        work->count = count;
        work->devp = devp;
        queue_work(devp->wq, (struct work_struct *)work);
        return 0;
    }
    return -EBUSY;
}

void workqueue_read(struct work_struct *work)
{
    struct i2c_msg msg[2];
    int i;
    int pageNum;
    uint8_t data[64];
    int buffer_start;

    struct eeprom_work_struct *my_work;

    my_work = (struct eeprom_work_struct *) work;

    gpio_set_value_cansleep(LED_PIN, 1);



    for (pageNum = 0; pageNum < my_work->count; pageNum++)
    {
        buffer_start = pageNum * 64;

        msg[0].addr = I2C_ADDR;
        msg[0].flags = I2C_M_RD;
        msg[0].len = 64;
        msg[0].buf = data;
        
        if (i2c_transfer(my_work->devp->i2c_adapter, msg, 1) < 0)
        {
            //printk(KERN_DEBUG "Error: failed to read messages.\n");
            // any read failure should prevent data from being copied to user
            set_data_status(0);
        }
        else
        {
            //printk(KERN_DEBUG "Read successful.\n");
            for (i = 0; i < 64; i++)
            {
                my_work->devp->read_buffer[buffer_start + i] = data[i];
            }
            
            increment_address(my_work->devp);
        }
        msleep(5);
    }
    set_data_status(1);
    set_eeprom_status(EEPROM_FREE);
    gpio_set_value_cansleep(LED_PIN, 0);
    kfree(work);
    return;
}

ssize_t eeprom_read(struct file *file, uint8_t *buf, size_t count, loff_t *loff)
{
    int i;
    uint8_t buffer[64 * count];

    struct eeprom_work_struct *work;

    struct eeprom_dev *devp = (struct eeprom_dev *)file->private_data;

    if (get_data_status() == 0) {
        if (get_eeprom_status() == EEPROM_FREE) { // eeprom is not busy
            set_eeprom_status(EEPROM_BUSY);

            // submit to workqueue
            work = (struct eeprom_work_struct *) kmalloc(sizeof(struct eeprom_work_struct), GFP_KERNEL);
            INIT_WORK((struct work_struct *)work, workqueue_read);


            devp->read_buffer = (uint8_t *) kmalloc(64 * count, GFP_KERNEL);
            work->count = count;
            work->devp = devp;
            queue_work(devp->wq, (struct work_struct *)work);
            return -EAGAIN;
        } else { // eeprom is busy
            //printk(KERN_DEBUG "eeprom is busy\n");
            return -EBUSY;
        }
    } else {
        //printk(KERN_DEBUG "Sending to user\n");
        copy_to_user((void *)buf, (void *)devp->read_buffer, 64 * count);
        kfree(devp->read_buffer);
        set_data_status(0);
        return 0;
    }    
}


static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = eeprom_open,
    .release = eeprom_release,
    .write = eeprom_write,
    .read = eeprom_read,
    .unlocked_ioctl = eeprom_ioctl,
};

int __init eeprom_init(void)
{
    if (alloc_chrdev_region(&eeprom_dev_number, 0, 1, DEVICE_NAME) < 0)
    {
        printk(KERN_ALERT "Error: failed to register dev number\n");
        return -1;
    }
    
    eeprom_dev_class = class_create(THIS_MODULE, DEVICE_NAME);
    
    eeprom_devp = kmalloc(sizeof(struct eeprom_dev), GFP_KERNEL);
    
    if (!eeprom_devp)
    {
        printk(KERN_ALERT "Error: bad kmalloc\n");
        return -ENOMEM;
    }
    
    cdev_init(&eeprom_devp->cdev, &fops);
    eeprom_devp->cdev.owner = THIS_MODULE;
    
    if (cdev_add(&eeprom_devp->cdev, eeprom_dev_number, 1))
    {
        printk(KERN_ALERT "Error: bad cdev\n");
        return -1;
    }
    
    eeprom_dev_device = device_create(eeprom_dev_class, NULL, MKDEV(MAJOR(eeprom_dev_number), 0), NULL, DEVICE_NAME);
    
    
    // setup led on IO10
    if (gpio_request(LED_PIN, "led_io10") < 0)
    {
        printk(KERN_ALERT "Error: requesting gpio.\n");
        return -1;
    }
    
    if (gpio_direction_output(LED_PIN, 0) < 0)
    {
        printk(KERN_ALERT "Error: setting direction to output.\n");
        return -1;
    }
    
    gpio_set_value_cansleep(LED_PIN, 0);
    
    // setup i2c mux
    if (gpio_request(I2C_PIN, I2C_MUX_NAME) < 0)
    {
        printk(KERN_ALERT "Error: requesting gpio for i2c mux setup.\n");
        return -1;
    }
    
    if (gpio_direction_output(I2C_PIN, 0) < 0)
    {
        printk(KERN_ALERT "Error: setting direction to output for i2c mux.\n");
        return -1;
    }
    
    gpio_set_value_cansleep(I2C_PIN, 0);
    
    // create i2c adapter
    eeprom_devp->i2c_adapter = i2c_get_adapter(0);

    // client struct
    eeprom_devp->i2c_client = (struct i2c_client *)kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
    eeprom_devp->i2c_client->addr = I2C_ADDR;
    snprintf(eeprom_devp->i2c_client->name, I2C_NAME_SIZE, "i2c_eeprom");
    eeprom_devp->i2c_client->adapter = eeprom_devp->i2c_adapter;
    
    eeprom_devp->current_addr = 0x00;

    eeprom_devp->wq = create_workqueue("eeprom_workqueue");
    eeprom_status = EEPROM_FREE;
    read_data_status = 0;

    //printk(KERN_DEBUG "eeprom driver initialized.\n");
    return 0;
}

void __exit eeprom_exit(void)
{
    // cleanup workqueue
    flush_workqueue(eeprom_devp->wq);
    destroy_workqueue(eeprom_devp->wq);

    // close i2c
    i2c_put_adapter(eeprom_devp->i2c_adapter);
    kfree(eeprom_devp->i2c_client);

    // turn off led and unexport
    gpio_set_value_cansleep(LED_PIN, 0);
    gpio_free(LED_PIN);
    
    // unexport gpio for i2c
    gpio_free(I2C_PIN);
    
    unregister_chrdev_region(eeprom_dev_number, 1);
    
    device_destroy(eeprom_dev_class, MKDEV(MAJOR(eeprom_dev_number), 0));
    cdev_del(&eeprom_devp->cdev);
    kfree(eeprom_devp);
    
    class_destroy(eeprom_dev_class);
    
    //printk(KERN_DEBUG "eeprom driver removed.\n");
}

module_init(eeprom_init);
module_exit(eeprom_exit);
MODULE_LICENSE("GPL");
