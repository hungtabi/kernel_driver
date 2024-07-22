#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/uaccess.h>  //copy_to/from_user()
#include <linux/gpio.h>     //GPIO
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/i2c.h>

#define I2C_BUS_AVAILABLE   (          1 )
#define SLAVE_DEVICE_NAME   ( "TABI_OLED" )
#define SSD1306_SLAVE_ADDR  (       0x3C )
#define SSD1306_MAX_SEG         (        128 )
#define SSD1306_MAX_LINE        (          7 )
#define SSD1306_DEF_FONT_SIZE   (          5 )

MODULE_LICENSE("GPL");
MODULE_AUTHOR("hungtabi1478@gmail.com");
MODULE_DESCRIPTION("on of led");
MODULE_VERSION("1.32");

static struct i2c_adapter *tabi_i2c_adapter     = NULL;
static struct i2c_client  *tabi_i2c_client_oled = NULL;

static uint8_t SSD1306_LineNum   = 0;
static uint8_t SSD1306_CursorPos = 0;
static uint8_t SSD1306_FontSize  = SSD1306_DEF_FONT_SIZE;
static char *value = "OFF";

// ky tu hien thi
static const unsigned char SSD1306_font[][SSD1306_DEF_FONT_SIZE]=
{
    {0x00, 0x00, 0x00, 0x00, 0x00},   // space
    {0x00, 0x00, 0x2f, 0x00, 0x00},   // !
    {0x00, 0x07, 0x00, 0x07, 0x00},   // "
    {0x14, 0x7f, 0x14, 0x7f, 0x14},   // #
    {0x24, 0x2a, 0x7f, 0x2a, 0x12},   // $
    {0x23, 0x13, 0x08, 0x64, 0x62},   // %
    {0x36, 0x49, 0x55, 0x22, 0x50},   // &
    {0x00, 0x05, 0x03, 0x00, 0x00},   // '
    {0x00, 0x1c, 0x22, 0x41, 0x00},   // (
    {0x00, 0x41, 0x22, 0x1c, 0x00},   // )
    {0x14, 0x08, 0x3E, 0x08, 0x14},   // *
    {0x08, 0x08, 0x3E, 0x08, 0x08},   // +
    {0x00, 0x00, 0xA0, 0x60, 0x00},   // ,
    {0x08, 0x08, 0x08, 0x08, 0x08},   // -
    {0x00, 0x60, 0x60, 0x00, 0x00},   // .
    {0x20, 0x10, 0x08, 0x04, 0x02},   // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E},   // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00},   // 1
    {0x42, 0x61, 0x51, 0x49, 0x46},   // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31},   // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10},   // 4
    {0x27, 0x45, 0x45, 0x45, 0x39},   // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30},   // 6
    {0x01, 0x71, 0x09, 0x05, 0x03},   // 7
    {0x36, 0x49, 0x49, 0x49, 0x36},   // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E},   // 9
    {0x00, 0x36, 0x36, 0x00, 0x00},   // :
    {0x00, 0x56, 0x36, 0x00, 0x00},   // ;
    {0x08, 0x14, 0x22, 0x41, 0x00},   // <
    {0x14, 0x14, 0x14, 0x14, 0x14},   // =
    {0x00, 0x41, 0x22, 0x14, 0x08},   // >
    {0x02, 0x01, 0x51, 0x09, 0x06},   // ?
    {0x32, 0x49, 0x59, 0x51, 0x3E},   // @
    {0x7C, 0x12, 0x11, 0x12, 0x7C},   // A
    {0x7F, 0x49, 0x49, 0x49, 0x36},   // B
    {0x3E, 0x41, 0x41, 0x41, 0x22},   // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C},   // D
    {0x7F, 0x49, 0x49, 0x49, 0x41},   // E
    {0x7F, 0x09, 0x09, 0x09, 0x01},   // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A},   // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F},   // H
    {0x00, 0x41, 0x7F, 0x41, 0x00},   // I
    {0x20, 0x40, 0x41, 0x3F, 0x01},   // J
    {0x7F, 0x08, 0x14, 0x22, 0x41},   // K
    {0x7F, 0x40, 0x40, 0x40, 0x40},   // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},   // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F},   // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E},   // O
    {0x7F, 0x09, 0x09, 0x09, 0x06},   // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E},   // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46},   // R
    {0x46, 0x49, 0x49, 0x49, 0x31},   // S
    {0x01, 0x01, 0x7F, 0x01, 0x01},   // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F},   // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F},   // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F},   // W
    {0x63, 0x14, 0x08, 0x14, 0x63},   // X
    {0x07, 0x08, 0x70, 0x08, 0x07},   // Y
    {0x61, 0x51, 0x49, 0x45, 0x43},   // Z
    {0x00, 0x7F, 0x41, 0x41, 0x00},   // [
    {0x55, 0xAA, 0x55, 0xAA, 0x55},   // Backslash (Checker pattern)
    {0x00, 0x41, 0x41, 0x7F, 0x00},   // ]
    {0x04, 0x02, 0x01, 0x02, 0x04},   // ^
    {0x40, 0x40, 0x40, 0x40, 0x40},   // _
    {0x00, 0x03, 0x05, 0x00, 0x00},   // `
    {0x20, 0x54, 0x54, 0x54, 0x78},   // a
    {0x7F, 0x48, 0x44, 0x44, 0x38},   // b
    {0x38, 0x44, 0x44, 0x44, 0x20},   // c
    {0x38, 0x44, 0x44, 0x48, 0x7F},   // d
    {0x38, 0x54, 0x54, 0x54, 0x18},   // e
    {0x08, 0x7E, 0x09, 0x01, 0x02},   // f
    {0x18, 0xA4, 0xA4, 0xA4, 0x7C},   // g
    {0x7F, 0x08, 0x04, 0x04, 0x78},   // h
    {0x00, 0x44, 0x7D, 0x40, 0x00},   // i
    {0x40, 0x80, 0x84, 0x7D, 0x00},   // j
    {0x7F, 0x10, 0x28, 0x44, 0x00},   // k
    {0x00, 0x41, 0x7F, 0x40, 0x00},   // l
    {0x7C, 0x04, 0x18, 0x04, 0x78},   // m
    {0x7C, 0x08, 0x04, 0x04, 0x78},   // n
    {0x38, 0x44, 0x44, 0x44, 0x38},   // o
    {0xFC, 0x24, 0x24, 0x24, 0x18},   // p
    {0x18, 0x24, 0x24, 0x18, 0xFC},   // q
    {0x7C, 0x08, 0x04, 0x04, 0x08},   // r
    {0x48, 0x54, 0x54, 0x54, 0x20},   // s
    {0x04, 0x3F, 0x44, 0x40, 0x20},   // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C},   // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C},   // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C},   // w
    {0x44, 0x28, 0x10, 0x28, 0x44},   // x
    {0x1C, 0xA0, 0xA0, 0xA0, 0x7C},   // y
    {0x44, 0x64, 0x54, 0x4C, 0x44},   // z
    {0x00, 0x10, 0x7C, 0x82, 0x00},   // {
    {0x00, 0x00, 0xFF, 0x00, 0x00},   // |
    {0x00, 0x82, 0x7C, 0x10, 0x00},   // }
    {0x00, 0x06, 0x09, 0x09, 0x06}    // ~ (Degrees)
};


// ham viet vao i2c
static int i2c_write (char *buf, int len)
{
    int ret = i2c_master_send(tabi_i2c_client_oled, buf, len);
    return ret;
}

// ham viet vao oled
static void SSD1306_Write(bool is_cmd, unsigned char data)
{
    int ret;
    unsigned char buf[2] = {0};
    
    if( is_cmd == true )
    {
        buf[0] = 0x00;
    }
    else
    {
        buf[0] = 0x40;
    }
    
    buf[1] = data;
    
    ret = i2c_write(buf, 2);
}

// xac dinh vi tri
static void SSD1306_SetCursor( uint8_t lineNo, uint8_t cursorPos )
{
  /* Move the Cursor to specified position only if it is in range */
  if((lineNo <= SSD1306_MAX_LINE) && (cursorPos < SSD1306_MAX_SEG))
  {
    SSD1306_LineNum   = lineNo;
    SSD1306_CursorPos = cursorPos;
    SSD1306_Write(true, 0x21);
    SSD1306_Write(true, cursorPos);
    SSD1306_Write(true, SSD1306_MAX_SEG-1);
    SSD1306_Write(true, 0x22);
    SSD1306_Write(true, lineNo);
    SSD1306_Write(true, SSD1306_MAX_LINE);
  }
}

static void  SSD1306_GoToNextLine( void )
{
  SSD1306_LineNum++;
  SSD1306_LineNum = (SSD1306_LineNum & SSD1306_MAX_LINE);
  SSD1306_SetCursor(SSD1306_LineNum,0);
}

static void SSD1306_PrintChar(unsigned char c)
{
  uint8_t data_byte;
  uint8_t temp = 0;
  if( (( SSD1306_CursorPos + SSD1306_FontSize ) >= SSD1306_MAX_SEG ) ||
      ( c == '\n' )
  )
  {
    SSD1306_GoToNextLine();
  }
  if( c != '\n' )
  {
    c -= 0x20;  //or c -= ' ';
    do
    {
      data_byte= SSD1306_font[c][temp];
      SSD1306_Write(false, data_byte);
      SSD1306_CursorPos++;
      
      temp++;
      
    } while ( temp < SSD1306_FontSize);
    SSD1306_Write(false, 0x00);
    SSD1306_CursorPos++;
  }
}

static void SSD1306_String(unsigned char *str)
{
  while(*str)
  {
    SSD1306_PrintChar(*str++);
  }
}

static int SSD1306_init (void)
{
    msleep(100);
    
    SSD1306_Write(true, 0xAE); // Entire Display OFF
    SSD1306_Write(true, 0xD5); // Set Display Clock Divide Ratio and Oscillator Frequency
    SSD1306_Write(true, 0x80); // Default Setting for Display Clock Divide Ratio and Oscillator Frequency that is recommended
    SSD1306_Write(true, 0xA8); // Set Multiplex Ratio
    SSD1306_Write(true, 0x3F); // 64 COM lines
    SSD1306_Write(true, 0xD3); // Set display offset
    SSD1306_Write(true, 0x00); // 0 offset
    SSD1306_Write(true, 0x40); // Set first line as the start line of the display
    SSD1306_Write(true, 0x8D); // Charge pump
    SSD1306_Write(true, 0x14); // Enable charge dump during display on
    SSD1306_Write(true, 0x20); // Set memory addressing mode
    SSD1306_Write(true, 0x00); // Horizontal addressing mode
    SSD1306_Write(true, 0xA1); // Set segment remap with column address 127 mapped to segment 0
    SSD1306_Write(true, 0xC8); // Set com output scan direction, scan from com63 to com 0
    SSD1306_Write(true, 0xDA); // Set com pins hardware configuration
    SSD1306_Write(true, 0x12); // Alternative com pin configuration, disable com left/right remap
    SSD1306_Write(true, 0x81); // Set contrast control
    SSD1306_Write(true, 0x80); // Set Contrast to 128
    SSD1306_Write(true, 0xD9); // Set pre-charge period
    SSD1306_Write(true, 0xF1); // Phase 1 period of 15 DCLK, Phase 2 period of 1 DCLK
    SSD1306_Write(true, 0xDB); // Set Vcomh deselect level
    SSD1306_Write(true, 0x20); // Vcomh deselect level ~ 0.77 Vcc
    SSD1306_Write(true, 0xA4); // Entire display ON, resume to RAM content display
    SSD1306_Write(true, 0xA6); // Set Display in Normal Mode, 1 = ON, 0 = OFF
    SSD1306_Write(true, 0x2E); // Deactivate scroll
    SSD1306_Write(true, 0xAF);
    
    return 0;
}

// oled fill
static void SSD1306_Fill(unsigned char data)
{
    unsigned int total  = 128 * 8;
    unsigned int i      = 0;
    
    for(i = 0; i < total; i++)
    {
        SSD1306_Write(false, data);
    }
}

static int tabi_probe (struct i2c_client *client, const struct i2c_device_id *id)
{
    SSD1306_init();
    SSD1306_SetCursor(0,55);
    SSD1306_String("TABI\n");
    SSD1306_String("LED MODE:");
    SSD1306_SetCursor(1,56);
    SSD1306_String(value);
    pr_info("OLED Probed!!!\n");
    return 0;
}

static void tabi_remove(struct i2c_client *client)
{
    msleep(1000);
    SSD1306_SetCursor(0,0);
    SSD1306_Fill(0x00);
    pr_info("OLED Removed!!!\n");
    return ;
}

static const struct i2c_device_id tabi_oled_id[] = {
        { SLAVE_DEVICE_NAME, 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, tabi_oled_id);

static struct i2c_driver tabi_oled_driver = {
        .driver = {
            .name   = SLAVE_DEVICE_NAME,
            .owner  = THIS_MODULE,
        },
        .probe          = tabi_probe,
        .remove         = tabi_remove,
        .id_table       = tabi_oled_id,
};

static struct i2c_board_info oled_i2c_board_info = {
        I2C_BOARD_INFO(SLAVE_DEVICE_NAME, SSD1306_SLAVE_ADDR)
    };
// dia chi vat ly
#define GPIO_ADDRESS 0x3f200000
// con trỏ thanh ghi
static unsigned int *gpio_register = NULL;
void led_on (void)
{
    unsigned int *gpio_on_set = (unsigned int*)((char*)gpio_register + 0x1C);
    *gpio_on_set |= (1 << 21);
}

void led_off (void)
{
    unsigned int *gpio_off_set = (unsigned int*)((char*)gpio_register + 0x28);
    *gpio_off_set |= (1 << 21);
}

void toogle_led (void)
{
    unsigned int *gpio_lev = (unsigned int*)((char*)gpio_register + 0x34);
    if ((*gpio_lev & (1 << 21)))
    {
        led_off();
        value = "OFF";
    }
    else
    {
        led_on();
        value = "ON";
    }
    SSD1306_SetCursor(1,56);
    SSD1306_String("   ");
    SSD1306_SetCursor(1,56);
    SSD1306_String(value);
}

void init_gpio (void)
{
    gpio_register = ioremap(GPIO_ADDRESS,PAGE_SIZE);
    if (!gpio_register) 
    {
        pr_err("Failed to map physical address to kernel address space\n");
        return;
    }
    unsigned int *gpio_sel1 = (unsigned int*)((char*)gpio_register + 0x08);
    *gpio_sel1 &= ~(7 << (3 * (21 % 10)));
    *gpio_sel1 |= (1 << (3 * (21 % 10)));
    *gpio_sel1 &= ~(7 << (3 * (20 % 10)));
    led_off();
}

static unsigned int irq_num;
dev_t dev = 0;
static struct class *tabi_class;
static struct cdev tabi_cdev;

static int __init etx_driver_init(void);
static void __exit etx_driver_exit(void);

/*************** Driver functions **********************/
static int tabi_open(struct inode *inode, struct file *file);
static int tabi_release(struct inode *inode, struct file *file);
static ssize_t tabi_read(struct file *filp, char __user *buf, size_t len, loff_t * off);
static ssize_t tabi_write(struct file *filp, const char __user *buf, size_t len, loff_t * off);
/******************************************************/

// File operation structure
static struct file_operations fops =
{
  .owner          = THIS_MODULE,
  .read           = tabi_read,
  .write          = tabi_write,
  .open           = tabi_open,
  .release        = tabi_release,
};

/*
** This function will be called when we open the Device file
*/
static int tabi_open(struct inode *inode, struct file *file)
{
    pr_info("Device File Opened...!!!\n");
    return 0;
}

/*
** This function will be called when we close the Device file
*/
static int tabi_release(struct inode *inode, struct file *file)
{
    pr_info("Device File Closed...!!!\n");
    return 0;
}

static ssize_t tabi_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    printk("read fops \n");
    return 0;
}

static ssize_t tabi_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
    
    printk("write fops\n");
    return len;
}

static irqreturn_t button_irq (int irq, void *dev_id, struct pt_regs *regs)
{
    toogle_led();
    return IRQ_HANDLED;
}

static int __init tabi_driver_init(void)
{
      int ret1 = -1;
      if (alloc_chrdev_region(&dev, 0, 1, "tabi_Dev") < 0)
      {
          pr_err("Cannot allocate major number\n");
          return -1;
      }
      pr_info("Major = %d Minor = %d \n", MAJOR(dev), MINOR(dev));
     
      cdev_init(&tabi_cdev, &fops);
     
      if (cdev_add(&tabi_cdev, dev, 1) < 0)
      {
          pr_err("Cannot add the device to the system\n");
          goto r_class;
      }
     
      if (IS_ERR(tabi_class = class_create(THIS_MODULE,"tabi_class")))
      {
          pr_err("Cannot create the struct class\n");
          goto r_class;
      }
     
    if (IS_ERR(device_create(tabi_class, NULL, dev, NULL, "tabi_device")))
    {
        pr_err("Cannot create the Device\n");
        goto r_device;
    }
    init_gpio();
    irq_num = gpio_to_irq(20);
    int ret;
    ret = request_irq(irq_num,(irq_handler_t) button_irq,IRQF_TRIGGER_RISING,"tabi_interrupt",NULL);
    if(ret)
    {
        printk("failed interrupt\n");
        goto r_reg;
    }
    tabi_i2c_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);
    if (tabi_i2c_adapter != NULL)
    {
        tabi_i2c_client_oled = i2c_new_client_device(tabi_i2c_adapter, &oled_i2c_board_info);
        if (tabi_i2c_client_oled != NULL)
        {
            i2c_add_driver(&tabi_oled_driver);
            ret1 = 0;
        }
        i2c_put_adapter(tabi_i2c_adapter);
    }
    
    pr_info("Device Driver Inserted...Done!!!\n");
    return ret1;
r_reg:
    iounmap(gpio_register);
    device_destroy(tabi_class, dev);
r_device:
    class_destroy(tabi_class);
r_class:
    unregister_chrdev_region(dev, 1);
    return -1;
}

/*
** Module exit function
*/
static void __exit tabi_driver_exit(void)
{
    i2c_unregister_device(tabi_i2c_client_oled);
    i2c_del_driver(&tabi_oled_driver);
    device_destroy(tabi_class, dev);
    class_destroy(tabi_class);
    cdev_del(&tabi_cdev);
    unregister_chrdev_region(dev, 1);
    iounmap(gpio_register);
    free_irq(irq_num,NULL);
    pr_info("Device Driver Removed...Done!!\n");
}

module_init(tabi_driver_init);
module_exit(tabi_driver_exit);

