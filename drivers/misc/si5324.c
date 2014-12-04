/*
 * Driver for Silicon Labs Si5324
 */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/log2.h>
#include <linux/slab.h>
//#include <linux/of_i2c.h>
#include <linux/i2c/si5324.h>

struct si5324_data {
	struct attribute_group attrs;
	struct mutex lock;
	u64 max_freq;
	u64 fout;		/* Factory default frequency */
	u64 fxtal;		/* Factory xtal frequency */
	unsigned int n1;
	unsigned int hs_div;
	u64 rfreq;
	u64 frequency;
};
static struct i2c_client *si5324_client;

    /*-------------------------------------------------
     * the following settings are borrowed from
     * si5324_clock_setup.psm
     *   param1: i2c client data structure
     *   param2: si5324 register
     *   param3: value to write in that register
     *-------------------------------------------------*/
struct si5324_setup {
	u8 addr;
	u8 cur_data;
	u8 req_data;
};
#define	SI5324_INIT_DATA_LEN	43

struct si5324_setup init_data[] =
{
	{ 0, 0x00, 0x54 },
	{ 1, 0x00, 0xE4 },
	{ 2, 0x00, 0x12 },
	{ 3, 0x00, 0x15 },
	{ 4, 0x00, 0x92 },
	{ 5, 0x00, 0xed },
	{ 6, 0x00, 0x2d },
	{ 7, 0x00, 0x2a },
	{ 8, 0x00, 0x00 },
	{ 9, 0x00, 0xc0 },
	{ 10, 0x00, 0x08 },
	{ 11, 0x00, 0x40 },
	{ 19, 0x00, 0x29 },
	{ 20, 0x00, 0x3e },
	{ 21, 0x00, 0xff },
	{ 22, 0x00, 0xdf },
	{ 23, 0x00, 0x1f },
	{ 24, 0x00, 0x3f },
	{ 25, 0x00, 0x60 },
	{ 31, 0x00, 0x00 },
	{ 32, 0x00, 0x00 },
	{ 33, 0x00, 0x05 },
	{ 34, 0x00, 0x00 },
	{ 35, 0x00, 0x00 },
	{ 36, 0x00, 0x05 },
	{ 40, 0x00, 0xc2 },
	{ 41, 0x00, 0x22 },
	{ 42, 0x00, 0xdf },
	{ 43, 0x00, 0x00 },
	{ 44, 0x00, 0x77 },
	{ 45, 0x00, 0x0b },
	{ 46, 0x00, 0x00 },
	{ 47, 0x00, 0x77 },
	{ 48, 0x00, 0x0b },
	{ 55, 0x00, 0x00 },
	{ 131, 0x00, 0x1f },
	{ 132, 0x00, 0x02 },
	{ 137, 0x00, 0x01 },
	{ 138, 0x00, 0x0f },
	{ 139, 0x00, 0xff },
	{ 142, 0x00, 0x00 },
	{ 143, 0x00, 0x00 },
	{ 136, 0x00, 0x40 },
};

static int si5324_set_defaults(struct i2c_client *client)
{
    //struct si5324_data *data = i2c_get_clientdata(client);
    int i;
    int update_needed=0;

    /*---------------------------*/
    /* Read current data         */
    /*---------------------------*/
    printk(KERN_ERR "si5324: reading current contents\n");
    for(i=0; i < SI5324_INIT_DATA_LEN; i++)
    {
	    init_data[i].cur_data = i2c_smbus_read_byte_data(client, init_data[i].addr );
    }

    /*---------------------------*/
    /* Verify current data       */
    /*---------------------------*/
    printk(KERN_ERR "si5324: verifying current settings for 125.00 MHz\n");
    for(i=0; i < SI5324_INIT_DATA_LEN; i++)
    {
	    if( init_data[i].cur_data != init_data[i].req_data)
	    {
      //printk(KERN_ERR "reg=%0x, cur_data=%0x, req_data=%0x\n", init_data[i].addr, init_data[i].cur_data, init_data[i].req_data);
      //- Do no compare ICAL register
      if (init_data[i].addr == 136)
        continue;
      else
      {
		    update_needed=1;
		    break;
      }
	   }
    }

    /*---------------------------*/
    /* Update with new  data     */
    /*---------------------------*/
    if(update_needed == 1)
    {
        printk(KERN_ERR "si5324: verify failed, setting si5324 for 125.00 MHz\n");
        for(i=0; i < SI5324_INIT_DATA_LEN; i++)
        {
            i2c_smbus_write_byte_data(client, init_data[i].addr, init_data[i].req_data );
	}
    }
    else
    {
        printk(KERN_ERR "si5324: verify passed, not setting si5324 again \n");
    }

    return 0;
}

static int si5324_get_defaults(struct i2c_client *client)
{
	return 0;
}

int get_frequency_si5324(struct device *dev, unsigned long *freq)
{
	return 0;
}
EXPORT_SYMBOL(get_frequency_si5324);


int set_frequency_si5324(struct device *dev, unsigned long freq)
{
	return 0;
}
EXPORT_SYMBOL(set_frequency_si5324);


int reset_si5324(struct device *dev, int id)
{
	return 0;
}
EXPORT_SYMBOL(reset_si5324);

struct i2c_client *get_i2c_client_si5324(void)
{
	return si5324_client;
}
EXPORT_SYMBOL(get_i2c_client_si5324);

static const struct i2c_device_id si5324_id[] = {
	{ "si5324", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, si5324_id);

static ssize_t show_si5324_frequency_attr(struct device *dev,
			      struct device_attribute *devattr,
			      char *buf)
{
	return 0;
}

static ssize_t set_si5324_frequency_attr(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	return 0;
}

static ssize_t show_si5324_reset_attr(struct device *dev,
			  struct device_attribute *devattr,
			  char *buf)
{
	return 0;
}

static ssize_t set_si5324_reset_attr(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
	return 0;
}


static DEVICE_ATTR(frequency, S_IWUSR | S_IRUGO, show_si5324_frequency_attr, set_si5324_frequency_attr);
static DEVICE_ATTR(reset, S_IWUSR | S_IRUGO, show_si5324_reset_attr, set_si5324_reset_attr);

static struct attribute *si5324_attr[] = {
	&dev_attr_frequency.attr,
	&dev_attr_reset.attr,
	NULL
};

static int si5324_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct si5324_data *data;
	int err;

	printk(KERN_ERR "probing si5324 ...\n");

	data = kzalloc(sizeof(struct si5324_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	err = si5324_get_defaults(client);
	if (err < 0)
		goto exit_free;

	mutex_init(&data->lock);

	/* Register sysfs hooks */
	data->attrs.attrs = si5324_attr;
	err = sysfs_create_group(&client->dev.kobj, &data->attrs);
	if (err)
		goto exit_free;

	/* Display a message indicating that we've successfully registered */
	dev_info(&client->dev, "registered %s\n", id->name);

	si5324_client = client;

	err = si5324_set_defaults(client);
	if (err < 0)
		goto exit_free;

	printk(KERN_ERR "si5324 probe is successful.\n");
	return 0;

exit_free:
	kfree(data);
exit:
	printk(KERN_ERR "si5324 probe failed.\n");
	return err;
}

static int si5324_remove(struct i2c_client *client)
{
	struct si5324_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &data->attrs);
	kfree(data);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id i2c_si5324_of_match[] = {
	{ .compatible = "si5324" },
	{ },
};
MODULE_DEVICE_TABLE(of, i2c_si5324_of_match);
#endif

static struct i2c_driver si5324_driver = {
	.driver = {
		.name	= "si5324",
		.of_match_table = of_match_ptr(i2c_si5324_of_match),
	},
	.probe		= si5324_probe,
	.remove		= si5324_remove,
	.id_table	= si5324_id,
};

static int __init si5324_init(void)
{
	return i2c_add_driver(&si5324_driver);
}

static void __exit si5324_exit(void)
{
	i2c_del_driver(&si5324_driver);
}

MODULE_AUTHOR("Guenter Roeck <guenter.roeck@ericsson.com>");
MODULE_DESCRIPTION("Si5324 driver");
MODULE_LICENSE("GPL");

module_init(si5324_init);
module_exit(si5324_exit);
