/*
 * ten file: vdy_driver.c
 * mod     : lesondydg@gmail.com
 * ngay tao: 3/6/2019
 * nguon   : thuc hien theo kien thuc tren vimentor cua tac gia "dat.a3cbq91@gmail.com"
 * mo ta   : char driver cho thiet bi gia lap vdy_device.
 *           vdy_device la mot thiet bi nam tren RAM.
 */

#include <linux/module.h> /* thu vien nay dinh nghia cac macro nhu module_init va module_exit */
#include <linux/fs.h> /*thu vien nay dung cho device number*/
#include <linux/device.h> /* thu vien nay dung cho cac ham tao devices file */
#include <linux/slab.h> /* thu vien nay chua cac ham kmalloc va kfree */
#include <linux/cdev.h>	/* thu vien nay chua cac ham lam viec voiws cdev*/
#include <linux/uaccess.h> /* thu vien nay chua cac ham trao doi du lieu giua user vaf kernel */
#include "vdy_driver02.h" /* thu vien chua cac mo ta cac thanh ghi cua vdy device */

/*#define DRIVER_AUTHOR "le dy"
#define DRIVER_DESC   "A sample character device driver"
#define DRIVER_VERSION "0.1"*/

/***************khai bao cho device thu cong************************/
/*	unsigned soluong = 1;                                       *
	dev_t devnum = MKDEV(124, 0);                               *
	const char *ten = "devicenameDynasty";                      */
/****************************************************************/	

typedef struct vdy_device{
	
	unsigned char * control_regs ;
	unsigned char * status_regs ; 
	unsigned char * data_regs ;

}vdy_device_t;


 struct _vdy_drv{

/***************khai bao cho device tu dong*********************************/
	                                                                   
	dev_t dev_num;
	unsigned int firstminor;

/***************************************************************************/

/*********************khai bao cho create class****************************/

	struct class *struclass;
	struct device *dev;
	vdy_device_t *vdy_hw;
	struct cdev *vcdev;
	unsigned open_cnt;

}vdy_dri;

/****************************** device specific - START *****************************/
/* ham khoi tao thiet bi */

int vdy_hw_init(vdy_device_t *hw)
{
	char *buf;
	buf = kzalloc(NUM_DEV_REGS * REG_SIZE, GFP_KERNEL);
	if(!buf){

		return -ENOMEM;
	}

	hw -> control_regs = buf;
	hw -> status_regs = hw -> control_regs + NUM_CTRL_REGS;
	hw -> data_regs = hw -> status_regs + NUM_STS_REGS; 


	//khoi tao cac gia tri ban dau cho cac thanh ghi 

	hw -> control_regs[CONTROL_ACCESS_REG] = 0x03;
	hw -> status_regs[DEVICE_STATUS_REG] = 0x03;

	return 0;

}

/* ham giai phong thiet bij*/

void vdy_hw_exit(vdy_device_t *hw)
{	
	kfree(hw -> control_regs);

}

/* ham doc tu cac thanh ghi du lieu cua thiet bi */


static ssize_t vdy_hw_read_data(vdy_device_t *hw, int start_reg, int num_regs, char* kbuf)
{
	int read_bytes = num_regs;

	// kiem tra xem co quyen doc du lieu khong 
	if((hw -> control_regs[CONTROL_ACCESS_REG] & CTRL_READ_DATA_BIT) == DISABLE)

		return -1;
	
	//keim tra xem dia chi cua kernel buffer co hop le khong 
	if(kbuf == NULL)

		return -1;
	

	//kiem tra xem vi tri cua cac thanh ghi can doc co hop ly khong
	if(start_reg > NUM_DATA_REGS)

		return -1;
	

	//dieu chinh lai luong thanh ghi du lieu cn doc co hop ly khong (neu can thiet)
	if(num_regs > (NUM_DATA_REGS - start_reg))

		read_bytes = NUM_DATA_REGS - start_reg;
	

	//ghi du lieu vao kernel buffer tu cac thanh ghi du lieu 
	memcpy(kbuf, hw -> data_regs + start_reg, read_bytes);

	//cap nhat so lan doc tu cac thanh ghi du lieu
	hw -> status_regs[READ_COUNT_L_REG] +=1;

	if(hw -> status_regs[READ_COUNT_L_REG] == 0)

		hw -> status_regs[READ_COUNT_H_REG] +=1;

	

	// tra ve so byte da doc duoc tu cac thanh ghi du lieu

	return read_bytes;
}

/********************************************************************************************/

/* ham ghi vao cac thanh ghi du lieu cua thiet bi */


static ssize_t vdy_hw_write_data(vdy_device_t *hw, int start_reg, int num_regs, char *kbuf)
{
	int write_bytes = num_regs;

	// kiem tra xem co quyen ghi du lieu khong 
	if((hw ->control_regs[CONTROL_ACCESS_REG] & CTRL_WRITE_DATA_BIT) == DISABLE)

		return -1;

	

	//keim tra xem dia chi cua kernel buffer co hop le khong 
	if(kbuf == NULL)

		return -1;
	

	//kiem tra xem vi tri cua cac thanh ghi can doc co hop ly khong
	if(start_reg > NUM_DATA_REGS)

		return -1;
	

	//dieu chinh lai luong thanh ghi du lieu cn doc co hop ly khong (neu can thiet)
	if(num_regs > (NUM_DATA_REGS - start_reg)){

		write_bytes = NUM_DATA_REGS - start_reg;

		hw -> status_regs[DEVICE_STATUS_REG] |= STS_DATAREGS_OVERFLOW_BIT;
	}

	//ghi du lieu tu kernel buffer vao cac thanh ghi du lieu 
	memcpy(hw -> data_regs + start_reg, kbuf, write_bytes);

	//cap nhat so lan ghi vao cac thanh ghi du lieu
	hw -> status_regs[WRITE_COUNT_L_REG] +=1;

	if(hw -> status_regs[WRITE_COUNT_L_REG] == 0)

		hw -> status_regs[WRITE_COUNT_H_REG] +=1;

	

	// tra ve so byte da doc duoc tu cac thanh ghi du lieu

	return write_bytes;

}




/* ham giai phong thiet bi */

/* ham doc tu cac thanh ghi du lieu cua thiet bi */

/* ham ghi vao cac thanh ghi du lieu cua thiet bi */

/* ham doc tu cac thanh ghi trang thai cua thiet bi */

/* ham ghi vao cac thanh ghi dieu khien cua thiet bi */

/* ham xu ly tin hieu ngat gui tu thiet bi */

/******************************* device specific - END *****************************/


/******************************** OS specific - START *******************************/
/* cac ham entry points */

static int vdy_driver_open(struct inode *inodep, struct file *filep)
{	vdy_dri.open_cnt++;
	printk("device file da mo dc %d lan\n", vdy_dri.open_cnt);
	return 0;
}
static int vdy_driver_close(struct inode *inodep, struct file *filep)
{
	printk("device file closed\n");
	return 0;
}

/********************************************************************************************/

static ssize_t vdy_driver_read(struct file *filp, char __user *user_buf, size_t len, loff_t *off)
{	
	char *kernel_buf = NULL;

	int num_bytes = 0;

	printk("su kien xu ly read bat dau tu %lld, %zu bytes\n", *off, len);

	kernel_buf = kzalloc(len, GFP_KERNEL);

	if(kernel_buf == NULL)

		return 0;
	

	num_bytes = vdy_hw_read_data(vdy_dri.vdy_hw, *off, len, kernel_buf);

	printk("read %d bytes tu HW\n", num_bytes);

	if(num_bytes < 0)

		return -EFAULT;
	

	if(copy_to_user(user_buf, kernel_buf, num_bytes))

		return -EFAULT;
	

	*off += num_bytes;

	return num_bytes;

}

/*******************************************************************************************************************/

static ssize_t vdy_driver_write(struct file *filp, const char __user *user_buf, size_t len, loff_t *off)
{
	char *kernel_buf = NULL;

	int num_bytes = 0;

	printk("su kien xu ly write bat dau tu %lld, %zu bytes\n", *off, len);

	kernel_buf = kzalloc(len, GFP_KERNEL);

	if(copy_from_user(kernel_buf, user_buf, len)) 

		return -EFAULT;


	num_bytes = vdy_hw_write_data(vdy_dri.vdy_hw, *off, len, kernel_buf);

	printk("write %d bytes den HW\n", num_bytes);


	if(num_bytes < 0)

		return -EFAULT;
	


	*off += num_bytes;

	return num_bytes;

}






static struct file_operations fops = {

	.owner    = THIS_MODULE,
	.open     = vdy_driver_open,
	.release  = vdy_driver_close,
	.read     = vdy_driver_read,
	.write    = vdy_driver_write,

};




/* ham khoi tao driver */
static int __init vdy_driver_init(void)
{
	/* cap phat device number */

	/* tao device file */

	/* cap phat bo nho cho cac cau truc du lieu cua driver va khoi tao */

	/* khoi tao thiet bi vat ly */

	/* dang ky cac entry point voi kernel */

	/* dang ky ham xu ly ngat */

/*****************************************************************************************/
	/*********************cap phat device number Tinh**************************/

/*	int tralai = register_chrdev_region(devnum, soluong, ten);

	if( tralai == 0)
	{
		printk("Dang ky device number thanh cong from dynasty");

	}

	else
		printk("Dang ky device khong thanh cong");*/

	//int register_chrdev_region(dev_t first, unsigned count, const char *name);
/*
 * Chức năng: đăng ký một dải gồm [cnt] device number bắt đầu từ [first] cho
 *            character device có tên là [name]
 * Tham số đầu vào:
 *    first [I]: device number đầu tiên muốn đăng ký <major, first_minor>. 
 *               Ta có thể dùng macro MKDEV để ghép số major và minor mà
 *               ta muốn đăng ký rồi truyền kết quả vào cho hàm này.
 *    cnt   [I]: số lượng device number mà ta muốn đăng ký. Driver sẽ đăng
 *               ký với kernel các device number, từ <major, first_minor>
 *               cho đến <major, first_minor + cnt – 1>
 *    *name [I]: tên của character device. Tên này sẽ xuất hiện trong thư mục 
 *               /proc/devices và khác với tên của device file trong thư mục /dev
 * Trả về:
 *    Nếu đăng ký thành công, hàm sẽ trả về 0.
 *    Nếu đăng ký thất bại, hàm sẽ trả về một giá trị < 0.
 */
/*******************************************************************************************/

// /**********************cap phat device number Dong**************************************/

/* int alloc_chrdev_region(dev_t *dev, unsigned int firstminor, unsigned int cnt, char *name) */

/*
 * Chức năng: yêu cầu kernel cấp phát một dải gồm [cnt] device number cho
 *            char device có tên là [name].
 * Tham số đầu vào:
 *    *dev        [O]: con trỏ này chứa giá trị trả về của hàm. Device number
 *                     đầu tiên của dải sẽ được trả về thông qua biến này.
 *     firstminor [I]: giá trị minor của số device number đầu tiên trong dải.
 *     cnt        [I]: là số lượng device number mà hàm này yêu cầu cấp phát.
 *     *name      [I]: tên của character device. Tên này sẽ xuất hiện trong
 *                     thư mục /proc/devices
 * Trả về:
 *     Nếu tìm được một device number, hàm này sẽ trả về 0. Device number đầu
 *     tiên trong dải sẽ được trả qua tham số *dev.
 *     Nếu không thể tìm được một device number nào, hàm sẽ trả về số nguyên âm.
 */

//int ret;


int trave = alloc_chrdev_region(&vdy_dri.dev_num, vdy_dri.firstminor, 1, "vdy_device");

if (trave == 0)
{

	printk("major number %d  minor number %d", MAJOR(vdy_dri.dev_num), MINOR(vdy_dri.dev_num));

}

else 
{
	printk("khong khoi tao device number thanh cong");

}


/****************************************************************************************/
	/**************************khoi tao devices file***************************/

/* Chức năng: Tạo ra một lớp các thiết bị có tên là [name] trong
 *            thư mục /sys/class. Lớp này chứa liên kết tới thông
 *            tin của các thiết bị cùng loại.
 * Tham số truyền vào:
 *   *owner [I]: con trỏ trỏ tới module sở hữu lớp thiết bị này
 *   *name  [I]: tên của lớp các thiết bị
 * trả về:
 *    Nếu thành công, thư mục có tên [name] được tạo ra trong
 *    /sys/class. Hàm trả về một con trỏ trỏ tới biến cấu trúc class.
 *    Nếu thất bại, trả về NULL
 */

/***** struct class* class_create(struct module *owner, const char *name) *************/

vdy_dri.struclass = class_create(THIS_MODULE, "class_name_from_dy");

if(vdy_dri.struclass == NULL){
	
	printk("tao lop thiet bi that bai \n");

	goto tao_lop_thiet_bi_fail;
}




/* Hàm hủy tương ứng với class_create */

/*void class_destroy(struct class *)*/

/*****************************************************************************************/


/*****************************************************************************************/
/*
 * Chức năng: Tao ra các thông tin của một thiết bị cụ thể.
 *            Khi có thông tin này, udev sẽ tạo ra một device file
 *            tương ứng trong /dev
 * Tham số truyền vào:
 *    *cls    [I]: con trỏ trỏ tới lớp các thiết bị. Con trỏ này là kết
 *                 quả của việc gọi hàm class_create
 *    *parent [I]: con trỏ trỏ tới thiết bị cha của thiết bị này. Nếu
 *                 thiết bị không có cha, ta truyền vào là NULL
 *    devt    [I]: device number của thiết bị.
 *    *drvdata[I]: dữ liệu bổ sung. Nếu không có, ta truyền vào là NULL.
 *    *name   [I]: tên của thiết bị. udev sẽ tạo ra device file với tên
 *                 này trong thư mục /dev
 */

/*struct device* device_create(struct class* cls, struct device *parent, dev_t devt, void *drvdata, const char *name)*/

/* Hàm hủy tương ứng với device_create */

/*void device_destroy(struct class * cls, dev_t devt)*/

vdy_dri.dev = device_create(vdy_dri.struclass, NULL, vdy_dri.dev_num, NULL, "vdy_device_file");
	
	if(IS_ERR(vdy_dri.dev)){

	printk("khoi tao thiet bi that bai \n");

	goto khoi_tao_thiet_bi_fail;

	}


/* cap phat bo nho cho cac cau truc du lieu cua driver va khoi tao*/

	vdy_dri.vdy_hw = kzalloc(sizeof(vdy_device_t), GFP_KERNEL);

	if(!vdy_dri.vdy_hw){

		printk("cap phat bo nho cho cau truc du lieu that bai\n");

		trave = -ENOMEM;
		goto cap_bo_nho_that_bai;
	}

/* khoi tao thiet bi vat ly */


	trave = vdy_hw_init(vdy_dri.vdy_hw);

	if(trave < 0){

		printk("khoi tao thiet bi ao that bai\n");

		goto khoi_tao_hw_that_bai;


	}

	/* dang ky cac entry point voi kernel */

	vdy_dri.vcdev = cdev_alloc();

	if(vdy_dri.vcdev == NULL){

		printk("cap phat bo nho cho cau truc cdev that bai\n");

		goto cap_bo_nho_cdev_faild;
	}

	cdev_init(vdy_dri.vcdev, &fops);
	
	trave = cdev_add(vdy_dri.vcdev, vdy_dri.dev_num, 1);

	if(trave < 0){

		printk("add thiet bi moi vao he thong that bai\n");

		goto cap_bo_nho_cdev_faild;
	}







	printk("khoi tao char driver vdy thanh cong\n");

	return 0;

cap_bo_nho_cdev_faild:
				vdy_hw_exit(vdy_dri.vdy_hw);

khoi_tao_hw_that_bai:
				kfree(vdy_dri.vdy_hw);

cap_bo_nho_that_bai:
				device_destroy(vdy_dri.struclass, vdy_dri.dev_num);

tao_lop_thiet_bi_fail: 	
				
				unregister_chrdev_region(vdy_dri.dev_num, 1);

khoi_tao_thiet_bi_fail: 	
				class_destroy(vdy_dri.struclass);
	

	

	
}




/* ham ket thuc driver */
static void __exit vdy_driver_exit(void)
{
	/* huy dang ky xu ly ngat */

	/* huy dang ky entry point voi kernel */

	/* giai phong thiet bi vat ly */

	/* giai phong bo nho da cap phat cau truc du lieu cua driver */

	/* xoa bo device file */

	/* giai phong device number */

/***********************************************************************************/
  /************************giai phong device number*******************************/ 

/*	void unregister_chrdev_region(dev_t devnum, unsigned int soluong);*/



	cdev_del(vdy_dri.vcdev);
	/* giai phong thiet bi vat ly */

	vdy_hw_exit(vdy_dri.vdy_hw);

	/* giai phong bo nho da cap phat cau truc du lieu cua driver */

	kfree(vdy_dri.vdy_hw);


	//dev_t number_devt = vdy_dri.dev_num;



	device_destroy(vdy_dri.struclass, vdy_dri.dev_num);

	class_destroy(vdy_dri.struclass);
	
	unregister_chrdev_region(vdy_dri.dev_num, 1);

	printk("thoat vdy driver thanh cong\n");

}
/********************************* OS specific - END ********************************/

module_init(vdy_driver_init);
module_exit(vdy_driver_exit);

MODULE_LICENSE("GPL"); /* giay phep su dung cua module */
MODULE_AUTHOR("dynasty"); /* cho biet tac gia cua module */
MODULE_DESCRIPTION("testdevice"); /* mo ta chuc nang cua module */
MODULE_VERSION("0.1"); /* mo ta phien ban cuar module */
MODULE_SUPPORTED_DEVICE("testdevice"); /* kieu device ma module ho tro */