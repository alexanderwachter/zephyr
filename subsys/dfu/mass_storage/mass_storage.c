
#include <errno.h>
#include <sys/byteorder.h>
#include <storage/stream_flash.h>
#include <usb/class/usb_msc.h>
#include <usb/usb_device.h>
#include <usb/usb_common.h>
#include <usb_descriptor.h>
#include <devicetree.h>
#include <stdint.h>
#include <string.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(usb_dfu_msc, CONFIG_MASS_DFU_LOG_LEVEL);

/* max USB packet size */
#define MAX_PACKET	CONFIG_MASS_DFU_BULK_EP_MPS

#define MASS_STORAGE_IN_EP_ADDR		0x82
#define MASS_STORAGE_OUT_EP_ADDR	0x01

#if !DT_HAS_CHOSEN(zephyr_update_partition)
#error Update partition does not exist! \
       add "chosen { zephyr,update-partition = &slot1_partition; }" to your devicetree
#endif

#define PARTITION_ADDR DT_REG_ADDR(DT_CHOSEN(zephyr_update_partition))
#define PARTITION_SIZE DT_REG_SIZE(DT_CHOSEN(zephyr_update_partition))

#define BLOCK_SIZE	CONFIG_MASS_DFU_BLOCK_SIZE
#define BLOCK_COUNT     (PARTITION_SIZE / BLOCK_SIZE)

#define BOOT_SECTOR_BLOCK_SIZE 512

struct usb_mass_config {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_in_ep;
	struct usb_ep_descriptor if0_out_ep;
} __packed;

USBD_CLASS_DESCR_DEFINE(primary, 0) struct usb_mass_config mass_cfg = {
	.if0 = {
		.bLength = sizeof(struct usb_if_descriptor),
		.bDescriptorType = USB_INTERFACE_DESC,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = MASS_STORAGE_CLASS,
		.bInterfaceSubClass = SCSI_TRANSPARENT_SUBCLASS,
		.bInterfaceProtocol = BULK_ONLY_PROTOCOL,
		.iInterface = 0,
	},
	.if0_in_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_ENDPOINT_DESC,
		.bEndpointAddress = MASS_STORAGE_IN_EP_ADDR,
		.bmAttributes = USB_DC_EP_BULK,
		.wMaxPacketSize =
			sys_cpu_to_le16(CONFIG_MASS_DFU_BULK_EP_MPS),
		.bInterval = 0x00,
	},
	.if0_out_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_ENDPOINT_DESC,
		.bEndpointAddress = MASS_STORAGE_OUT_EP_ADDR,
		.bmAttributes = USB_DC_EP_BULK,
		.wMaxPacketSize =
			sys_cpu_to_le16(CONFIG_MASS_DFU_BULK_EP_MPS),
		.bInterval = 0x00,
	},
};

static void mass_storage_bulk_out(uint8_t ep,
				  enum usb_dc_ep_cb_status_code ep_status);
static void mass_storage_bulk_in(uint8_t ep,
				 enum usb_dc_ep_cb_status_code ep_status);

static struct usb_ep_cfg_data mass_ep_data[] = {
	{
		.ep_cb	= mass_storage_bulk_out,
		.ep_addr = MASS_STORAGE_OUT_EP_ADDR
	},
	{
		.ep_cb = mass_storage_bulk_in,
		.ep_addr = MASS_STORAGE_IN_EP_ADDR
	}
};

static const struct usb_ep_cfg_data *out_ep = &mass_ep_data[0];
static const struct usb_ep_cfg_data *in_ep = &mass_ep_data[1];

/* CSW Status */
enum Status {
	CSW_PASSED,
	CSW_FAILED,
	CSW_ERROR,
};

enum SM_State {
	SM_RESET,
	SM_WAIT_CBW,
	SM_DATA_IN,
	SM_DATA_OUT,
	SM_TRANSFER_CSW,
	SM_WAIT_CSW
};

enum SM_Event {
	SM_NOP,
	/* There is data from the host */
	SM_EV_OUT_EP,
	/* Data was transferred to the host */
	SM_EV_IN_EP,
	/* Reset the State-Machine */
	SM_EV_RESET
};

struct mass_dfu_data {
	struct stream_flash_ctx ctx;
	struct CSW csw;
	struct k_work work;
	uint32_t block_addr;
	uint32_t offset;
	uint32_t residual;
	char filename[CONFIG_MASS_DFU_MAX_FILENAME + 1];
	uint32_t file_size;
	uint8_t state;
};

struct read10_cmd {
	uint8_t opcode;
	uint8_t obs  :2;
	uint8_t rarc :1;
	uint8_t fua  :1;
	uint8_t dpo  :1;
	uint8_t rdprotect :3;
	uint32_t lba;
	uint8_t group_nr :5;
	uint8_t res      :3;
	uint16_t tansfer_len;
	uint8_t conrol;
} __packed;

struct write10_cmd {
	uint8_t opcode;
	uint8_t obs  :2;
	uint8_t res1 :1;
	uint8_t fua  :1;
	uint8_t dpo  :1;
	uint8_t wrprotect :3;
	uint32_t lba;
	uint8_t group_nr :5;
	uint8_t res2     :3;
	uint16_t tansfer_len;
	uint8_t conrol;
} __packed;


#define SCSI_FROMAT_CAP_TYPE_UNFORMATED    0x01
#define SCSI_FROMAT_CAP_TYPE_FORMATED      0x02
#define SCSI_FROMAT_CAP_TYPE_NOT_PRESENT   0x03

struct capacity_descriptor {
	uint32_t num_blocks;
	uint8_t  desc_type : 2;
	uint8_t  res       : 6;
	uint8_t  block_len_msb;
	uint16_t block_len;
} __packed;

struct format_capacity {
	uint8_t res1;
	uint8_t res2;
	uint8_t res3;
	uint8_t list_len;
	struct capacity_descriptor descr_0;
} __packed;

struct inquiry_cmd {
	uint8_t cmd;
	uint8_t evpd : 1;
	uint8_t res  : 7;
	uint8_t page_code;
	uint16_t alloc_len;
	uint8_t control;
} __packed;

struct inquiry_data {
	uint8_t dev_type  : 5;
	uint8_t qualifier : 3;
	uint8_t res       : 7;
	uint8_t rmb       : 1;
	uint8_t version;
	uint8_t resp_data_format : 4;
	uint8_t hisup     : 1;
	uint8_t normaca   : 1;
	uint8_t obs1      : 2;
	uint8_t additional_len;
	uint8_t protect   : 1;
	uint8_t res1      : 2;
	uint8_t _3pc      : 1;
	uint8_t tpgs      : 2;
	uint8_t acc       : 1;
	uint8_t sccs      : 1;
	uint8_t obs2      : 4;
	uint8_t multip    : 1;
	uint8_t vs1       : 1;
	uint8_t encserv   : 1;
	uint8_t obs3      : 1;
	uint8_t vs2       : 1;
	uint8_t cmdque    : 1;
	uint8_t obs4      : 6;
	uint8_t vendor_id[8];
	uint8_t product_id[16];
	uint8_t product_rev[4];
} __packed;

struct mode_sense6_hdr {
	uint8_t data_len;
	uint8_t medium_type;
	uint8_t res1   : 4;
	uint8_t dpofua : 1;
	uint8_t res2   : 2;
	uint8_t wp     : 1;
	uint8_t block_desc_len;
} __packed;

/* Short LBA mode Parameter Block Descriptor */
struct mode_sense_pbd {
	uint32_t num_blocks;
	uint8_t res;
	uint8_t block_len_msb;
	uint16_t block_len;
} __packed;

struct mode_sense_data {
	struct mode_sense6_hdr hdr;
} __packed;

static struct mass_dfu_data data = {
	.state = SM_RESET,
	.csw = {.Signature = CSW_Signature},
};

struct mass_dfu_config {
	uint8_t lun;
};

static const struct mass_dfu_config cfg = {
	.lun = 0,
};

struct fat12_BS {
	uint8_t  BS_jmpBoot[3];
	char     BS_OEMName[8];
	uint16_t BPB_BytsPerSec;
	uint8_t  BPB_SecPerClus;
	uint16_t BPB_RsvdSecCnt;
	uint8_t  BPB_NumFATs;
	uint16_t BPB_RootEntCnt;
	uint16_t BPB_TotSec16;
	uint8_t  BPB_Media;
	uint16_t BPB_FATSz16;
	uint16_t BPB_SecPerTrk;
	uint16_t BPB_NumHeads;
	uint32_t BPB_HiddSec;
	uint32_t BPB_TotSec32;
	/* Extended BPB structure for FAT12 and FAT16 volumes */
	uint8_t BS_DrvNum;
	uint8_t BS_Reserved1;
	uint8_t BS_BootSig;
	uint32_t BS_VolID;
	char BS_VolLab[11];
	char BS_FilSysType[8];
} __packed;

struct fat12_BS_padded {
	struct fat12_BS bs;
	uint8_t pad[CONFIG_MASS_DFU_BULK_EP_MPS - sizeof(struct fat12_BS)];
};

#define FAT_BYTES_PER_SECTOR CONFIG_MASS_DFU_BYTES_PER_SECTOR

/* First sector is reserved (Bootsector)*/
#define FAT_BS_RSVD_SECTORS      1
/*Lets use 8*/
#define FAT_BS_SER_PER_CLUSTER   4
/* Number of fat tables following this boot sector. Two is the default */
#define FAT_BS_NUM_FATS          2
/* Defines the max number of entries (files or folders) in the root dir.
 * BPB_RootEntCnt * 32 % BPB_BytsPerSec has to be 0.
 * BPB_BytsPerSec / 32 = 16
 */
#define FAT_BS_ROOT_ENT_CNT (FAT_BYTES_PER_SECTOR / 32)
#define FAT_NUM_OF_CLUSTERS (ceiling_fraction(ceiling_fraction(PARTITION_SIZE, FAT_BYTES_PER_SECTOR), FAT_BS_SER_PER_CLUSTER))
/* Fat 12 uses 12 bits for each entry. This is 3/2 bytes */
#define FAT_SIZE            (ceiling_fraction(ceiling_fraction(FAT_NUM_OF_CLUSTERS * 3, 2), FAT_BYTES_PER_SECTOR))
#define ROOT_DIR_SECTORS    ceiling_fraction(FAT_BS_ROOT_ENT_CNT * 32, FAT_BYTES_PER_SECTOR)
#define FAT_BS_TOT_SEC      (FAT_BS_RSVD_SECTORS + FAT_BS_NUM_FATS * FAT_SIZE + ROOT_DIR_SECTORS + PARTITION_SIZE / FAT_BYTES_PER_SECTOR)
#define FAT_START_LBA       FAT_BS_RSVD_SECTORS
#define FAT_END_LBA         (FAT_START_LBA + (FAT_SIZE * FAT_BS_NUM_FATS))
#define ROOT_DIR_START_LBA  FAT_END_LBA
#define ROOT_DIR_END_LBA    (ROOT_DIR_START_LBA + ROOT_DIR_SECTORS)
#define DATA_START_LBA      ROOT_DIR_END_LBA
#define DATA_END_LBA        (DATA_START_LBA + (PARTITION_SIZE / FAT_BYTES_PER_SECTOR))

static const struct fat12_BS_padded boot_sector = {
	.bs = {
		/* Jump instruction on x86 */
		.BS_jmpBoot = {0xeb, 0x90, 0x3c},
		/* This is some indication of what system formatted the volume.*/
		.BS_OEMName = {'Z', 'e', 'p', 'y', 'r', ' ', 'M', 'S'},
		/* BPB_BytsPerSec * BPB_SecPerClus defines the length of one FAT entry */
		.BPB_BytsPerSec = sys_cpu_to_le16(FAT_BYTES_PER_SECTOR),
		.BPB_SecPerClus = FAT_BS_SER_PER_CLUSTER,
		.BPB_RsvdSecCnt = sys_cpu_to_le16(FAT_BS_RSVD_SECTORS),
		.BPB_NumFATs = FAT_BS_NUM_FATS,
		.BPB_RootEntCnt = sys_cpu_to_le16(FAT_BS_ROOT_ENT_CNT),
		.BPB_TotSec16 = sys_cpu_to_le16(FAT_BS_TOT_SEC),
		/* According to Microsoft FAT Specification, 0xF0 is for removable media */
		.BPB_Media = 0xF0,
		.BPB_FATSz16 = sys_cpu_to_le16(FAT_SIZE),
		.BPB_SecPerTrk = 0,
		.BPB_NumHeads = 0,
		.BPB_HiddSec = 0,
		/* On fat12 we use BPB_TotSec16 */
		.BPB_TotSec32 = 0,
		/* Spec says set to 0x80 or 0x00*/
		.BS_DrvNum = 0x00,
		.BS_Reserved1 = 0x00,
		/* 0x29 means that BS_VolID and label is set*/
		.BS_BootSig = 0x29,
		/* 15.11.2020 15:36*/
		.BS_VolID = sys_cpu_to_le32(0x5faaa57a),
		.BS_VolLab = {'U', 'P', 'D', 'A', 'T', 'E', ' ', ' ', ' ', ' ', ' '},
		.BS_FilSysType = {'F', 'A', 'T', '1', '2', ' ', ' ', ' '}
		},
	.pad = {0}
};

static const uint8_t zero_pkt[CONFIG_MASS_DFU_BULK_EP_MPS] = {0};

struct fat12_dir {
	char DIR_Name[11];
	uint8_t DIR_Attr;
	uint8_t DIR_NTRes;
	/* Component of the file creation time. Valid range: 0 - 199 */
	uint8_t DIR_CrtTimeTenth;
	/* Creation time. Granularity is 2 seconds. */
	uint16_t DIR_CrtTime;
	uint16_t DIR_CrtDate;
	uint16_t DIR_LstAccDate;
	/* High word of first data cluster number for file/directory described by this entry */
	uint16_t DIR_FstClusHI;
	uint16_t DIR_WrtTime;
	uint16_t DIR_WrtDate_day : 5;
	uint16_t DIR_WrtDate_month : 4;
	uint16_t DIR_WrtDate_year : 7;
	/* Low word of first data cluster number for file/directory described by this entry.*/
	uint16_t DIR_FstClusLO;
	/* 32-bit quantity containing size in bytes of file/directory described by this entry. */
	uint32_t DIR_FileSize;
} __packed;

#define LAST_LONG_ENTRY 0x40

struct fat12_long_filename {
	uint8_t LDIR_Ord;
	uint16_t LDIR_Name1[5];
	uint8_t LDIR_Attr;
	uint8_t LDIR_Type;
	uint8_t LDIR_Chksum;
	uint16_t LDIR_Name2[6];
	uint16_t LDIR_FstClusLO;
	uint16_t LDIR_Name3[2];
} __packed;

struct fat12_dir_padded {
	struct fat12_dir dir;
	uint8_t pad[CONFIG_MASS_DFU_BULK_EP_MPS - sizeof(struct fat12_dir)];
};

#define FAT_ATTR_READ_ONLY 0x01
#define FAT_ATTR_HIDDEN    0x02
#define FAT_ATTR_SYSTEM    0x04
#define FAT_ATTR_VOLUME_ID 0x08
#define FAT_ATTR_DIRECTORY 0x10
#define FAT_ATTR_ARCHIVE   0x20
#define FAT_ATTR_LONG_NAME 0x0f

static const struct fat12_dir_padded root_dir = {
	.dir = {
		.DIR_Name = {'U', 'p', 'd', 'a', 't', 'e', ' ', ' ', ' ', ' ', ' '},
		.DIR_Attr = FAT_ATTR_VOLUME_ID,
		.DIR_NTRes = 0x00,
		.DIR_CrtTimeTenth = 0x00,
		.DIR_CrtTime = 0x00,
		.DIR_CrtDate = 0x00,
		.DIR_LstAccDate = 0x00,
		.DIR_FstClusHI = 0x0000,
		.DIR_WrtTime = 0x00,
		.DIR_WrtDate_day = CONFIG_MASS_DFU_DATE_DAY,
		.DIR_WrtDate_month = CONFIG_MASS_DFU_DATE_MONTH,
		.DIR_WrtDate_year = CONFIG_MASS_DFU_DATE_YEAR - 1980,
		.DIR_FstClusLO = 0x0000,
		.DIR_FileSize = 0
	},
	.pad = {0}
};

#define MSD_OUT_EP_IDX 0
#define MSD_IN_EP_IDX  1

static int sm_process(enum SM_Event event, uint8_t ep);

static void send_CSW(uint8_t status)
{
	int ret;
	data.csw.Status = status;
	data.csw.DataResidue = sys_cpu_to_le32(data.residual);
	LOG_DBG("Send CSW. Residual: %d", data.residual);

	ret = usb_write(in_ep->ep_addr, (uint8_t *)&data.csw, sizeof(struct CSW), NULL);
	if (ret != 0) {
		LOG_ERR("Write CSW failed [%d]", ret);
		return;
	}

	data.state = SM_WAIT_CSW;
}

static int write(const uint8_t *buf, uint16_t len)
{
	uint32_t data_len = MIN(len, data.residual);
	int ret;

	ret = usb_write(in_ep->ep_addr, buf, data_len, NULL);
	if (ret != 0) {
		LOG_ERR("USB write failed [%d]", ret);
		return -EIO;
	}

	data.residual -= data_len;
	if (data.residual == 0) {
		data.state = SM_TRANSFER_CSW;
	}

	return 0;
}

static int mass_storage_class_handle_req(struct usb_setup_packet *pSetup,
					 int32_t *len, uint8_t **data)
{
	if (pSetup->wIndex != mass_cfg.if0.bInterfaceNumber ||
	    pSetup->wValue != 0) {
		LOG_WRN("Invalid setup parameters");
		return -EINVAL;
	}

	switch (pSetup->bRequest) {
	case MSC_REQUEST_RESET:
		LOG_DBG("MSC_REQUEST_RESET");

		if (pSetup->wLength) {
			LOG_WRN("Invalid length");
			return -EINVAL;
		}

		sm_process(SM_EV_RESET, 0);
		break;

	case MSC_REQUEST_GET_MAX_LUN:
		LOG_DBG("MSC_REQUEST_GET_MAX_LUN");

		if (pSetup->wLength != 1) {
			LOG_WRN("Invalid length");
			return -EINVAL;
		}

		*data = (uint8_t *)&cfg.lun;
		*len = sizeof(cfg.lun);
		break;

	default:
		LOG_WRN("Unknown request 0x%02x, value 0x%02x",
			pSetup->bRequest, pSetup->wValue);
		return -EINVAL;
	}

	return 0;
}

static void do_tur(struct CBW *cbw)
{
	if (cbw->DataLength != 0U) {
		if ((cbw->Flags & CBW_DIRECTION_DATA_IN)) {
			LOG_WRN("Stall IN endpoint");
			usb_ep_set_stall(in_ep->ep_addr);
		} else {
			LOG_WRN("Stall OUT endpoint");
			usb_ep_set_stall(out_ep->ep_addr);
		}
	}

	send_CSW(CSW_PASSED);
}

static void do_request_sense(void)
{
	static const uint8_t request_sense[] = {
		0x70,
		0x00,
		0x05,   /* Sense Key: illegal request */
		0x00,
		0x00,
		0x00,
		0x00,
		0x0A,
		0x00,
		0x00,
		0x00,
		0x00,
		0x30,
		0x01,
		0x00,
		0x00,
		0x00,
		0x00,
	};

	write(request_sense, sizeof(request_sense));
	data.csw.Status = CSW_PASSED;
	data.state = SM_TRANSFER_CSW;
}

static void do_inquiry_request(void)
{
	static const struct inquiry_data inquiry = {
		/* 0: Direct access block device */
		.dev_type = 0,
		/* 0: A peripheral device having the specified peripheral device
		 * type is connected to this logical unit */
		.qualifier = 0,
		/* Is removable media */
		.rmb = 1,
		/* 0: The device does not claim conformance to any standard. */
		.version = 0,
		.resp_data_format = 2,
		.hisup = 0,
		.normaca = 0,
		.additional_len = sizeof(struct inquiry_data) - 4,
		.protect = 0,
		._3pc = 0,
		.tpgs = 0,
		.acc = 0,
		.sccs = 1,
		.multip = 0,
		.encserv = 0,
		.cmdque = 0,
		.vendor_id = {'Z', 'E', 'P', 'H', 'Y', 'R', ' ', ' ',},
		.product_id = {'F', 'W', ' ', 'U', 'P', 'D', 'A', 'T', 'E', ' ',
			       ' ', ' ', ' ', ' ', ' ', ' '},
		.product_rev = {'1', '.', '0', ' '}
	};

	write((const uint8_t *)&inquiry, sizeof(inquiry));
	data.csw.Status = CSW_PASSED;
	data.state = SM_TRANSFER_CSW;
}

static void do_mode_sense6(void)
{
	static const struct mode_sense_data sense = {
		.hdr = {
			.data_len = sizeof(struct mode_sense_data) - sizeof(1),
			.medium_type = 0,
			.wp = 0,
			/* if zero, the host shall use the read capacity cmd */
			.block_desc_len = 0
		}
	};

	write((const uint8_t *)&sense, sizeof(sense));
	data.csw.Status = CSW_PASSED;
	data.state = SM_TRANSFER_CSW;
}

static void do_read_format_capacity(void)
{
	static const struct format_capacity capacity = {
		.list_len = sizeof(struct capacity_descriptor),
		.descr_0 = {
			.num_blocks = sys_cpu_to_be32(FAT_BS_TOT_SEC),
			.desc_type = SCSI_FROMAT_CAP_TYPE_FORMATED,
			.block_len_msb = BLOCK_SIZE >> 16,
			.block_len = sys_cpu_to_be16(BLOCK_SIZE & 0xFFFF)
		}
	};

	write((const uint8_t *)&capacity, sizeof(capacity));
	data.csw.Status = CSW_PASSED;
	data.state = SM_TRANSFER_CSW;
}

static void do_read_capacity(void)
{
	static const uint32_t capacity[] = {
		sys_cpu_to_be32(FAT_BS_TOT_SEC - 1),
		sys_cpu_to_be32(BLOCK_SIZE)
	};

	write((const uint8_t *)capacity, sizeof(capacity));
	data.csw.Status = CSW_PASSED;
	data.state = SM_TRANSFER_CSW;
}

static int transfer_boot_sector(uint32_t offset, uint32_t len)
{
	const uint8_t *data;

	LOG_DBG("Transfer BS. Len: %d, offs: %d", len, offset);

	if (offset + len <= sizeof(boot_sector)) {
		data = (const uint8_t *)&boot_sector;
		data += offset;
	} else {
		data = zero_pkt;
	}

	return write(data, len);
}

static int transfer_fat(uint32_t len)
{
	LOG_DBG("Transfer FAT. Len: %d", len);
	/* FAT is empty because we have no files */
	return write(zero_pkt, len);
}

static int transfer_root_dir(uint32_t offset, uint32_t len)
{
	const uint8_t *data;

	LOG_DBG("Transfer ROOT DIR. Len: %d, offs: %d", len, offset);

	if (offset + len <= sizeof(root_dir)) {
		data = (const uint8_t *)&root_dir;
		data += offset;
	} else {
		data = zero_pkt;
	}

	return write(data, len);
}

static int address_read_dispatch(uint32_t lba)
{
	uint32_t tansfer_len = MIN(data.residual, MAX_PACKET);
	int ret = -1;

	/* LOG_DBG("Dispatch lba %d, residual: %d", lba, data.residual); */

	/* Host reads Boot Sector */
	if (lba < FAT_START_LBA) {
		ret = transfer_boot_sector(lba * BLOCK_SIZE + data.offset, tansfer_len);
	} else if (lba < FAT_END_LBA) {
		ret = transfer_fat(tansfer_len);
	} else if (lba < ROOT_DIR_END_LBA) {
		ret = transfer_root_dir((lba - ROOT_DIR_START_LBA) * BLOCK_SIZE + data.offset, tansfer_len);
	} else {
		/*Data is all zero*/
		ret = write(zero_pkt, tansfer_len);
	}

	if (ret == 0) {
		data.offset += tansfer_len;
		if (data.offset >= BLOCK_SIZE) {
			data.offset -= BLOCK_SIZE;
			++data.block_addr;
		}
		if (data.residual) {
			data.state = SM_DATA_IN;
		} else {
			data.csw.Status = CSW_PASSED;
			data.state = SM_TRANSFER_CSW;
		}
	}

	return ret;
}

static void do_read(struct CBW *cbw)
{
	struct read10_cmd *cmd = (struct read10_cmd *)cbw->CB;
	uint32_t addr = sys_be32_to_cpu(cmd->lba);
	uint16_t num_blocks = sys_be16_to_cpu(cmd->tansfer_len);
	uint32_t len = num_blocks * BLOCK_SIZE;
	int ret;

	if (!(cbw->Flags & CBW_DIRECTION_DATA_IN)) {
		usb_ep_set_stall(out_ep->ep_addr);
		LOG_WRN("Stall OUT endpoint");
		send_CSW(CSW_ERROR);
		return;
	}

	if (cbw->DataLength != len) {
		LOG_ERR("CBW len (%d) is different from SCSI len (%d)",
			cbw->DataLength, len);
		data.csw.Status = CSW_FAILED;
		data.state = SM_TRANSFER_CSW;
		return;
	}

	data.state = SM_DATA_IN;
	data.block_addr = addr;
	data.offset = 0;
	data.residual = len;

	ret = address_read_dispatch(addr);
	if (ret != 0) {
		usb_ep_set_stall(out_ep->ep_addr);
		LOG_WRN("Stall OUT endpoint");
		send_CSW(CSW_FAILED);
		return;
	}
}

enum root_dir_state {DIR_INIT, VOL_ENTRY, FILE_DIR, LONG_FILE_NAME};

static int name_field_len(const uint16_t *src, size_t length)
{
	const uint16_t *src_ptr = src;
	int len;

	for (len = 0;
	     len < length && *src_ptr != 0x0000 && *src_ptr != 0xffff;
	     ++src_ptr, ++len) {}

	return len;
}

static bool cpy_name(char *dest, const uint16_t *src, size_t len)
{
	const uint16_t *src_ptr = src;
	const uint16_t *src_end = src + len;
	char *dest_ptr = dest;

	for (; src_ptr < src_end; ++src_ptr, ++dest_ptr) {
		/* Only use the lower bits (src is in le)*/
		*dest_ptr = sys_le16_to_cpu(*src_ptr) & 0xFF;
	}

	return false;
}

__weak void got_file(const char *filename, size_t size)
{
	LOG_INF("Got file %s, size: %zu", log_strdup(filename), size);
}

static void parse_rootdir_writes(const struct fat12_dir *dir, uint32_t dir_nr)
{
	static enum root_dir_state state = DIR_INIT;
	static uint8_t rem_ln_enties;
	static char *filename_ptr;
	static uint32_t short_entry_of_long;
	const struct fat12_long_filename *lfn =
		(const struct fat12_long_filename *)dir;
	int name_len;

	if (dir_nr == 0) {
		state = VOL_ENTRY;
	}

	if (dir->DIR_Name[0] == 0 || dir->DIR_Name[0] == 0xe5) {
		/* Empty */
		return;
	}

	switch (state) {
	case VOL_ENTRY:
		LOG_DBG("VOL Entry");
		state = FILE_DIR;
		break;

	case FILE_DIR:
		if ((dir->DIR_Attr & FAT_ATTR_LONG_NAME) != FAT_ATTR_LONG_NAME) {
			LOG_DBG("Short filename");
			if (dir_nr == short_entry_of_long) {
				/* Short entry from a long name*/
				short_entry_of_long = 0;
			} else {
				filename_ptr = memcpy(data.filename,
						dir->DIR_Name,
						sizeof(dir->DIR_Name));
				/* Terminate after full name */
				data.filename[sizeof(dir->DIR_Name)] = '\0';
				/* Terminate at first empty char */
				filename_ptr = memchr(data.filename, ' ',
						      sizeof(dir->DIR_Name));
				if (filename_ptr) {
					*filename_ptr = '\0';
				}

				filename_ptr = data.filename;
			}

			if (!(dir->DIR_Attr & FAT_ATTR_DIRECTORY)) {
				got_file(filename_ptr,
					 sys_le32_to_cpu(dir->DIR_FileSize));
			}

			break;
		}

		LOG_DBG("Long File Name");

		if (!(lfn->LDIR_Ord & LAST_LONG_ENTRY)) {
			LOG_ERR("Expected last LFN entry");
			return;
		}

		filename_ptr = &data.filename[sizeof(data.filename) - 1];
		*filename_ptr = '\0';
		state = LONG_FILE_NAME;
		rem_ln_enties = lfn->LDIR_Ord & ~LAST_LONG_ENTRY;

		/* fallthrough */
	case LONG_FILE_NAME:
		if ((lfn->LDIR_Ord & ~LAST_LONG_ENTRY) != rem_ln_enties) {
			LOG_ERR("Wrong LFN sequence");
			state = FILE_DIR;
			return;
		}

		name_len = name_field_len(lfn->LDIR_Name3,
					  ARRAY_SIZE(lfn->LDIR_Name3));
		if (name_len && (filename_ptr - name_len >= data.filename)) {
			filename_ptr -= name_len;
			cpy_name(filename_ptr, lfn->LDIR_Name3, name_len);
		} else {
			LOG_WRN("Filename too long");
		}

		name_len = name_field_len(lfn->LDIR_Name2,
					  ARRAY_SIZE(lfn->LDIR_Name2));
		if (name_len && (filename_ptr - name_len >= data.filename)) {
			filename_ptr -= name_len;
			cpy_name(filename_ptr, lfn->LDIR_Name2, name_len);
		} else {
			LOG_WRN("Filename too long");
		}

		name_len = name_field_len(lfn->LDIR_Name1,
					  ARRAY_SIZE(lfn->LDIR_Name1));
		if (name_len && (filename_ptr - name_len >= data.filename)) {
			filename_ptr -= name_len;
			cpy_name(filename_ptr, lfn->LDIR_Name1, name_len);
		} else {
			LOG_WRN("Filename too long");
		}

		rem_ln_enties--;
		if (rem_ln_enties == 0) {
			state = FILE_DIR;
			short_entry_of_long = dir_nr + 1;
		}

		break;

	default:
		break;
	}
}

static void rootdir_writes(uint32_t lba, uint32_t offset,
				const uint8_t *buf, uint32_t len)
{
	const struct fat12_dir *dir = (const struct fat12_dir *)buf;
	uint32_t dir_entry_number =
		(lba * FAT_BYTES_PER_SECTOR + offset) / sizeof(struct fat12_dir);

	for (int dir_cnt = len / sizeof(struct fat12_dir);
	     dir_cnt; --dir_cnt, ++dir, ++dir_entry_number) {
		parse_rootdir_writes(dir, dir_entry_number);
	}
}

static int address_write_dispatch(uint32_t lba, const uint8_t *buf, uint32_t len)
{
	int ret = 0;

	LOG_DBG("write lba %d, offs: %d, residual: %d", lba, data.offset, data.residual);
	/* LOG_HEXDUMP_DBG(buf, len, ""); */

	if (lba < FAT_START_LBA) {
		/* Ignore writes to Boot Sector */
		LOG_DBG("Host writes bootsector");
	} else if (lba < FAT_END_LBA) {
		/* Ignore writes to FAT */
		LOG_DBG("Host writes FAT");
	} else if (lba < ROOT_DIR_END_LBA) {
		rootdir_writes((lba - ROOT_DIR_START_LBA), data.offset, buf, len);
	} else if (lba < DATA_END_LBA) {
		/* Write data to the flash */
	} else {
		LOG_DBG("Write lba %d, offs %d to NULL", lba, data.offset);
	}

	data.residual -= len;
	data.offset += len;
	if (data.offset >= BLOCK_SIZE) {
		data.offset -= BLOCK_SIZE;
		++data.block_addr;
	}

	if (data.residual == 0) {
		send_CSW(CSW_PASSED);
	}

	return ret;
}

static void do_write(struct CBW *cbw)
{
	struct write10_cmd *cmd = (struct write10_cmd *)cbw->CB;
	uint32_t addr = sys_be32_to_cpu(cmd->lba);
	uint16_t num_blocks = sys_be16_to_cpu(cmd->tansfer_len);
	uint32_t len = num_blocks * BLOCK_SIZE;

	if (cbw->Flags & CBW_DIRECTION_DATA_IN) {
		usb_ep_set_stall(in_ep->ep_addr);
		LOG_WRN("Stall IN endpoint");
		send_CSW(CSW_ERROR);
		return;
	}

	if (cbw->DataLength != len) {
		LOG_ERR("CBW len (%d) is different from SCSI len (%d)",
			cbw->DataLength, len);
		data.csw.Status = CSW_FAILED;
		data.state = SM_TRANSFER_CSW;
		return;
	}

	data.state = SM_DATA_OUT;
	data.block_addr = addr;
	data.offset = 0;
	data.residual = len;
}

static void do_verify(struct CBW *cbw)
{
	data.state = SM_TRANSFER_CSW;
	data.csw.Status = CSW_PASSED;

	if (!(cbw->CB[1] & 0x02)) {
		return;
	}
}

static void do_media_rem(struct CBW *cbw)
{
	send_CSW(CSW_PASSED);
}

static void process_scsi(struct CBW *cbw)
{
	uint8_t cmd = cbw->CB[0];

	switch (cmd) {
	case TEST_UNIT_READY:
		LOG_DBG("SCSI TUR");
		do_tur(cbw);
		break;
	case REQUEST_SENSE:
		LOG_DBG("SCSI REQ_SENSE");
		do_request_sense();
		break;
	case INQUIRY:
		LOG_DBG("SCSI INQ");
		do_inquiry_request();
		break;
	case MODE_SENSE6:
		LOG_DBG("SCSI MODE_SENSE6");
		do_mode_sense6();
		break;
	case READ_CAPACITY:
		LOG_DBG("SCSI READ_CAPACITY");
		do_read_capacity();
		break;
	case READ_FORMAT_CAPACITIES:
		LOG_DBG("SCSI READ_FORMAT_CAPACITIES");
		do_read_format_capacity();
		break;
	case READ10:
	case READ12:
		LOG_DBG("SCSI READ");
		do_read(cbw);
		break;
	case WRITE10:
	case WRITE12:
		LOG_DBG("SCSI WRITE");
		do_write(cbw);
		break;
	case VERIFY10:
		do_verify(cbw);
		LOG_DBG("SCSI VERIFY10");
		break;
	case MEDIA_REMOVAL:
		LOG_DBG("SCSI MEDIA_REMOVAL");
		do_media_rem(cbw);
		break;
	default:
		LOG_INF("Unhandled SCSI command 0x%x", cmd);
		if ((cbw->Flags & CBW_DIRECTION_DATA_IN)) {
			usb_ep_set_stall(in_ep->ep_addr);
		} else {
			usb_ep_set_stall(out_ep->ep_addr);
		}

		send_CSW(CSW_FAILED);
	}
}

static int decode_cbw(struct CBW *cbw)
{
	if (cbw->Signature != CBW_Signature) {
		LOG_ERR("CBW Signature Mismatch");
		return -EINVAL;
	}

	data.csw.Tag = cbw->Tag;
	data.residual = sys_le32_to_cpu(cbw->DataLength);

	if (cbw->LUN != cfg.lun) {
		LOG_ERR("Wrong LUN %d", cbw->LUN);
		return -EINVAL;
	}

	if ((cbw->CBLength < 1) || (cbw->CBLength > sizeof(cbw->CB))) {
		LOG_WRN("cbw.CBLength %d", cbw->CBLength);;
		return -EINVAL;
	}

	LOG_DBG("DBW decoded. length: %d", data.residual);

	process_scsi(cbw);

	return 0;
}

static int sm_process(enum SM_Event event, uint8_t ep)
{
	int ret;
	uint32_t read_bytes;
	uint8_t buf[CONFIG_MASS_DFU_BULK_EP_MPS];

	if (event == SM_EV_OUT_EP) {
		ret = usb_read(ep, buf, sizeof(buf), &read_bytes);
		if (ret != 0) {
			LOG_ERR("Could not read EP out data [%d]", ret);
			usb_ep_set_stall(ep);
			return -EIO;
		}
	}

	if (event == SM_EV_RESET) {
		data.state = SM_WAIT_CBW;
		return 0;
	}

	switch (data.state) {
	case SM_WAIT_CBW:
		if (event != SM_EV_OUT_EP) {
			LOG_ERR("Expected CBW but didn't get data");
			ret = -EINVAL;
			break;
		}

		if (read_bytes != sizeof(struct CBW)) {
			LOG_ERR("CBW size wrong. %d", read_bytes);
			ret = -EINVAL;
			break;
		}

		decode_cbw((struct CBW *) buf);
		break;
	case SM_DATA_IN:
		address_read_dispatch(data.block_addr);
		break;
	case SM_DATA_OUT:
		address_write_dispatch(data.block_addr, buf, read_bytes);
		break;
	case SM_TRANSFER_CSW:
		if (event != SM_EV_IN_EP) {
			break;
		}

		send_CSW(data.csw.Status);
		break;
	case SM_WAIT_CSW:
		if (event == SM_EV_IN_EP) {
			data.state = SM_WAIT_CBW;
		}

	default:
		break;
	}

	return 0;
}

static void mass_storage_bulk_out(uint8_t ep,
		enum usb_dc_ep_cb_status_code ep_status)
{
	if (ep_status == USB_DC_EP_DATA_OUT) {
		sm_process(SM_EV_OUT_EP, ep);
	}

}


static void mass_storage_bulk_in(uint8_t ep,
				 enum usb_dc_ep_cb_status_code ep_status)
{
	if (ep_status == USB_DC_EP_DATA_IN) {
		sm_process(SM_EV_IN_EP, ep);
	}
}

static void mass_storage_status_cb(struct usb_cfg_data *cfg,
				   enum usb_dc_status_code status,
				   const uint8_t *param)
{
	ARG_UNUSED(param);
	ARG_UNUSED(cfg);

	/* Check the USB status and do needed action if required */
	switch (status) {
	case USB_DC_ERROR:
		LOG_DBG("USB device error");
		break;
	case USB_DC_RESET:
		LOG_DBG("USB device reset detected");
		sm_process(SM_EV_RESET, 0);
		break;
	case USB_DC_CONNECTED:
		LOG_DBG("USB device connected");
		break;
	case USB_DC_CONFIGURED:
		LOG_DBG("USB device configured");
		break;
	case USB_DC_DISCONNECTED:
		LOG_DBG("USB device disconnected");
		break;
	case USB_DC_SUSPEND:
		LOG_DBG("USB device supended");
		break;
	case USB_DC_RESUME:
		LOG_DBG("USB device resumed");
		break;
	case USB_DC_INTERFACE:
		LOG_DBG("USB interface selected");
		break;
	case USB_DC_SOF:
		break;
	case USB_DC_UNKNOWN:
	default:
		LOG_DBG("USB unknown state");
		break;
	}
}

static void mass_interface_config(struct usb_desc_header *head,
				  uint8_t bInterfaceNumber)
{
	ARG_UNUSED(head);

	mass_cfg.if0.bInterfaceNumber = bInterfaceNumber;
}

/* Configuration of the Mass Storage Device send to the USB Driver */
USBD_CFG_DATA_DEFINE(primary, msd) struct usb_cfg_data mass_storage_config = {
	.usb_device_description = NULL,
	.interface_config = mass_interface_config,
	.interface_descriptor = &mass_cfg.if0,
	.cb_usb_status = mass_storage_status_cb,
	.interface = {
		.class_handler = mass_storage_class_handle_req,
		.custom_handler = NULL,
	},
	.num_endpoints = ARRAY_SIZE(mass_ep_data),
	.endpoint = mass_ep_data
};
