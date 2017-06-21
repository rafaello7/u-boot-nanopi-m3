/*
 * (C) Copyright 2016 Nexell
 * Hyunseok, Jung <hsjung@nexell.co.kr>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <console.h>
#include <asm/io.h>

#include <asm/arch/reset.h>
#include <asm/arch/nexell.h>

#define VENDORID			0x04e8	/* Samsung Vendor ID */
#define PRODUCTID			0x1234	/* Nexell Product ID */
#define NXP4330_USBD_VID		0x2375
#define NXP4330_USBD_PID		0x4330

#define BASEADDR_BOOTSTATUS \
	(BASEADDR_SRAM+(INTERNAL_SRAM_SIZE/2))

#define	HIGH_USB_VER			0x0200		/* 2.0 */
#define	HIGH_MAX_PKT_SIZE_EP0		64
#define	HIGH_MAX_PKT_SIZE_EP1		512		/* bulk */
#define	HIGH_MAX_PKT_SIZE_EP2		512		/* bulk */

#define	FULL_USB_VER			0x0110		/* 1.1 */
#define	FULL_MAX_PKT_SIZE_EP0		8		/* Do not modify */
#define	FULL_MAX_PKT_SIZE_EP1		64		/* bulk */
#define	FULL_MAX_PKT_SIZE_EP2		64		/* bulk */

#define RX_FIFO_SIZE			512
#define NPTX_FIFO_START_ADDR		RX_FIFO_SIZE
#define NPTX_FIFO_SIZE			512
#define PTX_FIFO_SIZE			512

#define	DEVICE_DESCRIPTOR_SIZE		(18)
#define	CONFIG_DESCRIPTOR_SIZE		(9 + 9 + 7 + 7)

/* SPEC1.1 */

/* configuration descriptor: bmAttributes */
enum config_attributes {
	CONF_ATTR_DEFAULT		= 0x80,
	CONF_ATTR_REMOTE_WAKEUP		= 0x20,
	CONF_ATTR_SELFPOWERED		= 0x40
};

/* endpoint descriptor */
enum endpoint_attributes {
	EP_ADDR_IN			= 0x80,
	EP_ADDR_OUT			= 0x00,

	EP_ATTR_CONTROL			= 0x00,
	EP_ATTR_ISOCHRONOUS		= 0x01,
	EP_ATTR_BULK			= 0x02,
	EP_ATTR_INTERRUPT		= 0x03
};

/* Standard bRequest codes */
enum standard_request_code {
	STANDARD_GET_STATUS		= 0,
	STANDARD_CLEAR_FEATURE		= 1,
	STANDARD_RESERVED_1		= 2,
	STANDARD_SET_FEATURE		= 3,
	STANDARD_RESERVED_2		= 4,
	STANDARD_SET_ADDRESS		= 5,
	STANDARD_GET_DESCRIPTOR		= 6,
	STANDARD_SET_DESCRIPTOR		= 7,
	STANDARD_GET_CONFIGURATION	= 8,
	STANDARD_SET_CONFIGURATION	= 9,
	STANDARD_GET_INTERFACE		= 10,
	STANDARD_SET_INTERFACE		= 11,
	STANDARD_SYNCH_FRAME		= 12
};

enum descriptortype {
	DESCRIPTORTYPE_DEVICE		= 1,
	DESCRIPTORTYPE_CONFIGURATION	= 2,
	DESCRIPTORTYPE_STRING		= 3,
	DESCRIPTORTYPE_INTERFACE	= 4,
	DESCRIPTORTYPE_ENDPOINT		= 5
};

#define CONTROL_EP			0
#define BULK_IN_EP			1
#define BULK_OUT_EP			2

/*
 * USB2.0 HS OTG
 */

struct nx_usb_otg_gcsr_registerset {
	u32 gotgctl;			/* 0x000 R/W OTG Control and Status
					   Register */
	u32 gotgint;			/* 0x004 R/W OTG Interrupt Register */
	u32 gahbcfg;			/* 0x008 R/W Core AHB Configuration
					   Register */
	u32 gusbcfg;			/* 0x00C R/W Core USB Configuration
					   Register */
	u32 grstctl;			/* 0x010 R/W Core Reset Register */
	u32 gintsts;			/* 0x014 R/W Core Interrupt Register */
	u32 gintmsk;			/* 0x018 R/W Core Interrupt Mask
					   Register */
	u32 grxstsr;			/* 0x01C R   Receive Status Debug Read
					   Register */
	u32 grxstsp;			/* 0x020 R/W Receive Status Debug Pop
					   Register */
	u32 grxfsiz;			/* 0x024 R/W Receive FIFO Size Register
					   */
	u32 gnptxfsiz;			/* 0x028 R   Non-Periodic Transmit FIFO
					   Size Register */
	u32 gnptxsts;			/* 0x02C R/W Non-Periodic Transmit
					   FIFO/Queue Status Register */
	u32 reserved0;			/* 0x030     Reserved */
	u32 reserved1;			/* 0x034     Reserved */
	u32 reserved2;			/* 0x038     Reserved */
	u32 guid;			/* 0x03C R   User ID Register */
	u32 gsnpsid;			/* 0x040 R   Synopsys ID Register */
	u32 ghwcfg1;			/* 0x044 R   User HW Config1 Register */
	u32 ghwcfg2;			/* 0x048 R   User HW Config2 Register */
	u32 ghwcfg3;			/* 0x04C R   User HW Config3 Register */
	u32 ghwcfg4;			/* 0x050 R   User HW Config4 Register */
	u32 glpmcfg;			/* 0x054 R/W Core LPM Configuration
					   Register */
	u32 reserved3[(0x100-0x058)/4];	/* 0x058 ~ 0x0FC */
	u32 hptxfsiz;			/* 0x100 R/W Host Periodic Transmit FIFO
					   Size Register */
	u32 dieptxf[15];		/* 0x104 ~ 0x13C R/W Device IN Endpoint
					   Transmit FIFO Size Register */
	u32 reserved4[(0x400-0x140)/4];	/* 0x140 ~ 0x3FC */
};

struct nx_usb_otg_host_channel_registerset {
	u32 hcchar;			/* 0xn00 R/W Host Channel-n
					   Characteristics Register */
	u32 hcsplt;			/* 0xn04 R/W Host Channel-n Split
					   Control Register */
	u32 hcint;			/* 0xn08 R/W Host Channel-n Interrupt
					   Register */
	u32 hcintmsk;			/* 0xn0C R/W Host Channel-n Interrupt
					   Mask Register */
	u32 hctsiz;			/* 0xn10 R/W Host Channel-n Transfer
					   Size Register */
	u32 hcdma;			/* 0xn14 R/W Host Channel-n DMA Address
					   Register */
	u32 reserved[2];		/* 0xn18, 0xn1C Reserved */
};

struct nx_usb_otg_hmcsr_registerset {
	u32 hcfg;			/* 0x400 R/W Host Configuration Register
					   */
	u32 hfir;			/* 0x404 R/W Host Frame Interval
					   Register */
	u32 hfnum;			/* 0x408 R   Host Frame Number/Frame
					   Time Remaining Register */
	u32 reserved0;			/* 0x40C     Reserved */
	u32 hptxsts;			/* 0x410 R/W Host Periodic Transmit
					   FIFO/Queue Status Register */
	u32 haint;			/* 0x414 R   Host All Channels Interrupt
					   Register */
	u32 haintmsk;			/* 0x418 R/W Host All Channels Interrupt
					   Mask Register */
	u32 reserved1[(0x440-0x41C)/4];	/* 0x41C ~ 0x43C Reserved */
	u32 hprt;			/* 0x440 R/W Host Port Control and
					   Status Register */
	u32 reserved2[(0x500-0x444)/4];	/* 0x444 ~ 0x4FC Reserved */
	struct nx_usb_otg_host_channel_registerset hcc[16];/* 0x500 ~ 0x6FC */
	u32 reserved3[(0x800-0x700)/4];	/* 0x700 ~ 0x7FC */
};

struct nx_usb_otg_device_epi_registerset {
	u32 diepctl;			/* 0xn00 R/W Device Control IN endpoint
					   n Control Register */
	u32 reserved0;			/* 0xn04     Reserved */
	u32 diepint;			/* 0xn08 R/W Device Endpoint-n Interrupt
					   Register */
	u32 reserved1;			/* 0xn0C     Reserved */
	u32 dieptsiz;			/* 0xn10 R/W Device Endpoint-n Transfer
					   Size Register */
	u32 diepdma;			/* 0xn14 R/W Device Endpoint-n DMA
					   Address Register */
	u32 dtxfsts;			/* 0xn18 R   Device IN Endpoint Transmit
					   FIFO Status Register */
	u32 diepdmab;			/* 0xn1C R   Device Endpoint-n DMA
					   Buffer Address Register */
};

struct nx_usb_otg_device_ep0_registerset {
	u32 doepctl;			/* 0xn00 R/W Device Control OUT endpoint
					   n Control Register */
	u32 reserved0;			/* 0xn04     Reserved */
	u32 doepint;			/* 0xn08 R/W Device Endpoint-n Interrupt
					   Register */
	u32 reserved1;			/* 0xn0C     Reserved */
	u32 doeptsiz;			/* 0xn10 R/W Device Endpoint-n Transfer
					   Size Register */
	u32 doepdma;			/* 0xn14 R/W Device Endpoint-n DMA
					   Address Register */
	u32 reserved2;			/* 0xn18     Reserved */
	u32 doepdmab;			/* 0xn1C R   Device Endpoint-n DMA
					   Buffer Address Register */
};

struct nx_usb_otg_dmcsr_registerset {
	u32 dcfg;			/* 0x800 R/W Device Configuration
					   Register */
	u32 dctl;			/* 0x804 R/W Device Control Register */
	u32 dsts;			/* 0x808 R   Device Status Register */
	u32 reserved0;			/* 0x80C     Reserved */
	u32 diepmsk;			/* 0x810 R/W Device IN Endpoint Common
					   Interrupt Mask Register */
	u32 doepmsk;			/* 0x814 R/W Device OUT Endpoint Common
					   Interrupt Mask Register */
	u32 daint;			/* 0x818 R   Device All Endpoints
					   Interrupt Register */
	u32 daintmsk;			/* 0x81C R/W Device All Endpoints
					   Interrupt Mask Register */
	u32 reserved1;			/* 0x820     Reserved */
	u32 reserved2;			/* 0x824     Reserved */
	u32 dvbusdis;			/* 0x828 R/W Device VBUS Discharge Time
					   Register */
	u32 dvbuspulse;			/* 0x82C R/W Device VBUS Pulsing Time
					   Register */
	u32 dthrctl;			/* 0x830 R/W Device Threshold Control
					   Register */
	u32 diepempmsk;			/* 0x834 R/W Device IN Endpoint FIFO
					   Empty Interrupt Mask Register */
	u32 reserved3;			/* 0x838     Reserved */
	u32 reserved4;			/* 0x83C     Reserved */
	u32 reserved5[0x10];		/* 0x840 ~ 0x87C    Reserved */
	u32 reserved6[0x10];		/* 0x880 ~ 0x8BC    Reserved */
	u32 reserved7[0x10];		/* 0x8C0 ~ 0x8FC    Reserved */
	struct nx_usb_otg_device_epi_registerset depir[16]; /* 0x900 ~ 0xAFC */
	struct nx_usb_otg_device_ep0_registerset depor[16]; /* 0xB00 ~ 0xCFC */
};

struct nx_usb_otg_phyctrl_registerset {
	u32 reserved0[0x40/4];		/* 0x00 ~ 0x3C	Reserved */
	u32 phypor;			/* 0x40 */
	u32 vbusintenb;			/* 0x44 */
	u32 vbuspend;			/* 0x48 */
	u32 testparm3;			/* 0x4C */
	u32 testparm4;			/* 0x50 */
	u32 linkctl;			/* 0x54 */
	u32 testparm6;			/* 0x58 */
	u32 testparm7;			/* 0x5C */
	u32 testparm8;			/* 0x60 */
	u32 testparm9;			/* 0x64 */
	u32 reserved4[(0x100-0x68)/4];	/* 0x68 ~ 0xFC	Reserved */
};
struct nx_usb_otg_ifclk_registerset {
	u32 reserved0[0xc0/4];		/* 0x00 ~ 0xBC	Reserved  */
	u32 ifclk_mode;			/* 0xC0 */
	u32 ifclkgen;			/* 0xC4 */
	u32 reserved1[(0x100-0xc8)/4];	/* 0xC8 ~ 0xFC */
};
struct nx_usb_otg_registerset {
	struct nx_usb_otg_gcsr_registerset  gcsr;/* 0x0000 ~ 0x03FC */
	struct nx_usb_otg_hmcsr_registerset hcsr;/* 0x0400 ~ 0x07FC */
	struct nx_usb_otg_dmcsr_registerset dcsr;/* 0x0800 ~ 0x0CFC */
	u32 reserved0[(0xe00-0xd00)/4];	/* 0x0D00 ~ 0x0DFC	Reserved */
	u32 pcgcctl;			/* 0x0E00 R/W Power and Clock Gating
					   Control Register */
	u32 reserved1[(0x1000-0xe04)/4];/* 0x0E04 ~ 0x0FFC	Reserved */
	u32 epfifo[15][1024];		/* 0x1000 ~ 0xFFFC Endpoint Fifo */
	/*u32 epfifo[16][1024];*/	/* 0x1000 ~ 0x10FFC Endpoint Fifo */
	/*u32 reserved2[(0x20000-0x11000)/4];*//* 0x11000 ~ 0x20000 Reserved */
	/*u32 debugfifo[0x8000];*/	/* 0x20000 ~ 0x3FFFC Debug Purpose
					   Direct Fifo Acess Register */
};

/*definitions related to CSR setting */
/* USB Global Interrupt Status register(GINTSTS) setting value */
#define WKUP_INT			(1u<<31)
#define OEP_INT				(1<<19)
#define IEP_INT				(1<<18)
#define ENUM_DONE			(1<<13)
#define USB_RST				(1<<12)
#define USB_SUSP			(1<<11)
#define RXF_LVL				(1<<4)

/* NX_OTG_GOTGCTL*/
#define B_SESSION_VALID			(0x1<<19)
#define A_SESSION_VALID			(0x1<<18)

/* NX_OTG_GAHBCFG*/
#define PTXFE_HALF			(0<<8)
#define PTXFE_ZERO			(1<<8)
#define NPTXFE_HALF			(0<<7)
#define NPTXFE_ZERO			(1<<7)
#define MODE_SLAVE			(0<<5)
#define MODE_DMA			(1<<5)
#define BURST_SINGLE			(0<<1)
#define BURST_INCR			(1<<1)
#define BURST_INCR4			(3<<1)
#define BURST_INCR8			(5<<1)
#define BURST_INCR16			(7<<1)
#define GBL_INT_UNMASK			(1<<0)
#define GBL_INT_MASK			(0<<0)

/* NX_OTG_GRSTCTL*/
#define AHB_MASTER_IDLE			(1u<<31)
#define CORE_SOFT_RESET			(0x1<<0)

/* NX_OTG_GINTSTS/NX_OTG_GINTMSK core interrupt register */
#define INT_RESUME			(1u<<31)
#define INT_DISCONN			(0x1<<29)
#define INT_CONN_ID_STS_CNG		(0x1<<28)
#define INT_OUT_EP			(0x1<<19)
#define INT_IN_EP			(0x1<<18)
#define INT_ENUMDONE			(0x1<<13)
#define INT_RESET			(0x1<<12)
#define INT_SUSPEND			(0x1<<11)
#define INT_TX_FIFO_EMPTY		(0x1<<5)
#define INT_RX_FIFO_NOT_EMPTY		(0x1<<4)
#define INT_SOF				(0x1<<3)
#define INT_DEV_MODE			(0x0<<0)
#define INT_HOST_MODE			(0x1<<1)

/* NX_OTG_GRXSTSP STATUS*/
#define GLOBAL_OUT_NAK			(0x1<<17)
#define OUT_PKT_RECEIVED		(0x2<<17)
#define OUT_TRNASFER_COMPLETED		(0x3<<17)
#define SETUP_TRANSACTION_COMPLETED	(0x4<<17)
#define SETUP_PKT_RECEIVED		(0x6<<17)

/* NX_OTG_DCTL device control register */
#define NORMAL_OPERATION		(0x1<<0)
#define SOFT_DISCONNECT			(0x1<<1)

/* NX_OTG_DAINT device all endpoint interrupt register */
#define INT_IN_EP0			(0x1<<0)
#define INT_IN_EP1			(0x1<<1)
#define INT_IN_EP3			(0x1<<3)
#define INT_OUT_EP0			(0x1<<16)
#define INT_OUT_EP2			(0x1<<18)
#define INT_OUT_EP4			(0x1<<20)

/* NX_OTG_DIEPCTL0/NX_OTG_DOEPCTL0 */
#define DEPCTL_EPENA			(0x1u<<31)
#define DEPCTL_EPDIS			(0x1<<30)
#define DEPCTL_SNAK			(0x1<<27)
#define DEPCTL_CNAK			(0x1<<26)
#define DEPCTL_STALL			(0x1<<21)
#define DEPCTL_ISO_TYPE			(EP_TYPE_ISOCHRONOUS<<18)
#define DEPCTL_BULK_TYPE		(EP_TYPE_BULK<<18)
#define DEPCTL_INTR_TYPE		(EP_TYPE_INTERRUPT<<18)
#define DEPCTL_USBACTEP			(0x1<<15)

/*ep0 enable, clear nak, next ep0, max 64byte */
#define EPEN_CNAK_EP0_64 (DEPCTL_EPENA|DEPCTL_CNAK|(CONTROL_EP<<11)|(0<<0))

/*ep0 enable, clear nak, next ep0, 8byte */
#define EPEN_CNAK_EP0_8 (DEPCTL_EPENA|DEPCTL_CNAK|(CONTROL_EP<<11)|(3<<0))

/* DIEPCTLn/DOEPCTLn */
#define BACK2BACK_SETUP_RECEIVED	(0x1<<6)
#define INTKN_TXFEMP			(0x1<<4)
#define NON_ISO_IN_EP_TIMEOUT		(0x1<<3)
#define CTRL_OUT_EP_SETUP_PHASE_DONE	(0x1<<3)
#define AHB_ERROR			(0x1<<2)
#define TRANSFER_DONE			(0x1<<0)

struct nx_setup_packet {
	u8 requesttype;
	u8 request;
	u16 value;
	u16 index;
	u16 length;
};

enum usb_speed {
	USB_HIGH,
	USB_FULL,
	USB_LOW
	/*,0xFFFFFFFFUL*/
};

enum ep_type {
	EP_TYPE_CONTROL, EP_TYPE_ISOCHRONOUS, EP_TYPE_BULK, EP_TYPE_INTERRUPT
};

/* EP0 state */
enum ep0_state {
	EP0_STATE_INIT			    = 0,
	EP0_STATE_GET_DSCPT		    = 1,    // received GET_DESCRIPTOR request
	EP0_STATE_GET_INTERFACE		= 2,    // received GET_INTERFACE request
	EP0_STATE_GET_CONFIG		= 3,    // received GET_CONFIG request
	EP0_STATE_GET_STATUS		= 4,    // received GET_STATUS request
    EP0_STATE_SETUP_NODATA      = 5,    // received request not having DATA stage
    EP0_STATE_SETUP_NODATA_SENT = 6,    // in STATUS stage: empty packet stored in TxFIFO
    EP0_STATE_DATA_SENT         = 7,    // in DATA stage: descriptor stored in TxFIFO
    EP0_STATE_DATA_SENT_ACK     = 8     // in DATA stage: descriptor sent + recv ACK

};

struct  nx_usbboot_status {
	bool		downloading;
	u8		*rx_buf_addr;
	s32		rx_size;

	u32		ep0_state;
	enum usb_speed	speed;
	u32		ctrl_max_pktsize;
	u32		bulkin_max_pktsize;
	u32		bulkout_max_pktsize;

	u8		*current_ptr;
	u32		current_fifo_size;
	u32		remain_size;

	u32		up_addr;
	u32		up_size;
	u8		*up_ptr;

	u8		cur_config;
	u8		cur_interface;
	u8		cur_setting;
	u8		reserved;

	const u8	*device_descriptor;
	const u8	*config_descriptor;
} __aligned(4);

/* @brief ECID Module's Register List */
struct  nx_ecid_registerset {
	u32 ecid[4];           /* 0x00 ~ 0x0C	: 128bit ECID Register */
	u8  chipname[48];      /* 0x10 ~ 0x3C	: Chip Name Register */
	u32 reserved;          /* 0x40		: Reserved Region */
	u32 guid0;             /* 0x44		: GUID 0 Register */
	u16 guid1;             /* 0x48		: GUID 1 Register */
	u16 guid2;             /* 0x4A		: GUID 2 Register */
	u8  guid3[8];          /* 0x4C ~ x50    : GUID 3-0 ~ 3-7 Register */
	u32 ec[3];             /* 0x54 ~ 0x5C	: EC 0 ~ 3 Register */
};

struct nx_guid {
	u32 guid0;
	u16 guid1;
	u16 guid2;
	u8  guid3[8];
};

#if 0
#define LOGBUF_BEG ((unsigned*)0xffff0000)
#define LOGBUF_END ((unsigned*)0xffffffe0)

static unsigned *logbuf = LOGBUF_BEG;

//#define dolog_mypr

static void dolog_init(void)
{
    logbuf = LOGBUF_BEG;
}

static void dolog0(const char *fmt)
{
#ifdef dolog_mypr
    printf(fmt);
#endif
    if( logbuf < LOGBUF_END )
        *logbuf++ = (unsigned long)fmt & 0x3fffffff;
}

static void dolog1(const char *fmt, unsigned arg)
{
#ifdef dolog_mypr
    printf(fmt, arg);
#endif
    if( logbuf < LOGBUF_END ) {
        *logbuf++ = (unsigned long)fmt & 0x7fffffff;
        *logbuf++ = arg;
    }
}

static void dolog2(const char *fmt, unsigned arg1, unsigned arg2)
{
#ifdef dolog_mypr
    printf(fmt, arg1, arg2);
#endif
    if( logbuf < LOGBUF_END ) {
        *logbuf++ = ((unsigned long)fmt & 0x3fffffff) | 0x80000000;
        *logbuf++ = arg1;
        *logbuf++ = arg2;
    }
}

static void dolog3(const char *fmt, unsigned arg1, unsigned arg2, unsigned arg3)
{
#ifdef dolog_mypr
    printf(fmt, arg1, arg2, arg3);
#endif
    if( logbuf < LOGBUF_END ) {
        *logbuf++ = (unsigned long)fmt | 0xc0000000;
        *logbuf++ = arg1;
        *logbuf++ = arg2;
        *logbuf++ = arg3;
    }
}

static void doerr0(const char *fmt)
{
    printf(fmt);
    if( logbuf < LOGBUF_END )
        *logbuf++ = (unsigned long)fmt & 0x3fffffff;
}

static void doerr1(const char *fmt, unsigned arg)
{
    printf(fmt, arg);
    if( logbuf < LOGBUF_END ) {
        *logbuf++ = (unsigned long)fmt & 0x7fffffff;
        *logbuf++ = arg;
    }
}

int do_udownlog(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    unsigned *le = LOGBUF_BEG;
    long addr;
    unsigned arg1 = 0, arg2 = 0, arg3 = 0;

    printf("\n\n\nlog entries: %d%s\n", (unsigned)(logbuf - le),
            logbuf >= LOGBUF_END ? " (limit)" : "");
    while( le < logbuf ) {
        addr = (*le & 0x3fffffff) | 0x40000000;
        argc = *le++ >> 30;
        if( argc > 0 )
            arg1 = *le++;
        if( argc > 1 )
            arg2 = *le++;
        if( argc > 2 )
            arg3 = *le++;
        printf((const char*)addr, arg1, arg2, arg3);
    }
    printf("\n\n\n");
    return 0;
}

U_BOOT_CMD(
	udownlog, CONFIG_SYS_MAXARGS, 0, do_udownlog,
	"dump logs from last udown",
	"dumps debug logs from last execution of udown command"
);

static void printIntStatusFn(u32 int_status, u32 repeat, int isHandled)
{
    dolog0("int_status:");
    if( int_status & INT_RESUME )
        dolog0(" RESUME+31");
    if( int_status & (1<<30) )
        dolog0(" SESSREQ-30");
    if( int_status & INT_DISCONN )
        dolog0(" DISCON-29");
    if( int_status & INT_CONN_ID_STS_CNG )
        dolog0(" CONN_ID_STS_CNG-28");
    if( int_status & (1<<27) )
        dolog0(" 27");
    if( int_status & (1<<26) )
        dolog0(" 26");
    if( int_status & (1<<25) )
        dolog0(" 25");
    if( int_status & (1<<24) )
        dolog0(" 24");
    if( int_status & (1<<23) )
        dolog0(" 23");
    if( int_status & (1<<22) )
        dolog0(" 22");
    if( int_status & (1<<21) )
        dolog0(" 21");
    if( int_status & (1<<20) )
        dolog0(" 20");
    if( int_status & INT_OUT_EP )
        dolog0(" OUT_EP++19");
    if( int_status & INT_IN_EP )
        dolog0(" IN_EP++18");
    if( int_status & (1<<17) )
        dolog0(" 17");
    if( int_status & (1<<16) )
        dolog0(" 16");
    if( int_status & (1<<15) )
        dolog0(" 15");
    if( int_status & (1<<14) )
        dolog0(" 14");
    if( int_status & INT_ENUMDONE )
        dolog0(" ENUMDONE++13");
    if( int_status & INT_RESET )
        dolog0(" RESET++12");
    if( int_status & INT_SUSPEND )
        dolog0(" SUSPEND+11");
    if( int_status & (1<<10) )
        dolog0(" 10");
    if( int_status & (1<<9) )
        dolog0(" 9");
    if( int_status & (1<<8) )
        dolog0(" 8");
    if( int_status & (1<<7) )
        dolog0(" 7");
    if( int_status & (1<<6) )
        dolog0(" 6");
    if( int_status & INT_TX_FIFO_EMPTY )
        dolog0(" TX_FIFO_EMPTY-5");
    if( int_status & INT_RX_FIFO_NOT_EMPTY )
        dolog0(" RX_FIFO_NOT_EMPTY++4");
    if( int_status & INT_SOF )
        dolog0(" SOF-3");
    if( int_status & (1<<2) )
        dolog0(" 2");
    if( int_status & INT_HOST_MODE )
        dolog0(" HOST_MODE-1");
    if( int_status & INT_DEV_MODE )
        dolog0(" DEV_MODE-0");
    if( repeat > 1 )
        dolog1(" -- repeated %u times", repeat);
    if( isHandled )
        dolog0(" (handled)\n");
    else
        dolog0(" (ignored)\n");
}

static void print_int_status(u32 int_status, int isHandled)
{
    static u32 int_status_prev, repeat, isHandledPrev;

    if( ! isHandled && int_status == int_status_prev ) {
        if( ++repeat >= 10000000 ) {
            printIntStatusFn(int_status, repeat, isHandled);
            repeat = 0;
        }
    }else{
        if( repeat )
            printIntStatusFn(int_status_prev, repeat, isHandledPrev);
        if( int_status )
            printIntStatusFn(int_status, 1, isHandled);
        int_status_prev = int_status;
        repeat = 0;
        isHandledPrev = isHandled;
    }
}
#else
#define dolog_init()
#define dolog0(arg)
#define dolog1(arg1, arg2)
#define dolog2(arg1, arg2, arg3)
#define dolog3(arg1, arg2, arg3, arg4)
#define doerr0 printf
#define doerr1 printf
#define print_int_status(int_status, isHandled)
#endif

void cal_usbid(u16 *vid, u16 *pid, u32 ecid)
{
	if (ecid == 0) {   /* ecid is not burned */
		*vid = VENDORID;
		*pid = PRODUCTID;
		dolog2("\nECID Null!!\nVID %x, PID %x\n", *vid, *pid);
	} else {
		*vid = (ecid >> 16)&0xFFFF;
		*pid = (ecid >> 0)&0xFFFF;
		dolog2("  VID %x, PID %x\n", *vid, *pid);
	}
}

void get_usbid(u16 *vid, u16 *pid)
{
	struct nx_ecid_registerset * const nx_ecidreg
		= (struct nx_ecid_registerset *)PHY_BASEADDR_ECID;
	char *cmp_name = "NEXELL-NXP4330-R0-LF3000";
	char name[49];
	int i;

	for (i = 0 ; i < 48 ; i++)
		name[i] = (char)nx_ecidreg->chipname[i];

	for (i = 0; i < 48; i++) {
		if ((name[i] == '-') && (name[i+1] == '-')) {
			name[i] = 0;
			name[i+1] = 0;
		}
	}

	if (!strcmp(name, cmp_name)) {
		*vid = NXP4330_USBD_VID;
		*pid = NXP4330_USBD_PID;
	} else {
		u32 id = readl(&nx_ecidreg->ecid[3]);

		cal_usbid(vid, pid, id);
	}
}

static struct nx_usb_otg_registerset *nx_otgreg =
		(struct nx_usb_otg_registerset *)PHY_BASEADDR_HSOTG;
static struct nx_usbboot_status *usbboot_status;

static u8 gs_device_descriptor_fs[DEVICE_DESCRIPTOR_SIZE]
	__aligned(4) = {
	18,				/* 0 desc size */
	(u8)(DESCRIPTORTYPE_DEVICE),	/* 1 desc type (DEVICE) */
	(u8)(FULL_USB_VER % 0x100),	/* 2 USB release */
	(u8)(FULL_USB_VER / 0x100),	/* 3 => 1.00 */
	0xFF,				/* 4 class */
	0xFF,				/* 5 subclass */
	0xFF,				/* 6 protocol */
	(u8)FULL_MAX_PKT_SIZE_EP0,	/* 7 max pack size */
	(u8)(VENDORID % 0x100),		/* 8 vendor ID LSB */
	(u8)(VENDORID / 0x100),		/* 9 vendor ID MSB */
	(u8)(PRODUCTID % 0x100),	/* 10 product ID LSB
					   (second product) */
	(u8)(PRODUCTID / 0x100),	/* 11 product ID MSB */
	0x00,				/* 12 device release LSB */
	0x00,				/* 13 device release MSB */
	0x00,				/* 14 manufacturer string desc index */
	0x00,				/* 15 product string desc index */
	0x00,				/* 16 serial num string desc index */
	0x01				/* 17 num of possible configurations */
};

static u8 gs_device_descriptor_hs[DEVICE_DESCRIPTOR_SIZE]
	__aligned(4) = {
	18,				/* 0 desc size */
	(u8)(DESCRIPTORTYPE_DEVICE),	/* 1 desc type (DEVICE) */
	(u8)(HIGH_USB_VER % 0x100),	/*  2 USB release */
	(u8)(HIGH_USB_VER / 0x100),	/* 3 => 1.00 */
	0xFF,				/* 4 class */
	0xFF,				/* 5 subclass */
	0xFF,				/* 6 protocol */
	(u8)HIGH_MAX_PKT_SIZE_EP0,	/* 7 max pack size */
	(u8)(VENDORID	% 0x100),	/* 8 vendor ID LSB */
	(u8)(VENDORID	/ 0x100),	/* 9 vendor ID MSB */
	(u8)(PRODUCTID % 0x100),	/* 10 product ID LSB
					   (second product) */
	(u8)(PRODUCTID / 0x100),	/* 11 product ID MSB */
	0x00,				/* 12 device release LSB */
	0x00,				/* 13 device release MSB */
	0x00,				/* 14 manufacturer string desc index */
	0x00,				/* 15 product string desc index */
	0x00,				/* 16 serial num string desc index */
	0x01				/* 17 num of possible configurations */
};


static const u8	gs_config_descriptor_fs[CONFIG_DESCRIPTOR_SIZE]
	__aligned(4) = {
	/* Configuration Descriptor */
	0x09,					/* 0 desc size */
	(u8)(DESCRIPTORTYPE_CONFIGURATION),	/* 1 desc type (CONFIGURATION)
						 */
	(u8)(CONFIG_DESCRIPTOR_SIZE % 0x100),	/* 2 total length of data
						   returned LSB */
	(u8)(CONFIG_DESCRIPTOR_SIZE / 0x100),	/* 3 total length of data
						   returned MSB */
	0x01,					/* 4 num of interfaces */
	0x01,					/* 5 value to select config (1
						   for now) */
	0x00,					/* 6 index of string desc ( 0
						   for now) */
	CONF_ATTR_DEFAULT|CONF_ATTR_SELFPOWERED,/* 7 bus powered */
	25,					/* 8 max power, 50mA for now */

	/* Interface Decriptor */
	0x09,					/* 0 desc size */
	(u8)(DESCRIPTORTYPE_INTERFACE),		/* 1 desc type (INTERFACE) */
	0x00,					/* 2 interface index. */
	0x00,					/* 3 value for alternate setting
						   */
	0x02,					/* 4 bNumEndpoints (number
						   endpoints used, excluding
						   EP0) */
	0xFF,					/* 5 */
	0xFF,					/* 6 */
	0xFF,					/* 7 */
	0x00,					/* 8 string index, */

	/* Endpoint descriptor (EP 1 Bulk IN) */
	0x07,					/* 0 desc size */
	(u8)(DESCRIPTORTYPE_ENDPOINT),		/* 1 desc type (ENDPOINT) */
	BULK_IN_EP|EP_ADDR_IN,			/* 2 endpoint address: endpoint
						   1, IN */
	EP_ATTR_BULK,				/* 3 endpoint attributes: Bulk
						 */
	(u8)(FULL_MAX_PKT_SIZE_EP1 % 0x100),	/* 4 max packet size LSB */
	(u8)(FULL_MAX_PKT_SIZE_EP1 / 0x100),	/* 5 max packet size MSB */
	0x00,					/* 6 polling interval
						   (4ms/bit=time,500ms) */

	/* Endpoint descriptor (EP 2 Bulk OUT) */
	0x07,					/* 0 desc size */
	(u8)(DESCRIPTORTYPE_ENDPOINT),		/* 1 desc type (ENDPOINT) */
	BULK_OUT_EP|EP_ADDR_OUT,		/* 2 endpoint address: endpoint
						   2, OUT */
	EP_ATTR_BULK,				/* 3 endpoint attributes: Bulk
						 */
	(u8)(FULL_MAX_PKT_SIZE_EP2 % 0x100),	/* 4 max packet size LSB */
	(u8)(FULL_MAX_PKT_SIZE_EP2 / 0x100),	/* 5 max packet size MSB */
	0x00					/* 6 polling interval
						   (4ms/bit=time,500ms) */
};

static const u8	gs_config_descriptor_hs[CONFIG_DESCRIPTOR_SIZE]
	__aligned(4) = {
	/* Configuration Descriptor */
	0x09,					/* 0 desc size */
	(u8)(DESCRIPTORTYPE_CONFIGURATION),	/* 1 desc type (CONFIGURATION)
						 */
	(u8)(CONFIG_DESCRIPTOR_SIZE % 0x100),	/* 2 total length of data
						   returned LSB */
	(u8)(CONFIG_DESCRIPTOR_SIZE / 0x100),	/* 3 total length of data
						   returned MSB */
	0x01,					/* 4 num of interfaces */
	0x01,					/* 5 value to select config (1
						   for now) */
	0x00,					/* 6 index of string desc ( 0
						   for now) */
	CONF_ATTR_DEFAULT|CONF_ATTR_SELFPOWERED,/* 7 bus powered */
	25,					/* 8 max power, 50mA for now */

	/* Interface Decriptor */
	0x09,					/* 0 desc size */
	(u8)(DESCRIPTORTYPE_INTERFACE),		/* 1 desc type (INTERFACE) */
	0x00,					/* 2 interface index. */
	0x00,					/* 3 value for alternate setting
						   */
	0x02,					/* 4 bNumEndpoints (number
						   endpoints used, excluding
						   EP0) */
	0xFF,					/* 5 */
	0xFF,					/* 6 */
	0xFF,					/* 7 */
	0x00,					/* 8 string index, */

	/* Endpoint descriptor (EP 1 Bulk IN) */
	0x07,					/* 0 desc size */
	(u8)(DESCRIPTORTYPE_ENDPOINT),		/* 1 desc type (ENDPOINT) */
	BULK_IN_EP|EP_ADDR_IN,			/* 2 endpoint address: endpoint
						   1, IN */
	EP_ATTR_BULK,				/* 3 endpoint attributes: Bulk
						 */
	(u8)(HIGH_MAX_PKT_SIZE_EP1 % 0x100),	/* 4 max packet size LSB */
	(u8)(HIGH_MAX_PKT_SIZE_EP1 / 0x100),	/* 5 max packet size MSB */
	0x00,					/* 6 polling interval
						   (4ms/bit=time,500ms) */
	/* Endpoint descriptor (EP 2 Bulk OUT) */
	0x07,					/* 0 desc size */
	(u8)(DESCRIPTORTYPE_ENDPOINT),		/* 1 desc type (ENDPOINT) */
	BULK_OUT_EP|EP_ADDR_OUT,		/* 2 endpoint address: endpoint
						   2, OUT */
	EP_ATTR_BULK,				/* 3 endpoint attributes: Bulk
						 */
	(u8)(HIGH_MAX_PKT_SIZE_EP2 % 0x100),	/* 4 max packet size LSB */
	(u8)(HIGH_MAX_PKT_SIZE_EP2 / 0x100),	/* 5 max packet size MSB */
	0x00					/* 6 polling interval
						   (4ms/bit=time,500ms) */
};


static void ep0txwait(void)
{
    unsigned i = 0, dieptsiz;

    do {
        dieptsiz = readl(&nx_otgreg->dcsr.depir[CONTROL_EP].dieptsiz);
        ++i;
    }while( dieptsiz != 0 );
}

static void nx_usb_write_in_fifo(u32 ep, u8 *buf, u32 num)
{
	s32 i;
	u32 *dwbuf = (u32 *)buf;
    num = (num+3) >> 2;
	for (i = 0; i < num; i++)
		writel(dwbuf[i], &nx_otgreg->epfifo[ep][0]);
}

static void nx_usb_read_out_fifo(u32 ep, u8 *buf, s32 num)
{
	s32 i;
	u32 *dwbuf = (u32 *)buf;
    unsigned *addr = &nx_otgreg->epfifo[ep][0];

	for (i = 0; i < (num + 3) / 4; i++) {
		dwbuf[i] = readl(addr);
    }
}

static void nx_usb_ep0_int_hndlr(void)
{
	u32 buf[2];
	struct nx_setup_packet *setup_packet = (struct nx_setup_packet *)buf;
	u16 addr;

	//dolog0("Event EP0\n");
	dmb();

	if (usbboot_status->ep0_state == EP0_STATE_INIT) {
		buf[0] = readl(&nx_otgreg->epfifo[CONTROL_EP][0]);
		buf[1] = readl(&nx_otgreg->epfifo[CONTROL_EP][0]);


		switch (setup_packet->request) {
		case STANDARD_SET_ADDRESS:
			/* Set Address Update bit */
			addr = (setup_packet->value & 0xFF);
			dolog1("  std SET_ADDRESS: %x\n", addr);
			writel(1 << 18 | addr << 4 |
				usbboot_status->speed << 0,
				&nx_otgreg->dcsr.dcfg);
			usbboot_status->ep0_state = EP0_STATE_SETUP_NODATA;
            dolog0("  ep0_state set to NODATA\n");
			break;

		case STANDARD_SET_DESCRIPTOR:
			printf("  (warn) got std SET_DESCRIPTOR\n");
			dolog0("  std SET_DESCRIPTOR\n");
			usbboot_status->ep0_state = EP0_STATE_SETUP_NODATA;
            dolog0("  ep0_state set to NODATA\n");
			break;

		case STANDARD_SET_CONFIGURATION:
			dolog0("  std SET_CONFIGURATION\n");
			/* Configuration value in configuration descriptor */
			usbboot_status->cur_config = setup_packet->value;
			usbboot_status->ep0_state = EP0_STATE_SETUP_NODATA;
            dolog0("  ep0_state set to NODATA\n");
			//printf("  (info) got std SET_CONFIGURATION\n");
			break;

#if 0
		case STANDARD_GET_CONFIGURATION:
			dolog0("  std GET_CONFIGURATION\n");
			writel((1<<19)|(1<<0),
			       &nx_otgreg->dcsr.depir[CONTROL_EP].dieptsiz);
			/*ep0 enable, clear nak, next ep0, 8byte */
			writel(EPEN_CNAK_EP0_8,
			       &nx_otgreg->dcsr.depir[CONTROL_EP].diepctl);
			writel(usbboot_status->cur_config,
			       &nx_otgreg->epfifo[CONTROL_EP][0]);
			usbboot_status->ep0_state = EP0_STATE_INIT;
			break;
#endif

		case STANDARD_GET_DESCRIPTOR:
			usbboot_status->remain_size =
				(u32)setup_packet->length;
			switch (setup_packet->value>>8) {
			case DESCRIPTORTYPE_DEVICE:
                dolog0("  std device descriptor\n");
				usbboot_status->current_ptr =
					(u8 *)usbboot_status->device_descriptor;
				usbboot_status->current_fifo_size =
					usbboot_status->ctrl_max_pktsize;
				if (usbboot_status->remain_size
				   > DEVICE_DESCRIPTOR_SIZE)
					usbboot_status->remain_size =
						DEVICE_DESCRIPTOR_SIZE;
				usbboot_status->ep0_state = EP0_STATE_GET_DSCPT;
                dolog0("  ep0_state set to GET_DSCPT\n");
				break;

			case DESCRIPTORTYPE_CONFIGURATION:
                dolog0("  std config descriptor\n");
				usbboot_status->current_ptr =
					(u8 *)usbboot_status->config_descriptor;
				usbboot_status->current_fifo_size =
					usbboot_status->ctrl_max_pktsize;
				if (usbboot_status->remain_size
				   > CONFIG_DESCRIPTOR_SIZE)
					usbboot_status->remain_size =
						CONFIG_DESCRIPTOR_SIZE;
				usbboot_status->ep0_state = EP0_STATE_GET_DSCPT;
                dolog0("  ep0_state set to GET_DSCPT\n");
				break;
			default:
                dolog0("  std ?? descriptor\n");
				writel(readl(&nx_otgreg->dcsr.depir[0].diepctl)
				       | DEPCTL_STALL,
				       &nx_otgreg->dcsr.depir[0].diepctl);
				break;
			}
			break;

		case STANDARD_CLEAR_FEATURE:
			dolog0("  std CLEAR_FEATURE\n");
			usbboot_status->ep0_state = EP0_STATE_SETUP_NODATA;
            dolog0("  ep0_state set to NODATA\n");
			break;

		case STANDARD_SET_FEATURE:
			dolog0("  std SET_FEATURE\n");
			usbboot_status->ep0_state = EP0_STATE_SETUP_NODATA;
            dolog0("  ep0_state set to NODATA\n");
			break;

		case STANDARD_GET_STATUS:
			dolog0("  std GET_STATUS\n");
			usbboot_status->ep0_state = EP0_STATE_GET_STATUS;
            dolog0("  ep0_state set to GET_STATUS\n");
			break;

		case STANDARD_GET_INTERFACE:
			dolog0("  std GET_INTERFACE\n");
			usbboot_status->ep0_state = EP0_STATE_GET_INTERFACE;
            dolog0("  ep0_state set to GET_INTERFACE\n");
			break;

		case STANDARD_SET_INTERFACE:
			dolog0("  std SET_INTERFACE\n");
			usbboot_status->cur_interface = setup_packet->value;
			usbboot_status->cur_setting = setup_packet->value;
			usbboot_status->ep0_state = EP0_STATE_SETUP_NODATA;
            dolog0("  ep0_state set to NODATA\n");
			break;

		default:
			dolog1("  std ?? (%d)\n", setup_packet->request);
			printf("  got std ?? (%d)\n", setup_packet->request);
			break;
		}
	}
	writel((1<<19) | (usbboot_status->ctrl_max_pktsize<<0),
	       &nx_otgreg->dcsr.depir[CONTROL_EP].dieptsiz);

	if (usbboot_status->speed == USB_HIGH) {
		/*clear nak, next ep0, 64byte */
		writel(((1<<26)|(CONTROL_EP<<11)|(0<<0)),
		       &nx_otgreg->dcsr.depir[CONTROL_EP].diepctl);
	} else {
		/*clear nak, next ep0, 8byte */
		writel(((1<<26)|(CONTROL_EP<<11)|(3<<0)),
		       &nx_otgreg->dcsr.depir[CONTROL_EP].diepctl);
	}
	dmb();
}

static void nx_usb_transfer_ep0(void)
{
	switch (usbboot_status->ep0_state) {
	case EP0_STATE_INIT:
		dolog0("  EP0_STATE_INIT - STALL!\n");
        writel(readl(&nx_otgreg->dcsr.depir[CONTROL_EP].diepctl)
               | DEPCTL_STALL,
               &nx_otgreg->dcsr.depir[CONTROL_EP].diepctl);
        break;
	case EP0_STATE_SETUP_NODATA:
		dolog0("  EP0_STATE_SETUP_NODATA (send empty packet)\n");
		writel((1<<19)|(0<<0),
		       &nx_otgreg->dcsr.depir[CONTROL_EP].dieptsiz);
		/*ep0 enable, clear nak, next ep0, 8byte */
		writel(EPEN_CNAK_EP0_8,
		       &nx_otgreg->dcsr.depir[CONTROL_EP].diepctl);
        usbboot_status->ep0_state = EP0_STATE_SETUP_NODATA_SENT;
		break;

	/* GET_DESCRIPTOR:DEVICE */
	case EP0_STATE_GET_DSCPT:
		dolog0("  storing descriptor in TxFIFO\n");
		if (usbboot_status->speed == USB_HIGH) {
			/*ep0 enable, clear nak, next ep0, max 64byte */
			writel(EPEN_CNAK_EP0_64,
			       &nx_otgreg->dcsr.depir[CONTROL_EP].diepctl);
		} else {
			writel(EPEN_CNAK_EP0_8,
			       &nx_otgreg->dcsr.depir[CONTROL_EP].diepctl);
		}
		if (usbboot_status->current_fifo_size
		   >= usbboot_status->remain_size) {
			writel((1<<19)|(usbboot_status->remain_size<<0),
			       &nx_otgreg->dcsr.depir[CONTROL_EP].dieptsiz);
			nx_usb_write_in_fifo(CONTROL_EP,
					     usbboot_status->current_ptr,
					     usbboot_status->remain_size);
		} else {
			writel((1<<19)|(usbboot_status->current_fifo_size<<0),
			       &nx_otgreg->dcsr.depir[CONTROL_EP].dieptsiz);
			nx_usb_write_in_fifo(CONTROL_EP,
					     usbboot_status->current_ptr,
					     usbboot_status->current_fifo_size);
			usbboot_status->remain_size -=
				usbboot_status->current_fifo_size;
			usbboot_status->current_ptr +=
				usbboot_status->current_fifo_size;
		}
        ep0txwait();    // avoiding spurious TxFIFO empty interrupt
        usbboot_status->ep0_state = EP0_STATE_DATA_SENT;
		break;

	case EP0_STATE_GET_INTERFACE:
	case EP0_STATE_GET_CONFIG:
		dolog0("EP0_STATE_GET_..\n");

		writel((1<<19)|(1<<0),
		       &nx_otgreg->dcsr.depir[CONTROL_EP].dieptsiz);
		writel(EPEN_CNAK_EP0_8,
		       &nx_otgreg->dcsr.depir[CONTROL_EP].diepctl);

		if (usbboot_status->ep0_state == EP0_STATE_GET_INTERFACE)
			writel(usbboot_status->cur_interface,
			       &nx_otgreg->epfifo[CONTROL_EP][0]);
        else
			writel(usbboot_status->cur_config,
			       &nx_otgreg->epfifo[CONTROL_EP][0]);
        usbboot_status->ep0_state = EP0_STATE_DATA_SENT;
		break;
	case EP0_STATE_GET_STATUS:
		dolog0("EP0_STATE_GET_STATUS\n");

		writel((1<<19)|(2<<0),
		       &nx_otgreg->dcsr.depir[CONTROL_EP].dieptsiz);
		writel(EPEN_CNAK_EP0_8,
		       &nx_otgreg->dcsr.depir[CONTROL_EP].diepctl);

        writel(0, &nx_otgreg->epfifo[CONTROL_EP][0]);
        usbboot_status->ep0_state = EP0_STATE_DATA_SENT;
		break;
    case EP0_STATE_DATA_SENT:
        doerr0("ERROR! spurious Tx req on DATA_SENT\n");
        break;
    case EP0_STATE_SETUP_NODATA_SENT:
        doerr0("ERROR! spurious Tx req on NODATA_SENT\n");
        break;
	default:
        doerr1("ERROR! SETUP with IN data ep0_state=%d\n",
                usbboot_status->ep0_state);
		break;
	}
}


static void nx_usb_int_bulkin(void)
{
	u8 *bulkin_buf;
	u32 remain_cnt;

	dolog0("Bulk In Function\n");

	bulkin_buf = (u8 *)usbboot_status->up_ptr;
	remain_cnt = usbboot_status->up_size -
		((u32)((ulong)(usbboot_status->up_ptr -
			       usbboot_status->up_addr)));

	if (remain_cnt > usbboot_status->bulkin_max_pktsize) {
		writel((1<<19) | (usbboot_status->bulkin_max_pktsize<<0),
		       &nx_otgreg->dcsr.depir[BULK_IN_EP].dieptsiz);

		/* ep1 enable, clear nak, bulk, usb active,
		   next ep2, max pkt 64 */
		writel(1u<<31 | 1<<26 | 2<<18 | 1<<15 |
			usbboot_status->bulkin_max_pktsize<<0,
		       &nx_otgreg->dcsr.depir[BULK_IN_EP].diepctl);

		nx_usb_write_in_fifo(BULK_IN_EP, bulkin_buf,
				     usbboot_status->bulkin_max_pktsize);

		usbboot_status->up_ptr += usbboot_status->bulkin_max_pktsize;

	} else if (remain_cnt > 0) {
		writel((1<<19)|(remain_cnt<<0),
		       &nx_otgreg->dcsr.depir[BULK_IN_EP].dieptsiz);

		/* ep1 enable, clear nak, bulk, usb active,
		  next ep2, max pkt 64 */
		writel(1u<<31 | 1<<26 | 2<<18 | 1<<15 |
			usbboot_status->bulkin_max_pktsize<<0,
		       &nx_otgreg->dcsr.depir[BULK_IN_EP].diepctl);

		nx_usb_write_in_fifo(BULK_IN_EP, bulkin_buf, remain_cnt);

		usbboot_status->up_ptr += remain_cnt;

	} else { /*remain_cnt = 0*/
		writel((DEPCTL_SNAK|DEPCTL_BULK_TYPE),
		       &nx_otgreg->dcsr.depir[BULK_IN_EP].diepctl);
	}
}
static void nx_usb_int_bulkout(u32 fifo_cnt_byte)
{
	nx_usb_read_out_fifo(BULK_OUT_EP, (u8 *)usbboot_status->rx_buf_addr,
			     fifo_cnt_byte);

	usbboot_status->rx_buf_addr	+= fifo_cnt_byte;
	usbboot_status->rx_size -= fifo_cnt_byte;

	if (usbboot_status->rx_size <= 0) {
		dolog0("Download completed!\n");

		usbboot_status->downloading = false;
	}

	writel((1<<19)|(usbboot_status->bulkout_max_pktsize<<0),
	       &nx_otgreg->dcsr.depor[BULK_OUT_EP].doeptsiz);

	/*ep2 enable, clear nak, bulk, usb active, next ep2, max pkt 64*/
	writel(1u<<31|1<<26|2<<18|1<<15|usbboot_status->bulkout_max_pktsize<<0,
	       &nx_otgreg->dcsr.depor[BULK_OUT_EP].doepctl);
}

static void nx_usb_reset(void)
{
	u32 i;

    dolog0("  nx_usb_reset\n");
	/* set all out ep nak */
	for (i = 0; i < 16; i++)
		writel(readl(&nx_otgreg->dcsr.depor[i].doepctl) | DEPCTL_SNAK,
		       &nx_otgreg->dcsr.depor[i].doepctl);

	usbboot_status->ep0_state = EP0_STATE_INIT;
    dolog0("  ep0_state set to INIT\n");
	writel(((1<<BULK_OUT_EP)|(1<<CONTROL_EP))<<16 |
		((1<<BULK_IN_EP)|(1<<CONTROL_EP)),
	       &nx_otgreg->dcsr.daintmsk);
	writel(CTRL_OUT_EP_SETUP_PHASE_DONE|AHB_ERROR|TRANSFER_DONE,
	       &nx_otgreg->dcsr.doepmsk);
	writel(INTKN_TXFEMP|NON_ISO_IN_EP_TIMEOUT|AHB_ERROR|TRANSFER_DONE,
	       &nx_otgreg->dcsr.diepmsk);

	/* Rx FIFO Size */
	writel(RX_FIFO_SIZE, &nx_otgreg->gcsr.grxfsiz);


	/* Non Periodic Tx FIFO Size */
	writel(NPTX_FIFO_SIZE<<16 | NPTX_FIFO_START_ADDR<<0,
	       &nx_otgreg->gcsr.gnptxfsiz);

	/* clear all out ep nak */
	for (i = 0; i < 16; i++)
		writel(readl(&nx_otgreg->dcsr.depor[i].doepctl) |
		       (DEPCTL_EPENA|DEPCTL_CNAK),
		       &nx_otgreg->dcsr.depor[i].doepctl);

	/*clear device address */
	writel(readl(&nx_otgreg->dcsr.dcfg) & ~(0x7F<<4),
	       &nx_otgreg->dcsr.dcfg);
	dmb();
}

static bool nx_usb_set_init(void)
{
	u32 status = readl(&nx_otgreg->dcsr.dsts); /* System status read */
	u16	VID = VENDORID, PID = PRODUCTID;

	/* Set if Device is High speed or Full speed */
	if (((status & 0x6) >> 1) == USB_HIGH) {
		dolog0("  High Speed Detection\n");
	} else if (((status & 0x6) >> 1) == USB_FULL) {
		dolog0("  Full Speed Detection\n");
	} else {
		dolog0("**** Error:Neither High_Speed nor Full_Speed\n");
		return false;
	}

	if (((status & 0x6) >> 1) == USB_HIGH)
		usbboot_status->speed = USB_HIGH;
	else
		usbboot_status->speed = USB_FULL;

	usbboot_status->rx_size = 512;

	/*
	 * READ ECID for Product and Vendor ID
	 */
	get_usbid(&VID, &PID);
	dolog2("  nx_usb_set_init %x %x\n", VID, PID);
	gs_device_descriptor_fs[8] = (u8)(VID & 0xff);
	gs_device_descriptor_hs[8] = gs_device_descriptor_fs[8];
	gs_device_descriptor_fs[9] = (u8)(VID >> 8);
	gs_device_descriptor_hs[9] = gs_device_descriptor_fs[9];
	gs_device_descriptor_fs[10] = (u8)(PID & 0xff);
	gs_device_descriptor_hs[10] = gs_device_descriptor_fs[10];
	gs_device_descriptor_fs[11] = (u8)(PID >> 8);
	gs_device_descriptor_hs[11] = gs_device_descriptor_fs[11];

	/* set endpoint */
	/* Unmask NX_OTG_DAINT source */
	writel(0xFF, &nx_otgreg->dcsr.depir[CONTROL_EP].diepint);
	writel(0xFF, &nx_otgreg->dcsr.depor[CONTROL_EP].doepint);
	writel(0xFF, &nx_otgreg->dcsr.depir[BULK_IN_EP].diepint);
	writel(0xFF, &nx_otgreg->dcsr.depor[BULK_OUT_EP].doepint);

	/* Init For Ep0*/
	if (usbboot_status->speed == USB_HIGH) {
		usbboot_status->ctrl_max_pktsize = HIGH_MAX_PKT_SIZE_EP0;
		usbboot_status->bulkin_max_pktsize = HIGH_MAX_PKT_SIZE_EP1;
		usbboot_status->bulkout_max_pktsize = HIGH_MAX_PKT_SIZE_EP2;
		usbboot_status->device_descriptor = gs_device_descriptor_hs;
		usbboot_status->config_descriptor = gs_config_descriptor_hs;

		/*MPS:64bytes */
		writel(((1<<26)|(CONTROL_EP<<11)|(0<<0)),
		       &nx_otgreg->dcsr.depir[CONTROL_EP].diepctl);
		/*ep0 enable, clear nak */
		writel((1u<<31)|(1<<26)|(0<<0),
		       &nx_otgreg->dcsr.depor[CONTROL_EP].doepctl);
	} else {
		usbboot_status->ctrl_max_pktsize = FULL_MAX_PKT_SIZE_EP0;
		usbboot_status->bulkin_max_pktsize = FULL_MAX_PKT_SIZE_EP1;
		usbboot_status->bulkout_max_pktsize = FULL_MAX_PKT_SIZE_EP2;
		usbboot_status->device_descriptor = gs_device_descriptor_fs;
		usbboot_status->config_descriptor = gs_config_descriptor_fs;

		/*MPS:8bytes */
		writel(((1<<26)|(CONTROL_EP<<11)|(3<<0)),
		       &nx_otgreg->dcsr.depir[CONTROL_EP].diepctl);
		/*ep0 enable, clear nak */
		writel((1u<<31)|(1<<26)|(3<<0),
		       &nx_otgreg->dcsr.depor[CONTROL_EP].doepctl);
	}

	/* set_opmode */
	writel(INT_RESUME|INT_OUT_EP|INT_IN_EP|INT_ENUMDONE|
	       INT_RESET|INT_SUSPEND|INT_RX_FIFO_NOT_EMPTY,
	       &nx_otgreg->gcsr.gintmsk);

	writel(MODE_SLAVE|BURST_SINGLE|GBL_INT_UNMASK,
	       &nx_otgreg->gcsr.gahbcfg);

	writel((1<<19)|(usbboot_status->bulkout_max_pktsize<<0),
	       &nx_otgreg->dcsr.depor[BULK_OUT_EP].doeptsiz);

	writel((1<<19)|(0<<0), &nx_otgreg->dcsr.depor[BULK_IN_EP].doeptsiz);

	/*bulk out ep enable, clear nak, bulk, usb active, next ep2, max pkt */
	writel(1u<<31|1<<26|2<<18|1<<15|usbboot_status->bulkout_max_pktsize<<0,
	       &nx_otgreg->dcsr.depor[BULK_OUT_EP].doepctl);

	/*bulk in ep enable, clear nak, bulk, usb active, next ep1, max pkt */
	writel(0u<<31|1<<26|2<<18|1<<15|usbboot_status->bulkin_max_pktsize<<0,
	       &nx_otgreg->dcsr.depir[BULK_IN_EP].diepctl);

	dmb();

	return true;
}

static void nx_usb_pkt_receive(void)
{
	u32 rx_status;
	u32 fifo_cnt_byte;

	rx_status = readl(&nx_otgreg->gcsr.grxstsp);

	if ((rx_status & (0xf<<17)) == SETUP_PKT_RECEIVED) {
		dolog0("  rx fifo: SETUP_PKT_RECEIVED (got SETUP token + data)\n");
        switch( usbboot_status->ep0_state ) {
        case EP0_STATE_INIT:
            break;
        case EP0_STATE_SETUP_NODATA_SENT:
            doerr0("WARN: lost ACK of IN transaction in STATUS stage\n");
            usbboot_status->ep0_state = EP0_STATE_INIT;
            dolog0("  ep0_state set to INIT\n");
            break;
        default:
            doerr1("ERROR! SETUP pkt ep0_state=%d\n",
                    usbboot_status->ep0_state);
            break;
        }
		nx_usb_ep0_int_hndlr();
	} else if ((rx_status & (0xf<<17)) == OUT_PKT_RECEIVED) {
		fifo_cnt_byte = (rx_status & 0x7ff0)>>4;
		dolog1("  rx fifo: OUT_PKT_RECEIVED (%d bytes)\n", fifo_cnt_byte);
        if( (rx_status & 0xf) == 0 ) {  // CONTROL_EP
            if( fifo_cnt_byte == 0 ) {
                switch( usbboot_status->ep0_state ) {
                case EP0_STATE_DATA_SENT:
                    doerr0("WARN: lost ACK in DATA stage\n");
                    /* no break */
                case EP0_STATE_DATA_SENT_ACK:
                    dolog0("  ep0_state set to INIT\n");
                    usbboot_status->ep0_state = EP0_STATE_INIT;
                    break;
                default:
                    doerr1("ERROR! OUT pkt ep0_state=%d\n",
                            usbboot_status->ep0_state);
                    break;
                }
            }else{
                doerr1("ERROR! SETUP pkt with OUT data, ep0_state=%d\n",
                        usbboot_status->ep0_state);
            }
        }

		if ((rx_status & BULK_OUT_EP) && (fifo_cnt_byte)) {
			nx_usb_int_bulkout(fifo_cnt_byte);
			writel(INT_RESUME|INT_OUT_EP|INT_IN_EP|INT_ENUMDONE|
				INT_RESET|INT_SUSPEND|INT_RX_FIFO_NOT_EMPTY,
				&nx_otgreg->gcsr.gintmsk);
			dmb();
			return;
		}
	} else if ((rx_status & (0xf<<17)) == GLOBAL_OUT_NAK) {
		dolog0("  rx fifo: GLOBAL_OUT_NAK\n");
	} else if ((rx_status & (0xf<<17)) == OUT_TRNASFER_COMPLETED) {
		dolog0("  rx fifo: OUT_TRANSFER_COMPLETED "
                "(i.e. bulk recv + ACK sent)\n");
	} else if ((rx_status & (0xf<<17)) == SETUP_TRANSACTION_COMPLETED) {
		dolog0("  rx fifo: SETUP_TRANSACTION_COMPLETED (i.e. ACK sent)\n");
	} else {
		dolog0("  rx fifo: Reserved\n");
	}
	dmb();
}

static void nx_usb_transfer(void)
{
	u32 ep_int;
	u32 ep_int_status;

	ep_int = readl(&nx_otgreg->dcsr.daint);

    dolog3("  ep_int: out=0x%x, in=0x%x state=%d\n",
            ep_int >> 16, ep_int & 0xffff, usbboot_status->ep0_state);
	if (ep_int & (1<<CONTROL_EP)) {
		ep_int_status =
			readl(&nx_otgreg->dcsr.depir[CONTROL_EP].diepint);

        dolog1("  diepint0=%x (0x1 - send compl. (+recv ACK),"
                " 0x10 - IN token recv when TxFIFO empty)\n",
                ep_int_status);
		if (ep_int_status & INTKN_TXFEMP) {
			while (1) {
				if (!((readl(&nx_otgreg->gcsr.gnptxsts) &
				       0xFFFF) <
				      usbboot_status->ctrl_max_pktsize))
					break;
			}
			nx_usb_transfer_ep0();
		}
        if( ep_int_status & TRANSFER_DONE ) {
            u32 ep0_state = usbboot_status->ep0_state;
            switch( ep0_state ) {
            case EP0_STATE_DATA_SENT:
                /* received ACK - awaiting OUT transaction */
                dolog0("  ep0_state set to DATA SENT_ACK\n");
                usbboot_status->ep0_state = EP0_STATE_DATA_SENT_ACK;
                break;
            case EP0_STATE_SETUP_NODATA_SENT:
                /* received ACK on IN transaction in STATUS stage */
                dolog0("  ep0_state set to INIT\n");
                usbboot_status->ep0_state = EP0_STATE_INIT;
                break;
            default:
                doerr1("ERROR! TRANSFER_DONE ep0_state=%d\n", ep0_state);
            }
        }

		writel(ep_int_status,
		       &nx_otgreg->dcsr.depir[CONTROL_EP].diepint);
	}

	if (ep_int & ((1<<CONTROL_EP)<<16)) {
		ep_int_status =
			readl(&nx_otgreg->dcsr.depor[CONTROL_EP].doepint);
        dolog1("  doepint0=0x%x (0x1 - recv. completed, +sent ACK),"
                " 0x8 - SETUP phase done (+sent ACK))\n",
                ep_int_status);
		writel((1<<29)|(1<<19)|(8<<0),
		       &nx_otgreg->dcsr.depor[CONTROL_EP].doeptsiz);
		/*ep0 enable, clear nak */
		writel(1u<<31|1<<26,
		       &nx_otgreg->dcsr.depor[CONTROL_EP].doepctl);
		/* Interrupt Clear */
		writel(ep_int_status,
		       &nx_otgreg->dcsr.depor[CONTROL_EP].doepint);
	}

	if (ep_int & (1<<BULK_IN_EP)) {
		ep_int_status =
			readl(&nx_otgreg->dcsr.depir[BULK_IN_EP].diepint);

		/* Interrupt Clear */
		writel(ep_int_status,
		       &nx_otgreg->dcsr.depir[BULK_IN_EP].diepint);

		if (ep_int_status & INTKN_TXFEMP)
			nx_usb_int_bulkin();
	}

	if (ep_int & ((1<<BULK_OUT_EP)<<16)) {
		ep_int_status =
			readl(&nx_otgreg->dcsr.depor[BULK_OUT_EP].doepint);
		/* Interrupt Clear */
		writel(ep_int_status,
		       &nx_otgreg->dcsr.depor[BULK_OUT_EP].doepint);
	}
}

static void nx_udc_int_hndlr(void)
{
	u32 int_status;
	bool tmp;

	int_status = readl(&nx_otgreg->gcsr.gintsts); /* Core Interrupt Register */

	if ((int_status & INT_IN_EP) || (int_status & INT_OUT_EP)) {
		//dolog0("INT_IN or OUT_EP\n");
		/* Read only register field */
		nx_usb_transfer();
	}

	if (int_status & INT_RX_FIFO_NOT_EMPTY) {
		//dolog0("INT_RX_FIFO_NOT_EMPTY\n");
		/* Read only register field */

        if (int_status & (INT_IN_EP | INT_OUT_EP))
            dolog0("  --\n");
		writel(INT_RESUME|INT_OUT_EP|INT_IN_EP|
			INT_ENUMDONE|INT_RESET|INT_SUSPEND,
		       &nx_otgreg->gcsr.gintmsk);
		nx_usb_pkt_receive();
		writel(INT_RESUME|INT_OUT_EP|INT_IN_EP|
			INT_ENUMDONE|INT_RESET|INT_SUSPEND|
			INT_RX_FIFO_NOT_EMPTY, &nx_otgreg->gcsr.gintmsk);
	}

	if (int_status & INT_RESET) {
		nx_usb_reset();
	}

	if (int_status & INT_ENUMDONE) {
		//dolog0("INT_ENUMDONE\n");

		tmp = nx_usb_set_init();
		if (!tmp) {
			/* Interrupt Clear */
			writel(int_status, &nx_otgreg->gcsr.gintsts);
			return;
		}
	}

	if (int_status & INT_RESUME)
		dolog0("  INT_RESUME\n");


	if (int_status & INT_SUSPEND)
		dolog0("  INT_SUSPEND\n");

	writel(int_status, &nx_otgreg->gcsr.gintsts); /* Interrupt Clear */
}

/* Nexell USBOTG PHY registers */

/* USBOTG Configuration 0 Register */
#define NX_OTG_CON0				0x30
#define NX_OTG_CON0_SS_SCALEDOWN_MODE		(3 << 0)

/* USBOTG Configuration 1 Register */
#define NX_OTG_CON1				0x34
#define NX_OTG_CON1_VBUSVLDEXTSEL		BIT(25)
#define NX_OTG_CON1_VBUSVLDEXT			BIT(24)
#define NX_OTG_CON1_VBUS_INTERNAL ~( \
					NX_OTG_CON1_VBUSVLDEXTSEL | \
					NX_OTG_CON1_VBUSVLDEXT)
#define NX_OTG_CON1_VBUS_VLDEXT0 ( \
					NX_OTG_CON1_VBUSVLDEXTSEL | \
					NX_OTG_CON1_VBUSVLDEXT)

#define NX_OTG_CON1_POR				BIT(8)
#define NX_OTG_CON1_POR_ENB			BIT(7)
#define NX_OTG_CON1_POR_MASK			(0x3 << 7)
#define NX_OTG_CON1_RST				BIT(3)
#define NX_OTG_CON1_UTMI_RST			BIT(2)

/* USBOTG Configuration 2 Register */
#define NX_OTG_CON2				0x38
#define NX_OTG_CON2_OTGTUNE_MASK		(0x7 << 23)
#define NX_OTG_CON2_WORDINTERFACE		BIT(9)
#define NX_OTG_CON2_WORDINTERFACE_ENB		BIT(8)
#define NX_OTG_CON2_WORDINTERFACE_16 ( \
					NX_OTG_CON2_WORDINTERFACE | \
					NX_OTG_CON2_WORDINTERFACE_ENB)

/* USBOTG Configuration 3 Register */
#define NX_OTG_CON3				0x3C
#define NX_OTG_CON3_ACAENB			BIT(15)
#define NX_OTG_CON3_DCDENB			BIT(14)
#define NX_OTG_CON3_VDATSRCENB			BIT(13)
#define NX_OTG_CON3_VDATDETENB			BIT(12)
#define NX_OTG_CON3_CHRGSEL			BIT(11)
#define NX_OTG_CON3_DET_N_CHG ( \
					NX_OTG_CON3_ACAENB | \
					NX_OTG_CON3_DCDENB | \
					NX_OTG_CON3_VDATSRCENB | \
					NX_OTG_CON3_VDATDETENB | \
					NX_OTG_CON3_CHRGSEL)

void nx_otg_phy_init(void)
{
	void __iomem *phy = (void __iomem *)PHY_BASEADDR_TIEOFF;
	u32 reg;

	/* USB PHY0 Enable */
	dolog0("USB PHY0 Enable\n");

	writel(readl(phy + NX_OTG_CON3) & ~NX_OTG_CON3_DET_N_CHG,
	       phy + NX_OTG_CON3);

	nx_rstcon_setrst(RESET_ID_USB20OTG, RSTCON_ASSERT);
	udelay(10);
	nx_rstcon_setrst(RESET_ID_USB20OTG, RSTCON_NEGATE);
	udelay(10);

	reg  = readl(phy + NX_OTG_CON2) & ~NX_OTG_CON2_OTGTUNE_MASK;
	writel(reg, phy +  NX_OTG_CON2);

	writel(readl(phy + NX_OTG_CON0) & ~NX_OTG_CON0_SS_SCALEDOWN_MODE,
	       phy + NX_OTG_CON0);

	writel(readl(phy + NX_OTG_CON2) | NX_OTG_CON2_WORDINTERFACE_16,
	       phy + NX_OTG_CON2);

	writel(readl(phy + NX_OTG_CON1) & NX_OTG_CON1_VBUS_INTERNAL,
	       phy + NX_OTG_CON1);
	udelay(10);

	reg = readl(phy + NX_OTG_CON1);
	reg &= ~NX_OTG_CON1_POR_MASK;
	reg |= NX_OTG_CON1_POR_ENB;
	writel(reg, phy + NX_OTG_CON1);
	udelay(10);
	reg |= NX_OTG_CON1_POR_MASK;
	writel(reg, phy + NX_OTG_CON1);
	udelay(10);
	reg &= ~NX_OTG_CON1_POR;
	writel(reg, phy + NX_OTG_CON1);
	udelay(40);

	writel(readl(phy + NX_OTG_CON1) | NX_OTG_CON1_RST, phy + NX_OTG_CON1);
	udelay(10);

	writel(readl(phy + NX_OTG_CON1) | NX_OTG_CON1_UTMI_RST,
	       phy + NX_OTG_CON1);
	udelay(10);
}

void nx_otg_phy_off(void)
{
	void __iomem *phy = (void __iomem *)PHY_BASEADDR_TIEOFF;

	/* USB PHY0 Disable */
	dolog0("USB PHY0 Disable\n");

	writel(readl(phy + NX_OTG_CON1) | NX_OTG_CON1_VBUS_VLDEXT0,
	       phy + NX_OTG_CON1);
	udelay(10);

	writel(readl(phy + NX_OTG_CON1) & ~NX_OTG_CON1_RST, phy + NX_OTG_CON1);
	udelay(10);

	writel(readl(phy + NX_OTG_CON1) & ~NX_OTG_CON1_UTMI_RST,
	       phy + NX_OTG_CON1);
	udelay(10);

	writel(readl(phy + NX_OTG_CON1) | NX_OTG_CON1_POR_MASK,
	       phy + NX_OTG_CON1);
	udelay(10);

	nx_rstcon_setrst(RESET_ID_USB20OTG, RSTCON_ASSERT);
	udelay(10);
}

bool iusbboot(void)
{
	unsigned int *nsih;
    u32 int_status, isHandled;
    bool downloaded = false;

	nsih = (unsigned int *)usbboot_status->rx_buf_addr;

	nx_otg_phy_init();

	/* usb core soft reset */
	writel(CORE_SOFT_RESET, &nx_otgreg->gcsr.grstctl);
	dmb();

	while (1) {
		if (readl(&nx_otgreg->gcsr.grstctl) & AHB_MASTER_IDLE)
			break;
	}
	dmb();

	/* init_core */
	writel(PTXFE_HALF|NPTXFE_HALF|MODE_SLAVE|
		BURST_SINGLE|GBL_INT_UNMASK, &nx_otgreg->gcsr.gahbcfg);
	writel(0<<15		/* PHY Low Power Clock sel */
		|1<<14		/* Non-Periodic TxFIFO Rewind Enable */
		|5<<10		/* Turnaround time */
		|0<<9		/* 0:HNP disable, 1:HNP enable */
		|0<<8		/* 0:SRP disable, 1:SRP enable */
		|0<<7		/* ULPI DDR sel */
		|0<<6		/* 0: high speed utmi+, 1: full speed serial */
		|0<<4		/* 0: utmi+, 1:ulpi */
		|1<<3		/* phy i/f  0:8bit, 1:16bit */
		|7<<0		/* HS/FS Timeout**/,
	       &nx_otgreg->gcsr.gusbcfg);

	dmb();

	if ((readl(&nx_otgreg->gcsr.gintsts) & 0x1) == INT_DEV_MODE) {
		/* soft disconnect on */
		writel(readl(&nx_otgreg->dcsr.dctl) | SOFT_DISCONNECT,
		       &nx_otgreg->dcsr.dctl);
		udelay(10);
		/* soft disconnect off */
		writel(readl(&nx_otgreg->dcsr.dctl) & ~SOFT_DISCONNECT,
		       &nx_otgreg->dcsr.dctl);
		udelay(10);

		/* usb init device */
		writel(1<<18, &nx_otgreg->dcsr.dcfg); /* [][1: full speed(30Mhz)
							0:high speed] */
		writel(INT_RESUME|INT_OUT_EP|INT_IN_EP|
			INT_ENUMDONE|INT_RESET|INT_SUSPEND|
			INT_RX_FIFO_NOT_EMPTY, &nx_otgreg->gcsr.gintmsk);
		udelay(10);
	}
	dmb();

	usbboot_status->cur_config = 0;
	usbboot_status->cur_interface = 0;
	usbboot_status->cur_setting = 0;
	usbboot_status->speed = USB_HIGH;
	usbboot_status->ep0_state = EP0_STATE_INIT;
    dolog0("ep0_state set to INIT\n");

	usbboot_status->downloading = true;

	dmb();

	while (usbboot_status->downloading) {
		if (ctrlc())
			goto _exit;
        int_status = readl(&nx_otgreg->gcsr.gintsts);
        isHandled = int_status &
            (WKUP_INT|OEP_INT|IEP_INT|ENUM_DONE|USB_RST|USB_SUSP|RXF_LVL);
        print_int_status(int_status, isHandled);
		if( isHandled ) {
			nx_udc_int_hndlr();
			//writel(0xFFFFFFFF, &nx_otgreg->gcsr.gintsts);
			//mdelay(3);
		}
	}

	usbboot_status->rx_buf_addr -= 512;

	usbboot_status->rx_size = nsih[17];
	usbboot_status->downloading = true;
	printf(" Size  %d(hex : %x)\n", usbboot_status->rx_size,
	       usbboot_status->rx_size);
	dolog2(" Size  %d(hex : %x)\n", usbboot_status->rx_size,
	       usbboot_status->rx_size);
	dmb();

	while (usbboot_status->downloading) {
		if (ctrlc())
			goto _exit;

        int_status = readl(&nx_otgreg->gcsr.gintsts);
        isHandled = int_status &
            (WKUP_INT|OEP_INT|IEP_INT|ENUM_DONE|USB_RST|USB_SUSP|RXF_LVL);
        print_int_status(int_status, isHandled);
		if( isHandled ) {
			nx_udc_int_hndlr();
			writel(0xFFFFFFFF, &nx_otgreg->gcsr.gintsts);
		}
	}
    downloaded = true;
_exit:
    print_int_status(0, 0);
	dmb();
	/* usb core soft reset */
	writel(CORE_SOFT_RESET, &nx_otgreg->gcsr.grstctl);
	while (1) {
		if (readl(&nx_otgreg->gcsr.grstctl) & AHB_MASTER_IDLE)
			break;
	}

	nx_otg_phy_off();

	return downloaded;
}

int do_usbdown(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int addr;
	struct nx_usbboot_status status;
    bool complete;

	usbboot_status = &status;
	addr = simple_strtoul(argv[1], NULL, 16);

	if (addr < 0x40000000)
		goto usage;

    dolog_init();
	printf("Download Address %x", addr);
	usbboot_status->rx_buf_addr = (u8 *)((ulong)addr);
	complete = iusbboot();
	flush_dcache_all();
    if( complete )
        printf("Download complete\n");
    else
        printf(" interrupted\n");
    return !complete;
usage:
	cmd_usage(cmdtp);
	return 1;
}

U_BOOT_CMD(
	udown, CONFIG_SYS_MAXARGS, 1, do_usbdown,
	"Download USB",
	"udown addr(hex)"
);
