//{const definition}
#define	DM_VER_REL		0
#define	DM_VER_D1		1
#define	DM_VER_D2		2
#define	DM_VER_D3		3

#define	DM_VER_DEBUG		0x80
#define	DM_VER_OLDKERNEL	0x40

#define DM_VER_NEWINFO_MASK	0x0F //D[3:0]

//{const pre-setting}
//[In case need to call register_board_info]
//[SPI].(While DTS mode will be made by DTS mechanism)
#define DM_CONF_MAX_SPEED_HZ		15600000 				//When NDTS, 7800000 //31200000 //15600000(iperf middle)
#define DM_CONF_SPI_BUS_NUMBER		0					//When NDTS
#define DM_CONF_SPI_CHIP_SELECT		1					//When NDTS
//[In case interrupt]
//[DTS.(]While DTS mode will be made by DTS mechanism)
//[INT]
#define DM_NCONF_NDTS_IRQPIN		26					//When interrupt mode, Define IRQ PIN
#define DM_NCONF_IRQF_TRIGGER		(IRQF_TRIGGER_LOW | IRQF_ONESHOT)	//new
//[INT, more 2 manual active polarity settings]
#define DM_NCONF_INTCR_POL		INTCR_POL_LOW
#define DM_NCONF_POL_STR		"INT_ACTIVE_LOW"
				
//Usage:
// (DM_VER_D2 and DM_VER_D3) --for non-dts configuration 
// Used these parameters
//  max_speed_hz, bus_number, chip_select, irqnum, trigger
// in the implement source code
//
// (DM_VER_REL, DM_VER_D1) -- for dts configuration
// Get these parameters
//  max_speed_hz, bus_number, chip_select, irqnum, trigger
// from DTS tree

//{davicom config version}
//Usage examples:
//(#define CONF_VER (DM_VER_REL))
//(#define CONF_VER (DM_VER_REL | DM_VER_DEBUG))
//(#define CONF_VER (DM_VER_REL | DM_VER_OLDKERNEL))
//(#define CONF_VER (DM_VER_REL | DM_VER_DEBUG | DM_VER_OLDKERNEL))
//(#define CONF_VER (DM_VER_D1))
//(#define CONF_VER (DM_VER_D2 | DM_VER_DEBUG))
//(#define CONF_VER (DM_VER_D3))

//#define CONF_VER (DM_VER_REL | DM_VER_DEBUG)
//#define CONF_VER (DM_VER_REL | DM_VER_DEBUG)
//#define CONF_VER (DM_VER_D1 | DM_VER_DEBUG)
//#define CONF_VER (DM_VER_D2 | DM_VER_DEBUG)
//#define CONF_VER (DM_VER_D3 | DM_VER_DEBUG)
//#define CONF_VER (DM_VER_D1)	//I download linux kernel tree
#define CONF_VER (DM_VER_REL)	//idenical to Linux kernel version
//#define CONF_VER (DM_VER_D2)
//#define CONF_VER (DM_VER_D2 | DM_VER_DEBUG)
//#define CONF_VER (DM_VER_D3)
//#define CONF_VER (DM_VER_D3 | DM_VER_DEBUG)
