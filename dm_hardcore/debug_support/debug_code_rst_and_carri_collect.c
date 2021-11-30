//
// Debug for the driver._nb_info.s
//

#define nb_info		printk
//#define nb_info

//-----------------------------------------------------------------------------
// Do define 'NB_SUPPORT' in your header file to get below extra-print function
//-----------------------------------------------------------------------------
// debug: 'CONF_VER & DM_VER_NEWINFO_MASK'
// debug: _nb_info
//-----------------------------------------------------------------------------
#ifdef NB_SUPPORT

#undef nb_info
#define nb_info(format, args...)		func_nb_process(format, ##args)

struct {
	//int enab;
	int n;
	int conn_tail;
	char bff[100];
} nb = { 0, 0, };

void conf_print(char *bff) {
	int nbn = 0;
	char nbbff[109];
	if (!nb.conn_tail) 
		//nbn += sprintf(nbbff, "( )( d%d )", CONF_VER & DM_VER_NEWINFO_MASK);
		nbn += sprintf(nbbff, "( d%d )", CONF_VER & DM_VER_NEWINFO_MASK);
	nbn += sprintf(nbbff+nbn, "%s", nb.bff);
	printk(nbbff);
}

void func_nb_process(const char *format, ...)
{
	struct va_format vaf;
	va_list args;
	//if (!nb.enab)
	//	return;
	int nbn;
	char nbbff[100];

	va_start(args, format);
	vaf.fmt = format;
	vaf.va = &args;

	//[JJ extra-check-1-2-3] 20210617
	nbn = snprintf(&nbbff[0], sizeof(nbbff), "%pV", &vaf);
	if ((nb.n + nbn) >= 100) {
		conf_print(nb.bff);
		/*nbn = 0;
		if (!nb.conn_tail) 
			//printk("( d%d )", CONF_VER & DM_VER_NEWINFO_MASK);
			nbn += sprintf(nbbff, "( d%d )", CONF_VER & DM_VER_NEWINFO_MASK);
		//printk(nb.bff);
		nbn += sprintf(nbbff+nbn, "%s", nb.bff);
		printk(nbbff);*/
		nb.n = 0;
		nb.conn_tail = 1;
	}

	nb.n += snprintf(&nb.bff[nb.n], sizeof(nb.bff), "%pV", &vaf);
	va_end(args);

	if (nb.bff[nb.n - 1] == '\n') {
		conf_print(nb.bff);
		/*nbn = 0;
		if (!nb.conn_tail)
			//printk("( d%d )", CONF_VER & DM_VER_NEWINFO_MASK);
			nbn += sprintf(nbbff, "( d%d )", CONF_VER & DM_VER_NEWINFO_MASK);
		//printk(nb.bff);
		nbn += sprintf(nbbff+nbn, "%s", nb.bff);
		printk(nbbff);*/
		nb.n = 0;
		nb.conn_tail = 0;
	}
}
#endif

//
// Debug for the driver._nb_info.e
//

//
// Debug for the driver
//

//#ifdef DEBUG_CODE_RST_ANND_CARRI_COLLECT_H
//#endif
#if (DM9051_TST_DISP_ALL & DM9051_RELDISP_SPEED)
#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
#ifdef DM_CONF_INTERRUPT
	u32 dm_dts_spi_max_frequency(void) {
		u32 max_spi_speed;
		struct device_node *nc;
		int rc;
		if ((nc = of_find_compatible_node(NULL, NULL, DM_DM_CONF_DTS_COMPATIBLE_USAGE))) {
			rc = of_property_read_u32(nc, "spi-max-frequency", &max_spi_speed); // read to DTS, read-spi_max_speed
			if (!rc)
				return max_spi_speed;
			return 0;
		}
		return 0;
	}
	
	u32 dm_dts_interrupt_pin(void) {
		u32 value;
		struct device_node *nc;
		int rc;
		if ((nc = of_find_compatible_node(NULL, NULL, DM_DM_CONF_DTS_COMPATIBLE_USAGE))) {
			rc = of_property_read_u32_index(nc, "interrupts", 0, &value); // read to DTS index 0, interrupt-pin-num
			if (!rc)
				return value;
		}
		return 0;
	}
	u32 dm_dts_interrupt_trigger(void) {
		u32 value;
		struct device_node *nc;
		int rc;
		if ((nc = of_find_compatible_node(NULL, NULL, DM_DM_CONF_DTS_COMPATIBLE_USAGE))) {
			rc = of_property_read_u32_index(nc, "interrupts", 1, &value); // read to DTS index 1, irq-type-level
			if (!rc)
				return value;
		}
		return 0;
	}	
	/* If we need display spi-speed, we make coerce here! */
	#undef dm_msg_open
	#define dm_msg_open(ndev) dm_msg_open_receiving_dev_debug_dts_int(ndev)
	static void dm_msg_open_receiving_dev_debug_dts_int(struct net_device *ndev) {
		board_info_t *db = netdev_priv(ndev);
		struct device *dev = &db->spidev->dev;
		if (1) {
			/* Read DTS here! */
			u32 speed, pin, trigger;
			if ((speed = dm_dts_spi_max_frequency()))
				dev_info(dev, "spi-max-frequency: %d\n", speed); //INT,50000000(db->Drv_Version_speed)
			else
				dev_info(dev, " *dm9 WARN, No DTS compatible node or DTS has no spi-max-frequency\n");
			
			if ((pin = dm_dts_interrupt_pin()))
				dev_info(dev, "interrupts pin: %d\n", pin); 
			else
				dev_info(dev, " *dm9 WARN, No DTS compatible node or DTS has no interrupts pin definition\n");
				
			if ((trigger = dm_dts_interrupt_trigger()))
				dev_info(dev, "interrupts trigger: %d\n", trigger); 
			else
				dev_info(dev, " *dm9 WARN, No DTS compatible node or DTS has no interrupts trigger definition\n");
		}
		snprintf(db->DRV_VERSION, sizeof(db->DRV_VERSION), "%s_V%d.%d.%d.%s",
			DRV_PRODUCT_NAME, (DRV_VERSION_CODE >> 16 & 0xff),
			(DRV_VERSION_CODE >> 8 & 0xff), (DRV_VERSION_CODE & 0xff),
			DRV_VERSION_DATE);
		dev_info(dev, "version: %s\n", db->DRV_VERSION);
	}
#endif
#endif
#endif

//
//  All test write in 'dm9051_fifo_reset_debug'
//  All test read and display in 'dm_carrier_poll_plink_LOCK' while from link dow to up.
//

//[core-prototypes]
#define dm9051_write(db)
#define dm9051_display(db)
#define dm9051_mem_test(db)

//[essent(or opt)] in the following .. This can effect to the link-carrier
//essent(or opt)	<=essential(optional), write phy registers>

//#define dbg_spcr_phywrite(db,val)	//essent(or opt) located in this 'dm_spinb_design_ess.c' (operate inside 'dm9051_fifo_reset()')
					//so can be defined or undefined in the 'dm9051_usr7_def.h' for if need to test!
					//when defined 0x810 or 0x830 is to be determined
					//when undefined, it is toally identical to do commented in main source code (finally).
#undef dbg_spcr_phywrite
#define dbg_spcr_phywrite(db,val) dbg_spcr_phywrite_func(db,val)

void dbg_spcr_phywrite_func(board_info_t *db, int val) {
	nb_info("[dm9051] /PHYW /SPCR: %x\n", val);
	//nb_info("[dm9051 *dbgw] /PHYW /SPCR: %x\n", val);
	dm_phy_write_func(db, 20, val); //(dbg test) force-MDI or force-MDIX
}
//.#if DM9051_TEST_PHY_MDI
//.#endif
		
#undef dbg_atcr_write
#define dbg_atcr_write(db,val) dbg_atcr_write_func(db,val)
	
void dbg_atcr_write_func(board_info_t *db, u8 val) {
	nb_info("[dm9051] /REGW /ATCR: %x\n", val);
	//nb_info("[dm9051 -dbgw] /REGW /ATCR: %x\n", val);
	iow(db, DM9051_ATCR, val);
}
//.#if DM9051_TEST_ATCR
//.#endif

#if (DM9051_TST_WRFLGS & DM9051_TEST_MEM)
	#undef dm9051_mem_test
	#define dm9051_mem_test(db) dm9051_mem_test_func(db)
	
	//[debug]
	void wt_write(board_info_t *db, u8 *buf, unsigned blksize, int blknum) {
		int u;
		for (u=0;u<blknum;u++) //1,32,64
			dm9outblk(db, buf, blksize); //256
	}
	//[return] 1 succeed
	int wt_read(board_info_t *db, unsigned blksize, int blknum, u8 *operatedata) {
		int u;
		u8 sbuff[256];
		ior(db, DM_SPI_MRCMDX); /* Dummy read */
		for (u=0;u<blknum;u++){ //1,32,64
			dm9inblk(db, sbuff, blksize); //256
			if (memcmp(operatedata, sbuff, blksize)){ //256
				//nFail++;
				//break;
				return 0;
			}
		}
		//if (u==blknum) //1,32,64
		//	nOK++;
		return 1;
	}

	void dm9051_mem_test_func(board_info_t *db)
	{
		//[.] WRITE_READ_TEST_LOCK: dm_opencode_writeread_test_lock()
		//ADDR_LOCK_HEAD_ESSENTIAL(db);
		//if (1) {
			#define MAX_MEM_BLOCKS	/*(64)*/ (32) //Test(32)
			int n, i, nOK, nFail, resE;
			u8 operatedata[256];
			
			//[init]
			for (i=0;i<256;i++)
				operatedata[i] = (u8) i;
				
			//[SetMode]
			iow(db, DM9051_IMR, 0); //"iow(db, DM9051_IMR, db->imr_all);
			
			//[Test.InBegin].s
			iow(db, DM9051_NCR, NCR_RST); //NCR_RESET for reset RX/TX pointer is essensial while test before first NCR_RESET
			//[Test.InBegin].e
			
			for (i=0; i<MAX_MEM_BLOCKS; ){
				//[Write]
				wt_write(db, operatedata, 256, 1);
				i++;
			}
						
			nOK = 0; nFail = 0;
			for (n=0;n<125; ) { //36, 1000
				//[Test.Add].s
				iow(db, DM9051_NCR, NCR_RST); //= ncr_reg_reset(db);
				//[Test.Add].e
				resE = 0;
				for (i=0;i<MAX_MEM_BLOCKS; ){ //(max all_of_the_memory)
					
						//.[Write] //.wt_write(db, operatedata, 256, 1);
						//[Read]
						if (!wt_read(db, 256, 1, operatedata)){
							resE = 1;
							break;
						}
						//[Next]
						i++;
				}
				if (resE)
					nFail++;
				else
					nOK++;
					
				n++;
				if ((n < 100) && !(n%20))
					nb_info("[DM9051] MEMW/MEMR: nOK %3d, nFail %d\n", nOK, nFail);
			}
			nb_info("[DM9051] MEMW/MEMR: nOK %3d, nFail %d\n", nOK, nFail);
		//}
		//ADDR_LOCK_TAIL_ESSENTIAL(db);
		//[.] RECOVER-BACK
		//dm_opencode_lock(dev, db);
	}
#endif

//opt in the following .. This can used in the link-carrier
//opt			<=optional, read phy registers>
	
#define	intcr_wr_disp(db)
#define	intcr_reg_disp(db)

	#undef intcr_wr_disp
	#define	intcr_wr_disp(db) nb_info("[dm9051] /IOW /INTCR: %x\n", INTCR_POL_LOW); //not used, any more!
	
	#undef intcr_reg_disp
	#define	intcr_reg_disp(db) intcr_reg_disp_func(db)

	void intcr_reg_disp_func(board_info_t *db) {
		u8 intcr = ior(db, DM9051_INTCR);
		if (intcr & 1)
			nb_info("[dm9051] ior active_low/INTCR: %02x\n", intcr);
		else
			nb_info("[dm9051] ior active_high/INTCR: %02x\n", intcr);
	}

// called from dm9051.c
#if (DM9051_TST_WRFLGS & DM9051_TESTDISP_LCR) 
#undef	ledcr_wr_disp
#define	ledcr_wr_disp(db) nb_info("[dm9051] /IOW /LCR: %x\n", db->lcr_all);
#endif

// called from dm9051.c
#if (DM9051_TST_DISP_ALL & DM9051_TESTDISP_LCR) 
#undef	ledcr_reg_disp
#define	ledcr_reg_disp(db) ledcr_reg_disp_func(db)
void ledcr_reg_disp_func(board_info_t *db) { //Read and Display LEDMode Control Register
	u8 lcr = ior(db, DM9051_LMCR);
	if (lcr == 0x81)
		nb_info("[dm9051] ior mode_1/LCR: %02x\n", lcr); //LMCR:
	else
		nb_info("[dm9051] ior mode_x/LCR: %02x\n", lcr);
}
#endif
	
// called from dm9051.c
#if (DM9051_TST_DISP_ALL & DM9051_TEST_SPIBCR) 
#undef dbg_spibcr_peek
#define dbg_spibcr_peek(db) dbg_spibcr_peek_func(db)
void dbg_spibcr_peek_func(board_info_t *db) {
	u8 sbcr = ior(db, DM9051_SPIBCR);
	if ((sbcr & 0x60) == 0x60)
		nb_info("[dm9051] ior 8mA/SBCR: %x\n", sbcr); //DM9051_SBCR(=DM9051_SPIBCR)[temp coding]
	else if ((sbcr & 0x60) == 0x40)
		nb_info("[dm9051] ior 6mA/SBCR: %x\n", sbcr); //DM9051_SBCR(=DM9051_SPIBCR)[temp coding]
	else if ((sbcr & 0x60) == 0x20)
		nb_info("[dm9051] ior 4mA/SBCR: %x\n", sbcr);
	else if ((sbcr & 0x60) == 0x00)
		nb_info("[dm9051] ior 2mA/SBCR: %x\n", sbcr);
	else
		nb_info("[dm9051] ior EC/SBCR: %x\n", sbcr); //ErrChk
}
#endif

#undef dbg_spcr_phyread
#define dbg_spcr_phyread(db) dbg_spcr_phyread_func(db)
	
void dbg_spcr_phyread_func(board_info_t *db) {
	int val = dm_phy_read_func(db, 20);
	if (val & 0x80)
		nb_info("[dm9051] phyr MDIX/SPCR: %x\n", val); //(dbg Test)
		//nb_info("[dm9051 *dbgr] phyr MDIX/SPCR: %x\n", val); //(dbg Test)
	else
		nb_info("[dm9051] phyr MDI/SPCR: %x\n", val); //(dbg Test)
		//nb_info("[dm9051 *dbgr] phyr MDI/SPCR: %x\n", val); //(dbg Test)
}
	
#undef dbg_atcr_read
#define dbg_atcr_read(db) dbg_atcr_read_func(db)

void dbg_atcr_read_func(board_info_t *db) {
	u8 atcr = ior(db, DM9051_ATCR);
	nb_info("[dm9051] ior AUTO_TX/ATCR: %x\n", atcr); //(dbg Test)
	//nb_info("[dm9051 -dbgr] ior AUTO_TX/ATCR: %x\n", atcr); //(dbg Test)
}

#undef dm9051_display
#define dm9051_display(db) dm9051_display_debug(db)
	
void dm9051_display_debug(board_info_t *db)
{
	#if (DM9051_TST_DISP_ALL & DM9051_TEST_PHY_MDI)
		dbg_spcr_phyread(db); //(test) phy_read (test) force mode
	#endif
	#if (DM9051_TST_DISP_ALL & DM9051_TEST_ATCR)
		dbg_atcr_read(db);
	#endif
		ledcr_reg_disp(db);
	#if (DM9051_TST_DISP_ALL & DM9051_TEST_SPIBCR)
		dbg_spibcr_peek(db); //peek show the power-on default
	#endif
	#if (DM9051_TST_DISP_ALL & DM9051_TESTDISP_INTCR) && defined DM_CONF_INTERRUPT
		intcr_reg_disp(db);
	#endif
}

#undef dm9051_write
#define dm9051_write(db) dm9051_write_debug(db)

void dm9051_write_debug(board_info_t *db)
{
	#if (DM9051_TST_WRFLGS & DM9051_TEST_PHY_MDI)
		dbg_spcr_phywrite(db, 0x810); //(test) phy_write (810) force mode
	#endif
	#if (DM9051_TST_WRFLGS & DM9051_TEST_ATCR)
		dbg_atcr_write(db,ATCR_AUTO_TX); //[inside func_redesign_carrier.c] //ATCR_AUTO_TX= 0x80
	#endif
	#if (DM9051_TST_WRFLGS & DM9051_TESTDISP_INTCR) && defined DM_CONF_INTERRUPT
		intcr_wr_disp(db);
	#endif
}

#if (DM9051_TST_WRFLGS & 0xFFFF)	//(DM9051_TST_WRFLGS & _ALL_MASK)	
	#undef dm9051_fifo_reset
	#define dm9051_fifo_reset(stat,hstr,db) dm9051_fifo_reset_debug(stat,hstr,db)
	
	void dm9051_fifo_reset_debug(u8 state, u8 *hstr, board_info_t *db)
	{
		if (!db->bC.DO_FIFO_RST_counter)
			dm9051_mem_test(db);
		++db->bC.DO_FIFO_RST_counter;
		dm9051_reset(db);
		dm9051_write(db);
	}
#endif


//(Step0)
//[While has defined, Also need append the below code to work it out.]
#ifdef KT_LINK_DEBUG_READ

	#undef	dm_carrier_init
	#define	dm_carrier_init(db)					dm_mii_init_plink(db)
	#undef	dm_carrier_poll
	#define	dm_carrier_poll(db)					dm_mii_on_off_plink_LOCK(db)
		
static void dm_disp_func(board_info_t *db, int linkBool){
	//struct net_device *ndev = db->ndev;
	struct net_device *ndev = db->ndev;
	if (linkBool) {
		dm9051_display(db);
	} else {
		dbg_spcr_phyread(db); //only one
	}
	nb_info("[dm9051] KT Link Status is: %d\n", linkBool);
	netdev_info(ndev, "KT Link Status is: %d\n", linkBool);
}

	static void dm_mii_init_plink(board_info_t *db)
	{
		mii_check_link(&db->mii);
		db -> p.ec.linkBool = 0;
	}
	static void dm_mii_on_off_plink_LOCK(board_info_t *db)
	{
		struct net_device *dev = db->ndev;
		int link;
		ADDR_LOCK_HEAD_ESSENTIAL(db);
		mii_check_link(&db->mii);
		link = (int) netif_carrier_ok(dev);
		if (db->p.ec.linkBool != link) {
			//>ADDR_LOCK_HEAD_ESSENTIAL(db);
			dm_disp_func(db, link);
			db -> p.ec.linkBool = link;
			//>ADDR_LOCK_TAIL_ESSENTIAL(db);
		}
		ADDR_LOCK_TAIL_ESSENTIAL(db);
	}

//(Step1)
#elif defined DM_CONF_EXPLICITY_LINK_DEBUG_READ

	//#undef	polyn_mii_check_link_init
	//#define	polyn_mii_check_link_init(db)			explicity_init_on_off_carrier(db)
	
	//#undef	polyn_mii_check_link
	//#define	polyn_mii_check_link(db)			exlicity_on_off_carrier_lock(db)
	
	#undef	dm_carrier_init
	#define	dm_carrier_init(db)					explicity_init_on_off_carrier(db)
	#undef	dm_carrier_poll
	#define	dm_carrier_poll(db)					exlicity_on_off_carrier_lock(db)

static void dm_carrier_func(board_info_t *db, int linkBool){
	struct net_device *ndev = db -> ndev;
	//.char *hstr = "explicity_chk";
	//.if (netif_running(ndev)) {
		if (linkBool){
			netif_carrier_on(ndev);
			//dm_msg_carrier_on(hstr); //printk("[dm9 carrier].[%s] netif_carrier_on\n", hstr);
		}else{
			netif_carrier_off(ndev);
			//dm_msg_carrier_off(hstr); //printk("[dm9 carrier].[%s] netif_carrier_off\n", hstr);
		}
	//.}
}
static void dm_disp_func(board_info_t *db, int linkBool){
	struct net_device *ndev = db->ndev;
	if (linkBool){
		//.printk("dm9 no [netif_info] up\n");
		//.netif_info(db, link, ndev, "link is up\n");
		dm9051_display(db);
	}else{
		//.printk("dm9 no [netif_info] down\n");
		//.netif_info(db, link, ndev, "link is down\n");
		dbg_spcr_phyread(db); //only one
	}
	nb_info("[dm9051] Link Status is: %d\n", linkBool);
	netdev_info(ndev, "Link Status is: %d\n", linkBool);
}
	
static void explicity_init_on_off_carrier(board_info_t *db)
{
	mii_check_link(&db->mii);
	db -> p.ec.linkBool = 0;
}

static void exlicity_on_off_carrier_lock(board_info_t *db)
{
	//struct net_device *dev = db -> ndev;
	unsigned nsr;
	int link;
	ADDR_LOCK_HEAD_ESSENTIAL(db);
	nsr = ior(db, DM9051_NSR);
	nsr = ior(db, DM9051_NSR);
	link = !!(nsr & 0x40);
	if (db->p.ec.linkBool != link) {
		dm_carrier_func(db, link);
		dm_disp_func(db, link);
		db -> p.ec.linkBool = link;
	}
	ADDR_LOCK_TAIL_ESSENTIAL(db);
}
#endif
