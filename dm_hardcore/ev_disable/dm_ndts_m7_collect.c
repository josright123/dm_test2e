//[implementation _ndts_m7.c]

//1.[implementation (non) dts7.h]	
#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE //&& 0 /*= !test */
	/* this no need register board information ! */
#else
	//#undef OP_DTS_TYPE_STR
	#define OP_DTS_TYPE_STR			"NDTS"
	//#undef dm_msg_probe_enter
	//#define dm_msg_probe_enter()		printk("[dm9 spi_register board_info] spi%d.%d speed %d\n", 
	//						dm9051_spi_board_devs[0].bus_num, 
	//						dm9051_spi_board_devs[0].chip_select, 
	//						dm9051_spi_board_devs[0].max_speed_hz)
#endif

#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE //&& 0 /*= !test */
   /* this no need register board information ! */
#else
	#define DRV_MAX_SPEED_HZ		DM_CONF_MAX_SPEED_HZ
	#define DRV_SPI_BUS_NUMBER		DM_CONF_SPI_BUS_NUMBER
	#define DRV_SPI_CHIP_SELECT		DM_CONF_SPI_CHIP_SELECT

	/* While NonDTS modle, made it REGISTER spi max speed in the NDTS source file*/	
	static struct spi_board_info dm9051_spi_board_devs[] __initdata = {
		[0] = {
			.modalias = "dm9051",
			.max_speed_hz = DRV_MAX_SPEED_HZ,
			.bus_num = DRV_SPI_BUS_NUMBER,
			.chip_select = DRV_SPI_CHIP_SELECT,
			.mode = SPI_MODE_0,
		#ifdef DM_CONF_INTERRUPT
			.irq = DM_NCONF_NDTS_IRQPIN,
		#endif
		},
	};
	
	#ifdef DM_DEVEL_MODULE
		/* Joseph: register/unregister */
		static struct spi_device *spi_device;
		static void dm9051_device_spi_delete(struct spi_master *master, unsigned cs)
		{
			struct device *dev;
			char str[32];
			snprintf(str, sizeof(str), "%s.%u", dev_name(& master -> dev), cs);
			dev = bus_find_device_by_name(& spi_bus_type, NULL, str);
			if (dev) {
				device_del(dev);
			}
		}

		/* Joseph: register/unregister */
		static int dm_spi_register_board_info(struct spi_board_info *spibd, unsigned n)
		{
			/* Joseph_20151030: 'n' is always 1, ARRAY_SIZE(table) is 1 table-item in this design  */
			struct spi_master *master;
			master = spi_busnum_to_master(spibd -> bus_num);
			if (!master) {
				pr_err(DRVNAME_9051 ":  spi_busnum_to_master(%d) returned NULL\n",
				       spibd -> bus_num);
				return -EINVAL;
			}
			/* make sure it's available */
			dm9051_device_spi_delete(master, spibd -> chip_select);
			spi_device = spi_new_device(master, spibd);
			put_device(& master -> dev);
			if (!spi_device) {
				pr_err(DRVNAME_9051 ":    spi_new_device() returned NULL\n");
				return -EPERM;
			}
			return 0;
		}
		static void dm_spi_unregister_board_info(void)
		{
			if (spi_device) {
				device_del(& spi_device -> dev);
				kfree(spi_device);
			}
		}
	#endif
#endif

// *-----------------------------------------------------------------------------------
// *                                         [NDTS]
// *                                         struct spi_board_info dm9_spi_bd_devs[]
// *-----------------------------------------------------------------------------------
// .INT 
// .MOD  [DTS][INT].devel_mod                [NDTS][INT].devel_mod
// *
// *                                         dm_spi_regstr_bd_inf/dm_spi_unregstr_bd_inf
// *     spi_register_dr/dm9_init(void)pair  spi_register_dr/dm9_init(void)pair
// *
// .NMOD [DTS][INT].N_mod_KT                 [NDTS][INT].N_mod_KT
// *
// *                                         spi_register_board_inf
// *       O                                 spi_register_dr/dm9_init(void)pair
// *
// *Tips: INT and POLL are almost same, only INT need tell in the spi_board_inf not only
// *       .max_speed_hz/.bus_num/.chip_select/.mode but also .irq
// *-----------------------------------------------------------------------------------
// .POLL
// .MOD  [DTS][POLL].devel_mod               [NDTS][POLL].devel_mod
// *
// *                                         dm_spi_regstr_bd_inf/dm_spi_unregstr_bd_inf
// *     spi_register_dr/dm9_init(void)pair  spi_register_dr/dm9_init(void)pair
// *
// .NMOD [DTS][POLL].N_mod_KT                [NDTS][POLL].N_mod_KT
// *
// *                                         spi_register_board_inf
// *       O                                 spi_register_dr/dm9051_init(void)pair
// *

#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE //&& 0 /*= !test */
   /* this no need register board information ! */
#else
	#ifdef DM_DEVEL_MODULE
	//.....sdfgegr..
		//dm9051:  spi_busnum_to_master(0) returned NULL
		// need : #dtparam=spi=on ,un-comment
		#define	conf_spi_board()	dm_spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs))
		#define	unconf_spi_board()	dm_spi_unregister_board_info()
	#else
	.......%%%R%76r56t78eq8983*******
		extern int spi_register_board_info(struct spi_board_info const * info,
		  unsigned n); //#define CONFIG_SPI, used in local static-make for dm9051.o
		  
		#define	conf_spi_board()	spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs))
		#define	unconf_spi_board()	//empty
	#endif
#endif

//[All below are _int_ndts or _poll_ndts, or (int_dts_mod), poll_dts_mod]
#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE //&& 0 /*= !test */
	/* this no need register board information ! */
#else
	//1.[implementation (non) dts7.h]
	//[below are for both _int_ndts or _poll_ndts]
	#define	RET_DM9051_INIT() printk("\n"); \
				  printk("module: dm9051_init\n")
	#define	DM9051_CLEANUP()  printk("[dm9 cleanup]\n")

	#undef	module_spi_driver
	#define	module_spi_driver(dmdrvr)		\
		static int __init			\
		dm9051_init(void)			\
		{					\
			RET_DM9051_INIT();		\
			conf_spi_board();		\
			return spi_register_driver(&dmdrvr); \
		}					\
		static void dm9051_cleanup(void)	\
		{					\
			DM9051_CLEANUP();		\
			unconf_spi_board();		\
			spi_unregister_driver(&dmdrvr);	\
		}					\
		module_init(dm9051_init);		\
		module_exit(dm9051_cleanup)
#endif

//#if defined _DM_DEVEL_NDTS || defined _DM_DEVEL_MODULE
//[def user]
//1.[implementation (non) dts7.h] or 
//2.[implementation (has) dts7.h and (has) devel_module]

//[old]
	/*(1)#if _DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
	
		#define	conf_spi_board()	//empty
		#define	unconf_spi_board()	//empty
	
	#elif defined DM_DEVEL_MODULE
	
		#define	conf_spi_board()	dm_spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs))
		#define	unconf_spi_board()	dm_spi_unregister_board_info()
		
	#else //_NDTS && !_DM_DEVEL_MODULE
	
		extern int spi_register_board_info(struct spi_board_info const * info,
		  unsigned n); //#define CONFIG_SPI, used in local static-make for dm9051.o
		  
		#define	conf_spi_board()	spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs))
		#define	unconf_spi_board()	//empty
		
	#endif*/
	
	/*(2)void conf_spi_board(void)
	{
	#if _DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
	   // this no need register board information !
	#else
	   //.printk("[dm9 spi_register board_info] spi%d.%d speed %d\n",
		//.dm9051_spi_board_devs[0].bus_num,
		//.dm9051_spi_board_devs[0].chip_select,
		//.dm9051_spi_board_devs[0].max_speed_hz);
	    #ifdef DM_DEVEL_MODULE
		_dm_spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs));
	    #else
		extern int spi_register_board_info(struct spi_board_info const * info,
		  unsigned n); //#define CONFIG_SPI, used in local static-make for dm9051.o
		  //static-make for dm9051.o, looked can't link succeed, because leaked of kernel CONFIG_SPI lib. Get below:
		  //ERROR: modpost: "spi_register_board_info" [/home/pi/pi/github_202108/onload_dm9051/davicom_do_format/dm9051.ko] undefined!
		_spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs));
	    #endif
	#endif
	}
	void unconf_spi_board(void)
	{
	#if _DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
	   // this no need register board information ! 
	#else
	  #ifdef DM_DEVEL_MODULE
		_dm_spi_unregister_board_info();
	  #else
		//no this way unregister?? v.s. spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs));
	  #endif
	#endif
	}*/
//#endif //(defined _DM_DEVEL_MODULE || defined _DM_DEVEL_NDTS)
