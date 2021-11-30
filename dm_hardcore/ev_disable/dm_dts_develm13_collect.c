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

//[All below are (int_dts_mod), poll_dts_mod]
#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE //&& 0 /*= !test */
	#ifdef DM_DEVEL_MODULE
		.....%%76r56t78eq8983*******
		//2.[implementation (has) dts7.h and (has) devel_module]
		#undef	module_spi_driver
		#define	module_spi_driver(dmdrvr)		\
			static int __init			\
			dm9051_init(void)			\
			{					\
				return spi_register_driver(&dmdrvr); \
			}					\
			static void dm9051_cleanup(void)	\
			{					\
				spi_unregister_driver(&dmdrvr);	\
			}					\
			module_init(dm9051_init);		\
			module_exit(dm9051_cleanup)
	#endif
#endif
