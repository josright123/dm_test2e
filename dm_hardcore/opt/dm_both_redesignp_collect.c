//[POLL]

// .  [DTS][POLL]                          [NDTS][POLL]
// .     dm_info_get_poll_dts(db)          .empty again
//
// * [The _dm_opencode_receiving() in the source code compiler options.]
// *
// .  [DTS][INT]                           [NDTS][INT]
// .     dm_opencode_receiving_()           dm_opencode_receiving_ndts() //imply as (INT, can be overtoke!)
// *     //DEFAULT                         //ndts
// .  [DTS][POLL]                          [NDTS][POLL]
// .     dm_opencode_receiving_dts_poll()  dm_opencode_receiving_ndts_poll()
// *     //poll                            //poll
// *     //poll has only one implement for both (dts)&(ndts)

#ifndef DM_CONF_INTERRUPT //!DM_CONF_INTERRUPT

//#include "func_redesignp_subroutine.h" //#include "supporting/opt/dm_both_redesignp_collect.h"

//#undef OP_TYPE_STR
#define OP_TYPE_STR				"POLL"	//[when polling]
#undef dm_msg_open_done
#define dm_msg_open_done(dev)			def_msg_open_done_poll(dev)	//[when polling you can make it printed different]
#define def_msg_open_done_poll(dev)		printk("( dm9 )[dm_open][%s][MAC] %pM rx_delay_work-scheduling\n", dev->name, dev->dev_addr)
#define dm_sched_start(db)			schedule_delayed_work(&db->rx_work,0)
#define dm_sched_cyc(db,n)			schedule_delayed_work(&db->rx_work,n)

#ifdef DM_DEVEL_NDTS
		//#undef OP_DTS_TYPE_STR
		#define OP_DTS_TYPE_STR			"NDTS"
		
		//for ndts here!
		#undef dm_msg_open
		//#define dm_msg_open(dname) printk("dm9 [dm_open][%s][NDTS][POLL]\n", dname);
		#define dm_msg_open(ndev) dm_msg_open_ndts_poll(ndev)
		static void dm_msg_open_ndts_poll(struct net_device *ndev) {
			board_info_t *db = netdev_priv(ndev);
			struct device *dev = &db->spidev->dev;
			if (1) {
				/* Get Data here! */
				// cause by static struct spi_board_info xxx_board_devs[] __initdata, so driver crashed
				//dev_info(dev, "spi max_speed_hz: %d\n", dm9051_spi_board_devs[0].max_speed_hz); --- "PC is at dm9051_open+0x30/0x1b8 [dm9051]"
				dev_info(dev, "spi max_speed_hz: %d\n", DRV_MAX_SPEED_HZ); //POLL,25000000(db->Drv_Version_speed)
			}
			snprintf(db->DRV_VERSION, sizeof(db->DRV_VERSION), "%s_V%d.%d.%d.%s -d%d (%s)(%s)", 
				DRV_PRODUCT_NAME, (DRV_VERSION_CODE >> 16 & 0xff),
				(DRV_VERSION_CODE >> 8 & 0xff), (DRV_VERSION_CODE & 0xff),
				DRV_VERSION_DATE, CONF_VER & DM_VER_NEWINFO_MASK, OP_DTS_TYPE_STR, OP_TYPE_STR);
			printk("version: %s\n", db->DRV_VERSION);
			printk("( dm9 )[dm_open][%s][NDTS][POLL]\n", ndev->name);
		}
		
		//#undef DM_INFO_GET
		#define	DM_INFO_GET(db)	//_poll_ndts

		#undef dm_opencode_receiving
		#define dm_opencode_receiving(dev,db)	dm_opencode_receiving_ndts_poll(dev,db)	//[when polling]

		int dm_opencode_receiving_ndts_poll(struct net_device *dev, board_info_t *db){
			//printk("dm9 [dm_open][%s][NDTS][POLL] sched_start()\n", dev->name);
			dm_sched_start(db); //Finally, start the delay work, to be the last calling, for you can not read/wrie dm9051 register since poling schedule work has began!
			dm_msg_open_done(dev);
			return 0;
		}
#endif

#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
		u32 dm_dts_spi_max_frequency(struct device_node *nc) {
			u32 max_spi_speed;
			int rc = of_property_read_u32(nc, "spi-max-frequency", &max_spi_speed); // read to DTS, read-spi_max_speed
			if (!rc)
				return max_spi_speed;
			return 0;
		}
		/*void dm_info_get_max_spi_speed(struct device_node *nc) {
			u32 max_spi_speed;
			int rc = of_property_read_u32(nc, "spi-max-frequency", &max_spi_speed); // read to DTS, read-spi_max_speed
			if (rc) {
				printk("[ *dm9 WARN ] %s DTS has no spi-max-frequency\n", nc->full_name);
				return;
			}
			printk("dm9 [dts][%s] spi-max-frequency[= %d]\n", nc->full_name, max_spi_speed);
		}*/
		/*void dm_info_get_irq_type_level(board_info_t *db, struct device_node *nc) {
			u32 value;
			int rc = of_property_read_u32_index(nc, "interrupts", 1, &value); // read to DTS index 1, irq-type-level
			if (rc) {
				printk("[ *dm9 WARN ] DTS has no interrupts-polarity, interrupts trigger type unknow\n");
				//Set_IrqType(db, IIRQ_TYPE_LEVEL_LOW);
				return;
			}
			//Set_IrqType(db, (u8) value);
		}
		void dm_info_get_gpio_int_pin(board_info_t *db, struct device_node *nc) {
			u32 value;
			int rc = of_property_read_u32_index(nc, "interrupts", 0, &value); // read to DTS index 0, interrupt-pin-num
			if (rc) {
				printk("[ *dm9 WARN ] DTS has no interrupts-gpio-pin, interrupts gpio-pin unknow\n");
				return;
			}
			//Set_IntPinNun(db, (int) value);
		}*/

		//#undef OP_DTS_TYPE_STR
		#define OP_DTS_TYPE_STR			"DTS"
	
		//for dts here!
		#undef dm_msg_open
		//#define dm_msg_open(dname) printk("dm9 [dm_open][%s][DTS][POLL]\n", dname);
		#define dm_msg_open(ndev) dm_msg_open_dts_poll(ndev)
		static void dm_msg_open_dts_poll(struct net_device *ndev) {
			board_info_t *db = netdev_priv(ndev);
			struct device *dev = &db->spidev->dev;
			if (1) {
				/* Read DTS here! */
				struct device_node *nc = of_find_compatible_node(NULL, NULL, DM_DM_CONF_DTS_COMPATIBLE_USAGE);
				if (nc) {
					u32 speed;
					if ((speed = dm_dts_spi_max_frequency(nc)))
						dev_info(dev, "spi-max-frequency: %d\n", speed); //POLL,25000000(db->Drv_Version_speed)
					else
						dev_info(dev, " *dm9 WARN, DTS has no spi-max-frequency\n");
				} 
				else
					dev_info(dev, " *dm9 WARN, No DTS compatible node, spi speed unknow\n");
			}
			snprintf(db->DRV_VERSION, sizeof(db->DRV_VERSION), "%s_V%d.%d.%d.%s -d%d (%s)(%s)", 
				DRV_PRODUCT_NAME, (DRV_VERSION_CODE >> 16 & 0xff),
				(DRV_VERSION_CODE >> 8 & 0xff), (DRV_VERSION_CODE & 0xff),
				DRV_VERSION_DATE, CONF_VER & DM_VER_NEWINFO_MASK, OP_DTS_TYPE_STR, OP_TYPE_STR);
			printk("version: %s\n", db->DRV_VERSION);
			printk("dm9 [dm_open][%s][DTS][POLL]\n", ndev->name);
		}
		
		//#undef DM_INFO_GET
		#define	DM_INFO_GET(db)	dm_info_get_poll_dts(db)
		void dm_info_get_poll_dts(board_info_t *db) {
			/*struct device_node *nc = of_find_compatible_node(NULL, NULL, DM_DM_CONF_DTS_COMPATIBLE_USAGE);
			if (nc)
				dm_info_get_max_spi_speed(nc);
			else
				printk("[ *dm9 WARN ] No DTS compatible node, spi speed unknow\n");*/
			u32 speed;
			struct device_node *nc;
			if (!(nc = of_find_compatible_node(NULL, NULL, DM_DM_CONF_DTS_COMPATIBLE_USAGE))){
				printk("[ *dm9 WARN ] No DTS compatible node, spi speed unknow\n");
				return;
			}
			if ((speed = dm_dts_spi_max_frequency(nc)))
				printk("dm9 [dts][%s] spi-max-frequency[= %d]\n", nc->full_name, speed);
			else
				printk("[ *dm9 WARN ][%s] DTS has no spi-max-frequency\n", nc->full_name);
		}

		#undef dm_opencode_receiving
		#define dm_opencode_receiving(dev,db)	dm_opencode_receiving_dts_poll(dev,db)	//[when polling]

		int dm_opencode_receiving_dts_poll(struct net_device *dev, board_info_t *db){
			//printk("dm9 [dm_open][%s][DTS][POLL] sched_start()\n", dev->name);
			dm_sched_start(db); //Finally, start the delay work, to be the last calling, for you can not read/wrie dm9051 register since poling schedule work has began!
			dm_msg_open_done(dev);
			return 0;
		}
#endif
#endif //!DM_CONF_INTERRUPT
