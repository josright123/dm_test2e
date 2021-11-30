//[All below are _int_ndts]
#ifdef DM_CONF_INTERRUPT
#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE //&& 0 /*= !test */
   /* this no for this dts kind code, in this block ! */
#else
	//#undef OP_DTS_TYPE_STR
	#define OP_DTS_TYPE_STR				"NDTS"
	//#undef OP_TYPE_STR
	#define OP_TYPE_STR				"INT"	//[when interrupt]
	
	#undef dm_msg_open
	//#define dm_msg_open(db,dname) printk("dm9 [dm_open][%s][NDTS][INT]\n", dname)
	#define dm_msg_open(ndev) dm_msg_open_receiving_dev_ndts_int(ndev)
	static void dm_msg_open_receiving_dev_ndts_int(struct net_device *ndev) {
			//struct device *dev = ndev->dev.parent
			board_info_t *db = netdev_priv(ndev);
			struct device *dev = &db->spidev->dev;
			if (1) {
				/* Get Data here! */
				// cause by static struct spi_board_info xxx_board_devs[] __initdata, so driver crashed
				//dev_info(dev, "spi max_speed_hz: %d\n", dm9051_spi_board_devs[0].max_speed_hz); --- "! PC is at dm9051_open+0x48/0x1cc [dm9051]"
				dev_info(dev, "spi max_speed_hz: %d\n", DRV_MAX_SPEED_HZ); //INT,50000000(db->Drv_Version_speed)
				dev_info(dev, "gpioPIN_to_irq: %d\n", DM_NCONF_NDTS_IRQPIN);
			}
			snprintf(db->DRV_VERSION, sizeof(db->DRV_VERSION), "%s_V%d.%d.%d.%s -d%d (%s)(%s)", 
				DRV_PRODUCT_NAME, (DRV_VERSION_CODE >> 16 & 0xff),
				(DRV_VERSION_CODE >> 8 & 0xff), (DRV_VERSION_CODE & 0xff),
				DRV_VERSION_DATE, CONF_VER & DM_VER_NEWINFO_MASK, OP_DTS_TYPE_STR, OP_TYPE_STR);
			dev_info(dev, "version: %s\n", db->DRV_VERSION); //printk
			printk("( dm9 )[dm_open][%s][NDTS][INT]\n", ndev->name);
	}
	
	static int gpio_config_gpio_to_irq(board_info_t *db) { //_DM_DEVEL_NDTS_' local
		int irq;
		unsigned gpio_num = DM_NCONF_NDTS_IRQPIN; //Get_IntPinNun(db);
		if (gpio_request(gpio_num, GPIO_ANY_GPIO_DESC)) {
			printk("---------------------------------------------\n");
			printk("dm9051 gpio_request Error! pin %d, desc %s\n", gpio_num, GPIO_ANY_GPIO_DESC);
			printk("---------------------------------------------\n");
			#if 1
			printk("manually in 'conf_ver.h', in-developping-way-trouble, recovery by coerce _free\n");
			gpio_free(DM_NCONF_NDTS_IRQPIN); //manually in 'conf_ver.h', in-developping-way-trouble, recovery by coerce _free
			#endif
			return -1;
		}
		gpio_direction_input(gpio_num);
		irq = gpio_to_irq(gpio_num);  //exp: enum gpio_to_irq( 17) = 187
		if (irq > 0) { // jj:enum
			return irq;
		}
		printk("dm9051 failed to get irq_no, %d\n", irq);
		return irq;
	}
#endif

#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE //&& 0 /*= !test */
   /* this no for this dts kind code, in this block ! */
#else
	#undef intcr_reg_setval
	#define intcr_reg_setval(db) iow(db, DM9051_INTCR, DM_NCONF_INTCR_POL)	// INTCR, export that hook

	#undef dm_msg_open_done
	#define dm_msg_open_done(dev) def_msg_open_done_ndts(dev)	//[when ndts]
	#define def_msg_open_done_ndts(dev) printk("( dm9 )[dm_open][%s] %pM, NDTS irqno %d %s rx_threaded_irq-start\n", \
			dev->name, dev->dev_addr, dev->irq, DM_NCONF_POL_STR)

	//#undef DM_INFO_GET
	#define	DM_INFO_GET(db) dm_info_get_int_ndts(db)
	
	void dm_info_get_int_ndts(board_info_t *db) {
		//if (DM_NCONF_IRQF_TRIGGER == IIRQ_TYPE_EDGE_RISING || DM_NCONF_IRQF_TRIGGER == IIRQ_TYPE_LEVEL_HIGH)
		//	Set_IrqType(db, IIRQ_TYPE_LEVEL_HIGH);
		//else
		//	Set_IrqType(db, IIRQ_TYPE_LEVEL_LOW);
		//Set_IntPinNun(db, DM_NCONF_NDTS_IRQPIN); //db->_gpio_int_pin= DM_NCONF_NDTS_IRQNUM;
	}
	
	#undef dm_opencode_receiving
	#define dm_opencode_receiving(dev,db)	dm_opencode_receiving_ndts(dev,db)	//[for when ndts(INT), can in case be tokeover!]

	int dm_opencode_receiving_ndts(struct net_device *dev, board_info_t *db)
	{
		struct spi_device *spi = db->spidev;
		int ret;
		spi->irq = gpio_config_gpio_to_irq(db);
		if (spi->irq <= 0)
			return -1;
		//dm_msg_open_receiving_dev_ndts_int(dev->name);
		dev->irq = spi->irq; //in probe?
		//unsigned long val_trigger;
		//val_trigger = Get_IrqFlags(db); request_threaded_irq(,val_trigger  //int_polarity_disp(val_trigger);
		ret = request_threaded_irq(spi->irq, NULL, dm9051_rx_threaded_irq,
					DM_NCONF_IRQF_TRIGGER, dev->name, db); //="int_get_begin()"
		IMR_ENABLE_LOCK_ESSENTIAL(db);
		init_sched_phy(db); //changed. org.dm_sched_phy(db); //_sched_start
		if (!ret)
			dm_msg_open_done(dev);
		return 0; //'ret'
	}

	#undef dm_stopcode_release
	#define dm_stopcode_release(db) dm_stopcode_release_ndts(db)
	
	void dm_stopcode_release_ndts(board_info_t *db)	
	{
		//int_gpiopinfree_disp(db->gpio_int_pin); (opt)
		gpio_free(DM_NCONF_NDTS_IRQPIN); //manually in 'conf_ver.h', old=(db->gpio_int_pin)
		//int_irqnofree_disp(irq); (opt)
		free_irq(db->spidev->irq, db); //irq_config_free(db, db->spidev->irq); //can also contain the 'gpio_config_free'(_DRV_CONF_INTERRUPT_IRQ...)
		cancel_delayed_work_sync(&db->phy_poll);
		cancel_delayed_work_sync(&db->rxctrl_work);
		cancel_delayed_work_sync(&db->setmac_work);
		cancel_delayed_work_sync(&db->tx_work);
	}
#endif
#endif
