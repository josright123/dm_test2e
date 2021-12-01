//[POLL]
//[All below are _poll_dts and/or _poll_ndts]
#ifndef DM_CONF_INTERRUPT //!DM_CONF_INTERRUPT
	
static int work_ss_cycle_lock(board_info_t *db)    //POLL's local
{
	int nAllSum = 0;
	int nTx, nRx;
	ADDR_LOCK_HEAD_ESSENTIAL(db); //rx_mutex_head(db);  //mutex_lock(..
	do {
		nTx = dm9051_send(db);
		nRx = dm9051_lrx(db);
		nAllSum += nTx + nRx;
	} while (nTx || nRx);
	ADDR_LOCK_TAIL_ESSENTIAL(db); //rx_mutex_tail(db); //mutex_unlock(..
	return nAllSum;
}

static void dm_sched_keep_cyc(board_info_t *db, int has_txrx)    //POLL's local
{
	//if (has_txrx)
	//{ ... }

	#if defined DM_EXTREME_CPU_MODE

		//(lnx_dm9051_dts_Ver2.2zcd_R2_b2_savecpu5i2p_Tasklet5p_JabberP_pm_NEW2.0_extreme) //20210204
		dm_sched_cyc(db, 0);

	#else

		//(NOT extreme cpu mode && NOT light rx mode) //20210204
		#define DM_TIMER_EXPIRE1 1 //15
		#define DM_TIMER_EXPIRE2 0 //10
		#define DM_TIMER_EXPIRE3 0 //xx

		if (db -> DERFER_rwregs[RXM_WrtPTR] == db -> DERFER_rwregs1[RXM_WrtPTR])
			dm9051_INTPschedule_weight(db, DM_TIMER_EXPIRE1);
		else {
			dm_sched_cyc(db, DM_TIMER_EXPIRE3);  // faster..
		}
	#endif
}

/*
 * dm9_rx_work NOT only rx work, also carrier, tx, ...
 */
static void dm9051_rx_work_delay(struct work_struct *work)    //POLL's local
{
	int nAllSum = 0;
	struct delayed_work *dw = to_delayed_work(work);
	board_info_t *db = container_of(dw, board_info_t, rx_work);
	DLYWORK_MUTEX_HEAD_ESSENTIAL(db); //mutex_lock(spi_lock);
	
	IMR_DISABLE_LOCK_ESSENTIAL(db); //[set imr IMR_PAR, Later code arrange.., MAYBE have some trips keep imr IMR_PAR...]
	dm_carrier_poll(db); //polyn_mii_check_link(db); //on_off_carrier_lock(db); //_def_mii_check_link(&db->mii);
	dm_set_multicast_list_lock(db);
	dm_set_mac_lock(db);
	if (netif_carrier_ok(db->ndev)) {
		nAllSum = work_ss_cycle_lock(db); //=core of dm9051_mutex_dm9051(db);=
	}
	dm_sched_keep_cyc(db, nAllSum); //[CYCLE-KEEP]
	IMR_ENABLE_LOCK_ESSENTIAL(db); //"dm9 IMR-ENABLE:"
	DLYWORK_MUTEX_TAIL_ESSENTIAL(db); //mutex_unlock(spi_lock);
}

	#undef	int_sched_xmit
	#define int_sched_xmit(db)	//nothing
	#undef	int_sched_rxctl
	#define int_sched_rxctl(db)	//nothing
	#undef intcr_reg_setval //#undef intcr_reg_config
	#define intcr_reg_setval(db)	//nothing //#define intcr_reg_config(db) 
	//#undef intcr_val_make
	//#define intcr_val_make(db)	//nothing

	//#undef sched_delay_work_cancel
	//#define sched_delay_work_cancel(db,byfunc) sched_delay_work_cancel_poll(db,byfunc)	//[when polling]

//void sched_delay_work_cancel_poll(board_info_t *db, char *byfunc)    /*[when DM9051 stop]*/
//{
//	cancel_delayed_work_sync(& db -> rx_work);  //flush_work(&db->rx_work);
//}

	#undef dm_stopcode_release
	#define dm_stopcode_release(db) dm_stopcode_release_poll(db)
	
void dm_stopcode_release_poll(board_info_t *db)	
{
	cancel_delayed_work_sync(& db -> rx_work);  //flush_work(&db->rx_work);
}

	#undef dm_control_objects_init
	#define	dm_control_objects_init(db) dm_control_objects_init_poll(db)	//[when polling]
	
void dm_control_objects_init_poll(board_info_t *db)
{
	mutex_init(& db -> spi_lock);
	mutex_init(& db -> addr_lock);
	INIT_DELAYED_WORK(& db -> rx_work, dm9051_rx_work_delay);  //[once _defined in _probe() for cycles used!]
}
#endif
