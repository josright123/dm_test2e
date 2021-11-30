#include "def_generation.h"
//{dm_build_hardcore.h, dm_build_hardcore.c}

#if CONF_VER & DM_VER_NEWINFO_MASK
	//......................................................................................
	//[d3]
	//[d2]
	  // _dts_develm13.
	  #include "../dm_hardcore/ev_disable/dm_dts_develm13_collect.c"	//module (1. dts.devel_module)
	    // from _ndts_N_mod_KT7. to _ndts_develm7.
	    #include "../dm_hardcore/ev_disable/dm_ndts_m7_collect.c"		//module,_ndts (1. dts.devel_module 2.1(non)dts.devel_module 2.2(non)dts.N_mod_KT
	      //_when INT retrival the INT parameters from NDTS hard-coded
	      #include "../dm_hardcore/opt/dm_ndts_redesigni_collect.c"		//int,_ndts

	//[d1]
	//[poll-behavior functions.]
	#include "../dm_hardcore/opt/dm_both_redesignp_collect.h"		//header_file
	#include "../dm_hardcore/opt/dm_both_redesignp_collect.c"		//poll,_dts,_ndts
	#include "../dm_hardcore/opt/func_redesignp_subroutine.c"		//poll, for (dts_ndts)
	//......................................................................................
#endif
