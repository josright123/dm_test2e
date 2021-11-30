#ifndef __DEF_GENERATION_H__
#define __DEF_GENERATION_H__

#if (CONF_VER & DM_VER_NEWINFO_MASK) == DM_VER_REL //#if _DM_TCONF_VER == 0
	//[INT].used in debug for intcr_wr_disp()/intcr_reg_disp()
	#if CONF_VER & DM_VER_DEBUG
	#define DM_DM_CONF_RARE_PROJECTS_DTS_USAGE 1
	#define DM_CONF_INTERRUPT
	#endif
#elif (CONF_VER & DM_VER_NEWINFO_MASK)
	//
	//[There 3 cast selections]
	//
	#if (CONF_VER & DM_VER_NEWINFO_MASK) == DM_VER_D1 //#if _DM_TCONF_VER == 1
	#include "../dm_hardcore/d1_conf_ver.h"
	#elif (CONF_VER & DM_VER_NEWINFO_MASK) == DM_VER_D2 //#elif _DM_TCONF_VER == 2
	#include "../dm_hardcore/d2_conf_ver.h"
	#elif (CONF_VER & DM_VER_NEWINFO_MASK) == DM_VER_D3 //#elif _DM_TCONF_VER == 3
	#include "../dm_hardcore/d3_conf_ver.h"
	#endif

	//[def generation]

	#ifdef DM_DEVEL_NDTS
	#define DM_DM_CONF_RARE_PROJECTS_DTS_USAGE 0
	#else
	#define DM_DM_CONF_RARE_PROJECTS_DTS_USAGE 1
	#endif

	#ifdef DM_DEVEL_NATM
	#define DM_DEVEL_MODULE
	#endif

	//[INT]
	#ifdef DM_DECL_INT
	#define DM_CONF_INTERRUPT
	#endif
#endif

#endif //__DEF_GENERATION_H__
