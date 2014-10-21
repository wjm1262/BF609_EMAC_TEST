/*
** adi_initialize.c source file generated on ʮ�� 16, 2014 at 17:17:44.
**
** Copyright (C) 2000-2014 Analog Devices Inc., All Rights Reserved.
**
** This file is generated automatically. You should not modify this source file,
** as your changes will be lost if this source file is re-generated.
*/

#include <sys/platform.h>
#ifdef __ADI_HAS_SEC__
#include <services/int/adi_sec.h>
#endif

#include "adi_initialize.h"

extern int32_t adi_mcapi_Init(void);
extern int32_t adi_initpinmux(void);

int32_t adi_initComponents(void)
{
	int32_t result = 0;

#ifdef __ADI_HAS_SEC__
	result = adi_sec_Init();
#endif

	if (result == 0) {
		result = adi_initpinmux(); /* auto-generated code (order:0) */
	}
	if (result == 0) {
		result = adi_mcapi_Init(); /* auto-generated code (order:6) */
	}

	return result;
}

