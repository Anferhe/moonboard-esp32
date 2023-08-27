/*
 * map.c
 *
 *  Created on: Aug 27, 2023
 *      Author: yaroslav
 */

#include "map.h"
#include "defines.h"

#define DEFAULT 0

//! DEFAULT mapping coincides with indexing in moonboard application:
//! A1 -> A18 -> B18 -> B1 -> C1 -> C18 -> D18 ...
#if MOONBOARD_MAP_TYPE == DEFAULT

inline size_t mapLed(size_t moonboardAppIndex)
{
	return moonboardAppIndex;
}

#else
#error "Unsupported MOONBOARD_MAP_TYPE"
#endif
