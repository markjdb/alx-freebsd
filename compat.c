#include <sys/types.h>

#include <dev/mii/mii.h>

#include "compat.h"

uint32_t
ethtool_adv_to_mii_adv_t(uint32_t ethadv)
{
	uint32_t result = 0;

	if (ethadv & ADVERTISED_10baseT_Half)
		result |= ANAR_10;
	if (ethadv & ADVERTISED_10baseT_Full)
		result |= ANAR_10_FD;
	if (ethadv & ADVERTISED_100baseT_Half)
		result |= ANAR_TX;
	if (ethadv & ADVERTISED_100baseT_Full)
		result |= ANAR_TX_FD;
	if (ethadv & ADVERTISED_Pause)
		result |= ANAR_PAUSE_SYM;
	if (ethadv & ADVERTISED_Asym_Pause)
		result |= ANAR_PAUSE_ASYM;

	return (result);
}

uint32_t
ethtool_adv_to_mii_ctrl1000_t(uint32_t ethadv)
{
	uint32_t result = 0;

	if (ethadv & ADVERTISED_1000baseT_Half)
		result |= GTCR_ADV_1000THDX;
	if (ethadv & ADVERTISED_1000baseT_Full)
		result |= GTCR_ADV_1000TFDX;

	return (result);
}
