#ifndef __COMPAT_H__
#define __COMPAT_H__

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef uint32_t atomic_t; /* XXX */

#define PCI_VENDOR_ID_ATTANSIC	0x1969

#define __le32	u32
#define __le64	u64
#define __be16	u16
#define __be32	u32
#define __be64	u64

#define BIT(n)	(1UL << (n))

#define mdelay(x)	DELAY((x) * 1000)
#define udelay(x)	DELAY(x)
#ifdef msleep
#undef msleep
#endif
#define msleep(x)	pause("lnxsleep", (x) * hz / 1000);

#define cpu_to_be16(x)	htobe16(x)
#define cpu_to_be32(x)	htobe32(x)
#define be16_to_cpu(x)	be16toh(x)
#define be32_to_cpu(x)	be32toh(x)

typedef vm_paddr_t dma_addr_t;

#define ETH_ALEN	ETHER_ADDR_LEN
#define ETH_HLEN	ETHER_HDR_LEN
#define VLAN_HLEN	ETHER_VLAN_ENCAP_LEN
#define ETH_FCS_LEN	ETHER_CRC_LEN

#define DEFINE_DMA_UNMAP_ADDR(name)	dma_addr_t name

#define DEFINE_DMA_UNMAP_LEN(name)	u32 name

#define set_bit(bit, name)	bit_set((bitstr_t *)name, bit)
#define test_bit(bit, name)	bit_test((bitstr_t *)name, bit)

#define SPEED_10	10
#define SPEED_100	100
#define SPEED_1000	1000

#define ADVERTISED_10baseT_Half		(1 << 0)
#define ADVERTISED_10baseT_Full		(1 << 1)
#define ADVERTISED_100baseT_Half	(1 << 2)
#define ADVERTISED_100baseT_Full	(1 << 3)
#define ADVERTISED_1000baseT_Half	(1 << 4)
#define ADVERTISED_1000baseT_Full	(1 << 5)
#define ADVERTISED_Autoneg		(1 << 6)
#define ADVERTISED_Pause		(1 << 13)
#define ADVERTISED_Asym_Pause		(1 << 14)

#define BMCR_ANENABLE	BMCR_AUTOEN
#define BMCR_ANRESTART	BMCR_STARTNEG
#define BMCR_FULLDPLX	BMCR_FDX
#define BMCR_SPEED100	BMCR_S100

#define BMSR_LSTATUS	BMSR_LINK

#define ADVERTISE_PAUSE_CAP	ANAR_PAUSE_SYM
#define ADVERTISE_PAUSE_ASYM	ANAR_PAUSE_ASYM

#define MII_PHYSID1	MII_PHYIDR1
#define MII_PHYSID2	MII_PHYIDR2

#define MII_ADVERTISE	MII_ANAR
#define ADVERTISE_CSMA	ANAR_CSMA
#define MII_CTRL1000	MII_100T2CR

#define MII_LPA		MII_ANLPAR
#define LPA_10HALF	ANLPAR_10
#define LPA_10FULL	ANLPAR_10_FD
#define LPA_100HALF	ANLPAR_TX
#define LPA_100FULL	ANLPAR_TX_FD
#define LPA_LPACK	ANLPAR_ACK

/* XXX */
#define MDIO_DEVS1	5
#define MDIO_DEVS2	6

/* XXX */
#define PCI_MSIX_ENTRY_SIZE		16
#define PCI_MSIX_ENTRY_VECTOR_CTRL	12
#define PCI_MSIX_ENTRY_CTRL_MASKBIT	1

#define unlikely(x)	x

/* XXX */
#define spin_lock(x)
#define spin_unlock(x)

#define ARRAY_SIZE(x)   (sizeof(x) / sizeof(x[0]))

uint32_t        ethtool_adv_to_mii_adv_t(uint32_t ethadv);
uint32_t        ethtool_adv_to_mii_ctrl1000_t(uint32_t ethadv);

#endif /* __COMPAT_H__ */
