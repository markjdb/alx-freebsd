/*-
 * Copyright (c) 2012 Qualcomm Atheros, Inc.
 * Copyright (c) 2013, Mark Johnston <markj@FreeBSD.org>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bitstring.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <net/bpf.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_var.h>
#include <net/if_vlan_var.h>

#include <netinet/in.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <machine/bus.h>

#include "compat.h"
#include "alx_hw.h"
#include "if_alxreg.h"
#include "if_alxvar.h"

MODULE_DEPEND(alx, pci, 1, 1, 1);
MODULE_DEPEND(alx, ether, 1, 1, 1);

static struct alx_dev {
	uint16_t	 alx_vendorid;
	uint16_t	 alx_deviceid;
	const char	*alx_name;
} alx_devs[] = {
	{ ALX_VENDOR_ID, ALX_DEV_ID_AR8161,
	    "Qualcomm Atheros AR8161 Gigabit Ethernet" },
	{ ALX_VENDOR_ID, ALX_DEV_ID_AR8162,
	    "Qualcomm Atheros AR8162 Fast Ethernet" },
	{ ALX_VENDOR_ID, ALX_DEV_ID_AR8171,
	    "Qualcomm Atheros AR8171 Gigabit Ethernet" },
	{ ALX_VENDOR_ID, ALX_DEV_ID_AR8172,
	    "Qualcomm Atheros AR8172 Fast Ethernet" },
};

static int	alx_attach(device_t);
static int	alx_detach(device_t);
static int	alx_probe(device_t);
static int	alx_resume(device_t);
static int	alx_shutdown(device_t);
static void	alx_stop(struct alx_softc *);
static int	alx_suspend(device_t);

static int	alx_ioctl(struct ifnet *, u_long, caddr_t);
static void	alx_init(void *);
static void	alx_init_locked(struct alx_softc *);
static int	alx_media_change(struct ifnet *);
static void	alx_media_status(struct ifnet *, struct ifmediareq *);
static void	alx_start(struct ifnet *);
static void	alx_start_locked(struct ifnet *);

static int	alx_alloc_intr(struct alx_softc *);
static void	alx_free_intr(struct alx_softc *);
static void	alx_link_task(void *, int);
static void	alx_int_task(void *, int);
static void	alx_intr_disable(struct alx_softc *);
static void	alx_intr_enable(struct alx_softc *);
static int	alx_intr_legacy(void *);
static int	alx_intr_msi(void *);

static void	alx_reset(struct alx_softc *sc);
static void	alx_update_link(struct alx_softc *);

static int	alx_dma_alloc(struct alx_softc *);
static void	alx_dma_free(struct alx_softc *);
static void	alx_dmamap_cb(void *, bus_dma_segment_t *, int, int);

static void	alx_init_rx_ring(struct alx_softc *);
static void	alx_init_tx_ring(struct alx_softc *);
static int	alx_newbuf(struct alx_softc *, int);
static void	alx_rxintr(struct alx_softc *);
static void	alx_txintr(struct alx_softc *);
static int	alx_xmit(struct alx_softc *, struct mbuf **);

static device_method_t alx_methods[] = {
	DEVMETHOD(device_probe,		alx_probe),
	DEVMETHOD(device_attach,	alx_attach),
	DEVMETHOD(device_detach,	alx_detach),
	DEVMETHOD(device_shutdown,	alx_shutdown),
	DEVMETHOD(device_suspend,	alx_suspend),
	DEVMETHOD(device_resume,	alx_resume),

	DEVMETHOD_END
};

static driver_t alx_driver = {
	"alx",
	alx_methods,
	sizeof(struct alx_softc)
};

static devclass_t alx_devclass;

DRIVER_MODULE(alx, pci, alx_driver, alx_devclass, 0, 0);

static SYSCTL_NODE(_hw, OID_AUTO, alx, CTLFLAG_RD, 0, "alx driver settings");

static int alx_enable_msix = 0;
TUNABLE_INT("hw.alx.enable_msix", &alx_enable_msix);
SYSCTL_INT(_hw_alx, OID_AUTO, enable_msix, CTLFLAG_RDTUN, &alx_enable_msix,
    0, "Enable MSI-X interrupts");

static int alx_enable_msi = 0;
TUNABLE_INT("hw.alx.enable_msi", &alx_enable_msi);
SYSCTL_INT(_hw_alx, OID_AUTO, enable_msi, CTLFLAG_RDTUN, &alx_enable_msi,
    0, "Enable MSI interrupts");

static void
alx_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{

	if (error != 0)
		return;
	*(bus_addr_t *)arg = segs[0].ds_addr;
}

/*
 * XXX:
 * - multiple queues
 * - does the chipset's DMA engine support more than one segment?
 */
static int
alx_dma_alloc(struct alx_softc *sc)
{
	device_t dev;
	struct alx_hw *hw;
	struct alx_buffer *buf;
	int error, i;

	dev = sc->alx_dev;
	hw = &sc->hw;

	/* Create parent tag. */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->alx_dev),	/* parent */
	    1, 0,				/* alignment, boundary */
	    BUS_SPACE_MAXADDR,			/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filter, filterarg */
	    BUS_SPACE_MAXSIZE_32BIT,		/* maxsize */
	    1,					/* nsegments */
	    BUS_SPACE_MAXSIZE_32BIT,		/* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lockfunc, lockfuncarg */
	    &sc->alx_parent_tag);
	if (error != 0) {
		device_printf(dev, "could not create parent DMA tag\n");
		return (error);
	}

	/* Create the DMA tag for the transmit packet descriptor ring. */
	/* XXX assuming 1 queue at the moment. */
	error = bus_dma_tag_create(
	    sc->alx_parent_tag,			/* parent */
	    8, 0,				/* alignment, boundary */
	    BUS_SPACE_MAXADDR,			/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filter, filterarg */
	    sc->tx_ringsz * sizeof(struct tpd_desc), /* maxsize */
	    1,					/* nsegments */
	    sc->tx_ringsz * sizeof(struct tpd_desc), /* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lockfunc, lockfuncarg */
	    &sc->alx_tx_tag);
	if (error != 0) {
		device_printf(dev, "could not create TX descriptor ring tag\n");
		return (error);
	}

	/* Allocate DMA memory for the transmit packet descriptor ring. */
	error = bus_dmamem_alloc(sc->alx_tx_tag,
	    (void **)&sc->alx_tx_queue.tpd_hdr,
	    BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT,
	    &sc->alx_tx_dmamap);
	if (error != 0) {
		device_printf(dev,
		    "could not allocate DMA'able memory for TX ring\n");
		/* XXX cleanup */
		return (error);
	}

	/* Do the actual DMA mapping of the transmit packet descriptor ring. */
	error = bus_dmamap_load(sc->alx_tx_tag, sc->alx_tx_dmamap,
	    sc->alx_tx_queue.tpd_hdr, sc->tx_ringsz * sizeof(struct tpd_desc),
	    alx_dmamap_cb, &sc->alx_tx_queue.tpd_dma, 0);
	if (error != 0) {
		device_printf(dev, "could not load DMA map for TX ring\n");
		/* XXX cleanup */
		return (error);
	}

	/* Create the DMA tag for the receive ready descriptor ring. */
	/* XXX assuming 1 queue at the moment. */
	error = bus_dma_tag_create(
	    sc->alx_parent_tag,			/* parent */
	    8, 0,				/* alignment, boundary */
	    BUS_SPACE_MAXADDR,			/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filter, filterarg */
	    sc->rx_ringsz * sizeof(struct rrd_desc), /* maxsize */
	    1,					/* nsegments */
	    sc->rx_ringsz * sizeof(struct rrd_desc), /* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lockfunc, lockfuncarg */
	    &sc->alx_rr_tag);
	if (error != 0) {
		device_printf(dev, "could not create RX descriptor ring tag\n");
		return (error);
	}

	error = bus_dmamem_alloc(sc->alx_rr_tag,
	    (void **)&sc->alx_rx_queue.rrd_hdr,
	    BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT,
	    &sc->alx_rr_dmamap);
	if (error != 0) {
		device_printf(dev,
		    "could not allocate DMA'able memory for RX ring\n");
		/* XXX cleanup */
		return (error);
	}

	error = bus_dmamap_load(sc->alx_rr_tag, sc->alx_rr_dmamap,
	    sc->alx_rx_queue.rrd_hdr, sc->rx_ringsz * sizeof(struct rrd_desc),
	    alx_dmamap_cb, &sc->alx_rx_queue.rrd_dma, 0);
	if (error != 0) {
		device_printf(dev,
		    "could not load DMA map for RX ready ring\n");
		/* XXX cleanup */
		return (error);
	}

	/* Create the DMA tag for the receive free descriptor ring. */
	/* XXX assuming 1 queue at the moment. */
	error = bus_dma_tag_create(
	    sc->alx_parent_tag,			/* parent */
	    8, 0,				/* alignment, boundary */
	    BUS_SPACE_MAXADDR,			/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filter, filterarg */
	    sc->rx_ringsz * sizeof(struct rfd_desc), /* maxsize */
	    1,					/* nsegments */
	    sc->rx_ringsz * sizeof(struct rfd_desc), /* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lockfunc, lockfuncarg */
	    &sc->alx_rx_tag);
	if (error != 0) {
		device_printf(dev,
		    "could not create RX descriptor ring DMA tag\n");
		/* XXX cleanup */
		return (error);
	}

	error = bus_dmamem_alloc(sc->alx_rx_tag,
	    (void **)&sc->alx_rx_queue.rfd_hdr,
	    BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT,
	    &sc->alx_rx_dmamap);
	if (error != 0) {
		device_printf(dev,
		    "could not allocate DMA'able memory for RX ring\n");
		/* XXX cleanup */
		return (error);
	}

	error = bus_dmamap_load(sc->alx_rx_tag, sc->alx_rx_dmamap,
	    sc->alx_rx_queue.rfd_hdr, sc->rx_ringsz * sizeof(struct rfd_desc),
	    alx_dmamap_cb, &sc->alx_rx_queue.rfd_dma, 0);
	if (error != 0) {
		device_printf(dev, "could not load DMA map for RX free ring\n");
		/* XXX cleanup */
		return (error);
	}

	/* Create the DMA tag for the transmit buffers. */
	/* XXX where do maxsize, nsegments, maxsegsize come from? */
	error = bus_dma_tag_create(
	    sc->alx_parent_tag,			/* parent */
	    8, 0,				/* alignment, boundary */
	    BUS_SPACE_MAXADDR,			/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filter, filterarg */
	    PAGE_SIZE,				/* maxsize XXX */
	    32,					/* nsegments XXX */
	    PAGE_SIZE,				/* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lockfunc, lockarg */
	    &sc->alx_tx_buf_tag);
	if (error != 0) {
		device_printf(dev, "could not create TX buffer DMA tag\n");
		/* XXX cleanup */
		return (error);
	}

	/* Allocate space for the TX buffer ring. */
	sc->alx_tx_queue.bf_info = malloc(
	    sc->tx_ringsz * sizeof(struct alx_buffer), M_DEVBUF,
	    M_NOWAIT | M_ZERO);
	if (sc->alx_tx_queue.bf_info == NULL) {
		device_printf(dev,
		    "could not allocate memory for TX buffer ring\n");
		/* XXX cleanup */
		return (error);
	}

	/* Create DMA maps for the TX buffers. */
	buf = sc->alx_tx_queue.bf_info;
	for (i = 0; i < sc->tx_ringsz; i++, buf++) {
		error = bus_dmamap_create(sc->alx_tx_buf_tag, 0, &buf->dmamap);
		if (error != 0) {
			device_printf(dev, "could not create TX DMA map\n");
			/* XXX cleanup */
			return (error);
		}
	}

	/* Create the DMA tag for the receive buffers. */
	error = bus_dma_tag_create(
	    sc->alx_parent_tag,			/* parent */
	    8, 0,				/* alignment, boundary */
	    BUS_SPACE_MAXADDR,			/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filter, filterarg */
	    MCLBYTES,				/* maxsize */
	    1,					/* nsegments */
	    MCLBYTES,				/* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lockfunc, lockarg */
	    &sc->alx_rx_buf_tag);
	if (error != 0) {
		device_printf(dev, "could not create RX buffer DMA tag\n");
		/* XXX cleanup */
		return (error);
	}

	/* Allocate space for the RX buffer ring. */
	sc->alx_rx_queue.bf_info = malloc(
	    sc->rx_ringsz * sizeof(struct alx_buffer), M_DEVBUF,
	    M_NOWAIT | M_ZERO);
	if (sc->alx_rx_queue.bf_info == NULL) {
		device_printf(dev,
		    "could not allocate memory for RX buffer ring\n");
		/* XXX cleanup */
		return (error);
	}

	/* Create DMA maps for the RX buffers. */
	buf = sc->alx_rx_queue.bf_info;
	for (i = 0; i < sc->rx_ringsz; i++, buf++) {
		error = bus_dmamap_create(sc->alx_rx_buf_tag, 0, &buf->dmamap);
		if (error != 0) {
			device_printf(dev, "could not create RX DMA map\n");
			/* XXX cleanup */
			return (error);
		}
	}

	return (error);
}

static void __unused
alx_dma_free(struct alx_softc *sc)
{

}

static void
alx_intr_enable(struct alx_softc *sc)
{
	struct alx_hw *hw;

	hw = &sc->hw;

	/* level-1 interrupt switch */
	ALX_MEM_W32(hw, ALX_ISR, 0);
	ALX_MEM_W32(hw, ALX_IMR, hw->imask);
	ALX_MEM_FLUSH(hw);

#ifdef __notyet
	int i;
	/* enable all individual MSIX IRQs */
	if (ALX_FLAG(sc, USING_MSIX))
		for (i = 0; i < adpt->nr_vec; i++)
			alx_mask_msix(hw, i, false);
#endif
}

static void
alx_intr_disable(struct alx_softc *sc)
{
	struct alx_hw *hw = &sc->hw;

	ALX_MEM_W32(hw, ALX_ISR, ALX_ISR_DIS);
	ALX_MEM_W32(hw, ALX_IMR, 0);
	ALX_MEM_FLUSH(hw);

#ifdef __notyet
	int i;
	if (ALX_FLAG(sc, USING_MSIX))
		for (i = 0; i < sc->nr_vec; i++)
			alx_mask_msix(hw, i, true);
#endif
}

static int
alx_identify_hw(struct alx_softc *sc)
{
	device_t dev = sc->alx_dev;
	struct alx_hw *hw = &sc->hw;
	int rev;

	hw->device_id = pci_get_device(dev);
	hw->subdev_id = pci_get_subdevice(dev);
	hw->subven_id = pci_get_subvendor(dev);
	hw->revision = pci_get_revid(dev);
	rev = ALX_REVID(hw);

	switch (ALX_DID(hw)) {
	case ALX_DEV_ID_AR8161:
	case ALX_DEV_ID_AR8162:
	case ALX_DEV_ID_AR8171:
	case ALX_DEV_ID_AR8172:
		if (rev > ALX_REV_C0)
			break;
		ALX_CAP_SET(hw, L0S);
		ALX_CAP_SET(hw, L1);
		ALX_CAP_SET(hw, MTQ);
		ALX_CAP_SET(hw, RSS);
		ALX_CAP_SET(hw, MSIX);
		ALX_CAP_SET(hw, SWOI);
		hw->max_dma_chnl = rev >= ALX_REV_B0 ? 4 : 2;
		if (rev < ALX_REV_C0) {
			hw->ptrn_ofs = 0x600;
			hw->max_ptrns = 8;
		} else {
			hw->ptrn_ofs = 0x14000;
			hw->max_ptrns = 16;
		}
		break;
	default:
		return (EINVAL);
	}

	if (ALX_DID(hw) & 1)
		ALX_CAP_SET(hw, GIGA);

	return (0);
}

static const u8 def_rss_key[40] = {
	0xE2, 0x91, 0xD7, 0x3D, 0x18, 0x05, 0xEC, 0x6C,
	0x2A, 0x94, 0xB3, 0x0D, 0xA5, 0x4F, 0x2B, 0xEC,
	0xEA, 0x49, 0xAF, 0x7C, 0xE2, 0x14, 0xAD, 0x3D,
	0xB8, 0x55, 0xAA, 0xBE, 0x6A, 0x3E, 0x67, 0xEA,
	0x14, 0x36, 0x4D, 0x17, 0x3B, 0xED, 0x20, 0x0D,
};

/* alx_init_adapter -
 *    initialize general software structure (struct alx_adapter).
 *    fields are inited based on PCI device information.
 */
static int
alx_init_sw(struct alx_softc *sc)
{
	device_t dev = sc->alx_dev;
	struct alx_hw *hw = &sc->hw;
	int i, err;

	err = alx_identify_hw(sc);
	if (err) {
		device_printf(dev, "unrecognized chip, aborting\n");
		return (err);
	}

	/* assign patch flag for specific platforms */
	alx_patch_assign(hw);

	memcpy(hw->rss_key, def_rss_key, sizeof(def_rss_key));
	hw->rss_idt_size = 128;
	hw->rss_hash_type = ALX_RSS_HASH_TYPE_ALL;
	hw->smb_timer = 400;
	sc->tx_ringsz = 256;
	sc->rx_ringsz = 512;
	hw->sleep_ctrl = ALX_SLEEP_WOL_MAGIC | ALX_SLEEP_WOL_PHY;
	hw->imt = 200;
	hw->imask = ALX_ISR_MISC;
	hw->dma_chnl = hw->max_dma_chnl;
	hw->ith_tpd = sc->tx_ringsz / 3;
	hw->link_up = false;
	hw->link_duplex = 0;
	hw->link_speed = 0;
	hw->adv_cfg = ADVERTISED_Autoneg | ADVERTISED_10baseT_Half |
	    ADVERTISED_10baseT_Full | ADVERTISED_100baseT_Full |
	    ADVERTISED_100baseT_Half | ADVERTISED_1000baseT_Full;
	hw->flowctrl = ALX_FC_ANEG | ALX_FC_RX | ALX_FC_TX;
	hw->wrr_ctrl = ALX_WRR_PRI_RESTRICT_NONE;
	for (i = 0; i < ARRAY_SIZE(hw->wrr); i++)
		hw->wrr[i] = 4;

	hw->rx_ctrl = ALX_MAC_CTRL_WOLSPED_SWEN | ALX_MAC_CTRL_MHASH_ALG_HI5B |
	    ALX_MAC_CTRL_BRD_EN | ALX_MAC_CTRL_PCRCE | ALX_MAC_CTRL_CRCE |
	    ALX_MAC_CTRL_RXFC_EN | ALX_MAC_CTRL_TXFC_EN |
	    FIELDX(ALX_MAC_CTRL_PRMBLEN, 7);
	hw->is_fpga = false;

	sc->irq_sem = 0;

	return err;
}

static void
alx_init_rx_ring(struct alx_softc *sc)
{
	struct alx_hw *hw;
	int i, error;

	ALX_LOCK_ASSERT(sc);

	hw = &sc->hw;

	sc->alx_rx_queue.pidx = 0;
	sc->alx_rx_queue.p_reg = ALX_RFD_PIDX;
	sc->alx_rx_queue.cidx = 0;
	sc->alx_rx_queue.rrd_cidx = 0;
	sc->alx_rx_queue.c_reg = ALX_RFD_CIDX;
	sc->alx_rx_queue.qidx = 0;
	sc->alx_rx_queue.count = sc->rx_ringsz;

	hw->imask |= ALX_ISR_RX_Q0;

	/* XXX the rings are all supposed to come from the same 4GB block. */
	ALX_MEM_W32(hw, ALX_RX_BASE_ADDR_HI, sc->alx_rx_queue.rfd_dma >> 32);
	ALX_MEM_W32(hw, ALX_RRD_ADDR_LO, sc->alx_rx_queue.rrd_dma);
	ALX_MEM_W32(hw, ALX_RFD_ADDR_LO, sc->alx_rx_queue.rfd_dma);
	ALX_MEM_W32(hw, ALX_RRD_RING_SZ, sc->rx_ringsz);
	ALX_MEM_W32(hw, ALX_RFD_RING_SZ, sc->rx_ringsz);
	ALX_MEM_W32(hw, ALX_RFD_BUF_SZ, sc->rxbuf_size);

	/* XXX multiple queues. */
	for (i = 0; i < sc->rx_ringsz; i++) {
		error = alx_newbuf(sc, i);
		if (error != 0) {
			/* XXX this needs to be handled better. */
			device_printf(sc->alx_dev, "failed to refresh mbufs\n");
			break;
		}
	}

	ALX_MEM_W16(hw, ALX_RFD_PIDX, sc->rx_ringsz - 1);

	bus_dmamap_sync(sc->alx_rx_tag, sc->alx_rx_dmamap,
	    BUS_DMASYNC_PREWRITE);
}

static void
alx_init_tx_ring(struct alx_softc *sc)
{
	struct alx_hw *hw;
	struct alx_buffer *tx_buf;
	int i;

	ALX_LOCK_ASSERT(sc);

	hw = &sc->hw;

	sc->alx_tx_queue.pidx = 0;
	sc->alx_tx_queue.p_reg = ALX_TPD_PRI0_PIDX;
	sc->alx_tx_queue.cidx = 0;
	sc->alx_tx_queue.c_reg = ALX_TPD_PRI0_CIDX;
	sc->alx_tx_queue.qidx = 0;
	sc->alx_tx_queue.count = sc->tx_ringsz;

	hw->imask |= ALX_ISR_TX_Q0;

	for (i = 0; i < sc->tx_ringsz; i++) {
		tx_buf = &sc->alx_tx_queue.bf_info[i];
		tx_buf->m = NULL;
	}

	/* XXX multiple queues */
	ALX_MEM_W32(hw, ALX_TX_BASE_ADDR_HI, sc->alx_tx_queue.tpd_dma >> 32);
	ALX_MEM_W32(hw, ALX_TPD_PRI0_ADDR_LO, sc->alx_tx_queue.tpd_dma);
	ALX_MEM_W32(hw, ALX_TPD_RING_SZ, sc->tx_ringsz);
}

static void
alx_rxintr(struct alx_softc *sc)
{
	struct mbuf *m;
	struct ifnet *ifp;
	struct alx_buffer *rx_buf;
	struct rrd_desc *rrd;
	int rrd_cidx, rfd_cidx, rrd_pidx, count;

	ALX_LOCK_ASSERT(sc);

	bus_dmamap_sync(sc->alx_rr_tag, sc->alx_rr_dmamap,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
	bus_dmamap_sync(sc->alx_rx_tag, sc->alx_rx_dmamap,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	ifp = sc->alx_ifp;

	count = 0;
	rrd_cidx = sc->alx_rx_queue.cidx;
	printf("consuming packets starting at %d\n", rrd_cidx);
	while (1) {
		rrd = &sc->alx_rx_queue.rrd_hdr[rrd_cidx];
		if ((rrd->word3 & (1 << RRD_UPDATED_SHIFT)) == 0)
			break;
		rrd->word3 &= ~(1 << RRD_UPDATED_SHIFT);

		/* Get the index of the corresponding RFD. */
		rfd_cidx = FIELD_GETX(rrd->word0, RRD_SI);
		if (rfd_cidx != rrd_cidx ||
		    FIELD_GETX(rrd->word0, RRD_NOR) != 1) {
			device_printf(sc->alx_dev,
			    "RX consumer index mismatch: %d vs. %d, and %d\n",
			    rrd_cidx, rfd_cidx, FIELD_GETX(rrd->word0, RRD_NOR));
			/* XXX reset the chip? */
			return;
		}

		rx_buf = &sc->alx_rx_queue.bf_info[rfd_cidx];
		m = rx_buf->m;
		rx_buf->m = NULL;

		m->m_flags |= M_PKTHDR;
		m->m_len = FIELD_GETX(rrd->word3, RRD_PKTLEN) - ETHER_CRC_LEN;
		m->m_pkthdr.len = m->m_len;
		m->m_pkthdr.rcvif = ifp;

		printf("read a %d-byte packet\n", m->m_len);

		/* Pass the packet up the stack. */
		ALX_UNLOCK(sc);
		(*ifp->if_input)(ifp, m);
		ALX_LOCK(sc);

		count++;
		if (++rrd_cidx == sc->rx_ringsz)
			rrd_cidx = 0;
	}

	printf("consumed %d packets\n", count);

	if (count > 0) {
		sc->alx_rx_queue.cidx = rrd_cidx;

		/* Refresh mbufs. */
		rrd_pidx = sc->alx_rx_queue.pidx;
		while (rrd_pidx != rrd_cidx) {
			printf("refreshing mbuf at %d\n", rrd_pidx);
			if (alx_newbuf(sc, rrd_pidx) != 0)
				break;
			if (++rrd_pidx == sc->rx_ringsz)
				rrd_pidx = 0;
		}
		sc->alx_rx_queue.pidx = rrd_pidx;
		ALX_MEM_W16(&sc->hw, ALX_RFD_PIDX, rrd_pidx);

		/* Sync receive descriptors. */
		bus_dmamap_sync(sc->alx_rr_tag, sc->alx_rr_dmamap,
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
		bus_dmamap_sync(sc->alx_rx_tag, sc->alx_rx_dmamap,
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	}
}

static void
alx_txintr(struct alx_softc *sc)
{
	struct ifnet *ifp;
	struct alx_buffer *tx_buf;
	int tpd_cidx, tpd_hw_cidx;

	ALX_LOCK_ASSERT(sc);

	ifp = sc->alx_ifp;

	tpd_cidx = sc->alx_tx_queue.cidx;
	ALX_MEM_R16(&sc->hw, ALX_TPD_PRI0_CIDX, &tpd_hw_cidx);
#if 0
	bus_dmamap_sync(sc->alx_tx_tag, sc->alx_tx_dmamap,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
#endif

	printf("in txintr cidx is %d, hw_cidx is %d\n", tpd_cidx, tpd_hw_cidx);

	while (tpd_cidx != tpd_hw_cidx) {
		tx_buf = &sc->alx_tx_queue.bf_info[tpd_cidx];
		if (tx_buf->m == NULL)
			continue;

		/* Tear down the DMA mapping for the used mbuf. */
		bus_dmamap_sync(sc->alx_tx_buf_tag, tx_buf->dmamap,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->alx_tx_buf_tag, tx_buf->dmamap);

		m_freem(tx_buf->m);
		tx_buf->m = NULL;

		if (++tpd_cidx == sc->tx_ringsz)
			tpd_cidx = 0;
	}

	sc->alx_tx_queue.cidx = tpd_cidx;
}

static int
alx_newbuf(struct alx_softc *sc, int index)
{
	struct mbuf *m;
	bus_dma_segment_t seg;
	struct alx_buffer *rx_buf;
	struct rfd_desc *rfd;
	int nsegs;

	m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
	if (m == NULL)
		return (ENOBUFS);
	m->m_len = m->m_pkthdr.len = MCLBYTES;

	rx_buf = &sc->alx_rx_queue.bf_info[index];
	if (bus_dmamap_load_mbuf_sg(sc->alx_rx_buf_tag, rx_buf->dmamap, m, &seg,
	    &nsegs, 0) != 0) {
		m_freem(m);
		return (ENOBUFS);
	}
	rx_buf->m = m;
	bus_dmamap_sync(sc->alx_rx_buf_tag, rx_buf->dmamap,
	    BUS_DMASYNC_PREREAD);

	rfd = &sc->alx_rx_queue.rfd_hdr[index];
	rfd->addr = htole64(seg.ds_addr);

	return (0);
}

static int
alx_xmit(struct alx_softc *sc, struct mbuf **m_head)
{
	struct mbuf *m;
	bus_dma_segment_t seg;
	bus_dmamap_t txmap;
	struct tpd_desc *td;
	struct alx_buffer *tx_buf, *tx_buf_mapped;
	int desci, error, nsegs, i;
	uint16_t cidx;

	ALX_LOCK_ASSERT(sc);

	M_ASSERTPKTHDR(*m_head);

	ALX_MEM_R16(&sc->hw, sc->alx_tx_queue.c_reg, &cidx);

	desci = sc->alx_tx_queue.pidx;
	tx_buf = tx_buf_mapped = &sc->alx_tx_queue.bf_info[desci];
	txmap = tx_buf->dmamap;

	error = bus_dmamap_load_mbuf_sg(sc->alx_tx_buf_tag, txmap, *m_head,
	    &seg, &nsegs, 0);
	if (error == EFBIG) {
		m = m_collapse(*m_head, M_NOWAIT, 32 /* XXX magic */);
		if (m == NULL) {
			/* XXX increment counter? */
			m_freem(*m_head);
			*m_head = NULL;
			return (ENOBUFS);
		}
		*m_head = m;
		/*
		 * Try to create the mapping again now that some of the
		 * fragments have been coalesced.
		 */
		error = bus_dmamap_load_mbuf_sg(sc->alx_tx_buf_tag, txmap,
		    *m_head, &seg, &nsegs, 0);
		if (error != 0) {
			m_freem(*m_head);
			*m_head = NULL;
			return (error);
		}
	} else if (error != 0)
		/* XXX increment counter? */
		return (error);

	printf("mbuf is fragged into %d segments\n", nsegs);

	if (nsegs == 0) {
		m_freem(*m_head);
		*m_head = NULL;
		/* XXX increment counter? */
		return (EIO);
	}

	/* Make sure we have enough descriptors available. */
	/* XXX what's up with the - 2? It's in em(4) and age(4). */
	if (nsegs > sc->alx_tx_queue.count - 2) {
		/* XXX increment counter? */
		bus_dmamap_unload(sc->alx_tx_tag, txmap);
		return (ENOBUFS);
	}

	for (i = 0; i < nsegs; i++, desci = ALX_TX_INC(desci, sc)) {
		td = &sc->alx_tx_queue.tpd_hdr[desci];
		/* XXX handle multiple segments. */
		td->adrl.addr = htole64(seg.ds_addr);
		FIELD_SET32(td->word0, TPD_BUFLEN, seg.ds_len);
		td->word1 = 0;
	}

	/* This is the last descriptor for this packet. */
	td->word1 |= 1 << TPD_EOP_SHIFT;

	/* Update the producer index. */
	sc->alx_tx_queue.pidx = desci;

	/* Save the mbuf pointer so that we can unmap it later. */
	tx_buf->m = *m_head;

	/*
	 * Swap the maps between the first and last descriptors so that the last
	 * descriptor gets the real map. The first descriptor will end up with
	 * an unused map.
	 */
	tx_buf_mapped->dmamap = tx_buf->dmamap;
	tx_buf->dmamap = txmap;
	bus_dmamap_sync(sc->alx_tx_buf_tag, txmap, BUS_DMASYNC_PREWRITE);

	/* Let the hardware know that we're all set. */
	bus_dmamap_sync(sc->alx_tx_tag, sc->alx_tx_dmamap,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	ALX_MEM_W16(&sc->hw, sc->alx_tx_queue.p_reg, desci);

	return (0);
}

static void
alx_stop(struct alx_softc *sc)
{
	struct ifnet *ifp;
	struct alx_hw *hw;
	struct alx_buffer *tx_buf, *rx_buf;
	int i, error;

	ALX_LOCK_ASSERT(sc);

	hw = &sc->hw;
	ifp = sc->alx_ifp;
	ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);

	alx_intr_disable(sc);

	error = alx_stop_mac(hw);
	if (error != 0)
		device_printf(sc->alx_dev, "error stopping MAC\n");

	/* XXX what else? */
	for (i = 0; i < sc->tx_ringsz; i++) {
		tx_buf = &sc->alx_tx_queue.bf_info[i];
		if (tx_buf->m != NULL) {
			bus_dmamap_sync(sc->alx_tx_buf_tag, tx_buf->dmamap,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(sc->alx_tx_buf_tag, tx_buf->dmamap);
			m_freem(tx_buf->m);
			tx_buf->m = NULL;
		}
	}

	for (i = 0; i < sc->rx_ringsz; i++) {
		rx_buf = &sc->alx_rx_queue.bf_info[i];
		if (rx_buf->m != NULL) {
			bus_dmamap_sync(sc->alx_rx_buf_tag, rx_buf->dmamap,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(sc->alx_rx_buf_tag, rx_buf->dmamap);
			m_freem(rx_buf->m);
			rx_buf->m = NULL;
		}
	}
}

static void
alx_reset(struct alx_softc *sc)
{
	device_t dev;
	struct alx_hw *hw;
	bool phy_cfged;

	dev = sc->alx_dev;
	hw = &sc->hw;

	alx_reset_pcie(hw);

	phy_cfged = alx_phy_configed(hw);
	if (!phy_cfged)
		alx_reset_phy(hw, !hw->hib_patch);

	if (alx_reset_mac(hw))
		device_printf(dev, "failed to reset MAC\n");

	/* XXX what else? */
}

static void
alx_update_link(struct alx_softc *sc)
{
	struct alx_hw *hw;
	bool link_up, prev_link_up;
	int error;
	uint16_t speed, prev_speed;

	ALX_LOCK_ASSERT(sc);

	hw = &sc->hw;

	error = alx_get_phy_link(hw, &link_up, &speed);
	if (error != 0)
		return;

	if ((!link_up && !hw->link_up) ||
	    (sc->alx_ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
		return;

	prev_speed = hw->link_speed + hw->link_duplex;
	prev_link_up = hw->link_up;

	hw->link_up = link_up;
	if (link_up) {
		if (prev_link_up && prev_speed == speed)
			return;

		hw->link_duplex = speed % 10;
		hw->link_speed = speed - hw->link_duplex;

		alx_post_phy_link(hw, hw->link_speed, ALX_CAP(hw, AZ));
		alx_enable_aspm(hw, ALX_CAP(hw, L0S), ALX_CAP(hw, L1));
		alx_start_mac(hw);

		if_link_state_change(sc->alx_ifp, LINK_STATE_UP);
	} else {
		hw->link_duplex = 0;
		hw->link_speed = 0;

		error = alx_reset_mac(hw);
		if (error != 0) {
			device_printf(sc->alx_dev, "failed to reset MAC\n");
			return;
		}

		alx_intr_disable(sc);
		/* XXX refresh rings */
		alx_configure_basic(hw);
		alx_configure_rss(hw, 0 /* XXX */);
		alx_enable_aspm(hw, false, ALX_CAP(hw, L1));
		alx_post_phy_link(hw, 0, ALX_CAP(hw, AZ));
		alx_intr_enable(sc);

		if_link_state_change(sc->alx_ifp, LINK_STATE_DOWN);
	}
}

static void
alx_int_task(void *context, int pending __unused)
{
	struct alx_softc *sc;

	printf("in alx_int_task\n");

	sc = context;

	ALX_LOCK(sc);

	/* XXX check isr? */
	alx_rxintr(sc);
	alx_txintr(sc);

	ALX_UNLOCK(sc);
}

static void
alx_link_task(void *arg, int pending __unused)
{
	struct alx_softc *sc;
	struct alx_hw *hw;

	sc = arg;
	hw = &sc->hw;

	ALX_LOCK(sc);

	alx_clear_phy_intr(hw);

	hw->imask |= ALX_ISR_PHY;
	ALX_MEM_W32(hw, ALX_IMR, hw->imask);

	alx_update_link(sc);

	ALX_UNLOCK(sc);
}

static int
alx_intr_legacy(void *arg)
{
	struct alx_softc *sc;
	struct alx_hw *hw;
	uint32_t intr;

	sc = arg;
	hw = &sc->hw;

	ALX_MEM_R32(hw, ALX_ISR, &intr);
	if (intr & ALX_ISR_DIS || (intr & hw->imask) == 0)
		return (FILTER_STRAY);

	printf("intr is 0x%x, imask is 0x%x\n", intr, hw->imask);

	/* Acknowledge and disable interrupts. */
	ALX_MEM_W32(hw, ALX_ISR, intr | ALX_ISR_DIS);

	intr &= hw->imask;
	if (intr & ALX_ISR_PHY) {
		hw->imask &= ~ALX_ISR_PHY;
		ALX_MEM_W32(hw, ALX_IMR, hw->imask);
		taskqueue_enqueue(sc->alx_tq, &sc->alx_link_task);
	}

	if (intr & (ALX_ISR_TX_Q0 | ALX_ISR_RX_Q0))
		taskqueue_enqueue(sc->alx_tq, &sc->alx_int_task);

	ALX_MEM_W32(hw, ALX_ISR, 0);

	return (FILTER_HANDLED);
}

static int
alx_intr_msi(void *arg)
{
	struct alx_softc *sc;
	struct alx_hw *hw;
	uint32_t intr;

	sc = arg;
	hw = &sc->hw;

	ALX_MEM_R32(hw, ALX_ISR, &intr);

	ALX_MEM_W32(hw, ALX_ISR, intr | ALX_ISR_DIS);

	intr &= hw->imask;
	if (intr & ALX_ISR_PHY) {
		hw->imask &= ~ALX_ISR_PHY;
		ALX_MEM_W32(hw, ALX_IMR, hw->imask);
		taskqueue_enqueue(sc->alx_tq, &sc->alx_link_task);
	}

	ALX_MEM_W32(hw, ALX_ISR, 0);

	return (FILTER_HANDLED);
}

static int
alx_alloc_intr(struct alx_softc *sc)
{
	device_t dev;
	driver_filter_t *filter;
	struct alx_hw *hw;
	uint32_t msi_ctrl;
	int rid, error, nmsi;

	dev = sc->alx_dev;
	hw = &sc->hw;

	rid = 0; /* For legacy INTx interrupts. */
	filter = alx_intr_legacy;

	if (alx_enable_msi) {
		nmsi = 1;
		msi_ctrl = FIELDX(ALX_MSI_RETRANS_TM, hw->imt >> 1);
		if (pci_alloc_msi(dev, &nmsi) == 0 && nmsi == 1) {
			rid = 1;
			ALX_MEM_W32(hw, ALX_MSI_RETRANS_TIMER,
			    msi_ctrl | ALX_MSI_MASK_SEL_LINE);
			ALX_FLAG_SET(sc, USING_MSI);
			filter = alx_intr_msi;
		} else
			device_printf(dev,
		    "could not allocate MSI message, falling back to INTx\n");
	}

	if (!ALX_FLAG(sc, USING_MSIX) && !ALX_FLAG(sc, USING_MSI))
		ALX_MEM_W32(hw, ALX_MSI_RETRANS_TIMER, 0);

	sc->nr_txq = 1;
	sc->nr_rxq = 1; // XXX needed?
	sc->nr_vec = 1;
	sc->nr_hwrxq = 1;

	sc->alx_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->alx_irq == NULL) {
		device_printf(dev, "cannot allocate IRQ\n");
		return (ENXIO);
	}

	error = bus_setup_intr(dev, sc->alx_irq, INTR_TYPE_NET | INTR_MPSAFE,
	    filter, NULL, sc, &sc->alx_cookie);
	if (error != 0) {
		device_printf(dev, "failed to register interrupt handler\n");
		return (ENXIO);
	}

	TASK_INIT(&sc->alx_int_task, 0, alx_int_task, sc);
	TASK_INIT(&sc->alx_link_task, 0, alx_link_task, sc);
	sc->alx_tq = taskqueue_create_fast("alx_taskq", M_WAITOK,
	    taskqueue_thread_enqueue, &sc->alx_tq);
	if (sc->alx_tq == NULL) {
		device_printf(dev, "could not create taskqueue\n");
		return (ENXIO);
	}
	taskqueue_start_threads(&sc->alx_tq, 1, PI_NET, "%s taskq",
	    device_get_nameunit(sc->alx_dev));

	return (0);
}

static void
alx_free_intr(struct alx_softc *sc)
{
	device_t dev;

	dev = sc->alx_dev;

	if (sc->alx_tq != NULL) {
		taskqueue_drain(sc->alx_tq, &sc->alx_int_task);
		taskqueue_drain(taskqueue_swi, &sc->alx_link_task);
		taskqueue_free(sc->alx_tq);
	}

	if (sc->alx_cookie != NULL)
		bus_teardown_intr(dev, sc->alx_irq, sc->alx_cookie);

	if (sc->alx_irq != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->alx_irq);

	if (ALX_FLAG(sc, USING_MSI))
		pci_release_msi(dev);
}

static int
alx_media_change(struct ifnet *ifp)
{

	return (0);
}

static void
alx_media_status(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct alx_softc *sc;
	struct alx_hw *hw;

	sc = ifp->if_softc;
	hw = &sc->hw;

	ALX_LOCK(sc);

	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;

	if (hw->link_up)
		ifmr->ifm_status |= IFM_ACTIVE;
	else {
		ALX_UNLOCK(sc);
		return;
	}

	switch (hw->link_duplex) {
	case ALX_FULL_DUPLEX:
		ifmr->ifm_active |= IFM_FDX;
		break;
	case ALX_HALF_DUPLEX:
		ifmr->ifm_active |= IFM_HDX;
		break;
	default:
		device_printf(sc->alx_dev, "invalid duplex mode %u\n",
		    hw->link_duplex);
		break;
	}

	switch (hw->link_speed) {
	case SPEED_10:
		ifmr->ifm_active |= IFM_10_T;
		break;
	case SPEED_100:
		ifmr->ifm_active |= IFM_100_TX;
		break;
	case SPEED_1000:
		ifmr->ifm_active |= IFM_1000_T;
		break;
	default:
		device_printf(sc->alx_dev, "invalid link speed %u\n",
		    hw->link_speed);
		break;
	}

	ALX_UNLOCK(sc);
}

static int
alx_ioctl(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct alx_softc *sc;
	struct ifreq *ifr;
	int error = 0;

	sc = ifp->if_softc;
	ifr = (struct ifreq *)data;

	switch (command) {
	case SIOCSIFFLAGS:
		ALX_LOCK(sc);
		if ((ifp->if_flags & IFF_UP) &&
		    !(ifp->if_flags & IFF_DRV_RUNNING)) {
			alx_init_locked(sc);
		} else {
			if (!(ifp->if_flags & IFF_DRV_RUNNING))
				alx_stop(sc);
		}
		sc->alx_if_flags = ifp->if_flags;
		ALX_UNLOCK(sc);
		break;
	case SIOCGIFMEDIA:
		error = ifmedia_ioctl(ifp, ifr, &sc->alx_media, command);
		break;
	default:
		error = ether_ioctl(ifp, command, data);
		break;
	}

	return (error);
}

static void
alx_init(void *arg)
{
	struct alx_softc *sc;

	sc = arg;
	ALX_LOCK(sc);
	alx_init_locked(sc);
	ALX_UNLOCK(sc);
}

static void
alx_init_locked(struct alx_softc *sc)
{
	struct ifnet *ifp;
	struct alx_hw *hw;

	ALX_LOCK_ASSERT(sc);

	ifp = sc->alx_ifp;
	hw = &sc->hw;

	if (ifp->if_drv_flags & IFF_DRV_RUNNING)
		return;

	alx_stop(sc);

	/* Reset to a known good state. */
	alx_reset(sc);

	memcpy(hw->mac_addr, IF_LLADDR(ifp), ETHER_ADDR_LEN);
	alx_set_macaddr(hw, hw->mac_addr);

	alx_init_rx_ring(sc);
	alx_init_tx_ring(sc);

	printf("rfd: 0x%lx, rrd: 0x%lx, txd: 0x%lx\n", sc->alx_rx_queue.rfd_dma,
	    sc->alx_rx_queue.rrd_dma, sc->alx_tx_queue.tpd_dma);

	/* Load the DMA pointers. */
	ALX_MEM_W32(hw, ALX_SRAM9, ALX_SRAM_LOAD_PTR);

	/*
	 * XXX configure some VLAN rx strip thingy and some promiscuous mode
	 * stuff and some multicast stuff.
	 */

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	alx_update_link(sc);

	ALX_MEM_W32(hw, ALX_ISR, (uint32_t)~ALX_ISR_DIS);
	alx_intr_enable(sc);
}

static void
alx_start(struct ifnet *ifp)
{
	struct alx_softc *sc = ifp->if_softc;

	if (!(ifp->if_drv_flags & IFF_DRV_RUNNING))
		return;

	ALX_LOCK(sc);
	alx_start_locked(ifp);
	ALX_UNLOCK(sc);
}

/*
 * Transmit all of the frames in our send queue.
 */
static void
alx_start_locked(struct ifnet *ifp)
{
	struct alx_softc *sc;
	struct mbuf *m_head;

	sc = ifp->if_softc;
	ALX_LOCK_ASSERT(sc);

	if ((ifp->if_drv_flags & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING || !sc->hw.link_up)
		return;

	while (!IFQ_DRV_IS_EMPTY(&ifp->if_snd)) {
		IFQ_DRV_DEQUEUE(&ifp->if_snd, m_head);
		if (m_head == NULL)
			break;

		if (alx_xmit(sc, &m_head)) {
			if (m_head != NULL)
				IFQ_DRV_PREPEND(&ifp->if_snd, m_head);
			break;
		}

		/* Let BPF listeners know about this frame. */
		ETHER_BPF_MTAP(ifp, m_head);
	}

	/* XXX start wdog */
}

static int
alx_probe(device_t dev)
{
	struct alx_dev *alx;
	uint16_t vendor, device;
	int i;

	vendor = pci_get_vendor(dev);
	device = pci_get_device(dev);

	alx = alx_devs;
	for (i = 0; i < sizeof(alx_devs) / sizeof(alx_devs[0]); i++, alx++) {
		if (alx->alx_vendorid == vendor &&
		    alx->alx_deviceid == device) {
			device_set_desc(dev, alx->alx_name);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static int
alx_attach(device_t dev)
{
	struct alx_softc *sc;
	struct alx_hw *hw;
	struct ifnet *ifp;
	bool phy_cfged;
	int error, rid;

	sc = device_get_softc(dev);
	sc->alx_dev = dev;

	mtx_init(&sc->alx_mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK,
	    MTX_DEF);

	rid = PCIR_BAR(0);
	sc->alx_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->alx_res == NULL) {
		device_printf(dev, "cannot allocate memory resources\n");
		return (ENXIO);
	}

	hw = &sc->hw;
	hw->hw_addr = sc->alx_res;
	hw->dev = dev;

#ifdef __notyet
	error = pci_set_powerstate(dev, PCI_POWERSTATE_D0);
	if (error != 0) {
		device_printf(dev, "failed to set PCI power state to D0\n");
		error = ENXIO;
		goto fail;
	}
#endif

	pci_enable_busmaster(dev);

	error = alx_init_sw(sc);
	if (error != 0) {
		device_printf(dev, "failed to initialize device softc\n");
		error = ENXIO;
		goto fail;
	}

	alx_reset_pcie(hw);

	phy_cfged = alx_phy_configed(hw);
	if (!phy_cfged)
		alx_reset_phy(hw, !hw->hib_patch);

	error = alx_reset_mac(hw);
	if (error) {
		device_printf(dev, "MAC reset failed with error %d\n", error);
		error = ENXIO;
		goto fail;
	}

	if (!phy_cfged) {
		error = alx_setup_speed_duplex(hw, hw->adv_cfg, hw->flowctrl);
		if (error) {
			device_printf(dev,
			    "failed to configure PHY with error %d\n", error);
			error = ENXIO;
			goto fail;
		}
	}

	error = alx_get_perm_macaddr(hw, hw->perm_addr);
	if (error != 0) {
		/* XXX Generate a random MAC address instead? */
		device_printf(dev, "could not retrieve MAC address\n");
		error = ENXIO;
		goto fail;
	}
	memcpy(&hw->mac_addr, &hw->perm_addr, ETHER_ADDR_LEN);

	alx_configure_basic(hw);

	error = alx_alloc_intr(sc);
	if (error != 0)
		goto fail;

	if (!alx_get_phy_info(hw)) {
		device_printf(dev, "failed to identify PHY\n");
		error = ENXIO;
		goto fail;
	}

	error = alx_dma_alloc(sc);
	if (error != 0) {
		device_printf(dev, "cannot initialize DMA mappings\n");
		error = ENXIO;
		goto fail;
	}

	sc->alx_ifp = if_alloc(IFT_ETHER);
	if (sc->alx_ifp == NULL) {
		device_printf(dev, "failed to allocate an ifnet\n");
		error = ENOSPC;
		goto fail;
	}

	ifp = sc->alx_ifp;
	ifp->if_softc = sc;
	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST; /* XXX */
	ifp->if_capabilities = 0; //IFCAP_HWCSUM; /* XXX others? */
	ifp->if_ioctl = alx_ioctl;
	ifp->if_start = alx_start;
	ifp->if_init = alx_init;

	ether_ifattach(ifp, hw->mac_addr);

	ifmedia_init(&sc->alx_media, IFM_IMASK, alx_media_change,
	    alx_media_status);
	ifmedia_add(&sc->alx_media, IFM_ETHER | IFM_AUTO, 0, NULL);
	ifmedia_add(&sc->alx_media, IFM_ETHER | IFM_10_T, 0, NULL);
	ifmedia_add(&sc->alx_media, IFM_ETHER | IFM_10_T | IFM_FDX, 0, NULL);
	ifmedia_add(&sc->alx_media, IFM_ETHER | IFM_100_TX, 0, NULL);
	ifmedia_add(&sc->alx_media, IFM_ETHER | IFM_100_TX | IFM_FDX, 0, NULL);
	if (ALX_CAP(hw, GIGA)) {
		ifmedia_add(&sc->alx_media, IFM_ETHER | IFM_1000_T, 0, NULL);
		ifmedia_add(&sc->alx_media, IFM_ETHER | IFM_1000_T | IFM_FDX, 0,
		    NULL);
	}
	ifmedia_set(&sc->alx_media, IFM_ETHER | IFM_AUTO);

	hw->mtu = sc->alx_ifp->if_mtu;
	sc->rxbuf_size = ALIGN(ALX_RAW_MTU(hw->mtu));
	printf("rxbuf size is %d\n", sc->rxbuf_size);
fail:
	if (error != 0)
		alx_detach(dev);

	return (error);
}

static int
alx_detach(device_t dev)
{
	struct alx_softc *sc;
	struct alx_hw *hw;

	sc = device_get_softc(dev);
	hw = &sc->hw;

	/* Restore permanent mac address. */
	alx_set_macaddr(hw, hw->perm_addr);

	/* XXX Free DMA */
	free(sc->alx_tx_queue.bf_info, M_DEVBUF);
	free(sc->alx_rx_queue.bf_info, M_DEVBUF);

	if (sc->alx_ifp != NULL) {
		ether_ifdetach(sc->alx_ifp);
		if_free(sc->alx_ifp);
		sc->alx_ifp = NULL;
	}

	alx_free_intr(sc);

	if (sc->alx_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, PCIR_BAR(0),
		    sc->alx_res);

	bus_generic_detach(dev);

	mtx_destroy(&sc->alx_mtx);

	return (0);
}

static int
alx_shutdown(device_t dev)
{

	return (alx_suspend(dev));
}

static int
alx_suspend(device_t dev)
{
	struct alx_softc *sc;
	struct alx_hw *hw;

	sc = device_get_softc(dev);
	hw = &sc->hw;

	ALX_LOCK(sc);
	alx_stop(sc);
	alx_clear_phy_intr(hw);
	ALX_UNLOCK(sc);

	return (bus_generic_suspend(dev));
}

static int
alx_resume(device_t dev)
{

	return (0);
}
