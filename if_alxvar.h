/*
 * Copyright (c) 2012 Qualcomm Atheros, Inc.
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

#ifndef _ALX_H_
#define _ALX_H_

#define ALX_WATCHDOG_TIME   (5 * HZ)

/*
 * alx_ring_header is a single, contiguous block of memory space
 * used by the three descriptor rings (tpd, rfd, rrd)
 */
struct alx_ring_header {
	/* virt addr */
	void		*desc;
	/* phy addr */
	bus_dmamap_t	 dma;
	uint32_t	 size;
};

struct alx_buffer {
	struct mbuf	*m;
	bus_dmamap_t	 dmamap;

	/* buffer size */
	DEFINE_DMA_UNMAP_LEN(size);
	/* information of this buffer */
	uint16_t	flags;
};
#define ALX_BUF_TX_FIRSTFRAG	0x1

/* rx queue */
struct alx_rx_queue {
	/* rrd ring virtual addr */
	struct rrd_desc *rrd_hdr;
	/* rrd ring physical addr */
	dma_addr_t rrd_dma;
	/* rfd ring virtual addr */
	struct rfd_desc *rfd_hdr;
	/* rfd ring physical addr */
	dma_addr_t rfd_dma;
	/* info for rx-skbs */
	struct alx_buffer *bf_info;

	/* number of ring elements */
	uint16_t count;
	/* rfd producer index */
	uint16_t pidx;
	/* rfd consumer index */
	uint16_t cidx;
	uint16_t rrd_cidx;
	/* register saving producer index */
	uint16_t p_reg;
	/* register saving consumer index */
	uint16_t c_reg;
	/* queue index */
	uint16_t qidx;
	unsigned long flag;

#ifdef notyet
	struct sk_buff_head list;
#endif
};
#define ALX_RQ_USING		1
#define ALX_RX_ALLOC_THRESH	32

/* tx queue */
struct alx_tx_queue {
	/* tpd ring virtual addr */
	struct tpd_desc *tpd_hdr;
	/* tpd ring physical address. */
	dma_addr_t tpd_dma;
	/* info for tx-skbs pending on HW */
	struct alx_buffer *bf_info;
	/* number of ring elements  */
	uint16_t count;
	/* producer index */
	uint16_t pidx;
	/* consumer index */
	atomic_t cidx;
	/* register saving producer index */
	uint16_t p_reg;
	/* register saving consumer index */
	uint16_t c_reg;
	/* queue index */
	u16 qidx;
};

#define ALX_TX_WAKEUP_THRESH(_tq) ((_tq)->count / 4)
#define ALX_DEFAULT_TX_WORK		128

enum ALX_FLAGS {
	ALX_FLAG_USING_MSIX = 0,
	ALX_FLAG_USING_MSI,
	ALX_FLAG_RESETING,
	ALX_FLAG_TESTING,
	ALX_FLAG_HALT,
	ALX_FLAG_FPGA,
	ALX_FLAG_TASK_PENDING,
	ALX_FLAG_TASK_CHK_LINK,
	ALX_FLAG_TASK_RESET,
	ALX_FLAG_TASK_UPDATE_SMB,

	ALX_FLAG_NUMBER_OF_FLAGS,
};


struct alx_hw;
/*
 *board specific private data structure
 */
struct alx_softc {
	struct alx_hw		hw;

	u16		bd_number;

	/* totally msix vectors */
	int			nr_vec;
	struct msix_entry	*msix_ent;

	/* all descriptor memory */
	struct alx_ring_header	ring_header;
	int			tx_ringsz;
	int			rx_ringsz;
	int			rxbuf_size;

#ifdef notyet
	struct alx_napi		*qnapi[8];
#endif
	/* number of napi for TX-Q */
	int			nr_txq;
	/* number of napi for RX-Q */
	int			nr_rxq;
	/* number independent hw RX-Q */
	int			nr_hwrxq;

#ifdef notyet
	/* lock for updating stats */
	spinlock_t		smb_lock;

	struct net_device_stats net_stats;
#endif
	atomic_t		irq_sem;
	u16			msg_enable;

	unsigned long		flags;

	/* ethtool private flags */
	u32			eth_pflags;
	int			eth_diag_vect;
	int			eth_diag_cnt;

	/* FreeBSD stuff is below. */
	device_t		 alx_dev;
	device_t		 alx_miibus;
	int			 alx_phyaddr;

	struct resource		*alx_res;
	struct resource		*alx_irq;
	void			*alx_cookie;
        struct ifnet		*alx_ifp;
	int			 alx_if_flags;

	struct taskqueue	*alx_tq;
	struct task		 alx_int_task;
        struct task		 alx_link_task;

	bus_dma_tag_t		 alx_parent_tag;

	bus_dma_tag_t		 alx_tx_tag;
	bus_dmamap_t		 alx_tx_dmamap;

	bus_dma_tag_t		 alx_tx_buf_tag;
	bus_dma_tag_t		 alx_rx_buf_tag;

	bus_dma_tag_t		 alx_rx_tag;
	bus_dmamap_t		 alx_rx_dmamap;

	bus_dma_tag_t		 alx_rr_tag;
	bus_dmamap_t		 alx_rr_dmamap;

	struct alx_tx_queue	 alx_tx_queue;
	struct alx_rx_queue	 alx_rx_queue;

	struct mtx		 alx_mtx;
};

#define ALX_LOCK(sc)		mtx_lock(&(sc)->alx_mtx)
#define ALX_UNLOCK(sc)		mtx_unlock(&(sc)->alx_mtx)
#define ALX_LOCK_ASSERT(sc)	mtx_assert(&(sc)->alx_mtx, MA_OWNED)

#define ALX_FLAG(_adpt, _FLAG) (\
	test_bit(ALX_FLAG_##_FLAG, &(_adpt)->flags))
#define ALX_FLAG_SET(_adpt, _FLAG) (\
	set_bit(ALX_FLAG_##_FLAG, &(_adpt)->flags))
#define ALX_FLAG_CLEAR(_adpt, _FLAG) (\
	clear_bit(ALX_FLAG_##_FLAG, &(_adpt)->flags))

#define ALX_TX_INC(i, s)	(((i) + 1) % (s)->tx_ringsz)
#define ALX_TX_DEC(i, s)	(((i) + (s)->tx_ringsz - 1) % (s)->tx_ringsz)

#ifdef notyet
static inline struct
alx_rx_queue *alx_hw_rxq(struct alx_rx_queue *rxq)
{
	struct alx_adapter *adpt = netdev_priv(rxq->netdev);

	return (ALX_CAP(&adpt->hw, MRQ) ? rxq : adpt->qnapi[0]->rxq);
}

/* needed by alx_ethtool.c */
extern void alx_configure(struct alx_adapter *adpt);
extern void alx_free_all_ring_resources(struct alx_adapter *adpt);
extern int alx_setup_all_ring_resources(struct alx_adapter *adpt);
extern void alx_init_def_rss_idt(struct alx_adapter *adpt);
extern int alx_alloc_rxring_buf(struct alx_adapter *adpt,
				struct alx_rx_queue *rxq);
extern void alx_init_intr(struct alx_adapter *adpt);
extern void alx_disable_advanced_intr(struct alx_adapter *adpt);
extern void alx_reinit(struct alx_adapter *adpt, bool in_task);
extern void alx_set_ethtool_ops(struct net_device *dev);
extern char alx_drv_name[];
extern char alx_drv_version[];
#endif

#endif
