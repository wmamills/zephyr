/*
 * Copyright (c) 2021 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <devicetree.h>
#include <device.h>
#include <drivers/virtio/virtio.h>
#include <net/ethernet.h>
#include <sys/slist.h>
#include <logging/log.h>

#define DT_DRV_COMPAT virtio_net

#define LOG_MODULE_NAME virtio_net
LOG_LEVEL_SET(CONFIG_VIRTIO_NET_LOG_LEVEL);

#define DEV_CFG(dev) ((struct virtio_net_config*)(dev->config))
#define DEV_DATA(dev) ((struct virtio_net_data*)(dev->data))

#define VQIN_SIZE    4
#define RXDESC_COUNT 4
#define RXPOOL_SIZE  6

#if defined(CONFIG_VIRTIO_NET_ZEROCOPY_TX)
#define VQOUT_SIZE   32
#define TXDESC_COUNT 16
#else
#define VQOUT_SIZE   4
#define TXDESC_COUNT 4
#endif

#define VIRTIO_NET_F_MAC BIT(5)
#define VIRTIO_NET_F_MRG_RXBUF BIT(15)

struct virtio_net_hdr {
    uint8_t flags;
    uint8_t gso_type;
    uint16_t hdr_len;
    uint16_t gso_size;
    uint16_t csum_start;
    uint16_t csum_offset;
    uint16_t num_buffers;
};

struct virtio_net_rx_pkt {
    struct virtio_net_hdr hdr;
    uint8_t pkt[NET_ETH_MAX_FRAME_SIZE];
};

struct virtio_net_tx_pkt {
    struct virtio_net_hdr hdr;
#if defined(CONFIG_VIRTIO_NET_ZEROCOPY_TX)
    uint8_t pkt[0];
#else
    uint8_t pkt[NET_ETH_MAX_FRAME_SIZE];
#endif
};

struct virtio_net_rx_desc {
    sys_snode_t node;
#if defined(CONFIG_VIRTIO_NET_ZEROCOPY_RX)
    struct net_pkt *npkt;
#else
    struct virtio_net_rx_pkt *pkt;
    uint8_t *data;
#endif
};

struct virtio_net_tx_desc {
    sys_snode_t node;
    struct k_sem done_sem;
#if defined(CONFIG_VIRTIO_NET_ZEROCOPY_TX)
    struct net_pkt *npkt;
#endif
    struct virtio_net_tx_pkt *pkt;
#if !defined(CONFIG_VIRTIO_NET_ZEROCOPY_TX)
    uint8_t *data;
#endif
};

struct virtio_net_config {
    const struct device *bus;
    LOG_INSTANCE_PTR_DECLARE(log);
    int vq_count;
    struct virtqueue **vqs;
    struct virtio_net_tx_pkt *txbuf;
    struct virtio_net_tx_desc *txdesc;
#if defined(CONFIG_VIRTIO_NET_ZEROCOPY_RX)
    struct k_mem_slab *rxslab;
    struct net_buf_pool *rxpool;
#else
    struct virtio_net_rx_pkt *rxbuf;
#endif
    struct virtio_net_rx_desc *rxdesc;
};

struct virtio_net_data {
    const struct device *dev;
    struct net_if *iface;
    uint8_t mac_addr[6];
    struct virtqueue *vqin, *vqout;
    int hdrsize;
    sys_slist_t tx_free_list;
    sys_slist_t rx_free_list;
};

static void virtio_net_iface_init(struct net_if *iface);
static int virtio_net_send(const struct device *dev, struct net_pkt *pkt);

static void virtio_net_vqin_cb(void *arg);
static void virtio_net_vqout_cb(void *arg);
static void virtio_net_rx_refill(struct virtio_net_data *pdata);

static const struct ethernet_api virtio_net_api = {
    .iface_api.init = virtio_net_iface_init,
    .send = virtio_net_send,
};


#if defined(CONFIG_VIRTIO_NET_ZEROCOPY_RX)
#define CREATE_VIRTIO_NET_RXBUFS(inst) \
    NET_PKT_SLAB_DEFINE(rxslab_##inst, RXPOOL_SIZE);\
    NET_BUF_POOL_FIXED_DEFINE(rxpool_##inst, RXPOOL_SIZE,\
        sizeof(struct virtio_net_hdr) + NET_ETH_MAX_FRAME_SIZE, NULL);
#define SET_VIRTIO_NET_RXBUFS(inst) \
        .rxslab = &rxslab_##inst,\
        .rxpool = &rxpool_##inst,
#else
#define CREATE_VIRTIO_NET_RXBUFS(inst) \
    static struct virtio_net_rx_pkt rxbuf_##inst[RXDESC_COUNT];
#define SET_VIRTIO_NET_RXBUFS(inst) \
        .rxbuf = rxbuf_##inst,
#endif


#define CREATE_VIRTIO_NET_DEVICE(inst) \
    LOG_INSTANCE_REGISTER(LOG_MODULE_NAME, inst, CONFIG_VIRTIO_NET_LOG_LEVEL);\
    CREATE_VIRTIO_NET_RXBUFS(inst);\
    static struct virtio_net_rx_desc rxdesc_##inst[RXDESC_COUNT];\
    static struct virtio_net_tx_pkt txbuf_##inst[TXDESC_COUNT];\
    static struct virtio_net_tx_desc txdesc_##inst[TXDESC_COUNT];\
    VQ_DECLARE(vq0_##inst, VQIN_SIZE, 4096);\
    VQ_DECLARE(vq1_##inst, VQOUT_SIZE, 4096);\
    static struct virtqueue *vq_list_##inst[] = {VQ_PTR(vq0_##inst), VQ_PTR(vq1_##inst)};\
    static const struct virtio_net_config virtio_net_cfg_##inst = {\
        .bus = DEVICE_DT_GET(DT_BUS(DT_INST(inst, DT_DRV_COMPAT))),\
        LOG_INSTANCE_PTR_INIT(log, LOG_MODULE_NAME, inst)\
        .vq_count = 2,\
        .vqs = &vq_list_##inst[0],\
        SET_VIRTIO_NET_RXBUFS(inst)\
        .rxdesc = rxdesc_##inst,\
        .txbuf = txbuf_##inst,\
        .txdesc = txdesc_##inst,\
        };\
    static struct virtio_net_data virtio_net_data_##inst = {\
    };\
    ETH_NET_DEVICE_DT_INST_DEFINE(inst,\
        virtio_net_init,\
        NULL,\
        &virtio_net_data_##inst,\
        &virtio_net_cfg_##inst,\
        CONFIG_ETH_INIT_PRIORITY,\
        &virtio_net_api,\
        NET_ETH_MTU);

static int virtio_net_init(const struct device *dev)
{
    uint32_t devid, features;
    int i;
    struct virtio_device *vdev = virtio_get_vmmio_dev(DEV_CFG(dev)->bus);

    LOG_INST_DBG(DEV_CFG(dev)->log, "(%p)\n", dev);

    __ASSERT(DEV_CFG(dev)->bus != NULL, "DEV_CFG(dev)->bus != NULL");
    if (!device_is_ready(DEV_CFG(dev)->bus))
        return -1;

    LOG_INST_DBG(DEV_CFG(dev)->log, "bus %p\n", DEV_CFG(dev)->bus);
    devid = virtio_get_devid(vdev);
    if (devid != VIRTIO_ID_NETWORK)
        {
        LOG_INST_ERR(DEV_CFG(dev)->log, "Expected devid %04x, got %04x\n", VIRTIO_ID_NETWORK, devid);
        return -1;
        }
    DEV_DATA(dev)->dev = dev;
    virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
    virtio_set_features(vdev, VIRTIO_NET_F_MAC/*VIRTIO_F_NOTIFY_ON_EMPTY*/);
    features = virtio_get_features(vdev);

    LOG_INST_DBG(DEV_CFG(dev)->log, "features: %08x\n", features);

    DEV_DATA(dev)->vqin =
        virtio_mmio_setup_virtqueue(
            vdev,
            0,
            DEV_CFG(dev)->vqs[0],
            virtio_net_vqin_cb,
            (struct device *)dev,
            "nq1");
    if (!DEV_DATA(dev)->vqin)
        return -1;

    DEV_DATA(dev)->vqout =
        virtio_mmio_setup_virtqueue(
            vdev,
            1,
            DEV_CFG(dev)->vqs[1],
            virtio_net_vqout_cb,
            DEV_DATA(dev),
            "nq2");

    if (!DEV_DATA(dev)->vqout)
        return -1;

    virtio_mmio_register_device(vdev, DEV_CFG(dev)->vq_count, DEV_CFG(dev)->vqs);
    virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);

    DEV_DATA(dev)->hdrsize = sizeof(struct virtio_net_hdr);
    if (!(features & VIRTIO_NET_F_MRG_RXBUF))
        DEV_DATA(dev)->hdrsize -= 2;

    sys_slist_init(&DEV_DATA(dev)->rx_free_list);
    for (i = 0; i < RXDESC_COUNT; i++)
        {
        LOG_INST_DBG(DEV_CFG(dev)->log, "rx %d at %p\n",i,&DEV_CFG(dev)->rxdesc[i]);
#if defined(CONFIG_VIRTIO_NET_ZEROCOPY_RX)
        /* Nothing to do here */
#else
        DEV_CFG(dev)->rxdesc[i].pkt = &DEV_CFG(dev)->rxbuf[i];
        if (features & VIRTIO_NET_F_MRG_RXBUF)
            DEV_CFG(dev)->rxdesc[i].data = DEV_CFG(dev)->rxbuf[i].pkt;
        else
            DEV_CFG(dev)->rxdesc[i].data = (uint8_t*)&DEV_CFG(dev)->rxbuf[i].hdr.num_buffers;
#endif
        sys_slist_append(&DEV_DATA(dev)->rx_free_list, &DEV_CFG(dev)->rxdesc[i].node);
        }
    virtio_net_rx_refill(DEV_DATA(dev));
    sys_slist_init(&DEV_DATA(dev)->tx_free_list);
    for (i = 0; i < TXDESC_COUNT; i++)
        {
        LOG_INST_DBG(DEV_CFG(dev)->log, "tx %d at %p\n",i,&DEV_CFG(dev)->txdesc[i]);
        DEV_CFG(dev)->txdesc[i].pkt = &DEV_CFG(dev)->txbuf[i];
        k_sem_init(&DEV_CFG(dev)->txdesc[i].done_sem, 0, 1);

#if !defined(CONFIG_VIRTIO_NET_ZEROCOPY_TX)
        if (features & VIRTIO_NET_F_MRG_RXBUF)
            DEV_CFG(dev)->txdesc[i].data = DEV_CFG(dev)->txbuf[i].pkt;
        else
            DEV_CFG(dev)->txdesc[i].data = (uint8_t*)&DEV_CFG(dev)->txbuf[i].hdr.num_buffers;
#endif
        sys_slist_append(&DEV_DATA(dev)->tx_free_list, &DEV_CFG(dev)->txdesc[i].node);
        }
    virtqueue_kick(DEV_DATA(dev)->vqin);
    virtqueue_kick(DEV_DATA(dev)->vqout);

    if (VIRTIO_NET_F_MAC & features)
        virtio_read_config(vdev, 0, DEV_DATA(dev)->mac_addr, 6);
    else
        {
        __ASSERT(0, "should generate a MAC address");
        }

    return 0;
}

static void virtio_net_iface_init(struct net_if *iface)
{
    const struct device *dev = net_if_get_device(iface);

    LOG_INST_DBG(DEV_CFG(dev)->log, "(%p)\n", iface);

    DEV_DATA(dev)->iface = iface;
    net_if_set_link_addr(iface, DEV_DATA(dev)->mac_addr, 6, NET_LINK_ETHERNET);
    ethernet_init(iface);
    //net_if_flag_set(iface, NET_IF_NO_AUTO_START);
}

static int virtio_net_send(const struct device *dev, struct net_pkt *pkt)
{
    sys_snode_t *node;
    struct virtio_net_tx_desc *desc;
    uint16_t total_len;
    int key;
    int ret = -EIO;
#if defined(CONFIG_VIRTIO_NET_ZEROCOPY_TX)
    struct virtqueue_buf vqb[32];
    size_t vqblen = 1;
    struct net_buf *frag;
#endif

    //net_if_tx() checks if pkt is non-NULL
    total_len = net_pkt_get_len(pkt);
    LOG_INST_DBG(DEV_CFG(dev)->log, "(%d)\n", total_len);
    if ((total_len > NET_ETH_MAX_FRAME_SIZE) || (total_len == 0))
        {
        return -EINVAL;
        }

    key = irq_lock();
    /* VQ callback can't access the free list if interrupts are locked */
    node = sys_slist_get(&DEV_DATA(dev)->tx_free_list);
    irq_unlock(key);
    if (!node)
        return ret;
    desc = SYS_SLIST_CONTAINER(node, desc, node);
    LOG_INST_DBG(DEV_CFG(dev)->log, "desc=%p\n", desc);
    memset(&desc->pkt->hdr, 0, sizeof(desc->pkt->hdr));
#if defined(CONFIG_VIRTIO_NET_ZEROCOPY_TX)
    desc->npkt = pkt;
    vqb[0].buf = &desc->pkt->hdr;
    vqb[0].len = DEV_DATA(dev)->hdrsize;
    for (frag = pkt->frags; frag ; frag = frag->frags, vqblen++) {
            if (vqblen >= 32)
                {
                LOG_INST_ERR(DEV_CFG(dev)->log, " vqblen %ld\n", vqblen);
                goto recycle;
                }
            vqb[vqblen].buf = frag->data;
            vqb[vqblen].len = frag->len;
            LOG_INST_DBG(DEV_CFG(dev)->log, " vqblen %ld len %d\n", vqblen, frag->len);

        }
    if (virtqueue_add_buffer(DEV_DATA(dev)->vqout, &vqb[0], vqblen, 0, (void*)desc))
        goto recycle;
#else /* CONFIG_VIRTIO_NET_ZEROCOPY_TX */
    if (net_pkt_read(pkt, desc->data, total_len))
        {
            LOG_INST_WRN(DEV_CFG(dev)->log, "Failed to read packet into buffer");
            goto recycle;
        }
    /* should not happen, VQ size == desc count and we use one VQ entry per desc, maybe __ASSERT() */
    /* maybe when doin scatter-gather this will change */
    struct virtqueue_buf vb[1] = {{.buf = desc->pkt, .len = total_len + DEV_DATA(dev)->hdrsize} };
    if (virtqueue_add_buffer(DEV_DATA(dev)->vqout, vb, 1, 0, (void*)desc))
        goto recycle;
#endif /* CONFIG_VIRTIO_NET_ZEROCOPY_TX */
    virtqueue_kick(DEV_DATA(dev)->vqout);
    k_sem_take(&desc->done_sem, K_FOREVER);
    return 0;

recycle:
    key = irq_lock();
    /* VQ callback can't access the free list if interrupts are locked */
    sys_slist_append(&DEV_DATA(dev)->tx_free_list, &desc->node);
    irq_unlock(key);
    return ret;
}

static void virtio_net_vqout_cb(void *arg)
{
    struct virtio_net_data *pdata = arg;
    struct virtio_net_tx_desc *desc;
    while ((desc = virtqueue_get_buffer(pdata->vqout, NULL, NULL)))
        {
#if !defined(CONFIG_LOG_MODE_IMMEDIATE) && !defined(CONFIG_LOG2_MODE_IMMEDIATE)
        LOG_INST_DBG(DEV_CFG(pdata->dev)->log, "dequeued %p\n", desc);
#endif
        sys_slist_append(&pdata->tx_free_list, &desc->node);
        k_sem_give(&desc->done_sem);
        }
}

#if defined (CONFIG_VIRTIO_NET_ZEROCOPY_RX)
static void virtio_net_vqin_cb(void *arg)
{
    const struct device *dev = arg;
    struct virtio_net_rx_desc *desc;
    int length;

    while ((desc = virtqueue_get_buffer(DEV_DATA(dev)->vqin, &length, NULL)))
        {
        length -= DEV_DATA(dev)->hdrsize;
#if !defined(CONFIG_LOG_MODE_IMMEDIATE) && !defined(CONFIG_LOG2_MODE_IMMEDIATE)
        LOG_INST_DBG(DEV_CFG(dev)->log, "dequeued %p len=%d\n", desc, length);
#endif
        desc->npkt->frags->len = length;
        desc->npkt->frags->data += DEV_DATA(dev)->hdrsize;
        if (net_recv_data(DEV_DATA(dev)->iface, desc->npkt) < 0)
            {
#if !defined(CONFIG_LOG_MODE_IMMEDIATE) && !defined(CONFIG_LOG2_MODE_IMMEDIATE)
            LOG_INST_WRN(DEV_CFG(dev)->log, "net_recv_data() failed");
#endif
            net_pkt_unref(desc->npkt);
            }

        sys_slist_append(&DEV_DATA(dev)->rx_free_list, &desc->node);
        }
    virtio_net_rx_refill(DEV_DATA(dev));
}

static void virtio_net_rx_refill(struct virtio_net_data *pdata)
{
    /* This is called from .init once and then from VQ callback only so no locking. For now. */
    while (!virtqueue_full(pdata->vqin))
        {
        struct net_buf *buf = net_buf_alloc_fixed(DEV_CFG(pdata->dev)->rxpool, K_NO_WAIT);
        if (buf == NULL)
            {
            break;
            }
        struct net_pkt *pkt = net_pkt_alloc_from_slab(DEV_CFG(pdata->dev)->rxslab, K_NO_WAIT);
        __ASSERT(pkt, "should have one net_pkt per net_buf");

        sys_snode_t *node = sys_slist_get(&pdata->rx_free_list);
        struct virtio_net_rx_desc *desc;
        if (!node)
            {
            net_buf_unref(buf);
            net_pkt_unref(pkt);
            __ASSERT(desc, "should have one descriptor per VQ buffer");
            break;
            }
        desc = SYS_SLIST_CONTAINER(node, desc, node);
        net_buf_reset(buf);
        desc->npkt = pkt;
        pkt->frags = buf;

        memset(buf->data, 0, pdata->hdrsize);
        if (virtqueue_enqueue_buf(pdata->vqin, (void*)desc, 1, buf->data, pdata->hdrsize + NET_ETH_MTU))
            {
            __ASSERT(0, "should have one descriptor per VQ buffer. this should really never happen");
            net_buf_unref(buf);
            net_pkt_unref(pkt);
            sys_slist_append(&pdata->rx_free_list, &desc->node);
            break;
            }
        }
}

#else /* defined(CONFIG_VIRTIO_NET_ZEROCOPY_RX) */
static void virtio_net_vqin_cb(void *arg)
{
    const struct device *dev = arg;
    struct virtio_net_rx_desc *desc;
    int length;
    struct net_pkt *pkt;

    while ((desc = virtqueue_get_buffer(DEV_DATA(dev)->vqin, &length, NULL)))
        {
        length -= DEV_DATA(dev)->hdrsize;
#if !defined(CONFIG_LOG_MODE_IMMEDIATE) && !defined(CONFIG_LOG2_MODE_IMMEDIATE)
        LOG_INST_DBG(DEV_CFG(dev)->log, "dequeued %p len=%d\n", desc, length);
#endif
        pkt = net_pkt_rx_alloc_with_buffer(DEV_DATA(dev)->iface, length, AF_UNSPEC, 0, K_NO_WAIT);
        if (pkt == NULL)
            {
#if !defined(CONFIG_LOG_MODE_IMMEDIATE) && !defined(CONFIG_LOG2_MODE_IMMEDIATE)
            LOG_INST_WRN(DEV_CFG(dev)->log, "packet allocation failure");
#endif
            }
        else
            {
            net_pkt_write(pkt, desc->data, length);
            if (net_recv_data(DEV_DATA(dev)->iface, pkt) < 0)
                {
#if !defined(CONFIG_LOG_MODE_IMMEDIATE) && !defined(CONFIG_LOG2_MODE_IMMEDIATE)
                LOG_INST_WRN(DEV_CFG(dev)->log, "net_recv_data() failed");
#endif
                net_pkt_unref(pkt);
                }
            }
        sys_slist_append(&DEV_DATA(dev)->rx_free_list, &desc->node);
        }
    virtio_net_rx_refill(DEV_DATA(dev));
}

static void virtio_net_rx_refill(struct virtio_net_data *pdata)
{
    /* This is called from .init once and then from VQ callback only so no locking. For now. */
    while (!virtqueue_full(pdata->vqin))
        {
        sys_snode_t *node = sys_slist_get(&pdata->rx_free_list);
        struct virtio_net_rx_desc *desc;
        if (!node)
            {
            __ASSERT(desc, "should have one descriptor per VQ buffer");
            break;
            }
        desc = SYS_SLIST_CONTAINER(node, desc, node);
        memset(&desc->pkt->hdr, 0, sizeof(desc->pkt->hdr));
        struct virtqueue_buf vb[1] = {{.buf = desc->pkt, .len = pdata->hdrsize + NET_ETH_MTU} };
        if (virtqueue_add_buffer(pdata->vqin, vb, 0, 1, (void*)desc))
            {
            __ASSERT(0, "should have one descriptor per VQ buffer. this should really never happen");
            sys_slist_append(&pdata->rx_free_list, &desc->node);
            break;
            }
        }
}

#endif /* defined(CONFIG_VIRTIO_NET_ZEROCOPY_RX) */

DT_INST_FOREACH_STATUS_OKAY(CREATE_VIRTIO_NET_DEVICE)
