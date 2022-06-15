/*
 * Copyright (c) 2021 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr.h>
#include <openamp/open_amp.h>
#include <openamp/virtqueue.h>
#include <openamp/virtio.h>
#include <metal/device.h>
#include <devicetree.h>
#include <device.h>
#include <drivers/virtio/virtio.h>
#include <linker/linker-defs.h>

#define DT_DRV_COMPAT virtio_mmio

struct virtio_mmio_config {
    void (*irq_config)(const struct device *dev);
};

#define DEV_CFG(dev) ((struct virtio_mmio_config *)(dev->config))
#define DEV_DATA(dev) ((struct virtio_mmio_device *)(dev->data))

static int virtio_mmio_init(const struct device *dev);
struct virtio_device* virtio_mmio_get_virtio_device(const struct device *dev) {
    return &DEV_DATA(dev)->vdev;
}

static const struct virtio_driver_api virtio_mmio_api = {
    .get_virtio_device = virtio_mmio_get_virtio_device,
};

#define CREATE_VIRTIO_MMIO_DEVICE(inst)                              \
    static struct virtio_mmio_device virtio_mmio_data_##inst = {       \
        .vdev = { \
            .priv = &virtio_mmio_data_##inst, \
        }, \
        .irq = DT_INST_IRQN(inst),                               \
        .device_mode = !DT_PROP(DT_INST(inst, DT_DRV_COMPAT), driver_mode),   \
        .cfg_mem = { \
            .base = (uint8_t *)DT_INST_REG_ADDR(inst),                   \
            .size = (unsigned int)DT_INST_REG_SIZE(inst),              \
        }, \
        .shm_mem = { \
            .base = (uint8_t *)DT_REG_ADDR(DT_PHANDLE(DT_INST(inst, DT_DRV_COMPAT), memory_region)), \
            .size = (unsigned int)LINKER_DT_RESERVED_MEM_GET_SIZE(DT_PHANDLE(DT_INST(inst, DT_DRV_COMPAT), memory_region)), \
        }, \
        .shm_device = {                                      \
            .name = DT_PROP(DT_INST(inst, DT_DRV_COMPAT), label),                         \
            .bus = NULL,                                     \
            .num_regions = 2,                                \
            {                                                \
                {                                            \
                    .virt       = (void *) NULL,         \
                    .physmap    = NULL,                  \
                    .size       = 0,                     \
                    .page_shift = 0xffffffffffffffff,    \
                    .page_mask  = 0xffffffffffffffff,    \
                    .mem_flags  = 0,                     \
                    .ops        = { NULL },              \
                },                                           \
                {                                            \
                    .virt       = (void *) NULL,         \
                    .physmap    = NULL,                  \
                    .size       = 0,                     \
                    .page_shift = 0xffffffffffffffff,    \
                    .page_mask  = 0xffffffffffffffff,    \
                    .mem_flags  = 0,                     \
                    .ops        = { NULL },              \
                },                                           \
            },                                               \
            .node = { NULL },                                \
            .irq_num = 0,                                    \
            .irq_info = NULL                                 \
        }                                                    \
    };                                                               \
    static void irq_config_##inst(const struct device *dev)          \
    {\
        IRQ_CONNECT(DT_INST_IRQ(inst, irq),\
                    DT_INST_IRQ(inst, priority),\
                    virtio_mmio_isr,\
                    &virtio_mmio_data_##inst.vdev,\
                    0);\
        irq_enable(DT_INST_IRQ(inst, irq));\
    }\
    static struct virtio_mmio_config virtio_mmio_cfg_##inst = {      \
        .irq_config = irq_config_##inst,                             \
    };                                                               \
    DEVICE_DT_INST_DEFINE(inst,                                      \
                          virtio_mmio_init,                          \
                          NULL,                                      \
                          &virtio_mmio_data_##inst,                  \
                          &virtio_mmio_cfg_##inst,                   \
                          /*POST_KERNEL*/PRE_KERNEL_1,                               \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE,        \
                          &virtio_mmio_api);

DT_INST_FOREACH_STATUS_OKAY(CREATE_VIRTIO_MMIO_DEVICE)

static int virtio_mmio_init(const struct device *dev)
{
    uintptr_t sram0_addr;
    uintptr_t virt_mem_ptr;
    uintptr_t cfg_mem_ptr;
    struct virtio_mmio_device *vmdev = DEV_DATA(dev);

    /* Map config */
    cfg_mem_ptr = (uintptr_t)vmdev->cfg_mem.base;
    device_map(&cfg_mem_ptr, (uintptr_t)vmdev->cfg_mem.base,
               vmdev->cfg_mem.size, K_MEM_CACHE_NONE);

    //TODO: rework detection of full virtualization mode
    //For now guest mode implies that the memory-region configured in the DTS
    //is check if the memory region sram0, e.g. memory-region = <&sram0>;

    virt_mem_ptr = (uintptr_t)vmdev->shm_mem.base;
    sram0_addr = (uintptr_t)DT_REG_ADDR(DT_NODELABEL(sram0));

    if (sram0_addr == (uintptr_t)vmdev->shm_mem.base) {
        //memory already mapped
        virt_mem_ptr = (uintptr_t)Z_MEM_VIRT_ADDR((uintptr_t)vmdev->shm_mem.base);
    } else {
        /* Map dedicated mem region */
        device_map(&virt_mem_ptr, (uintptr_t)vmdev->shm_mem.base,
                    vmdev->shm_mem.size, K_MEM_CACHE_NONE);
    }

    if (virtio_mmio_device_init(vmdev, virt_mem_ptr, cfg_mem_ptr, (void *)dev))
        return -1;

    metal_log(METAL_LOG_DEBUG, "device %s @%p\n", dev->name, dev);
    metal_log(METAL_LOG_DEBUG, "iobase %lx\n", cfg_mem_ptr);

    DEV_CFG(dev)->irq_config(dev);

    return 0;
}



