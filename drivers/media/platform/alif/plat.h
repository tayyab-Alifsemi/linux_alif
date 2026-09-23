/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Alif CPI Host platform device driver
 *
 * Copyright (C) 2026 Alif Semiconductor
 * Author: Harith George <harith.g@alifsemi.com>
 *
 * Based on code from Synopsys, Inc.
 */

#ifndef __CPI_PLAT_H_
#define __CPI_PLAT_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/videobuf2-v4l2.h>
#include <media/dwc/dw-mipi-csi-pltfrm.h>

#define CSI_OF_NODE_NAME	"csi2"

#define to_plat_csi_pipeline(_ep) container_of(_ep, struct plat_csi_pipeline, ep)

#define N_BUFFERS 4
#define ALIF_MAX_CSI_SENSORS 2

int pipeline_set_format(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state,
			struct v4l2_subdev_format *fmt);

enum plat_csi_subdev_index {
	IDX_SENSOR1,
	IDX_SENSOR2,
	IDX_CSI,
	IDX_CPI,
	IDX_VDEV,
	IDX_MAX,
};

enum cpi_isp_port {
	PORT_INPUT = 0,
	PORT_OUTPUT,
	PORT_MAX = PORT_OUTPUT,
};

/**
 * struct plat_csi_sensor_info - image data source subdev information
 * @pdata: sensor's attributes passed as media device's platform data
 * @asd: asynchronous subdev registration data structure
 * @subdev: image sensor v4l2 subdev
 * @host: csi device the sensor is currently linked to
 *
 * This data structure applies to image sensor and the writeback subdevs.
 */
struct plat_csi_sensor_info {
	struct v4l2_subdev *subdev;
	struct v4l2_fwnode_endpoint cmos_ep_config;
};

/**
 * This structure represents a chain of media entities, including a data
 * source entity (e.g. an image sensor subdevice), a data capture entity
 * - a video capture device node and any remaining entities.
 */
struct plat_csi_pipeline {
	struct plat_csi_media_pipeline ep;
	struct list_head list;
	struct media_entity *vdev_entity;
	struct v4l2_subdev *subdevs[IDX_MAX];
};

struct rx_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	dma_addr_t dma_addr;
	void *cpu_addr;
};

struct dmaqueue {
	struct list_head active;
	wait_queue_head_t wq;
};

struct cpi_dev {
	u32 version;
	struct platform_device *pdev;
	int irq;
	struct mutex lock; /* serialises ioctl calls */
	spinlock_t slock; /* protects buffer list and active pointer */
	struct plat_csi_video_entity ve;
	struct v4l2_format format;
	const struct plat_csi_fmt *fmt;

	struct plat_csi_sensor_info sensor[PLAT_MAX_SENSORS];
	struct v4l2_fwnode_endpoint cmos_ep_config[ALIF_MAX_CSI_SENSORS];
	int num_sensors;
	/* Buffer and DMA */
	struct vb2_queue vb_queue;

	int sequence;
	struct list_head fb_list_head;
	struct rx_buffer *active;

	void *bounce_cpu;
	dma_addr_t bounce_dma;
	size_t bounce_size;

	struct media_pad vd_pad;
	struct media_pad subdev_pads[CPI_PADS_NUM];
	struct v4l2_subdev subdev;
	struct v4l2_subdev *remote_sd;
	u32 remote_pad;
	u64 enabled_pad_mask;
	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct device *dev;
	struct v4l2_async_notifier subdev_notifier;
	struct media_graph link_setup_graph;
	struct list_head pipelines;
	void __iomem *base_addr;

	struct clk *apb_clk;
	struct clk *pix_clk;
	bool pix_clk_enabled;

	bool code_10_on_8;
	bool msb;
	bool pclk_active;
	/*
	 * Capture data only when both VSYNC & HSYNC are high
	 */
	bool vsync_en;

	/*
	 * Start to capture video frame on the rising edge of VSYNC.
	 * If VSYNC & HSYNC are aligned, we can begin capture without
	 *     waiting for VSYNC.
	 * Else, we start capture on rising edge of VSYNC.
	 */
	bool wait_vsync_event;

	bool is_parallel_interface;
	bool axi_bus_ep;
	bool is_isp_connected;
	bool in_pipeline_propagation;
	bool streaming;
	bool stopping;
};

static inline struct cpi_dev *
entity_to_plat_csi_mdev(struct media_entity *me)
{
	return !me->graph_obj.mdev ? NULL :
		container_of(me->graph_obj.mdev, struct cpi_dev, media_dev);
}

static inline struct cpi_dev *
notifier_to_plat_csi(struct v4l2_async_notifier *n)
{
	return container_of(n, struct cpi_dev, subdev_notifier);
}

static inline void plat_csi_graph_unlock(struct plat_csi_video_entity *ve)
{
	mutex_unlock(&ve->vdev.entity.graph_obj.mdev->graph_mutex);
}

int plat_csi_remove(struct platform_device *pdev);
int plat_csi_probe(struct platform_device *pdev, struct cpi_dev *cpi);
#endif	/* __CPI_PLAT_H_ */
