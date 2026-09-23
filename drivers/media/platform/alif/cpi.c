// SPDX-License-Identifier: GPL-2.0-only
/*
 * Alif Camera Controller CPI device driver
 *
 * Copyright (C) 2026 Alif Semiconductor
 * Author: Harith George <harith.g@alifsemi.com>
 *
 * Based on sample code from Synopsys
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/of_reserved_mem.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/videodev2.h>
#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-dma-sg.h>
#include <media/videobuf2-vmalloc.h>
#include <media/v4l2-subdev.h>

#include "plat.h"
#include "cpi.h"

/* Registers offset for CPI */
#define CPI_CTRL		0x00 /* Control register */
#define CPI_INTR		0x04 /* Interrupt Status register */
#define CPI_INTR_ENA		0x08 /* Interrupt Enable register */
#define CPI_CFG			0x10 /* Configuration register */
#define CPI_FIFO_CTRL		0x14 /* FIFO Control register */
#define CPI_AXI_ERR_STAT	0x18 /* AXI Error Status register */
#define CPI_VIDEO_FCFG		0x28 /* Video Frame Config register */
#define CPI_CSI_CMCFG		0x2C /* MIPI CSI Color Mode Config */
#define CPI_FRAME_ADDR		0x30 /* Video Frame Start Address */

#define CTRL_FIFO_CLK_SEL	BIT(12)
#define CTRL_SW_RESET		BIT(8)
#define CTRL_SNAPSHOT		BIT(4)
#define CTRL_BUSY		BIT(2)
#define CTRL_START		BIT(0)

#define INTR_HSYNC		BIT(20)
#define INTR_VSYNC		BIT(16)
#define INTR_BRESP_ERR		BIT(6)
#define INTR_OUTFIFO_OVERRUN	BIT(5)
#define INTR_INFIFO_OVERRUN	BIT(4)
#define INTR_STOP		BIT(0)

#define CFG_MIPI_CSI			BIT(0)
#define CFG_CSI_HALT_EN			BIT(1)
#define CFG_AXI_PORT_EN			BIT(2)
#define CFG_ISP_PORT_EN			BIT(3)
#define CFG_WAIT_VSYNC			BIT(4)
#define CFG_VSYNC_EN			BIT(5)
#define CFG_ROW_ROUNDUP			BIT(8)
#define CFG_PCLK_POL			BIT(12)
#define CFG_HSYNC_POL			BIT(13)
#define CFG_VSYNC_POL			BIT(14)
#define CFG_MSB				BIT(20)
#define CFG_CODE10ON8			BIT(24)
#define CFG_DATA_MASK			GENMASK(1, 0)
#define CFG_DATA_MASK_SHIFT		28
#define CFG_DATA_MODE_MASK		GENMASK(2, 0)
#define CFG_DATA_MODE_MASK_SHIFT	16

#define FIFO_RD_WMARK_MASK		GENMASK(4, 0)
#define FIFO_RD_WMARK_SHIFT		0
#define FIFO_WR_WMARK_MASK		GENMASK(4, 0)
#define FIFO_WR_WMARK_SHIFT		8

#define CPI_FIFO_RD_WMARK_DEFAULT	0x8
#define CPI_FIFO_WR_WMARK_DEFAULT	0x18

#define CPI_BUSY_POLL_USEC	10
#define CPI_BUSY_TIMEOUT_USEC	20000
static const struct plat_csi_fmt cpi_formats[] = {
	{
		.name = "BGR888",
		.fourcc = V4L2_PIX_FMT_BGR24,
		.depth = 24,
		.mbus_code = MEDIA_BUS_FMT_RGB888_2X12_LE,
	}, {
		.name = "RGB565",
		.fourcc = V4L2_PIX_FMT_RGB565,
		.depth = 16,
		.mbus_code = MEDIA_BUS_FMT_RGB565_2X8_BE,
	}, {
		.name = "GREY",
		.fourcc = V4L2_PIX_FMT_GREY,
		.depth = 8,
		.mbus_code = MEDIA_BUS_FMT_SBGGR8_1X8,
	}, {
		.name = "Y10",
		.fourcc = V4L2_PIX_FMT_Y10,
		.depth = 10,
		.mbus_code = MEDIA_BUS_FMT_SBGGR10_1X10,
	}, {
		.name = "Y12P",
		.fourcc = V4L2_PIX_FMT_Y12P,
		.depth = 12,
		.mbus_code = MEDIA_BUS_FMT_SBGGR12_1X12,
	}
};

static u32 cpi_color_mode_from_bus(u32 data_mode, u32 data_mask)
{
	if (data_mode == CPI_DATA_MODE_16_BIT) {
		switch (data_mask) {
		case CPI_DATA_MASK_10_BIT:
			return CPI_COLOR_MODE_CONFIG_IPI16_RAW10;
		case CPI_DATA_MASK_12_BIT:
			return CPI_COLOR_MODE_CONFIG_IPI16_RAW12;
		case CPI_DATA_MASK_14_BIT:
			return CPI_COLOR_MODE_CONFIG_IPI16_RAW14;
		case CPI_DATA_MASK_16_BIT:
		default:
			return CPI_COLOR_MODE_CONFIG_IPI16_RAW16;
		}
	}

	if (data_mode == CPI_DATA_MODE_8_BIT)
		return CPI_COLOR_MODE_CONFIG_IPI16_RAW8;

	return CPI_COLOR_MODE_CONFIG_IPI16_RAW6;
}

static const struct plat_csi_fmt *cpi_find_format(struct v4l2_format *f)
{
	const struct plat_csi_fmt *fmt = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cpi_formats); ++i) {
		fmt = &cpi_formats[i];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			return fmt;
	}
	return NULL;
}

static const struct plat_csi_fmt *cpi_find_mbus_code(u32 mbus_code)
{
	const struct plat_csi_fmt *fmt = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cpi_formats); ++i) {
		fmt = &cpi_formats[i];
		if (fmt->mbus_code == mbus_code)
			return fmt;
	}
	return NULL;
}

static int
cpi_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	struct cpi_dev *cpi = video_drvdata(file);

	strscpy(cap->driver, "cpi-video-device", sizeof(cap->driver));
	strscpy(cap->card, "cpi-video-device", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(&cpi->pdev->dev));
	return 0;
}

static const struct v4l2_fwnode_endpoint *cpi_get_active_ep(struct cpi_dev *cpi)
{
	if (cpi->num_sensors > 0 &&
	    cpi->sensor[0].cmos_ep_config.bus_type)
		return &cpi->sensor[0].cmos_ep_config;

	if (cpi->cmos_ep_config[0].bus_type)
		return &cpi->cmos_ep_config[0];

	return NULL;
}

static int
cpi_enum_fmt_vid_cap(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	const struct plat_csi_fmt *p_fmt;

	if (f->index >= ARRAY_SIZE(cpi_formats))
		return -EINVAL;

	p_fmt = &cpi_formats[f->index];

	f->pixelformat = p_fmt->fourcc;

	return 0;
}

static int cpi_g_fmt_vid_cap(struct file *file, void *fh,
			     struct v4l2_format *f)
{
	struct cpi_dev *cpi = video_drvdata(file);

	f->fmt.pix = cpi->format.fmt.pix;

	return 0;
}

static int
cpi_try_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	const struct plat_csi_fmt *fmt;
	const struct v4l2_format_info *info;

	fmt = cpi_find_format(f);
	if (!fmt) {
		f->fmt.pix.pixelformat = V4L2_PIX_FMT_Y10;
		fmt = cpi_find_format(f);
	}

	info = v4l2_format_info(f->fmt.pix.pixelformat);

	f->fmt.pix.field = V4L2_FIELD_NONE;
	v4l_bound_align_image(&f->fmt.pix.width, 48, MAX_WIDTH, 2,
			      &f->fmt.pix.height, 32, MAX_HEIGHT, 0, 0);

	f->fmt.pix.bytesperline = (f->fmt.pix.width * ((fmt->depth >= 10) ? 16 : fmt->depth)) >> 3;
	f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.bytesperline;

	if (info) {
		if (info->pixel_enc == V4L2_PIXEL_ENC_RGB)
			f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
		else if (info->pixel_enc == V4L2_PIXEL_ENC_BAYER)
			f->fmt.pix.colorspace = V4L2_COLORSPACE_RAW;
	} else {
		f->fmt.pix.colorspace = V4L2_COLORSPACE_RAW;
	}

	f->fmt.pix.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	f->fmt.pix.quantization = V4L2_QUANTIZATION_DEFAULT;
	f->fmt.pix.xfer_func = V4L2_XFER_FUNC_DEFAULT;
	return 0;
}

static int cpi_s_fmt_vid_cap(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct cpi_dev *cpi = video_drvdata(file);
	struct v4l2_subdev *sd = &cpi->subdev;
	struct v4l2_subdev_format sd_fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = CPI_PAD_SINK,
	};
	const struct plat_csi_fmt *plat_fmt;
	struct v4l2_subdev_state *state;
	int ret;

	if (vb2_is_busy(&cpi->vb_queue))
		return -EBUSY;

	ret = cpi_try_fmt_vid_cap(file, cpi, f);
	if (ret)
		return ret;

	plat_fmt = cpi_find_format(f);
	if (!plat_fmt)
		return -EINVAL;

	sd_fmt.format.width        = f->fmt.pix.width;
	sd_fmt.format.height       = f->fmt.pix.height;
	sd_fmt.format.code         = plat_fmt->mbus_code;
	sd_fmt.format.field        = f->fmt.pix.field;
	sd_fmt.format.colorspace   = f->fmt.pix.colorspace;
	sd_fmt.format.ycbcr_enc    = f->fmt.pix.ycbcr_enc;
	sd_fmt.format.quantization = f->fmt.pix.quantization;
	sd_fmt.format.xfer_func    = f->fmt.pix.xfer_func;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	ret = v4l2_subdev_call(sd, pad, set_fmt, state, &sd_fmt);
	if (ret) {
		v4l2_subdev_unlock_state(state);
		return ret;
	}
	v4l2_subdev_unlock_state(state);

	f->fmt.pix.width        = sd_fmt.format.width;
	f->fmt.pix.height       = sd_fmt.format.height;
	f->fmt.pix.field        = sd_fmt.format.field;
	f->fmt.pix.colorspace   = sd_fmt.format.colorspace;
	f->fmt.pix.ycbcr_enc    = sd_fmt.format.ycbcr_enc;
	f->fmt.pix.quantization = sd_fmt.format.quantization;
	f->fmt.pix.xfer_func    = sd_fmt.format.xfer_func;
	f->fmt.pix.pixelformat  = plat_fmt->fourcc;
	{
		unsigned int bpp = (plat_fmt->depth >= 10) ? 16 : plat_fmt->depth;

		f->fmt.pix.bytesperline = ALIGN(DIV_ROUND_UP(bpp * f->fmt.pix.width, 8), 8);
	}
	f->fmt.pix.sizeimage    = f->fmt.pix.bytesperline * f->fmt.pix.height;

	return 0;
}

static int cpi_enum_framesizes(struct file *file, void *fh,
			       struct v4l2_frmsizeenum *fsize)
{
	static const struct v4l2_frmsize_stepwise sizes = {
		48, MAX_WIDTH, 4,
		32, MAX_HEIGHT, 1
	};
	int i;

	if (fsize->index)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(cpi_formats); i++)
		if (cpi_formats[i].fourcc == fsize->pixel_format)
			break;
	if (i == ARRAY_SIZE(cpi_formats))
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise = sizes;
	return 0;
}

static int cpi_enum_input(struct file *file, void *priv,
			  struct v4l2_input *input)
{
	if (input->index != 0)
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->std = 0;
	strscpy(input->name, "Camera", sizeof(input->name));

	return 0;
}

static int cpi_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int cpi_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i != 0)
		return -EINVAL;
	return 0;
}

static int
cpi_vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type type)
{
	struct cpi_dev *cpi = video_drvdata(file);

	if (cpi->version == CPI_VERSION_2)
		cpi->axi_bus_ep = true;

	return vb2_ioctl_streamon(file, priv, type);
}

static int
cpi_streamoff(struct file *file, void *priv, enum v4l2_buf_type type)
{
	struct cpi_dev *cpi = video_drvdata(file);

	if (cpi->version == CPI_VERSION_2)
		cpi->axi_bus_ep = false;

	return vb2_ioctl_streamoff(file, priv, type);
}

static const struct v4l2_ioctl_ops cpi_ioctl_ops = {
	.vidioc_querycap = cpi_querycap,

	.vidioc_enum_fmt_vid_cap = cpi_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = cpi_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = cpi_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = cpi_try_fmt_vid_cap,
	.vidioc_enum_framesizes = cpi_enum_framesizes,

	.vidioc_enum_input = cpi_enum_input,
	.vidioc_g_input = cpi_g_input,
	.vidioc_s_input = cpi_s_input,

	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = cpi_vidioc_streamon,
	.vidioc_streamoff = cpi_streamoff,
};

static const struct v4l2_file_operations cpi_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.write = vb2_fop_write,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

static inline struct rx_buffer *to_rx_buffer(struct vb2_v4l2_buffer *vb2)
{
	return container_of(vb2, struct rx_buffer, vb);
}

static inline void cpi_hw_enable_interrupts(struct cpi_dev *cpi, u32 intr_mask)
{
	u32 val = readl(cpi->base_addr + CPI_INTR_ENA);

	val |= intr_mask;
	writel(val, cpi->base_addr + CPI_INTR_ENA);
}

static inline void cpi_hw_disable_interrupts(struct cpi_dev *cpi,
					     u32 intr_mask)
{
	u32 val = readl(cpi->base_addr + CPI_INTR_ENA);

	val &= ~intr_mask;
	writel(val, cpi->base_addr + CPI_INTR_ENA);
}

static inline int cpi_hw_setup_buffer(struct cpi_dev *cpi)
{
	if (!cpi->active)
		return -EFAULT;

	writel(cpi->active->dma_addr, cpi->base_addr + CPI_FRAME_ADDR);
	return 0;
}

static void cpi_hw_start_video_capture(struct cpi_dev *cpi)
{
	writel(0, cpi->base_addr + CPI_CTRL);
	writel(CTRL_SW_RESET, cpi->base_addr + CPI_CTRL);
	writel(0, cpi->base_addr + CPI_CTRL);

	if (cpi->active)
		writel(cpi->active->dma_addr, cpi->base_addr + CPI_FRAME_ADDR);

	writel(CTRL_START |
		CTRL_SNAPSHOT |
		CTRL_FIFO_CLK_SEL, cpi->base_addr + CPI_CTRL);
}

static inline int cpi_hw_status(struct cpi_dev *cpi)
{
	u32 val;

	val = readl(cpi->base_addr + CPI_CTRL);
	if (val & CTRL_BUSY)
		return -EBUSY;

	return 0;
}

static int cpi_enable_axi_port(struct cpi_dev *cpi)
{
	u32 val;

	val = readl(cpi->base_addr + CPI_CFG);
	val |= CFG_AXI_PORT_EN;
	writel(val, cpi->base_addr + CPI_CFG);

	return 0;
}

static int cpi_enable_isp_port(struct cpi_dev *cpi)
{
	u32 val;

	val = readl(cpi->base_addr + CPI_CFG);
	val |= CFG_ISP_PORT_EN;
	writel(val, cpi->base_addr + CPI_CFG);

	return 0;
}

static int cpi_disable_axi_port(struct cpi_dev *cpi)
{
	u32 val;

	val = readl(cpi->base_addr + CPI_CFG);
	val &= ~CFG_AXI_PORT_EN;
	writel(val, cpi->base_addr + CPI_CFG);

	return 0;
}

static int cpi_disable_isp_port(struct cpi_dev *cpi)
{
	u32 val;

	val = readl(cpi->base_addr + CPI_CFG);
	val &= ~CFG_ISP_PORT_EN;
	writel(val, cpi->base_addr + CPI_CFG);

	return 0;
}

static int cpi_hw_set_geometry(struct cpi_dev *cpi)
{
	const struct plat_csi_fmt *csi_fmt = cpi->fmt;
	struct v4l2_format *v4l2_fmt = &cpi->format;
	struct device *dev = &cpi->pdev->dev;
	const struct v4l2_fwnode_endpoint *ep = cpi_get_active_ep(cpi);
	const struct v4l2_mbus_config_parallel *conf = NULL;
	u32 bus_width;
	u32 data_mode;
	u32 data_mask;
	u32 val;
	int i;
	enum v4l2_mbus_type bus_type;
	bool is_mipi;

	if (!csi_fmt) {
		dev_err(dev, "Format not initialized\n");
		return -EINVAL;
	}

	if (ep)
		bus_type = ep->bus_type;
	else
		bus_type = cpi->is_parallel_interface ?
			   V4L2_MBUS_PARALLEL : V4L2_MBUS_CSI2_DPHY;

	is_mipi = (bus_type == V4L2_MBUS_CSI2_DPHY);
	if (!ep)
		dev_dbg(dev, "Endpoint info missing, assuming %s interface\n",
			is_mipi ? "CSI-2" : "parallel");

	val = (CPI_FIFO_WR_WMARK_DEFAULT << FIFO_WR_WMARK_SHIFT) |
	      (CPI_FIFO_RD_WMARK_DEFAULT << FIFO_RD_WMARK_SHIFT);
	writel(val, cpi->base_addr + CPI_FIFO_CTRL);

	val =	(v4l2_fmt->fmt.pix.width & GENMASK(13, 0)) |
		(((v4l2_fmt->fmt.pix.height - 1) & GENMASK(11, 0)) << 16);
	writel(val, cpi->base_addr + CPI_VIDEO_FCFG);

	if (is_mipi)
		bus_width = csi_fmt->depth;
	else if (ep && ep->bus.parallel.bus_width)
		bus_width = ep->bus.parallel.bus_width;
	else
		bus_width = csi_fmt->depth;

	switch (bus_width) {
	case 1:
		data_mode = CPI_DATA_MODE_1_BIT;
		data_mask = 0;
		break;
	case 2:
		data_mode = CPI_DATA_MODE_2_BIT;
		data_mask = 0;
		break;
	case 4:
		data_mode = CPI_DATA_MODE_4_BIT;
		data_mask = 0;
		break;
	case 8:
		data_mode = CPI_DATA_MODE_8_BIT;
		data_mask = 0;
		break;
	case 10:
		data_mode = CPI_DATA_MODE_16_BIT;
		data_mask = CPI_DATA_MASK_10_BIT;
		break;
	case 12:
		data_mode = CPI_DATA_MODE_16_BIT;
		data_mask = CPI_DATA_MASK_12_BIT;
		break;
	case 14:
		data_mode = CPI_DATA_MODE_16_BIT;
		data_mask = CPI_DATA_MASK_14_BIT;
		break;
	case 16:
	default:
		data_mode = CPI_DATA_MODE_16_BIT;
		data_mask = CPI_DATA_MASK_16_BIT;
		break;
	}

	if (is_mipi) {
		val = CFG_MIPI_CSI | CFG_WAIT_VSYNC | CFG_AXI_PORT_EN;
		val |= ((data_mode & CFG_DATA_MODE_MASK) << CFG_DATA_MODE_MASK_SHIFT) |
			((data_mask & CFG_DATA_MASK) << CFG_DATA_MASK_SHIFT);
		if (cpi->version == CPI_VERSION_2) {
			val = (cpi->axi_bus_ep) ? (CFG_AXI_PORT_EN | val) : val;
			val = (cpi->is_isp_connected) ? (CFG_ISP_PORT_EN | val) : val;
		}
		writel(val, cpi->base_addr + CPI_CFG);

		val = cpi_color_mode_from_bus(data_mode, data_mask);
		for (i = 0; i < ARRAY_SIZE(mappings); i++) {
			if (v4l2_fmt->fmt.pix.pixelformat == mappings[i].fourcc) {
				val = mappings[i].col_mode;
				break;
			}
		}

		writel(val, cpi->base_addr + CPI_CSI_CMCFG);
	} else {
		conf = ep ? &ep->bus.parallel : NULL;

		if (conf && conf->flags & (V4L2_MBUS_PCLK_SAMPLE_FALLING |
				   V4L2_MBUS_PCLK_SAMPLE_DUALEDGE)) {
			dev_err(dev, "only rising-edge sampling of data is supported\n");
			return -EOPNOTSUPP;
		}

		val = CFG_ROW_ROUNDUP;
		if (cpi->version == CPI_VERSION_2) {
			val = (cpi->axi_bus_ep) ? (CFG_AXI_PORT_EN | val) : val;
			val = (cpi->is_isp_connected) ? (CFG_ISP_PORT_EN | val) : val;
		}
		if (conf && conf->flags & V4L2_MBUS_HSYNC_ACTIVE_LOW)
			val |= CFG_HSYNC_POL;
		else
			val &= ~CFG_HSYNC_POL;

		if (conf && conf->flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)
			val |= CFG_VSYNC_POL;
		else
			val &= ~CFG_VSYNC_POL;

		if (cpi->code_10_on_8)
			val |= CFG_CODE10ON8;
		else
			val &= ~CFG_CODE10ON8;

		if (cpi->msb)
			val |= CFG_MSB;
		else
			val &= ~CFG_MSB;

		if (cpi->pclk_active)
			val &= ~CFG_PCLK_POL;
		else
			val |= CFG_PCLK_POL;

		if (cpi->vsync_en)
			val |= CFG_VSYNC_EN;
		else
			val &= ~CFG_VSYNC_EN;

		if (cpi->wait_vsync_event)
			val |= CFG_WAIT_VSYNC;
		else
			val &= ~CFG_WAIT_VSYNC;

		val |= ((data_mode & CFG_DATA_MODE_MASK) << CFG_DATA_MODE_MASK_SHIFT) |
			((data_mask & CFG_DATA_MASK) << CFG_DATA_MASK_SHIFT);
		writel(val, cpi->base_addr + CPI_CFG);
	}

	return 0;
}

static bool cpi_is_userptr(struct cpi_dev *cpi)
{
	return cpi->vb_queue.memory == VB2_MEMORY_USERPTR;
}

static void cpi_free_bounce(struct cpi_dev *cpi)
{
	if (!cpi->bounce_cpu)
		return;

	dma_free_coherent(&cpi->pdev->dev, cpi->bounce_size,
			  cpi->bounce_cpu, cpi->bounce_dma);
	cpi->bounce_cpu = NULL;
	cpi->bounce_dma = 0;
	cpi->bounce_size = 0;
}

static int cpi_alloc_bounce(struct cpi_dev *cpi)
{
	unsigned int size = cpi->format.fmt.pix.sizeimage;

	if (!size)
		return -EINVAL;

	if (cpi->bounce_cpu) {
		if (cpi->bounce_size >= size)
			return 0;
		cpi_free_bounce(cpi);
	}

	cpi->bounce_cpu = dma_alloc_coherent(&cpi->pdev->dev, size, &cpi->bounce_dma, GFP_KERNEL);
	if (!cpi->bounce_cpu)
		return -ENOMEM;

	cpi->bounce_size = size;
	return 0;
}

static void cpi_copy_bounce_to_user(struct cpi_dev *cpi)
{
	struct rx_buffer *buf;
	unsigned int size = cpi->format.fmt.pix.sizeimage;

	spin_lock_irq(&cpi->slock);
	if (cpi->stopping && !cpi->active) {
		spin_unlock_irq(&cpi->slock);
		return;
	}
	buf = cpi->active;
	spin_unlock_irq(&cpi->slock);

	if (!buf || !buf->cpu_addr || !cpi->bounce_cpu || !size)
		return;

	if (size > cpi->bounce_size)
		size = cpi->bounce_size;

	memcpy(buf->cpu_addr, cpi->bounce_cpu, size);
}

static int cpi_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			   unsigned int *nplanes, unsigned int sizes[],
			struct device *alloc_devs[])
{
	struct cpi_dev *cpi = vb2_get_drv_priv(vq);
	unsigned long size;

	size = cpi->format.fmt.pix.sizeimage;
	if (size == 0)
		return -EINVAL;

	/*
	 * USERPTR buffers are only virtually contiguous. CPI can DMA to a
	 * single physical address, so map the user pages with vmalloc memops
	 * and bounce through reserved SRAM.
	 */
	if (vq->memory == VB2_MEMORY_USERPTR)
		vq->mem_ops = &vb2_vmalloc_memops;
	else
		vq->mem_ops = &vb2_dma_contig_memops;

	if (*nbuffers > N_BUFFERS)
		*nbuffers = N_BUFFERS;
	*nplanes = 1;
	sizes[0] = size;

	cpi->active = NULL;
	return 0;
}

static int cpi_buffer_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rx_buffer *buf = container_of(vbuf, struct rx_buffer, vb);

	buf->dma_addr = 0;
	buf->cpu_addr = NULL;
	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static int cpi_buffer_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct cpi_dev *cpi = vb2_get_drv_priv(vb->vb2_queue);

	vb2_set_plane_payload(vb, 0, cpi->format.fmt.pix.sizeimage);

	if (vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0))
		return -EINVAL;

	vbuf->field = cpi->format.fmt.pix.field;
	return 0;
}

static void cpi_buffer_queue(struct vb2_buffer *vb)
{
	struct cpi_dev *cpi = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rx_buffer *buf = to_rx_buffer(vbuf);
	unsigned long flags = 0;

	spin_lock_irqsave(&cpi->slock, flags);
	list_add_tail(&buf->list, &cpi->fb_list_head);
	if (cpi_is_userptr(cpi)) {
		buf->dma_addr = cpi->bounce_dma;
		buf->cpu_addr = vb2_plane_vaddr(vb, 0);
	} else {
		buf->dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
		buf->cpu_addr = vb2_plane_vaddr(vb, 0);
	}

	if (!cpi->active) {
		cpi->active = buf;
		if (vb2_is_streaming(vb->vb2_queue)) {
			cpi_hw_setup_buffer(cpi);
			cpi_hw_start_video_capture(cpi);
		}
	}
	spin_unlock_irqrestore(&cpi->slock, flags);

	dev_dbg(&cpi->pdev->dev,
		"Queued buffer: dma_addr - %pad cpu_addr - %p\n",
		&buf->dma_addr,
		buf->cpu_addr);
}

static void cpi_return_all_buffers_locked(struct cpi_dev *cpi, enum vb2_buffer_state state)
{
	struct rx_buffer *buf, *node;

	lockdep_assert_held(&cpi->slock);

	list_for_each_entry_safe(buf, node, &cpi->fb_list_head, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	cpi->active = NULL;
}

static int cpi_vb2_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct cpi_dev *cpi = vb2_get_drv_priv(q);
	struct video_device *vfd = &cpi->ve.vdev;
	struct v4l2_subdev *sd = &cpi->subdev;
	u32 pad = CPI_PAD_SOURCE;
	u64 streams_mask = BIT(0);
	int ret;

	cpi->stopping = false;

	if (cpi_is_userptr(cpi)) {
		struct rx_buffer *buf;

		ret = cpi_alloc_bounce(cpi);
		if (ret)
			goto error_state;

		spin_lock_irq(&cpi->slock);
		list_for_each_entry(buf, &cpi->fb_list_head, list)
			buf->dma_addr = cpi->bounce_dma;
		spin_unlock_irq(&cpi->slock);
	}

	ret = video_device_pipeline_alloc_start(vfd);
	if (ret < 0)
		goto error_bounce;

	ret = v4l2_subdev_enable_streams(sd, pad, streams_mask);
	if (ret)
		goto error_streaming;

	return 0;
error_streaming:
	video_device_pipeline_stop(vfd);
error_bounce:
	cpi_free_bounce(cpi);
error_state:
	spin_lock_irq(&cpi->slock);
	cpi_return_all_buffers_locked(cpi, VB2_BUF_STATE_QUEUED);
	spin_unlock_irq(&cpi->slock);
	return ret;
}

static void cpi_vb2_stop_streaming(struct vb2_queue *q)
{
	struct cpi_dev *cpi = vb2_get_drv_priv(q);
	struct video_device *vfd = &cpi->ve.vdev;
	struct v4l2_subdev *sd = &cpi->subdev;
	u64 streams_mask = BIT(0);

	spin_lock_irq(&cpi->slock);
	cpi->stopping = true;
	cpi_hw_disable_interrupts(cpi, INTR_VSYNC | INTR_BRESP_ERR | INTR_OUTFIFO_OVERRUN |
				  INTR_STOP);
	spin_unlock_irq(&cpi->slock);

	synchronize_irq(cpi->irq);

	v4l2_subdev_disable_streams(sd, CPI_PAD_SOURCE, streams_mask);

	video_device_pipeline_stop(vfd);

	spin_lock_irq(&cpi->slock);
	cpi_return_all_buffers_locked(cpi, VB2_BUF_STATE_ERROR);
	spin_unlock_irq(&cpi->slock);

	cpi_free_bounce(cpi);
}

static const struct vb2_ops vb2_video_qops = {
	.queue_setup = cpi_queue_setup,
	.buf_init = cpi_buffer_init,
	.buf_prepare = cpi_buffer_prepare,
	.buf_queue = cpi_buffer_queue,
	.start_streaming = cpi_vb2_start_streaming,
	.stop_streaming = cpi_vb2_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int cpi_subdev_init_state(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_subdev_route routes[] = {
		{
			.sink_pad = CPI_PAD_SINK,
			.sink_stream = 0,
			.source_pad = CPI_PAD_SOURCE,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
		{
			.sink_pad = CPI_PAD_SINK,
			.sink_stream = 0,
			.source_pad = CPI_PAD_SOURCE_ISP,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};
	struct v4l2_subdev_krouting routing = {
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};
	unsigned int i;
	int ret;

	ret = v4l2_subdev_set_routing(subdev, state, &routing);
	if (ret)
		return ret;

	for (i = 0; i < CPI_PADS_NUM; i++) {
		fmt = v4l2_subdev_state_get_format(state, i);
		fmt->width = 1920;
		fmt->height = 1080;
		fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
		fmt->field = V4L2_FIELD_NONE;
		fmt->colorspace = V4L2_COLORSPACE_SRGB;
		fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
		fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	}

	return 0;
}

/*
 * Video device creation is deferred until the async notifier has fully
 * bound the remote CSI-2/sensor subdev (see cpi_subdev_notifier_complete()).
 * Registering it earlier (e.g. from the .registered internal op, which
 * fires synchronously during CPI's own subdev registration) would let
 * userspace call VIDIOC_STREAMON before cpi->remote_sd is set, crashing
 * in cpi_enable_streams().
 */
static int cpi_register_video_device(struct cpi_dev *cpi)
{
	struct v4l2_subdev *sd = &cpi->subdev;
	struct vb2_queue *q = &cpi->vb_queue;
	struct video_device *vfd = &cpi->ve.vdev;
	int ret;

	/*
	 * If ISP is connected, video device creation and IOCTL support is ISP
	 * device's responsibility.
	 */
	if (cpi->is_isp_connected)
		return 0;

	memset(vfd, 0, sizeof(*vfd));
	strscpy(vfd->name, "alif-cam", sizeof(vfd->name));
	vfd->fops = &cpi_fops;
	vfd->ioctl_ops = &cpi_ioctl_ops;
	vfd->v4l2_dev = sd->v4l2_dev;
	vfd->minor = -1;
	vfd->release = video_device_release_empty;
	vfd->queue = q;
	vfd->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE;
	vfd->vfl_dir = VFL_DIR_RX;
	vfd->vfl_type = VFL_TYPE_VIDEO;

	memset(q, 0, sizeof(*q));
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_READ;

	q->ops = &vb2_video_qops;
	q->mem_ops = &vb2_dma_contig_memops;

	q->buf_struct_size = sizeof(struct rx_buffer);
	q->drv_priv = cpi;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &cpi->lock;
	q->dev = &cpi->pdev->dev;

	ret = vb2_queue_init(q);
	if (ret < 0)
		return ret;

	vfd->entity.ops = NULL;

	cpi->vd_pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&vfd->entity, 1, &cpi->vd_pad);
	if (ret < 0)
		return ret;

	video_set_drvdata(vfd, cpi);
	vfd->lock = &cpi->lock;

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (ret < 0) {
		media_entity_cleanup(&vfd->entity);
		cpi->ve.pipe = NULL;
		return ret;
	}

	ret = media_create_pad_link(&cpi->subdev.entity, CPI_PAD_SOURCE,
				    &vfd->entity, VIDEO_DEV_PAD_SINK_CPI,
				    MEDIA_LNK_FL_ENABLED);
	if (ret) {
		dev_err(&cpi->pdev->dev, "Failed to create video pad link: %d\n", ret);
		goto unreg_video;
	}

	v4l2_info(sd->v4l2_dev, "Registered %s as /dev/%s\n",
		  vfd->name, video_device_node_name(vfd));

	return 0;

unreg_video:
	video_unregister_device(vfd);
	media_entity_cleanup(&vfd->entity);
	cpi->ve.pipe = NULL;
	return ret;
}

static void cpi_subdev_unregistered(struct v4l2_subdev *sd)
{
	struct cpi_dev *cpi = v4l2_get_subdevdata(sd);

	if (!cpi)
		return;

	mutex_lock(&cpi->lock);

	if (video_is_registered(&cpi->ve.vdev)) {
		video_unregister_device(&cpi->ve.vdev);
		media_entity_cleanup(&cpi->ve.vdev.entity);
		cpi->ve.pipe = NULL;
	}

	mutex_unlock(&cpi->lock);
}

static const struct v4l2_subdev_internal_ops cpi_subdev_internal_ops = {
	.init_state = cpi_subdev_init_state,
	.unregistered = cpi_subdev_unregistered,
};

static int cpi_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *fmt)
{
	struct cpi_dev *cpi = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *mbus_fmt = &fmt->format;

	if (fmt->pad >= CPI_PADS_NUM)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*mbus_fmt = *v4l2_subdev_state_get_format(state, fmt->pad);
	} else {
		fmt->format.width = cpi->format.fmt.pix.width;
		fmt->format.height = cpi->format.fmt.pix.height;
		fmt->format.code = cpi->fmt->mbus_code;
		fmt->format.field = cpi->format.fmt.pix.field;
		fmt->format.colorspace = cpi->format.fmt.pix.colorspace;
		fmt->format.ycbcr_enc = cpi->format.fmt.pix.ycbcr_enc;
		fmt->format.quantization = cpi->format.fmt.pix.quantization;
		fmt->format.xfer_func = cpi->format.fmt.pix.xfer_func;
	}

	return 0;
}

static int cpi_set_pixclk(struct cpi_dev *cpi, u64 pix_clk, bool enable)
{
	int ret = 0;

	if (!cpi->pix_clk)
		return 0;

	if (!enable) {
		if (cpi->pix_clk_enabled) {
			clk_disable_unprepare(cpi->pix_clk);
			cpi->pix_clk_enabled = false;
		}
		return 0;
	}

	/* Re-enable with the new rate: disable first to keep refcount balanced */
	if (cpi->pix_clk_enabled)
		clk_disable_unprepare(cpi->pix_clk);

	if (pix_clk) {
		ret = clk_set_rate(cpi->pix_clk, pix_clk);
		if (ret) {
			dev_err(&cpi->pdev->dev,
				"Failed to set pixel clock rate to %llu Hz: %d\n",
				pix_clk, ret);
			cpi->pix_clk_enabled = false;
			return ret;
		}
	}

	ret = clk_prepare_enable(cpi->pix_clk);
	if (ret) {
		dev_err(&cpi->pdev->dev, "Failed to enable pixel clock: %d\n", ret);
		cpi->pix_clk_enabled = false;
		return ret;
	}

	cpi->pix_clk_enabled = true;
	return 0;
}

static void cpi_try_fmt(struct cpi_dev *cpi, struct v4l2_mbus_framefmt *fmt)
{
	const struct plat_csi_fmt *plat_csi;
	const struct v4l2_format_info *info;

	plat_csi = cpi_find_mbus_code(fmt->code);
	if (!plat_csi) {
		fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
		plat_csi = cpi_find_mbus_code(fmt->code);
	}

	fmt->width = clamp(fmt->width, 48U, MAX_WIDTH);
	fmt->height = clamp(fmt->height, 32U, MAX_HEIGHT);
	fmt->code = plat_csi->mbus_code;
	fmt->field = V4L2_FIELD_NONE;

	info = v4l2_format_info(plat_csi->fourcc);
	if (info) {
		if (info->pixel_enc == V4L2_PIXEL_ENC_RGB)
			fmt->colorspace = V4L2_COLORSPACE_SRGB;
		else if (info->pixel_enc == V4L2_PIXEL_ENC_BAYER)
			fmt->colorspace = V4L2_COLORSPACE_RAW;
	} else {
		fmt->colorspace = V4L2_COLORSPACE_RAW;
	}

	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int cpi_get_sensor_ep_parameters(struct media_entity *start,
					struct plat_csi_sensor_info *sensor,
					int max_sensors);

static int cpi_set_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *sd_state,
		       struct v4l2_subdev_format *fmt)
{
	struct cpi_dev *cpi = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *subdev_fmt;
	struct v4l2_mbus_framefmt *src_fmt;
	const struct plat_csi_fmt *plat_csi;
	struct v4l2_subdev_format upstream_fmt;
	u64 max_pix_clk = 0;
	int ret = 0;
	int i;

	cpi_try_fmt(cpi, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		subdev_fmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		if (subdev_fmt) {
			*subdev_fmt = fmt->format;
			if (fmt->pad == CPI_PAD_SINK) {
				for (i = 1; i < CPI_PADS_NUM; i++) {
					src_fmt = v4l2_subdev_state_get_format(sd_state, i);
					if (src_fmt)
						*src_fmt = fmt->format;
				}
			}
		}
		return 0;
	}

	if (fmt->pad == CPI_PAD_SINK && cpi->remote_sd) {
		/*
		 * If already in pipeline propagation, skip propagation
		 * but continue to update local format with negotiated values
		 */
		if (!cpi->in_pipeline_propagation) {
			upstream_fmt = *fmt;
			upstream_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;

			cpi->in_pipeline_propagation = true;
			ret = pipeline_set_format(sd, sd_state, &upstream_fmt);
			cpi->in_pipeline_propagation = false;

			if (ret) {
				dev_err(&cpi->pdev->dev, "Failed to propagate format: %d\n", ret);
				return ret;
			}

			fmt->format = upstream_fmt.format;
		} else {
			dev_dbg(&cpi->pdev->dev,
				"Already in propagation, skipping recursion\n");
		}
	} else if (fmt->pad == CPI_PAD_SOURCE || fmt->pad == CPI_PAD_SOURCE_ISP) {
		if (!cpi->fmt) {
			dev_err(&cpi->pdev->dev, "Format not initialized\n");
			return -EINVAL;
		}
		fmt->format.width = cpi->format.fmt.pix.width;
		fmt->format.height = cpi->format.fmt.pix.height;
		fmt->format.code = cpi->fmt->mbus_code;
		fmt->format.field = cpi->format.fmt.pix.field;
		fmt->format.colorspace = cpi->format.fmt.pix.colorspace;
		fmt->format.ycbcr_enc = cpi->format.fmt.pix.ycbcr_enc;
		fmt->format.quantization = cpi->format.fmt.pix.quantization;
		fmt->format.xfer_func = cpi->format.fmt.pix.xfer_func;

		subdev_fmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		if (subdev_fmt)
			*subdev_fmt = fmt->format;

		return 0;
	}

	plat_csi = cpi_find_mbus_code(fmt->format.code);
	if (!plat_csi) {
		dev_err(&cpi->pdev->dev,
			"requested media bus format not found\n");
		return -EINVAL;
	}
	cpi->fmt = plat_csi;
	cpi->format.fmt.pix.width = fmt->format.width;
	cpi->format.fmt.pix.height = fmt->format.height;
	cpi->format.fmt.pix.field = fmt->format.field;
	cpi->format.fmt.pix.pixelformat = plat_csi->fourcc;
	{
		unsigned int bpp = (plat_csi->depth >= 10) ? 16 : plat_csi->depth;

		cpi->format.fmt.pix.bytesperline =
			ALIGN(DIV_ROUND_UP(bpp * fmt->format.width, 8), 8);
	}
	cpi->format.fmt.pix.sizeimage =
		cpi->format.fmt.pix.bytesperline * fmt->format.height;
	cpi->format.fmt.pix.colorspace = fmt->format.colorspace;
	cpi->format.fmt.pix.ycbcr_enc = fmt->format.ycbcr_enc;
	cpi->format.fmt.pix.quantization = fmt->format.quantization;
	cpi->format.fmt.pix.xfer_func = fmt->format.xfer_func;

	subdev_fmt = v4l2_subdev_state_get_format(sd_state, CPI_PAD_SINK);
	if (subdev_fmt)
		*subdev_fmt = fmt->format;

	for (i = 1; i < CPI_PADS_NUM; i++) {
		struct v4l2_mbus_framefmt *src_fmt;

		src_fmt = v4l2_subdev_state_get_format(sd_state, i);
		if (src_fmt)
			*src_fmt = fmt->format;
	}

	if (cpi->num_sensors == 0) {
		int sf = cpi_get_sensor_ep_parameters(&cpi->subdev.entity,
					cpi->sensor,
					ALIF_MAX_CSI_SENSORS);

		if (sf > 0)
			cpi->num_sensors = sf;
	}

	if (cpi->num_sensors > 0 && cpi->sensor[0].cmos_ep_config.bus_type == V4L2_MBUS_CSI2_DPHY) {
		for (i = 0; i < cpi->num_sensors; i++) {
			struct v4l2_fwnode_endpoint *ep = &cpi->sensor[i].cmos_ep_config;
			struct v4l2_subdev *sensor_sd = cpi->sensor[i].subdev;
			u64 link_freq = 0;

			{
				struct v4l2_ctrl_handler *hdl = sensor_sd->ctrl_handler;
				unsigned int lanes = ep->bus.mipi_csi2.num_data_lanes;
				unsigned int depth = plat_csi->depth;

				link_freq = v4l2_get_link_freq(hdl, depth, lanes * 2);
				link_freq = div_u64((link_freq << 1) * lanes, depth);
			}
			max_pix_clk = (max_pix_clk < link_freq) ? link_freq :
				max_pix_clk;
		}
		max_pix_clk = div64_u64(max_pix_clk * 12, 10);
		ret = cpi_set_pixclk(cpi, max_pix_clk, true);
		if (ret) {
			dev_err(&cpi->pdev->dev, "Failed to enable pixel clock\n");
			return ret;
		}
	}

	return 0;
}

static int cpi_hw_configure(struct cpi_dev *cpi)
{
	int ret;

	cpi->sequence = 0;
	spin_lock_irq(&cpi->slock);

	ret = cpi_hw_status(cpi);
	if (ret) {
		spin_unlock_irq(&cpi->slock);
		dev_err(&cpi->pdev->dev, "CPI is busy\n");
		return ret;
	}

	cpi_hw_set_geometry(cpi);

	if (cpi->active) {
		cpi_hw_setup_buffer(cpi);
	} else if (cpi->version == CPI_VERSION_1) {
		spin_unlock_irq(&cpi->slock);
		dev_err(&cpi->pdev->dev,
			"No buffers available for capture\n");
		return -EINVAL;
	}

	cpi_hw_enable_interrupts(cpi, INTR_VSYNC |
		     INTR_BRESP_ERR |
		     INTR_OUTFIFO_OVERRUN |
		     INTR_STOP);

	spin_unlock_irq(&cpi->slock);

	return 0;
}

static void cpi_hw_enable(struct cpi_dev *cpi)
{
	spin_lock_irq(&cpi->slock);

	cpi_hw_start_video_capture(cpi);
	spin_unlock_irq(&cpi->slock);

	dev_dbg(&cpi->pdev->dev, "cpi: Start capture | Video mode!! 0x%x\n",
		readl(cpi->base_addr + CPI_CTRL));
}

static void cpi_hw_disable(struct cpi_dev *cpi)
{
	u32 val;
	int ret;

	spin_lock_irq(&cpi->slock);

	cpi_hw_disable_interrupts(cpi, INTR_VSYNC |
				INTR_BRESP_ERR |
				INTR_OUTFIFO_OVERRUN |
				INTR_STOP);
	writel(0, cpi->base_addr + CPI_CTRL);

	spin_unlock_irq(&cpi->slock);

	ret = readl_poll_timeout(cpi->base_addr + CPI_CTRL,
				 val, !(val & CTRL_BUSY), CPI_BUSY_POLL_USEC,
			CPI_BUSY_TIMEOUT_USEC);
	if (ret) {
		dev_err(&cpi->pdev->dev,
			"Failed to stop the Camera controller.\n");
	}
}

static int cpi_enable_streams(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state, u32 pad, u64 streams_mask)
{
	struct cpi_dev *cpi = v4l2_get_subdevdata(sd);
	u64 sink_streams;
	int ret;

	if (pad != CPI_PAD_SOURCE && pad != CPI_PAD_SOURCE_ISP)
		return -EINVAL;

	if (!cpi->remote_sd)
		return -ENOLINK;

	if (!cpi->enabled_pad_mask) {
		ret = cpi_hw_configure(cpi);
		if (ret)
			return ret;

		sink_streams = v4l2_subdev_state_xlate_streams(state,
							       pad, CPI_PAD_SINK, &streams_mask);

		ret = v4l2_subdev_enable_streams(cpi->remote_sd,
						 cpi->remote_pad, sink_streams);
		if (ret)
			return ret;

		/* Enable CPI after upstream CSI/sensor streaming is active */
		cpi_hw_enable(cpi);
	} else {
		if (pad == CPI_PAD_SOURCE && cpi->axi_bus_ep) {
			spin_lock_irq(&cpi->slock);
			if (cpi->active) {
				cpi_hw_setup_buffer(cpi);
			} else {
				spin_unlock_irq(&cpi->slock);
				dev_err(&cpi->pdev->dev,
					"No buffers available for AXI capture!\n");
				return -EINVAL;
			}
			spin_unlock_irq(&cpi->slock);
		}
	}

	if (cpi->version == CPI_VERSION_2) {
		if (pad == CPI_PAD_SOURCE)
			cpi_enable_axi_port(cpi);
		else if (pad == CPI_PAD_SOURCE_ISP)
			cpi_enable_isp_port(cpi);
	}

	cpi->enabled_pad_mask |= BIT_ULL(pad);

	return 0;
}

static int cpi_disable_streams(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state, u32 pad, u64 streams_mask)
{
	struct cpi_dev *cpi = v4l2_get_subdevdata(sd);
	u64 sink_streams;

	if (pad != CPI_PAD_SOURCE && pad != CPI_PAD_SOURCE_ISP)
		return -EINVAL;

	if (!(cpi->enabled_pad_mask & BIT_ULL(pad)))
		return 0;

	if (cpi->version == CPI_VERSION_2) {
		if (pad == CPI_PAD_SOURCE)
			cpi_disable_axi_port(cpi);
		else if (pad == CPI_PAD_SOURCE_ISP)
			cpi_disable_isp_port(cpi);
	}

	cpi->enabled_pad_mask &= (~BIT_ULL(pad));

	if (!cpi->enabled_pad_mask) {
		int ret = 0;

		if (cpi->remote_sd) {
			sink_streams = v4l2_subdev_state_xlate_streams(state, pad, CPI_PAD_SINK,
								       &streams_mask);
			ret = v4l2_subdev_disable_streams(cpi->remote_sd, cpi->remote_pad,
							  sink_streams);
		} else {
			ret = -ENOLINK;
		}

		cpi_hw_disable(cpi);
		return ret;
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops cpi_pad_ops = {
	.set_fmt = cpi_set_fmt,
	.get_fmt = cpi_get_fmt,
	.link_validate = v4l2_subdev_link_validate_default,
	.enable_streams = cpi_enable_streams,
	.disable_streams = cpi_disable_streams,
};

static const struct v4l2_subdev_ops cpi_subdev_ops = {
	.pad = &cpi_pad_ops,
};

/**
 * cpi_get_fwnode_pad - Map device tree port to CPI pad
 * @entity: Media entity (CPI subdev)
 * @endpoint: Parsed endpoint from device tree
 *
 * CPI has 3 pads but only 2 ports in device tree:
 * - Port 0 (input) -> Pad 0 (CPI_PAD_SINK) - connects to CSI-2/sensor
 * - Port 1 (output) -> Pad 2 (CPI_PAD_SOURCE_ISP) - connects to ISP
 *
 * Note: Pad 1 (CPI_PAD_SOURCE) is for video device and has no DT port.
 * The video device link is created programmatically in registered callback.
 *
 * Returns: Pad index on success, negative error code on failure.
 */
static int cpi_get_fwnode_pad(struct media_entity *entity,
			      struct fwnode_endpoint *endpoint)
{
	switch (endpoint->port) {
	case PORT_INPUT:
		return CPI_PAD_SINK;
	case PORT_OUTPUT:
		return CPI_PAD_SOURCE_ISP;
	default:
		return -ENXIO;
	}
}

static struct media_entity_operations cpi_subdev_mdev_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.get_fwnode_pad = cpi_get_fwnode_pad,
};

static const struct media_device_ops cpi_media_ops = {
	.link_notify = v4l2_pipeline_link_notify,
};

static int cpi_subdev_notifier_bound(struct v4l2_async_notifier *notifier,
				     struct v4l2_subdev *remote_subdev,
				     struct v4l2_async_connection *async_subdev)
{
	struct cpi_dev *cpi = notifier_to_plat_csi(notifier);
	struct device *dev = &cpi->pdev->dev;
	int ret;
	int pad;

	pad = media_entity_get_fwnode_pad(&remote_subdev->entity,
					  async_subdev->match.fwnode, MEDIA_PAD_FL_SOURCE);
	if (pad < 0) {
		dev_err(dev, "Failed to find source pad for %s\n", remote_subdev->name);
		return pad;
	}

	ret = media_create_pad_link(&remote_subdev->entity, pad,
				    &cpi->subdev.entity, CPI_PAD_SINK, MEDIA_LNK_FL_ENABLED);
	if (ret) {
		dev_err(dev, "Failed to create pad link: %s to %s\n",
			remote_subdev->entity.name, cpi->subdev.entity.name);
		return ret;
	}

	cpi->remote_sd = remote_subdev;
	cpi->remote_pad = pad;
	return 0;
}

static int cpi_get_sensor_ep_parameters(struct media_entity *start,
					struct plat_csi_sensor_info *sensor,
		int max_sensors)
{
	struct media_entity *entity;
	struct media_graph graph;
	int found = 0;
	int ret = 0;

	ret = media_graph_walk_init(&graph, start->graph_obj.mdev);
	if (ret)
		return ret;

	media_graph_walk_start(&graph, start);

	while ((entity = media_graph_walk_next(&graph))) {
		if (entity->function != MEDIA_ENT_F_CAM_SENSOR)
			continue;

		if (is_media_entity_v4l2_subdev(entity)) {
			struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
			struct fwnode_handle *handle =
				fwnode_graph_get_next_endpoint(sd->fwnode, NULL);

			ret = v4l2_fwnode_endpoint_parse(handle,
							 &sensor[found].cmos_ep_config);
			if (ret) {
				fwnode_handle_put(handle);
				media_graph_walk_cleanup(&graph);
				return ret;
			}

			fwnode_handle_put(handle);

			sensor[found].subdev = sd;
			found++;

			if (found >= max_sensors)
				break;
		}
	}

	media_graph_walk_cleanup(&graph);

	return found;
}

static int cpi_subdev_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct cpi_dev *cpi = notifier_to_plat_csi(notifier);
	struct device *dev = &cpi->pdev->dev;
	int sensors_found = 0;
	int ret;
	int i;

	sensors_found = cpi_get_sensor_ep_parameters(&cpi->subdev.entity,
						     cpi->sensor,
				ALIF_MAX_CSI_SENSORS);
	if (sensors_found < 0) {
		dev_err(dev, "error while parsing CMOS sensor endpoint config: %d\n",
			sensors_found);
		return sensors_found;
	} else if (sensors_found == 0) {
		cpi->num_sensors = 0;
	} else {
		cpi->num_sensors = sensors_found;
	}

	for (i = 0; i < min(cpi->num_sensors, ALIF_MAX_CSI_SENSORS); i++)
		cpi->cmos_ep_config[i] = cpi->sensor[i].cmos_ep_config;

	if (cpi->is_isp_connected)
		return 0;

	ret = v4l2_device_register_subdev_nodes(&cpi->v4l2_dev);
	if (ret)
		return ret;

	/*
	 * Only now, after the remote CSI-2/sensor subdev has bound
	 * (cpi->remote_sd is set), is it safe to expose /dev/videoX
	 * to userspace: VIDIOC_STREAMON would otherwise be able to run
	 * before cpi->remote_sd is set, crashing in cpi_enable_streams().
	 */
	return cpi_register_video_device(cpi);
}

static const struct v4l2_async_notifier_operations cpi_subdev_notifier_ops = {
	.bound = cpi_subdev_notifier_bound,
	.complete = cpi_subdev_notifier_complete,
};

static int cpi_subdev_parse_dt(struct cpi_dev *cpi)
{
	struct platform_device *pdev = cpi->pdev;
	struct device *dev = &pdev->dev;
	struct device_node *of_node = dev->of_node;
	struct fwnode_handle *handle;
	struct v4l2_fwnode_endpoint ep = {};
	struct v4l2_async_connection *asd;
	int ret = 0;

	cpi->code_10_on_8 = of_property_read_bool(of_node, "alif,cpi-code-10-on-8");
	cpi->msb = of_property_read_bool(of_node, "alif,cpi-msb-first");
	cpi->pclk_active = of_property_read_bool(of_node, "alif,cpi-pclk-active-high");
	cpi->vsync_en = of_property_read_bool(of_node, "alif,cpi-vsync-en");
	cpi->wait_vsync_event = of_property_read_bool(of_node, "alif,cpi-wait-vsync");

	handle = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev),
						 PORT_INPUT, 0, FWNODE_GRAPH_ENDPOINT_NEXT);

	ret = v4l2_fwnode_endpoint_parse(handle, &ep);
	if (ret) {
		dev_err(dev, "error parsing input port: %d\n", ret);
		goto dt_parse_err;
	}

	cpi->cmos_ep_config[0] = ep;

	if (ep.bus_type == V4L2_MBUS_PARALLEL)
		cpi->is_parallel_interface = true;
	else
		cpi->is_parallel_interface = false;

	asd = v4l2_async_nf_add_fwnode_remote(&cpi->subdev_notifier, handle,
					      struct v4l2_async_connection);
	if (IS_ERR(asd)) {
		ret = PTR_ERR(asd);
		goto dt_parse_err;
	}

dt_parse_err:
	fwnode_handle_put(handle);
	return ret;
}

static int cpi_create_bridge_subdev(struct cpi_dev *cpi)
{
	struct v4l2_subdev *sd = &cpi->subdev;
	struct device *dev = &cpi->pdev->dev;
	int ret;

	v4l2_subdev_init(sd, &cpi_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "alif-cpi");

	cpi->subdev_pads[CPI_PAD_SINK].flags =
		MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	cpi->subdev_pads[CPI_PAD_SOURCE].flags =
		MEDIA_PAD_FL_SOURCE | MEDIA_PAD_FL_MUST_CONNECT;
	cpi->subdev_pads[CPI_PAD_SOURCE_ISP].flags =
		MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, CPI_PADS_NUM,
				     cpi->subdev_pads);
	if (ret)
		return ret;

	sd->internal_ops = &cpi_subdev_internal_ops;
	sd->owner = THIS_MODULE;
	sd->dev = &cpi->pdev->dev;
	v4l2_set_subdevdata(sd, cpi);

	ret = v4l2_subdev_init_finalize(sd);
	if (ret) {
		media_entity_cleanup(&sd->entity);
		return ret;
	}

	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &cpi_subdev_mdev_ops;

	if (cpi->is_isp_connected) {
		ret = v4l2_async_register_subdev(sd);
		if (ret) {
			dev_err(dev, "v4l2 async subdev register failed: %d\n", ret);
			goto media_pads_uninit;
		}
		v4l2_async_subdev_nf_init(&cpi->subdev_notifier, sd);
	} else {
		ret = v4l2_device_register_subdev(&cpi->v4l2_dev, sd);
		if (ret) {
			dev_err(dev, "v4l2 sub-device register failed: %d\n", ret);
			goto media_pads_uninit;
		}
		v4l2_async_nf_init(&cpi->subdev_notifier, &cpi->v4l2_dev);
	}

	cpi->subdev_notifier.ops = &cpi_subdev_notifier_ops;

	ret = cpi_subdev_parse_dt(cpi);
	if (ret)
		goto v4l2_async_register_failed;

	ret = v4l2_async_nf_register(&cpi->subdev_notifier);
	if (ret) {
		dev_err(sd->dev, "failed to register V4L2 async notifier: %d\n",
			ret);
		goto v4l2_async_register_failed;
	}

	return 0;
v4l2_async_register_failed:
	if (cpi->is_isp_connected)
		v4l2_async_unregister_subdev(sd);
	else
		v4l2_device_unregister_subdev(sd);
	v4l2_async_nf_cleanup(&cpi->subdev_notifier);
media_pads_uninit:
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	return ret;
}

static void cpi_unregister_subdev(struct cpi_dev *cpi)
{
	struct v4l2_subdev *sd = &cpi->subdev;

	if (cpi->is_isp_connected)
		v4l2_async_unregister_subdev(sd);
	else
		v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_set_subdevdata(sd, NULL);
}

static inline void cpi_change_buffer(struct cpi_dev *cpi)
{
	unsigned long flags;

	spin_lock_irqsave(&cpi->slock, flags);

	if (cpi->active) {
		struct vb2_v4l2_buffer *vbuf = &cpi->active->vb;
		struct rx_buffer *buf = cpi->active;

		list_del_init(&buf->list);
		vbuf->vb2_buf.timestamp = ktime_get_ns();
		vbuf->sequence = cpi->sequence++;
		vbuf->field = cpi->format.fmt.pix.field;
		vb2_buffer_done(&vbuf->vb2_buf, VB2_BUF_STATE_DONE);
	}

	if (cpi->stopping || list_empty(&cpi->fb_list_head)) {
		cpi->active = NULL;
	} else {
		cpi->active = list_entry(cpi->fb_list_head.next, struct rx_buffer, list);
		cpi_hw_setup_buffer(cpi);
		cpi_hw_start_video_capture(cpi);
	}

	spin_unlock_irqrestore(&cpi->slock, flags);
}

static irqreturn_t cpi_isr(int irq, void *dev)
{
	struct cpi_dev *cpi = dev;
	struct platform_device *pdev = cpi->pdev;
	u32 capture_error_mask;
	u32 int_st;

	capture_error_mask = INTR_INFIFO_OVERRUN |
		INTR_OUTFIFO_OVERRUN |
		INTR_BRESP_ERR;
	int_st = readl(cpi->base_addr + CPI_INTR) &
		readl(cpi->base_addr + CPI_INTR_ENA);
	writel(int_st, cpi->base_addr + CPI_INTR);

	if (!int_st)
		return IRQ_NONE;

	if (int_st & INTR_HSYNC)
		dev_dbg(&pdev->dev, "CPI HSYNC interrupt\n");

	if (int_st & INTR_VSYNC)
		dev_dbg(&pdev->dev, "CPI VSYNC interrupt\n");

	if (int_st & INTR_BRESP_ERR) {
		u32 axi_err_stat =
			readl(cpi->base_addr + CPI_AXI_ERR_STAT);
		dev_err(&pdev->dev,
			"AXI Bus response error, BRESP code - %d\n",
			axi_err_stat & 0x3);
	}

	if (int_st & capture_error_mask)
		dev_err(&pdev->dev, "frame capture error, int_st = 0x%08x\n",
			int_st);

	if (int_st & INTR_STOP) {
		dev_dbg(&pdev->dev, "capture complete\n");
		if (cpi_is_userptr(cpi))
			return IRQ_WAKE_THREAD;
		cpi_change_buffer(cpi);
	}

	return IRQ_HANDLED;
}

static irqreturn_t cpi_isr_thread(int irq, void *dev)
{
	struct cpi_dev *cpi = dev;

	cpi_copy_bounce_to_user(cpi);
	cpi_change_buffer(cpi);

	return IRQ_HANDLED;
}

static int cpi_isp_present(struct cpi_dev *cpi)
{
	struct device *dev = &cpi->pdev->dev;
	struct fwnode_handle *handle;

	handle = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev),
						 PORT_OUTPUT, 0, 0);

	if (!handle) {
		cpi->is_isp_connected = false;
		return 0;
	}

	fwnode_handle_put(handle);
	cpi->is_isp_connected = true;

	return 0;
}

static int cpi_v4l2_setup(struct cpi_dev *cpi)
{
	struct media_device *mdev = &cpi->media_dev;
	struct device *dev = &cpi->pdev->dev;
	struct v4l2_device *v4l2_dev;
	int ret;

	strscpy(mdev->model, "Alif Platform", sizeof(mdev->model));

	mdev->ops = &cpi_media_ops;
	mdev->dev = dev;
	mdev->hw_revision = 0;

	v4l2_dev = &cpi->v4l2_dev;
	v4l2_dev->mdev = mdev;
	strscpy(v4l2_dev->name, "alif-cam-pipeline", sizeof(v4l2_dev->name));

	media_device_init(mdev);

	ret = media_device_register(mdev);
	if (ret) {
		dev_err(dev, "Failed to register media device: %d\n", ret);
		goto error_media_device_registration;
	}

	ret = v4l2_device_register(dev, v4l2_dev);
	if (ret) {
		dev_err(dev, "Failed to register V4L2 device: %d\n", ret);
		goto error_v4l2_registration;
	}

	return 0;
error_v4l2_registration:
	media_device_unregister(mdev);
error_media_device_registration:
	media_device_cleanup(mdev);

	return ret;
}

static int cpi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = 0;
	struct cpi_dev *cpi;
	struct resource *res;

	if (!dev->of_node)
		return -ENODEV;

	cpi = devm_kzalloc(dev, sizeof(*cpi), GFP_KERNEL);
	if (!cpi)
		return -ENOMEM;

	cpi->pdev = pdev;

	platform_set_drvdata(pdev, cpi);

	/* Use reserved SRAM region for frame buffer DMA allocations */
	of_reserved_mem_device_init(dev);

	cpi->active = NULL;
	cpi->axi_bus_ep = false;
	cpi->in_pipeline_propagation = false;
	spin_lock_init(&cpi->slock);
	mutex_init(&cpi->lock);
	INIT_LIST_HEAD(&cpi->fb_list_head);

	cpi->fmt = &cpi_formats[0];
	cpi->format.fmt.pix.width = 560;
	cpi->format.fmt.pix.height = 560;
	cpi->format.fmt.pix.field = V4L2_FIELD_NONE;
	cpi->format.fmt.pix.pixelformat = cpi->fmt->fourcc;
	cpi->format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
	cpi->format.fmt.pix.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	cpi->format.fmt.pix.quantization = V4L2_QUANTIZATION_DEFAULT;
	cpi->format.fmt.pix.xfer_func = V4L2_XFER_FUNC_DEFAULT;

	cpi->version = (u32)(unsigned long)of_device_get_match_data(&pdev->dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	cpi->base_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(cpi->base_addr)) {
		dev_err(dev, "Base address not set.\n");
		return PTR_ERR(cpi->base_addr);
	}

	cpi->irq = platform_get_irq(pdev, 0);
	if (cpi->irq < 0) {
		dev_err(&cpi->pdev->dev, "platform_get_irq() failed with %d\n",
			cpi->irq);
		return cpi->irq;
	}

	ret = devm_request_threaded_irq(&pdev->dev, cpi->irq, cpi_isr,
					cpi_isr_thread, IRQF_ONESHOT,
					"CPI", cpi);
	if (ret) {
		dev_err(&cpi->pdev->dev, "devm_request_threaded_irq() failed with %d\n",
			ret);
		return ret;
	}

	cpi->apb_clk = devm_clk_get(dev, "apb_clk");
	if (IS_ERR(cpi->apb_clk)) {
		ret = PTR_ERR(cpi->apb_clk);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get APB clock: %d\n", ret);
		return ret;
	}

	cpi->pix_clk = devm_clk_get(dev, "pix_clk");
	if (IS_ERR(cpi->pix_clk)) {
		if (PTR_ERR(cpi->pix_clk) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		/* Pixel clock is optional, set to NULL if not found */
		cpi->pix_clk = NULL;
	}

	ret = clk_prepare_enable(cpi->apb_clk);
	if (ret) {
		dev_err(dev, "Failed to enable APB clock: %d\n", ret);
		return ret;
	}

	cpi_isp_present(cpi);

	if (!cpi->is_isp_connected) {
		ret = cpi_v4l2_setup(cpi);
		if (ret)
			goto err_disable_apb_clk;
	}

	ret = cpi_create_bridge_subdev(cpi);
	if (ret)
		goto err_v4l2_cleanup;

	dev_info(dev, "Alif CPI device registered\n");
	return 0;

err_v4l2_cleanup:
	if (!cpi->is_isp_connected) {
		v4l2_device_unregister(&cpi->v4l2_dev);
		media_device_unregister(&cpi->media_dev);
		media_device_cleanup(&cpi->media_dev);
	}
err_disable_apb_clk:
	clk_disable_unprepare(cpi->apb_clk);
	dev_err(dev, "alif CPI probe failed: %d\n", ret);
	return ret;
}

static void cpi_remove(struct platform_device *pdev)
{
	struct cpi_dev *cpi = platform_get_drvdata(pdev);

	cpi_set_pixclk(cpi, 0, false);

	if (cpi->apb_clk)
		clk_disable_unprepare(cpi->apb_clk);

	v4l2_async_nf_unregister(&cpi->subdev_notifier);
	v4l2_async_nf_cleanup(&cpi->subdev_notifier);
	cpi_unregister_subdev(cpi);

	if (!cpi->is_isp_connected) {
		v4l2_device_unregister(&cpi->v4l2_dev);
		media_device_unregister(&cpi->media_dev);
		media_device_cleanup(&cpi->media_dev);
		vb2_queue_release(&cpi->vb_queue);
	}

	cpi_free_bounce(cpi);
	of_reserved_mem_device_release(&pdev->dev);
}

static const struct of_device_id cpi_of_match[] = {
	{ .compatible = "alif,ensemble-cpi", .data = (void *)CPI_VERSION_1 },
	{ .compatible = "alif,ensemble-cpi2", .data = (void *)CPI_VERSION_2 },
	{}
};

MODULE_DEVICE_TABLE(of, cpi_of_match);

static struct platform_driver __refdata cpi_pdrv = {
	.remove = cpi_remove,
	.probe = cpi_probe,
	.driver = {
		   .name = "alif-cpi",
		   .owner = THIS_MODULE,
		   .of_match_table = cpi_of_match,
		   },
};

module_platform_driver(cpi_pdrv);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harith George <harith.g@alifsemi.com>");
MODULE_DESCRIPTION("Driver for Camera Parallel Interface Controller");
