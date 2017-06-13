/*
 * (C) Copyright 2016 Nexell
 * Hyunseok, Jung <hsjung@nexell.co.kr>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <common.h>
#include <asm/errno.h>
#include <linux/list.h>
#include <malloc.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <asm/io.h>

#include <asm/arch/nexell.h>
#include <asm/arch/reset.h>

#include "nexell_udc_otg_regs.h"
#include "dwc2_udc_otg_priv.h"
#include <usb/lin_gadget_compat.h>
#include <usb/dwc2_udc.h>

void otg_phy_init(struct dwc2_udc *dev)
{
	void __iomem *phy = (void __iomem *)(uintptr_t)dev->pdata->regs_phy;
	u32 reg;

	/* USB PHY0 Enable */
	debug("USB PHY0 Enable\n");

	writel(readl(phy + NX_OTG_CON3) & ~NX_OTG_CON3_DET_N_CHG,
	       phy + NX_OTG_CON3);

	nx_rstcon_setrst(RESET_ID_USB20OTG, RSTCON_ASSERT);
	udelay(10);
	nx_rstcon_setrst(RESET_ID_USB20OTG, RSTCON_NEGATE);
	udelay(10);

	reg  = readl(phy + NX_OTG_CON2) & ~NX_OTG_CON2_OTGTUNE_MASK;
	writel(reg, phy +  NX_OTG_CON2);

	writel(readl(phy + NX_OTG_CON0) & ~NX_OTG_CON0_SS_SCALEDOWN_MODE,
	       phy + NX_OTG_CON0);

	writel(readl(phy + NX_OTG_CON2) | NX_OTG_CON2_WORDINTERFACE_16,
	       phy + NX_OTG_CON2);

	writel(readl(phy + NX_OTG_CON1) & NX_OTG_CON1_VBUS_INTERNAL,
	       phy + NX_OTG_CON1);
	udelay(10);

	reg = readl(phy + NX_OTG_CON1);
	reg &= ~NX_OTG_CON1_POR_MASK;
	reg |= NX_OTG_CON1_POR_ENB;
	writel(reg, phy + NX_OTG_CON1);
	udelay(10);
	reg |= NX_OTG_CON1_POR_MASK;
	writel(reg, phy + NX_OTG_CON1);
	udelay(10);
	reg &= ~NX_OTG_CON1_POR;
	writel(reg, phy + NX_OTG_CON1);
	udelay(40);

	writel(readl(phy + NX_OTG_CON1) | NX_OTG_CON1_RST, phy + NX_OTG_CON1);
	udelay(10);

	writel(readl(phy + NX_OTG_CON1) | NX_OTG_CON1_UTMI_RST,
	       phy + NX_OTG_CON1);
	udelay(10);
}

void otg_phy_off(struct dwc2_udc *dev)
{
	void __iomem *phy = (void __iomem *)(uintptr_t)dev->pdata->regs_phy;

	/* USB PHY0 Disable */
	debug("USB PHY0 Disable\n");

	writel(readl(phy + NX_OTG_CON1) | NX_OTG_CON1_VBUS_VLDEXT0,
	       phy + NX_OTG_CON1);
	udelay(10);

	writel(readl(phy + NX_OTG_CON1) & ~NX_OTG_CON1_RST, phy + NX_OTG_CON1);
	udelay(10);

	writel(readl(phy + NX_OTG_CON1) & ~NX_OTG_CON1_UTMI_RST,
	       phy + NX_OTG_CON1);
	udelay(10);

	writel(readl(phy + NX_OTG_CON1) | NX_OTG_CON1_POR_MASK,
	       phy + NX_OTG_CON1);
	udelay(10);

	nx_rstcon_setrst(RESET_ID_USB20OTG, RSTCON_ASSERT);
	udelay(10);
}
