// SPDX-License-Identifier: GPL-2.0
/*
 * pci-of-overlay: Generic driver for PCI endpoints whose internals are
 * described by a firmware-provided device-tree overlay.
 *
 * Two entry points are exported:
 *   pci_of_overlay_fdt_apply() -- takes the raw FDT bytes; registers an
 *     irqdomain backed by MSI-X/MSI (one hwirq per vector) or by shared
 *     INTx (single hwirq 0 demuxed in leaf drivers), applies the overlay,
 *     and populates platform devices for stock subsystem drivers to probe.
 *   pci_of_overlay_fw_apply() -- thin wrapper that acquires the overlay
 *     via request_firmware() before calling the FDT-apply entry point.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdesc.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/pci-of-overlay.h>
#include <linux/slab.h>

/* Recover the owning pci_of_overlay from a flex-array element. */
static inline struct pci_of_overlay *pov_from_vec(struct pov_vector *v)
{
	return (void *)(v - v->hwirq) - offsetof(struct pci_of_overlay, vec);
}

/*
 * Each parent MSI/MSI-X vector demuxes to exactly one child hwirq in this
 * domain (1:1 map). There is no shared-vector demux state to write, no
 * physical mask register to manipulate. dummy_irq_chip provides all-noop
 * mask/unmask/ack ops for exactly this case; the child virq's own
 * irq_desc depth counter is what enable_irq/disable_irq manipulate at the
 * leaf-driver level. Touching the parent chained vector via
 * enable_irq/disable_irq from mask ops would unbalance its depth counter
 * (WARN "Unbalanced enable for IRQ X").
 */
static int pov_irq_map(struct irq_domain *d, unsigned int virq,
		       irq_hw_number_t hw)
{
	struct pci_of_overlay *pov = d->host_data;

	if (hw >= (irq_hw_number_t)pov->nvec)
		return -EINVAL;

	irq_set_chip_and_handler(virq, &dummy_irq_chip, handle_simple_irq);
	irq_set_chip_data(virq, pov);
	return 0;
}

static const struct irq_domain_ops pov_irq_domain_ops = {
	.map	= pov_irq_map,
	.xlate	= irq_domain_xlate_onecell,
};

static void pov_msi_chained_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct pov_vector *v = irq_desc_get_handler_data(desc);
	struct pci_of_overlay *pov = pov_from_vec(v);

	chained_irq_enter(chip, desc);
	generic_handle_domain_irq(pov->irq_domain, v->hwirq);
	chained_irq_exit(chip, desc);
}

static irqreturn_t pov_intx_handler(int irq, void *data)
{
	struct pci_of_overlay *pov = data;

	return generic_handle_domain_irq(pov->irq_domain, 0) ?
		IRQ_NONE : IRQ_HANDLED;
}

static void pov_msi_unchain(void *data)
{
	struct pci_of_overlay *pov = data;
	int i;

	for (i = 0; i < pov->nvec; i++)
		irq_set_chained_handler_and_data(pov->vec[i].parent_irq,
						 NULL, NULL);
}

static void pov_intx_free(void *data)
{
	struct pci_of_overlay *pov = data;

	free_irq(pov->vec[0].parent_irq, pov);
}

static void pov_remove_irq_domain(void *data)
{
	irq_domain_remove(data);
}

static void pov_free_irq_vectors(void *data)
{
	pci_free_irq_vectors(data);
}

/*
 * Allocate exactly @nvec_req MSI/MSI-X (or INTx) vectors. @nvec_req == 0
 * means the overlay declares no interrupt lines: return 0 without touching
 * PCI IRQ state so the caller can skip irqdomain setup. Otherwise, passing
 * @nvec_req as both the min and max of pci_alloc_irq_vectors() makes it
 * fail cleanly if the host cannot grant that many -- required to preserve
 * the 1:1 vector-to-hwirq contract documented on pci_of_overlay_fdt_apply().
 */
static int pov_alloc_vectors(struct pci_dev *pdev, unsigned int nvec_req)
{
	int nvec, ret;

	if (nvec_req == 0)
		return 0;

	nvec = pci_alloc_irq_vectors(pdev, nvec_req, nvec_req, PCI_IRQ_ALL_TYPES);
	if (nvec < 0)
		return dev_err_probe(&pdev->dev, nvec,
				     "IRQ vector allocation failed (want %u)\n",
				     nvec_req);

	ret = devm_add_action_or_reset(&pdev->dev, pov_free_irq_vectors, pdev);
	if (ret)
		return ret;

	return nvec;
}

/*
 * Populate the vector table, create the irqdomain, and hook the parent
 * MSI/INTx handlers. Assumes pov->vec[] is already sized to pov->nvec.
 * A pov->nvec == 0 is a valid no-op (overlay declares no interrupt lines).
 */
static int pov_setup_irq_domain(struct pci_dev *pdev, struct pci_of_overlay *pov)
{
	struct device *dev = &pdev->dev;
	struct fwnode_handle *fwnode;
	unsigned int i;
	int ret;

	if (pov->nvec == 0)
		return 0;

	fwnode = of_fwnode_handle(dev_of_node(dev));
	if (!fwnode)
		return dev_err_probe(dev, -ENODEV,
				     "no of_node fwnode for irqdomain\n");

	for (i = 0; i < pov->nvec; i++) {
		ret = pci_irq_vector(pdev, i);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "pci_irq_vector(%u) failed\n", i);

		pov->vec[i].hwirq = i;
		pov->vec[i].parent_irq = ret;
	}

	pov->irq_domain = irq_domain_create_linear(fwnode, pov->nvec,
						   &pov_irq_domain_ops, pov);
	if (!pov->irq_domain)
		return dev_err_probe(dev, -ENOMEM,
				     "failed to create irqdomain\n");

	ret = devm_add_action_or_reset(dev, pov_remove_irq_domain,
				       pov->irq_domain);
	if (ret)
		return ret;

	if (pci_dev_msi_enabled(pdev)) {
		/* Chain each MSI vector: nothing runs on parent_irq otherwise. */
		for (i = 0; i < pov->nvec; i++)
			irq_set_chained_handler_and_data(pov->vec[i].parent_irq,
							 pov_msi_chained_handler,
							 &pov->vec[i]);
		return devm_add_action_or_reset(dev, pov_msi_unchain, pov);
	}

	ret = request_irq(pov->vec[0].parent_irq, pov_intx_handler,
			  IRQF_SHARED, dev_name(dev), pov);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request INTx irq %d\n",
				     pov->vec[0].parent_irq);

	return devm_add_action_or_reset(dev, pov_intx_free, pov);
}

static void pci_of_overlay_remove(void *data)
{
	of_overlay_remove(data);
}

static void pci_of_overlay_depopulate(void *data)
{
	of_platform_depopulate(data);
}

struct pci_of_overlay *pci_of_overlay_fdt_apply(struct pci_dev *pdev,
						unsigned int nvec_req,
						const void *fdt, size_t size)
{
	struct device *dev = &pdev->dev;
	struct pci_of_overlay *pov;
	int ret, nvec;

	if (!fdt || !size)
		return ERR_PTR(-EINVAL);

	/*
	 * PCI core only auto-creates an of_node for bridges and for a handful
	 * of quirked endpoint VID:DIDs; a generic FPGA endpoint won't have one
	 * yet. Ensure one exists so leaf platform devices spawned by the
	 * overlay can inherit iommu/dma-mapping/interrupt config from the PCI
	 * parent. If it already exists (DT system, or an upstream quirk fired),
	 * the helper is a no-op and its creator retains ownership.
	 */
	ret = devm_of_pci_make_dev_node(pdev);
	if (ret)
		return ERR_PTR(ret);

	nvec = pov_alloc_vectors(pdev, nvec_req);
	if (nvec < 0)
		return ERR_PTR(nvec);

	pov = devm_kzalloc(dev, struct_size(pov, vec, nvec), GFP_KERNEL);
	if (!pov)
		return ERR_PTR(-ENOMEM);
	pov->nvec = nvec;

	ret = pov_setup_irq_domain(pdev, pov);
	if (ret)
		return ERR_PTR(ret);

	ret = of_overlay_fdt_apply(fdt, size, &pov->ovcs_id, dev_of_node(dev));
	if (ret)
		return dev_err_ptr_probe(dev, ret, "failed to apply overlay\n");

	ret = devm_add_action_or_reset(dev, pci_of_overlay_remove,
				       &pov->ovcs_id);
	if (ret)
		return ERR_PTR(ret);

	ret = of_platform_default_populate(dev_of_node(dev), NULL, dev);
	if (ret)
		return dev_err_ptr_probe(dev, ret,
					 "failed to populate platform devs\n");

	ret = devm_add_action_or_reset(dev, pci_of_overlay_depopulate, dev);
	if (ret)
		return ERR_PTR(ret);

	return pov;
}
EXPORT_SYMBOL_GPL(pci_of_overlay_fdt_apply);

struct pci_of_overlay *pci_of_overlay_fw_apply(struct pci_dev *pdev,
					       unsigned int nvec_req,
					       const char *fw_name)
{
	struct device *dev = &pdev->dev;
	struct pci_of_overlay *pov;
	const struct firmware *fw;
	int ret;

	if (!fw_name)
		return ERR_PTR(-EINVAL);

	ret = request_firmware(&fw, fw_name, dev);
	if (ret)
		return dev_err_ptr_probe(dev, ret, "missing overlay %s\n",
					 fw_name);

	pov = pci_of_overlay_fdt_apply(pdev, nvec_req, fw->data, fw->size);
	release_firmware(fw);
	return pov;
}
EXPORT_SYMBOL_GPL(pci_of_overlay_fw_apply);

static char *overlay;
module_param(overlay, charp, 0644);
MODULE_PARM_DESC(overlay, "DTB overlay firmware name to apply on next bind");

static unsigned int nvec;
module_param(nvec, uint, 0644);
MODULE_PARM_DESC(nvec, "Number of MSI/MSI-X vectors to allocate");

static int pov_generic_probe(struct pci_dev *pdev,
			     const struct pci_device_id *id)
{
	struct pci_of_overlay *pov;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	pci_set_master(pdev);

	pov = pci_of_overlay_fw_apply(pdev, nvec, overlay);
	if (IS_ERR(pov))
		return PTR_ERR(pov);

	return 0;
}

static struct pci_driver pov_generic_driver = {
	.name		= "pci-of-overlay",
	.probe		= pov_generic_probe,
};
module_pci_driver(pov_generic_driver);

MODULE_DESCRIPTION("Generic PCI OF overlay");
MODULE_AUTHOR("Rodrigo Alencar <rodrigo.alencar@analog.com>");
MODULE_LICENSE("GPL");
