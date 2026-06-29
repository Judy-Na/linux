/* SPDX-License-Identifier: GPL-2.0 */
/*
 * pci-of-overlay: module for PCI drivers that describe their endpoint's
 * internals via a firmware-provided device-tree overlay.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#ifndef __LINUX_PCI_OF_OVERLAY_H
#define __LINUX_PCI_OF_OVERLAY_H

#include <linux/compiler_attributes.h>
#include <linux/types.h>

struct irq_domain;
struct pci_dev;
struct pci_of_overlay;

/*
 * Per-MSI-vector state referenced by the irqdomain. Kept in a flex array at
 * the end of struct pci_of_overlay so both the fixed state and the vector
 * table live in one allocation. The owning pci_of_overlay is recovered from
 * a pov_vector pointer via container math (see pov_from_vec()).
 */
struct pov_vector {
	unsigned int hwirq;
	int parent_irq;
};

/**
 * struct pci_of_overlay - per-device state populated by the module.
 * @irq_domain: irqdomain covering the endpoint's MSI/MSI-X vectors (or a
 *              single hwirq 0 for shared INTx). Callers may read this to
 *              wire irq lookups from leaf platform devices.
 * @nvec: number of entries in @vec; also the size of the irqdomain.
 *
 * The module allocates and owns this struct (devm_-tied to the PCI device);
 * callers receive it back from the *_apply() entry points. All fields not
 * documented above are private.
 */
struct pci_of_overlay {
	struct irq_domain *irq_domain;
	unsigned int nvec;

	/* private: */
	int ovcs_id;
	struct pov_vector vec[] __counted_by(nvec);
};

/**
 * pci_of_overlay_fdt_apply() - set up IRQs, apply an in-memory DT overlay,
 * and populate platform devices behind a PCI endpoint.
 * @pdev: PCI device the overlay describes. The caller must have already
 *        enabled the device (pcim_enable_device() or pci_enable_device())
 *        and set bus-master where appropriate (pci_set_master()); this
 *        module only decorates the endpoint, it does not take ownership of
 *        PCI power/enable state. Either enable API works: only the caller
 *        is responsible for pairing an enable with a disable on teardown.
 * @nvec: exact number of MSI/MSI-X vectors to allocate, or 0 if the overlay
 *        declares no interrupt lines. Non-zero values are passed as both
 *        min and max to pci_alloc_irq_vectors(), so probe fails cleanly
 *        if the host cannot grant exactly that many (see "Interrupt model"
 *        below for why this is a hard requirement).
 * @fdt: raw FDT bytes (device-tree overlay blob).
 * @size: size of @fdt in bytes.
 *
 * Return: a devm_-allocated struct pci_of_overlay on success (freed on
 *         pdev device teardown), or an ERR_PTR() on failure.
 *
 * Interrupt model
 * ---------------
 * On success the returned pov->irq_domain provides a strict 1:1 mapping
 * between child hwirq N (0..@nvec-1) and PCI MSI/MSI-X vector N. Leaf
 * peripherals declared in the DTBO reach vector N via
 *
 *   interrupts = <N>;
 *
 * Vector sharing is intentionally not supported: allocating fewer vectors
 * than @nvec would let two overlay hwirqs alias onto one MSI vector, and
 * the module refuses that up front by requesting min == max.
 *
 * When the host cannot grant MSI/MSI-X, pci_alloc_irq_vectors() falls back
 * to legacy INTx, which has exactly one shared line. In that case an
 * @nvec > 1 request will fail probe. If the overlay needs more than one
 * child IRQ line under INTx, model an interrupt controller inside the
 * overlay itself (e.g. axi_intc): the controller consumes the single INTx
 * hwirq 0 from this module and re-exposes N cascaded lines under its own
 * `interrupt-controller` node, which the leaf peripherals then reference.
 */
struct pci_of_overlay *pci_of_overlay_fdt_apply(struct pci_dev *pdev,
						unsigned int nvec,
						const void *fdt, size_t size);

/**
 * pci_of_overlay_fw_apply() - request an overlay via request_firmware() and
 * hand it to pci_of_overlay_fdt_apply().
 * @pdev: PCI device the overlay describes.
 * @nvec: same semantics as for pci_of_overlay_fdt_apply().
 * @fw_name: firmware name of the dtbo to request; must be non-NULL.
 */
struct pci_of_overlay *pci_of_overlay_fw_apply(struct pci_dev *pdev,
					       unsigned int nvec,
					       const char *fw_name);

#endif /* __LINUX_PCI_OF_OVERLAY_H */
