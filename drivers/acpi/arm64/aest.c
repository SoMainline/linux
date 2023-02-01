// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023, Linaro Limited
 */

#define pr_fmt(fmt)	"ACPI: AEST: " fmt

#include <linux/acpi_aest.h>
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-map-ops.h>

#include <acpi/actbl2.h>

/* Arm ARM RAS Supplement Chapter 4.1 / (RTPFWF) */
#define RAS_ERR_REC_GRP_WIDTH			0x1000
#define RAS_SINGLE_ERR_REC_WIDTH		0x40
#define AEST_INTR_TYPE_LEVEL			0b1

/* Error interrupt + Fault interrupt + MMIO base */
#define ARM_RAS_RES_COUNT 3

/* Root pointer to the mapped AEST table */
static struct acpi_table_header *aest_table;

struct aest_fwnode {
	struct list_head list; /* aest_fwnode_list links */
	struct acpi_aest_node *aest_node;
	struct fwnode_handle *fwnode;
};
static LIST_HEAD(aest_fwnode_list);
static DEFINE_SPINLOCK(aest_fwnode_lock);

/**
 * aest_set_fwnode() - Create aest_fwnode and use it to register
 *		       iommu data in the aest_fwnode_list
 *
 * @aest_node: AEST table node associated with the IOMMU
 * @fwnode: fwnode associated with the AEST node
 *
 * Returns: 0 on success, negative error code otherwise .
 */
static int aest_set_fwnode(struct acpi_aest_node *aest_node,
			   struct fwnode_handle *fwnode)
{
	struct aest_fwnode *np;

	np = kzalloc(sizeof(struct aest_fwnode), GFP_ATOMIC);

	if (!np)
		return -ENOMEM;

	INIT_LIST_HEAD(&np->list);
	np->aest_node = aest_node;
	np->fwnode = fwnode;

	spin_lock(&aest_fwnode_lock);
	list_add_tail(&np->list, &aest_fwnode_list);
	spin_unlock(&aest_fwnode_lock);

	return 0;
}

/**
 * aest_get_fwnode() - Retrieve fwnode associated with an AEST node
 *
 * @node: AEST table node to be looked-up
 *
 * Returns: fwnode_handle pointer on success, NULL on failure
 */
static struct fwnode_handle *aest_get_fwnode(struct acpi_aest_node *node)
{
	struct aest_fwnode *curr;
	struct fwnode_handle *fwnode = NULL;

	spin_lock(&aest_fwnode_lock);
	list_for_each_entry(curr, &aest_fwnode_list, list) {
		if (curr->aest_node == node) {
			fwnode = curr->fwnode;
			break;
		}
	}
	spin_unlock(&aest_fwnode_lock);

	return fwnode;
}

/**
 * aest_delete_fwnode() - Delete fwnode associated with an AEST node
 *
 * @node: AEST table node associated with fwnode to delete
 */
static inline void aest_delete_fwnode(struct acpi_aest_node *node)
{
	struct aest_fwnode *curr, *tmp;

	spin_lock(&aest_fwnode_lock);
	list_for_each_entry_safe(curr, tmp, &aest_fwnode_list, list) {
		if (curr->aest_node == node) {
			list_del(&curr->list);
			kfree(curr);
			break;
		}
	}
	spin_unlock(&aest_fwnode_lock);
}

/**
 * aest_delete_fwnode() - Delete fwnode associated with an AEST node
 *
 * @hwirq: Index of the hwirq to register
 * @name: Name of the irq to register
 * @trigger: Interrupt trigger
 * @res: Pointer to the associated resource
 *
 * Returns: 0 on success, -EINVAL otherwise
 */
static int __init acpi_aest_register_irq(int hwirq, const char *name,
					 int trigger, struct resource *res)
{
	int irq = acpi_register_gsi(NULL, hwirq, trigger, ACPI_ACTIVE_HIGH);

	if (irq < 0) {
		pr_err("Could not register gsi hwirq %d name [%s]\n", hwirq, name);
		return -EINVAL;
	}

	res->start = irq;
	res->end = irq;
	res->flags = IORESOURCE_IRQ;
	res->name = name;

	return 0;
}

/**
 * aest_get_node_data_size() - Get the size of the node-type-specific data
 *
 * @node: Pointer to the AEST node
 *
 * Returns: Positive data size on success, -EINVAL otherwise
 */
static u64 aest_get_node_data_size(struct acpi_aest_node *node)
{
	u64 struct_off;

	switch (node->hdr.type) {
	case ACPI_AEST_PROCESSOR_ERROR_NODE:
		struct_off = sizeof(node->processor.proc);
		switch (node->processor.proc.resource_type) {
		case ACPI_AEST_CACHE_RESOURCE:
			struct_off += sizeof(acpi_aest_processor_cache);
			break;
		case ACPI_AEST_TLB_RESOURCE:
			struct_off += sizeof(acpi_aest_processor_tlb);
			break;
		case ACPI_AEST_GENERIC_RESOURCE:
			struct_off += sizeof(acpi_aest_processor_generic);
			break;
		default:
			/* Other values are reserved */
			return -EINVAL;
		}
		break;

	case ACPI_AEST_MEMORY_ERROR_NODE:
		struct_off = sizeof(node->mem);
		break;

	case ACPI_AEST_SMMU_ERROR_NODE:
		struct_off = sizeof(node->smmu);
		break;

	case ACPI_AEST_VENDOR_ERROR_NODE:
		struct_off = sizeof(node->vendor);
		break;

	case ACPI_AEST_GIC_ERROR_NODE:
		struct_off = sizeof(node->gic);
		break;

	default:
		/* Other values are reserved */
		return -EINVAL;
	}

	return struct_off;
}

/**
 * aest_init_node() - Initialize the AEST node struct with ACPI data
 *
 * @node: Pointer to the AEST node
 * @data: Pointer to the ACPI data
 * @data: Pointer to the node resource array
 *
 * Returns: 0 on success, negative errno otherwise.
 */
static int __init aest_init_node(struct acpi_aest_node *node,
				 struct acpi_aest_node *data,
				 struct resource *r)
{
	int i, num_res = 0;
	u64 struct_off;
	u8 intr_type;

	data->hdr = node->hdr;

	/* Fill in the variable-size data union based on node type. */
	struct_off = aest_get_node_data_size(node);
	if (struct_off < 0)
		return struct_off;

	/* Copy the node-type-specific data, which resides after the header. */
	memcpy(data + sizeof(data->hdr), node + sizeof(node->hdr), struct_off);

	/* The data field can have different sizes, so we need some clever pointer math here! */
	memcpy(&data->intf, (void *)(node + struct_off), sizeof(data->intf));

	struct_off += sizeof(acpi_aest_node_interface);

	memcpy(&data->intr, (void *)(node + struct_off),
	       struct_size(data, intr, data->hdr.node_interrupt_count));

	if (node->intf.address) {
		if (node->intf.type == ACPI_AEST_NODE_SYSTEM_REGISTER)
			pr_err("Faulty table! MMIO address specified for a SR interface!\n");

		r[num_res].start = node->intf.address;
		/*
		 * TODO: The 4.3.1.4 Memory-mapped single error record view might be repeated in
		 * the control registers for a memory-mapped component that implements a small
		 * number of error records. Each error record has its own IMPLEMENTATION DEFINED
		 * base within the control registers of the component.
		 *  - Arm ARM RAS Supplement Chapter 4.1 / (RDHYDC)
		 */

		/* BIG TODO: what if multiple nodes point to the same error record group? :// */

		//TODO: this if condition is a guesstimate
		if (node->intf.error_record_count == 1)
			r[num_res].end = node->intf.address + RAS_SINGLE_ERR_REC_WIDTH - 1;
		else
			r[num_res].end = node->intf.address + RAS_ERR_REC_GRP_WIDTH - 1;
		r[num_res].flags = IORESOURCE_MEM;
		num_res++;
	}

	num_res += node->hdr.node_interrupt_count;

	/* Register interrupts, they will be requested in the EDAC driver later. */
	// TODO: handle MSIs?
	for (i = 0; i < node->hdr.node_interrupt_count; i ++) {
		intr_type = node->intr[i].flags = AEST_INTR_TYPE_LEVEL ?
						  ACPI_LEVEL_SENSITIVE : ACPI_EDGE_SENSITIVE;
		if (node->intr[i].type == ACPI_AEST_NODE_FAULT_HANDLING)
			acpi_aest_register_irq(node->intr[i].gsiv, "fault",
					       intr_type, &r[num_res++]);
		if (node->intr[i].type == ACPI_AEST_NODE_ERROR_RECOVERY)
			acpi_aest_register_irq(node->intr[i].gsiv, "err",
					       intr_type, &r[num_res++]);
		else {
			pr_err("Faulty table! Illegal interrupt type %u\n", node->intr[i].type);
			return -EINVAL;
		}
	}

	return num_res;
}

/**
 * aest_add_platform_device() - Allocate a platform device for AEST node
 * @node: Pointer to device ACPI AEST node
 * @ops: Pointer to AEST device config struct
 *
 * Returns: 0 on success, negative errno on failure.
 */
static int __init aest_add_platform_device(struct acpi_aest_node *node)
{
	struct fwnode_handle *fwnode;
	struct platform_device *pdev;
	struct acpi_aest_node *data;
	struct resource *r;
	int ret, count;

	pdev = platform_device_alloc("arm-ras-edac", PLATFORM_DEVID_AUTO);
	if (!pdev)
		return -ENOMEM;

	r = kcalloc(ARM_RAS_RES_COUNT, sizeof(*r), GFP_KERNEL);
	if (!r) {
		ret = -ENOMEM;
		goto dev_put;
	}

	data = kzalloc(sizeof(data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto dev_put;
	}

	count = aest_init_node(node, data, r);
	if (count < 0) {
		ret = count;
		goto dev_put;
	}

	ret = platform_device_add_resources(pdev, r, count);
	/*
	 * Resources are duplicated in platform_device_add_resources,
	 * free their allocated memory
	 */
	kfree(r);

	if (ret)
		goto dev_put;

	ret = platform_device_add_data(pdev, &node, sizeof(node));
	if (ret)
		goto dev_put;

	fwnode = aest_get_fwnode(node);
	if (!fwnode) {
		ret = -ENODEV;
		goto dev_put;
	}

	pdev->dev.fwnode = fwnode;

	ret = platform_device_add(pdev);

	return ret;

dev_put:
	platform_device_put(pdev);

	return ret;
}

/**
 * aest_init_platform_devices() - Initialize AEST platform devices
 */
static void __init aest_init_platform_devices(void)
{
	struct acpi_aest_node *aest_node, *aest_end;
	struct acpi_table_aest *aest;
	struct fwnode_handle *fwnode;
	int i, ret, node_count = 0;
	u32 tbl_sz;

	/* Get the AEST table, with the correct type */
	aest = container_of(aest_table, struct acpi_table_aest, header);

	/* Get the first AEST node */
	aest_node = ACPI_ADD_PTR(struct acpi_aest_node, aest, sizeof(struct acpi_aest_hdr));

	/* We aren't given the number of nodes, so we gotta figure them out on our own.. */
	tbl_sz = sizeof(aest->header);

	while (tbl_sz > 0) {
		/* Add the length of the current node to the total size */
		tbl_sz += aest_node->hdr.length;

		/* Jump to the next node */
		aest_node += aest_node->hdr.length;
		node_count++;

		if (tbl_sz != aest->header.length)
			pr_err("Faulty table! Header and nodes lengths don't sum up!\n");
	}

	pr_debug("Found %d AEST nodes!\n", node_count);

	/* Get the first AEST node, again */
	aest_node = ACPI_ADD_PTR(struct acpi_aest_node, aest, sizeof(struct acpi_aest_hdr));

	/* Get the end of the AEST table */
	aest_end = ACPI_ADD_PTR(struct acpi_aest_node, aest, aest_table->length);

	for (i = 0; i < node_count; i++) {
		if (aest_node + aest_node->hdr.length > aest_end) {
			pr_err("AEST node pointer overflows, bad table\n");
			return;
		}

		fwnode = acpi_alloc_fwnode_static();
		if (!fwnode)
			return;

		aest_set_fwnode(aest_node, fwnode);

		ret = aest_add_platform_device(aest_node);
		if (ret) {
			aest_delete_fwnode(aest_node);
			acpi_free_fwnode_static(fwnode);
			return;
		}

		aest_node = ACPI_ADD_PTR(struct acpi_aest_node, aest_node, aest_node->hdr.length);
	}
}

/**
 * acpi_aest_init() - Initialize the AEST driver
 */
void __init acpi_aest_init(void)
{
	acpi_status status;

	status = acpi_get_table(ACPI_SIG_AEST, 0, &aest_table);
	if (ACPI_FAILURE(status)) {
		if (status != AE_NOT_FOUND) {
			const char *msg = acpi_format_exception(status);
			pr_err("Failed to get table, %s\n", msg);
		}

		return;
	}

	aest_init_platform_devices();
	acpi_put_table(aest_table);
}
