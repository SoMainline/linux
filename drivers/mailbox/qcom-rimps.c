// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Linaro Limited
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define RIMPS_SEND_IRQ_OFFSET		0xC
 #define RIMPS_SEND_IRQ_VAL		BIT(28)

#define RIMPS_CLEAR_IRQ_OFFSET		0x308
 #define RIMPS_CLEAR_IRQ_VAL		BIT(3)

#define RIMPS_STATUS_IRQ_OFFSET		0x30C
 #define RIMPS_STATUS_IRQ_VAL		BIT(3)

#define RIMPS_CLOCK_DOMAIN_OFFSET	0x1000

//todo: remove log or move this to bindings
enum {
	RIMPS_IPC_CHAN_SCMI,
	RIMPS_IPC_CHAN_LOG,
	RIMPS_IPC_CHAN_NUM,
};

/**
 * struct rimps_mbox - RIMPS driver data
 * @dev: Device associated with this instance
 * @tx_base: Base of the TX region
 * @rx_base: Base of the RX region
 * @mbox: The mailbox controller
 * @chans: The mailbox channels array
 * @irq: RIMPS-to-HLOS irq
 */
struct qcom_rimps_mbox {
	struct device *dev;
	void __iomem *tx_base;
	void __iomem *rx_base;
	struct mbox_controller mbox;
	struct mbox_chan chans[RIMPS_IPC_CHAN_NUM];
	int irq;
};

static irqreturn_t qcom_rimps_mbox_rx_interrupt(int irq, void *data)
{
	struct qcom_rimps_mbox *rimps_mbox = data;
	void __iomem *base = rimps_mbox->rx_base;
	unsigned long flags;
	u32 cd_off, val;
	int i;

	for (i = 0; i < RIMPS_IPC_CHAN_NUM; i++) {
		cd_off = (i * RIMPS_CLOCK_DOMAIN_OFFSET);

		val = readl(base + cd_off + RIMPS_STATUS_IRQ_OFFSET);
		if (val & RIMPS_STATUS_IRQ_VAL) {
			writel(RIMPS_CLEAR_IRQ_VAL, base + cd_off + RIMPS_CLEAR_IRQ_OFFSET);
			/* Make sure register write is complete before proceeding */
			mb();

			spin_lock_irqsave(&rimps_mbox->chans[i].lock, flags);

			if (rimps_mbox->chans[i].con_priv)
				mbox_chan_received_data(&rimps_mbox->chans[i], NULL);

			spin_unlock_irqrestore(&rimps_mbox->chans[i].lock, flags);
		}
	}

	return IRQ_HANDLED;
}

static void qcom_rimps_mbox_shutdown(struct mbox_chan *chan)
{
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);
	chan->con_priv = NULL;
	spin_unlock_irqrestore(&chan->lock, flags);
}

static int qcom_rimps_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct qcom_rimps_mbox *rimps_mbox = container_of(chan->mbox, struct qcom_rimps_mbox, mbox);

	writel(RIMPS_SEND_IRQ_VAL, rimps_mbox->tx_base + RIMPS_SEND_IRQ_OFFSET);

	return 0;
}

static struct mbox_chan *qcom_rimps_mbox_xlate(struct mbox_controller *mbox,
					       const struct of_phandle_args *sp)
{
	struct qcom_rimps_mbox *rimps_mbox = container_of(mbox, struct qcom_rimps_mbox, mbox);
	unsigned long idx = sp->args[0];

	if (sp->args_count != 1)
		return ERR_PTR(-EINVAL);

	if (idx >= RIMPS_IPC_CHAN_NUM)
		return ERR_PTR(-EINVAL);

	if (mbox->chans[idx].con_priv)
		return ERR_PTR(-EBUSY);

	mbox->chans[idx].con_priv = rimps_mbox;

	return &mbox->chans[idx];
}

static const struct mbox_chan_ops rimps_mbox_chan_ops = {
	.send_data = qcom_rimps_mbox_send_data,
	.shutdown = qcom_rimps_mbox_shutdown,
};

static int qcom_rimps_mbox_probe(struct platform_device *pdev)
{
	struct qcom_rimps_mbox *rimps_mbox;
	struct device *dev = &pdev->dev;
	struct mbox_controller *mbox;
	int i, ret;

	rimps_mbox = devm_kzalloc(dev, sizeof(*rimps_mbox), GFP_KERNEL);
	if (!rimps_mbox)
		return dev_err_probe(dev, -ENOMEM, "Failed to allocate rimps_mbox\n");

	rimps_mbox->dev = dev;

	rimps_mbox->tx_base = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (!rimps_mbox->tx_base)
		return dev_err_probe(dev, -ENOMEM, "Failed to ioremap the TX base\n");

	rimps_mbox->rx_base = devm_platform_get_and_ioremap_resource(pdev, 1, NULL);
	if (!rimps_mbox->rx_base)
		return dev_err_probe(dev, -ENOMEM, "Failed to ioremap the RX base\n");

	rimps_mbox->irq = platform_get_irq(pdev, 0);
	if (rimps_mbox->irq < 0)
		return dev_err_probe(dev, rimps_mbox->irq, "Failed to get the IRQ\n");

	/* Initialize channel identifiers */
	for (i = 0; i < RIMPS_IPC_CHAN_NUM; i++)
		rimps_mbox->chans[i].con_priv = NULL;

	mbox = &rimps_mbox->mbox;
	mbox->dev = dev;
	mbox->num_chans = RIMPS_IPC_CHAN_NUM;
	mbox->chans = rimps_mbox->chans;
	mbox->ops = &rimps_mbox_chan_ops;
	mbox->of_xlate = qcom_rimps_mbox_xlate;
	mbox->txdone_irq = false;
	mbox->txdone_poll = false;

	ret = devm_mbox_controller_register(dev, mbox);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register RIMPS mailbox\n");

	ret = devm_request_irq(&pdev->dev, rimps_mbox->irq,
			       qcom_rimps_mbox_rx_interrupt,
			       IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND,
			       "qcom_rimps_mbox", rimps_mbox);
	if (ret < 0)
		return dev_err_probe(dev, -EINVAL, "Failed to register irq %d\n", ret);

	platform_set_drvdata(pdev, rimps_mbox);

	return 0;
}

static const struct of_device_id qcom_rimps_mbox_of_match[] = {
	{ .compatible = "qcom,rimps-mbox" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_rimps_mbox_of_match);

static struct platform_driver qcom_rimps_mbox_driver = {
	.probe = qcom_rimps_mbox_probe,
	.driver = {
		.name = "qcom_rimps_mbox",
		.of_match_table = qcom_rimps_mbox_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(qcom_rimps_mbox_driver);

MODULE_DESCRIPTION("Qualcomm RIMPS mailbox driver");
MODULE_LICENSE("GPL");
