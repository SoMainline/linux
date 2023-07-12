// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2023, Linaro Limited

#include <linux/input-event-codes.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/soundwire/sdw.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <sound/soc.h>

#include "qdsp6/q6afe.h"
#include "common.h"
#include "sdw.h"

#define MI2S_BCLK_RATE		(48000ULL * 2 * 16)
#define CODEC_FMT		SND_SOC_DAIFMT_CBS_CFS
#define CODEC_DAI_FMT		(SND_SOC_DAIFMT_CBS_CFS |\
				 SND_SOC_DAIFMT_NB_NF |\
				 SND_SOC_DAIFMT_I2S)
struct sm6115_snd_data {
	bool stream_prepared[AFE_PORT_MAX];
	struct snd_soc_card *card;
	struct sdw_stream_runtime *sruntime[AFE_PORT_MAX];
	struct snd_soc_jack jack;
	bool jack_setup;
};

static int sm6115_init(struct snd_soc_pcm_runtime *rtd)
{
	struct sm6115_snd_data *data = snd_soc_card_get_drvdata(rtd->card);

	return qcom_snd_wcd_jack_setup(rtd, &data->jack, &data->jack_setup);
}

static int sm6115_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	struct snd_interval *channels = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_interval *rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	rate->min = rate->max = 48000;

	switch (cpu_dai->id) {
	case VA_CODEC_DMA_TX_0:
	case VA_CODEC_DMA_TX_1:
	case VA_CODEC_DMA_TX_2:
		channels->max = 8;
		break;
	default:
		channels->max = 2;
		break;
	}

	channels->min = channels->max;
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);

	return 0;
}

static int sm6115_snd_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sm6115_snd_data *pdata = snd_soc_card_get_drvdata(rtd->card);

	return qcom_snd_sdw_hw_params(substream, params, &pdata->sruntime[cpu_dai->id]);
}

static int sm6115_snd_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sm6115_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct sdw_stream_runtime *sruntime = data->sruntime[cpu_dai->id];

	return qcom_snd_sdw_prepare(substream, sruntime,
				    &data->stream_prepared[cpu_dai->id]);
}

static int sm6115_snd_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct sm6115_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sdw_stream_runtime *sruntime = data->sruntime[cpu_dai->id];

	return qcom_snd_sdw_hw_free(substream, sruntime,
				    &data->stream_prepared[cpu_dai->id]);
}

static int sm6115_snd_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int clk_id;

	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX:
		clk_id = Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT;
		break;
	case SECONDARY_MI2S_RX:
		clk_id = Q6AFE_LPASS_CLK_ID_SEC_MI2S_IBIT;
		break;
	case TERTIARY_MI2S_RX:
		clk_id = Q6AFE_LPASS_CLK_ID_TER_MI2S_IBIT;
		break;
	case QUATERNARY_MI2S_RX:
		clk_id = Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT;
		break;
	default:
		return 0;
	}

	snd_soc_dai_set_sysclk(cpu_dai, clk_id, MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
	snd_soc_dai_set_fmt(cpu_dai, CODEC_FMT);
	snd_soc_dai_set_fmt(codec_dai, CODEC_DAI_FMT);

	return 0;
}

static void sm6115_snd_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int clk_id;

	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX:
		clk_id = Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT;
		break;
	case SECONDARY_MI2S_RX:
		clk_id = Q6AFE_LPASS_CLK_ID_SEC_MI2S_IBIT;
		break;
	case TERTIARY_MI2S_RX:
		clk_id = Q6AFE_LPASS_CLK_ID_TER_MI2S_IBIT;
		break;
	case QUATERNARY_MI2S_RX:
		clk_id = Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT;
		break;
	default:
		return;
	}

	snd_soc_dai_set_sysclk(cpu_dai, clk_id, 0, SNDRV_PCM_STREAM_PLAYBACK);
	snd_soc_dai_set_fmt(cpu_dai, CODEC_FMT);
	snd_soc_dai_set_fmt(codec_dai, CODEC_DAI_FMT);
}

static const struct snd_soc_ops sm6115_be_ops = {
	.hw_free = sm6115_snd_hw_free,
	.hw_params = sm6115_snd_hw_params,
	.prepare = sm6115_snd_prepare,
	.shutdown = sm6115_snd_shutdown,
	.startup = sm6115_snd_startup,
};

static int sm6115_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_dai_link *link;
	struct sm6115_snd_data *data;
	struct snd_soc_card *card;
	int i, ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card->dev = dev;
	dev_set_drvdata(dev, card);
	snd_soc_card_set_drvdata(card, data);
	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	card->owner = THIS_MODULE;
	card->driver_name = "sm6115";

	for_each_card_prelinks(card, i, link) {
		link->init = sm6115_init;
		link->ops = &sm6115_be_ops;
		if (link->no_pcm == 1)
			link->be_hw_params_fixup = sm6115_be_hw_params_fixup;
	}

	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id snd_sm6115_dt_match[] = {
	{ .compatible = "qcom,sm6115-sndcard" },
	{ }
};

MODULE_DEVICE_TABLE(of, snd_sm6115_dt_match);

static struct platform_driver snd_sm6115_driver = {
	.probe = sm6115_platform_probe,
	.driver = {
		.name = "snd-sm6115",
		.of_match_table = snd_sm6115_dt_match,
		.pm = &snd_soc_pm_ops,
	},
};
module_platform_driver(snd_sm6115_driver);
MODULE_DESCRIPTION("SM6115 ASoC Machine Driver");
MODULE_LICENSE("GPL");
