//license
// copyright

#ifndef __DEVFREQ_GOV_ADRENO_TZ_H__
#define __DEVFREQ_GOV_ADRENO_TZ_H__

#define DEVFREQ_GOV_ADRENO_TZ "adreno_tz"

struct devfreq_msm_adreno_tz_data {
	/* Proprerties sent to TZ */
	u32 ctxt_aware_target_pwrlevel;
	u32 ctxt_aware_busy_penalty;

	/* Other variables */
	s64 total_time;
	s64 busy_time;
	bool ctxt_aware_enable;
	bool is_64;
};

int governor_adreno_tz_init(struct device *dev);

#endif
