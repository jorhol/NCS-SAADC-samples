/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include <nrfx_saadc.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(nrfx_saadc_lowpower_sample, LOG_LEVEL_INF);
 
#define SAADC_CHANNEL_COUNT   1
#define SAADC_SAMPLE_INTERVAL_MS 250

static volatile bool is_ready = true;
static nrf_saadc_value_t samples[SAADC_CHANNEL_COUNT];
static nrfx_saadc_channel_t channels[SAADC_CHANNEL_COUNT] = {NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0, 0)};

void sample_timer_handler(struct k_timer *dummy);

K_TIMER_DEFINE(my_timer, sample_timer_handler, NULL);

static void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        for(int i = 0; i < p_event->data.done.size; i++)
        {
            LOG_INF("CH%d: %d", i, p_event->data.done.p_buffer[i]);
        }

        is_ready = true;
    }
}

/**@brief Timeout handler for the repeated timer.
 */
void sample_timer_handler(struct k_timer *dummy)
{
    if(is_ready)
    {
        nrfx_err_t err;

        err = nrfx_saadc_simple_mode_set((1<<0),
                                              NRF_SAADC_RESOLUTION_12BIT,
                                              NRF_SAADC_OVERSAMPLE_DISABLED,
                                              saadc_event_handler);
        if (err != NRFX_SUCCESS) {
			LOG_ERR("nrfx_saadc_simple_mode_set error: %08x", err);
			return;
		}
        
        err = nrfx_saadc_buffer_set(samples, SAADC_CHANNEL_COUNT);
        if (err != NRFX_SUCCESS) {
			LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
			return;
		}

        err = nrfx_saadc_mode_trigger();
        if (err != NRFX_SUCCESS) {
			LOG_ERR("nrfx_saadc_mode_trigger error: %08x", err);
			return;
		}

        is_ready = false;
	}
}


 
void main(void)
{
	LOG_INF("nrfx_saadc_lowpower sample on %s", CONFIG_BOARD);

	nrfx_err_t err;

	/* Connect ADC_0 IRQ to nrfx_saadc_irq_handler */
	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
		    DT_IRQ(DT_NODELABEL(adc), priority),
		    nrfx_isr, nrfx_saadc_irq_handler, 0);

	err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_saadc_mode_trigger error: %08x", err);
		return;
	}
 
    err = nrfx_saadc_channels_config(channels, SAADC_CHANNEL_COUNT);
    if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_saadc_channels_config error: %08x", err);
		return;
	}

    /* start periodic timer that expires once every second */
	k_timer_start(&my_timer, K_MSEC(SAADC_SAMPLE_INTERVAL_MS), K_MSEC(SAADC_SAMPLE_INTERVAL_MS));

}