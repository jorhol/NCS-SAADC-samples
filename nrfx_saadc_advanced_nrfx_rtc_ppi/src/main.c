/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include <nrfx_saadc.h>
#include <nrfx_rtc.h>
#include <nrfx_ppi.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(nrfx_saadc_lowpower_sample, LOG_LEVEL_INF);
 
#define SAADC_CHANNEL_COUNT   1
#define SAADC_SAMPLE_INTERVAL_MS 250u

static nrf_ppi_channel_t m_rtc_saadc_start_ppi_channel;
static nrf_ppi_channel_t m_rtc_saadc_sample_ppi_channel;

static volatile bool is_ready = true;
static nrf_saadc_value_t samples[SAADC_CHANNEL_COUNT];
static const nrf_saadc_input_t ANALOG_INPUT_MAP[SAADC_CHANNEL_COUNT] = {NRF_SAADC_INPUT_AIN0};

static void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    nrfx_err_t err;
    switch (p_event->type)
    {
        case NRFX_SAADC_EVT_FINISHED:
            LOG_INF("ADC Values: %6d",
                p_event->data.done.p_buffer[0]);
            break;

        case NRFX_SAADC_EVT_BUF_REQ:
            // Set up the next available buffer
            err = nrfx_saadc_buffer_set(&samples[0], SAADC_CHANNEL_COUNT);
            if (err != NRFX_SUCCESS) {
                LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
                return;
            }
            break;
                default:
            LOG_INF("SAADC evt %d", p_event->type);
            break;
    }
}

const nrfx_rtc_t rtc = NRFX_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_event_handler(nrfx_rtc_int_type_t int_type)
{
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_configure(void)
{
    nrfx_err_t err;

    //Initialize RTC instance
    nrfx_rtc_config_t config = NRFX_RTC_DEFAULT_CONFIG;
    err = nrfx_rtc_init(&rtc, &config, rtc_event_handler);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_rtc_init error: %08x", err);
        return;
    }

    uint32_t rtc_ticks = NRFX_RTC_US_TO_TICKS(SAADC_SAMPLE_INTERVAL_MS*1000, RTC_INPUT_FREQ);

    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
    err = nrfx_rtc_cc_set(&rtc,0,rtc_ticks,false);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_rtc_cc_set error: %08x", err);
        return;
    }

    //Power on RTC instance
    nrfx_rtc_enable(&rtc);
}

static void ppi_configure(void)
{
    // Trigger task sample from timer
    nrfx_err_t err = nrfx_ppi_channel_alloc(&m_rtc_saadc_start_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_ppi_channel_alloc error: %08x", err);
        return;
    }
    err = nrfx_ppi_channel_alloc(&m_rtc_saadc_sample_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_ppi_channel_alloc error: %08x", err);
        return;
    }

    err = nrfx_ppi_channel_assign(m_rtc_saadc_start_ppi_channel, 
                                       nrfx_rtc_event_address_get(&rtc, NRF_RTC_EVENT_COMPARE_0),
                                       nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_ppi_channel_assign error: %08x", err);
        return;
    }

    err = nrfx_ppi_channel_assign(m_rtc_saadc_sample_ppi_channel, 
                                       nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_STARTED),
                                       nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));
    

    err = nrfx_ppi_channel_enable(m_rtc_saadc_start_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_ppi_channel_enable error: %08x", err);
        return;
    }
    err = nrfx_ppi_channel_enable(m_rtc_saadc_sample_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_ppi_channel_enable error: %08x", err);
        return;
    }
}

static void adc_configure(void)
{
    nrfx_err_t err;

    nrfx_saadc_adv_config_t saadc_adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    saadc_adv_config.internal_timer_cc = 0;
    saadc_adv_config.start_on_end = false;

    err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_init error: %08x", err);
        return;
    }

    static nrfx_saadc_channel_t channel_configs[SAADC_CHANNEL_COUNT];

    uint8_t channel_mask = 0;
    for(int i = 0; i < SAADC_CHANNEL_COUNT; i++) {
        nrf_saadc_input_t pin = ANALOG_INPUT_MAP[i];
        // Apply default config to each channel
        nrfx_saadc_channel_t config = NRFX_SAADC_DEFAULT_CHANNEL_SE(pin, i);

        // Replace some parameters in default config
        config.channel_config.reference = NRF_SAADC_REFERENCE_VDD4;          
        config.channel_config.gain = NRF_SAADC_GAIN1_4;

        // Copy to list of channel configs
        memcpy(&channel_configs[i], &config, sizeof(config));

        // Update channel mask
        channel_mask |= 1 << i;
    }

    err = nrfx_saadc_channels_config(channel_configs, SAADC_CHANNEL_COUNT);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_channels_config error: %08x", err);
        return;
    }

    err = nrfx_saadc_advanced_mode_set(channel_mask,
                                            NRF_SAADC_RESOLUTION_14BIT,
                                            &saadc_adv_config,
                                            saadc_event_handler);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_advanced_mode_set error: %08x", err);
        return;
    }
                                            
    // Configure two buffers to ensure double buffering of samples, to avoid data loss when the sampling frequency is high
    err = nrfx_saadc_buffer_set(&samples[0], SAADC_CHANNEL_COUNT);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
        return;
    }

    err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_mode_trigger error: %08x", err);
        return;
    }

    // STOP SAADC, START task will be triggered from PPI
    nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_STOP);
}

 
void main(void)
{
	LOG_INF("nrfx_saadc_advanced_lowpower_rtc_ppi sample on %s", CONFIG_BOARD);

	/* Connect ADC_0 IRQ to nrfx_saadc_irq_handler */
	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
		    DT_IRQ(DT_NODELABEL(adc), priority),
		    nrfx_isr, nrfx_saadc_irq_handler, 0);

    adc_configure();
    ppi_configure();
    rtc_configure();


}