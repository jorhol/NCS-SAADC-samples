/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <hal/nrf_gpio.h>
#include <nrfx_saadc.h>
#include <nrfx_rtc.h>
#include <helpers/nrfx_gppi.h>
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif


#include <logging/log.h>
LOG_MODULE_REGISTER(nrfx_saadc_lowpower_sample, LOG_LEVEL_INF);

#define LED0_PIN	NRF_GPIO_PIN_MAP(DT_PROP(DT_GPIO_CTLR(DT_ALIAS(led0), gpios), port), DT_GPIO_PIN(DT_ALIAS(led0), gpios)) 
 
#define SAADC_CHANNEL_COUNT   1
#define SAADC_SAMPLE_INTERVAL_MS 250u

static nrf_saadc_value_t samples[SAADC_CHANNEL_COUNT];
static const nrf_saadc_input_t ANALOG_INPUT_MAP[SAADC_CHANNEL_COUNT] = {NRF_SAADC_INPUT_AIN0};

#if defined(NRF5340_XXAA)
const nrfx_rtc_t rtc = NRFX_RTC_INSTANCE(1); /**< Declaring an instance of nrf_drv_rtc for RTC0. */
#elif defined(NRF52832_XXAA) ||                                    \
	  defined(NRF52833_XXAA) ||                                 \
      defined(NRF52840_XXAA)
const nrfx_rtc_t rtc = NRFX_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC0. */
#endif


static void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    nrfx_err_t err;
    switch (p_event->type)
    {
        case NRFX_SAADC_EVT_DONE:
            LOG_INF("ADC Values: %6d", p_event->data.done.p_buffer[0]);

            // Set up the next available buffer
            err = nrfx_saadc_buffer_set(&samples[0], SAADC_CHANNEL_COUNT);
            if (err != NRFX_SUCCESS) {
                LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
                return;
            }
            nrf_gpio_pin_toggle(LED0_PIN);

            
            break;
        case NRFX_SAADC_EVT_READY:
            LOG_INF("Stopping SAADC");
            // STOP SAADC, START task will be triggered from PPI. 
            // This should only happen after first configuration of SAADC
            nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_STOP);

        default:
            LOG_INF("SAADC evt %d", p_event->type);
            break;
    }
}



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
    static uint8_t m_rtc_clear_saadc_start_ppi_channel;
    static uint8_t m_saadc_sample_ppi_channel;

    // Trigger task sample from timer
    nrfx_err_t err = nrfx_gppi_channel_alloc(&m_rtc_clear_saadc_start_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_gppi_channel_alloc error: %08x", err);
        return;
    }
    err = nrfx_gppi_channel_alloc(&m_saadc_sample_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_gppi_channel_alloc error: %08x", err);
        return;
    }
    
    nrfx_gppi_channel_endpoints_setup(m_rtc_clear_saadc_start_ppi_channel, 
                                       nrfx_rtc_event_address_get(&rtc, NRF_RTC_EVENT_COMPARE_0),
                                       nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));
    nrfx_gppi_fork_endpoint_setup(m_rtc_clear_saadc_start_ppi_channel,
                                       nrfx_rtc_task_address_get(&rtc, NRF_RTC_TASK_CLEAR));

    nrfx_gppi_channel_endpoints_setup(m_saadc_sample_ppi_channel, 
                                       nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_STARTED),
                                       nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));


    nrfx_gppi_channels_enable(BIT(m_rtc_clear_saadc_start_ppi_channel));
    nrfx_gppi_channels_enable(BIT(m_saadc_sample_ppi_channel));
}

static void adc_configure(void)
{
    nrfx_err_t err;

    /* Connect ADC_0 IRQ to nrfx_saadc_irq_handler */
    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
    DT_IRQ(DT_NODELABEL(adc), priority),
    nrfx_isr, nrfx_saadc_irq_handler, 0);

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
        config.channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;          
        config.channel_config.gain = NRF_SAADC_GAIN1_6;

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
                                            NRF_SAADC_RESOLUTION_12BIT,
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
}

 
void main(void)
{
	LOG_INF("nrfx_saadc_advanced_lowpower_rtc_ppi sample on %s", CONFIG_BOARD);

    nrf_gpio_cfg_output(LED0_PIN);

    adc_configure();
    ppi_configure();
    rtc_configure();

}