/*
 * Copyright (c) 2022 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #define DT_DRV_COMPAT nrf_w1_serial
 /**
 * @brief 1-Wire Bus Master driver using Zephyr serial interface.
 *
 * This driver implements the 1-Wire interface using an uart.
 * The driver uses a uart peripheral with a baudrate of 115.2 kBd to send
 * and receive data bits and a baurade of 9.6 kBd for slave reset and
 * presence detection as suggested for normal speed operating mode in:
 * https://www.analog.com/en/resources/technical-articles/using-a-uart-to-implement-a-1wire-bus-master.html
 * For overdrive speed communication baudrates of 1 MBd and 115.2 kBd
 * are used, respectively.
 */

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/drivers/w1.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>

#include <haly/nrfy_uarte.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <hal/nrf_gpio.h>

// #define NRFX_LOG_MODULE UARTE
// #include <nrfx_log.h>

// Size of the RX HW FIFO
#define UARTE_HW_RX_FIFO_SIZE 5

#define UARTE_FLAG_TX_CNT      12 // Max amount of TX flags
#define UARTE_FLAG_TX_OFFSET   0
#define UARTE_FLAG_RX_OFFSET   UARTE_FLAG_TX_CNT

#define UARTE_FLAG(type, i) \
    NRFX_BIT(NRFX_CONCAT(UARTE_FLAG_,type,_OFFSET) + i)
// Flag is set when receiver is active.
#define UARTE_FLAG_RX_ENABLED              UARTE_FLAG(RX, 0)
#define UARTE_FLAG_RX_CNT      12 // Max amount of RX flags
// Flag set when there is a PPI connection set up between ENDTX event and TXSTOP task.
#define UARTE_FLAG_TX_STOP_ON_END          UARTE_FLAG(TX, 0)
// Flag indicates that TX abort is in progress.
#define UARTE_FLAG_TX_ABORTED              UARTE_FLAG(TX, 3)
// Flag is set if receiver is enabled with flag NRFX_UARTE_RX_ENABLE_STOP_ON_END.
#define UARTE_FLAG_RX_STOP_ON_END          UARTE_FLAG(RX, 1)
// Flag is set if receiver is enabled with flag NRFX_UARTE_RX_ENABLE_CONT.
#define UARTE_FLAG_RX_CONT                 UARTE_FLAG(RX, 2)
// Flag is set if receiver is enabled with flag NRFX_UARTE_RX_ENABLE_KEEP_FIFO_CONTENT.
#define UARTE_FLAG_RX_KEEP_FIFO_CONTENT    UARTE_FLAG(RX, 3)
// Flag indicates that RX abort was done to seamlessly switch the DMA buffer and not to abort the
// transfer.
#define UARTE_FLAG_RX_RESTARTED            UARTE_FLAG(RX, 4)
// Flag indicates that RX was aborted.
#define UARTE_FLAG_RX_ABORTED              UARTE_FLAG(RX, 6)

// There is a HW bug which results in RX amount value not being updated when FIFO was empty.
// It is then hard to determine if FIFO contained anything or not.
#define USE_WORKAROUND_FOR_FLUSHRX_ANOMALY 1

// Mask with all RX flags
#define UARTE_RX_FLAGS (NRFX_BIT_MASK(UARTE_FLAG_RX_CNT) << UARTE_FLAG_RX_OFFSET)

#define PIN1_24 NRF_GPIO_PIN_MAP(1, 24)

LOG_MODULE_REGISTER(nrf_w1_serial, CONFIG_W1_LOG_LEVEL);

#define W1_SERIAL_READ_REQ_BYTE 0xFF
#define W1_SERIAL_BIT_1         0xFF
#define W1_SERIAL_BIT_0         0x00

#define W1_SERIAL_STD_RESET_BYTE 0xF0

/*
 * Overdrive speed signal parameters:
 * Unlike standard speed, RST and DATA timing defined in Devicetree
 */
#define W1_SERIAL_OD_RESET_BYTE 0xE0

typedef enum {
    NRFX_UARTE_EVT_TX_DONE,
    NRFX_UARTE_EVT_RX_DONE,
    NRFX_UARTE_EVT_ERROR,
    NRFX_UARTE_EVT_RX_BUF_REQUEST,
    NRFX_UARTE_EVT_RX_DISABLED,
    NRFX_UARTE_EVT_RX_BUF_TOO_LATE,
    NRFX_UARTE_EVT_RX_BYTE,
} nrfx_uarte_event_type_t;

typedef struct {
    uint8_t *p_buffer;
    size_t   length;
} nrfx_uarte_rx_evt_t;

typedef struct {
    const uint8_t *p_buffer;
    size_t         length;
    uint32_t       flags;
} nrfx_uarte_tx_evt_t;

typedef struct {
    uint32_t error_mask;
} nrfx_uarte_error_evt_t;

typedef struct {
    size_t flush_cnt;
} nrfx_uarte_rx_disabled_evt_t;

typedef struct {
    nrfx_uarte_event_type_t type;
    union {
        nrfx_uarte_rx_evt_t          rx;
        nrfx_uarte_tx_evt_t          tx;
        nrfx_uarte_error_evt_t       error;
        nrfx_uarte_rx_disabled_evt_t rx_disabled;
    } data;
} nrfx_uarte_event_t;

typedef void (*nrfx_uarte_event_handler_t)(nrfx_uarte_event_t const *p_event, void *p_context);

typedef struct {
    nrfy_uarte_buffer_t curr;
    nrfy_uarte_buffer_t next;
    nrfy_uarte_buffer_t flush;
    size_t               off;
} nrfx_uarte_rx_data_t;

typedef struct {
    nrfy_uarte_buffer_t curr;
    nrfy_uarte_buffer_t next;
    size_t               off;
    int                  amount;
} nrfx_uarte_tx_data_t;

typedef struct {
    void                       *p_context;
    nrfx_uarte_event_handler_t  handler;
    nrfx_uarte_rx_data_t        rx;
    nrfx_uarte_tx_data_t        tx;
    nrfx_drv_state_t            state;
    nrfx_atomic_t               flags;
} nrfx_uarte_control_block_t;

typedef struct {
    NRF_UARTE_Type             *p_reg;
    nrfx_uarte_control_block_t  cb;
} nrfx_uarte_t;

#define NRFX_UARTE_INSTANCE(reg) \
{ \
    .p_reg = (NRF_UARTE_Type *)reg, \
    .cb    = {0}, \
}

typedef struct {
    uint32_t             txd_pin;
    uint32_t             rxd_pin;
    uint32_t             rts_pin;
    uint32_t             cts_pin;
    void                *p_context;
    nrf_uarte_baudrate_t baudrate;
    nrf_uarte_config_t   config;
    bool                 skip_psel_cfg;
    bool                 skip_gpio_cfg;
    bool                 tx_stop_on_end;
    uint8_t              interrupt_priority;
} nrfx_uarte_config_t;

#define NRFX_UARTE_DEFAULT_CONFIG(_pin_tx, _pin_rx)                    \
{                                                                       \
    .txd_pin            = _pin_tx,                                     \
    .rxd_pin            = _pin_rx,                                     \
    .rts_pin            = NRF_UARTE_PSEL_DISCONNECTED,                 \
    .cts_pin            = NRF_UARTE_PSEL_DISCONNECTED,                 \
    .p_context          = NULL,                                       \
    .baudrate           = NRF_UARTE_BAUDRATE_115200,                   \
    .config             =                                              \
    {                                                                   \
        .hwfc           = NRF_UARTE_HWFC_DISABLED,                     \
        .parity         = NRF_UARTE_PARITY_EXCLUDED,                   \
        NRFX_COND_CODE_1(NRF_UARTE_HAS_STOP_BITS,                       \
                (.stop = (nrf_uarte_stop_t)NRF_UARTE_STOP_ONE,), ())   \
        NRFX_COND_CODE_1(NRF_UARTE_HAS_PARITY_BIT,                      \
                (.paritytype = NRF_UARTE_PARITYTYPE_EVEN,), ())         \
        NRFX_COND_CODE_1(NRF_UARTE_HAS_FRAME_SIZE,                      \
                (.frame_size = NRF_UARTE_FRAME_SIZE_8_BIT,              \
                 .endian = NRF_UARTE_ENDIAN_MSB,), ())                  \
    },                                                                  \
    .skip_psel_cfg      = false,                                       \
    .skip_gpio_cfg      = false,                                       \
    .tx_stop_on_end     = false,                                       \
    .interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY        \
}

#define NRFX_UARTE_TX_DONE_ABORTED NRFX_BIT(0)

static const uint32_t rx_int_mask = NRF_UARTE_INT_ERROR_MASK |
                                    NRF_UARTE_INT_ENDRX_MASK |
                                    NRF_UARTE_INT_RXTO_MASK |
                                    NRF_UARTE_INT_RXSTARTED_MASK;

struct w1_serial_config {
	/** w1 master config, common to all drivers */
	struct w1_master_config master_config;
	/** Baud rate for overdrive speed data communication */
	uint32_t od_data_baud;
	/** Baud rate for overdrive speed reset/presence detection */
	uint32_t od_reset_baud;
    /** Pin control configuration */
    const struct pinctrl_dev_config *pcfg;
    /** IRQ configuration function */
    void (*irq_config_func)(void);
};

struct w1_serial_data {
	/** w1 master data, common to all drivers */
	struct w1_master_data master_data;
	bool overdrive_active;
    nrfx_uarte_t inst;
    struct k_sem rx_sem;
};

static void w1_uarte_event_handler(nrfx_uarte_event_t const *p_event, void *p_context)
{
    struct w1_serial_data *data = p_context;

    switch (p_event->type){
        // case NRFX_UARTE_EVT_TX_DONE:
        //     break;

        case NRFX_UARTE_EVT_RX_DONE:
            k_sem_give(&data->rx_sem);
            break;

        case NRFX_UARTE_EVT_RX_BUF_REQUEST:
            break;

        case NRFX_UARTE_EVT_ERROR:
            k_sem_give(&data->rx_sem);
            break;

        default:
            break;
    }
}

static bool is_rx_active(nrfx_uarte_control_block_t * p_cb)
{
    return (p_cb->flags & UARTE_FLAG_RX_ENABLED) ? true : false;
}

static void release_rx(nrfx_uarte_control_block_t * p_cb)
{
    /* Clear all RX flags. */
    NRFX_ATOMIC_FETCH_AND(&p_cb->flags, ~UARTE_RX_FLAGS);
}

static int w1_uarte_rx_enable(nrfx_uarte_t *p_instance)
{
    NRFX_ASSERT(p_instance);

    nrfx_uarte_control_block_t * p_cb = &p_instance->cb;
    NRF_UARTE_Type * p_uarte = p_instance->p_reg;
    uint32_t prev_flags;

    prev_flags = NRFX_ATOMIC_FETCH_OR(&p_cb->flags, UARTE_FLAG_RX_ENABLED);
    if (prev_flags & UARTE_FLAG_RX_ENABLED)
    {
        return -EBUSY;
    }

    nrfx_uarte_event_t event = {
        .type = NRFX_UARTE_EVT_RX_BUF_REQUEST
    };

    nrfy_uarte_int_disable(p_uarte, rx_int_mask);
    nrfy_uarte_enable(p_uarte);
    nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDRX);
    nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXSTARTED);
    nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXTO);

    if ((p_cb->rx.curr.p_buffer == NULL) && p_cb->handler)
    {
        w1_uarte_event_handler(&event, p_cb->p_context);
    }
    // Expecting to get buffer set as a response to the request.
    if (p_cb->rx.curr.p_buffer == NULL)
    {
        // If RX is not active then it means that reception is already stopped.
        // It may happen if RX flush had enough data to fill the user buffer.
        // In that case reception is disabled from the context of RX buffer set function.
        if (!is_rx_active(p_cb))
        {
            return 0;
        }

        release_rx(p_cb);
        return -ENOMEM;
    }
    // Expecting to get buffer set as a response to the request.
    if ((p_cb->rx.curr.p_buffer == NULL) && p_cb->handler)
    {
        w1_uarte_event_handler(&event, p_cb->p_context);
    }

    /* Check if instance is still enabled. It might get disabled at some point. */
    if (p_cb->flags & UARTE_FLAG_RX_ENABLED)
    {
        if (!nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXSTARTED))
        {
            /* Manually trigger RX if it was not yet started. */
            nrfy_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STARTRX);
        }

        if (p_cb->handler)
        {
            nrfy_uarte_int_enable(p_uarte, rx_int_mask);
        }
    }

    return 0;
}

static bool w1_is_tx_ready(NRF_UARTE_Type * p_uarte, bool stop_on_end)
{
    return nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_TXSTOPPED) ||
        (!stop_on_end && nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDTX));
}
#if !defined(CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
static void w1_tx_start(NRF_UARTE_Type * p_uarte, const uint8_t *buf, size_t len, bool en_int)
{
    nrfy_uarte_tx_buffer_set(p_uarte, buf, len);
    nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDTX);
    nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_TXSTOPPED);
    nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_TXSTARTED);

    nrfy_uarte_enable(p_uarte);

    if (en_int)
    {
        nrfy_uarte_int_enable(p_uarte, NRF_UARTE_INT_TXSTOPPED_MASK);
    }

    nrfy_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STARTTX);
}

static void w1_tx_prepare_start(NRF_UARTE_Type *p_uarte, nrfx_uarte_control_block_t *p_cb)
{
    NRFX_ASSERT(w1_is_tx_ready(p_uarte, p_cb->flags & UARTE_FLAG_TX_STOP_ON_END));

    p_cb->tx.amount = -1;
    w1_tx_start(p_uarte, p_cb->tx.curr.p_buffer, p_cb->tx.curr.length, true);
}
#endif
#if defined(CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
static int w1_blocking_tx(NRF_UARTE_Type *p_uarte, uint8_t const *p_buffer,
				 size_t length, k_timepoint_t end)
{
    nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDTX);
    nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_TXSTOPPED);
    nrfy_uarte_tx_buffer_set(p_uarte, p_buffer, length);
    nrfy_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STARTTX);

    while (!nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDTX)) {
        if (sys_timepoint_expired(end)) {
            nrfy_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPTX);
            return -ETIMEDOUT;
        }
    }
    return 0;
}
#endif
/* Must be called with interrupts locked. */
static void disable_hw_from_tx(NRF_UARTE_Type *             p_uarte,
                               nrfx_uarte_control_block_t * p_cb)
{
    if (nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_TXSTOPPED) && !is_rx_active(p_cb))
    {
        nrfy_uarte_disable(p_uarte);
    }
}
#if !defined(CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
static int w1_uarte_tx(nrfx_uarte_t *p_instance, uint8_t const *p_data, size_t length)
{
    NRFX_ASSERT(p_instance);

    nrfx_uarte_control_block_t * p_cb = &p_instance->cb;
    NRF_UARTE_Type * p_uarte = p_instance->p_reg;

    NRFX_ASSERT((p_cb->state == NRFX_DRV_STATE_INITIALIZED) && (length > 0) && p_data);
    NRFX_ASSERT(nrf_dma_accessible_check(p_uarte, p_data));

    int err_code = 0;

    if (length == 0)
    {
        return -E2BIG;
    }

    NRFX_CRITICAL_SECTION_ENTER();
    if (p_cb->tx.curr.length == 0)
    {
        p_cb->tx.curr.length = length;

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif

        p_cb->tx.curr.p_buffer = (uint8_t *)p_data;

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

        w1_tx_prepare_start(p_uarte, p_cb);
    }
    else
    {
        err_code = -EBUSY;
    }
    NRFX_CRITICAL_SECTION_EXIT();

    return err_code;
}
#endif
static bool w1_prepare_rx(NRF_UARTE_Type * p_uarte)
{
    /**
     * Stop any currently running RX operations. This can occur when a
     * bootloader sets up the UART hardware and does not clean it up
     * before jumping to the next application.
     */
    if (nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXSTARTED))
    {
        bool res;

        nrfy_uarte_enable(p_uarte);
        nrfy_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPRX);

        NRFX_WAIT_FOR(nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXTO) ||
                      nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ERROR), 100, 10, res);

        nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXSTARTED);
        nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDRX);
        nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXTO);
        nrfy_uarte_disable(p_uarte);

        return res;
    }
    return true;
}

static bool w1_prepare_tx(NRF_UARTE_Type * p_uarte, bool stop_on_end)
{
    (void)stop_on_end;
    uint8_t dummy = 0;

    nrfy_uarte_enable(p_uarte);

    /* Set TXSTOPPED event by requesting fake (zero-length) transfer.
     * Pointer to RAM variable (data->tx_buffer) is set because otherwise
     * such operation may result in HardFault or RAM corruption.
     */
    nrfy_uarte_tx_buffer_set(p_uarte, &dummy, 0);
    nrfy_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STARTTX);

    /* switch off transmitter to save an energy */
    nrfy_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPTX);

    bool res;

    NRFX_WAIT_FOR(w1_is_tx_ready(p_uarte, true), 10, 1, res);

    if (!res)
    {
        return false;
    }

    nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDTX);
    nrfy_uarte_disable(p_uarte);
    return true;
}

static void apply_workaround_for_enable_anomaly(nrfx_uarte_t const * p_instance)
{
    volatile uint32_t const * rxenable_reg =
        (volatile uint32_t *)(((uint32_t)p_instance->p_reg) + 0x564);
    volatile uint32_t const * txenable_reg =
        (volatile uint32_t *)(((uint32_t)p_instance->p_reg) + 0x568);

    if (*txenable_reg == 1)
    {
        nrfy_uarte_task_trigger(p_instance->p_reg, NRF_UARTE_TASK_STOPTX);
    }

    if (*rxenable_reg == 1)
    {
        nrfy_uarte_enable(p_instance->p_reg);
        nrfy_uarte_task_trigger(p_instance->p_reg, NRF_UARTE_TASK_STOPRX);

        bool workaround_succeded;
        // The UARTE is able to receive up to four bytes after the STOPRX task has been triggered.
        // On lowest supported baud rate (1200 baud), with parity bit and two stop bits configured
        // (resulting in 12 bits per data byte sent), this may take up to 40 ms.
        NRFX_WAIT_FOR(*rxenable_reg == 0, 40000, 1, workaround_succeded);
        if (!workaround_succeded)
        {
            // NRFX_LOG_ERROR("Failed to apply workaround for instance with base address: %p.",
            //                (void *)p_instance->p_reg);
        }

        (void)nrfy_uarte_errorsrc_get_and_clear(p_instance->p_reg);
        nrfy_uarte_disable(p_instance->p_reg);
    }
}

static void uarte_configure(nrfx_uarte_t *              p_instance,
                            nrfx_uarte_config_t const * p_config)
{
    nrfy_uarte_config_t nrfy_config =
    {
        .skip_psel_cfg = NRFX_IS_ENABLED(NRFX_UARTE_CONFIG_SKIP_PSEL_CONFIG) ?
            true : p_config->skip_psel_cfg
    };
    nrfy_config.config = p_config->config;

#if defined(UARTE_NEEDS_BAUDRATE_FACTOR)
    uint32_t base_frequency = NRF_UARTE_BASE_FREQUENCY_GET(p_instance->p_reg);
    if (base_frequency != NRF_UARTE_BASE_FREQUENCY_16MHZ)
    {
        uint32_t baudrate_factor = base_frequency / NRF_UARTE_BASE_FREQUENCY_16MHZ;
        nrfy_config.baudrate =
            (nrf_uarte_baudrate_t)((uint32_t)p_config->baudrate / baudrate_factor);
    }
    else
#endif
    {
        nrfy_config.baudrate = p_config->baudrate;
    }

    nrfy_uarte_periph_configure(p_instance->p_reg, &nrfy_config);

    if (NRF_ERRATA_DYNAMIC_CHECK(53, 44) || NRF_ERRATA_DYNAMIC_CHECK(91, 23))
    {
        apply_workaround_for_enable_anomaly(p_instance);
    }

    nrfy_uarte_int_init(p_instance->p_reg,
                        NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ENDRX) |
                        NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ENDTX) |
                        NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ERROR) |
                        NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_RXTO)  |
                        NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_TXSTOPPED),
                        p_config->interrupt_priority,
                        false);
}

int w1_uarte_init(nrfx_uarte_t *              p_instance,
                    nrfx_uarte_config_t const * p_config,
                    nrfx_uarte_event_handler_t  event_handler)
{
    NRFX_ASSERT(p_instance && p_config);
    nrfx_uarte_control_block_t * p_cb = &p_instance->cb;
    int err_code;

    if (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = -EALREADY;
        // LOG_WRN("Function: %s, error code: %s.",
        //                  __func__,
        //                  NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    memset(p_cb, 0, sizeof(nrfx_uarte_control_block_t));

    p_cb->p_context = p_config->p_context;

    uint32_t tx_int_mask = (!event_handler || p_config->tx_stop_on_end) ?
                               0 : NRF_UARTE_INT_ENDTX_MASK;

    if (p_config->tx_stop_on_end)
    {
        p_cb->flags |= UARTE_FLAG_TX_STOP_ON_END;
    }

    p_cb->handler = event_handler;
    p_cb->state   = NRFX_DRV_STATE_INITIALIZED;

    // Handle case when other user (e.g. bootloader) left RX in active state.
    if (!w1_prepare_rx(p_instance->p_reg))
    {
        return -ECANCELED;
    }

    uarte_configure(p_instance, p_config);

    if (!w1_prepare_tx(p_instance->p_reg, p_config->tx_stop_on_end))
    {

        return -ECANCELED;
    }

    uint32_t int_mask = tx_int_mask | ((event_handler) ? rx_int_mask : 0);

    nrfy_uarte_int_enable(p_instance->p_reg, int_mask);

    return 0;
}

static bool is_tx_active(NRF_UARTE_Type * p_uarte)
{
    return !nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_TXSTOPPED) ||
        nrfy_uarte_int_enable_check(p_uarte, NRF_UARTE_INT_TXSTOPPED_MASK);
}

static void disable_hw_from_rx(NRF_UARTE_Type * p_uarte)
{
    NRFX_CRITICAL_SECTION_ENTER();

    if (!is_tx_active(p_uarte))
    {
        nrfy_uarte_disable(p_uarte);
    }

    NRFX_CRITICAL_SECTION_EXIT();
}

static uint32_t uarte_int_lock(NRF_UARTE_Type * p_uarte)
{
    uint32_t int_enabled = nrfy_uarte_int_enable_check(p_uarte, UINT32_MAX);

    nrfy_uarte_int_disable(p_uarte, int_enabled);

    return int_enabled;
}
#if defined(CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
static void wait_for_rx_completion(NRF_UARTE_Type *             p_uarte,
                                   nrfx_uarte_control_block_t * p_cb)
{
    while(nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXTO) == false)
    {}

    nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXSTARTED);
    nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDRX);
    nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXTO);

    disable_hw_from_rx(p_uarte);

    p_cb->rx.curr.p_buffer = NULL;
    p_cb->rx.next.p_buffer = NULL;
    release_rx(p_cb);
}
#endif
static void uarte_int_unlock(NRF_UARTE_Type * p_uarte, uint32_t int_mask)
{
    nrfy_uarte_int_enable(p_uarte, int_mask);
}

#if defined(CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
static int rx_abort(nrfx_uarte_t * p_instance,
                    bool           disable_all,
                    bool           sync)
{
    NRF_UARTE_Type * p_uarte = p_instance->p_reg;
    nrfx_uarte_control_block_t * p_cb = &p_instance->cb;
    uint32_t int_enabled;


    int_enabled = uarte_int_lock(p_uarte);

    if (sync || !p_cb->handler)
    {
        nrfy_uarte_int_disable(p_uarte, rx_int_mask);
        nrfy_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPRX);
        wait_for_rx_completion(p_uarte, p_cb);
        int_enabled &= ~rx_int_mask;
    }
    else
    {
        nrfy_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPRX);
    }

    uarte_int_unlock(p_uarte, int_enabled);

    return 0;
}

static int w1_uarte_rx_abort(nrfx_uarte_t * p_instance, bool disable_all, bool sync)
{
    NRFX_ASSERT(p_instance && (p_instance->cb.state == NRFX_DRV_STATE_INITIALIZED));

    return rx_abort(p_instance, disable_all, sync);
}

static int w1_uarte_rx_ready(nrfx_uarte_t const * p_instance, size_t * p_rx_amount)
{
    NRFX_ASSERT(p_instance);

    nrfx_uarte_control_block_t const * p_cb = &p_instance->cb;

    NRFX_ASSERT(p_cb->state == NRFX_DRV_STATE_INITIALIZED);

    if (p_cb->handler)
    {
        return -EPERM;
    }

    if (!nrfy_uarte_enable_check(p_instance->p_reg))
    {
        if (p_rx_amount)
        {
            *p_rx_amount = nrfy_uarte_rx_amount_get(p_instance->p_reg);
        }

        return -EALREADY;
    }
    else if (nrfy_uarte_event_check(p_instance->p_reg, NRF_UARTE_EVENT_ENDRX))
    {
        if (p_rx_amount)
        {
            *p_rx_amount = nrfy_uarte_rx_amount_get(p_instance->p_reg);
        }
        return 0;
    }
    else
    {
        return -EBUSY;
    }
}

#endif
static inline bool event_check_and_clear(NRF_UARTE_Type *  p_uarte,
                                         nrf_uarte_event_t event,
                                         uint32_t          int_mask)
{
    if (nrfy_uarte_event_check(p_uarte, event) && (int_mask & NRFY_EVENT_TO_INT_BITMASK(event))) {
        nrfy_uarte_event_clear(p_uarte, event);
        return true;
    }

    return false;
}

static inline bool event_check(NRF_UARTE_Type *  p_uarte,
                               nrf_uarte_event_t event,
                               uint32_t          int_mask)
{
    return nrfy_uarte_event_check(p_uarte, event) &&
           (int_mask & NRFY_EVENT_TO_INT_BITMASK(event));
}

static void user_handler_on_error(NRF_UARTE_Type * p_uarte, nrfx_uarte_control_block_t * p_cb)
{
    nrfx_uarte_event_t event = {
        .type = NRFX_UARTE_EVT_ERROR,
        .data = {
            .error = {
                .error_mask = nrfy_uarte_errorsrc_get_and_clear(p_uarte)
            }
        }
    };

    p_cb->handler(&event, p_cb->p_context);
}

static void error_irq_handler(NRF_UARTE_Type *             p_uarte,
                              nrfx_uarte_control_block_t * p_cb)
{
    user_handler_on_error(p_uarte, p_cb);
}

static void user_handler(nrfx_uarte_control_block_t * p_cb, nrfx_uarte_event_type_t type)
{
    nrfx_uarte_event_t event = {
        .type = type
    };

    p_cb->handler(&event, p_cb->p_context);
}

static void user_handler_on_rx_done(nrfx_uarte_control_block_t * p_cb,
                                    const uint8_t *              p_data,
                                    size_t                       len)
{
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif

    nrfx_uarte_event_t event = {
        .type = NRFX_UARTE_EVT_RX_DONE,
        .data = {
            .rx = {
                .p_buffer = (uint8_t *)p_data,
                .length = len
            }
        }
    };

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

    p_cb->handler(&event, p_cb->p_context);
}


static void handler_on_rx_done(nrfx_uarte_control_block_t * p_cb,
                               uint8_t *                    p_data,
                               size_t                       len,
                               bool                         abort)
{
    user_handler_on_rx_done(p_cb, p_data, len);
    return;
}

static bool endrx_irq_handler(NRF_UARTE_Type *             p_uarte,
                              nrfx_uarte_control_block_t * p_cb,
                              bool                         rxstarted)
{
    size_t rx_amount = (size_t)nrfy_uarte_rx_amount_get(p_uarte);
    bool premature = p_cb->flags & (UARTE_FLAG_RX_RESTARTED | UARTE_FLAG_RX_ABORTED);
    bool aborted = false;
    bool late = false;
    uint8_t *p_buf = p_cb->rx.curr.p_buffer;
    size_t len = rx_amount + p_cb->rx.off;

    p_cb->rx.curr = p_cb->rx.next;
    p_cb->rx.next = (nrfy_uarte_buffer_t){ NULL, 0 };
    handler_on_rx_done(p_cb, p_buf, len, premature);
    p_cb->rx.off = 0;

    NRFX_CRITICAL_SECTION_ENTER();

    p_cb->flags &= ~UARTE_FLAG_RX_RESTARTED;
    nrfy_uarte_shorts_disable(p_uarte, NRF_UARTE_SHORT_ENDRX_STARTRX);
    if (p_cb->flags & UARTE_FLAG_RX_ABORTED)
    {
            aborted = true;
    }
    else if (p_cb->rx.curr.p_buffer == NULL)
    {
        if (p_cb->flags & UARTE_FLAG_RX_STOP_ON_END)
        {
            nrfy_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPRX);
            p_cb->flags |= UARTE_FLAG_RX_ABORTED;
        }
    }
    else if (!((p_cb->flags & UARTE_FLAG_RX_CONT) &&
                (rxstarted || nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXSTARTED))))
    {
        // If continuous transfer is set then we expect that new RX is already started by short.
        // However it is possible that buffer was provided after ENDRX occurred. It can be checked
        // by examining RXSTARTED event. If it did not occur that indicates that second buffer did
        // not start but was set. It must be started manually. Usually RXSTARTED occurs almost
        // immediately after ENDRX (if short is set) so rxstarted variable check seems to be enough
        // but there are cases when RXSTARTED may be slightly delayed so if it is not set we
        // need to check HW register to make sure that it did not occur.
        nrfy_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STARTRX);
        if (nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXTO))
        {
            late = true;
        }
    }

    NRFX_CRITICAL_SECTION_EXIT();

    if (late)
    {
        nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXTO);
        nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXSTARTED);
        nrfy_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPRX);
        user_handler(p_cb, NRFX_UARTE_EVT_RX_BUF_TOO_LATE);
    }

    return aborted;
}

static void rxstarted_irq_handler(NRF_UARTE_Type * p_reg, nrfx_uarte_control_block_t * p_cb)
{
    // bool cache_used = RX_CACHE_SUPPORTED && (p_cb->flags & UARTE_FLAG_RX_USE_CACHE);

    // If both buffers are already setup just leave. It is possible in case
    // of following scenario. 1 byte was requested and received. RX done event
    // is generated and from that event nrfx_uarte_rx_buffer_set is called to
    // receive next RX. This sets current buffer. After ENDRX event is processed
    // then RXSTARTED event is processed (from that 1 byte transfer). From there
    // RX buffer request is called and next buffer is provided - both buffers are
    // set. Next RX is started and RXSTARTED event is triggered and we are in
    // the situation where both buffers are already set.
    if ((p_cb->rx.curr.p_buffer != NULL) && (p_cb->rx.next.p_buffer != NULL))
    {
        return;
    }

    user_handler(p_cb, NRFX_UARTE_EVT_RX_BUF_REQUEST);
    return;

}

static void user_handler_on_rx_disabled(nrfx_uarte_control_block_t * p_cb, size_t flush_cnt)
{
    nrfx_uarte_event_t event = {
        .type = NRFX_UARTE_EVT_RX_DISABLED,
        .data = {
            .rx_disabled = {
                .flush_cnt = flush_cnt
            }
        }
    };

    p_cb->handler(&event, p_cb->p_context);
}

static void on_rx_disabled(NRF_UARTE_Type *             p_uarte,
                           nrfx_uarte_control_block_t * p_cb,
                           size_t                       flush_cnt)
{
    nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXDRDY);
    nrfy_uarte_shorts_disable(p_uarte, NRF_UARTE_SHORT_ENDRX_STARTRX);
    nrfy_uarte_int_disable(p_uarte, rx_int_mask);
    disable_hw_from_rx(p_uarte);

    p_cb->rx.curr.p_buffer = NULL;
    p_cb->rx.next.p_buffer = NULL;
    release_rx(p_cb);
    user_handler_on_rx_disabled(p_cb, flush_cnt);
}

static void rxto_irq_handler(NRF_UARTE_Type *        p_uarte,
                             nrfx_uarte_control_block_t * p_cb)
{
    if (p_cb->rx.curr.p_buffer)
    {
        handler_on_rx_done(p_cb, p_cb->rx.curr.p_buffer, 0, true);
        p_cb->rx.curr.p_buffer = NULL;
    }

    on_rx_disabled(p_uarte, p_cb, p_cb->rx.flush.length);
}



static void user_handler_on_tx_done(nrfx_uarte_control_block_t * p_cb,
                                    const uint8_t *              p_data,
                                    size_t                       len,
                                    bool                         abort)
{
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif

    nrfx_uarte_event_t event = {
        .type = NRFX_UARTE_EVT_TX_DONE,
        .data = {
            .tx = {
                .p_buffer = (uint8_t *)p_data,
                .length = len,
                .flags = abort ? NRFX_UARTE_TX_DONE_ABORTED : 0
            }
        }
    };

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

    p_cb->handler(&event, p_cb->p_context);
}


static void txstopped_irq_handler(NRF_UARTE_Type *             p_uarte,
                                  nrfx_uarte_control_block_t * p_cb)
{
    nrfy_uarte_int_disable(p_uarte, NRF_UARTE_INT_TXSTOPPED_MASK);

    NRFX_CRITICAL_SECTION_ENTER();
    disable_hw_from_tx(p_uarte, p_cb);
    NRFX_CRITICAL_SECTION_EXIT();

    // If no length set, it means that it was a blocking transfer.
    if (p_cb->tx.curr.length == 0)
    {
        return;
    }

    bool aborted = p_cb->flags & UARTE_FLAG_TX_ABORTED;
    size_t amount = p_cb->tx.amount >= 0 ? (size_t)p_cb->tx.amount : nrfy_uarte_tx_amount_get(p_uarte);

    p_cb->tx.off += amount;

    NRFX_ASSERT(p_cb->tx.off == p_cb->tx.curr.length || aborted);

    uint32_t off = p_cb->tx.off;

    // Transfer completed.
    p_cb->flags &= ~UARTE_FLAG_TX_ABORTED;
    p_cb->tx.curr.length = 0;
    p_cb->tx.off = 0;
    user_handler_on_tx_done(p_cb, p_cb->tx.curr.p_buffer, off, aborted);
}


static void endtx_irq_handler(NRF_UARTE_Type * p_uarte, nrfx_uarte_control_block_t * p_cb)
{
    // Locking since blocking transfer can interrupt at anytime. In that case we don't
    // want to stop ongoing blocking transfer.
    NRFX_CRITICAL_SECTION_ENTER();
    if (nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDTX))
    {
        nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDTX);
        nrfy_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPTX);
    }
    NRFX_CRITICAL_SECTION_EXIT();
}

static int rx_buffer_set(NRF_UARTE_Type *            p_uarte,
                        nrfx_uarte_control_block_t * p_cb,
                        uint8_t *                    p_data,
                        size_t                       length)
{
    int err_code = 0;

    if (p_cb->rx.curr.p_buffer == NULL ||
        (!p_cb->handler && nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDRX)))
    {
        if (p_cb->rx.curr.p_buffer)
        {
            nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDRX);
        }

        p_cb->rx.curr.p_buffer = p_data;
        p_cb->rx.curr.length = length;

        nrfy_uarte_rx_buffer_set(p_uarte,
                                &p_cb->rx.curr.p_buffer[p_cb->rx.off],
                                p_cb->rx.curr.length - p_cb->rx.off);
        if (p_cb->flags & UARTE_FLAG_RX_ENABLED)
        {
            NRFX_ATOMIC_FETCH_AND(&p_cb->flags, ~UARTE_FLAG_RX_ABORTED);
            nrfy_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STARTRX);
            if (nrfy_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXTO))
            {
                nrfy_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXTO);
            }
        }

    }
    else if (p_cb->rx.next.p_buffer == NULL)
    {
        p_cb->rx.next.p_buffer = p_data;
        p_cb->rx.next.length = length;

        nrfy_uarte_rx_buffer_set(p_uarte, p_data, length);
        if (p_cb->flags & UARTE_FLAG_RX_CONT)
        {
            nrfy_uarte_shorts_enable(p_uarte, NRF_UARTE_SHORT_ENDRX_STARTRX);
        }
    }
    else
    {
        err_code = -EBUSY;
    }

    return err_code;
}

static int w1_uarte_rx_buffer_set(nrfx_uarte_t * p_instance,
                             uint8_t *      p_data,
                             size_t         length)
{
    NRFX_ASSERT(p_instance);

    nrfx_uarte_control_block_t * p_cb = &p_instance->cb;

    NRFX_ASSERT((p_cb->state == NRFX_DRV_STATE_INITIALIZED) && (length > 0) && p_data);

    NRF_UARTE_Type * p_uarte = p_instance->p_reg;
    bool cont = false;
    uint32_t int_enabled;
    int err = 0;

    int_enabled = uarte_int_lock(p_uarte);

    if (p_cb->flags & UARTE_FLAG_RX_ABORTED)
    {
        err = -EINPROGRESS;
    }
    else
    {
        cont = true;
    }

    if (cont)
    {
        err = rx_buffer_set(p_uarte, p_cb, p_data, length);
    }

    uarte_int_unlock(p_uarte, int_enabled);

    return err;
}

void w1_uarte_irq_handler(nrfx_uarte_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    nrfx_uarte_control_block_t * p_cb = &p_instance->cb;
    NRF_UARTE_Type * p_uarte = p_instance->p_reg;

    // ENDTX must be handled before TXSTOPPED so we read event status in the reversed order of
    // handling.
    uint32_t int_mask = nrfy_uarte_int_enable_check(p_uarte, UINT32_MAX);
    bool txstopped = event_check(p_uarte, NRF_UARTE_EVENT_TXSTOPPED, int_mask);
    bool endtx =  event_check(p_uarte, NRF_UARTE_EVENT_ENDTX, int_mask);

    if (p_cb->handler)
    {
        if (event_check_and_clear(p_uarte, NRF_UARTE_EVENT_ERROR, int_mask))
        {
            error_irq_handler(p_uarte, p_cb);
        }

        // ENDRX must be handled before RXSTARTED. RXTO must be handled as the last one. We collect
        // state of all 3 events before processing to prevent reordering in case of higher interrupt
        // preemption. We read event status in the reversed order of handling.
        bool rxto = event_check_and_clear(p_uarte, NRF_UARTE_EVENT_RXTO, int_mask);
        bool rxstarted = event_check_and_clear(p_uarte, NRF_UARTE_EVENT_RXSTARTED, int_mask);
        bool endrx = event_check_and_clear(p_uarte, NRF_UARTE_EVENT_ENDRX, int_mask);
        bool rxdrdy = event_check_and_clear(p_uarte, NRF_UARTE_EVENT_RXDRDY, int_mask);

        if (rxdrdy)
        {
            user_handler(p_cb, NRFX_UARTE_EVT_RX_BYTE);
        }

        if (endrx)
        {
            // If interrupt was executed exactly when ENDRX occurred it is possible
            // that RXSTARTED (which is read before ENDRX) is read as false but it
            // actually occurred (if there is a linked reception). Read again to be sure.
            if (!rxstarted)
            {
                rxstarted = event_check_and_clear(p_uarte, NRF_UARTE_EVENT_RXSTARTED, int_mask);
            }

            if (endrx_irq_handler(p_uarte, p_cb, rxstarted) == true)
            {
                rxstarted = false;
            }
        }

        if (rxstarted)
        {
            rxstarted_irq_handler(p_uarte, p_cb);
        }

        if (rxto)
        {
            rxto_irq_handler(p_uarte, p_cb);
        }
    }

    if (endtx)
    {
        endtx_irq_handler(p_uarte, p_cb);
    }

    if (txstopped)
    {
        txstopped_irq_handler(p_uarte, p_cb);
    }

}

/*
 * Concurrently transmits and receives one 1-Wire bit
 * by sending and receiving one uart byte
 */
static int serial_tx_rx(const struct device *dev, const uint8_t *tx_data, uint8_t *rx_data,
			size_t len, uint32_t timeout)
{
    struct w1_serial_data * data = dev->data;
    int ret;

    #if defined(CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
    k_timepoint_t end;
    #endif

	__ASSERT_NO_MSG(tx_data != NULL);
	__ASSERT_NO_MSG(rx_data != NULL);

    ret = w1_uarte_rx_buffer_set(&data->inst, rx_data, len);
    if (ret != 0) {
        LOG_ERR("Failed to arm rx buffer");
        return -EIO;
    }

    ret = w1_uarte_rx_enable(&data->inst);
    if (ret != 0 && ret != -EBUSY) {
        LOG_ERR("Failed to enable uarte RX");
        return -EIO;
    }

    #if defined(CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
    end = sys_timepoint_calc(K_USEC((uint64_t)timeout * len));

    ret = w1_blocking_tx(data->inst.p_reg, tx_data, len, end);

    if (ret != 0) {
        (void)w1_uarte_rx_abort(&data->inst, true, true);
        LOG_ERR("Failed to send data");
        return -EIO;
    }

    do {
        ret = w1_uarte_rx_ready(&data->inst, NULL);
    }
    while (ret != 0 && !sys_timepoint_expired(end));

    if (ret != 0) {
        (void)w1_uarte_rx_abort(&data->inst, true, true);
        LOG_ERR("Failed to receive data");
        return -EIO;
    }

    #else
    ret = w1_uarte_tx(&data->inst, tx_data, len);
    if (ret != 0) {
        LOG_ERR("Failed to send data");
        return -EIO;
    }

    if (k_sem_take(&data->rx_sem, K_USEC((uint64_t)timeout * len)) != 0) {
        LOG_ERR("Failed to take rx semaphore");
        return -EIO;
    }
    #endif
    return 0;
}

/* Concurretly transmits and receives one 1-Wire byte */
static int serial_tx_rx_byte(const struct device *dev, uint8_t tx_byte, uint8_t *rx_byte)
{
	__ASSERT_NO_MSG(rx_byte != NULL);
	uint8_t tx_representation[8];
	uint8_t rx_representation[8];

	for (int i = 0; i < 8; ++i) {
		/*
		 * The uart start bit pulls the line low to indicate either
		 * write Bit 1, or read low time.
		 * Bit representations vary by speed mode.
		 */
		tx_representation[i] =
			((tx_byte & (1 << i)) != 0) ? W1_SERIAL_BIT_1 : W1_SERIAL_BIT_0;
	}

	if (serial_tx_rx(dev, tx_representation, rx_representation, 8,
			 CONFIG_NRF_W1_SERIAL_BIT_TIMEOUT) < 0) {
		return -EIO;
	}

	*rx_byte = 0;
	for (int i = 0; i < 8; ++i) {
		/*
		 * rx-byte different from 0xFF indicates that a slave has
		 * pulled line low to transmit a 0 bit, otherwise a 1 bit.
		 */
		*rx_byte |= (uint8_t)(rx_representation[i] == 0xFF) << i;
	}

	return 0;
}

/*
 * Maps a raw devicetree baud rate (Hz) to the nrf_uarte_baudrate_t enum required by nrfx.
 */
static nrf_uarte_baudrate_t w1_serial_baudrate_from_hz(uint32_t baud)
{
	switch (baud) {
	case 1200:    return NRF_UARTE_BAUDRATE_1200;
	case 2400:    return NRF_UARTE_BAUDRATE_2400;
	case 4800:    return NRF_UARTE_BAUDRATE_4800;
	case 9600:    return NRF_UARTE_BAUDRATE_9600;
	case 14400:   return NRF_UARTE_BAUDRATE_14400;
	case 19200:   return NRF_UARTE_BAUDRATE_19200;
	case 28800:   return NRF_UARTE_BAUDRATE_28800;
	case 31250:   return NRF_UARTE_BAUDRATE_31250;
	case 38400:   return NRF_UARTE_BAUDRATE_38400;
	case 56000:   return NRF_UARTE_BAUDRATE_56000;
	case 57600:   return NRF_UARTE_BAUDRATE_57600;
	case 76800:   return NRF_UARTE_BAUDRATE_76800;
	case 115200:  return NRF_UARTE_BAUDRATE_115200;
	case 230400:  return NRF_UARTE_BAUDRATE_230400;
	case 250000:  return NRF_UARTE_BAUDRATE_250000;
	case 460800:  return NRF_UARTE_BAUDRATE_460800;
	case 921600:  return NRF_UARTE_BAUDRATE_921600;
	case 1000000: return NRF_UARTE_BAUDRATE_1000000;
	default:
		LOG_WRN("Unsupported baud rate %u, falling back to 115200", baud);
		return NRF_UARTE_BAUDRATE_115200;
	}
}

static int w1_serial_reset_bus(const struct device *dev)
{
	const struct w1_serial_config *cfg = dev->config;
	struct w1_serial_data *data = dev->data;
	const uint8_t reset_byte_tx =
		data->overdrive_active ? W1_SERIAL_OD_RESET_BYTE : W1_SERIAL_STD_RESET_BYTE;
	uint8_t reset_byte_rx = 0;
	/* reset uses 115200/9600=12 slower baudrate,
	 * adjust timeout accordingly, also valid for overdrive speed.
	 */
	const uint32_t reset_timeout = CONFIG_NRF_W1_SERIAL_BIT_TIMEOUT * 12;

	nrf_uarte_baudrate_set(data->inst.p_reg, data->overdrive_active ?
		w1_serial_baudrate_from_hz(cfg->od_reset_baud) : NRF_UARTE_BAUDRATE_9600);

	if (serial_tx_rx(dev, &reset_byte_tx, &reset_byte_rx, 1, reset_timeout) < 0) {
		LOG_ERR("tx_rx_error reset_present");
		return -EIO;
	}

	nrf_uarte_baudrate_set(data->inst.p_reg, data->overdrive_active ?
		w1_serial_baudrate_from_hz(cfg->od_data_baud) : NRF_UARTE_BAUDRATE_115200);

	/* At least 1 device is present on bus if reset_byte_rx is different
	 * from reset_byte_tx (which varies depending on standard or overdrive mode).
	 * Bus is probably shorted if reset_byte_rx is 0x00.
	 */
	return (reset_byte_rx != reset_byte_tx) && (reset_byte_rx != 0x00);
}

static int w1_serial_read_bit(const struct device *dev)
{
	uint8_t tx_bit = W1_SERIAL_READ_REQ_BYTE;
	uint8_t rx_bit;

	if (serial_tx_rx(dev, &tx_bit, &rx_bit, 1, CONFIG_NRF_W1_SERIAL_BIT_TIMEOUT) != 0) {
		return -EIO;
	};

	return rx_bit == W1_SERIAL_READ_REQ_BYTE;
}

static int w1_serial_write_bit(const struct device *dev, const bool bit)
{
	uint8_t tx_bit;
	uint8_t rx_bit;

	tx_bit = bit ? W1_SERIAL_BIT_1 : W1_SERIAL_BIT_0;

	if (serial_tx_rx(dev, &tx_bit, &rx_bit, 1, CONFIG_NRF_W1_SERIAL_BIT_TIMEOUT) != 0) {
		return -EIO;
	}
	return 0;
}

static int w1_serial_read_byte(const struct device *dev)
{
	uint8_t tx_byte = 0xFF;
	uint8_t rx_byte;

	if (serial_tx_rx_byte(dev, tx_byte, &rx_byte) != 0) {
		return -EIO;
	}

	return rx_byte;
}

static int w1_serial_write_byte(const struct device *dev, const uint8_t byte)
{
	uint8_t rx_byte;

	return serial_tx_rx_byte(dev, byte, &rx_byte);
}

static int w1_serial_configure(const struct device *dev, enum w1_settings_type type, uint32_t value)
{
	const struct w1_serial_config *cfg = dev->config;
	struct w1_serial_data *data = dev->data;
	int ret = 0;
	bool temp;

	switch (type) {
	case W1_SETTING_SPEED:
		temp = (bool)value;
		if (temp == data->overdrive_active) {
			break;
		}

		data->overdrive_active = temp;
		nrf_uarte_baudrate_set(data->inst.p_reg, data->overdrive_active ?
			w1_serial_baudrate_from_hz(cfg->od_data_baud) : NRF_UARTE_BAUDRATE_115200);
		break;
	default:
		ret = -ENOTSUP;
	}
	return ret;
}

static int w1_serial_init(const struct device *dev)
{
	const struct w1_serial_config *cfg = dev->config;
	struct w1_serial_data *data = dev->data;
    int ret = 0;

    #if !defined(CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
    k_sem_init(&data->rx_sem, 0, 1);
    #endif

    nrfx_uarte_config_t config = NRFX_UARTE_DEFAULT_CONFIG(NRF_UARTE_PSEL_DISCONNECTED,
                                                           NRF_UARTE_PSEL_DISCONNECTED);

    config.skip_gpio_cfg = true;
    config.skip_psel_cfg = true;

    config.p_context = data;

    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret != 0) {
        LOG_ERR("Failed to apply pin control state");
        return ret;
    }
    #if defined(CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
    if (w1_uarte_init(&data->inst, &config, NULL) != 0) {
        LOG_ERR("Failed to initialize uarte");
        return -EIO;
    }
    #else
    if (w1_uarte_init(&data->inst, &config, w1_uarte_event_handler) != 0) {
        LOG_ERR("Failed to initialize uarte");
        return -EIO;
    }

    cfg->irq_config_func();

    #endif

	data->overdrive_active = false;

	LOG_DBG("w1-serial initialized, with %d slave devices", cfg->master_config.slave_count);
	return 0;
}

static DEVICE_API(w1, w1_serial_driver_api) = {
	.reset_bus = w1_serial_reset_bus,
	.read_bit = w1_serial_read_bit,
	.write_bit = w1_serial_write_bit,
	.read_byte = w1_serial_read_byte,
	.write_byte = w1_serial_write_byte,
	.configure = w1_serial_configure,
};

#define NRF_W1_SERIAL_INIT(n)       \
    PINCTRL_DT_INST_DEFINE(n);                                                          \
    static struct w1_serial_data w1_serial_data_##n = {                                 \
        .inst = NRFX_UARTE_INSTANCE(DT_INST_REG_ADDR(n)),                               \
    };                                                                                  \
    static void w1_uarte_isr_##n(void)                                                  \
    {                                                                                   \
        w1_uarte_irq_handler(&w1_serial_data_##n.inst);                               \
    }                                                                                   \
    static void w1_uarte_irq_connect_##n(void)                                          \
    {                                                                                   \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),                          \
                    w1_uarte_isr_##n, NULL, 0);                                         \
        irq_enable(DT_INST_IRQN(n));                                                     \
    }                                                                                   \
    static const struct w1_serial_config w1_serial_cfg_##n = {                         \
        .master_config.slave_count = W1_INST_SLAVE_COUNT(n),                           \
        .od_data_baud = DT_INST_PROP(n, overdrive_data_baud),                          \
        .od_reset_baud = DT_INST_PROP(n, overdrive_reset_baud),                        \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                     \
        .irq_config_func = w1_uarte_irq_connect_##n,                                   \
    };                                                                                 \
    DEVICE_DT_INST_DEFINE(n, &w1_serial_init, NULL, &w1_serial_data_##n,               \
			      &w1_serial_cfg_##n, POST_KERNEL, CONFIG_W1_INIT_PRIORITY, \
			      &w1_serial_driver_api);
DT_INST_FOREACH_STATUS_OKAY(NRF_W1_SERIAL_INIT)