/*
 * ZMK module: Framework 16 kscan driver
 *
 * Scans a 16x8 key matrix using:
 * - 16 column drive GPIOs (active-low sink)
 * - 3 MUX select GPIOs + 1 MUX enable (active-low) to select a row
 * - 1 ADC channel (RP2040 ADC) to read the selected row voltage
 *
 * Drive each column low one at a time, select each row via the MUX, read ADC
 * millivolts, and compare against a dynamic threshold to determine
 * pressed/released state. Debouncing is handled using ZMK's debounce helper
 * per-matrix position.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/debounce.h>

LOG_MODULE_REGISTER(kscan_fw16, LOG_LEVEL_DBG);

#define DT_DRV_COMPAT zmk_kscan_fw16

/* Timing constants (microseconds) */
#define COLUMN_SETTLE_US 100
#define MUX_SETTLE_US    30

/* ADC settings */
#define ADC_RESOLUTION   12

/* Heuristic threshold levels in millivolts based on number of keys "low" in a row */
#define THRESH_0_OR_1_PRESSED 1000 /* 1.0 V */
#define THRESH_2_PRESSED      2000 /* 2.0 V */
#define THRESH_3_PRESSED      2500 /* 2.5 V */
#define THRESH_4_PLUS         2900 /* 2.9 V */

struct fw16_cfg {
    /* Column drive GPIOs */
    const struct gpio_dt_spec *cols;
    size_t col_cnt;

    /* Row selection MUX: 3 select pins + 1 enable pin (active low) */
    struct gpio_dt_spec mux[3];
    struct gpio_dt_spec mux_en;

    /* Optional GPIO descriptor for the ADC pad to apply pull (e.g. pull-up) */
    struct gpio_dt_spec adc_pad;

    /* ADC device + channel */
    const struct device *adc_dev;
    uint8_t adc_channel_id;

    /* Matrix geometry */
    uint8_t rows;
    uint8_t cols_count;

    /* Debounce config */
    struct zmk_debounce_config debounce_config;
    int32_t debounce_scan_period_ms;

    /* Polling config */
    int32_t poll_period_ms;
};

struct fw16_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_work_delayable work;
    int64_t scan_time;
    struct zmk_debounce_state *state; /* rows * cols_count */
};

static inline int idx_rc(const struct fw16_cfg *cfg, int r, int c) {
    return (c * cfg->rows) + r;
}

/*
 * The row index to mux select value mapping is non-linear for rows 0..2.
 */
static inline uint8_t mux_sel_for_row(uint8_t r) {
    switch (r) {
    case 0:
        return 2;
    case 1:
        return 0;
    case 2:
        return 1;
    default:
        return r;
    }
}

static int set_mux(const struct fw16_cfg *cfg, uint8_t row) {
    uint8_t sel = mux_sel_for_row(row) & 0x7;

    int err = 0;
    err |= gpio_pin_set_dt(&cfg->mux[0], (sel & 0x01) ? 1 : 0);
    err |= gpio_pin_set_dt(&cfg->mux[1], (sel & 0x02) ? 1 : 0);
    err |= gpio_pin_set_dt(&cfg->mux[2], (sel & 0x04) ? 1 : 0);
    return err;
}

static int adc_read_mv(const struct fw16_cfg *cfg, int32_t *out_mv) {
    struct adc_channel_cfg ch = {
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = cfg->adc_channel_id,
    };

    int err = adc_channel_setup(cfg->adc_dev, &ch);
    if (err) {
        return err;
    }

    uint16_t raw = 0;
    struct adc_sequence seq = {
        .buffer = &raw,
        .buffer_size = sizeof(raw),
        .channels = BIT(cfg->adc_channel_id),
        .resolution = ADC_RESOLUTION,
    };

    err = adc_read(cfg->adc_dev, &seq);
    if (err) {
        return err;
    }

    int32_t mv = raw;
    int32_t vref = adc_ref_internal(cfg->adc_dev);

    if (vref > 0) {
        if (adc_raw_to_millivolts(vref, ch.gain, seq.resolution, &mv) < 0) {
            mv = raw; /* Fallback if conversion helper not available */
        }
    }

    *out_mv = mv;
    return 0;
}

static inline int32_t threshold_for_pressed_count(int pressed_in_row) {
    switch (pressed_in_row) {
    case 0:
    case 1:
        return THRESH_0_OR_1_PRESSED;
    case 2:
        return THRESH_2_PRESSED;
    case 3:
        return THRESH_3_PRESSED;
    default:
        return THRESH_4_PLUS;
    }
}

static void schedule_read_continue(const struct device *dev) {
    struct fw16_data *d = dev->data;
    const struct fw16_cfg *c = dev->config;

    d->scan_time += c->debounce_scan_period_ms;
    k_work_reschedule(&d->work, K_TIMEOUT_ABS_MS(d->scan_time));
}

static void schedule_read_end(const struct device *dev) {
    struct fw16_data *d = dev->data;
    const struct fw16_cfg *c = dev->config;

    d->scan_time += c->poll_period_ms;
    k_work_reschedule(&d->work, K_TIMEOUT_ABS_MS(d->scan_time));
}

static int scan_once(const struct device *dev) {
    struct fw16_data *d = dev->data;
    const struct fw16_cfg *c = dev->config;

    /* Ensure all columns are inactive (high) before scanning */
    for (int col = 0; col < c->cols_count; col++) {
        int err = gpio_pin_set_dt(&c->cols[col], 1);
        if (err) {
            return err;
        }
    }

    /* Capture readings: mv[row][col] */
    int16_t mv[8][16] = {0}; /* Sized for max rows/cols per DTS: 8x16 */

    for (int col = 0; col < c->cols_count; col++) {
        int err = gpio_pin_set_dt(&c->cols[col], 0); /* active: sink */
        if (err) {
            return err;
        }

        k_busy_wait(COLUMN_SETTLE_US);

        for (int row = 0; row < c->rows; row++) {
            err = set_mux(c, row);
            if (err) {
                return err;
            }

            k_busy_wait(MUX_SETTLE_US);

            int32_t v_mv = 0;
            err = adc_read_mv(c, &v_mv);
            if (err) {
                return err;
            }

            mv[row][col] = (int16_t)v_mv;
        }

        err = gpio_pin_set_dt(&c->cols[col], 1); /* inactive */
        if (err) {
            return err;
        }
    }

    /* Debounce + report changes */
    bool any_active = false;

    for (int row = 0; row < c->rows; row++) {
        int pressed_in_row = 0;
        for (int col = 0; col < c->cols_count; col++) {
            if (mv[row][col] < THRESH_0_OR_1_PRESSED) {
                pressed_in_row++;
            }
        }

        if (pressed_in_row > 0) {
            LOG_DBG("fw16: row %d voltages mV: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                    row, mv[row][0], mv[row][1], mv[row][2], mv[row][3], mv[row][4], mv[row][5], mv[row][6],
                    mv[row][7], mv[row][8], mv[row][9], mv[row][10], mv[row][11], mv[row][12], mv[row][13],
                    mv[row][14], mv[row][15]);
        }

        const int32_t row_thr_mv = threshold_for_pressed_count(pressed_in_row);

        for (int col = 0; col < c->cols_count; col++) {
            const bool pressed = (mv[row][col] < row_thr_mv);
            const int i = idx_rc(c, row, col);

            zmk_debounce_update(&d->state[i], pressed, c->debounce_scan_period_ms, &c->debounce_config);

            if (zmk_debounce_get_changed(&d->state[i])) {
                const bool is_pressed = zmk_debounce_is_pressed(&d->state[i]);
                LOG_DBG("fw16: rc(%d,%d) mv=%d thr=%d pressed=%d", row, col, mv[row][col], row_thr_mv,
                        is_pressed);
                d->callback(dev, row, col, is_pressed);
            }

            any_active = any_active || zmk_debounce_is_active(&d->state[i]);
        }
    }

    if (any_active) {
        schedule_read_continue(dev);
    } else {
        schedule_read_end(dev);
    }

    return 0;
}

static void work_handler(struct k_work *w) {
    struct fw16_data *d = CONTAINER_OF(k_work_delayable_from_work(w), struct fw16_data, work);
    scan_once(d->dev);
}

static int fw16_configure(const struct device *dev, kscan_callback_t callback) {
    struct fw16_data *d = dev->data;

    if (!callback) {
        return -EINVAL;
    }

    d->callback = callback;
    return 0;
}

static int fw16_enable(const struct device *dev) {
    struct fw16_data *d = dev->data;
    const struct fw16_cfg *c = dev->config;

    d->scan_time = k_uptime_get();
    return k_work_reschedule(&d->work, K_MSEC(c->poll_period_ms));
}

static int fw16_disable(const struct device *dev) {
    struct fw16_data *d = dev->data;
    return k_work_cancel_delayable(&d->work);
}

static int fw16_init(const struct device *dev) {
    struct fw16_data *d = dev->data;
    const struct fw16_cfg *c = dev->config;

    d->dev = dev;
    k_work_init_delayable(&d->work, work_handler);

    /* Configure columns as outputs (inactive high) */
    for (size_t i = 0; i < c->col_cnt; i++) {
        if (!device_is_ready(c->cols[i].port)) {
            return -ENODEV;
        }
        int err = gpio_pin_configure_dt(&c->cols[i], GPIO_OUTPUT);
        if (err) {
            return err;
        }
        err = gpio_pin_set_dt(&c->cols[i], 1);
        if (err) {
            return err;
        }
    }

    /* Configure mux selects and enable */
    for (int i = 0; i < 3; i++) {
        if (!device_is_ready(c->mux[i].port)) {
            return -ENODEV;
        }
        int err = gpio_pin_configure_dt(&c->mux[i], GPIO_OUTPUT);
        if (err) {
            return err;
        }
    }

    if (!device_is_ready(c->mux_en.port)) {
        return -ENODEV;
    }
    int err = gpio_pin_configure_dt(&c->mux_en, GPIO_OUTPUT);
    if (err) {
        return err;
    }

    /* Active-low: drive low to enable MUX */
    err = gpio_pin_set_dt(&c->mux_en, 0);
    if (err) {
        return err;
    }

    /* Optionally bias ADC pad (e.g. pull-up) to avoid floating input */
    if (c->adc_pad.port) {
        if (!device_is_ready(c->adc_pad.port)) {
            return -ENODEV;
        }
        err = gpio_pin_configure_dt(&c->adc_pad, GPIO_INPUT);
        if (err) {
            return err;
        }
    }

    return 0;
}

static const struct kscan_driver_api fw16_api = {
    .config = fw16_configure,
    .enable_callback = fw16_enable,
    .disable_callback = fw16_disable,
};

/*
 * Device instantiation from DT
 */
#define FW16_COL_ELEM(n, prop, idx) GPIO_DT_SPEC_GET_BY_IDX(n, prop, idx),

#define FW16_INST(n)                                                                               \
    static const struct gpio_dt_spec fw16_cols_##n[] = {                                           \
        DT_FOREACH_PROP_ELEM(DT_DRV_INST(n), col_gpios, FW16_COL_ELEM)                             \
    };                                                                                             \
                                                                                                   \
    static const struct fw16_cfg fw16_cfg_##n = {                                                  \
        .cols = fw16_cols_##n,                                                                     \
        .col_cnt = ARRAY_SIZE(fw16_cols_##n),                                                      \
        .mux = {                                                                                   \
            GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), mux_gpios, 0),                                 \
            GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), mux_gpios, 1),                                 \
            GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), mux_gpios, 2),                                 \
        },                                                                                         \
        .mux_en = GPIO_DT_SPEC_GET(DT_DRV_INST(n), mux_enable_gpios),                              \
        .adc_pad = COND_CODE_1(DT_NODE_HAS_PROP(DT_DRV_INST(n), adc_gpios),                        \
                               (GPIO_DT_SPEC_GET(DT_DRV_INST(n), adc_gpios)),                      \
                               ({0})),                                                             \
        .adc_dev = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR_BY_IDX(DT_DRV_INST(n), 0)),                   \
        .adc_channel_id = DT_IO_CHANNELS_INPUT_BY_IDX(DT_DRV_INST(n), 0),                          \
        .rows = DT_INST_PROP(n, rows),                                                             \
        .cols_count = DT_INST_PROP(n, columns),                                                    \
        .debounce_config = {                                                                       \
            .debounce_press_ms = DT_INST_PROP_OR(n, debounce_press_ms, 5),                         \
            .debounce_release_ms = DT_INST_PROP_OR(n, debounce_release_ms, 5),                     \
        },                                                                                         \
        .debounce_scan_period_ms = DT_INST_PROP_OR(n, debounce_scan_period_ms, 1),                 \
        .poll_period_ms = DT_INST_PROP_OR(n, poll_period_ms, 15),                                  \
    };                                                                                             \
                                                                                                   \
    static struct zmk_debounce_state fw16_state_##n[DT_INST_PROP(n, rows) *                        \
                                                    DT_INST_PROP(n, columns)];                     \
                                                                                                   \
    static struct fw16_data fw16_data_##n = {                                                      \
        .state = fw16_state_##n,                                                                   \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, fw16_init, NULL, &fw16_data_##n, &fw16_cfg_##n, POST_KERNEL,          \
                          CONFIG_KSCAN_INIT_PRIORITY, &fw16_api);

DT_INST_FOREACH_STATUS_OKAY(FW16_INST)
