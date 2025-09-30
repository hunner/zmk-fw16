/* ZMK module: Framework 16 kscan driver */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/debounce.h>

LOG_MODULE_REGISTER(kscan_fw16, LOG_LEVEL_DBG);
#define DT_DRV_COMPAT zmk_kscan_fw16

struct fw16_cfg {
    const struct gpio_dt_spec *cols; size_t col_cnt;
    struct gpio_dt_spec mux[3]; struct gpio_dt_spec mux_en;
    struct gpio_dt_spec adc_pad; /* optional: to enable pull-up on ADC pin (adc-gpios) */
    const struct device *adc_dev; uint8_t adc_channel_id; uint8_t rows; uint8_t cols_count;
    struct zmk_debounce_config debounce_config; int32_t debounce_scan_period_ms; int32_t poll_period_ms;
};
struct fw16_data { const struct device *dev; kscan_callback_t callback; struct k_work_delayable work; int64_t scan_time; struct zmk_debounce_state *state; };
static inline int idx_rc(const struct fw16_cfg *cfg, int r, int c){ return (c*cfg->rows)+r; }
static inline uint8_t mux_sel_for_row(uint8_t r){ switch(r){case 0:return 2;case 1:return 0;case 2:return 1;default:return r;} }

static int set_mux(const struct fw16_cfg *cfg,uint8_t r){ uint8_t i=mux_sel_for_row(r)&7; int e=0; e|=gpio_pin_set_dt(&cfg->mux[0], (i&1)); e|=gpio_pin_set_dt(&cfg->mux[1], (i&2)?1:0); e|=gpio_pin_set_dt(&cfg->mux[2], (i&4)?1:0); return e; }
static int adc_read_mv(const struct fw16_cfg *cfg,int32_t *out){ struct adc_channel_cfg ch={.gain=ADC_GAIN_1,.reference=ADC_REF_INTERNAL,.acquisition_time=ADC_ACQ_TIME_DEFAULT,.channel_id=cfg->adc_channel_id}; int e=adc_channel_setup(cfg->adc_dev,&ch); if(e)return e; uint16_t raw=0; struct adc_sequence seq={.buffer=&raw,.buffer_size=sizeof(raw),.channels=BIT(cfg->adc_channel_id),.resolution=12}; e=adc_read(cfg->adc_dev,&seq); if(e)return e; int32_t mv=raw; int32_t vref=adc_ref_internal(cfg->adc_dev); if(vref>0){ if(adc_raw_to_millivolts(vref,ch.gain,seq.resolution,&mv)<0) mv=raw;} *out=mv; return 0; }
static void read_continue(const struct device *dev){ struct fw16_data *d=dev->data; const struct fw16_cfg *c=dev->config; d->scan_time+=c->debounce_scan_period_ms; k_work_reschedule(&d->work,K_TIMEOUT_ABS_MS(d->scan_time)); }
static void read_end(const struct device *dev){ struct fw16_data *d=dev->data; const struct fw16_cfg *c=dev->config; d->scan_time+=c->poll_period_ms; k_work_reschedule(&d->work,K_TIMEOUT_ABS_MS(d->scan_time)); }
static int scan_once(const struct device *dev){
    struct fw16_data *d = dev->data;
    const struct fw16_cfg *c = dev->config;

    /* Drive all columns inactive (high) before scan */
    for (int col = 0; col < c->cols_count; col++) {
        gpio_pin_set_dt(&c->cols[col], 1);
    }

    /* Capture readings */
    int16_t mv[8][16] = {0};
    for (int col = 0; col < c->cols_count; col++) {
        int e = gpio_pin_set_dt(&c->cols[col], 0); /* active: sink */
        if (e) return e;
        k_busy_wait(100); /* allow column to settle */

        for (int row = 0; row < c->rows; row++) {
            set_mux(c, row);
            k_busy_wait(30); /* mux settle */
            int32_t v = 0;
            e = adc_read_mv(c, &v);
            if (e) return e;
            mv[row][col] = (int16_t)v;
        }

        e = gpio_pin_set_dt(&c->cols[col], 1); /* inactive */
        if (e) return e;
    }

    /* Dynamic threshold: adjust based on number of low readings in a row */
    bool cont = false;
    for (int row = 0; row < c->rows; row++) {
        int pressed_in_row = 0;
        /* Heuristic: only count as "low" when below ~1.0V to avoid noisy mid-level rows */
        for (int col = 0; col < c->cols_count; col++) {
            if (mv[row][col] < 1000) {
                pressed_in_row++;
            }
        }
        if (pressed_in_row > 0) {
            LOG_DBG("fw16: row %d voltages mV: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                   row,
                   mv[row][0], mv[row][1], mv[row][2], mv[row][3], mv[row][4], mv[row][5], mv[row][6], mv[row][7],
                   mv[row][8], mv[row][9], mv[row][10], mv[row][11], mv[row][12], mv[row][13], mv[row][14], mv[row][15]);
        }
        for (int col = 0; col < c->cols_count; col++) {
            int32_t thr_mv;
            switch (pressed_in_row) {
                case 0:
                case 1:
                    thr_mv = 1000; /* 1.0V */
                    break;
                case 2:
                    thr_mv = 2000; /* 2.0V */
                    break;
                case 3:
                    thr_mv = 2500; /* 2.5V */
                    break;
                default:
                    thr_mv = 2900; /* 2.9V */
                    break;
            }
            bool a = (mv[row][col] < thr_mv);
            int i = idx_rc(c, row, col);
            zmk_debounce_update(&d->state[i], a, c->debounce_scan_period_ms, &c->debounce_config);
            if (zmk_debounce_get_changed(&d->state[i])) {
                bool p = zmk_debounce_is_pressed(&d->state[i]);
                LOG_DBG("fw16: rc(%d,%d) mv=%d thr=%d pressed=%d", row, col, mv[row][col], thr_mv, p);
                d->callback(dev, row, col, p);
            }
            cont = cont || zmk_debounce_is_active(&d->state[i]);
        }
    }

    if (cont) read_continue(dev); else read_end(dev);
    return 0;
}
static void work_handler(struct k_work *w){ struct fw16_data *d=CONTAINER_OF(k_work_delayable_from_work(w),struct fw16_data,work); scan_once(d->dev); }
static int conf(const struct device *dev, kscan_callback_t cb){ struct fw16_data *d=dev->data; if(!cb) return -EINVAL; d->callback=cb; return 0; }
static int en(const struct device *dev){ struct fw16_data *d=dev->data; const struct fw16_cfg *c=dev->config; d->scan_time=k_uptime_get(); k_work_reschedule(&d->work,K_MSEC(c->poll_period_ms)); return 0; }
static int dis(const struct device *dev){ struct fw16_data *d=dev->data; k_work_cancel_delayable(&d->work); return 0; }
static int init(const struct device *dev){
    struct fw16_data *d=dev->data; const struct fw16_cfg *c=dev->config; d->dev=dev;
    k_work_init_delayable(&d->work,work_handler);
    /* Configure columns as outputs (inactive high) */
    for(size_t i=0;i<c->col_cnt;i++){
        if(!device_is_ready(c->cols[i].port)) return -ENODEV;
        int e=gpio_pin_configure_dt(&c->cols[i],GPIO_OUTPUT);
        if(e) return e;
        gpio_pin_set_dt(&c->cols[i],1);
    }
    /* Configure mux selects and enable */
    for(int i=0;i<3;i++){
        if(!device_is_ready(c->mux[i].port)) return -ENODEV;
        int e=gpio_pin_configure_dt(&c->mux[i],GPIO_OUTPUT);
        if(e) return e;
    }
    if(!device_is_ready(c->mux_en.port)) return -ENODEV;
    int e=gpio_pin_configure_dt(&c->mux_en,GPIO_OUTPUT);
    if(e) return e;
    gpio_pin_set_dt(&c->mux_en,0); /* active-low enable */
    /* Optionally bias ADC pad pull-up to avoid floating input */
    if (c->adc_pad.port) {
        if(!device_is_ready(c->adc_pad.port)) return -ENODEV;
        /* Merge any flags from DT (e.g. GPIO_PULL_UP) */
        e = gpio_pin_configure_dt(&c->adc_pad, GPIO_INPUT);
        if(e) return e;
    }
    return 0;
}
static const struct kscan_driver_api api={ .config=conf,.enable_callback=en,.disable_callback=dis };
#define FW16_COL_ELEM(n, prop, idx) GPIO_DT_SPEC_GET_BY_IDX(n, prop, idx),
#define FW16_INST(n) \
    static const struct gpio_dt_spec fw16_cols_##n[] = { \
        DT_FOREACH_PROP_ELEM(DT_DRV_INST(n), col_gpios, FW16_COL_ELEM) \
    }; \
    static const struct fw16_cfg fw16_cfg_##n={ .cols=fw16_cols_##n,.col_cnt=ARRAY_SIZE(fw16_cols_##n), .mux={ GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n),mux_gpios,0), GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n),mux_gpios,1), GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n),mux_gpios,2), }, .mux_en=GPIO_DT_SPEC_GET(DT_DRV_INST(n),mux_enable_gpios), .adc_pad=COND_CODE_1(DT_NODE_HAS_PROP(DT_DRV_INST(n), adc_gpios), (GPIO_DT_SPEC_GET(DT_DRV_INST(n), adc_gpios)), ({0})), .adc_dev=DEVICE_DT_GET(DT_IO_CHANNELS_CTLR_BY_IDX(DT_DRV_INST(n),0)), .adc_channel_id=DT_IO_CHANNELS_INPUT_BY_IDX(DT_DRV_INST(n),0), .rows=DT_INST_PROP(n,rows), .cols_count=DT_INST_PROP(n,columns), .debounce_config={ .debounce_press_ms=DT_INST_PROP_OR(n,debounce_press_ms,5), .debounce_release_ms=DT_INST_PROP_OR(n,debounce_release_ms,5), }, .debounce_scan_period_ms=DT_INST_PROP_OR(n,debounce_scan_period_ms,1), .poll_period_ms=DT_INST_PROP_OR(n,poll_period_ms,15), }; \
    static struct zmk_debounce_state fw16_state_##n[DT_INST_PROP(n,rows)*DT_INST_PROP(n,columns)]; \
    static struct fw16_data fw16_data_##n={ .state=fw16_state_##n, }; \
    DEVICE_DT_INST_DEFINE(n, init, NULL, &fw16_data_##n, &fw16_cfg_##n, POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY, &api);
DT_INST_FOREACH_STATUS_OKAY(FW16_INST)
