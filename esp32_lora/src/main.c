#include <zephyr/kernel.h>  
#include <zephyr/logging/log.h>  
#include <zephyr/drivers/lora.h>  
#include <zephyr/drivers/gpio.h>  
#include <zephyr/device.h>  
#include <zephyr/display/cfb.h>  
#include <zephyr/sys/printk.h>  
#include <zephyr/sys/util.h>  
#include <zephyr/sys/sys_heap.h>  
#include <zephyr/sys/byteorder.h>
#include <zephyr/console/console.h>


LOG_MODULE_REGISTER(IoTProj, LOG_LEVEL_INF);
  
/* ---------- Config ---------- */  
#define STACK_SIZE   1024  
#define PRIORITY_TX  4  
#define PRIORITY_RX  5  
#define PRIORITY_DISP 6  
#define PRIORITY_EVT 3  
#define PRIORITY_MON 2  
#define PRIORITY_SER 2
  
#define BTN_NODE      DT_ALIAS(sw0)  
#define DISPLAY_NODE  DT_CHOSEN(zephyr_display)  
#define LORA_NODE     DT_ALIAS(lora0)  
  
/* ---------- Structuri ---------- */  
struct event_t {  
	uint32_t ts_ms;  
	uint8_t  id;   /* id buton, 0,1… */  
};  
  
static struct lora_modem_config cfg_tx = {  
        .frequency      = 865100000,  
        .bandwidth      = BW_125_KHZ,  
        .datarate       = SF_10,  
        .preamble_len   = 8,  
        .coding_rate    = CR_4_5,  
        .iq_inverted    = false,  
        .public_network = false,  
        .tx_power       = 4,  
        .tx             = true,          /* <-- TX mode */  
};  
  
static struct lora_modem_config cfg_rx = {  
        .frequency      = 865100000,  
        .bandwidth      = BW_125_KHZ,  
        .datarate       = SF_10,  
        .preamble_len   = 8,  
        .coding_rate    = CR_4_5,  
        .iq_inverted    = false,  
        .public_network = false,  
        .tx_power       = 4,             /* nu contează în RX */  
        .tx             = false,         /* <-- RX mode */  
};

K_MSGQ_DEFINE(events_q, sizeof(struct event_t), 16, 4);  
K_SEM_DEFINE(tx_sem, 0, 1);  
K_MUTEX_DEFINE(disp_mutex);  
K_MUTEX_DEFINE(lora_lock);
  
/* forward decl */  
static void lora_rx_cb(const struct device *, uint8_t *, uint16_t,  
		       int16_t, int8_t);  
  
static const struct device *lora_dev;  
static const struct device *display_dev;  
  
/* ---------- Statistici ---------- */  
struct stats_t {  
	uint32_t btn_cnt;  
	uint32_t tx_cnt;  
	uint32_t rx_cnt;  
	size_t   free_heap;  
	size_t   stack_free[5];  
};  
static struct stats_t g_stats;  
  
/* ---------- ISR Buton ---------- */  
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(BTN_NODE, gpios);  
  
static void button_isr(const struct device *dev, struct gpio_callback *cb,  
		       uint32_t pins)  
{  
	static struct event_t ev;  
	ev.id = 0;  
	ev.ts_ms = k_uptime_get_32();  
	k_msgq_put(&events_q, &ev, K_NO_WAIT);  
}  
  
static struct gpio_callback btn_cb;  

void event_thread(void);
void lora_rx_thread(void);
void lora_tx_thread(void);
void display_thread(void);
void monitor_thread(void);
void serial_thread(void);

/* ---------- Thread definitions ---------- */  
K_THREAD_DEFINE(tid_evt,  STACK_SIZE, event_thread,  NULL, NULL, NULL,  
		PRIORITY_EVT, 0, 0);  
K_THREAD_DEFINE(tid_tx,   STACK_SIZE, lora_tx_thread,   NULL, NULL, NULL,  
		PRIORITY_TX,  0, 0);  
K_THREAD_DEFINE(tid_rx,   STACK_SIZE, lora_rx_thread, NULL, NULL, NULL,  
                PRIORITY_RX, 0, 0);   
K_THREAD_DEFINE(tid_disp, STACK_SIZE, display_thread,NULL, NULL, NULL,  
		PRIORITY_DISP,0, 0);  
K_THREAD_DEFINE(tid_mon,  STACK_SIZE, monitor_thread,NULL, NULL, NULL,  
		PRIORITY_MON, 0, 0);  
K_THREAD_DEFINE(tid_ser,  STACK_SIZE, serial_thread, NULL, NULL, NULL,
		PRIORITY_SER, 0, 0);

/* ---------- TASK 1 – EVENT_DISPATCH ---------- */  
void event_thread(void)  
{  
	struct event_t ev;  
	for (;;) {  
		k_msgq_get(&events_q, &ev, K_FOREVER);  
		k_mutex_lock(&disp_mutex, K_FOREVER);  
		g_stats.btn_cnt++;  
		k_mutex_unlock(&disp_mutex);  
        LOG_INF("Signal");  
		k_sem_give(&tx_sem);   
	}  
}  
  
void lora_rx_thread(void)  
{  
    // lora_dev = DEVICE_DT_GET(LORA_NODE);  
    if (!device_is_ready(lora_dev)) {  
        LOG_ERR("LoRa device not ready!");  
        return;  
    }  
    for (;;) {    
        k_mutex_lock(&lora_lock, K_FOREVER); 
 
        lora_recv_async(lora_dev, NULL);  

        while (lora_config(lora_dev, &cfg_rx) < 0) {
            LOG_ERR("LoRa rx config failed");
            k_sleep(K_MSEC(100));  
        }

        if (lora_recv_async(lora_dev, lora_rx_cb) == 0) {  
            k_mutex_unlock(&lora_lock);  
            k_sleep(K_FOREVER);  
            // LOG_INF("Morning %d",lora_lock.lock_count);  
        } else {   
            k_mutex_unlock(&lora_lock);  
            k_sleep(K_MSEC(200));  
            LOG_ERR("LoRa Rx not ready! %d",lora_lock.lock_count);  
        }  
    } 
} 

/* ---------- TASK 2 – LORA_TX_RX ---------- */  
void lora_tx_thread(void)  
{  
	uint8_t frame[8];  
  
	lora_dev = DEVICE_DT_GET(LORA_NODE);  
	if (!device_is_ready(lora_dev)) {  
		LOG_ERR("LoRa not ready");  
		return;  
	}  
	lora_config(lora_dev, &cfg_tx);  
  
	for (;;) {  
        LOG_INF("Wait %d",lora_lock.lock_count);  
		k_sem_take(&tx_sem, K_FOREVER);  
        k_mutex_lock(&lora_lock, K_FOREVER);  
        
        lora_recv_async(lora_dev, NULL);  
        while (lora_config(lora_dev, &cfg_tx) < 0) {
            LOG_ERR("LoRa tx config failed");
            k_sleep(K_MSEC(100));  
        }

        uint32_t uptime = k_uptime_get_32();
		sys_put_le32(g_stats.tx_cnt, &frame[0]);  
		sys_put_le32(uptime, &frame[4]);  

        LOG_INF("Send");  

        lora_recv_async(lora_dev, NULL);  

        int status = lora_send(lora_dev, frame, sizeof(frame));
		if (status == 0) {  
			LOG_INF("TX %u (Frame Hex) @%u", g_stats.tx_cnt, uptime);
            LOG_HEXDUMP_INF(frame, sizeof(frame), "Packet Data:");
            k_mutex_lock(&disp_mutex, K_FOREVER);  
            g_stats.tx_cnt++;  
            k_mutex_unlock(&disp_mutex); 
		}  else {
            LOG_ERR("LoRa not sent %d", status);  
        }
        k_mutex_unlock(&lora_lock);
        k_wakeup(tid_rx);
	}  
}  
  
/* callback RX – rulează în contextul driverului, NU în ISR hard */  
static void lora_rx_cb(const struct device *dev, uint8_t *data, uint16_t len,  
		       int16_t rssi, int8_t snr)  
{  
	LOG_INF("RX pkt, RSSI=%d", rssi);  
	k_mutex_lock(&disp_mutex, K_NO_WAIT);  
	g_stats.rx_cnt++;  
	k_mutex_unlock(&disp_mutex);  
    k_wakeup(tid_rx);
}  
  
/* ---------- TASK 3 – DISPLAY_UPDATE ---------- */  
void display_thread(void)  
{  
	display_dev = DEVICE_DT_GET(DISPLAY_NODE);  
	cfb_framebuffer_init(display_dev);  
  
	char buf[32];  
  
	for (;;) {  
		k_sleep(K_MSEC(100));  
  
		k_mutex_lock(&disp_mutex, K_FOREVER);  
		cfb_framebuffer_clear(display_dev, true);  
  
		snprintk(buf, sizeof(buf), "TX:%u RX:%u",  
			 g_stats.tx_cnt, g_stats.rx_cnt);  
		cfb_print(display_dev, buf, 0, 0);  
		snprintk(buf, sizeof(buf), "BTN:%u",  
			 g_stats.btn_cnt);  
		cfb_print(display_dev, buf, 0, 16);  
  
		snprintk(buf, sizeof(buf), "Heap:%u", g_stats.free_heap);  
		cfb_print(display_dev, buf, 0, 32);  
		cfb_framebuffer_finalize(display_dev);  
		k_mutex_unlock(&disp_mutex);  
	}  
}  
  
/* ---------- TASK 4 – RESOURCE_MON ---------- */  
void monitor_thread(void)  
{  
    size_t unused;  
  
    for (;;) {  
        k_sleep(K_SECONDS(1));    
        k_wakeup(tid_rx);
        k_thread_stack_space_get(k_current_get(), &unused);  
  
        k_mutex_lock(&disp_mutex, K_NO_WAIT);  
        g_stats.stack_free[0] = unused;  
        k_mutex_unlock(&disp_mutex);  
    }  
}  

/* ---------- TASK 5 – SERIAL_COMMAND ---------- */
void serial_thread(void)
{
	if (console_init() != 0) {
		LOG_ERR("Console init failed");
		return;
	}

	for (;;) {
		uint8_t c = console_getchar();
		if (c == 's') {
			LOG_INF("Serial command 's' received. Sending 5 packets...");
			for (int i = 0; i < 5; i++) {
				k_sem_give(&tx_sem);
				k_sleep(K_SECONDS(1));
			}
			LOG_INF("Serial sequence finished.");
		}
	}
}
  

  
/* ---------- main() ---------- */  
int main(void)  
{  
    lora_dev = DEVICE_DT_GET(LORA_NODE);  
    if (!device_is_ready(lora_dev)) {  
        LOG_ERR("LoRa device not ready!");  
        return 0;  
    }  
	/* configure button GPIO + interrupt */  
	gpio_pin_configure_dt(&btn, GPIO_INPUT);  
	gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);  
	gpio_init_callback(&btn_cb, button_isr, BIT(btn.pin));  
	gpio_add_callback(btn.port, &btn_cb);  
  
	LOG_INF("LoRa Event Logger started");  
	return 0;  
}  
