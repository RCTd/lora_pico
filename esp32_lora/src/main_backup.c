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
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/dns_resolve.h>
#include <zephyr/posix/fcntl.h>

LOG_MODULE_REGISTER(IoTProj, LOG_LEVEL_INF);

/* ---------- Wi-Fi & TTN Config ---------- */
#define WIFI_SSID "DIGI-gU2b"
#define WIFI_PSK  "bC9yCPkUbz"
#define TTN_SERVER "eu1.cloud.thethings.network"
#define TTN_PORT   "1700"

/* ---------- Config ---------- */  
#define STACK_SIZE   2048  
#define PRIORITY_TX  4  
#define PRIORITY_RX  5  
#define PRIORITY_FWD 5
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

struct lora_packet_t {
	uint8_t data[256];
	uint16_t len;
	int16_t rssi;
	int8_t snr;
	uint32_t freq;
	uint8_t sf;
	uint8_t bw;
};

static struct lora_modem_config cfg_tx = {  
        .frequency      = 864920000,  
        .bandwidth      = BW_125_KHZ,  
        .datarate       = SF_10,  
        .preamble_len   = 12,  
        .coding_rate    = CR_4_8,  
        .iq_inverted    = false,  
        .public_network = false,  
        .tx_power       = 4,  
        .tx             = true,  
};  
  
static struct lora_modem_config cfg_rx = {  
        .frequency      = 864920000,  
        .bandwidth      = BW_125_KHZ,  
        .datarate       = SF_10,  
        .preamble_len   = 12,  
        .coding_rate    = CR_4_8,  
        .iq_inverted    = false,  
        .public_network = false,  
        .tx             = false,  
};

K_MSGQ_DEFINE(events_q, sizeof(struct event_t), 16, 4);  
K_MSGQ_DEFINE(fwd_q, sizeof(struct lora_packet_t), 8, 4);
K_SEM_DEFINE(tx_sem, 0, 1);  
K_SEM_DEFINE(wifi_connected_sem, 0, 1);
K_MUTEX_DEFINE(disp_mutex);  
K_MUTEX_DEFINE(lora_lock);
  
static void lora_rx_cb(const struct device *, uint8_t *, uint16_t, int16_t, int8_t);  
  
static const struct device *lora_dev;  
static const struct device *display_dev;  
static uint8_t g_gw_eui[8];

struct stats_t {  
	uint32_t btn_cnt;  
	uint32_t tx_cnt;  
	uint32_t rx_cnt;  
	uint32_t fwd_cnt;
	uint32_t pull_ack_cnt;
	bool wifi_ok;
};  
static struct stats_t g_stats;  
  
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(BTN_NODE, gpios);  
  
static void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)  
{  
	static struct event_t ev;  
	ev.id = 0;  
	ev.ts_ms = k_uptime_get_32();  
	k_msgq_put(&events_q, &ev, K_NO_WAIT);  
}  
static struct gpio_callback btn_cb;  

/* ---------- Base64 Helper ---------- */
static const char base64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static void base64_encode(const uint8_t *src, size_t len, char *dst) {
	while (len > 0) {
		*dst++ = base64_table[src[0] >> 2];
		if (len > 1) {
			*dst++ = base64_table[((src[0] & 0x03) << 4) | (src[1] >> 4)];
			if (len > 2) {
				*dst++ = base64_table[((src[1] & 0x0f) << 2) | (src[2] >> 6)];
				*dst++ = base64_table[src[2] & 0x3f];
				len -= 3; src += 3;
			} else {
				*dst++ = base64_table[(src[1] & 0x0f) << 2];
				*dst++ = '=';
				len = 0;
			}
		} else {
			*dst++ = base64_table[(src[0] & 0x03) << 4];
			*dst++ = '=';
			*dst++ = '=';
			len = 0;
		}
	}
	*dst = '\0';
}

/* ---------- Wi-Fi Manager ---------- */
static struct net_mgmt_event_callback wifi_cb;
static void wifi_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface) {
	if (mgmt_event == NET_EVENT_WIFI_CONNECT_RESULT) {
		const struct wifi_status *status = (const struct wifi_status *)cb->info;
		if (status && status->status) {
			LOG_ERR("Connection failed: %d", status->status);
		} else {
			LOG_INF("Wi-Fi Connected. Starting DHCP...");
			net_dhcpv4_start(iface);
		}
	} else if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD) {
		LOG_INF("IP Address Assigned");
		g_stats.wifi_ok = true;
		k_sem_give(&wifi_connected_sem);
	}
}

void connect_wifi(void) {
	struct net_if *iface = net_if_get_default();
	struct wifi_connect_req_params params = {
		.ssid = (uint8_t *)WIFI_SSID,
		.ssid_length = strlen(WIFI_SSID),
		.psk = (uint8_t *)WIFI_PSK,
		.psk_length = strlen(WIFI_PSK),
		.channel = WIFI_CHANNEL_ANY,
		.security = WIFI_SECURITY_TYPE_PSK,
	};
	net_mgmt_init_event_callback(&wifi_cb, wifi_handler, NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&wifi_cb);
	
	LOG_INF("Waiting 5s for Wi-Fi driver to settle...");
	k_sleep(K_SECONDS(5));
	
	LOG_INF("Connecting to Wi-Fi: %s", WIFI_SSID);
	if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params))) {
		LOG_ERR("Wi-Fi Connection Request Failed");
	}
}

/* Forward thread handles */
void event_thread(void);
void lora_rx_thread(void);
void lora_tx_thread(void);
void lora_fwd_thread(void);
void display_thread(void);
void monitor_thread(void);
void serial_thread(void);

K_THREAD_DEFINE(tid_evt,  1024, event_thread,  NULL, NULL, NULL, PRIORITY_EVT, 0, 0);  
K_THREAD_DEFINE(tid_tx,   1024, lora_tx_thread,   NULL, NULL, NULL, PRIORITY_TX,  0, 0);  
K_THREAD_DEFINE(tid_rx,   1024, lora_rx_thread, NULL, NULL, NULL, PRIORITY_RX, 0, 0);   
K_THREAD_DEFINE(tid_fwd,  4096, lora_fwd_thread, NULL, NULL, NULL, PRIORITY_FWD, 0, 0);
K_THREAD_DEFINE(tid_disp, 1024, display_thread,NULL, NULL, NULL, PRIORITY_DISP,0, 0);  
K_THREAD_DEFINE(tid_mon,  1024, monitor_thread,NULL, NULL, NULL, PRIORITY_MON, 0, 0);  
// K_THREAD_DEFINE(tid_ser,  1024, serial_thread, NULL, NULL, NULL, PRIORITY_SER, 0, 0);

/* ---------- TASK 1 – EVENT_DISPATCH ---------- */  
void event_thread(void) {  
	struct event_t ev;  
	for (;;) {  
		k_msgq_get(&events_q, &ev, K_FOREVER);  
		k_mutex_lock(&disp_mutex, K_FOREVER);  
		g_stats.btn_cnt++;  
		k_mutex_unlock(&disp_mutex);  
		k_sem_give(&tx_sem);   
	}  
}  
  
void lora_rx_thread(void) {  
	if (!device_is_ready(lora_dev)) return;  
	for (;;) {    
		k_mutex_lock(&lora_lock, K_FOREVER); 
		lora_recv_async(lora_dev, NULL);  
		while (lora_config(lora_dev, &cfg_rx) < 0) k_sleep(K_MSEC(100));  
		if (lora_recv_async(lora_dev, lora_rx_cb) == 0) {  
			k_mutex_unlock(&lora_lock);  
			k_sleep(K_FOREVER);  
		} else {   
			k_mutex_unlock(&lora_lock);  
			k_sleep(K_MSEC(200));  
		}  
	} 
} 

/* ---------- TASK 2 – LORA_TX ---------- */  
void lora_tx_thread(void) {  
	uint8_t frame[8];  
	for (;;) {  
		k_sem_take(&tx_sem, K_FOREVER);  
		k_mutex_lock(&lora_lock, K_FOREVER);  
		lora_recv_async(lora_dev, NULL);  
		while (lora_config(lora_dev, &cfg_tx) < 0) k_sleep(K_MSEC(100));  
		sys_put_le32(g_stats.tx_cnt, &frame[0]);  
		if (lora_send(lora_dev, frame, sizeof(frame)) == 0) {  
			g_stats.tx_cnt++;  
		}
		k_mutex_unlock(&lora_lock);
		k_wakeup(tid_rx);
	}  
}  
  
static void lora_rx_cb(const struct device *dev, uint8_t *data, uint16_t len, int16_t rssi, int8_t snr) {  
	struct lora_packet_t pkt;
	pkt.len = len;
	pkt.rssi = rssi;
	pkt.snr = snr;
	pkt.freq = cfg_rx.frequency;
	pkt.sf = 10; // match cfg_rx
	pkt.bw = 125;
	memcpy(pkt.data, data, len);
	k_msgq_put(&fwd_q, &pkt, K_NO_WAIT);

	k_mutex_lock(&disp_mutex, K_NO_WAIT);  
	g_stats.rx_cnt++;  
	k_mutex_unlock(&disp_mutex);  
	k_wakeup(tid_rx);
}

/* ---------- TASK 3 – TTN FORWARDER ---------- */
void lora_fwd_thread(void) {
	k_sem_take(&wifi_connected_sem, K_FOREVER);
	
	struct zsock_addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_DGRAM,
		.ai_protocol = IPPROTO_UDP,
	};
	struct zsock_addrinfo *res;
	int ret;

	printk("Resolving TTN Server: %s\n", TTN_SERVER);
	ret = zsock_getaddrinfo(TTN_SERVER, TTN_PORT, &hints, &res);
	if (ret != 0) {
		printk("DNS Resolution Failed: %d\n", ret);
		return;
	}

	char addr_str[INET_ADDRSTRLEN];
	zsock_inet_ntop(AF_INET, &((struct sockaddr_in *)res->ai_addr)->sin_addr, addr_str, sizeof(addr_str));
	printk("TTN Resolved to %s\n", addr_str);

	int sock = zsock_socket(res->ai_family, res->ai_socktype, res->ai_protocol);
	if (sock < 0) {
		printk("Socket creation failed: %d\n", errno);
		zsock_freeaddrinfo(res);
		return;
	}

	/* Set socket non-blocking for ACK checks */
	int flags = zsock_fcntl(sock, F_GETFL, 0);
	zsock_fcntl(sock, F_SETFL, flags | O_NONBLOCK);

	uint8_t buffer[1024];
	struct lora_packet_t pkt;
	uint32_t last_pull = 0;

	for (;;) {
		uint32_t now = k_uptime_get_32();
		
		/* Heartbeat / PULL_DATA */
		if (now - last_pull > 10000) {
			buffer[0] = 0x02; // Version
			sys_put_be16(k_cycle_get_32() & 0xFFFF, &buffer[1]); // Token
			buffer[3] = 0x02; // PULL_DATA
			memcpy(&buffer[4], g_gw_eui, 8);
			
			ret = zsock_sendto(sock, buffer, 12, 0, res->ai_addr, res->ai_addrlen);
			if (ret < 0) {
				printk("PULL_DATA failed: %d\n", errno);
			} else {
				// printk("PULL_DATA sent\n");
			}
			last_pull = now;
		}

		/* Check for incoming ACKs or data */
		ret = zsock_recvfrom(sock, buffer, sizeof(buffer), 0, NULL, NULL);
		if (ret > 0) {
			if (buffer[3] == 0x04) { // PULL_ACK
				printk("TTN Connection Verified (PULL_ACK received)\n");
				g_stats.pull_ack_cnt++;
			} else if (buffer[3] == 0x01) { // PUSH_ACK
				printk("PUSH_ACK received\n");
			}
		}

		/* Process and Forward received LoRa packets */
		if (k_msgq_get(&fwd_q, &pkt, K_MSEC(500)) == 0) {
			char b64[400]; base64_encode(pkt.data, pkt.len, b64);
			char json[800];
			snprintk(json, sizeof(json), "{\"rxpk\":[{\"tmst\":%u,\"freq\":%.3f,\"chan\":0,\"rfch\":0,\"stat\":1,\"modu\":\"LORA\",\"datr\":\"SF%uBW%u\",\"codr\":\"4/8\",\"rssi\":%d,\"lsnr\":%.1f,\"size\":%u,\"data\":\"%s\"}]}",
				now, (double)pkt.freq/1e6, pkt.sf, pkt.bw, pkt.rssi, (double)pkt.snr, pkt.len, b64);
			
			buffer[0] = 0x02;
			sys_put_be16(k_cycle_get_32() & 0xFFFF, &buffer[1]);
			buffer[3] = 0x00; // PUSH_DATA
			memcpy(&buffer[4], g_gw_eui, 8);
			int header_len = 12;
			int json_len = strlen(json);
			memcpy(&buffer[header_len], json, json_len);
			
			ret = zsock_sendto(sock, buffer, header_len + json_len, 0, res->ai_addr, res->ai_addrlen);
			if (ret < 0) {
				printk("Forwarding failed: %d\n", errno);
			} else {
				printk("LoRa Packet forwarded to TTN\n");
				g_stats.fwd_cnt++;
			}
		}
	}
	
	zsock_freeaddrinfo(res);
}
  
void display_thread(void) {  
	k_sem_take(&wifi_connected_sem, K_FOREVER);
	k_sem_give(&wifi_connected_sem);
	display_dev = DEVICE_DT_GET(DISPLAY_NODE);  
	if (device_is_ready(display_dev)) {
		cfb_framebuffer_init(display_dev);
		cfb_framebuffer_set_font(display_dev, 0);
	}
	char buf[32];  
	for (;;) {  
		k_sleep(K_MSEC(500));  
		k_mutex_lock(&disp_mutex, K_FOREVER);  
		if (device_is_ready(display_dev)) {
			cfb_framebuffer_clear(display_dev, false);  
			snprintk(buf, sizeof(buf), "TX:%u RX:%u", g_stats.tx_cnt, g_stats.rx_cnt);  
			cfb_print(display_dev, buf, 0, 0);  
			snprintk(buf, sizeof(buf), "FWD:%u ACK:%u", g_stats.fwd_cnt, g_stats.pull_ack_cnt);  
			cfb_print(display_dev, buf, 0, 16);  
			snprintk(buf, sizeof(buf), "WiFi:%s EUI:..%02X%02X", g_stats.wifi_ok ? "OK" : "..", g_gw_eui[6], g_gw_eui[7]);  
			cfb_print(display_dev, buf, 0, 32);  
			cfb_framebuffer_finalize(display_dev);  
		}
		k_mutex_unlock(&disp_mutex);  
	}  
}  
  
void monitor_thread(void) {  
	k_sem_take(&wifi_connected_sem, K_FOREVER);
	k_sem_give(&wifi_connected_sem);
	for (;;) {  
		k_sleep(K_SECONDS(5));    
		k_wakeup(tid_rx);
	}  
}  

void serial_thread(void) {
	console_init();
	for (;;) {
		uint8_t c = console_getchar();
		if (c == 's') for (int i = 0; i < 5; i++) { k_sem_give(&tx_sem); k_sleep(K_SECONDS(1)); }
	}
}

int main(void) {  
	// lora_dev = DEVICE_DT_GET(LORA_NODE);  
/*
	gpio_pin_configure_dt(&btn, GPIO_INPUT);  

	gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);  
	gpio_init_callback(&btn_cb, button_isr, BIT(btn.pin));  
	gpio_add_callback(btn.port, &btn_cb);  
*/

	/* Generate Gateway EUI from MAC */
/*
	struct net_if *iface = net_if_get_default();
	if (iface) {
		uint8_t mac[6]; memcpy(mac, net_if_get_link_addr(iface)->addr, 6);
		g_gw_eui[0] = mac[0]; g_gw_eui[1] = mac[1]; g_gw_eui[2] = mac[2];
		g_gw_eui[3] = 0xFF;   g_gw_eui[4] = 0xFE;
		g_gw_eui[5] = mac[3]; g_gw_eui[6] = mac[4]; g_gw_eui[7] = mac[5];

		LOG_INF("GW EUI: %02x%02x%02x%02x%02x%02x%02x%02x", 
			g_gw_eui[0], g_gw_eui[1], g_gw_eui[2], g_gw_eui[3],
			g_gw_eui[4], g_gw_eui[5], g_gw_eui[6], g_gw_eui[7]);
	}
*/
	connect_wifi();

	while(1) {
		k_sleep(K_SECONDS(10));
	}
	return 0;  
	}
