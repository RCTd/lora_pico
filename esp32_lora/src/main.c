#include <zephyr/kernel.h>  
#include <zephyr/logging/log.h>  
#include <zephyr/drivers/lora.h>  
#include <zephyr/drivers/gpio.h>  
#include <zephyr/device.h>  
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
#include <fcntl.h>

LOG_MODULE_REGISTER(IoTProj, LOG_LEVEL_INF);

/* ---------- Config Networking ---------- */
#define WIFI_SSID "DIGI-gU2b"
#define WIFI_PSK  "bC9yCPkUbz"
#define TTN_SERVER "63.34.215.128" 
#define TTN_PORT   "1700"

/* ---------- Keys for Local Decryption (match Pico) ---------- */
static const uint8_t app_skey[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
static const uint32_t dev_addr = 0x260B2125;

/* ---------- Config Threaduri ---------- */  
#define STACK_SIZE   2048  
#define PRIORITY_RX  5  
#define PRIORITY_FWD 5
#define PRIORITY_MON 2  
  
#define BTN_NODE      DT_ALIAS(sw0)  
#define LORA_NODE     DT_ALIAS(lora0)  

/* ---------- Structuri de Date ---------- */  
struct lora_packet_t {
	uint8_t data[256];
	uint16_t len;
	int16_t rssi;
	int8_t snr;
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

/* ---------- Resurse Kernel ---------- */
K_MSGQ_DEFINE(fwd_q, sizeof(struct lora_packet_t), 8, 4);
K_SEM_DEFINE(wifi_connected_sem, 0, 1);
K_MUTEX_DEFINE(stats_mutex);  
K_MUTEX_DEFINE(lora_lock);
  
static const struct device *lora_dev;  
static uint8_t g_gw_eui[8];

struct stats_t {  
	uint32_t rx_cnt;  
	uint32_t fwd_cnt;
	uint32_t pull_cnt;
	uint32_t ack_cnt;
};  
static struct stats_t g_stats;  

/* ---------- Forward Decls ---------- */
static void lora_rx_cb(const struct device *dev, uint8_t *data, uint16_t len, int16_t rssi, int8_t snr);
void lora_rx_thread(void *p1, void *p2, void *p3);
void lora_fwd_thread(void *p1, void *p2, void *p3);
void monitor_thread(void *p1, void *p2, void *p3);

K_THREAD_STACK_DEFINE(stack_rx, STACK_SIZE);
K_THREAD_STACK_DEFINE(stack_fwd, 4096);
K_THREAD_STACK_DEFINE(stack_mon, STACK_SIZE);

struct k_thread thread_rx;
struct k_thread thread_fwd;
struct k_thread thread_mon;

#include <mbedtls/aes.h>

/* ---------- Tiny AES Helper (LoRaWAN CTR Decryption) ---------- */
void debug_print_payload(uint8_t *data, uint16_t len) {
    if (len < 13) return; // MHDR(1)+Addr(4)+Ctrl(1)+Cnt(2)+Port(1) ... MIC(4)
    
    int fopts_len = data[5] & 0x0F;
    int payload_offset = 1 + 4 + 1 + 2 + fopts_len;
    if (payload_offset < len - 4) payload_offset++; // Skip FPort
    
    int payload_len = len - 4 - payload_offset;
    if (payload_len <= 0) return;

    printk("  LoRaWAN Header: MHDR=0x%02X, Addr=0x%08X, FCnt=%u\n", 
           data[0], sys_get_le32(&data[1]), sys_get_le16(&data[6]));
    
    // Construct A_block for AES-CTR
    uint8_t a_block[16] = {0};
    a_block[0] = 0x01;
    a_block[5] = 0; // Dir = 0 (Uplink)
    memcpy(&a_block[6], &data[1], 4);  // DevAddr (Little Endian)
    memcpy(&a_block[10], &data[6], 2); // FCnt (Little Endian, top 16 bits assumed 0)
    
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, app_skey, 128);

    uint8_t s_block[16];
    uint8_t plain[128] = {0};
    if (payload_len > 127) payload_len = 127;
    
    for (int i = 0; i < payload_len; i++) {
        if (i % 16 == 0) {
            a_block[15] = (i / 16) + 1; // Block index i
            mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, a_block, s_block);
        }
        plain[i] = data[payload_offset + i] ^ s_block[i % 16];
    }
    mbedtls_aes_free(&aes);
    
    // Replace non-printable characters for safe logging
    for (int i = 0; i < payload_len; i++) {
        if (plain[i] < 32 || plain[i] > 126) plain[i] = '.';
    }
    
    printk("  Decrypted Text: [%s]\n", plain);
}

/* ---------- Base64 Encode (pentru TTN) ---------- */
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

/* ---------- Wi-Fi Management ---------- */
static struct net_mgmt_event_callback wifi_cb;
static void wifi_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface) {
	if (mgmt_event == NET_EVENT_WIFI_CONNECT_RESULT) {
		LOG_INF("Wi-Fi Connected! Configuring Static IP Network...");
		struct in_addr addr, mask, gw;
		net_addr_pton(AF_INET, "192.168.1.50", &addr);
		net_addr_pton(AF_INET, "255.255.255.0", &mask);
		net_addr_pton(AF_INET, "192.168.1.1", &gw);
		
		net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0);
		net_if_ipv4_set_netmask_by_addr(iface, &addr, &mask);
		net_if_ipv4_set_gw(iface, &gw);
		k_sem_give(&wifi_connected_sem);
	}
}

static struct wifi_connect_req_params wifi_params;
void connect_wifi(void) {
	struct net_if *iface = net_if_get_default();
	wifi_params.ssid = (uint8_t *)WIFI_SSID;
	wifi_params.ssid_length = strlen(WIFI_SSID);
	wifi_params.psk = (uint8_t *)WIFI_PSK;
	wifi_params.psk_length = strlen(WIFI_PSK);
	wifi_params.channel = WIFI_CHANNEL_ANY;
	wifi_params.security = WIFI_SECURITY_TYPE_PSK;

	net_mgmt_init_event_callback(&wifi_cb, wifi_handler, NET_EVENT_WIFI_CONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_cb);
	net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &wifi_params, sizeof(wifi_params));
}

/* ---------- TASK 1: LoRa Receiver ---------- */  
void lora_rx_thread(void *p1, void *p2, void *p3)  
{  
    if (!device_is_ready(lora_dev)) return;  
    for (;;) {    
        k_mutex_lock(&lora_lock, K_FOREVER); 
        lora_recv_async(lora_dev, NULL);  
        lora_config(lora_dev, &cfg_rx);
        if (lora_recv_async(lora_dev, lora_rx_cb) == 0) {  
            k_mutex_unlock(&lora_lock);  
            k_sleep(K_FOREVER); 
        } else {   
            k_mutex_unlock(&lora_lock);  
            k_sleep(K_MSEC(500));  
        }  
    } 
} 

static void lora_rx_cb(const struct device *dev, uint8_t *data, uint16_t len, int16_t rssi, int8_t snr)  
{  
	LOG_INF("LoRa RX: %u bytes, RSSI: %d dBm", len, rssi);
    debug_print_payload(data, len);

	struct lora_packet_t pkt;
	pkt.len = len; pkt.rssi = rssi; pkt.snr = snr;
	memcpy(pkt.data, data, len);
	k_msgq_put(&fwd_q, &pkt, K_NO_WAIT);
	k_mutex_lock(&stats_mutex, K_NO_WAIT); g_stats.rx_cnt++; k_mutex_unlock(&stats_mutex);  
	
	k_wakeup(&thread_rx);
}  

/* ---------- TASK 2: TDM Forwarder (ACK Sync Version + BIND) ---------- */
void lora_fwd_thread(void *p1, void *p2, void *p3) {
	struct zsock_addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_DGRAM, .ai_protocol = IPPROTO_UDP };
	struct zsock_addrinfo *res;
	k_sem_take(&wifi_connected_sem, K_FOREVER);
	if (zsock_getaddrinfo(TTN_SERVER, TTN_PORT, &hints, &res) != 0) return;
	int sock = zsock_socket(res->ai_family, res->ai_socktype, res->ai_protocol);
	if (sock < 0) return;

	struct sockaddr_in local_addr;
	local_addr.sin_family = AF_INET;
	local_addr.sin_port = htons(1700);
	local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	zsock_bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr));

	struct lora_packet_t pkt;
	uint8_t buffer[1024];
	uint32_t last_pull = 0;

	for (;;) {
		uint32_t now = k_uptime_get_32();

		// A: PULL_DATA
		if (last_pull == 0 || now - last_pull > 10000) {
			k_mutex_lock(&lora_lock, K_FOREVER);
			lora_recv_async(lora_dev, NULL);
			buffer[0] = 0x02; sys_put_be16(k_cycle_get_32() & 0xFFFF, &buffer[1]);
			buffer[3] = 0x02; memcpy(&buffer[4], g_gw_eui, 8);
			zsock_sendto(sock, buffer, 12, 0, res->ai_addr, res->ai_addrlen);
			
			uint8_t ack[12];
			struct zsock_pollfd fds = { .fd = sock, .events = ZSOCK_POLLIN };
			if (zsock_poll(&fds, 1, 1000) > 0) {
				if (zsock_recv(sock, ack, 12, 0) > 0) {
					LOG_INF("TTN Server: Connected! (PULL_ACK)");
					k_mutex_lock(&stats_mutex, K_NO_WAIT); g_stats.ack_cnt++; k_mutex_unlock(&stats_mutex);
				}
			} else {
				LOG_WRN("TTN Server: ACK Timeout.");
			}
			
			k_mutex_lock(&stats_mutex, K_NO_WAIT); g_stats.pull_cnt++; k_mutex_unlock(&stats_mutex);
			k_sleep(K_MSEC(50)); k_mutex_unlock(&lora_lock); k_wakeup(&thread_rx); 
			last_pull = now;
		}

		// B: PUSH_DATA (Masquerade 868.1/4/5)
		if (k_msgq_get(&fwd_q, &pkt, K_MSEC(500)) == 0) {
			k_mutex_lock(&lora_lock, K_FOREVER);
			lora_recv_async(lora_dev, NULL);
			char b64[400]; base64_encode(pkt.data, pkt.len, b64);
			char json[800];
			snprintk(json, sizeof(json), "{\"rxpk\":[{\"tmst\":%u,\"freq\":868.1,\"chan\":0,\"rfch\":0,\"stat\":1,\"modu\":\"LORA\",\"datr\":\"SF10BW125\",\"codr\":\"4/5\",\"rssi\":%d,\"lsnr\":%.1f,\"size\":%u,\"data\":\"%s\"}]}",
				k_uptime_get_32(), pkt.rssi, (double)pkt.snr, pkt.len, b64);
			buffer[0] = 0x02; sys_put_be16(k_cycle_get_32() & 0xFFFF, &buffer[1]);
			buffer[3] = 0x00; memcpy(&buffer[4], g_gw_eui, 8);
			int json_len = strlen(json); memcpy(&buffer[12], json, json_len);
			zsock_sendto(sock, buffer, 12 + json_len, 0, res->ai_addr, res->ai_addrlen);
			
			LOG_INF("TDM: WIFI Uplink...");
			k_mutex_lock(&stats_mutex, K_NO_WAIT); g_stats.fwd_cnt++; k_mutex_unlock(&stats_mutex);
			k_sleep(K_MSEC(150)); k_mutex_unlock(&lora_lock); k_wakeup(&thread_rx); 
		}
	}
}

void monitor_thread(void *p1, void *p2, void *p3)  
{  
    for (;;) {  
        k_sleep(K_SECONDS(20));    
		LOG_INF("Stats: RX:%u, FWD:%u, PULL:%u, ACK:%u", g_stats.rx_cnt, g_stats.fwd_cnt, g_stats.pull_cnt, g_stats.ack_cnt);
        k_wakeup(&thread_rx);
    }  
}  

/* ---------- main() ---------- */  
int main(void)  
{  
    lora_dev = DEVICE_DT_GET(LORA_NODE);  
    if (!device_is_ready(lora_dev)) return 0;  
	struct net_if *iface = net_if_get_default();
	if (iface) {
		uint8_t mac[6]; memcpy(mac, net_if_get_link_addr(iface)->addr, 6);
		g_gw_eui[0] = mac[0]; g_gw_eui[1] = mac[1]; g_gw_eui[2] = mac[2];
		g_gw_eui[3] = 0xFF;   g_gw_eui[4] = 0xFE;
		g_gw_eui[5] = mac[3]; g_gw_eui[6] = mac[4]; g_gw_eui[7] = mac[5];
	}
	LOG_INF("LoRa TDM Gateway Debug Started");  
	k_thread_create(&thread_rx, stack_rx, STACK_SIZE, lora_rx_thread, NULL, NULL, NULL, PRIORITY_RX, 0, K_NO_WAIT);
	k_thread_create(&thread_fwd, stack_fwd, 4096, lora_fwd_thread, NULL, NULL, NULL, PRIORITY_FWD, 0, K_NO_WAIT);
	k_thread_create(&thread_mon, stack_mon, STACK_SIZE, monitor_thread, NULL, NULL, NULL, PRIORITY_MON, 0, K_NO_WAIT);
	connect_wifi();
	return 0;  
}