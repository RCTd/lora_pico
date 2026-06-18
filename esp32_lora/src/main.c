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
#include <zephyr/sys/base64.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <mbedtls/aes.h>
#include <fcntl.h>

LOG_MODULE_REGISTER(IoTProj, LOG_LEVEL_INF);

/* ---------- Config Networking ---------- */
#define WIFI_SSID "Lora-Fi"
#define WIFI_PSK  "lora-pico"
#define TTN_SERVER "63.34.215.128" 
#define TTN_PORT   "1700"

/* ---------- Keys for Local Decryption ---------- */
static const uint8_t app_skey[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xf7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

/* ---------- Config Threaduri ---------- */  
#define STACK_SIZE   2048  
#define PRIORITY_RX  5  
#define PRIORITY_FWD 5
#define PRIORITY_MON 2  
#define PRIORITY_DISP 6
  
#define DISPLAY_NODE  DT_CHOSEN(zephyr_display)  
#define LORA_NODE     DT_ALIAS(lora0)  

/* ---------- Structuri de Date ---------- */  
struct __attribute__((packed)) lora_packet_t {
	uint32_t timestamp; 
	uint16_t len;
	int16_t rssi;
	int8_t snr;
	uint8_t data[256];
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
static const struct device *display_dev;
static uint8_t g_gw_eui[8];

struct stats_t {  
	uint32_t rx_cnt;  
	uint32_t fwd_cnt;
	uint32_t pull_cnt;
	uint32_t ack_cnt;
	uint32_t backlog_cnt;
};  
static struct stats_t g_stats;  

/* ---------- NVS Configuration ---------- */
static struct nvs_fs fs;
#define STORAGE_NODE DT_NODE_BY_FIXED_PARTITION_LABEL(storage)
#define NVS_PACKET_ID_START 100
#define NVS_MAX_PACKETS     100
static uint16_t nvs_write_idx = 0;
static uint16_t nvs_read_idx = 0;

/* ---------- Forward Decls ---------- */
static void lora_rx_cb(const struct device *dev, uint8_t *data, uint16_t len, int16_t rssi, int8_t snr);
void lora_rx_thread(void *p1, void *p2, void *p3);
void lora_fwd_thread(void *p1, void *p2, void *p3);
void monitor_thread(void *p1, void *p2, void *p3);
void display_thread(void *p1, void *p2, void *p3);

K_THREAD_STACK_DEFINE(stack_rx, STACK_SIZE);
K_THREAD_STACK_DEFINE(stack_fwd, 4096);
K_THREAD_STACK_DEFINE(stack_mon, STACK_SIZE);
K_THREAD_STACK_DEFINE(stack_disp, 1024);

struct k_thread thread_rx;
struct k_thread thread_fwd;
struct k_thread thread_mon;
struct k_thread thread_disp;

/* ---------- Tiny AES Helper (LoRaWAN CTR Decryption) ---------- */
void debug_print_payload(uint8_t *data, uint16_t len) {
    if (len < 13) return;
    int fopts_len = data[5] & 0x0F;
    int payload_offset = 1 + 4 + 1 + 2 + fopts_len;
    if (payload_offset < len - 4) payload_offset++; 
    int payload_len = len - 4 - payload_offset;
    if (payload_len <= 0) return;
    
    uint8_t a_block[16] = {0x01, 0, 0, 0, 0, 0};
    memcpy(&a_block[6], &data[1], 4);
    memcpy(&a_block[10], &data[6], 2);
    
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, app_skey, 128);
    uint8_t s_block[16];
    uint8_t plain[128] = {0};
    if (payload_len > 127) payload_len = 127;
    for (int i = 0; i < payload_len; i++) {
        if (i % 16 == 0) {
            a_block[15] = (i / 16) + 1;
            mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, a_block, s_block);
        }
        plain[i] = data[payload_offset + i] ^ s_block[i % 16];
    }
    mbedtls_aes_free(&aes);
    for (int i = 0; i < payload_len; i++) if (plain[i] < 32 || plain[i] > 126) plain[i] = '.';
    plain[payload_len] = '\0';
    printk("  Decrypted Text: [%s]\n", plain);
}

/* ---------- Wi-Fi Management ---------- */
static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

static void ipv4_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface) {
	if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD) {
		LOG_INF("IPv4 Address Assigned via DHCP!");
		k_sem_give(&wifi_connected_sem);
	}
}

static void wifi_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface) {
	if (mgmt_event == NET_EVENT_WIFI_CONNECT_RESULT) {
		LOG_INF("Wi-Fi Connected! Starting DHCP...");
		net_dhcpv4_start(iface);
	}
}

void connect_wifi(void) {
	struct net_if *iface = net_if_get_default();
	static struct wifi_connect_req_params wifi_params;
	wifi_params.ssid = (uint8_t *)WIFI_SSID;
	wifi_params.ssid_length = strlen(WIFI_SSID);
	wifi_params.psk = (uint8_t *)WIFI_PSK;
	wifi_params.psk_length = strlen(WIFI_PSK);
	wifi_params.channel = WIFI_CHANNEL_ANY;
	wifi_params.security = WIFI_SECURITY_TYPE_PSK;
	net_mgmt_init_event_callback(&wifi_cb, wifi_handler, NET_EVENT_WIFI_CONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_cb);
	net_mgmt_init_event_callback(&ipv4_cb, ipv4_handler, NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&ipv4_cb);
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
	memset(&pkt, 0, sizeof(pkt));
	pkt.len = len; pkt.rssi = rssi; pkt.snr = snr;
	pkt.timestamp = k_uptime_get_32() * 1000;
	memcpy(pkt.data, data, len);
	k_msgq_put(&fwd_q, &pkt, K_NO_WAIT);
	k_mutex_lock(&stats_mutex, K_NO_WAIT); g_stats.rx_cnt++; k_mutex_unlock(&stats_mutex);  
	k_wakeup(&thread_rx);
}  

/* ---------- TASK 2: TDM Forwarder ---------- */
void lora_fwd_thread(void *p1, void *p2, void *p3) {
	k_sem_take(&wifi_connected_sem, K_FOREVER);
	
	int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	while (sock < 0) {
		LOG_ERR("Socket not ready, retrying in 3s...");
		k_sleep(K_SECONDS(3));
		sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	}

	struct sockaddr_in local_addr;
	local_addr.sin_family = AF_INET; 
	local_addr.sin_port = htons(1700); 
	local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	zsock_bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr));

	struct sockaddr_in dest_addr;
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_port = htons(1700);
	zsock_inet_pton(AF_INET, TTN_SERVER, &dest_addr.sin_addr);

	struct lora_packet_t pkt;
	uint8_t buffer[1024];
	uint32_t last_pull = 0;
	bool ttn_online = false;

	for (;;) {
		uint32_t now = k_uptime_get_32();
		
		if (last_pull == 0 || now - last_pull > 10000) {
			k_mutex_lock(&lora_lock, K_FOREVER);
			lora_recv_async(lora_dev, NULL);
			buffer[0] = 0x02; sys_put_be16(k_cycle_get_32() & 0xFFFF, &buffer[1]);
			buffer[3] = 0x02; memcpy(&buffer[4], g_gw_eui, 8);
			zsock_sendto(sock, buffer, 12, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
			uint8_t ack[12];
			struct zsock_pollfd fds = { .fd = sock, .events = ZSOCK_POLLIN };
			if (zsock_poll(&fds, 1, 1000) > 0) {
				if (zsock_recv(sock, ack, 12, 0) > 0) {
					ttn_online = true;
					k_mutex_lock(&stats_mutex, K_NO_WAIT); g_stats.ack_cnt++; k_mutex_unlock(&stats_mutex);
				}
			} else { ttn_online = false; }
			k_mutex_lock(&stats_mutex, K_NO_WAIT); g_stats.pull_cnt++; k_mutex_unlock(&stats_mutex);
			k_sleep(K_MSEC(50)); k_mutex_unlock(&lora_lock); k_wakeup(&thread_rx); 
			last_pull = now;
		}

		if (ttn_online && g_stats.backlog_cnt > 0) {
			struct lora_packet_t bl_pkt;
			if (nvs_read(&fs, NVS_PACKET_ID_START + nvs_read_idx, &bl_pkt, sizeof(bl_pkt)) > 0) {
				k_mutex_lock(&lora_lock, K_FOREVER);
				lora_recv_async(lora_dev, NULL);
				char b64[400]; size_t olen;
				base64_encode((uint8_t *)b64, sizeof(b64), &olen, bl_pkt.data, bl_pkt.len); b64[olen] = '\0';
				char json[800];
				snprintk(json, sizeof(json), "{\"rxpk\":[{\"tmst\":%u,\"freq\":868.1,\"chan\":0,\"rfch\":0,\"stat\":1,\"modu\":\"LORA\",\"datr\":\"SF10BW125\",\"codr\":\"4/5\",\"rssi\":%d,\"lsnr\":%d.0,\"size\":%u,\"data\":\"%s\"}]}",
					bl_pkt.timestamp, bl_pkt.rssi, (int)bl_pkt.snr, bl_pkt.len, b64);
				buffer[0] = 0x02; sys_put_be16(k_cycle_get_32() & 0xFFFF, &buffer[1]);
				buffer[3] = 0x00; memcpy(&buffer[4], g_gw_eui, 8);
				int json_len = strlen(json); memcpy(&buffer[12], json, json_len);
				zsock_sendto(sock, buffer, 12 + json_len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
				LOG_INF("JSON Backlog: %s", json);
				nvs_delete(&fs, NVS_PACKET_ID_START + nvs_read_idx);
				nvs_read_idx = (nvs_read_idx + 1) % NVS_MAX_PACKETS;
				k_mutex_lock(&stats_mutex, K_NO_WAIT); g_stats.backlog_cnt--; k_mutex_unlock(&stats_mutex);
				k_sleep(K_MSEC(150)); k_mutex_unlock(&lora_lock); k_wakeup(&thread_rx); 
			}
		}

		if (k_msgq_get(&fwd_q, &pkt, K_MSEC(500)) == 0) {
			if (ttn_online) {
				k_mutex_lock(&lora_lock, K_FOREVER);
				lora_recv_async(lora_dev, NULL);
				char b64[400]; size_t olen;
				base64_encode((uint8_t *)b64, sizeof(b64), &olen, pkt.data, pkt.len); b64[olen] = '\0';
				char json[800];
				snprintk(json, sizeof(json), "{\"rxpk\":[{\"tmst\":%u,\"freq\":868.1,\"chan\":0,\"rfch\":0,\"stat\":1,\"modu\":\"LORA\",\"datr\":\"SF10BW125\",\"codr\":\"4/5\",\"rssi\":%d,\"lsnr\":%d.0,\"size\":%u,\"data\":\"%s\"}]}",
					pkt.timestamp, pkt.rssi, (int)pkt.snr, pkt.len, b64);
				buffer[0] = 0x02; sys_put_be16(k_cycle_get_32() & 0xFFFF, &buffer[1]);
				buffer[3] = 0x00; memcpy(&buffer[4], g_gw_eui, 8);
				int json_len = strlen(json); memcpy(&buffer[12], json, json_len);
				zsock_sendto(sock, buffer, 12 + json_len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
				LOG_INF("JSON RT: %s", json);
				k_mutex_lock(&stats_mutex, K_NO_WAIT); g_stats.fwd_cnt++; k_mutex_unlock(&stats_mutex);
				k_sleep(K_MSEC(150)); k_mutex_unlock(&lora_lock); k_wakeup(&thread_rx); 
			} else {
				nvs_write(&fs, NVS_PACKET_ID_START + nvs_write_idx, &pkt, sizeof(pkt));
				nvs_write_idx = (nvs_write_idx + 1) % NVS_MAX_PACKETS;
				k_mutex_lock(&stats_mutex, K_NO_WAIT); g_stats.backlog_cnt++; k_mutex_unlock(&stats_mutex);
			}
		}
	}
}

/* ---------- TASK 3: Display Thread ---------- */
void display_thread(void *p1, void *p2, void *p3)  
{  
	display_dev = DEVICE_DT_GET(DISPLAY_NODE);  
	if (device_is_ready(display_dev)) {
        cfb_framebuffer_init(display_dev);
        cfb_framebuffer_set_font(display_dev, 0);
    }
	char buf[32];  
	for (;;) {  
		k_sleep(K_SECONDS(5));  
		if (device_is_ready(display_dev)) {
            k_mutex_lock(&stats_mutex, K_FOREVER);  
            cfb_framebuffer_clear(display_dev, false);  
            snprintk(buf, sizeof(buf), "RX:%u FWD:%u", g_stats.rx_cnt, g_stats.fwd_cnt);  
            cfb_print(display_dev, buf, 0, 0);  
            snprintk(buf, sizeof(buf), "ACK:%u BKLOG:%u", g_stats.ack_cnt, g_stats.backlog_cnt);  
            cfb_print(display_dev, buf, 0, 16);  
            cfb_framebuffer_finalize(display_dev);  
            k_mutex_unlock(&stats_mutex);  
        }
	}  
}  

void monitor_thread(void *p1, void *p2, void *p3)  
{  
    for (;;) {  
        k_sleep(K_SECONDS(20));    
		LOG_INF("Stats: RX:%u, FWD:%u, ACK:%u, BACKLOG:%u", g_stats.rx_cnt, g_stats.fwd_cnt, g_stats.ack_cnt, g_stats.backlog_cnt);
        k_wakeup(&thread_rx);
    }  
}  

/* ---------- main() ---------- */  
int main(void)  
{  
    lora_dev = DEVICE_DT_GET(LORA_NODE);  
    if (!device_is_ready(lora_dev)) return 0;  

	struct flash_pages_info info;
	fs.flash_device = DEVICE_DT_GET(DT_MTD_FROM_FIXED_PARTITION(STORAGE_NODE));
	fs.offset = DT_REG_ADDR(STORAGE_NODE);
	flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	fs.sector_size = info.size;
	fs.sector_count = 3; 
	nvs_mount(&fs);

	struct net_if *iface = net_if_get_default();
	if (iface) {
		uint8_t mac[6]; memcpy(mac, net_if_get_link_addr(iface)->addr, 6);
		g_gw_eui[0]=mac[0]; g_gw_eui[1]=mac[1]; g_gw_eui[2]=mac[2]; g_gw_eui[3]=0xFF; g_gw_eui[4]=0xFE; g_gw_eui[5]=mac[3]; g_gw_eui[6]=mac[4]; g_gw_eui[7]=mac[5];
	}
	LOG_INF("LoRa Store-and-Forward Gateway Ready");  
	k_thread_create(&thread_rx, stack_rx, STACK_SIZE, lora_rx_thread, NULL, NULL, NULL, PRIORITY_RX, 0, K_NO_WAIT);
	k_thread_create(&thread_fwd, stack_fwd, 4096, lora_fwd_thread, NULL, NULL, NULL, PRIORITY_FWD, 0, K_NO_WAIT);
	k_thread_create(&thread_mon, stack_mon, STACK_SIZE, monitor_thread, NULL, NULL, NULL, PRIORITY_MON, 0, K_NO_WAIT);
	k_thread_create(&thread_disp, stack_disp, 1024, display_thread, NULL, NULL, NULL, PRIORITY_DISP, 0, K_NO_WAIT);
	connect_wifi();
	return 0;  
}