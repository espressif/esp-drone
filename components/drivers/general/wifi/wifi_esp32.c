#include <string.h>

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/api.h"

#include  "queuemonitor.h"
#include "wifi_esp32.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE  "WIFI_UDP"
#include "debug_cf.h"

#define UDP_SERVER_PORT         2390
#define UDP_SERVER_PORT2        2392
#define UDP_REMOTE_PORT         2399
#define UDP_REMOTE_ADDR         "192.168.43.43"
#define UDP_SERVER_RX_BUFSIZE   128

struct netconn *udp_server_netconn = NULL;
struct netconn *udp_server_netconn2 = NULL;
static ip_addr_t server_ipaddr;

//#define WIFI_SSID      "Udp Server"
static char WIFI_SSID[32] = "ESPLANE";
static char WIFI_PWD[64] = "12345678" ;
#define MAX_STA_CONN (1)

static xQueueHandle udpDataRx;
static xQueueHandle udpDataTx;
static UDPPacket inPacket;
static UDPPacket inPacket2;
static UDPPacket outPacket;

static bool isInit = false;

static esp_err_t udp_server_create(void *arg);

static uint8_t calculate_cksum(void *data, size_t len)
{
    unsigned char *c = data;
    int i;
    unsigned char cksum = 0;

    for (i = 0; i < len; i++) {
        cksum += *(c++);
    }

    return cksum;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        DEBUG_PRINT_LOCAL("station "MACSTR" join, AID=%d",
                          MAC2STR(event->mac), event->aid);

        if (udp_server_create(NULL) == ESP_FAIL) {
            DEBUG_PRINT_LOCAL("UDP server create socket failed!!!");
        } else {
            DEBUG_PRINT_LOCAL("UDP server create socket succeed!!!");
        }  

    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        DEBUG_PRINT_LOCAL("station "MACSTR" leave, AID=%d",
                          MAC2STR(event->mac), event->aid);

    }
}



bool wifiTest(void)
{
    return isInit;
};

bool wifiGetDataBlocking(UDPPacket *in)
{
    /* command step - receive  02  from udp rx queue */
    while (xQueueReceive(udpDataRx, in, portMAX_DELAY) != pdTRUE) {
        vTaskDelay(1);
    }; // Don't return until we get some data on the UDP

    return true;
};

bool wifiSendData(uint32_t size, uint8_t *data)
{
    static UDPPacket outStage;
    outStage.size = size;
    memcpy(outStage.data, data, size);
    // Dont' block when sending
    return (xQueueSend(udpDataTx, &outStage, M2T(100)) == pdTRUE);
};

static esp_err_t udp_server_create(void *arg)
{
    err_t err = ERR_OK;
    udp_server_netconn = netconn_new(NETCONN_UDP);  //创建socket

    if (udp_server_netconn == NULL) {
        return ESP_FAIL;
    }

    udp_server_netconn->recv_timeout = 10;
    err = netconn_bind(udp_server_netconn, &server_ipaddr, UDP_SERVER_PORT);//绑定IP地址和端口号

    if (err != ERR_OK) {
        netconn_close(udp_server_netconn);
        netconn_delete(udp_server_netconn);
        return ESP_FAIL;

    }

#ifdef CONFIG_ENABLE_LEGACY_APP
    udp_server_netconn2 = netconn_new(NETCONN_UDP);  //创建socket
    udp_server_netconn2->recv_timeout = 10;

    if (udp_server_netconn2 == NULL) {
        return ESP_FAIL;
    }

    err = netconn_bind(udp_server_netconn2, &server_ipaddr, UDP_SERVER_PORT2);//绑定IP地址和端口号

    if (err != ERR_OK) {
        netconn_close(udp_server_netconn2);
        netconn_delete(udp_server_netconn2);
        return ESP_FAIL;
    }

#endif

    return ESP_OK;
}

static void udp_server_rx_task(void *pvParameters)
{
    struct pbuf *q = NULL;

    uint8_t cksum = 0;
    struct netbuf *recvbuf = NULL;

    while (true) {
        if (udp_server_netconn == NULL) {
            vTaskDelay(20);
            continue;
        }

        /* command step - receive  01 from Wi-Fi UDP */
        if (netconn_recv(udp_server_netconn, &recvbuf) == ERR_OK) {
            for (q = recvbuf->p; q != NULL; q = q->next) {
                if (q->len > WIFI_RX_TX_PACKET_SIZE - 4) {
                    //TODO:
                    DEBUG_PRINTW("Received data length = %d > 64", q->len);
                } else {
                    //copy part of the UDP packet
                    memcpy(inPacket.data, q->payload, q->len);
                    cksum = inPacket.data[q->len - 1];
                    //remove cksum, do not belong to CRTP
                    inPacket.size = q->len - 1;

                    //check packet
                    if (cksum == calculate_cksum(inPacket.data, q->len - 1)) {
                        xQueueSend(udpDataRx, &inPacket, M2T(2));
                    } else {
                        DEBUG_PRINTW("udp packet cksum unmatched");
                    }

#ifdef DEBUG_UDP
                    DEBUG_PRINT_LOCAL("1.Received data size = %d  %02X \n cksum = %02X", q->len, inPacket.data[0], cksum);

                    for (size_t i = 0; i < q->len; i++) {
                        DEBUG_PRINT_LOCAL(" data[%d] = %02X ", i, inPacket.data[i]);
                    }

#endif

                }
            }
        }

        if (recvbuf != NULL) {
            netbuf_delete(recvbuf);
        }
    }

    vTaskDelete(NULL);
}

static void udp_server_rx2_task(void *pvParameters)
{
    struct pbuf *q = NULL;

    uint8_t cksum = 0;
    struct netbuf *recvbuf = NULL;

    while (true) {
        if (udp_server_netconn2 == NULL) {
            vTaskDelay(20);
            continue;
        }
        if (netconn_recv(udp_server_netconn2, &recvbuf) == ERR_OK) {
            for (q = recvbuf->p; q != NULL; q = q->next) {
                if (q->len > (sizeof(UDPPacket) + 1)) {
                    //TODO:
                } else {
                    memcpy(inPacket2.data, q->payload, q->len);
                    cksum = inPacket2.data[q->len - 1];
                    inPacket2.size = q->len - 1;
                    xQueueSend(udpDataRx, &inPacket2, 0);
#ifdef DEBUG_UDP
                    DEBUG_PRINT_LOCAL("1.Received data LENGTH = %d  %02X \n cksum = %02X", q->len, inPacket2.data[0], cksum);

                    for (size_t i = 0; i < q->len; i++) {
                        DEBUG_PRINT_LOCAL(" data[%d] = %02X ", i, inPacket2.data[i]);
                    }

#endif
                }
            }
        }

        netbuf_delete(recvbuf);
    }

    vTaskDelete(NULL);
}

static void udp_server_tx_task(void *pvParameters)
{
    struct netbuf *sendbuf = NULL;
    uint8_t sendbuffTemp[64] = {0};
    
    while (TRUE) {
        if (xQueueReceive(udpDataTx, &outPacket, 5) == pdTRUE) {
            memcpy(sendbuffTemp, outPacket.data, outPacket.size);
            sendbuffTemp[outPacket.size + 1] = '\0';
#ifdef DEBUG_UDP
            DEBUG_PRINTD("udpDataTx get QUEUE size = %d data = %02x", outPacket.size, outPacket.data[0]);
#endif
            sendbuffTemp[outPacket.size] =  calculate_cksum(sendbuffTemp, outPacket.size);
            sendbuf = netbuf_new();
            ip4addr_aton(UDP_REMOTE_ADDR, &(sendbuf->addr.u_addr.ip4));
            sendbuf->port = UDP_REMOTE_PORT;
            netbuf_alloc(sendbuf, outPacket.size + 1);
            sendbuf->p->payload = sendbuffTemp;

            if (udp_server_netconn != NULL) {
                netconn_sendto(udp_server_netconn, sendbuf, &sendbuf->addr, UDP_REMOTE_PORT) ;

#ifdef DEBUG_UDP
                DEBUG_PRINT_LOCAL("Send data to");

                for (size_t i = 0; i < outPacket.size + 1; i++) {
                    DEBUG_PRINT_LOCAL(" data_send[%d] = %02X ", i, sendbuffTemp[i]);
                }

#endif
            }
        netbuf_delete(sendbuf);
        }    
    }

    vTaskDelete(NULL);
}


void wifiInit(void)
{
    if (isInit) {
        return;
    }

    esp_netif_t *ap_netif = NULL;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ap_netif = esp_netif_create_default_wifi_ap();
    uint8_t mac[6];
// static wifi_country_t wifi_country = {.cc = "JP", .schan = 1, .nchan = 14, .policy = WIFI_COUNTRY_POLICY_MANUAL};

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID,
                    &wifi_event_handler,
                    NULL,
                    NULL));

// ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country)); // set locales for RF and channels
    ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
    sprintf(WIFI_SSID, "ESPLANE_%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    wifi_config_t wifi_config;
    memcpy(wifi_config.ap.ssid, WIFI_SSID, strlen(WIFI_SSID) + 1) ;
    wifi_config.ap.ssid_len = strlen(WIFI_SSID);
    memcpy(wifi_config.ap.password, WIFI_PWD, strlen(WIFI_PWD) + 1) ;
    wifi_config.ap.max_connection = MAX_STA_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_config.ap.channel  = 13;

    if (strlen(WIFI_PWD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info = {
        .ip.addr = ipaddr_addr("192.168.43.42"),
        .netmask.addr = ipaddr_addr("255.255.255.0"),
        .gw.addr      = ipaddr_addr("192.168.43.42"),
    };
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    DEBUG_PRINT_LOCAL("wifi_init_softap complete.SSID:%s password:%s", WIFI_SSID, WIFI_PWD);

    // This should probably be reduced to a CRTP packet size
    udpDataRx = xQueueCreate(5, sizeof(UDPPacket)); /* Buffer packets (max 64 bytes) */
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataRx);
    udpDataTx = xQueueCreate(1, sizeof(UDPPacket)); /* Buffer packets (max 64 bytes) */
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataTx);

    xTaskCreate(udp_server_tx_task, UDP_TX_TASK_NAME, UDP_TX_TASK_STACKSIZE, NULL, UDP_TX_TASK_PRI, NULL);
    xTaskCreate(udp_server_rx_task, UDP_RX_TASK_NAME, UDP_RX_TASK_STACKSIZE, NULL, UDP_RX_TASK_PRI, NULL);
#ifdef CONFIG_ENABLE_LEGACY_APP
     xTaskCreate(udp_server_rx2_task, UDP_RX2_TASK_NAME, UDP_RX2_TASK_STACKSIZE, NULL, UDP_RX2_TASK_PRI, NULL);
#endif
    isInit = true;
}