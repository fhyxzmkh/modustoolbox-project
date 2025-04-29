/******************************************************************************
* File Name:   tcp_client.c
*
* Description: This file contains task and functions related to TCP client
* operation.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2019-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/* Header file includes. */
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/* RTOS header file. */
#include "cyabs_rtos.h"

/* Standard C header file. */
#include <string.h>

/* Cypress secure socket header file. */
#include "cy_secure_sockets.h"

/* Wi-Fi connection manager header files. */
#include "cy_wcm.h"
#include "cy_wcm_error.h"

/* TCP client task header file. */
#include "tcp_client.h"

/* IP address related header files. */
#include "cy_nw_helper.h"

/* Standard C header files */
#include <inttypes.h>

/*******************************************************************************
* Macros
********************************************************************************/
/* To use the Wi-Fi device in AP interface mode, set this macro as '1' */
#define USE_AP_INTERFACE                         (0)

#define MAKE_IP_PARAMETERS(a, b, c, d)           ((((uint32_t) d) << 24) | \
                                                 (((uint32_t) c) << 16) | \
                                                 (((uint32_t) b) << 8) |\
                                                 ((uint32_t) a))

#if(USE_AP_INTERFACE)
    #define WIFI_INTERFACE_TYPE                  CY_WCM_INTERFACE_TYPE_AP

    /* SoftAP Credentials: Modify SOFTAP_SSID and SOFTAP_PASSWORD as required */
    #define SOFTAP_SSID                          "MY_SOFT_AP"
    #define SOFTAP_PASSWORD                      "psoc1234"

    /* Security type of the SoftAP. See 'cy_wcm_security_t' structure
     * in "cy_wcm.h" for more details.
     */
    #define SOFTAP_SECURITY_TYPE                  CY_WCM_SECURITY_WPA2_AES_PSK

    #define SOFTAP_IP_ADDRESS_COUNT               (2u)

    #define SOFTAP_IP_ADDRESS                     MAKE_IP_PARAMETERS(192, 168, 10, 1)
    #define SOFTAP_NETMASK                        MAKE_IP_PARAMETERS(255, 255, 255, 0)
    #define SOFTAP_GATEWAY                        MAKE_IP_PARAMETERS(192, 168, 10, 1)
    #define SOFTAP_RADIO_CHANNEL                  (1u)
#else
    #define WIFI_INTERFACE_TYPE                   CY_WCM_INTERFACE_TYPE_STA

    /* Wi-Fi Credentials: Modify WIFI_SSID, WIFI_PASSWORD, and WIFI_SECURITY_TYPE
     * to match your Wi-Fi network credentials.
     * Note: Maximum length of the Wi-Fi SSID and password is set to
     * CY_WCM_MAX_SSID_LEN and CY_WCM_MAX_PASSPHRASE_LEN as defined in cy_wcm.h file.
     */
    #define WIFI_SSID                             "Redmi K50"
    #define WIFI_PASSWORD                         "YuanDe1PersonHeart"

	// "Redmi K50" "YuanDe1PersonHeart"

    /* Security type of the Wi-Fi access point. See 'cy_wcm_security_t' structure
     * in "cy_wcm.h" for more details.
     */
    #define WIFI_SECURITY_TYPE                    CY_WCM_SECURITY_WPA2_AES_PSK
    /* Maximum number of connection retries to a Wi-Fi network. */
    #define MAX_WIFI_CONN_RETRIES                 (10u)

    /* Wi-Fi re-connection time interval in milliseconds */
    #define WIFI_CONN_RETRY_INTERVAL_MSEC         (1000u)
#endif /* USE_AP_INTERFACE */

/* Maximum number of connection retries to the TCP server. */
#define MAX_TCP_SERVER_CONN_RETRIES               (5u)

/* Length of the TCP data packet. */
#define MAX_TCP_DATA_PACKET_LENGTH                (20u)

/* TCP keep alive related macros. */
#define TCP_KEEP_ALIVE_IDLE_TIME_MS               (10000u)
#define TCP_KEEP_ALIVE_INTERVAL_MS                (1000u)
#define TCP_KEEP_ALIVE_RETRY_COUNT                (2u)

/* Length of the LED ON/OFF command issued from the TCP server. */
#define TCP_LED_CMD_LEN                           (1u)
#define LED_ON_CMD                                '1'
#define LED_OFF_CMD                               '0'
#define ACK_LED_ON                                "LED ON ACK"
#define ACK_LED_OFF                               "LED OFF ACK"
#define MSG_INVALID_CMD                           "Invalid command"

#define TCP_SERVER_PORT                           (50007u)
#define ASCII_BACKSPACE                           (0x08)
#define RTOS_TICK_TO_WAIT                         (50u)
#define UART_INPUT_TIMEOUT_MS                     (1u)
#define UART_BUFFER_SIZE                          (20u)

#define SEMAPHORE_LIMIT                           (1u)


/*******************************************************************************
* Function Prototypes
********************************************************************************/
cy_rslt_t create_tcp_client_socket();
cy_rslt_t tcp_client_recv_handler(cy_socket_t socket_handle, void *arg);
cy_rslt_t tcp_disconnection_handler(cy_socket_t socket_handle, void *arg);
cy_rslt_t connect_to_tcp_server(cy_socket_sockaddr_t address);
void read_uart_input(uint8_t* input_buffer_ptr);

#if(USE_AP_INTERFACE)
    static cy_rslt_t softap_start(void);
#else
    static cy_rslt_t connect_to_wifi_ap(void);
#endif /* USE_AP_INTERFACE */

/*******************************************************************************
* Global Variables
********************************************************************************/
/* TCP client socket handle */
cy_socket_t client_handle;

/* Binary semaphore handle to keep track of TCP server connection. */
cy_semaphore_t connect_to_server;

/* Holds the IP address obtained for SoftAP using Wi-Fi Connection Manager (WCM). */
cy_wcm_ip_address_t softap_ip_address;

/*******************************************************************************
 * Function Name: tcp_client_task
 *******************************************************************************
 * Summary:
 *  Task used to establish a connection to a remote TCP server and
 *  control the LED state (ON/OFF) based on the command received from TCP server.
 *
 * Parameters:
 *  void *args : Task parameter defined during task creation (unused).
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void tcp_client_task(void *arg) {
    cy_rslt_t result;
    uint8_t response_buffer[HTTP_RESPONSE_BUFFER_SIZE];
    cy_awsport_server_info_t server_info = {
        .host_name = HTTP_SERVER_HOST,
        .port = HTTP_SERVER_PORT
    };

    cy_http_client_t http_handle;
    cy_http_client_request_header_t request;
    cy_http_client_response_t response;
    cy_http_client_header_t header[1];
    uint32_t num_header = 1;

    cy_wcm_config_t wifi_config = { .interface = WIFI_INTERFACE_TYPE };

    /* Initialize Wi-Fi connection manager. */
    result = cy_wcm_init(&wifi_config);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("Wi-Fi Connection Manager initialization failed! Error code: 0x%08"PRIx32"\n", (uint32_t)result);
        CY_ASSERT(0);
    }
    printf("Wi-Fi Connection Manager initialized.\r\n");

    #if(USE_AP_INTERFACE)

        /* Start the Wi-Fi device as a Soft AP interface. */
        result = softap_start();
        if (result != CY_RSLT_SUCCESS)
        {
            printf("Failed to Start Soft AP! Error code: 0x%08"PRIx32"\n", (uint32_t)result);
            CY_ASSERT(0);
        }
    #else
        /* Connect to Wi-Fi AP */
        result = connect_to_wifi_ap();
        if(result!= CY_RSLT_SUCCESS )
        {
            printf("\n Failed to connect to Wi-Fi AP! Error code: 0x%08"PRIx32"\n", (uint32_t)result);
            CY_ASSERT(0);
        }
    #endif /* USE_AP_INTERFACE */

    /* 初始化 HTTP 客户端库 */
    result = cy_http_client_init();
    if(result != CY_RSLT_SUCCESS) {
        printf("HTTP 客户端初始化失败！错误码: 0x%08"PRIx32"\n", (uint32_t)result);
        CY_ASSERT(0);
    }
    printf("HTTP 客户端库初始化成功\r\n");

    /* 主循环：轮询 HTTP API */
    while(1) {
        /* 创建 HTTP 客户端实例 */
        result = cy_http_client_create(NULL, &server_info, NULL, NULL, &http_handle);
        if(result != CY_RSLT_SUCCESS) {
            printf("HTTP 客户端创建失败！错误码: 0x%08"PRIx32"\n", (uint32_t)result);
            goto retry;
        }
        // printf("HTTP 客户端创建成功\r\n");

        /* 连接到服务器 */
        result = cy_http_client_connect(http_handle, HTTP_REQUEST_TIMEOUT_MS, HTTP_REQUEST_TIMEOUT_MS);
        if(result != CY_RSLT_SUCCESS) {
            printf("连接服务器失败！错误码: 0x%08"PRIx32"\n", (uint32_t)result);
            goto cleanup;
        }
        // printf("成功连接到HTTP服务器\r\n");

        /* 配置 HTTP GET 请求 */
        memset(&request, 0, sizeof(request));
        request.buffer = response_buffer;
        request.buffer_len = HTTP_RESPONSE_BUFFER_SIZE;
        request.headers_len = 0;
        request.method = CY_HTTP_CLIENT_METHOD_GET;
        request.range_end = -1;
        request.range_start = 0;
        request.resource_path = HTTP_API_PATH;

        /* 设置请求头 */
        header[0].field = "Connection";
        header[0].field_len = strlen("Connection");
        header[0].value = "keep-alive";
        header[0].value_len = strlen("keep-alive");

        /* 生成标准头和用户定义头，并更新到请求结构中 */
        result = cy_http_client_write_header(http_handle, &request, &header[0], num_header);
        if(result != CY_RSLT_SUCCESS) {
            printf("写入HTTP头失败！错误码: 0x%08"PRIx32"\n", (uint32_t)result);
            goto cleanup;
        }

        /* 发送请求并接收响应 */
        result = cy_http_client_send(http_handle, &request, NULL, 0, &response);
        if(result == CY_RSLT_SUCCESS) {
            // printf("HTTP 响应状态: %d\n", response.status_code);
            // printf("响应数据: %.*s\n", (int)response.body_len, (char*)response.body);

            /* 解析 JSON 响应（假设格式为 {"status":1}） */
            if(strstr((char*)response.body, "1") != NULL) {
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);  // 开灯
                // printf("LED 已打开\r\n");
            } 
            else if(strstr((char*)response.body, "0") != NULL) {
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF); // 关灯
                // printf("LED 已关闭\r\n");
            }
        } else {
            printf("HTTP 请求失败！错误码: 0x%08"PRIx32"\n", (uint32_t)result);
        }

        /* 清理资源 */
        cleanup:
            result = cy_http_client_disconnect(http_handle);
            if(result != CY_RSLT_SUCCESS) {
                printf("断开HTTP连接失败！错误码: 0x%08"PRIx32"\n", (uint32_t)result);
            }
            
            result = cy_http_client_delete(http_handle);
            if(result != CY_RSLT_SUCCESS) {
                printf("删除HTTP客户端失败！错误码: 0x%08"PRIx32"\n", (uint32_t)result);
            }

        /* 等待下次轮询 */
        retry:
            cy_rtos_delay_milliseconds(HTTP_POLL_INTERVAL_MS);
    }
}

#if(!USE_AP_INTERFACE)
/*******************************************************************************
 * Function Name: connect_to_wifi_ap()
 *******************************************************************************
 * Summary:
 *  Connects to Wi-Fi AP using the user-configured credentials, retries up to a
 *  configured number of times until the connection succeeds.
 *
 *******************************************************************************/
cy_rslt_t connect_to_wifi_ap(void)
{
    cy_rslt_t result;
    char ip_addr_str[UART_BUFFER_SIZE];

    /* Variables used by Wi-Fi connection manager.*/
    cy_wcm_connect_params_t wifi_conn_param;

    cy_wcm_ip_address_t ip_address;

    /* IP variable for network utility functions */
    cy_nw_ip_address_t nw_ip_addr =
    {
        .version = NW_IP_IPV4
    };

     /* Set the Wi-Fi SSID, password and security type. */
    memset(&wifi_conn_param, 0, sizeof(cy_wcm_connect_params_t));
    memcpy(wifi_conn_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
    memcpy(wifi_conn_param.ap_credentials.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
    wifi_conn_param.ap_credentials.security = WIFI_SECURITY_TYPE;

    printf("Connecting to Wi-Fi Network: %s\n", WIFI_SSID);

    /* Join the Wi-Fi AP. */
    for(uint32_t conn_retries = 0; conn_retries < MAX_WIFI_CONN_RETRIES; conn_retries++ )
    {
        result = cy_wcm_connect_ap(&wifi_conn_param, &ip_address);

        if(result == CY_RSLT_SUCCESS)
        {
            printf("Successfully connected to Wi-Fi network '%s'.\n",
                                wifi_conn_param.ap_credentials.SSID);
            nw_ip_addr.ip.v4 = ip_address.ip.v4;
            cy_nw_ntoa(&nw_ip_addr, ip_addr_str);
            printf("IP Address Assigned: %s\n", ip_addr_str);
            return result;
        }

        printf("Connection to Wi-Fi network failed with error code %d."
               "Retrying in %d ms...\n", (int)result, WIFI_CONN_RETRY_INTERVAL_MSEC);

        cy_rtos_delay_milliseconds(WIFI_CONN_RETRY_INTERVAL_MSEC);
    }

    /* Stop retrying after maximum retry attempts. */
    printf("Exceeded maximum Wi-Fi connection attempts\n");

    return result;
}
#endif /* USE_AP_INTERFACE */

#if(USE_AP_INTERFACE)
/********************************************************************************
 * Function Name: softap_start
 ********************************************************************************
 * Summary:
 *  This function configures device in AP mode and initializes
 *  a SoftAP with the given credentials (SOFTAP_SSID, SOFTAP_PASSWORD and
 *  SOFTAP_SECURITY_TYPE).
 *
 * Parameters:
 *  void
 *
 * Return:
 *  cy_rslt_t: Returns CY_RSLT_SUCCESS if the Soft AP is started successfully,
 *  a WCM error code otherwise.
 *
 *******************************************************************************/
static cy_rslt_t softap_start(void)
{
    cy_rslt_t result;
    char ip_addr_str[UART_BUFFER_SIZE];

    /* IP variable for network utility functions */
    cy_nw_ip_address_t nw_ip_addr =
    {
        .version = NW_IP_IPV4
    };

    /* Initialize the Wi-Fi device as a Soft AP. */
    cy_wcm_ap_credentials_t softap_credentials = {SOFTAP_SSID, SOFTAP_PASSWORD,
                                                  SOFTAP_SECURITY_TYPE};
    cy_wcm_ip_setting_t softap_ip_info = {
        .ip_address = {.version = CY_WCM_IP_VER_V4, .ip.v4 = SOFTAP_IP_ADDRESS},
        .gateway = {.version = CY_WCM_IP_VER_V4, .ip.v4 = SOFTAP_GATEWAY},
        .netmask = {.version = CY_WCM_IP_VER_V4, .ip.v4 = SOFTAP_NETMASK}};

    cy_wcm_ap_config_t softap_config = {softap_credentials, SOFTAP_RADIO_CHANNEL,
                                        softap_ip_info,
                                        NULL};

    /* Start the the Wi-Fi device as a Soft AP. */
    result = cy_wcm_start_ap(&softap_config);

    if(result == CY_RSLT_SUCCESS)
    {
        printf("Wi-Fi Device configured as Soft AP\n");
        printf("Connect TCP client device to the network: SSID: %s Password:%s\n",
                SOFTAP_SSID, SOFTAP_PASSWORD);
        nw_ip_addr.ip.v4 = softap_ip_info.ip_address.ip.v4;
        cy_nw_ntoa(&nw_ip_addr, ip_addr_str);
        printf("SofAP IP Address : %s\n\n", ip_addr_str);
    }

    return result;
}
#endif /* USE_AP_INTERFACE */

/*******************************************************************************
 * Function Name: create_tcp_client_socket
 *******************************************************************************
 * Summary:
 *  Function to create a socket and set the socket options
 *  to set call back function for handling incoming messages, call back
 *  function to handle disconnection.
 *
 *******************************************************************************/
cy_rslt_t create_tcp_client_socket()
{
    cy_rslt_t result;

    /* TCP keep alive parameters. */
    int keep_alive = 1;
#if defined (COMPONENT_LWIP)
    uint32_t keep_alive_interval = TCP_KEEP_ALIVE_INTERVAL_MS;
    uint32_t keep_alive_count    = TCP_KEEP_ALIVE_RETRY_COUNT;
    uint32_t keep_alive_idle_time = TCP_KEEP_ALIVE_IDLE_TIME_MS;
#endif

    /* Variables used to set socket options. */
    cy_socket_opt_callback_t tcp_recv_option;
    cy_socket_opt_callback_t tcp_disconnect_option;

    /* Create a new secure TCP socket. */
    result = cy_socket_create(CY_SOCKET_DOMAIN_AF_INET, CY_SOCKET_TYPE_STREAM,
                              CY_SOCKET_IPPROTO_TCP, &client_handle);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("Failed to create socket!\n");
        return result;
    }

    /* Register the callback function to handle messages received from TCP server. */
    tcp_recv_option.callback = tcp_client_recv_handler;
    tcp_recv_option.arg = NULL;
    result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_SOCKET,
                                  CY_SOCKET_SO_RECEIVE_CALLBACK,
                                  &tcp_recv_option, sizeof(cy_socket_opt_callback_t));
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Set socket option: CY_SOCKET_SO_RECEIVE_CALLBACK failed\n");
        return result;
    }

    /* Register the callback function to handle disconnection. */
    tcp_disconnect_option.callback = tcp_disconnection_handler;
    tcp_disconnect_option.arg = NULL;

    result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_SOCKET,
                                  CY_SOCKET_SO_DISCONNECT_CALLBACK,
                                  &tcp_disconnect_option, sizeof(cy_socket_opt_callback_t));
    if(result != CY_RSLT_SUCCESS)
    {
        printf("Set socket option: CY_SOCKET_SO_DISCONNECT_CALLBACK failed\n");
    }

#if defined (COMPONENT_LWIP)
    /* Set the TCP keep alive interval. */
    result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_TCP,
                                  CY_SOCKET_SO_TCP_KEEPALIVE_INTERVAL,
                                  &keep_alive_interval, sizeof(keep_alive_interval));
    if(result != CY_RSLT_SUCCESS)
    {
        printf("Set socket option: CY_SOCKET_SO_TCP_KEEPALIVE_INTERVAL failed\n");
        return result;
    }

    /* Set the retry count for TCP keep alive packet. */
    result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_TCP,
                                  CY_SOCKET_SO_TCP_KEEPALIVE_COUNT,
                                  &keep_alive_count, sizeof(keep_alive_count));
    if(result != CY_RSLT_SUCCESS)
    {
        printf("Set socket option: CY_SOCKET_SO_TCP_KEEPALIVE_COUNT failed\n");
        return result;
    }

    /* Set the network idle time before sending the TCP keep alive packet. */
    result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_TCP,
                                  CY_SOCKET_SO_TCP_KEEPALIVE_IDLE_TIME,
                                  &keep_alive_idle_time, sizeof(keep_alive_idle_time));
    if(result != CY_RSLT_SUCCESS)
    {
        printf("Set socket option: CY_SOCKET_SO_TCP_KEEPALIVE_IDLE_TIME failed\n");
        return result;
    }
#endif

    /* Enable TCP keep alive. */
    result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_SOCKET,
                                      CY_SOCKET_SO_TCP_KEEPALIVE_ENABLE,
                                          &keep_alive, sizeof(keep_alive));
    if(result != CY_RSLT_SUCCESS)
    {
        printf("Set socket option: CY_SOCKET_SO_TCP_KEEPALIVE_ENABLE failed\n");
        return result;
    }

    return result;
}

/*******************************************************************************
 * Function Name: connect_to_tcp_server
 *******************************************************************************
 * Summary:
 *  Function to connect to TCP server.
 *
 * Parameters:
 *  cy_socket_sockaddr_t address: Address of TCP server socket
 *
 * Return:
 *  cy_result result: Result of the operation
 *
 *******************************************************************************/
cy_rslt_t connect_to_tcp_server(cy_socket_sockaddr_t address)
{
    cy_rslt_t result = CY_RSLT_MODULE_SECURE_SOCKETS_TIMEOUT;
    cy_rslt_t conn_result;

    for(uint32_t conn_retries = 0; conn_retries < MAX_TCP_SERVER_CONN_RETRIES; conn_retries++)
    {
        /* Create a TCP socket */
        conn_result = create_tcp_client_socket();

        if(conn_result != CY_RSLT_SUCCESS)
        {
            printf("Socket creation failed!\n");
            CY_ASSERT(0);
        }

        conn_result = cy_socket_connect(client_handle, &address, sizeof(cy_socket_sockaddr_t));

        if (conn_result == CY_RSLT_SUCCESS)
        {
            printf("============================================================\n");
            printf("Connected to TCP server\n");

            return conn_result;
        }

        printf("Could not connect to TCP server. Error code: 0x%08"PRIx32"\n", (uint32_t)result);
        printf("Trying to reconnect to TCP server... Please check if the server is listening\n");

        /* The resources allocated during the socket creation (cy_socket_create)
         * should be deleted.
         */
        cy_socket_delete(client_handle);
    }

     /* Stop retrying after maximum retry attempts. */
     printf("Exceeded maximum connection attempts to the TCP server\n");

     return result;
}

/*******************************************************************************
 * Function Name: tcp_client_recv_handler
 *******************************************************************************
 * Summary:
 *  Callback function to handle incoming TCP server messages.
 *
 * Parameters:
 *  cy_socket_t socket_handle: Connection handle for the TCP client socket
 *  void *args : Parameter passed on to the function (unused)
 *
 * Return:
 *  cy_result result: Result of the operation
 *
 *******************************************************************************/
cy_rslt_t tcp_client_recv_handler(cy_socket_t socket_handle, void *arg)
{
    /* Variable to store number of bytes send to the TCP server. */
    uint32_t bytes_sent = 0;

    /* Variable to store number of bytes received. */
    uint32_t bytes_received = 0;

    char message_buffer[MAX_TCP_DATA_PACKET_LENGTH];
    cy_rslt_t result ;

    printf("============================================================\n");
    result = cy_socket_recv(socket_handle, message_buffer, TCP_LED_CMD_LEN,
                            CY_SOCKET_FLAGS_NONE, &bytes_received);

    if(message_buffer[0] == LED_ON_CMD)
    {
        /* Turn the LED ON. */
        cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
        printf("LED turned ON\n");
        sprintf(message_buffer, ACK_LED_ON);
    }
    else if(message_buffer[0] == LED_OFF_CMD)
    {
        /* Turn the LED OFF. */
        cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
        printf("LED turned OFF\n");
        sprintf(message_buffer, ACK_LED_OFF);
    }
    else
    {
        printf("Invalid command\n");
        sprintf(message_buffer, MSG_INVALID_CMD);
    }

    /* Send acknowledgment to the TCP server in receipt of the message received. */
    result = cy_socket_send(socket_handle, message_buffer, strlen(message_buffer),
                            CY_SOCKET_FLAGS_NONE, &bytes_sent);
    if(result == CY_RSLT_SUCCESS)
    {
        printf("Acknowledgment sent to TCP server\n");
    }

    return result;
}

/*******************************************************************************
 * Function Name: tcp_disconnection_handler
 *******************************************************************************
 * Summary:
 *  Callback function to handle TCP socket disconnection event.
 *
 * Parameters:
 *  cy_socket_t socket_handle: Connection handle for the TCP client socket
 *  void *args : Parameter passed on to the function (unused)
 *
 * Return:
 *  cy_result result: Result of the operation
 *
 *******************************************************************************/
cy_rslt_t tcp_disconnection_handler(cy_socket_t socket_handle, void *arg)
{
    cy_rslt_t result;

    /* Disconnect the TCP client. */
    result = cy_socket_disconnect(socket_handle, 0);

    /* Free the resources allocated to the socket. */
    cy_socket_delete(socket_handle);

    printf("Disconnected from the TCP server! \n");

    /* Give the semaphore so as to connect to TCP server. */
    cy_rtos_semaphore_set(&connect_to_server);

    return result;
}

/*******************************************************************************
 * Function Name: read_uart_input
 *******************************************************************************
 * Summary:
 *  Function to read user input from UART terminal.
 *
 * Parameters:
 *  uint8_t* input_buffer_ptr: Pointer to input buffer
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void read_uart_input(uint8_t* input_buffer_ptr)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    uint8_t *ptr = input_buffer_ptr;
    uint32_t numBytes;

    do
    {
        /* Check for data in the UART buffer with zero timeout. */
        numBytes = cyhal_uart_readable(&cy_retarget_io_uart_obj);

        if(numBytes > 0)
        {
            result = cyhal_uart_getc(&cy_retarget_io_uart_obj, ptr,
                            UART_INPUT_TIMEOUT_MS);

            if(result == CY_RSLT_SUCCESS)
            {
                if((*ptr == '\r') || (*ptr == '\n'))
                {
                    printf("\n");
                }
                else
                {
                    /* Echo the received character */
                    cyhal_uart_putc(&cy_retarget_io_uart_obj, *ptr);

                    if (*ptr != '\b')
                    {
                        ptr++;
                    }
                    else if(ptr != input_buffer_ptr)
                    {
                        ptr--;
                    }
                }
            }
        }

        cy_rtos_delay_milliseconds(RTOS_TICK_TO_WAIT);

    } while((*ptr != '\r') && (*ptr != '\n'));

    /* Terminate string with NULL character. */
    *ptr = '\0';
}


/* [] END OF FILE */
