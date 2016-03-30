#ifndef LAN_H
#define LAN_H 100

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif
#include <time.h>
#include "attributes.h"
#include "enc28j60.h"
#include "lan_conf.h"

extern uint32_t ip_addr;
extern uint32_t ip_mask;
extern uint32_t ip_gateway;

/*
 * BE conversion
 */
#define htons(a)			((((a)>>8)&0xff)|(((a)<<8)&0xff00))
#define ntohs(a)			htons(a)
#define htonl(a)			( (((a)>>24)&0xff) | (((a)>>8)&0xff00) | (((a)<<8)&0xff0000) | (((a)<<24)&0xff000000) )
#define ntohl(a)			htonl(a)

#define inet_addr(a,b,c,d)	( ((uint32_t)a) | ((uint32_t)b << 8) | ((uint32_t)c << 16) | ((uint32_t)d << 24) )

#define INET_ADDR_A(addr)	((int)(addr & 0xff))
#define INET_ADDR_B(addr)	((int)((addr>>8) & 0xff))
#define INET_ADDR_C(addr)	((int)((addr>>16) & 0xff))
#define INET_ADDR_D(addr)	((int)((addr>>24) & 0xff))

/*
 * Ethernet
 */
#define ETH_TYPE_ARP		htons(0x0806)
#define ETH_TYPE_IP			htons(0x0800)

#pragma pack(push, 1)
typedef struct eth_frame {
	uint8_t to_addr[6];
	uint8_t from_addr[6];
	uint16_t type;
	uint8_t data[];
} eth_frame_t;
#pragma pack(pop)

/*
 * ARP
 */

#define ARP_HW_TYPE_ETH		htons(0x0001)
#define ARP_PROTO_TYPE_IP	htons(0x0800)

#define ARP_TYPE_REQUEST	htons(1)
#define ARP_TYPE_RESPONSE	htons(2)

#pragma pack(push, 1)
typedef struct arp_message {
	uint16_t hw_type;
	uint16_t proto_type;
	uint8_t hw_addr_len;
	uint8_t proto_addr_len;
	uint16_t type;
	uint8_t mac_addr_from[6];
	uint32_t ip_addr_from;
	uint8_t mac_addr_to[6];
	uint32_t ip_addr_to;
} arp_message_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct arp_cache_entry {
	uint32_t ip_addr;
	uint8_t mac_addr[6];
} arp_cache_entry_t;

/*
 * IP
 */
#define IP_PROTOCOL_ICMP	1
#define IP_PROTOCOL_TCP		6
#define IP_PROTOCOL_UDP		17

#pragma pack(push, 1)
typedef struct ip_packet {
	uint8_t ver_head_len;
	uint8_t tos;
	uint16_t total_len;
	uint16_t fragment_id;
	uint16_t flags_framgent_offset;
	uint8_t ttl;
	uint8_t protocol;
	uint16_t cksum;
	uint32_t from_addr;
	uint32_t to_addr;
	uint8_t data[];
} ip_packet_t;
#pragma pack(pop)

#ifdef WITH_ICMP
/*----------------------------------------------------------
 *                       ICMP
 *----------------------------------------------------------*/
#define ICMP_TYPE_ECHO_RQ	8
#define ICMP_TYPE_ECHO_RPLY	0

#pragma pack(push, 1)
typedef struct icmp_echo_packet {
	uint8_t type;
	uint8_t code;
	uint16_t cksum;
	uint16_t id;
	uint16_t seq;
	uint8_t data[];
} icmp_echo_packet_t;
#pragma pack(pop)

#endif // WITH_ICMP

#ifdef WITH_UDP
/*----------------------------------------------------------
 *                       UDP
 *----------------------------------------------------------*/
#pragma pack(push, 1)
typedef struct udp_packet {
	uint16_t from_port;
	uint16_t to_port;
	uint16_t len;
	uint16_t cksum;
	uint8_t data[];
} udp_packet_t;
#pragma pack(pop)

/**
 * \brief  LAN callback with new UDP packet
 * \note
 * \param  *frame: pointer to ETH frame with UDP data (need to be custom)
 * \param  len: len is UDP data payload length
 * \note   With weak parameter to prevent link errors if not defined by user
 */
void LAN_Callback_UDPPacket(eth_frame_t *frame, uint16_t len);

/**
 * \brief  LAN send UDP packet
 * \note fields must be set: ip.dst, udp.src_port, udp.dst_port
 * \param  *frame: pointer to eth frame
 * \param  len: len is UDP data payload length
 * \retval
 *           0 - some error occurred
 *           1 - success
 */
uint8_t LAN_UDPSend(eth_frame_t *frame, uint16_t len);

/**
 * \brief  LAN reply to UDP packet
 * \note
 * \param  *frame: pointer to ETH frame
 * \param  len: len is UDP data payload length
 * \note   With weak parameter to prevent link errors if not defined by user
 */
void LAN_UDPReply(eth_frame_t *frame, uint16_t len);

#endif // WITH_UDP

#ifdef WITH_TCP
/*----------------------------------------------------------
 *                       TCP
 *----------------------------------------------------------*/

#pragma pack(push, 1)
typedef struct tcp_packet {
	uint16_t from_port;
	uint16_t to_port;
	uint32_t seq_num;
	uint32_t ack_num;
	uint8_t data_offset;

#define TCP_FLAG_URG		0x20
#define TCP_FLAG_ACK		0x10
#define TCP_FLAG_PSH		0x08
#define TCP_FLAG_RST		0x04
#define TCP_FLAG_SYN		0x02
#define TCP_FLAG_FIN		0x01
	uint8_t flags;
	uint16_t window;
	uint16_t cksum;
	uint16_t urgent_ptr;
	uint8_t data[];
} tcp_packet_t;
#pragma pack(pop)

#define tcp_head_size(tcp)	(((tcp)->data_offset & 0xf0) >> 2)
#define tcp_get_data(tcp)	((uint8_t*)(tcp) + tcp_head_size(tcp))

#pragma pack(push, 1)
typedef enum tcp_status_code {
	TCP_CLOSED,
	TCP_SYN_SENT,
	TCP_SYN_RECEIVED,
	TCP_ESTABLISHED,
	TCP_FIN_WAIT
} tcp_status_code_t;

#pragma pack(push, 1)
typedef struct tcp_state {
	tcp_status_code_t status;
	uint32_t event_time;
	uint32_t seq_num;
	uint32_t ack_num;
	uint32_t remote_addr;
	uint16_t remote_port;
	uint16_t local_port;
#ifdef WITH_TCP_REXMIT
	uint8_t is_closing;
	uint8_t rexmit_count;
	uint32_t seq_num_saved;
#endif
} tcp_state_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef enum tcp_sending_mode {
	TCP_SENDING_SEND,
	TCP_SENDING_REPLY,
	TCP_SENDING_RESEND
} tcp_sending_mode_t;
#pragma pack(pop)

#define TCP_OPTION_PUSH			0x01
#define TCP_OPTION_CLOSE		0x02

// TCP callbacks
/**
 * \brief  LAN callback with new connection request
 * \note
 * \param  id: connection identifier
 * \param  *frame: pointer to ETH frame
 * \note   With weak parameter to prevent link errors if not defined by user
 */
uint8_t LAN_Callback_TCPListen(uint8_t id, eth_frame_t *frame);

/**
 * \brief  LAN callback put some data from application to socket
 * \note   this mehod will be called if is possible to put some data to socket
 * \param  id: connection identifier
 * \param  *frame: pointer to ETH frame
 * \param  *re: ???
 * \note   With weak parameter to prevent link errors if not defined by user
 */
void LAN_Callback_TCPRead(uint8_t id, eth_frame_t *frame, uint8_t re);

/**
 * \brief  LAN feed data to application
 * \note   callback with data from remote host to local
 * \param  id: connection identifier
 * \param  *frame: pointer to ETH frame
 * \param  len: data length
 * \note   With weak parameter to prevent link errors if not defined by user
 */
void LAN_Callback_TCPWrite(uint8_t id, eth_frame_t *frame, uint16_t len);

/**
 * \brief  LAN close socket event
 * \note   callback when some connection are closed
 * \param  id: connection identifier
 * \param  hard: reason of close:
 *            0 -> remote host closed connection
 *            1 -> exception occurred
 * \note   With weak parameter to prevent link errors if not defined by user
 */
void LAN_Callback_TCPClosed(uint8_t id, uint8_t hard);

/**
 * \brief  open the connection
 * \note
 * \param  addr: remote host address
 * \param  port: remote host port
 * \param  local_port: local host port
 */
uint8_t LAN_TCPOpen(uint32_t addr, uint16_t port, uint16_t local_port);

/**
 * \brief  send TCP data to remote host
 * \note Don't use somewhere except tcp_write callback!
 * \param  id:
 * \param  *frame: pointer to ETH frame
 * \param  len: sizeof frame
 * \param  options:
 */
void LAN_TCPSend(uint8_t id, eth_frame_t *frame, uint16_t len, uint8_t options);

#endif // WITH_TCP

/*
 * LAN
 */
extern uint8_t net_buf[];

// LAN calls
void LAN_init(void);
void LAN_poll(void);;

#ifdef WITH_DHCP

typedef enum dhcp_status_code {
	DHCP_INIT,
	DHCP_ASSIGNED,
	DHCP_WAITING_OFFER,
	DHCP_WAITING_ACK
} dhcp_status_code_t;

/**
 * \brief  Get status for DHCP request
 * \note
 * \retval refer to dhcp_status_code_t enum
 */
dhcp_status_code_t LAN_GetDHCPStatus(void);

/**
 * \brief  LAN callback with new configuration from DHCP
 * \note
 * \param  ip: ip
 * \param  netmask:
 * \param  gateway:
 * \retval None
 * \note   With weak parameter to prevent link errors if not defined by user
 */
void LAN_Callback_DHCPGetLANConfig( uint32_t ip, uint32_t netmask, uint32_t gateway );

#endif // WITH_DHCP

#ifdef WITH_NTP

typedef enum ntp_status_code {
	NTP_INIT,
	NTP_CONNECTING,
	NTP_SYNCHRONIZED
} ntp_status_code_t;

/**
 * \brief  Get status for NTP
 * \note
 * \retval refer to ntp_status_code_t enum
 */
ntp_status_code_t LAN_GetNTPStatus(void);

/**
 * \brief  LAN callback about new time from SNTP
 * \note
 * \param  time: time from SNTP server
 * \retval None
 * \note   With weak parameter to prevent link errors if not defined by user
 */
void LAN_Callback_SNTPGetDateTime(time_t time);

#endif // WITH_NTP

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif // LAN_H

