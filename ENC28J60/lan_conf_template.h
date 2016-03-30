#ifndef LAN_CONF_H
#define LAN_CONF_H 020

/**************************************************************************/
/**************************************************************************/
/**************************************************************************/
/*                                                                        */
/*   Edit file name to lan_conf.h and edit values for your platform   */
/*                                                                        */
/**************************************************************************/
/**************************************************************************/
/**************************************************************************/

/**
 * @defgroup LAN_CONF
 * @brief    Configuration parameters for LAN library
 * @{
 */

/*
 * Options
 */
/**
 * @brief   This options allows you use ICMP protocol
 *
 */
#define WITH_ICMP

/**
 * @brief   This options run DHCP query for automatic network configuration
 *          Please refer LAN_Callback_DHCPGetLANConfig method to check new configuration
 */
#define WITH_DHCP

/**
 * @brief   This options run NTP query
 *          Please refer LAN_Callback_SNTPGetDateTime method to check new time configuration
 */
#define WITH_NTP

/**
 * @brief   Enables (1) or disables (0) UDP protocol
 */
#define WITH_UDP

/**
 * @brief   Enables (1) or disables (0) TCP protocol
 */
#define WITH_TCP
#define WITH_TCP_REXMIT

/**
 * @brief   Maximal cache size for ARP
 *
 */
#define ARP_CACHE_SIZE			3

/**
 * @brief   Set TTL value for ethernet frames
 *
 */
#define IP_PACKET_TTL			64

/**
 * @brief   Maximal TCP connections
 *
 */
#define TCP_MAX_CONNECTIONS		5

/**
 * @brief   Window size for TCP frames
 *
 */
#define TCP_WINDOW_SIZE			65535

#define TCP_SYN_MSS				512

#ifdef WITH_TCP_REXMIT
#	define TCP_REXMIT_TIMEOUT	1000
#	define TCP_REXMIT_LIMIT		5
#else
#	define TCP_CONN_TIMEOUT		2500
#endif

/**
 * @brief   Default MAC address
 *
 */
#define MAC_ADDR				{0x00,0x13,0x37,0x01,0x23,0x45}

/**
 * @brief   Default IP address
 *
 */
#define IP_ADDR					inet_addr(10,1,20,55)

/**
 * @brief   Default Netmask
 *
 */
#define IP_SUBNET_MASK			inet_addr(255,255,255,0)

/**
 * @brief   Default Gateway
 *
 */
#define IP_DEFAULT_GATEWAY		inet_addr(10,1,20,1)

/**
 * @brief   SNTP server address as IPv4 address in "u32_t" format
 *
 */
#define SNTP_SERVER_ADDRESS		inet_addr(213,161,194,93) /* pool.ntp.org */
//#define SNTP_SERVER_ADDRESS		inet_addr(213,199,255,30) /* ntp.tktelekom.pl */

/**
 * @}
 */

#endif // LAN_CONF_H
