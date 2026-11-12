#ifndef LWIP_LWIPOPTS_H
#define LWIP_LWIPOPTS_H

#define NO_SYS                          1
#define SYS_LIGHTWEIGHT_PROT            0
#define LWIP_SOCKET                     0
#define LWIP_NETCONN                    0
#define LWIP_TCPIP_CORE_LOCKING         0

#define LWIP_IPV4                       1
#define LWIP_IPV6                       0
#define LWIP_ARP                        1
#define LWIP_ICMP                       1
#define LWIP_RAW                        0
#define LWIP_UDP                        1
#define LWIP_TCP                        1
#define LWIP_DHCP                       1
#define LWIP_DHCP_DOES_ACD_CHECK        0
#define LWIP_AUTOIP                     0
#define LWIP_IGMP                       1
#define LWIP_DNS                        0

#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        (80 * 1024)
#define MEMP_NUM_PBUF                   8
#define MEMP_NUM_UDP_PCB                6
#define MEMP_NUM_TCP_PCB                4
#define MEMP_NUM_TCP_PCB_LISTEN         2
#define MEMP_NUM_TCP_SEG                96
#define MEMP_NUM_SYS_TIMEOUT            12
#define MEMP_NUM_IGMP_GROUP             4
#define PBUF_POOL_SIZE                  10
#define PBUF_POOL_BUFSIZE               1536

#define TCP_MSS                         1460
#define TCP_SND_BUF                     (43 * TCP_MSS)
#define TCP_WND                         (8 * TCP_MSS)
#define TCP_SND_QUEUELEN                96
#define LWIP_TCP_KEEPALIVE              1

#define LWIP_NETIF_HOSTNAME             1
#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_NETIF_LINK_CALLBACK        1
#define LWIP_MDNS_RESPONDER             1
#define MDNS_MAX_SERVICES               1
#define LWIP_NUM_NETIF_CLIENT_DATA      1

#define CHECKSUM_GEN_IP                 1
#define CHECKSUM_GEN_UDP                1
#define CHECKSUM_GEN_TCP                1
#define CHECKSUM_GEN_ICMP               1
#define CHECKSUM_CHECK_IP               1
#define CHECKSUM_CHECK_UDP              1
#define CHECKSUM_CHECK_TCP              1
#define CHECKSUM_CHECK_ICMP             1

#define LWIP_STATS                      0
#define LWIP_DEBUG                      0

#endif
