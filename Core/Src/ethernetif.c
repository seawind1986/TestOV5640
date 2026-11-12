#include "main.h"
#include "ov5640.h"

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/etharp.h"
#include "netif/ethernet.h"

#include <string.h>

#define ETH_RX_BUF_SIZE 1536U

ETH_HandleTypeDef heth;
ETH_TxPacketConfigTypeDef TxConfig;

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".bss.ARM.__at_0x30000000"), aligned(32)));
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".bss.ARM.__at_0x30000080"), aligned(32)));

static uint8_t rx_buf[ETH_RX_DESC_CNT][ETH_RX_BUF_SIZE] __attribute__((section(".bss.ARM.__at_0x30000100"), aligned(32)));
static uint8_t rx_used[ETH_RX_DESC_CNT];
static uint8_t tx_frame[ETH_RX_BUF_SIZE] __attribute__((section(".bss.ARM.__at_0x30001900"), aligned(32)));
static uint8_t mac_addr[6];
volatile uint32_t eth_tx_n;
volatile uint32_t eth_tx_ms;
volatile uint32_t eth_tx_max;
volatile uint32_t eth_tx_fail;
static uint32_t phy_addr;
static uint32_t phy_id;
static uint8_t mac_ok;

static err_t low_level_output(struct netif *netif, struct pbuf *p);
void ETH_MspResetPhy(void);

static uint32_t phy_read(uint32_t reg)
{
  uint32_t value = 0U;
  if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, reg, &value) != HAL_OK)
  {
    return 0xFFFFFFFFU;
  }
  return value;
}

static void phy_write(uint32_t reg, uint32_t value)
{
  (void)HAL_ETH_WritePHYRegister(&heth, phy_addr, reg, value);
}

static void phy_find(void)
{
  uint32_t addr;
  phy_addr = 0U;
  phy_id = 0U;
  for (addr = 0U; addr < 32U; addr++)
  {
    uint32_t id1 = 0U;
    uint32_t id2 = 0U;
    if (HAL_ETH_ReadPHYRegister(&heth, addr, 2U, &id1) != HAL_OK)
    {
      continue;
    }
    if (HAL_ETH_ReadPHYRegister(&heth, addr, 3U, &id2) != HAL_OK)
    {
      continue;
    }
    if ((id1 == 0xFFFFU) || (id2 == 0xFFFFU) || ((id1 == 0U) && (id2 == 0U)))
    {
      continue;
    }
    phy_addr = addr;
    phy_id = (id1 << 16) | (id2 & 0xFFFFU);
    return;
  }
}

static void phy_ext_write(uint32_t reg, uint32_t value)
{
  phy_write(0x1EU, reg);
  phy_write(0x1FU, value);
}

static uint32_t phy_ext_read(uint32_t reg)
{
  phy_write(0x1EU, reg);
  return phy_read(0x1FU);
}

void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
  uint32_t i;
  *buff = NULL;
  for (i = 0U; i < ETH_RX_DESC_CNT; i++)
  {
    if (rx_used[i] == 0U)
    {
      rx_used[i] = 1U;
      *buff = rx_buf[i];
      return;
    }
  }
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length)
{
  struct pbuf *p;
  uint32_t i;

  p = pbuf_alloc(PBUF_RAW, Length, PBUF_POOL);
  if (p != NULL)
  {
    (void)memcpy(p->payload, buff, Length);
    if (*pStart == NULL)
    {
      *pStart = p;
    }
    else
    {
      pbuf_cat((struct pbuf *)*pEnd, p);
    }
    *pEnd = p;
  }

  for (i = 0U; i < ETH_RX_DESC_CNT; i++)
  {
    if (buff == rx_buf[i])
    {
      rx_used[i] = 0U;
      break;
    }
  }
}

static void eth_hw_init(uint8_t *hwaddr)
{
  ETH_MACFilterConfigTypeDef filter = {0};
  uint32_t id = HAL_GetUIDw0() ^ HAL_GetUIDw1();

  mac_addr[0] = 0x02U;
  mac_addr[1] = 0x80U;
  mac_addr[2] = 0xE1U;
  mac_addr[3] = (uint8_t)(id >> 16);
  mac_addr[4] = (uint8_t)(id >> 8);
  mac_addr[5] = (uint8_t)id;
  (void)memcpy(hwaddr, mac_addr, 6);

  heth.Instance = ETH;
  heth.Init.MACAddr = mac_addr;
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = ETH_RX_BUF_SIZE;
  heth.gState = HAL_ETH_STATE_RESET;
  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    heth.gState = HAL_ETH_STATE_RESET;
    ETH_MspResetPhy();
    if (HAL_ETH_Init(&heth) != HAL_OK)
    {
      mac_ok = 0U;
      return;
    }
  }

  memset(&TxConfig, 0, sizeof(TxConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_DISABLE;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

  (void)HAL_ETH_GetMACFilterConfig(&heth, &filter);
  filter.PassAllMulticast = ENABLE;
  filter.BroadcastFilter = DISABLE;
  (void)HAL_ETH_SetMACFilterConfig(&heth, &filter);

  phy_find();
  if (phy_id != 0U)
  {
    uint32_t sleep = phy_ext_read(0x2027U);
    phy_write(0U, 0x8000U);
    HAL_Delay(50);
    if (sleep != 0xFFFFFFFFU)
    {
      phy_ext_write(0x2027U, sleep & ~(1UL << 15));
    }
    phy_write(0U, 0x1200U);
  }
  mac_ok = 1U;
}

void ETH_MspResetPhy(void)
{
  /* YT8512C 复位低有效，直连 PH5。高电平退出复位。 */
  OV5640_BoardIo(7U, 0U);
  HAL_Delay(20);
  OV5640_BoardIo(7U, 1U);
  HAL_Delay(100);
}

err_t ethernetif_init(struct netif *netif)
{
  netif->hwaddr_len = ETH_HWADDR_LEN;
  netif->mtu = 1500;
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_IGMP;
  netif->name[0] = 'e';
  netif->name[1] = 'n';
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;
  eth_hw_init(netif->hwaddr);
  return ERR_OK;
}

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  ETH_BufferTypeDef tx = {0};
  uint16_t len;

  (void)netif;
  len = pbuf_copy_partial(p, tx_frame, ETH_RX_BUF_SIZE, 0);
  if ((len == 0U) || (len < p->tot_len))
  {
    return ERR_BUF;
  }
  tx.buffer = tx_frame;
  tx.len = len;
  TxConfig.Length = len;
  TxConfig.TxBuffer = &tx;
  TxConfig.pData = NULL;
  /* 完成标志在外设寄存器里。紧盯 SRAM 里的描述符会把 DMA 写回拖到十几毫秒。 */
  heth.Instance->DMACSR = ETH_DMACSR_TI;
  if (HAL_ETH_Transmit_IT(&heth, &TxConfig) != HAL_OK)
  {
    return ERR_IF;
  }
  {
    uint32_t t0 = HAL_GetTick();
    uint32_t prev = heth.TxDescList.CurTxDesc;
    uint32_t dt;
    ETH_DMADescTypeDef *done;

    prev = (prev == 0U) ? ((uint32_t)ETH_TX_DESC_CNT - 1U) : (prev - 1U);
    done = (ETH_DMADescTypeDef *)heth.TxDescList.TxDesc[prev];
    while ((heth.Instance->DMACSR & ETH_DMACSR_TI) == 0U)
    {
      if ((HAL_GetTick() - t0) > 5U)
      {
        eth_tx_fail++;
        done->DESC3 = (ETH_DMATXNDESCWBF_FD | ETH_DMATXNDESCWBF_LD);
        return ERR_IF;
      }
    }
    dt = HAL_GetTick() - t0;
    eth_tx_n++;
    eth_tx_ms += dt;
    if (dt > eth_tx_max)
    {
      eth_tx_max = dt;
    }
  }
  return ERR_OK;
}

void ethernetif_input(struct netif *netif)
{
  void *packet = NULL;
  while (HAL_ETH_ReadData(&heth, &packet) == HAL_OK)
  {
    struct pbuf *p = (struct pbuf *)packet;
    packet = NULL;
    if (p == NULL)
    {
      continue;
    }
    if (netif->input(p, netif) != ERR_OK)
    {
      pbuf_free(p);
    }
  }
}

static void mac_wait_idle(void)
{
  uint32_t t0 = HAL_GetTick();

  /* 速率位只有在收发都关掉之后才会写进去。 */
  while ((heth.Instance->MACCR & (ETH_MACCR_TE | ETH_MACCR_RE)) != 0U)
  {
    if ((HAL_GetTick() - t0) > 20U)
    {
      break;
    }
  }
}

void ethernetif_set_link(uint32_t speed_100m, uint32_t full_duplex, uint32_t up)
{
  ETH_MACConfigTypeDef mac = {0};
  uint32_t fes;
  uint32_t dm;

  if (up == 0U)
  {
    if (heth.gState == HAL_ETH_STATE_STARTED)
    {
      (void)HAL_ETH_Stop(&heth);
    }
    return;
  }

  if (heth.gState == HAL_ETH_STATE_STARTED)
  {
    (void)HAL_ETH_Stop(&heth);
  }
  mac_wait_idle();
  if (HAL_ETH_GetMACConfig(&heth, &mac) == HAL_OK)
  {
    mac.Speed = (speed_100m != 0U) ? ETH_SPEED_100M : ETH_SPEED_10M;
    mac.DuplexMode = (full_duplex != 0U) ? ETH_FULLDUPLEX_MODE : ETH_HALFDUPLEX_MODE;
    (void)HAL_ETH_SetMACConfig(&heth, &mac);
  }
  fes = (speed_100m != 0U) ? ETH_MACCR_FES : 0U;
  dm = (full_duplex != 0U) ? ETH_MACCR_DM : 0U;
  MODIFY_REG(heth.Instance->MACCR, ETH_MACCR_FES | ETH_MACCR_DM, fes | dm);
  if (heth.gState == HAL_ETH_STATE_READY)
  {
    (void)HAL_ETH_Start(&heth);
  }
}

uint32_t ethernetif_speed_matches(uint32_t speed_100m, uint32_t full_duplex)
{
  uint32_t cr = heth.Instance->MACCR;
  uint32_t fes = (speed_100m != 0U) ? ETH_MACCR_FES : 0U;
  uint32_t dm = (full_duplex != 0U) ? ETH_MACCR_DM : 0U;

  return ((cr & (ETH_MACCR_FES | ETH_MACCR_DM)) == (fes | dm)) ? 1U : 0U;
}

uint32_t ethernetif_link_up(uint32_t *speed_100m, uint32_t *full_duplex)
{
  uint32_t bsr;
  uint32_t status;
  uint32_t speed;

  if ((mac_ok == 0U) || (phy_id == 0U))
  {
    return 0U;
  }
  bsr = phy_read(1U);
  bsr = phy_read(1U);
  if ((bsr == 0xFFFFFFFFU) || ((bsr & 0x0004U) == 0U))
  {
    return 0U;
  }
  if ((phy_id >> 16) == 0x0007U)
  {
    status = phy_read(0x1FU);
    if ((status == 0xFFFFFFFFU) || ((status & 0x1000U) == 0U))
    {
      return 0U;
    }
    switch (status & 0x001CU)
    {
      case 0x0018U:
        *speed_100m = 1U;
        *full_duplex = 1U;
        break;
      case 0x0008U:
        *speed_100m = 1U;
        *full_duplex = 0U;
        break;
      case 0x0014U:
        *speed_100m = 0U;
        *full_duplex = 1U;
        break;
      default:
        *speed_100m = 0U;
        *full_duplex = 0U;
        break;
    }
    return 1U;
  }
  status = phy_read(0x11U);
  if ((status == 0xFFFFFFFFU) || ((status & 0x0400U) == 0U))
  {
    return 0U;
  }
  if ((status & 0x0800U) != 0U)
  {
    speed = (status >> 14) & 0x3U;
    *speed_100m = (speed == 0x1U) ? 1U : 0U;
    *full_duplex = ((status & 0x2000U) != 0U) ? 1U : 0U;
    return 1U;
  }
  /* 链路已经起来，速率位还没锁存时先按路由器的 100M 全双工。 */
  *speed_100m = 1U;
  *full_duplex = 1U;
  return 1U;
}

uint32_t ethernetif_mac_ok(void)
{
  return mac_ok;
}

uint32_t ethernetif_phy_id(void)
{
  return phy_id;
}
