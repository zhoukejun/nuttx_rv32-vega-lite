/****************************************************************************
 * apps/wireless/ieee802154/libmac/sixlowpan_setreq.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <nuttx/fs/ioctl.h>

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include "wireless/ieee802154.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sixlowpan_set_req(int sock, FAR const char *ifname,
                      FAR const struct ieee802154_set_req_s *req)
{
  struct ieee802154_netmac_s arg;
  int ret;

  strncpy(arg.ifr_name, ifname, IFNAMSIZ);
  memcpy(&arg.u.setreq, req, sizeof(struct ieee802154_set_req_s));

  ret = ioctl(sock, MAC802154IOC_MLME_SET_REQUEST,
              (unsigned long)((uintptr_t)&arg));
  if (ret < 0)
    {
      ret = -errno;
      fprintf(stderr, "MAC802154IOC_MLME_SET_REQUEST failed: %d\n", ret);
    }

  return ret;
}

/* Wrappers around set_req to make it even easier to set common attributes */

int sixlowpan_setchan(int sock, FAR const char *ifname, uint8_t chan)
{
  struct ieee802154_set_req_s req;

  req.attr = IEEE802154_ATTR_PHY_CHAN;
  req.attrval.phy.chan = chan;

  return sixlowpan_set_req(sock, ifname, &req);
}

int sixlowpan_seteaddr(int sock, FAR const char *ifname, FAR const uint8_t *eaddr)
{
  struct ieee802154_set_req_s req;

  req.attr = IEEE802154_ATTR_MAC_EADDR;
  IEEE802154_EADDRCOPY(req.attrval.mac.eaddr, eaddr);

  return sixlowpan_set_req(sock, ifname, &req);
}

int sixlowpan_setsaddr(int sock, FAR const char *ifname, FAR const uint8_t *saddr)
{
  struct ieee802154_set_req_s req;

  req.attr = IEEE802154_ATTR_MAC_SADDR;
  IEEE802154_SADDRCOPY(req.attrval.mac.saddr, saddr);

  return sixlowpan_set_req(sock, ifname, &req);
}

int sixlowpan_setpanid(int sock, FAR const char *ifname, FAR const uint8_t *panid)
{
  struct ieee802154_set_req_s req;

  req.attr = IEEE802154_ATTR_MAC_PANID;
  IEEE802154_PANIDCOPY(req.attrval.mac.panid, panid);

  return sixlowpan_set_req(sock, ifname, &req);
}

int sixlowpan_setrxonidle(int sock, FAR const char *ifname, bool rxonidle)
{
  struct ieee802154_set_req_s req;

  req.attr = IEEE802154_ATTR_MAC_RX_ON_WHEN_IDLE;
  req.attrval.mac.rxonidle = rxonidle;

  return sixlowpan_set_req(sock, ifname, &req);
}

int sixlowpan_setpromisc(int sock, FAR const char *ifname, bool promisc)
{
  struct ieee802154_set_req_s req;

  req.attr = IEEE802154_ATTR_MAC_PROMISCUOUS_MODE;
  req.attrval.mac.promisc_mode = promisc;

  return sixlowpan_set_req(sock, ifname, &req);
}

int sixlowpan_setassocpermit(int sock, FAR const char *ifname, bool assocpermit)
{
  struct ieee802154_set_req_s req;

  req.attr = IEEE802154_ATTR_MAC_ASSOCIATION_PERMIT;
  req.attrval.mac.assocpermit = assocpermit;

  return sixlowpan_set_req(sock, ifname, &req);
}

int sixlowpan_settxpwr(int sock, FAR const char *ifname, int32_t txpwr)
{
  struct ieee802154_set_req_s req;

  req.attr = IEEE802154_ATTR_PHY_TX_POWER;
  req.attrval.phy.txpwr = txpwr;

  return sixlowpan_set_req(sock, ifname, &req);
}

int sixlowpan_setmaxretries(int sock, FAR const char *ifname, uint8_t retries)
{
  struct ieee802154_set_req_s req;

  req.attr = IEEE802154_ATTR_MAC_MAX_FRAME_RETRIES;
  req.attrval.mac.max_retries = retries;

  return sixlowpan_set_req(sock, ifname, &req);
}
