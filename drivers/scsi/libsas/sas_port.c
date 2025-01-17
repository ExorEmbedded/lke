#error CVE-2017-18232

/*
 * Serial Attached SCSI (SAS) Port class
 *
 * Copyright (C) 2005 Adaptec, Inc.  All rights reserved.
 * Copyright (C) 2005 Luben Tuikov <luben_tuikov@adaptec.com>
 *
 * This file is licensed under GPLv2.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "sas_internal.h"

#include <scsi/scsi_transport.h>
#include <scsi/scsi_transport_sas.h>
#include "../scsi_sas_internal.h"

static bool phy_is_wideport_member(struct asd_sas_port *port, struct asd_sas_phy *phy)
{
	struct sas_ha_struct *sas_ha = phy->ha;

	if (memcmp(port->attached_sas_addr, phy->attached_sas_addr,
		   SAS_ADDR_SIZE) != 0 || (sas_ha->strict_wide_ports &&
	     memcmp(port->sas_addr, phy->sas_addr, SAS_ADDR_SIZE) != 0))
		return false;
	return true;
}

static void sas_resume_port(struct asd_sas_phy *phy)
{
	struct domain_device *dev;
	struct asd_sas_port *port = phy->port;
	struct sas_ha_struct *sas_ha = phy->ha;
	struct sas_internal *si = to_sas_internal(sas_ha->core.shost->transportt);

	if (si->dft->lldd_port_formed)
		si->dft->lldd_port_formed(phy);

	if (port->suspended)
		port->suspended = 0;
	else {
		/* we only need to handle "link returned" actions once */
		return;
	}

	/* if the port came back:
	 * 1/ presume every device came back
	 * 2/ force the next revalidation to check all expander phys
	 */
	list_for_each_entry(dev, &port->dev_list, dev_list_node) {
		int i, rc;

		rc = sas_notify_lldd_dev_found(dev);
		if (rc) {
			sas_unregister_dev(port, dev);
			continue;
		}

		if (dev->dev_type == SAS_EDGE_EXPANDER_DEVICE || dev->dev_type == SAS_FANOUT_EXPANDER_DEVICE) {
			dev->ex_dev.ex_change_count = -1;
			for (i = 0; i < dev->ex_dev.num_phys; i++) {
				struct ex_phy *phy = &dev->ex_dev.ex_phy[i];

				phy->phy_change_count = -1;
			}
		}
	}

	sas_discover_event(port, DISCE_RESUME);
}

/**
 * sas_form_port -- add this phy to a port
 * @phy: the phy of interest
 *
 * This function adds this phy to an existing port, thus creating a wide
 * port, or it creates a port and adds the phy to the port.
 */
static void sas_form_port(struct asd_sas_phy *phy)
{
	int i;
	struct sas_ha_struct *sas_ha = phy->ha;
	struct asd_sas_port *port = phy->port;
	struct sas_internal *si =
		to_sas_internal(sas_ha->core.shost->transportt);
	unsigned long flags;

	if (port) {
		if (!phy_is_wideport_member(port, phy))
			sas_deform_port(phy, 0);
		else if (phy->suspended) {
			phy->suspended = 0;
			sas_resume_port(phy);

			/* phy came back, try to cancel the timeout */
			wake_up(&sas_ha->eh_wait_q);
			return;
		} else {
			SAS_DPRINTK("%s: phy%d belongs to port%d already(%d)!\n",
				    __func__, phy->id, phy->port->id,
				    phy->port->num_phys);
			return;
		}
	}

	/* see if the phy should be part of a wide port */
	spin_lock_irqsave(&sas_ha->phy_port_lock, flags);
	for (i = 0; i < sas_ha->num_phys; i++) {
		port = sas_ha->sas_port[i];
		spin_lock(&port->phy_list_lock);
		if (*(u64 *) port->sas_addr &&
		    phy_is_wideport_member(port, phy) && port->num_phys > 0) {
			/* wide port */
			SAS_DPRINTK("phy%d matched wide port%d\n", phy->id,
				    port->id);
			break;
		}
		spin_unlock(&port->phy_list_lock);
	}
	/* The phy does not match any existing port, create a new one */
	if (i == sas_ha->num_phys) {
		for (i = 0; i < sas_ha->num_phys; i++) {
			port = sas_ha->sas_port[i];
			spin_lock(&port->phy_list_lock);
			if (*(u64 *)port->sas_addr == 0
				&& port->num_phys == 0) {
				memcpy(port->sas_addr, phy->sas_addr,
					SAS_ADDR_SIZE);
				break;
			}
			spin_unlock(&port->phy_list_lock);
		}
	}

	if (i >= sas_ha->num_phys) {
		printk(KERN_NOTICE "%s: couldn't find a free port, bug?\n",
		       __func__);
		spin_unlock_irqrestore(&sas_ha->phy_port_lock, flags);
		return;
	}

	/* add the phy to the port */
	list_add_tail(&phy->port_phy_el, &port->phy_list);
	sas_phy_set_target(phy, port->port_dev);
	phy->port = port;
	port->num_phys++;
	port->phy_mask |= (1U << phy->id);

	if (*(u64 *)port->attached_sas_addr == 0) {
		port->class = phy->class;
		memcpy(port->attached_sas_addr, phy->attached_sas_addr,
		       SAS_ADDR_SIZE);
		port->iproto = phy->iproto;
		port->tproto = phy->tproto;
		port->oob_mode = phy->oob_mode;
		port->linkrate = phy->linkrate;
	} else
		port->linkrate = max(port->linkrate, phy->linkrate);
	spin_unlock(&port->phy_list_lock);
	spin_unlock_irqrestore(&sas_ha->phy_port_lock, flags);

	if (!port->port) {
		port->port = sas_port_alloc(phy->phy->dev.parent, port->id);
		BUG_ON(!port->port);
		sas_port_add(port->port);
	}
	sas_port_add_phy(port->port, phy->phy);

	SAS_DPRINTK("%s added to %s, phy_mask:0x%x (%16llx)\n",
		    dev_name(&phy->phy->dev), dev_name(&port->port->dev),
		    port->phy_mask,
		    SAS_ADDR(port->attached_sas_addr));

	if (port->port_dev)
		port->port_dev->pathways = port->num_phys;

	/* Tell the LLDD about this port formation. */
	if (si->dft->lldd_port_formed)
		si->dft->lldd_port_formed(phy);

	sas_discover_event(phy->port, DISCE_DISCOVER_DOMAIN);
}

/**
 * sas_deform_port -- remove this phy from the port it belongs to
 * @phy: the phy of interest
 *
 * This is called when the physical link to the other phy has been
 * lost (on this phy), in Event thread context. We cannot delay here.
 */
void sas_deform_port(struct asd_sas_phy *phy, int gone)
{
	struct sas_ha_struct *sas_ha = phy->ha;
	struct asd_sas_port *port = phy->port;
	struct sas_internal *si =
		to_sas_internal(sas_ha->core.shost->transportt);
	struct domain_device *dev;
	unsigned long flags;

	if (!port)
		return;		  /* done by a phy event */

	dev = port->port_dev;
	if (dev)
		dev->pathways--;

	if (port->num_phys == 1) {
		sas_unregister_domain_devices(port, gone);
		sas_port_delete(port->port);
		port->port = NULL;
	} else {
		sas_port_delete_phy(port->port, phy->phy);
		sas_device_set_phy(dev, port->port);
	}

	if (si->dft->lldd_port_deformed)
		si->dft->lldd_port_deformed(phy);

	spin_lock_irqsave(&sas_ha->phy_port_lock, flags);
	spin_lock(&port->phy_list_lock);

	list_del_init(&phy->port_phy_el);
	sas_phy_set_target(phy, NULL);
	phy->port = NULL;
	port->num_phys--;
	port->phy_mask &= ~(1U << phy->id);

	if (port->num_phys == 0) {
		INIT_LIST_HEAD(&port->phy_list);
		memset(port->sas_addr, 0, SAS_ADDR_SIZE);
		memset(port->attached_sas_addr, 0, SAS_ADDR_SIZE);
		port->class = 0;
		port->iproto = 0;
		port->tproto = 0;
		port->oob_mode = 0;
		port->phy_mask = 0;
	}
	spin_unlock(&port->phy_list_lock);
	spin_unlock_irqrestore(&sas_ha->phy_port_lock, flags);

	return;
}

/* ---------- SAS port events ---------- */

void sas_porte_bytes_dmaed(struct work_struct *work)
{
	struct asd_sas_event *ev = to_asd_sas_event(work);
	struct asd_sas_phy *phy = ev->phy;

	clear_bit(PORTE_BYTES_DMAED, &phy->port_events_pending);

	sas_form_port(phy);
}

void sas_porte_broadcast_rcvd(struct work_struct *work)
{
	struct asd_sas_event *ev = to_asd_sas_event(work);
	struct asd_sas_phy *phy = ev->phy;
	unsigned long flags;
	u32 prim;

	clear_bit(PORTE_BROADCAST_RCVD, &phy->port_events_pending);

	spin_lock_irqsave(&phy->sas_prim_lock, flags);
	prim = phy->sas_prim;
	spin_unlock_irqrestore(&phy->sas_prim_lock, flags);

	SAS_DPRINTK("broadcast received: %d\n", prim);
	sas_discover_event(phy->port, DISCE_REVALIDATE_DOMAIN);
}

void sas_porte_link_reset_err(struct work_struct *work)
{
	struct asd_sas_event *ev = to_asd_sas_event(work);
	struct asd_sas_phy *phy = ev->phy;

	clear_bit(PORTE_LINK_RESET_ERR, &phy->port_events_pending);

	sas_deform_port(phy, 1);
}

void sas_porte_timer_event(struct work_struct *work)
{
	struct asd_sas_event *ev = to_asd_sas_event(work);
	struct asd_sas_phy *phy = ev->phy;

	clear_bit(PORTE_TIMER_EVENT, &phy->port_events_pending);

	sas_deform_port(phy, 1);
}

void sas_porte_hard_reset(struct work_struct *work)
{
	struct asd_sas_event *ev = to_asd_sas_event(work);
	struct asd_sas_phy *phy = ev->phy;

	clear_bit(PORTE_HARD_RESET, &phy->port_events_pending);

	sas_deform_port(phy, 1);
}

/* ---------- SAS port registration ---------- */

static void sas_init_port(struct asd_sas_port *port,
			  struct sas_ha_struct *sas_ha, int i)
{
	memset(port, 0, sizeof(*port));
	port->id = i;
	INIT_LIST_HEAD(&port->dev_list);
	INIT_LIST_HEAD(&port->disco_list);
	INIT_LIST_HEAD(&port->destroy_list);
	spin_lock_init(&port->phy_list_lock);
	INIT_LIST_HEAD(&port->phy_list);
	port->ha = sas_ha;

	spin_lock_init(&port->dev_list_lock);
}

int sas_register_ports(struct sas_ha_struct *sas_ha)
{
	int i;

	/* initialize the ports and discovery */
	for (i = 0; i < sas_ha->num_phys; i++) {
		struct asd_sas_port *port = sas_ha->sas_port[i];

		sas_init_port(port, sas_ha, i);
		sas_init_disc(&port->disc, port);
	}
	return 0;
}

void sas_unregister_ports(struct sas_ha_struct *sas_ha)
{
	int i;

	for (i = 0; i < sas_ha->num_phys; i++)
		if (sas_ha->sas_phy[i]->port)
			sas_deform_port(sas_ha->sas_phy[i], 0);

}
