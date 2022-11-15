/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * PRUSS Remote Processor specific types
 *
 * Copyright (C) 2014-2018 Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 */

#ifndef _PRU_REMOTEPROC_H_
#define _PRU_REMOTEPROC_H_

/**
 * enum pruss_rsc_types - PRU specific resource types
 *
 * @PRUSS_RSC_INTRS: Resource holding information on PRU PINTC configuration
 * @PRUSS_RSC_MAX: Indicates end of known/defined PRU resource types.
 *		   This should be the last definition.
 *
 * Introduce new custom resource types before PRUSS_RSC_MAX.
 */
enum pruss_rsc_types {
	PRUSS_RSC_INTRS	= 1,
	PRUSS_RSC_MAX	= 2,
};

/**
 * struct pruss_event_chnl - PRU system events _to_ channel mapping
 * @event: number of the system event
 * @chnl: channel number assigned to a given @event
 *
 * PRU system events are mapped to channels, and these channels are mapped
 * to host interrupts. Events can be mapped to channels in a one-to-one or
 * many-to-one ratio (multiple events per channel), and channels can be
 * mapped to host interrupts in a one-to-one or many-to-one ratio (multiple
 * channels per interrupt).
 *
 */
struct pruss_event_chnl {
	s8 event;
	s8 chnl;
};

/**
 * struct fw_rsc_custom_intrmap - custom resource to define PRU interrupts
 * @reserved: reserved field providing backward compatibility, field used
 *            previously to provide version number
 * @chnl_host_intr_map: array of PRU channels to host interrupt mappings
 * @event_chnl_map_size: number of event_channel mappings defined in
 *			 @event_chnl_map
 * @event_chnl_map: PRU device address of pointer to array of events to
 *		    channel mappings
 *
 * PRU system events are mapped to channels, and these channels are mapped
 * to host interrupts. Events can be mapped to channels in a one-to-one or
 * many-to-one ratio (multiple events per channel), and channels can be
 * mapped to host interrupts in a one-to-one or many-to-one ratio (multiple
 * channels per interrupt).
 */
struct fw_rsc_custom_intrmap {
	u16 reserved;
	s8 chnl_host_intr_map[10];
	u32 event_chnl_map_size;
	u32 event_chnl_map;
};

/**
 * struct fw_rsc_custom_intrmap_k3 - K3 custom resource to define PRU interrupts
 * @chnl_host_intr_map: array of PRU channels to host interrupt mappings
 * @event_chnl_map_size: number of event_channel mappings defined in
 *			 @event_chnl_map
 * @event_chnl_map: PRU device address of pointer to array of events to channel
 *		    mappings
 *
 * PRU system events are mapped to channels, and these channels are mapped
 * to host interrupts. Events can be mapped to channels in a one-to-one or
 * many-to-one ratio (multiple events per channel), and channels can be
 * mapped to host interrupts in a one-to-one or many-to-one ratio (multiple
 * channels per interrupt).
 *
 * This structure needs to be used using custom interrupt resource version
 * number 1. This structure is to be used with firmwares dealing with the
 * additional host interrupts on ICSSG IP instances. The firmwares for PRU
 * cores on ICSSG can get away with the standard version (if not dealing with
 * Task Manager), but the firmwares for RTU cores would definitely need this
 * for mapping to the corresponding higher host interrupts.
 */
struct fw_rsc_custom_intrmap_k3 {
	s8 chnl_host_intr_map[20];
	u32 event_chnl_map_size;
	u32 event_chnl_map;
};

#endif	/* _PRU_REMOTEPROC_H_ */
