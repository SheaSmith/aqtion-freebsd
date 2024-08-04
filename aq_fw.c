/*
 * aQuantia Corporation Network Driver
 * Copyright (C) 2014-2017 aQuantia Corporation. All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   (1) Redistributions of source code must retain the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer.
 *
 *   (2) Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 *
 *   (3) The name of the author may not be used to endorse or promote
 *   products derived from this software without specific prior
 *   written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @file aq_fw.c
 * Firmware-related functions implementation.
 * @date 2017.12.07  @author roman.agafonov@aquantia.com
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <errno.h>

#include "aq_common.h"

#include "aq_hw.h"
#include "aq_hw_llh.h"
#include "aq_hw_llh_internal.h"

#include "aq_fw.h"
#include "aq_common.h"

#include "aq_dbg.h"


#define AQ2_MIF_HOST_FINISHED_STATUS_WRITE_REG	0x0e00
#define AQ2_MIF_HOST_FINISHED_STATUS_READ_REG	0x0e04
#define  AQ2_MIF_HOST_FINISHED_STATUS_ACK	(1 << 0)

#define AQ2_MCP_HOST_REQ_INT_REG		0x0f00
#define  AQ2_MCP_HOST_REQ_INT_READY		(1 << 0)
#define AQ2_MCP_HOST_REQ_INT_SET_REG		0x0f04
#define AQ2_MCP_HOST_REQ_INT_CLR_REG		0x0f08

#define AQ2_MIF_BOOT_REG			0x3040
#define  AQ2_MIF_BOOT_HOST_DATA_LOADED		(1 << 16)
#define  AQ2_MIF_BOOT_BOOT_STARTED		(1 << 24)
#define  AQ2_MIF_BOOT_CRASH_INIT		(1 << 27)
#define  AQ2_MIF_BOOT_BOOT_CODE_FAILED		(1 << 28)
#define  AQ2_MIF_BOOT_FW_INIT_FAILED		(1 << 29)
#define  AQ2_MIF_BOOT_FW_INIT_COMP_SUCCESS	(1U << 31)

/* AQ2 action resolver table */
#define AQ2_ART_ACTION_ACT_SHIFT		8
#define AQ2_ART_ACTION_RSS			0x0080
#define AQ2_ART_ACTION_INDEX_SHIFT		2
#define AQ2_ART_ACTION_ENABLE			0x0001
#define AQ2_ART_ACTION(act, rss, idx, en)		\
	(((act) << AQ2_ART_ACTION_ACT_SHIFT) |		\
	((rss) ? AQ2_ART_ACTION_RSS : 0) |		\
	((idx) << AQ2_ART_ACTION_INDEX_SHIFT) |		\
	((en) ? AQ2_ART_ACTION_ENABLE : 0))
#define AQ2_ART_ACTION_DROP			AQ2_ART_ACTION(0, 0, 0, 1)
#define AQ2_ART_ACTION_DISABLE			AQ2_ART_ACTION(0, 0, 0, 0)
#define AQ2_ART_ACTION_ASSIGN_QUEUE(q)		AQ2_ART_ACTION(1, 0, (q), 1)
#define AQ2_ART_ACTION_ASSIGN_TC(tc)		AQ2_ART_ACTION(1, 1, (tc), 1)

#define AQ2_RPF_TAG_PCP_MASK			0xe0000000
#define AQ2_RPF_TAG_PCP_SHIFT			29
#define AQ2_RPF_TAG_FLEX_MASK			0x18000000
#define AQ2_RPF_TAG_UNKNOWN_MASK		0x07000000
#define AQ2_RPF_TAG_L4_MASK			0x00e00000
#define AQ2_RPF_TAG_L3_V6_MASK			0x001c0000
#define AQ2_RPF_TAG_L3_V4_MASK			0x00038000
#define AQ2_RPF_TAG_UNTAG_MASK			0x00004000
#define AQ2_RPF_TAG_VLAN_MASK			0x00003c00
#define AQ2_RPF_TAG_ET_MASK			0x00000380
#define AQ2_RPF_TAG_ALLMC_MASK			0x00000040
#define AQ2_RPF_TAG_UC_MASK			0x0000002f

/* index of aq2_filter_art_set() */
#define AQ2_RPF_INDEX_L2_PROMISC_OFF		0
#define AQ2_RPF_INDEX_VLAN_PROMISC_OFF		1
#define AQ2_RPF_INDEX_L3L4_USER			8
#define AQ2_RPF_INDEX_ET_PCP_USER		24
#define AQ2_RPF_INDEX_VLAN_USER			40
#define AQ2_RPF_INDEX_PCP_TO_TC			56

#define AQ2_RPF_L2BC_TAG_REG			0x50f0
#define  AQ2_RPF_L2BC_TAG_MASK			0x0000003f

#define AQ2_RPF_NEW_CTRL_REG			0x5104
#define  AQ2_RPF_NEW_CTRL_ENABLE		(1 << 11)

#define AQ2_RPF_REDIR2_REG			0x54c8
#define  AQ2_RPF_REDIR2_INDEX			(1 << 12)
#define  AQ2_RPF_REDIR2_HASHTYPE		0x00000100
#define  AQ2_RPF_REDIR2_HASHTYPE_NONE		0
#define  AQ2_RPF_REDIR2_HASHTYPE_IP		(1 << 0)
#define  AQ2_RPF_REDIR2_HASHTYPE_TCP4		(1 << 1)
#define  AQ2_RPF_REDIR2_HASHTYPE_UDP4		(1 << 2)
#define  AQ2_RPF_REDIR2_HASHTYPE_IP6		(1 << 3)
#define  AQ2_RPF_REDIR2_HASHTYPE_TCP6		(1 << 4)
#define  AQ2_RPF_REDIR2_HASHTYPE_UDP6		(1 << 5)
#define  AQ2_RPF_REDIR2_HASHTYPE_IP6EX		(1 << 6)
#define  AQ2_RPF_REDIR2_HASHTYPE_TCP6EX		(1 << 7)
#define  AQ2_RPF_REDIR2_HASHTYPE_UDP6EX		(1 << 8)
#define  AQ2_RPF_REDIR2_HASHTYPE_ALL		0x00000100

#define AQ2_RPF_REC_TAB_ENABLE_REG		0x6ff0
#define  AQ2_RPF_REC_TAB_ENABLE_MASK		0x0000ffff

#define AQ2_LAUNCHTIME_CTRL_REG			0x7a1c
#define  AQ2_LAUNCHTIME_CTRL_RATIO		0x0000ff00
#define  AQ2_LAUNCHTIME_CTRL_RATIO_SPEED_QUARTER 4
#define  AQ2_LAUNCHTIME_CTRL_RATIO_SPEED_HALF	2
#define  AQ2_LAUNCHTIME_CTRL_RATIO_SPEED_FULL	1

#define AQ2_TX_INTR_MODERATION_CTL_REG(i)	(0x7c28 + (i) * 0x40)
#define  AQ2_TX_INTR_MODERATION_CTL_EN		(1 << 1)
#define  AQ2_TX_INTR_MODERATION_CTL_MIN		0x0000ff00
#define  AQ2_TX_INTR_MODERATION_CTL_MAX		0x01ff0000

#define AQ2_FW_INTERFACE_IN_MTU_REG		0x12000
#define AQ2_FW_INTERFACE_IN_MAC_ADDRESS_REG	0x12008

#define AQ2_FW_INTERFACE_IN_LINK_CONTROL_REG	0x12010
#define  AQ2_FW_INTERFACE_IN_LINK_CONTROL_MODE	0x0000000f
#define  AQ2_FW_INTERFACE_IN_LINK_CONTROL_MODE_INVALID	0
#define  AQ2_FW_INTERFACE_IN_LINK_CONTROL_MODE_ACTIVE	1
#define  AQ2_FW_INTERFACE_IN_LINK_CONTROL_MODE_SLEEP_PROXY 2
#define  AQ2_FW_INTERFACE_IN_LINK_CONTROL_MODE_LOWPOWER	3
#define  AQ2_FW_INTERFACE_IN_LINK_CONTROL_MODE_SHUTDOWN	4

#define AQ2_FW_INTERFACE_IN_LINK_OPTIONS_REG	0x12018
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_DOWNSHIFT	(1 << 27)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_PAUSE_TX	(1 << 25)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_PAUSE_RX	(1 << 24)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_EEE_10G	(1 << 20)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_EEE_5G	(1 << 19)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_EEE_2G5	(1 << 18)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_EEE_1G	(1 << 17)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_EEE_100M	(1 << 16)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_RATE_10G	(1 << 15)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_RATE_N5G	(1 << 14)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_RATE_5G	(1 << 13)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_RATE_N2G5	(1 << 12)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_RATE_2G5	(1 << 11)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_RATE_1G	(1 << 10)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_RATE_100M	(1 << 9)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_RATE_10M	(1 << 8)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_RATE_1G_HD	(1 << 7)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_RATE_100M_HD	(1 << 6)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_RATE_10M_HD	(1 << 5)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_EXTERNAL_LOOPBACK (1 << 4)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_INTERNAL_LOOPBACK (1 << 3)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_MINIMAL_LINK_SPEED (1 << 2)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_LINK_RENEGOTIATE (1 << 1)
#define  AQ2_FW_INTERFACE_IN_LINK_OPTIONS_LINK_UP	(1 << 0)

#define AQ2_FW_INTERFACE_IN_REQUEST_POLICY_REG	0x12a58
#define  AQ2_FW_INTERFACE_IN_REQUEST_POLICY_MCAST_QUEUE_OR_TC		0x00800000
#define  AQ2_FW_INTERFACE_IN_REQUEST_POLICY_MCAST_RX_QUEUE_TC_INDEX	0x007c0000
#define  AQ2_FW_INTERFACE_IN_REQUEST_POLICY_MCAST_ACCEPT		0x00010000
#define  AQ2_FW_INTERFACE_IN_REQUEST_POLICY_BCAST_QUEUE_OR_TC		0x00008000
#define  AQ2_FW_INTERFACE_IN_REQUEST_POLICY_BCAST_RX_QUEUE_TC_INDEX	0x00007c00
#define  AQ2_FW_INTERFACE_IN_REQUEST_POLICY_BCAST_ACCEPT		0x00000100
#define  AQ2_FW_INTERFACE_IN_REQUEST_POLICY_PROMISC_QUEUE_OR_TC		0x00000080
#define  AQ2_FW_INTERFACE_IN_REQUEST_POLICY_PROMISC_RX_QUEUE_TX_INDEX	0x0000007c
#define  AQ2_FW_INTERFACE_IN_REQUEST_POLICY_PROMISC_MCAST		0x00000002
#define  AQ2_FW_INTERFACE_IN_REQUEST_POLICY_PROMISC_ALL			0x00000001

#define AQ2_FW_INTERFACE_OUT_TRANSACTION_ID_REG	0x13000
#define  AQ2_FW_INTERFACE_OUT_TRANSACTION_ID_B	0xffff0000
#define  AQ2_FW_INTERFACE_OUT_TRANSACTION_ID_B_S 16
#define  AQ2_FW_INTERFACE_OUT_TRANSACTION_ID_A	0x0000ffff
#define  AQ2_FW_INTERFACE_OUT_TRANSACTION_ID_A_S 0

#define AQ2_FW_INTERFACE_OUT_VERSION_BUNDLE_REG	0x13004
#define AQ2_FW_INTERFACE_OUT_VERSION_MAC_REG	0x13008

#define AQ2_FW_INTERFACE_OUT_VERSION_PHY_REG	0x1300c
#define  AQ2_FW_INTERFACE_OUT_VERSION_BUILD	0xffff0000
#define  AQ2_FW_INTERFACE_OUT_VERSION_BUILD_S	16
#define  AQ2_FW_INTERFACE_OUT_VERSION_MINOR	0x0000ff00
#define  AQ2_FW_INTERFACE_OUT_VERSION_MINOR_S	8
#define  AQ2_FW_INTERFACE_OUT_VERSION_MAJOR	0x000000ff
#define  AQ2_FW_INTERFACE_OUT_VERSION_MAJOR_S	0

#define AQ2_FW_INTERFACE_OUT_VERSION_IFACE_REG	0x13010
#define  AQ2_FW_INTERFACE_OUT_VERSION_IFACE_VER	0x0000000f
#define  AQ2_FW_INTERFACE_OUT_VERSION_IFACE_VER_A0 0
#define  AQ2_FW_INTERFACE_OUT_VERSION_IFACE_VER_B0 1

#define AQ2_FW_INTERFACE_OUT_LINK_STATUS_REG	0x13014
#define  AQ2_FW_INTERFACE_OUT_LINK_STATUS_DUPLEX	(1 << 11)
#define  AQ2_FW_INTERFACE_OUT_LINK_STATUS_EEE		(1 << 10)
#define  AQ2_FW_INTERFACE_OUT_LINK_STATUS_PAUSE_RX	(1 << 9)
#define  AQ2_FW_INTERFACE_OUT_LINK_STATUS_PAUSE_TX	(1 << 8)
#define  AQ2_FW_INTERFACE_OUT_LINK_STATUS_RATE		0x000000f0
#define  AQ2_FW_INTERFACE_OUT_LINK_STATUS_RATE_S	4
#define  AQ2_FW_INTERFACE_OUT_LINK_STATUS_RATE_10G	6
#define  AQ2_FW_INTERFACE_OUT_LINK_STATUS_RATE_5G	5
#define  AQ2_FW_INTERFACE_OUT_LINK_STATUS_RATE_2G5	4
#define  AQ2_FW_INTERFACE_OUT_LINK_STATUS_RATE_1G	3
#define  AQ2_FW_INTERFACE_OUT_LINK_STATUS_RATE_100M	2
#define  AQ2_FW_INTERFACE_OUT_LINK_STATUS_RATE_10M	1
#define  AQ2_FW_INTERFACE_OUT_LINK_STATUS_RATE_INVALID	0
#define  AQ2_FW_INTERFACE_OUT_LINK_STATUS_STATE		0x0000000f

#define AQ2_FW_INTERFACE_OUT_FILTER_CAPS_REG	0x13774
#define  AQ2_FW_INTERFACE_OUT_FILTER_CAPS3_RESOLVER_BASE_INDEX 0x00ff0000
#define  AQ2_FW_INTERFACE_OUT_FILTER_CAPS3_RESOLVER_BASE_INDEX_SHIFT 16

#define AQ2_RPF_ACT_ART_REQ_TAG_REG(i)		(0x14000 + (i) * 0x10)
#define AQ2_RPF_ACT_ART_REQ_MASK_REG(i)		(0x14004 + (i) * 0x10)
#define AQ2_RPF_ACT_ART_REQ_ACTION_REG(i)	(0x14008 + (i) * 0x10)

#define FW_VERSION_MAJOR(sc)	(((sc)->sc_fw_version >> 24) & 0xff)
#define FW_VERSION_MINOR(sc)	(((sc)->sc_fw_version >> 16) & 0xff)
#define FW_VERSION_BUILD(sc)	((sc)->sc_fw_version & 0xffff)

#define FEATURES_AQ2		0x10000000
#define FEATURES_AQ2_IFACE_A0	0x20000000
#define FEATURES_AQ2_IFACE_B0	0x40000000





typedef enum aq_fw_bootloader_mode
{
    boot_mode_unknown = 0,
    boot_mode_flb,
    boot_mode_rbl_flash,
    boot_mode_rbl_host_bootload,
} aq_fw_bootloader_mode;

#define AQ_CFG_HOST_BOOT_DISABLE 0
// Timeouts
#define RBL_TIMEOUT_MS              10000
#define MAC_FW_START_TIMEOUT_MS     10000
#define FW_LOADER_START_TIMEOUT_MS  10000

const u32 NO_RESET_SCRATCHPAD_ADDRESS = 0;
const u32 NO_RESET_SCRATCHPAD_LEN_RES = 1;
const u32 NO_RESET_SCRATCHPAD_RBL_STATUS = 2;
const u32 NO_RESET_SCRATCHPAD_RBL_STATUS_2 = 3;
const u32 WRITE_DATA_COMPLETE = 0x55555555;
const u32 WRITE_DATA_CHUNK_DONE = 0xaaaaaaaa;
const u32 WRITE_DATA_FAIL_WRONG_ADDRESS = 0x66666666;

const u32 WAIT_WRITE_TIMEOUT = 1;
const u32 WAIT_WRITE_TIMEOUT_COUNT = 1000;

const u32 RBL_STATUS_SUCCESS = 0xabba;
const u32 RBL_STATUS_FAILURE = 0xbad;
const u32 RBL_STATUS_HOST_BOOT = 0xf1a7;

const u32 SCRATCHPAD_FW_LOADER_STATUS = (0x40 / sizeof(u32));


extern struct aq_firmware_ops aq_fw1x_ops;
extern struct aq_firmware_ops aq_fw2x_ops;


int mac_soft_reset_(struct aq_hw* hw, aq_fw_bootloader_mode* mode);
int mac_soft_reset_flb_(struct aq_hw* hw);
int mac_soft_reset_rbl_(struct aq_hw* hw, aq_fw_bootloader_mode* mode);
int wait_init_mac_firmware_(struct aq_hw* hw);


int
aq2_interface_buffer_read(struct aq_hw *sc, uint32_t reg0, uint32_t *data0,
    uint32_t size0)
{
	uint32_t tid0, tid1, reg, *data, size;
	int timo;

	for (timo = 10000; timo > 0; timo--) {
		tid0 = AQ_READ_REG(sc, AQ2_FW_INTERFACE_OUT_TRANSACTION_ID_REG);
		if (((tid0 & AQ2_FW_INTERFACE_OUT_TRANSACTION_ID_A)
		    >> AQ2_FW_INTERFACE_OUT_TRANSACTION_ID_A_S) !=
		    ((tid0 & AQ2_FW_INTERFACE_OUT_TRANSACTION_ID_B)
		    >> AQ2_FW_INTERFACE_OUT_TRANSACTION_ID_B_S)) {
			msec_delay(10);
			continue;
		}

		for (reg = reg0, data = data0, size = size0;
		    size >= 4; reg += 4, data++, size -= 4) {
			*data = AQ_READ_REG(sc, reg);
		}

		tid1 = AQ_READ_REG(sc, AQ2_FW_INTERFACE_OUT_TRANSACTION_ID_REG);
		if (tid0 == tid1)
			break;
	}
	if (timo == 0) {
		aq_log("interface buffer read timeout\n");
		return ETIMEDOUT;
	}
	return 0;
}

int aq2_fw_reset(struct aq_hw* sc) {
    uint32_t v;
	int timo, err;
	char buf[32];
	uint32_t filter_caps[3];

	// sc->sc_fw_ops = &aq2_fw_ops;
	sc->sc_features = FEATURES_AQ2;

	AQ_WRITE_REG(sc, AQ2_MCP_HOST_REQ_INT_CLR_REG, 1);
	AQ_WRITE_REG(sc, AQ2_MIF_BOOT_REG, 1);	/* reboot request */
	for (timo = 200000; timo > 0; timo--) {
		v = AQ_READ_REG(sc, AQ2_MIF_BOOT_REG);
		if ((v & AQ2_MIF_BOOT_BOOT_STARTED) && v != 0xffffffff)
			break;
		msec_delay(10);
	}
	if (timo <= 0) {
		printf(": FW reboot timeout\n");
		return ETIMEDOUT;
	}

	for (timo = 2000000; timo > 0; timo--) {
		v = AQ_READ_REG(sc, AQ2_MIF_BOOT_REG);
		if ((v & AQ2_MIF_BOOT_FW_INIT_FAILED) ||
		    (v & AQ2_MIF_BOOT_FW_INIT_COMP_SUCCESS))
			break;
		v = AQ_READ_REG(sc, AQ2_MCP_HOST_REQ_INT_REG);
		if (v & AQ2_MCP_HOST_REQ_INT_READY)
			break;
		msec_delay(10);
	}
	if (timo <= 0) {
		printf(": FW restart timeout\n");
		return ETIMEDOUT;
	}

	v = AQ_READ_REG(sc, AQ2_MIF_BOOT_REG);
	if (v & AQ2_MIF_BOOT_FW_INIT_FAILED) {
		printf(": FW restart failed\n");
		return ETIMEDOUT;
	}

	v = AQ_READ_REG(sc, AQ2_MCP_HOST_REQ_INT_REG);
	if (v & AQ2_MCP_HOST_REQ_INT_READY) {
		printf(": firmware required\n");
		return ENXIO;
	}

	/*
	 * Get aq2 firmware version.
	 * Note that the bit layout and its meaning are different from aq1.
	 */
	err = aq2_interface_buffer_read(sc, AQ2_FW_INTERFACE_OUT_VERSION_BUNDLE_REG,
	    (uint32_t *)&v, sizeof(v));
	if (err != 0)
		return err;

	sc->sc_fw_version =
	    (((v & AQ2_FW_INTERFACE_OUT_VERSION_MAJOR) >>
		AQ2_FW_INTERFACE_OUT_VERSION_MAJOR_S) << 24) |
	    (((v & AQ2_FW_INTERFACE_OUT_VERSION_MINOR) >>
		AQ2_FW_INTERFACE_OUT_VERSION_MINOR_S) << 16) |
	    (((v & AQ2_FW_INTERFACE_OUT_VERSION_BUILD) >>
		AQ2_FW_INTERFACE_OUT_VERSION_BUILD_S));

	err = aq2_interface_buffer_read(sc, AQ2_FW_INTERFACE_OUT_VERSION_IFACE_REG,
	    (uint32_t *)&v, sizeof(v));
	if (err != 0)
		return err;

	switch (v & AQ2_FW_INTERFACE_OUT_VERSION_IFACE_VER) {
	case AQ2_FW_INTERFACE_OUT_VERSION_IFACE_VER_A0:
		sc->sc_features |= FEATURES_AQ2_IFACE_A0;
		strncpy(buf, "A0", sizeof(buf));
		break;
	case AQ2_FW_INTERFACE_OUT_VERSION_IFACE_VER_B0:
		sc->sc_features |= FEATURES_AQ2_IFACE_B0;
		strncpy(buf, "B0", sizeof(buf));
		break;
	default:
		snprintf(buf, sizeof(buf), "(unknown 0x%08x)", v);
		break;
	}
	printf(", Atlantic2 %s, F/W version %d.%d.%d", buf,
	    FW_VERSION_MAJOR(sc), FW_VERSION_MINOR(sc), FW_VERSION_BUILD(sc));

	aq2_interface_buffer_read(sc, AQ2_FW_INTERFACE_OUT_FILTER_CAPS_REG,
	    filter_caps, sizeof(filter_caps));
	sc->sc_art_filter_base_index = ((filter_caps[2] &
	    AQ2_FW_INTERFACE_OUT_FILTER_CAPS3_RESOLVER_BASE_INDEX) >>
	    AQ2_FW_INTERFACE_OUT_FILTER_CAPS3_RESOLVER_BASE_INDEX_SHIFT) * 8;

	/* debug info */
	v = AQ_READ_REG(sc, AQ_HW_REVISION_REG);
	DPRINTF(("%s: HW Rev: 0x%08x\n", "atlantic", v));

	return 0;
}

int aq_fw_reset(struct aq_hw* hw)
{
    int ver = AQ_READ_REG(hw, 0x18);
    u32 bootExitCode = 0;
    int k;

    for (k = 0; k < 1000; ++k) {
        u32 flbStatus = reg_glb_daisy_chain_status1_get(hw);
        bootExitCode = AQ_READ_REG(hw, 0x388);
        if (flbStatus != 0x06000000 || bootExitCode != 0)
            break;
    }

    if (k == 1000) {
        aq_log_error("Neither RBL nor FLB started");
        return (-EBUSY);
    }

    hw->rbl_enabled = bootExitCode != 0;

    trace(dbg_init, "RBL enabled = %d", hw->rbl_enabled);

    /* Having FW version 0 is an indicator that cold start
     * is in progress. This means two things:
     * 1) Driver have to wait for FW/HW to finish boot (500ms giveup)
     * 2) Driver may skip reset sequence and save time.
     */
    if (hw->fast_start_enabled && !ver) {
        int err = wait_init_mac_firmware_(hw);
        /* Skip reset as it just completed */
        if (!err)
            return (0);
    }

    aq_fw_bootloader_mode mode = boot_mode_unknown;
    int err = mac_soft_reset_(hw, &mode);
    if (err < 0) {
        aq_log_error("MAC reset failed: %d", err);
        return (err);
    }

    switch (mode) {
    case boot_mode_flb:
        aq_log("FLB> F/W successfully loaded from flash.");
        hw->flash_present = true;
        return wait_init_mac_firmware_(hw);

    case boot_mode_rbl_flash:
        aq_log("RBL> F/W loaded from flash. Host Bootload disabled.");
        hw->flash_present = true;
        return wait_init_mac_firmware_(hw);

    case boot_mode_unknown:
        aq_log_error("F/W bootload error: unknown bootloader type");
        return (-ENOTSUP);

    case boot_mode_rbl_host_bootload:
#if AQ_CFG_HOST_BOOT_DISABLE
        aq_log_error("RBL> Host Bootload mode: this driver does not support Host Boot");
        return (-ENOTSUP);
#else
        trace(dbg_init, "RBL> Host Bootload mode");
        break;
#endif // HOST_BOOT_DISABLE
    }

    /*
     * #todo: Host Boot
     */
    aq_log_error("RBL> F/W Host Bootload not implemented");

    return (-ENOTSUP);
}

int aq_fw_ops_init(struct aq_hw* hw)
{
    if (hw->fw_version.raw == 0)
        hw->fw_version.raw = AQ_READ_REG(hw, 0x18);

    aq_log("MAC F/W version is %d.%d.%d",
        hw->fw_version.major_version, hw->fw_version.minor_version,
        hw->fw_version.build_number);

    if (hw->fw_version.major_version == 1) {
        trace(dbg_init, "using F/W ops v1.x");
        hw->fw_ops = &aq_fw1x_ops;
        return (EOK);
    } else if (hw->fw_version.major_version >= 2) {
        trace(dbg_init, "using F/W ops v2.x");
        hw->fw_ops = &aq_fw2x_ops;
        return (EOK);
    }

    aq_log_error("aq_fw_ops_init(): invalid F/W version %#x", hw->fw_version.raw);
    return (-ENOTSUP);
}


int mac_soft_reset_(struct aq_hw* hw, aq_fw_bootloader_mode* mode /*= nullptr*/)
{
    if (hw->rbl_enabled) {
        return mac_soft_reset_rbl_(hw, mode);
    } else {
        if (mode)
            *mode = boot_mode_flb;

        return mac_soft_reset_flb_(hw);
    }
}

int mac_soft_reset_flb_(struct aq_hw* hw)
{
    int k;

    reg_global_ctl2_set(hw, 0x40e1);
    // Let Felicity hardware to complete SMBUS transaction before Global software reset.
    msec_delay(50);

    /*
     * If SPI burst transaction was interrupted(before running the script), global software
     * reset may not clear SPI interface. Clean it up manually before global reset.
     */
    reg_glb_nvr_provisioning2_set(hw, 0xa0);
    reg_glb_nvr_interface1_set(hw, 0x9f);
    reg_glb_nvr_interface1_set(hw, 0x809f);
    msec_delay(50);

    reg_glb_standard_ctl1_set(hw, (reg_glb_standard_ctl1_get(hw) & ~glb_reg_res_dis_msk) | glb_soft_res_msk);

    // Kickstart.
    reg_global_ctl2_set(hw, 0x80e0);
    reg_mif_power_gating_enable_control_set(hw, 0);
    if (!hw->fast_start_enabled)
        reg_glb_general_provisioning9_set(hw, 1);

    /*
     * For the case SPI burst transaction was interrupted (by MCP reset above),
     * wait until it is completed by hardware.
     */
    msec_delay(50); // Sleep for 10 ms.

    /* MAC Kickstart */
    if (!hw->fast_start_enabled) {
        reg_global_ctl2_set(hw, 0x180e0);

        u32 flb_status = 0;
        int k;
        for (k = 0; k < 1000; ++k) {
            flb_status = reg_glb_daisy_chain_status1_get(hw) & 0x10;
            if (flb_status != 0)
                break;
            msec_delay(10); // Sleep for 10 ms.
        }

        if (flb_status == 0) {
            trace_error(dbg_init, "FLB> MAC kickstart failed: timed out");
            return (false);
        }

        trace(dbg_init, "FLB> MAC kickstart done, %d ms", k);
        /* FW reset */
        reg_global_ctl2_set(hw, 0x80e0);
        // Let Felicity hardware complete SMBUS transaction before Global software reset.
        msec_delay(50);
    }
    reg_glb_cpu_sem_set(hw, 1, 0);

    // PHY Kickstart: #undone

    // Global software reset
    rx_rx_reg_res_dis_set(hw, 0);
    tx_tx_reg_res_dis_set(hw, 0);
    mpi_tx_reg_res_dis_set(hw, 0);
    reg_glb_standard_ctl1_set(hw, (reg_glb_standard_ctl1_get(hw) & ~glb_reg_res_dis_msk) | glb_soft_res_msk);

    bool restart_completed = false;
    for (k = 0; k < 1000; ++k) {
        restart_completed = reg_glb_fw_image_id1_get(hw) != 0;
        if (restart_completed)
            break;
        msec_delay(10);
    }

    if (!restart_completed) {
        trace_error(dbg_init, "FLB> Global Soft Reset failed");
        return (false);
    }

    trace(dbg_init, "FLB> F/W restart: %d ms", k * 10);
    return (true);
}

int mac_soft_reset_rbl_(struct aq_hw* hw, aq_fw_bootloader_mode* mode)
{
    trace(dbg_init, "RBL> MAC reset STARTED!");

    reg_global_ctl2_set(hw, 0x40e1);
    reg_glb_cpu_sem_set(hw, 1, 0);
    reg_mif_power_gating_enable_control_set(hw, 0);

    // MAC FW will reload PHY FW if 1E.1000.3 was cleaned - #undone

    reg_glb_cpu_no_reset_scratchpad_set(hw, 0xDEAD, NO_RESET_SCRATCHPAD_RBL_STATUS);

    // Global software reset
    rx_rx_reg_res_dis_set(hw, 0);
    tx_tx_reg_res_dis_set(hw, 0);
    mpi_tx_reg_res_dis_set(hw, 0);
    reg_glb_standard_ctl1_set(hw, (reg_glb_standard_ctl1_get(hw) & ~glb_reg_res_dis_msk) | glb_soft_res_msk);

    reg_global_ctl2_set(hw, 0x40e0);

    // Wait for RBL to finish boot process.
    u16 rbl_status = 0;
    for (int k = 0; k < RBL_TIMEOUT_MS; ++k) {
        rbl_status = LOWORD(reg_glb_cpu_no_reset_scratchpad_get(hw, NO_RESET_SCRATCHPAD_RBL_STATUS));
        if (rbl_status != 0 && rbl_status != 0xDEAD)
            break;

        msec_delay(1);
    }

    if (rbl_status == 0 || rbl_status == 0xDEAD) {
        trace_error(dbg_init, "RBL> RBL restart failed: timeout");
        return (-EBUSY);
    }

    if (rbl_status == RBL_STATUS_SUCCESS) {
        if (mode)
            *mode = boot_mode_rbl_flash;
        trace(dbg_init, "RBL> reset complete! [Flash]");
    } else if (rbl_status == RBL_STATUS_HOST_BOOT) {
        if (mode)
            *mode = boot_mode_rbl_host_bootload;
        trace(dbg_init, "RBL> reset complete! [Host Bootload]");
    } else {
        trace_error(dbg_init, "unknown RBL status 0x%x", rbl_status);
        return (-EBUSY);
    }

    return (EOK);
}

int wait_init_mac_firmware_(struct aq_hw* hw)
{
    for (int i = 0; i < MAC_FW_START_TIMEOUT_MS; ++i) {
        if ((hw->fw_version.raw = AQ_READ_REG(hw, 0x18)) != 0)
            return (EOK);

        msec_delay(1);
    }

    trace_error(dbg_init, "timeout waiting for reg 0x18. MAC f/w NOT READY");
    return (-EBUSY);
}
