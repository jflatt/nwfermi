/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * NextWindow Fermi touch engine: turns the raw camera stream into contacts.
 *
 * Integer-only and free of kernel object dependencies, so the same code is
 * built into the driver and into a userspace replay harness.
 */
#ifndef _NWFERMI_ENGINE_H
#define _NWFERMI_ENGINE_H

#ifdef __KERNEL__
#include <linux/types.h>
#endif

#define NWF_PIXELS		512
#define NWF_CAMS		2
#define NWF_SEQS		2
#define NWF_MAX_DIPS		8
#define NWF_MAX_CONTACTS	2
#define NWF_MAX_COORD		0x7fff
#define NWF_BUF_SIZE		8192
#define NWF_SETTINGS_MAX	4096

struct nwf_dip {
	s32 pos;		/* centre, pixels in Q8 */
	u16 width;		/* pixels */
	u32 area;		/* sum of (1 - ratio), Q8 */
};

struct nwf_cam {
	u16 wave[NWF_SEQS][NWF_PIXELS];
	u16 first[NWF_SEQS], last[NWF_SEQS];
	u8 have;			/* bitmask of sequences received */

	u32 base[NWF_PIXELS];		/* LED-on minus LED-off baseline, Q4 */
	unsigned int base_frames;
	bool base_valid;

	s32 lens[NWF_PIXELS];		/* lens angle term per pixel, Q24 rad */
	bool lens_valid;
	s32 rot;			/* mounting rotation, Q24 rad */
	bool rot_valid;
	s32 corner_pos;			/* far corner in the baseline, Q8 px */

	struct nwf_dip dips[NWF_MAX_DIPS];
	unsigned int ndips;
	bool fresh;			/* dips updated since last solve */
};

struct nwf_contact {
	s32 x, y;
	u8 id;
	bool active;
	unsigned int age;		/* frames seen */
	unsigned int missed;		/* consecutive frames not seen */
};

struct nwf_engine {
	/* stream reassembly */
	u8 buf[NWF_BUF_SIZE];
	unsigned int len;

	/* device settings */
	u8 settings[NWF_SETTINGS_MAX];
	unsigned int settings_len;
	bool settings_valid;
	s32 aspect_w, aspect_h;		/* physical screen size */
	s64 cal_x[3], cal_y[3];		/* affine raw->output, Q16 */
	bool cal_valid;
	s32 tilt;			/* camera baseline vs. screen, Q24 rad */

	struct nwf_cam cam[NWF_CAMS];

	/* per-frame scratch, kept off the stack */
	u32 sig[NWF_PIXELS];
	u16 ratio[NWF_PIXELS];

	struct nwf_contact contact[NWF_MAX_CONTACTS];
	u8 next_id;
	unsigned int asym_frames;	/* cameras disagree on the finger count */

	/* statistics */
	unsigned long packets, bad_packets, frames;

	/* output: called once per solved camera cycle */
	void (*report)(void *ctx, const struct nwf_contact *c, unsigned int n);
	void *ctx;
};

void nwf_engine_init(struct nwf_engine *e,
		     void (*report)(void *ctx, const struct nwf_contact *c,
				    unsigned int n),
		     void *ctx);
void nwf_engine_feed(struct nwf_engine *e, const u8 *data, unsigned int len);
void nwf_engine_idle(struct nwf_engine *e);

/* Release contacts when no frame has arrived for this long. */
#define NWF_IDLE_TIMEOUT_MS	60

#endif /* _NWFERMI_ENGINE_H */
