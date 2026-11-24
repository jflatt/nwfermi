// SPDX-License-Identifier: GPL-2.0-only
/*
 * NextWindow Fermi touch engine
 *
 * A Fermi controller has two line-scan cameras in the top corners of the
 * screen looking across a retro-reflective bezel.  It streams, per camera,
 * one waveform with the IR LEDs off and one with them on.  A finger shows up
 * as a shadow in the LED-on minus LED-off signal; the shadow's pixel position
 * maps to an angle through a per-camera lens polynomial, and the two angles
 * are intersected to find the contact.
 *
 * Stream format (all multi-byte fields big endian):
 *
 *   packet:   '[' ']' seq:u8 len:u16 type:u8 sub:u8 flags:u8 payload
 *   type 1:   waveform, delta encoded:
 *               ts:u16 cam<<4|seq:u8 first:u16 count:u16 flags:u8 aux:u16 x:u8
 *               then "count" pixels: s8 delta from the previous pixel,
 *               or 0x80 followed by an absolute u16 (so the payload may run
 *               past "len"; count == len - 11)
 *   type 2/2: settings: tree_len:u16 values_len:u16 0:u16, a tree of
 *             (group:u8 id:u8 type:u8 count:u8) descriptors, then the values
 *
 * The device settings carry the lens polynomials and a four point calibration
 * (raw positions measured at four known targets).  The camera mounting
 * rotation is not stored; it is recovered from the far bottom corner of the
 * bezel, which appears as a sharp bright-to-dark step in each camera's
 * baseline.
 *
 * All arithmetic is integer: angles are Q24 radians, sines and cosines Q30.
 */

#ifdef __KERNEL__
#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/limits.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#endif

#include "nwfermi_engine.h"

#define PKT_HDR			8
#define WAVE_HDR		11
#define PKT_WAVEFORM		1
#define PKT_SETTINGS		2
#define SETTINGS_SUB		2

#define Q24_LENS_MAX		(4LL << 24)	/* |lens angle| bound, Q24 */
#define Q24_PI_2		26353589
#define Q30_CORDIC_K		652032874

#define BASE_INIT_FRAMES	4
#define BASE_MIN_Q4		(16 << 4)	/* ignore pixels darker than this */
#define DIP_RATIO_Q8		128		/* shadow: below 50% of baseline */
#define DIP_DEPTH_Q8		96		/* ...and somewhere below 37.5% */
#define DIP_MIN_WIDTH		2
#define DIP_MERGE_GAP		3
#define CORNER_LO		100	/* search window for the far corner */
#define CORNER_HI		400
#define CORNER_SPAN		30	/* side wall sampled this far before it */
#define CORNER_GAP		8	/* ...excluding the last pixels of the step */
#define ROT_REFRESH_FRAMES	256

#define TRACK_MAX_DIST		6000
#define ASYM_FRAMES		6	/* new finger seen by one camera only */
#define TOUCH_DOWN_FRAMES	2
#define TOUCH_UP_FRAMES		2

/* Settings keys (group << 8 | id) */
#define KEY_SCREEN_W		0x2022
#define KEY_SCREEN_H		0x2023
#define KEY_CAL_MEAS		0x703b		/* 8 floats: x,y for 4 points */
#define KEY_CAL_TARGET		0x7043		/* 8 floats: x,y for 4 points */
#define KEY_LENS_C0		0x29		/* + cam group, 7 coefficients */
#define CAM_GROUP(c)		(0x30 + 0x10 * (c))

static const s32 atan_q24[] = {
	13176795, 7778716, 4110060, 2086331, 1047214, 524117, 262123, 131069,
	65536, 32768, 16384, 8192, 4096, 2048, 1024, 512, 256, 128, 64, 32,
	16, 8, 4, 2,
};

static const u8 setting_size[10] = { 0, 1, 2, 4, 1, 2, 4, 4, 14, 9 };

/* ---------------------------------------------------------------------- */
/* Fixed-point helpers                                                     */

/* sin/cos of a Q24 angle in [-pi/2, pi/2], Q30 results. */
static void cordic_sincos(s32 angle, s32 *c, s32 *s)
{
	s64 x = Q30_CORDIC_K, y = 0, t;
	s32 z = angle;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(atan_q24); i++) {
		if (z >= 0) {
			t = x - (y >> i);
			y += x >> i;
			z -= atan_q24[i];
		} else {
			t = x + (y >> i);
			y -= x >> i;
			z += atan_q24[i];
		}
		x = t;
	}
	*c = (s32)x;
	*s = (s32)y;
}

/* atan2(y, x) for x > 0, Q24 result. */
static s32 cordic_atan2(s64 y, s64 x)
{
	s32 z = 0;
	s64 t;
	unsigned int i;

	while (abs(x) > (1LL << 40) || abs(y) > (1LL << 40)) {
		x >>= 1;
		y >>= 1;
	}
	x <<= 16;
	y <<= 16;
	for (i = 0; i < ARRAY_SIZE(atan_q24); i++) {
		if (y > 0) {
			t = x + (y >> i);
			y -= x >> i;
			z += atan_q24[i];
		} else {
			t = x - (y >> i);
			y += x >> i;
			z -= atan_q24[i];
		}
		x = t;
	}
	return z;
}

/*
 * coef * u^k * 2^shift, with coef given as IEEE 754 single precision bits.
 * Zero, denormal, infinite and NaN coefficients contribute nothing.
 */
static s64 float_term(u32 bits, s32 u, unsigned int k, int shift)
{
	unsigned int exp = (bits >> 23) & 0xff;
	bool neg = bits >> 31;
	u64 man, p = 1, r;
	unsigned int i;
	int e;

	if (!exp || exp == 0xff)
		return 0;

	man = (bits & 0x7fffff) | 0x800000;
	e = (int)exp - 150 + shift;

	for (i = 0; i < k; i++)
		p *= (u64)abs(u);
	if (u < 0 && (k & 1))
		neg = !neg;

	if (e >= 0) {
		if (e > 20 || p > (1ULL << 20))
			return 0;	/* implausibly large */
		r = (man * p) << e;
	} else {
		if (e < -120)
			return 0;
		r = mul_u64_u64_shr(man, p, -e);
	}
	return neg ? -(s64)r : (s64)r;
}

/* Integer value of a float, truncated toward zero; false if not finite. */
static bool float_to_int(u32 bits, s32 *out)
{
	unsigned int exp = (bits >> 23) & 0xff;

	if (exp == 0xff)
		return false;
	*out = (s32)float_term(bits, 1, 0, 0);
	return true;
}

/* ---------------------------------------------------------------------- */
/* Settings                                                                */

/*
 * Locate element @idx of setting @key.  Scalar settings live in a first
 * region in descriptor order; every descriptor also reserves count * size
 * bytes in a second region, which holds the array settings (type bit 6).
 */
static const u8 *setting_find(const struct nwf_engine *e, u16 key,
			      unsigned int idx, unsigned int *size)
{
	const u8 *s = e->settings;
	unsigned int tl, vl, nd, i, a = 0, b = 0, sz;
	const u8 *d, *vals;
	bool scalar;

	if (!e->settings_valid)
		return NULL;

	tl = get_unaligned_be16(s);
	vl = get_unaligned_be16(s + 2);
	nd = tl / 4;
	d = s + 6;
	vals = d + tl;

	for (i = 0; i < nd && d[4 * i + 2]; i++) {
		u8 t = d[4 * i + 2];

		sz = (t & 0x3f) < ARRAY_SIZE(setting_size) ?
		     setting_size[t & 0x3f] : 0;
		if (!sz)
			return NULL;
		if ((~t & 0xc0) == 0xc0 || (~t & 0xc0) == 0x40)
			b += sz;
	}

	for (i = 0; i < nd && d[4 * i + 2]; i++) {
		u8 t = d[4 * i + 2];
		unsigned int count = d[4 * i + 3];
		unsigned int off;

		sz = setting_size[t & 0x3f];
		scalar = (~t & 0xc0) == 0xc0 || (~t & 0xc0) == 0x40;

		if ((d[4 * i] << 8 | d[4 * i + 1]) == key) {
			if (scalar && idx)
				return NULL;
			if (!scalar && idx >= count)
				return NULL;
			off = scalar ? a : b + idx * sz;
			if (off + sz > vl)
				return NULL;
			*size = sz;
			return vals + off;
		}
		if (scalar)
			a += sz;
		b += count * sz;
	}
	return NULL;
}

static bool setting_float(const struct nwf_engine *e, u16 key,
			  unsigned int idx, u32 *bits)
{
	unsigned int sz;
	const u8 *p = setting_find(e, key, idx, &sz);

	if (!p || sz != 4)
		return false;
	*bits = get_unaligned_be32(p);
	return ((*bits >> 23) & 0xff) != 0xff;
}

/*
 * Array settings are append-only logs in the controller's flash: a new
 * value goes into the next erased (all 0xff) element, and the last element
 * written is the one in force.  Returns its index, or -1.
 */
static int setting_active(const struct nwf_engine *e, u16 key)
{
	unsigned int idx, sz, i;
	const u8 *p;
	int last = -1;

	for (idx = 0; (p = setting_find(e, key, idx, &sz)); idx++) {
		for (i = 0; i < sz && p[i] == 0xff; i++)
			;
		if (i < sz)
			last = idx;
	}
	return last;
}

static bool setting_s16(const struct nwf_engine *e, u16 key, s32 *v)
{
	unsigned int sz;
	const u8 *p = setting_find(e, key, 0, &sz);

	if (!p || sz != 2)
		return false;
	*v = (s16)get_unaligned_be16(p);
	return true;
}

/* Lens angle term poly(512 - 2p) / 512 per pixel, Q24 radians. */
static void build_lens(struct nwf_engine *e, unsigned int c)
{
	struct nwf_cam *cam = &e->cam[c];
	u16 key = (CAM_GROUP(c) << 8) | KEY_LENS_C0;
	u32 coef[7];
	unsigned int k, p;
	int set;

	cam->lens_valid = false;

	/*
	 * Use the newest set of coefficients; element 0 holds the factory
	 * default (identity).  The linear term decides which set is newest:
	 * a set is written term by term with the linear term last, and a
	 * term left erased in a set is zero.
	 */
	for (set = setting_active(e, key + 1); set >= 0; set--) {
		for (k = 0; k < 7; k++)
			if (!setting_float(e, key + k, set, &coef[k]))
				coef[k] = 0;
		if (coef[1])
			break;
	}
	if (set < 0)
		return;

	for (p = 0; p < NWF_PIXELS; p++) {
		s64 v = 0;

		for (k = 0; k < 7; k++)
			v += float_term(coef[k], 512 - 2 * (s32)p, k, 24 - 9);
		cam->lens[p] = (s32)clamp(v, -Q24_LENS_MAX, Q24_LENS_MAX);
	}
	cam->lens_valid = true;
	cam->rot_valid = false;
}

/* Least-squares affine map from the stored measured points to the targets. */
static void build_calibration(struct nwf_engine *e)
{
	s64 mx[4], my[4], tx[4], ty[4];
	s64 sx = 0, sy = 0, stx = 0, sty = 0;
	s64 sxx = 0, sxy = 0, syy = 0, sxtx = 0, sytx = 0, sxty = 0, syty = 0;
	s64 det, dets;
	unsigned int i;
	u32 bits;
	s32 v;
	int set;

	e->cal_valid = false;
	e->tilt = 0;

	for (i = 0; i < 4; i++) {
		if (!setting_float(e, KEY_CAL_TARGET + 2 * i, 0, &bits) ||
		    !float_to_int(bits, &v))
			return;
		tx[i] = v;
		if (!setting_float(e, KEY_CAL_TARGET + 2 * i + 1, 0, &bits) ||
		    !float_to_int(bits, &v))
			return;
		ty[i] = v;
	}

	/* The newest record: the last one all eight measured values reached. */
	set = setting_active(e, KEY_CAL_MEAS);
	for (i = 1; i < 8; i++)
		set = min(set, setting_active(e, KEY_CAL_MEAS + i));

	for (; set >= 0; set--) {
		for (i = 0; i < 4; i++) {
			if (!setting_float(e, KEY_CAL_MEAS + 2 * i, set, &bits) ||
			    !float_to_int(bits, &v) || v < 0 || v > 2 * NWF_MAX_COORD)
				break;
			mx[i] = v;
			if (!setting_float(e, KEY_CAL_MEAS + 2 * i + 1, set, &bits) ||
			    !float_to_int(bits, &v) || v < 0 || v > 2 * NWF_MAX_COORD)
				break;
			my[i] = v;
		}
		if (i == 4)
			break;
	}
	if (set < 0)
		return;

	for (i = 0; i < 4; i++) {
		sx += mx[i];
		sy += my[i];
		stx += tx[i];
		sty += ty[i];
	}
	for (i = 0; i < 4; i++) {
		s64 dx = 4 * mx[i] - sx, dy = 4 * my[i] - sy;
		s64 dtx = 4 * tx[i] - stx, dty = 4 * ty[i] - sty;

		/* centred values are 4x; divide to keep products in range */
		dx /= 4;
		dy /= 4;
		dtx /= 4;
		dty /= 4;
		sxx += dx * dx;
		sxy += dx * dy;
		syy += dy * dy;
		sxtx += dx * dtx;
		sytx += dy * dtx;
		sxty += dx * dty;
		syty += dy * dty;
	}

	det = sxx * syy - sxy * sxy;
	dets = det >> 16;
	if (dets <= 0)
		return;

	e->cal_x[0] = div64_s64(syy * sxtx - sxy * sytx, dets);
	e->cal_x[1] = div64_s64(sxx * sytx - sxy * sxtx, dets);
	e->cal_y[0] = div64_s64(syy * sxty - sxy * syty, dets);
	e->cal_y[1] = div64_s64(sxx * syty - sxy * sxty, dets);
	e->cal_x[2] = (stx * 65536 - e->cal_x[0] * sx - e->cal_x[1] * sy) / 4;
	e->cal_y[2] = (sty * 65536 - e->cal_y[0] * sx - e->cal_y[1] * sy) / 4;

	/*
	 * The rotation part of the map is the tilt of the camera baseline
	 * against the screen edges (small angle: tan ~ angle).  Averaged
	 * over both axes, as the map also carries some shear.
	 */
	if (e->cal_x[0] > 0 && e->cal_y[1] > 0)
		e->tilt = (s32)((div64_s64(e->cal_y[0] << 24, e->cal_y[1]) -
				 div64_s64(e->cal_x[1] << 24, e->cal_x[0])) / 2);
	e->cal_valid = true;
}

static void process_settings(struct nwf_engine *e, const u8 *p, unsigned int len)
{
	unsigned int c;
	s32 w, h;

	if (len < 6 || len > NWF_SETTINGS_MAX ||
	    6U + get_unaligned_be16(p) + get_unaligned_be16(p + 2) > len)
		return;
	if (e->settings_valid && len == e->settings_len &&
	    !memcmp(e->settings, p, len))
		return;		/* resent unchanged about once a second */

	memcpy(e->settings, p, len);
	e->settings_len = len;
	e->settings_valid = true;

	if (setting_s16(e, KEY_SCREEN_W, &w) && setting_s16(e, KEY_SCREEN_H, &h) &&
	    w > 0 && h > 0) {
		e->aspect_w = w;
		e->aspect_h = h;
	} else {
		e->aspect_w = 16;
		e->aspect_h = 9;
	}

	for (c = 0; c < NWF_CAMS; c++)
		build_lens(e, c);
	build_calibration(e);

	pr_debug("nwfermi: settings %u bytes, screen %d:%d, lens %d/%d, cal %d\n",
		 len, e->aspect_w, e->aspect_h, e->cam[0].lens_valid,
		 e->cam[1].lens_valid, e->cal_valid);
}

/* ---------------------------------------------------------------------- */
/* Shadow detection                                                        */

static s32 lens_at(const struct nwf_cam *cam, s32 pos_q8)
{
	s32 i = pos_q8 >> 8, f = pos_q8 & 0xff;

	if (i < 0)
		return cam->lens[0];
	if (i >= NWF_PIXELS - 1)
		return cam->lens[NWF_PIXELS - 1];
	return cam->lens[i] + (s32)(((s64)(cam->lens[i + 1] - cam->lens[i]) * f) >> 8);
}

/*
 * The far bottom corner of the bezel gives the mounting rotation.  In the
 * baseline it is a sharp step: the bright far side wall ends and the bottom
 * wall, seen at a grazing angle, is dark.  The corner is the step's
 * half-height crossing.  (The dark region's own minimum lies a variable 4-7
 * pixels past the corner, so it is a poor reference.)
 */
static void update_rotation(struct nwf_engine *e, struct nwf_cam *cam)
{
	u32 *tmp = e->sig;	/* scratch; detect_dips is done with it */
	unsigned int i, j, n, best = CORNER_LO;
	u32 hi, lo, half;
	s32 pos;

	if (!cam->lens_valid || !cam->base_valid)
		return;

	for (i = CORNER_LO; i < CORNER_HI; i++)
		if (cam->base[i] < cam->base[best])
			best = i;

	/* bright level: median of the side wall just before the step */
	n = CORNER_SPAN - CORNER_GAP;
	for (i = 0; i < n; i++) {
		u32 v = cam->base[best - CORNER_SPAN + i];

		for (j = i; j > 0 && tmp[j - 1] > v; j--)
			tmp[j] = tmp[j - 1];
		tmp[j] = v;
	}
	hi = tmp[n / 2];
	lo = cam->base[best];
	if (hi < 2 * lo || hi < BASE_MIN_Q4)
		return;		/* no clear step */
	half = (hi + lo) / 2;

	for (i = best; i > best - CORNER_SPAN && cam->base[i] < half; i--)
		;
	if (cam->base[i] < half)
		return;
	/* crossing between i (bright) and i + 1 (dark), Q8 */
	pos = (s32)(i << 8) + (s32)div_u64((u64)(cam->base[i] - half) << 8,
					   cam->base[i] - cam->base[i + 1]);

	cam->corner_pos = pos;
	/*
	 * The corner's direction is fixed to the screen, but angles are
	 * measured from the baseline between the cameras: a tilted
	 * baseline opens the corner angle for one camera and closes it
	 * for the other.
	 */
	cam->rot = cordic_atan2(e->aspect_h, e->aspect_w) + lens_at(cam, pos) +
		   (cam == e->cam ? e->tilt : -e->tilt);
	cam->rot_valid = true;
}

static void detect_dips(struct nwf_engine *e, unsigned int c)
{
	struct nwf_cam *cam = &e->cam[c];
	unsigned int on = 1, off = 0, first, last, p;
	u64 sum_on = 0, sum_off = 0;
	struct nwf_dip *d = NULL;
	u32 *sig = e->sig;
	u16 *ratio = e->ratio;
	u16 min_ratio = 0xff;
	int gap = 0;

	first = max(cam->first[0], cam->first[1]) + 4;
	last = min(cam->last[0], cam->last[1]);
	last = last > 4 ? last - 4 : 0;

	for (p = first; p < last; p++) {
		sum_on += cam->wave[1][p];
		sum_off += cam->wave[0][p];
	}
	if (sum_off > sum_on) {
		on = 0;
		off = 1;
	}

	for (p = 0; p < NWF_PIXELS; p++) {
		s32 v = p >= first && p < last ?
			(s32)cam->wave[on][p] - cam->wave[off][p] : 0;

		sig[p] = max(v, 0) << 4;
	}

	if (!cam->base_valid) {
		for (p = 0; p < NWF_PIXELS; p++)
			cam->base[p] = cam->base_frames ? max(cam->base[p], sig[p]) : sig[p];
		if (++cam->base_frames >= BASE_INIT_FRAMES)
			cam->base_valid = true;
		cam->ndips = 0;
		return;
	}

	for (p = 0; p < NWF_PIXELS; p++) {
		u32 b = cam->base[p];

		ratio[p] = b < BASE_MIN_Q4 ? 256 :
			   (u16)min_t(u64, div_u64((u64)sig[p] << 8, b), 256);
	}

	/* Runs below threshold, merging across small gaps. */
	cam->ndips = 0;
	for (p = first; p <= last; p++) {
		bool in = p < last && ratio[p] < DIP_RATIO_Q8;

		if (in) {
			if (!d) {
				if (cam->ndips == NWF_MAX_DIPS)
					break;
				d = &cam->dips[cam->ndips];
				memset(d, 0, sizeof(*d));
				d->pos = p;	/* first pixel, fixed up below */
				min_ratio = 0xff;
			}
			d->area += 256 - ratio[p];
			d->width = p - d->pos + 1;	/* pos holds the start */
			min_ratio = min_t(u16, min_ratio, ratio[p]);
			gap = 0;
			continue;
		}
		if (d && ++gap > DIP_MERGE_GAP) {
			if (d->width >= DIP_MIN_WIDTH && min_ratio < DIP_DEPTH_Q8)
				cam->ndips++;
			d = NULL;
		}
	}
	if (d && d->width >= DIP_MIN_WIDTH && min_ratio < DIP_DEPTH_Q8)
		cam->ndips++;

	/* Weighted centroids (Q8). */
	for (p = 0; p < cam->ndips; p++) {
		struct nwf_dip *dp = &cam->dips[p];
		unsigned int s = dp->pos, i;
		s64 w = 0, wp = 0;

		for (i = s; i < s + dp->width; i++) {
			if (ratio[i] < 256) {
				w += 256 - ratio[i];
				wp += (s64)(256 - ratio[i]) * i;
			}
		}
		dp->pos = w ? (s32)div64_s64(wp << 8, w) : (s32)(s << 8);
	}

	/* Adapt the baseline outside shadows: fast up, slow down. */
	for (p = 0; p < NWF_PIXELS; p++) {
		u32 b = cam->base[p];
		bool shadowed = false;
		unsigned int i;

		for (i = 0; i < cam->ndips; i++) {
			s32 lo = (cam->dips[i].pos >> 8) - cam->dips[i].width;
			s32 hi = (cam->dips[i].pos >> 8) + cam->dips[i].width;

			if ((s32)p >= lo && (s32)p <= hi)
				shadowed = true;
		}
		if (shadowed)
			continue;
		if (sig[p] > b)
			cam->base[p] = b + ((sig[p] - b) >> 3);
		else
			cam->base[p] = b - ((b - sig[p]) >> 8);
	}

	if (!cam->rot_valid || !(e->frames % ROT_REFRESH_FRAMES))
		update_rotation(e, cam);
}

/* ---------------------------------------------------------------------- */
/* Triangulation and tracking                                              */

struct nwf_point {
	s32 x, y;
	s64 d0, d1;		/* squared distance to each camera (raw units) */
	bool ok;
};

/* Intersect the rays of dip d0 (camera 0) and d1 (camera 1). */
static struct nwf_point triangulate(const struct nwf_engine *e,
				    const struct nwf_dip *d0,
				    const struct nwf_dip *d1)
{
	const struct nwf_cam *c0 = &e->cam[0], *c1 = &e->cam[1];
	struct nwf_point pt = { 0 };
	s32 a0 = c0->rot - lens_at(c0, d0->pos);
	s32 a1 = c1->rot - lens_at(c1, d1->pos);
	s32 cs0, sn0, cs1, sn1;
	s64 den, rx, ry;

	if (a0 <= 0 || a1 <= 0 || a0 >= Q24_PI_2 || a1 >= Q24_PI_2)
		return pt;

	cordic_sincos(a0, &cs0, &sn0);
	cordic_sincos(a1, &cs1, &sn1);
	cs0 >>= 15; sn0 >>= 15; cs1 >>= 15; sn1 >>= 15;	/* Q15 */

	den = (s64)sn0 * cs1 + (s64)cs0 * sn1;		/* sin(a0 + a1), Q30 */
	if (den < (1 << 20))
		return pt;

	/* Raw position, both axes in units of the screen width / 32767. */
	rx = div64_s64((s64)NWF_MAX_COORD * sn1 * cs0, den);
	ry = div64_s64((s64)NWF_MAX_COORD * sn1 * sn0, den);
	pt.d0 = rx * rx + ry * ry;
	pt.d1 = (NWF_MAX_COORD - rx) * (NWF_MAX_COORD - rx) + ry * ry;

	if (e->cal_valid) {
		pt.x = (s32)((e->cal_x[0] * rx + e->cal_x[1] * ry + e->cal_x[2]) >> 16);
		pt.y = (s32)((e->cal_y[0] * rx + e->cal_y[1] * ry + e->cal_y[2]) >> 16);
	} else {
		pt.x = (s32)rx;
		pt.y = (s32)div64_s64(ry * e->aspect_w, e->aspect_h);
	}

	pt.ok = pt.x > -NWF_MAX_COORD / 16 && pt.x < NWF_MAX_COORD + NWF_MAX_COORD / 16 &&
		pt.y > -NWF_MAX_COORD / 16 && pt.y < NWF_MAX_COORD + NWF_MAX_COORD / 16;
	pt.x = clamp(pt.x, 0, NWF_MAX_COORD);
	pt.y = clamp(pt.y, 0, NWF_MAX_COORD);
	return pt;
}

/*
 * Apparent finger size squared: shadow width times distance, averaged over
 * both cameras.  Widths are Q8 pixels, so this stays well inside s64.
 */
static s64 finger_size2(const struct nwf_point *pt, const struct nwf_dip *a,
			const struct nwf_dip *b)
{
	s64 wa = a->area >> 4, wb = b->area >> 4;

	return (wa * wa * (pt->d0 >> 8) + wb * wb * (pt->d1 >> 8)) / 2;
}

/* Relative difference of two positive values, parts per 1024. */
static s64 rel_diff(s64 a, s64 b)
{
	return a + b > 0 ? div64_s64(abs(a - b) << 10, a + b) : 1024;
}

static s64 dist2(s32 ax, s32 ay, s32 bx, s32 by)
{
	return (s64)(ax - bx) * (ax - bx) + (s64)(ay - by) * (ay - by);
}

/* Strongest dips first. */
static void sort_dips(struct nwf_cam *cam)
{
	unsigned int i, j;

	for (i = 1; i < cam->ndips; i++) {
		struct nwf_dip v = cam->dips[i];

		for (j = i; j > 0 && cam->dips[j - 1].area < v.area; j--)
			cam->dips[j] = cam->dips[j - 1];
		cam->dips[j] = v;
	}
}

static unsigned int solve_points(struct nwf_engine *e, struct nwf_point *pts)
{
	struct nwf_cam *c0 = &e->cam[0], *c1 = &e->cam[1];
	unsigned int n0, n1, i, n = 0;
	struct nwf_point a[2], b[2];
	s64 cost_a = 0, cost_b = 0;
	unsigned int active = 0, k;

	sort_dips(c0);
	sort_dips(c1);
	n0 = min_t(unsigned int, c0->ndips, NWF_MAX_CONTACTS);
	n1 = min_t(unsigned int, c1->ndips, NWF_MAX_CONTACTS);

	for (i = 0; i < NWF_MAX_CONTACTS; i++)
		active += e->contact[i].active;

	if (n0 == n1)
		e->asym_frames = 0;

	if (!n0 || !n1)
		return 0;

	if (n0 == 1 && n1 == 1) {
		pts[0] = triangulate(e, &c0->dips[0], &c1->dips[0]);
		return pts[0].ok;
	}

	if (n0 == 1 || n1 == 1) {
		/*
		 * A second finger usually reaches one camera a frame or two
		 * before the other, and its crossing with the old finger's
		 * shadow is then a ghost.  With one finger tracked, report
		 * just that finger until both cameras see the new one, or
		 * until it has been in line with one camera for a while.
		 */
		if (active == 1 && e->asym_frames++ < ASYM_FRAMES) {
			const struct nwf_contact *ct = e->contact[0].active ?
						       &e->contact[0] : &e->contact[1];
			s64 best = S64_MAX;

			for (i = 0; i < max(n0, n1); i++) {
				struct nwf_point p = triangulate(e,
						&c0->dips[n0 == 1 ? 0 : i],
						&c1->dips[n1 == 1 ? 0 : i]);
				s64 d = dist2(p.x, p.y, ct->x, ct->y);

				if (p.ok && d < best) {
					best = d;
					pts[0] = p;
				}
			}
			return best < S64_MAX;
		}

		/* One camera sees both fingers in line: both crossings are real. */
		for (i = 0; i < max(n0, n1); i++) {
			pts[n] = triangulate(e, &c0->dips[n0 == 1 ? 0 : i],
					     &c1->dips[n1 == 1 ? 0 : i]);
			n += pts[n].ok;
		}
		return n;
	}

	/* 2 x 2: two real contacts and two ghosts. */
	a[0] = triangulate(e, &c0->dips[0], &c1->dips[0]);
	a[1] = triangulate(e, &c0->dips[1], &c1->dips[1]);
	b[0] = triangulate(e, &c0->dips[0], &c1->dips[1]);
	b[1] = triangulate(e, &c0->dips[1], &c1->dips[0]);

	if (active) {
		/*
		 * Keep the confirmed contacts where they are.  Fingers rarely
		 * land in the same frame, so usually one is already tracked
		 * when the second appears.  Unconfirmed candidates do not
		 * count: one camera often sees a new finger a frame before
		 * the other, and the crossing made then is a ghost.
		 */
		for (k = 0; k < NWF_MAX_CONTACTS; k++) {
			const struct nwf_contact *ct = &e->contact[k];

			if (!ct->active)
				continue;
			cost_a += min(dist2(a[0].x, a[0].y, ct->x, ct->y),
				      dist2(a[1].x, a[1].y, ct->x, ct->y));
			cost_b += min(dist2(b[0].x, b[0].y, ct->x, ct->y),
				      dist2(b[1].x, b[1].y, ct->x, ct->y));
		}
	} else {
		/*
		 * New pair of contacts.  Real fingers are about the same size;
		 * the two ghosts of a pair are one large and one small "finger".
		 */
		cost_a = (!a[0].ok + !a[1].ok) * (1LL << 40) +
			 rel_diff(finger_size2(&a[0], &c0->dips[0], &c1->dips[0]),
				  finger_size2(&a[1], &c0->dips[1], &c1->dips[1]));
		cost_b = (!b[0].ok + !b[1].ok) * (1LL << 40) +
			 rel_diff(finger_size2(&b[0], &c0->dips[0], &c1->dips[1]),
				  finger_size2(&b[1], &c0->dips[1], &c1->dips[0]));
	}

	for (i = 0; i < 2; i++) {
		const struct nwf_point *pp = cost_a <= cost_b ? &a[i] : &b[i];

		if (pp->ok)
			pts[n++] = *pp;
	}
	return n;
}

static void track(struct nwf_engine *e, const struct nwf_point *pts,
		  unsigned int n)
{
	bool used[NWF_MAX_CONTACTS] = { false };
	struct nwf_contact out[NWF_MAX_CONTACTS];
	unsigned int i, k, nout = 0;

	/* Greedy nearest match: at most two contacts. */
	for (k = 0; k < NWF_MAX_CONTACTS; k++) {
		struct nwf_contact *ct = &e->contact[k];
		s64 best = (s64)TRACK_MAX_DIST * TRACK_MAX_DIST;
		int bi = -1;

		if (!ct->age)
			continue;
		for (i = 0; i < n; i++) {
			s64 d = dist2(pts[i].x, pts[i].y, ct->x, ct->y);

			if (!used[i] && d < best) {
				best = d;
				bi = i;
			}
		}
		if (bi >= 0) {
			used[bi] = true;
			ct->x = pts[bi].x;
			ct->y = pts[bi].y;
			ct->age++;
			ct->missed = 0;
		} else if (!ct->active || ++ct->missed >= TOUCH_UP_FRAMES) {
			/* an unconfirmed candidate gets no second chance */
			ct->age = 0;
			ct->active = false;
		}
	}

	for (i = 0; i < n; i++) {
		if (used[i])
			continue;
		for (k = 0; k < NWF_MAX_CONTACTS; k++) {
			struct nwf_contact *ct = &e->contact[k];

			if (ct->age)
				continue;
			ct->x = pts[i].x;
			ct->y = pts[i].y;
			ct->id = e->next_id++;
			ct->age = 1;
			ct->missed = 0;
			ct->active = false;
			break;
		}
	}

	for (k = 0; k < NWF_MAX_CONTACTS; k++) {
		struct nwf_contact *ct = &e->contact[k];

		if (ct->age >= TOUCH_DOWN_FRAMES)
			ct->active = true;
		if (ct->active)
			out[nout++] = *ct;
	}

	if (e->report)
		e->report(e->ctx, out, nout);
}

static void solve(struct nwf_engine *e)
{
	struct nwf_point pts[NWF_MAX_CONTACTS];
	unsigned int n = 0;

	e->frames++;
	if (e->cam[0].rot_valid && e->cam[1].rot_valid)
		n = solve_points(e, pts);
	track(e, pts, n);
}

/* ---------------------------------------------------------------------- */
/* Stream parsing                                                          */

/*
 * Decode a waveform packet starting at @p (payload, @avail bytes).
 * Returns bytes consumed, 0 if more data is needed, or -1 if malformed.
 */
static int process_waveform(struct nwf_engine *e, const u8 *p,
			    unsigned int avail, unsigned int len)
{
	unsigned int count, first, i, off = WAVE_HDR, c, seq;
	struct nwf_cam *cam;
	u16 *wave;
	s32 v = 0;

	if (len < WAVE_HDR)
		return -1;
	if (avail < WAVE_HDR)
		return 0;

	c = p[2] >> 4;
	seq = p[2] & 0xf;
	first = get_unaligned_be16(p + 3);
	count = len - WAVE_HDR;
	if (first + count > NWF_PIXELS)
		return -1;

	/* Find the end first: escapes make the payload longer than len. */
	for (i = 0; i < count; i++) {
		if (off >= avail)
			return 0;
		off += p[off] == 0x80 ? 3 : 1;
	}
	if (off > avail)
		return 0;

	if (c >= NWF_CAMS || seq >= NWF_SEQS)
		return off;		/* not a camera we use */

	cam = &e->cam[c];
	wave = cam->wave[seq];
	memset(wave, 0, sizeof(cam->wave[seq]));
	off = WAVE_HDR;
	for (i = 0; i < count; i++) {
		if (p[off] == 0x80) {
			v = get_unaligned_be16(p + off + 1);
			off += 3;
		} else {
			v = max(v + (s8)p[off], 0);
			off += 1;
		}
		wave[first + i] = (u16)min(v, 0xffff);
	}

	/* Mirrored sensor: flip so pixel 0 looks along the top edge. */
	if (p[7] & 1) {
		for (i = 0; i < NWF_PIXELS / 2; i++) {
			u16 t = wave[i];

			wave[i] = wave[NWF_PIXELS - 1 - i];
			wave[NWF_PIXELS - 1 - i] = t;
		}
		first = NWF_PIXELS - first - count;
	}
	cam->first[seq] = first;
	cam->last[seq] = first + count;
	cam->have |= BIT(seq);

	if (cam->have == (BIT(0) | BIT(1))) {
		cam->have = 0;
		detect_dips(e, c);
		cam->fresh = true;
		if (e->cam[0].fresh && e->cam[1].fresh) {
			e->cam[0].fresh = false;
			e->cam[1].fresh = false;
			solve(e);
		}
	}
	return off;
}

void nwf_engine_feed(struct nwf_engine *e, const u8 *data, unsigned int len)
{
	unsigned int pos = 0;

	while (len) {
		unsigned int n = min(len, (unsigned int)sizeof(e->buf) - e->len);

		memcpy(e->buf + e->len, data, n);
		e->len += n;
		data += n;
		len -= n;

		for (;;) {
			unsigned int avail = e->len - pos, plen;
			const u8 *p = e->buf + pos;
			int used;

			if (avail < PKT_HDR)
				break;
			if (p[0] != '[' || p[1] != ']') {
				pos++;
				continue;
			}
			plen = get_unaligned_be16(p + 3);

			if (p[5] == PKT_WAVEFORM) {
				used = process_waveform(e, p + PKT_HDR,
							avail - PKT_HDR, plen);
			} else {
				if (plen > sizeof(e->buf) - PKT_HDR) {
					used = -1;
				} else if (avail - PKT_HDR < plen) {
					used = 0;
				} else {
					if (p[5] == PKT_SETTINGS && p[6] == SETTINGS_SUB)
						process_settings(e, p + PKT_HDR, plen);
					used = plen;
				}
			}

			if (used < 0) {
				e->bad_packets++;
				pos++;		/* resynchronise on the next header */
				continue;
			}
			if (!used) {
				if (pos == 0 && e->len == sizeof(e->buf)) {
					pos++;	/* cannot complete: skip it */
					e->bad_packets++;
					continue;
				}
				break;
			}
			e->packets++;
			pos += PKT_HDR + used;
		}

		memmove(e->buf, e->buf + pos, e->len - pos);
		e->len -= pos;
		pos = 0;
	}
}

/* No frame for a while: the controller stops streaming once untouched. */
void nwf_engine_idle(struct nwf_engine *e)
{
	unsigned int k;
	bool any = false;

	for (k = 0; k < NWF_MAX_CONTACTS; k++) {
		any |= e->contact[k].active;
		e->contact[k].age = 0;
		e->contact[k].active = false;
	}
	if (any && e->report)
		e->report(e->ctx, NULL, 0);
}

void nwf_engine_init(struct nwf_engine *e,
		     void (*report)(void *ctx, const struct nwf_contact *c,
				    unsigned int n),
		     void *ctx)
{
	memset(e, 0, sizeof(*e));
	e->aspect_w = 16;
	e->aspect_h = 9;
	e->report = report;
	e->ctx = ctx;
}
