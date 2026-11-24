// SPDX-License-Identifier: GPL-2.0-only
/* Replay a captured Fermi stream through the touch engine. */
#include "../../nwfermi_engine.h"

static double now, last_frame;
static struct nwf_engine e;
static FILE *dump;	/* NWF_DUMP=file: per-frame shadows and final geometry */

static void report(void *ctx, const struct nwf_contact *c, unsigned int n)
{
	unsigned int i;

	last_frame = now;
	if (dump) {
		unsigned int k, d;

		fprintf(dump, "D %.6f", now);
		for (k = 0; k < NWF_CAMS; k++) {
			fprintf(dump, " %u", e.cam[k].ndips);
			for (d = 0; d < e.cam[k].ndips; d++)
				fprintf(dump, " %d %u %u", e.cam[k].dips[d].pos,
					e.cam[k].dips[d].width, e.cam[k].dips[d].area);
		}
		fprintf(dump, "\n");
	}
	printf("%.6f %u", now, n);
	for (i = 0; i < n; i++)
		printf(" %u %d %d", c[i].id, c[i].x, c[i].y);
	printf("\n");
}

int main(int argc, char **argv)
{
	static unsigned char buf[1 << 17];
	FILE *f = fopen(argv[1], "rb");
	unsigned int len;

	if (!f)
		return 1;
	if (getenv("NWF_DUMP"))
		dump = fopen(getenv("NWF_DUMP"), "w");
	nwf_engine_init(&e, report, NULL);
	while (fread(&now, sizeof(now), 1, f) == 1 &&
	       fread(&len, sizeof(len), 1, f) == 1 && len <= sizeof(buf) &&
	       fread(buf, 1, len, f) == len) {
		if (last_frame && now - last_frame > NWF_IDLE_TIMEOUT_MS / 1000.0) {
			double t = now;

			now = last_frame + NWF_IDLE_TIMEOUT_MS / 1000.0;
			nwf_engine_idle(&e);
			now = t;
			last_frame = 0;
		}
		nwf_engine_feed(&e, buf, len);
	}
	fprintf(stderr, "packets %lu bad %lu frames %lu lens %d/%d rot %d/%d cal %d\n",
		e.packets, e.bad_packets, e.frames, e.cam[0].lens_valid,
		e.cam[1].lens_valid, e.cam[0].rot_valid, e.cam[1].rot_valid,
		e.cal_valid);
	fprintf(stderr, "rot deg %.3f %.3f  cal x %.5f %.5f %.1f  y %.5f %.5f %.1f\n",
		e.cam[0].rot / 16777216.0 * 57.29578, e.cam[1].rot / 16777216.0 * 57.29578,
		e.cal_x[0] / 65536.0, e.cal_x[1] / 65536.0, e.cal_x[2] / 65536.0,
		e.cal_y[0] / 65536.0, e.cal_y[1] / 65536.0, e.cal_y[2] / 65536.0);
	if (dump) {
		unsigned int k, p;

		for (k = 0; k < NWF_CAMS; k++) {
			fprintf(dump, "L %u %d %d", k, e.cam[k].rot, e.cam[k].corner_pos);
			for (p = 0; p < NWF_PIXELS; p++)
				fprintf(dump, " %d", e.cam[k].lens[p]);
			fprintf(dump, "\n");
		}
		fprintf(dump, "C %d %d %lld %lld %lld %lld %lld %lld\n", e.aspect_w, e.aspect_h,
			(long long)e.cal_x[0], (long long)e.cal_x[1], (long long)e.cal_x[2],
			(long long)e.cal_y[0], (long long)e.cal_y[1], (long long)e.cal_y[2]);
		fclose(dump);
	}
	return 0;
}
