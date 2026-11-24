/* SPDX-License-Identifier: GPL-2.0-only */
/* Minimal kernel-API shim so nwfermi_engine.c builds in userspace. */
#ifndef _KSHIM_H
#define _KSHIM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

#define BIT(n)			(1UL << (n))
#define S64_MAX			INT64_MAX
#define ARRAY_SIZE(a)		(sizeof(a) / sizeof((a)[0]))
#define min(a, b)		((a) < (b) ? (a) : (b))
#define max(a, b)		((a) > (b) ? (a) : (b))
#define min_t(t, a, b)		((t)(a) < (t)(b) ? (t)(a) : (t)(b))
#define max_t(t, a, b)		((t)(a) > (t)(b) ? (t)(a) : (t)(b))
#define clamp(v, lo, hi)	min(max(v, lo), hi)
#define abs(x)			((x) < 0 ? -(x) : (x))

static inline u16 get_unaligned_be16(const void *p)
{
	const u8 *b = p;

	return (u16)(b[0] << 8 | b[1]);
}

static inline u32 get_unaligned_be32(const void *p)
{
	const u8 *b = p;

	return (u32)b[0] << 24 | (u32)b[1] << 16 | (u32)b[2] << 8 | b[3];
}

static inline u64 mul_u64_u64_shr(u64 a, u64 b, unsigned int shift)
{
	return (u64)(((unsigned __int128)a * b) >> shift);
}

static inline u64 div_u64(u64 a, u32 b)
{
	return a / b;
}

static inline s64 div64_s64(s64 a, s64 b)
{
	return a / b;
}

#define pr_debug(...)		do { if (getenv("NWF_DEBUG")) fprintf(stderr, __VA_ARGS__); } while (0)

#endif
