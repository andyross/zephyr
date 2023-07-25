/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>

/* (Cribbed from Zephyr: arch/x86/zefi/printf.h) */

/* Tiny, but not-as-primitive-as-it-looks implementation of something
 * like s/n/printf().  Handles %d, %x, %p, %c and %s only, allows a
 * "l" qualifier on %d and %x (and silently ignores one %s/%c/%p).
 * Accepts, but ignores, field width and precision values that match:
 * the regex: [0-9]*\.?[0-9]*
 */

/* Use this 1M region for the log data (see notes in mt8195-load.py).
 * Remember that the firmware loader doesn't (!!!) zero this RAM, and
 * that it starts out populated with seeming garbage from firmware
 * startup (there are some bootloader-looking strings in there).  So
 * something very early on needs to set MTPRINTF_LEN=0.
 *
 * Obviously this will cause overruns the second log data enters the
 * last four bytes.  It's a debug tool.  NOT RELIABLE FOR PRODUCTION
 * USE.
 */
#define MTPRINTF_BUF ((volatile char *)0x60700000)
#define MTPRINTF_LEN (*(volatile int*)0x607ffffc)

static inline void z_putchar(char c)
{
	volatile char *p = &MTPRINTF_BUF[MTPRINTF_LEN];
	p[1] = 0;
	p[0] = c;
	MTPRINTF_LEN++;
	__asm__ volatile("dhwb %0, 0; dhwb %0, 4" :: "r"(p));
}

struct _pfr {
	char *buf;
	int len;
	int idx;
};

static inline void pc(struct _pfr *r, int c)
{
	if (r->buf != NULL) {
		if (r->idx <= r->len) {
			r->buf[r->idx] = c;
		}
	} else {
		z_putchar(c);
	}
	r->idx++;
}

static inline void prdec(struct _pfr *r, long v)
{
	if (v < 0) {
		pc(r, '-');
		v = -v;
	}

	char digs[11 * sizeof(long)/4];
	int i = sizeof(digs) - 1;

	digs[i--] = 0;
	while (v || i == 9) {
		digs[i--] = '0' + (v % 10);
		v /= 10;
	}

	while (digs[++i] != '\0') {
		pc(r, digs[i]);
	}
}

static inline void endrec(struct _pfr *r)
{
	if (r->buf && r->idx < r->len) {
		r->buf[r->idx] = 0;
	}
}

static inline int vpf(struct _pfr *r, const char *f, va_list ap)
{
	for (/**/; *f != '\0'; f++) {
		bool islong = false;

		if (*f != '%') {
			pc(r, *f);
			continue;
		}

		if (f[1] == 'l') {
			islong = sizeof(long) > 4;
			f++;
		}

		/* Ignore (but accept) field width and precision values */
		while (f[1] >= '0' && f[1] <= '9') {
			f++;
		}
		if (f[1] == '.') {
			f++;
		}
		while (f[1] >= '0' && f[1] <= '9') {
			f++;
		}

		switch (*(++f)) {
		case 0:
			return r->idx;
		case '%':
			pc(r, '%');
			break;
		case 'c':
			pc(r, va_arg(ap, int));
			break;
		case 's': {
			char *s = va_arg(ap, char *);

			while (*s != '\0')
				pc(r, *s++);
			break;
		}
		case 'p':
			pc(r, '0');
			pc(r, 'x'); /* fall through... */
			islong = sizeof(long) > 4;
		case 'x': {
			int i, sig = 0;
			unsigned long v = islong ? va_arg(ap, unsigned long)
				: va_arg(ap, unsigned int);
			for (i = 2*sizeof(long) - 1; i >= 0; i--) {
				int d = (v >> (i*4)) & 0xf;

				sig += !!d;
				if (sig || i == 0)
					pc(r, "0123456789abcdef"[d]);
			}
			break;
		}
		case 'd':
			prdec(r, va_arg(ap, int));
			break;
		default:
			pc(r, '%');
			pc(r, *f);
		}
	}
	endrec(r);
	return r->idx;
}

#define CALL_VPF(rec)				\
	va_list ap;				\
	va_start(ap, f);			\
	ret = vpf(&r, f, ap);			\
	va_end(ap);

static inline int mtsnprintf(char *buf, unsigned long len, const char *f, ...)
{
	int ret = 0;
	struct _pfr r = { .buf = buf, .len = len };

	CALL_VPF(&r);
	return ret;
}

static inline int mtsprintf(char *buf, const char *f, ...)
{
	int ret = 0;
	struct _pfr r = { .buf = buf, .len = 0x7fffffff };

	CALL_VPF(&r);
	return ret;
}

static inline int mtprintf(const char *f, ...)
{
	int ret = 0;
	struct _pfr r = {0};

	CALL_VPF(&r);
	return ret;
}
