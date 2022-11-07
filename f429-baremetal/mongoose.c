// Copyright (c) 2004-2013 Sergey Lyubka
// Copyright (c) 2013-2022 Cesanta Software Limited
// All rights reserved
//
// This software is dual-licensed: you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation. For the terms of this
// license, see http://www.gnu.org/licenses/
//
// You are free to use this software under the terms of the GNU General
// Public License, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// Alternatively, you can license this software under a commercial
// license, as set out in https://www.mongoose.ws/licensing/
//
// SPDX-License-Identifier: GPL-2.0-only or commercial

#include "mongoose.h"

#ifdef MG_ENABLE_LINES
#line 1 "src/base64.c"
#endif



static int mg_b64idx(int c) {
  if (c < 26) {
    return c + 'A';
  } else if (c < 52) {
    return c - 26 + 'a';
  } else if (c < 62) {
    return c - 52 + '0';
  } else {
    return c == 62 ? '+' : '/';
  }
}

static int mg_b64rev(int c) {
  if (c >= 'A' && c <= 'Z') {
    return c - 'A';
  } else if (c >= 'a' && c <= 'z') {
    return c + 26 - 'a';
  } else if (c >= '0' && c <= '9') {
    return c + 52 - '0';
  } else if (c == '+') {
    return 62;
  } else if (c == '/') {
    return 63;
  } else if (c == '=') {
    return 64;
  } else {
    return -1;
  }
}

int mg_base64_update(unsigned char ch, char *to, int n) {
  int rem = (n & 3) % 3;
  if (rem == 0) {
    to[n] = (char) mg_b64idx(ch >> 2);
    to[++n] = (char) ((ch & 3) << 4);
  } else if (rem == 1) {
    to[n] = (char) mg_b64idx(to[n] | (ch >> 4));
    to[++n] = (char) ((ch & 15) << 2);
  } else {
    to[n] = (char) mg_b64idx(to[n] | (ch >> 6));
    to[++n] = (char) mg_b64idx(ch & 63);
    n++;
  }
  return n;
}

int mg_base64_final(char *to, int n) {
  int saved = n;
  // printf("---[%.*s]\n", n, to);
  if (n & 3) n = mg_base64_update(0, to, n);
  if ((saved & 3) == 2) n--;
  // printf("    %d[%.*s]\n", n, n, to);
  while (n & 3) to[n++] = '=';
  to[n] = '\0';
  return n;
}

int mg_base64_encode(const unsigned char *p, int n, char *to) {
  int i, len = 0;
  for (i = 0; i < n; i++) len = mg_base64_update(p[i], to, len);
  len = mg_base64_final(to, len);
  return len;
}

int mg_base64_decode(const char *src, int n, char *dst) {
  const char *end = src == NULL ? NULL : src + n;  // Cannot add to NULL
  int len = 0;
  while (src != NULL && src + 3 < end) {
    int a = mg_b64rev(src[0]), b = mg_b64rev(src[1]), c = mg_b64rev(src[2]),
        d = mg_b64rev(src[3]);
    if (a == 64 || a < 0 || b == 64 || b < 0 || c < 0 || d < 0) return 0;
    dst[len++] = (char) ((a << 2) | (b >> 4));
    if (src[2] != '=') {
      dst[len++] = (char) ((b << 4) | (c >> 2));
      if (src[3] != '=') dst[len++] = (char) ((c << 6) | d);
    }
    src += 4;
  }
  dst[len] = '\0';
  return len;
}

#ifdef MG_ENABLE_LINES
#line 1 "src/dns.c"
#endif







struct dns_data {
  struct dns_data *next;
  struct mg_connection *c;
  uint64_t expire;
  uint16_t txnid;
};

static void mg_sendnsreq(struct mg_connection *, struct mg_str *, int,
                         struct mg_dns *, bool);

static void mg_dns_free(struct mg_connection *c, struct dns_data *d) {
  LIST_DELETE(struct dns_data,
              (struct dns_data **) &c->mgr->active_dns_requests, d);
  free(d);
}

void mg_resolve_cancel(struct mg_connection *c) {
  struct dns_data *tmp, *d = (struct dns_data *) c->mgr->active_dns_requests;
  for (; d != NULL; d = tmp) {
    tmp = d->next;
    if (d->c == c) mg_dns_free(c, d);
  }
}

static size_t mg_dns_parse_name_depth(const uint8_t *s, size_t len, size_t ofs,
                                      char *to, size_t tolen, size_t j,
                                      int depth) {
  size_t i = 0;
  if (tolen > 0 && depth == 0) to[0] = '\0';
  if (depth > 5) return 0;
  // MG_INFO(("ofs %lx %x %x", (unsigned long) ofs, s[ofs], s[ofs + 1]));
  while (ofs + i + 1 < len) {
    size_t n = s[ofs + i];
    if (n == 0) {
      i++;
      break;
    }
    if (n & 0xc0) {
      size_t ptr = (((n & 0x3f) << 8) | s[ofs + i + 1]);  // 12 is hdr len
      // MG_INFO(("PTR %lx", (unsigned long) ptr));
      if (ptr + 1 < len && (s[ptr] & 0xc0) == 0 &&
          mg_dns_parse_name_depth(s, len, ptr, to, tolen, j, depth + 1) == 0)
        return 0;
      i += 2;
      break;
    }
    if (ofs + i + n + 1 >= len) return 0;
    if (j > 0) {
      if (j < tolen) to[j] = '.';
      j++;
    }
    if (j + n < tolen) memcpy(&to[j], &s[ofs + i + 1], n);
    j += n;
    i += n + 1;
    if (j < tolen) to[j] = '\0';  // Zero-terminate this chunk
    // MG_INFO(("--> [%s]", to));
  }
  if (tolen > 0) to[tolen - 1] = '\0';  // Make sure make sure it is nul-term
  return i;
}

static size_t mg_dns_parse_name(const uint8_t *s, size_t n, size_t ofs,
                                char *dst, size_t dstlen) {
  return mg_dns_parse_name_depth(s, n, ofs, dst, dstlen, 0, 0);
}

size_t mg_dns_parse_rr(const uint8_t *buf, size_t len, size_t ofs,
                       bool is_question, struct mg_dns_rr *rr) {
  const uint8_t *s = buf + ofs, *e = &buf[len];

  memset(rr, 0, sizeof(*rr));
  if (len < sizeof(struct mg_dns_header)) return 0;  // Too small
  if (len > 512) return 0;  //  Too large, we don't expect that
  if (s >= e) return 0;     //  Overflow

  if ((rr->nlen = (uint16_t) mg_dns_parse_name(buf, len, ofs, NULL, 0)) == 0)
    return 0;
  s += rr->nlen + 4;
  if (s > e) return 0;
  rr->atype = (uint16_t) (((uint16_t) s[-4] << 8) | s[-3]);
  rr->aclass = (uint16_t) (((uint16_t) s[-2] << 8) | s[-1]);
  if (is_question) return (size_t) (rr->nlen + 4);

  s += 6;
  if (s > e) return 0;
  rr->alen = (uint16_t) (((uint16_t) s[-2] << 8) | s[-1]);
  if (s + rr->alen > e) return 0;
  return (size_t) (rr->nlen + rr->alen + 10);
}

bool mg_dns_parse(const uint8_t *buf, size_t len, struct mg_dns_message *dm) {
  const struct mg_dns_header *h = (struct mg_dns_header *) buf;
  struct mg_dns_rr rr;
  size_t i, n, ofs = sizeof(*h);
  memset(dm, 0, sizeof(*dm));

  if (len < sizeof(*h)) return 0;                // Too small, headers dont fit
  if (mg_ntohs(h->num_questions) > 1) return 0;  // Sanity
  if (mg_ntohs(h->num_answers) > 10) return 0;   // Sanity
  dm->txnid = mg_ntohs(h->txnid);

  for (i = 0; i < mg_ntohs(h->num_questions); i++) {
    if ((n = mg_dns_parse_rr(buf, len, ofs, true, &rr)) == 0) return false;
    // MG_INFO(("Q %lu %lu %hu/%hu", ofs, n, rr.atype, rr.aclass));
    ofs += n;
  }
  for (i = 0; i < mg_ntohs(h->num_answers); i++) {
    if ((n = mg_dns_parse_rr(buf, len, ofs, false, &rr)) == 0) return false;
    // MG_INFO(("A -- %lu %lu %hu/%hu %s", ofs, n, rr.atype, rr.aclass,
    // dm->name));
    mg_dns_parse_name(buf, len, ofs, dm->name, sizeof(dm->name));
    ofs += n;

    if (rr.alen == 4 && rr.atype == 1 && rr.aclass == 1) {
      dm->addr.is_ip6 = false;
      memcpy(&dm->addr.ip, &buf[ofs - 4], 4);
      dm->resolved = true;
      break;  // Return success
    } else if (rr.alen == 16 && rr.atype == 28 && rr.aclass == 1) {
      dm->addr.is_ip6 = true;
      memcpy(&dm->addr.ip6, &buf[ofs - 16], 16);
      dm->resolved = true;
      break;  // Return success
    }
  }
  return true;
}

static void dns_cb(struct mg_connection *c, int ev, void *ev_data,
                   void *fn_data) {
  struct dns_data *d, *tmp;
  if (ev == MG_EV_POLL) {
    uint64_t now = *(uint64_t *) ev_data;
    for (d = (struct dns_data *) c->mgr->active_dns_requests; d != NULL;
         d = tmp) {
      tmp = d->next;
      // MG_DEBUG ("%lu %lu dns poll", d->expire, now));
      if (now > d->expire) mg_error(d->c, "DNS timeout");
    }
  } else if (ev == MG_EV_READ) {
    struct mg_dns_message dm;
    int resolved = 0;
    if (mg_dns_parse(c->recv.buf, c->recv.len, &dm) == false) {
      MG_ERROR(("Unexpected DNS response:"));
      mg_hexdump(c->recv.buf, c->recv.len);
    } else {
      // MG_VERBOSE(("%s %d", dm.name, dm.resolved));
      for (d = (struct dns_data *) c->mgr->active_dns_requests; d != NULL;
           d = tmp) {
        tmp = d->next;
        // MG_INFO(("d %p %hu %hu", d, d->txnid, dm.txnid));
        if (dm.txnid != d->txnid) continue;
        if (d->c->is_resolving) {
          if (dm.resolved) {
            char buf[100];
            dm.addr.port = d->c->rem.port;  // Save port
            d->c->rem = dm.addr;            // Copy resolved address
            MG_DEBUG(("%lu %s is %s", d->c->id, dm.name,
                      mg_ntoa(&d->c->rem, buf, sizeof(buf))));
            mg_connect_resolved(d->c);
#if MG_ENABLE_IPV6
          } else if (dm.addr.is_ip6 == false && dm.name[0] != '\0' &&
                     c->mgr->use_dns6 == false) {
            struct mg_str x = mg_str(dm.name);
            mg_sendnsreq(d->c, &x, c->mgr->dnstimeout, &c->mgr->dns6, true);
#endif
          } else {
            mg_error(d->c, "%s DNS lookup failed", dm.name);
          }
        } else {
          MG_ERROR(("%lu already resolved", d->c->id));
        }
        mg_dns_free(c, d);
        resolved = 1;
      }
    }
    if (!resolved) MG_ERROR(("stray DNS reply"));
    c->recv.len = 0;
  } else if (ev == MG_EV_CLOSE) {
    for (d = (struct dns_data *) c->mgr->active_dns_requests; d != NULL;
         d = tmp) {
      tmp = d->next;
      mg_error(d->c, "DNS error");
      mg_dns_free(c, d);
    }
  }
  (void) fn_data;
}

static bool mg_dns_send(struct mg_connection *c, const struct mg_str *name,
                        uint16_t txnid, bool ipv6) {
  struct {
    struct mg_dns_header header;
    uint8_t data[256];
  } pkt;
  size_t i, n;
  memset(&pkt, 0, sizeof(pkt));
  pkt.header.txnid = mg_htons(txnid);
  pkt.header.flags = mg_htons(0x100);
  pkt.header.num_questions = mg_htons(1);
  for (i = n = 0; i < sizeof(pkt.data) - 5; i++) {
    if (name->ptr[i] == '.' || i >= name->len) {
      pkt.data[n] = (uint8_t) (i - n);
      memcpy(&pkt.data[n + 1], name->ptr + n, i - n);
      n = i + 1;
    }
    if (i >= name->len) break;
  }
  memcpy(&pkt.data[n], "\x00\x00\x01\x00\x01", 5);  // A query
  n += 5;
  if (ipv6) pkt.data[n - 3] = 0x1c;  // AAAA query
  // memcpy(&pkt.data[n], "\xc0\x0c\x00\x1c\x00\x01", 6);  // AAAA query
  // n += 6;
  return mg_send(c, &pkt, sizeof(pkt.header) + n);
}

static void mg_sendnsreq(struct mg_connection *c, struct mg_str *name, int ms,
                         struct mg_dns *dnsc, bool ipv6) {
  struct dns_data *d = NULL;
  if (dnsc->url == NULL) {
    mg_error(c, "DNS server URL is NULL. Call mg_mgr_init()");
  } else if (dnsc->c == NULL) {
    dnsc->c = mg_connect(c->mgr, dnsc->url, NULL, NULL);
    if (dnsc->c != NULL) {
      dnsc->c->pfn = dns_cb;
      // dnsc->c->is_hexdumping = 1;
    }
  }
  if (dnsc->c == NULL) {
    mg_error(c, "resolver");
  } else if ((d = (struct dns_data *) calloc(1, sizeof(*d))) == NULL) {
    mg_error(c, "resolve OOM");
  } else {
    struct dns_data *reqs = (struct dns_data *) c->mgr->active_dns_requests;
    char buf[100];
    d->txnid = reqs ? (uint16_t) (reqs->txnid + 1) : 1;
    d->next = (struct dns_data *) c->mgr->active_dns_requests;
    c->mgr->active_dns_requests = d;
    d->expire = mg_millis() + (uint64_t) ms;
    d->c = c;
    c->is_resolving = 1;
    MG_VERBOSE(("%lu resolving %.*s @ %s, txnid %hu", c->id, (int) name->len,
                name->ptr, mg_ntoa(&dnsc->c->rem, buf, sizeof(buf)), d->txnid));
    if (!mg_dns_send(dnsc->c, name, d->txnid, ipv6)) {
      mg_error(dnsc->c, "DNS send");
    }
  }
}

void mg_resolve(struct mg_connection *c, const char *url) {
  struct mg_str host = mg_url_host(url);
  c->rem.port = mg_htons(mg_url_port(url));
  if (mg_aton(host, &c->rem)) {
    // host is an IP address, do not fire name resolution
    mg_connect_resolved(c);
  } else {
    // host is not an IP, send DNS resolution request
    struct mg_dns *dns = c->mgr->use_dns6 ? &c->mgr->dns6 : &c->mgr->dns4;
    mg_sendnsreq(c, &host, c->mgr->dnstimeout, dns, c->mgr->use_dns6);
  }
}

#ifdef MG_ENABLE_LINES
#line 1 "src/event.c"
#endif





void mg_call(struct mg_connection *c, int ev, void *ev_data) {
  // Run user-defined handler first, in order to give it an ability
  // to intercept processing (e.g. clean input buffer) before the
  // protocol handler kicks in
  if (c->fn != NULL) c->fn(c, ev, ev_data, c->fn_data);
  if (c->pfn != NULL) c->pfn(c, ev, ev_data, c->pfn_data);
}

void mg_error(struct mg_connection *c, const char *fmt, ...) {
  char buf[64];
  va_list ap;
  va_start(ap, fmt);
  mg_vsnprintf(buf, sizeof(buf), fmt, &ap);
  va_end(ap);
  MG_ERROR(("%lu %p %s", c->id, c->fd, buf));
  c->is_closing = 1;             // Set is_closing before sending MG_EV_CALL
  mg_call(c, MG_EV_ERROR, buf);  // Let user handler to override it
}

#ifdef MG_ENABLE_LINES
#line 1 "src/fmt.c"
#endif



static void mg_pfn_iobuf_private(char ch, void *param, bool expand) {
  struct mg_iobuf *io = (struct mg_iobuf *) param;
  if (expand && io->len + 2 > io->size) mg_iobuf_resize(io, io->len + 2);
  if (io->len + 2 <= io->size) {
    io->buf[io->len++] = (uint8_t) ch;
    io->buf[io->len] = 0;
  } else if (io->len < io->size) {
    io->buf[io->len++] = 0;  // Guarantee to 0-terminate
  }
}

static void mg_putchar_iobuf_static(char ch, void *param) {
  mg_pfn_iobuf_private(ch, param, false);
}

void mg_pfn_iobuf(char ch, void *param) {
  mg_pfn_iobuf_private(ch, param, true);
}

size_t mg_vsnprintf(char *buf, size_t len, const char *fmt, va_list *ap) {
  struct mg_iobuf io = {(uint8_t *) buf, len, 0, 0};
  size_t n = mg_vxprintf(mg_putchar_iobuf_static, &io, fmt, ap);
  if (n < len) buf[n] = '\0';
  return n;
}

size_t mg_snprintf(char *buf, size_t len, const char *fmt, ...) {
  va_list ap;
  size_t n;
  va_start(ap, fmt);
  n = mg_vsnprintf(buf, len, fmt, &ap);
  va_end(ap);
  return n;
}

char *mg_vmprintf(const char *fmt, va_list *ap) {
  struct mg_iobuf io = {0, 0, 0, 256};
  mg_vxprintf(mg_pfn_iobuf, &io, fmt, ap);
  return (char *) io.buf;
}

char *mg_mprintf(const char *fmt, ...) {
  char *s;
  va_list ap;
  va_start(ap, fmt);
  s = mg_vmprintf(fmt, &ap);
  va_end(ap);
  return s;
}

size_t mg_xprintf(void (*out)(char, void *), void *ptr, const char *fmt, ...) {
  size_t len = 0;
  va_list ap;
  va_start(ap, fmt);
  len = mg_vxprintf(out, ptr, fmt, &ap);
  va_end(ap);
  return len;
}

static bool is_digit(int c) {
  return c >= '0' && c <= '9';
}

static int addexp(char *buf, int e, int sign) {
  int n = 0;
  buf[n++] = 'e';
  buf[n++] = (char) sign;
  if (e > 400) return 0;
  if (e < 10) buf[n++] = '0';
  if (e >= 100) buf[n++] = (char) (e / 100 + '0'), e -= 100 * (e / 100);
  if (e >= 10) buf[n++] = (char) (e / 10 + '0'), e -= 10 * (e / 10);
  buf[n++] = (char) (e + '0');
  return n;
}

static int xisinf(double x) {
  union {
    double f;
    uint64_t u;
  } ieee754 = {x};
  return ((unsigned) (ieee754.u >> 32) & 0x7fffffff) == 0x7ff00000 &&
         ((unsigned) ieee754.u == 0);
}

static int xisnan(double x) {
  union {
    double f;
    uint64_t u;
  } ieee754 = {x};
  return ((unsigned) (ieee754.u >> 32) & 0x7fffffff) +
             ((unsigned) ieee754.u != 0) >
         0x7ff00000;
}

static size_t mg_dtoa(char *dst, size_t dstlen, double d, int width) {
  char buf[40];
  int i, s = 0, n = 0, e = 0;
  double t, mul, saved;
  if (d == 0.0) return mg_snprintf(dst, dstlen, "%s", "0");
  if (xisinf(d)) return mg_snprintf(dst, dstlen, "%s", d > 0 ? "inf" : "-inf");
  if (xisnan(d)) return mg_snprintf(dst, dstlen, "%s", "nan");
  if (d < 0.0) d = -d, buf[s++] = '-';

  // Round
  saved = d;
  mul = 1.0;
  while (d >= 10.0 && d / mul >= 10.0) mul *= 10.0;
  while (d <= 1.0 && d / mul <= 1.0) mul /= 10.0;
  for (i = 0, t = mul * 5; i < width; i++) t /= 10.0;
  d += t;
  // Calculate exponent, and 'mul' for scientific representation
  mul = 1.0;
  while (d >= 10.0 && d / mul >= 10.0) mul *= 10.0, e++;
  while (d < 1.0 && d / mul < 1.0) mul /= 10.0, e--;
  // printf(" --> %g %d %g %g\n", saved, e, t, mul);

  if (e >= width) {
    n = (int) mg_dtoa(buf, sizeof(buf), saved / mul, width);
    // printf(" --> %.*g %d [%.*s]\n", 10, d / t, e, n, buf);
    n += addexp(buf + s + n, e, '+');
    return mg_snprintf(dst, dstlen, "%.*s", n, buf);
  } else if (e <= -width) {
    n = (int) mg_dtoa(buf, sizeof(buf), saved / mul, width);
    // printf(" --> %.*g %d [%.*s]\n", 10, d / mul, e, n, buf);
    n += addexp(buf + s + n, -e, '-');
    return mg_snprintf(dst, dstlen, "%.*s", n, buf);
  } else {
    for (i = 0, t = mul; t >= 1.0 && s + n < (int) sizeof(buf); i++) {
      int ch = (int) (d / t);
      if (n > 0 || ch > 0) buf[s + n++] = (char) (ch + '0');
      d -= ch * t;
      t /= 10.0;
    }
    // printf(" --> [%g] -> %g %g (%d) [%.*s]\n", saved, d, t, n, s + n, buf);
    if (n == 0) buf[s++] = '0';
    while (t >= 1.0 && n + s < (int) sizeof(buf)) buf[n++] = '0', t /= 10.0;
    if (s + n < (int) sizeof(buf)) buf[n + s++] = '.';
    // printf(" 1--> [%g] -> [%.*s]\n", saved, s + n, buf);
    for (i = 0, t = 0.1; s + n < (int) sizeof(buf) && n < width; i++) {
      int ch = (int) (d / t);
      buf[s + n++] = (char) (ch + '0');
      d -= ch * t;
      t /= 10.0;
    }
  }
  while (n > 0 && buf[s + n - 1] == '0') n--;  // Trim trailing zeros
  if (n > 0 && buf[s + n - 1] == '.') n--;     // Trim trailing dot
  n += s;
  if (n >= (int) sizeof(buf)) n = (int) sizeof(buf) - 1;
  buf[n] = '\0';
  return mg_snprintf(dst, dstlen, "%s", buf);
}

static size_t mg_lld(char *buf, int64_t val, bool is_signed, bool is_hex) {
  const char *letters = "0123456789abcdef";
  uint64_t v = (uint64_t) val;
  size_t s = 0, n, i;
  if (is_signed && val < 0) buf[s++] = '-', v = (uint64_t) (-val);
  // This loop prints a number in reverse order. I guess this is because we
  // write numbers from right to left: least significant digit comes last.
  // Maybe because we use Arabic numbers, and Arabs write RTL?
  if (is_hex) {
    for (n = 0; v; v >>= 4) buf[s + n++] = letters[v & 15];
  } else {
    for (n = 0; v; v /= 10) buf[s + n++] = letters[v % 10];
  }
  // Reverse a string
  for (i = 0; i < n / 2; i++) {
    char t = buf[s + i];
    buf[s + i] = buf[s + n - i - 1], buf[s + n - i - 1] = t;
  }
  if (val == 0) buf[n++] = '0';  // Handle special case
  return n + s;
}

static size_t scpy(void (*out)(char, void *), void *ptr, char *buf,
                   size_t len) {
  size_t i = 0;
  while (i < len && buf[i] != '\0') out(buf[i++], ptr);
  return i;
}

static char mg_esc(int c, bool esc) {
  const char *p, *esc1 = "\b\f\n\r\t\\\"", *esc2 = "bfnrt\\\"";
  for (p = esc ? esc1 : esc2; *p != '\0'; p++) {
    if (*p == c) return esc ? esc2[p - esc1] : esc1[p - esc2];
  }
  return 0;
}

static char mg_escape(int c) {
  return mg_esc(c, true);
}

static size_t qcpy(void (*out)(char, void *), void *ptr, char *buf,
                   size_t len) {
  size_t i = 0, extra = 0;
  for (i = 0; i < len && buf[i] != '\0'; i++) {
    char c = mg_escape(buf[i]);
    if (c) {
      out('\\', ptr), out(c, ptr), extra++;
    } else {
      out(buf[i], ptr);
    }
  }
  return i + extra;
}

static size_t Qcpy(void (*out)(char, void *), void *ptr, char *buf,
                   size_t len) {
  size_t n = 2;
  out('"', ptr);
  n += qcpy(out, ptr, buf, len);
  out('"', ptr);
  return n;
}

static size_t bcpy(void (*out)(char, void *), void *ptr, uint8_t *buf,
                   size_t len) {
  size_t i, n = 0;
  const char *t =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  out('"', ptr), n++;
  for (i = 0; i < len; i += 3) {
    uint8_t c1 = buf[i], c2 = i + 1 < len ? buf[i + 1] : 0,
            c3 = i + 2 < len ? buf[i + 2] : 0;
    char tmp[4] = {t[c1 >> 2], t[(c1 & 3) << 4 | (c2 >> 4)], '=', '='};
    if (i + 1 < len) tmp[2] = t[(c2 & 15) << 2 | (c3 >> 6)];
    if (i + 2 < len) tmp[3] = t[c3 & 63];
    n += scpy(out, ptr, tmp, sizeof(tmp));
  }
  out('"', ptr), n++;
  return n;
}

size_t mg_vxprintf(void (*out)(char, void *), void *param, const char *fmt,
                   va_list *ap) {
  size_t i = 0, n = 0;
  while (fmt[i] != '\0') {
    if (fmt[i] == '%') {
      size_t j, k, x = 0, is_long = 0, w = 0 /* width */, pr = ~0U /* prec */;
      char pad = ' ', minus = 0, c = fmt[++i];
      if (c == '#') x++, c = fmt[++i];
      if (c == '-') minus++, c = fmt[++i];
      if (c == '0') pad = '0', c = fmt[++i];
      while (is_digit(c)) w *= 10, w += (size_t) (c - '0'), c = fmt[++i];
      if (c == '.') {
        c = fmt[++i];
        if (c == '*') {
          pr = (size_t) va_arg(*ap, int);
          c = fmt[++i];
        } else {
          pr = 0;
          while (is_digit(c)) pr *= 10, pr += (size_t) (c - '0'), c = fmt[++i];
        }
      }
      while (c == 'h') c = fmt[++i];  // Treat h and hh as int
      if (c == 'l') {
        is_long++, c = fmt[++i];
        if (c == 'l') is_long++, c = fmt[++i];
      }
      if (c == 'p') x = 1, is_long = 1;
      if (c == 'd' || c == 'u' || c == 'x' || c == 'X' || c == 'p' ||
          c == 'g' || c == 'f') {
        bool s = (c == 'd'), h = (c == 'x' || c == 'X' || c == 'p');
        char tmp[40];
        size_t xl = x ? 2 : 0;
        if (c == 'g' || c == 'f') {
          double v = va_arg(*ap, double);
          if (pr == ~0U) pr = 6;
          k = mg_dtoa(tmp, sizeof(tmp), v, (int) pr);
        } else if (is_long == 2) {
          int64_t v = va_arg(*ap, int64_t);
          k = mg_lld(tmp, v, s, h);
        } else if (is_long == 1) {
          long v = va_arg(*ap, long);
          k = mg_lld(tmp, s ? (int64_t) v : (int64_t) (unsigned long) v, s, h);
        } else {
          int v = va_arg(*ap, int);
          k = mg_lld(tmp, s ? (int64_t) v : (int64_t) (unsigned) v, s, h);
        }
        for (j = 0; j < xl && w > 0; j++) w--;
        for (j = 0; pad == ' ' && !minus && k < w && j + k < w; j++)
          n += scpy(out, param, &pad, 1);
        n += scpy(out, param, (char *) "0x", xl);
        for (j = 0; pad == '0' && k < w && j + k < w; j++)
          n += scpy(out, param, &pad, 1);
        n += scpy(out, param, tmp, k);
        for (j = 0; pad == ' ' && minus && k < w && j + k < w; j++)
          n += scpy(out, param, &pad, 1);
      } else if (c == 'M') {
        mg_pm_t f = va_arg(*ap, mg_pm_t);
        n += f(out, param, ap);
      } else if (c == 'c') {
        int ch = va_arg(*ap, int);
        out((char) ch, param);
        n++;
      } else if (c == 'H') {
        // Print hex-encoded double-quoted string
        size_t bl = (size_t) va_arg(*ap, int);
        uint8_t *p = va_arg(*ap, uint8_t *), dquote = '"';
        const char *hex = "0123456789abcdef";
        n += scpy(out, param, (char *) &dquote, 1);
        for (j = 0; j < bl; j++) {
          n += scpy(out, param, (char *) &hex[(p[j] >> 4) & 15], 1);
          n += scpy(out, param, (char *) &hex[p[j] & 15], 1);
        }
        n += scpy(out, param, (char *) &dquote, 1);
      } else if (c == 'V') {
        // Print base64-encoded double-quoted string
        size_t len = (size_t) va_arg(*ap, int);
        uint8_t *buf = va_arg(*ap, uint8_t *);
        n += bcpy(out, param, buf, len);
      } else if (c == 's' || c == 'Q' || c == 'q') {
        char *p = va_arg(*ap, char *);
        size_t (*f)(void (*)(char, void *), void *, char *, size_t) = scpy;
        if (c == 'Q') f = Qcpy;
        if (c == 'q') f = qcpy;
        if (pr == ~0U) pr = p == NULL ? 0 : strlen(p);
        for (j = 0; !minus && pr < w && j + pr < w; j++)
          n += f(out, param, &pad, 1);
        n += f(out, param, p, pr);
        for (j = 0; minus && pr < w && j + pr < w; j++)
          n += f(out, param, &pad, 1);
      } else if (c == '%') {
        out('%', param);
        n++;
      } else {
        out('%', param);
        out(c, param);
        n += 2;
      }
      i++;
    } else {
      out(fmt[i], param), n++, i++;
    }
  }
  return n;
}

#ifdef MG_ENABLE_LINES
#line 1 "src/fs.c"
#endif



struct mg_fd *mg_fs_open(struct mg_fs *fs, const char *path, int flags) {
  struct mg_fd *fd = (struct mg_fd *) calloc(1, sizeof(*fd));
  if (fd != NULL) {
    fd->fd = fs->op(path, flags);
    fd->fs = fs;
    if (fd->fd == NULL) {
      free(fd);
      fd = NULL;
    }
  }
  return fd;
}

void mg_fs_close(struct mg_fd *fd) {
  if (fd != NULL) {
    fd->fs->cl(fd->fd);
    free(fd);
  }
}

char *mg_file_read(struct mg_fs *fs, const char *path, size_t *sizep) {
  struct mg_fd *fd;
  char *data = NULL;
  size_t size = 0;
  fs->st(path, &size, NULL);
  if ((fd = mg_fs_open(fs, path, MG_FS_READ)) != NULL) {
    data = (char *) calloc(1, size + 1);
    if (data != NULL) {
      if (fs->rd(fd->fd, data, size) != size) {
        free(data);
        data = NULL;
      } else {
        data[size] = '\0';
        if (sizep != NULL) *sizep = size;
      }
    }
    mg_fs_close(fd);
  }
  return data;
}

bool mg_file_write(struct mg_fs *fs, const char *path, const void *buf,
                   size_t len) {
  bool result = false;
  struct mg_fd *fd;
  char tmp[MG_PATH_MAX];
  mg_snprintf(tmp, sizeof(tmp), "%s..%d", path, rand());
  if ((fd = mg_fs_open(fs, tmp, MG_FS_WRITE)) != NULL) {
    result = fs->wr(fd->fd, buf, len) == len;
    mg_fs_close(fd);
    if (result) {
      fs->rm(path);
      fs->mv(tmp, path);
    } else {
      fs->rm(tmp);
    }
  }
  return result;
}

bool mg_file_printf(struct mg_fs *fs, const char *path, const char *fmt, ...) {
  va_list ap;
  char *data;
  bool result = false;
  va_start(ap, fmt);
  data = mg_vmprintf(fmt, &ap);
  va_end(ap);
  result = mg_file_write(fs, path, data, strlen(data));
  free(data);
  return result;
}

#ifdef MG_ENABLE_LINES
#line 1 "src/fs_fat.c"
#endif



#if MG_ENABLE_FATFS
#include <ff.h>

static int mg_days_from_epoch(int y, int m, int d) {
  y -= m <= 2;
  int era = y / 400;
  int yoe = y - era * 400;
  int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

static time_t mg_timegm(const struct tm *t) {
  int year = t->tm_year + 1900;
  int month = t->tm_mon;  // 0-11
  if (month > 11) {
    year += month / 12;
    month %= 12;
  } else if (month < 0) {
    int years_diff = (11 - month) / 12;
    year -= years_diff;
    month += 12 * years_diff;
  }
  int x = mg_days_from_epoch(year, month + 1, t->tm_mday);
  return 60 * (60 * (24L * x + t->tm_hour) + t->tm_min) + t->tm_sec;
}

static time_t ff_time_to_epoch(uint16_t fdate, uint16_t ftime) {
  struct tm tm;
  memset(&tm, 0, sizeof(struct tm));
  tm.tm_sec = (ftime << 1) & 0x3e;
  tm.tm_min = ((ftime >> 5) & 0x3f);
  tm.tm_hour = ((ftime >> 11) & 0x1f);
  tm.tm_mday = (fdate & 0x1f);
  tm.tm_mon = ((fdate >> 5) & 0x0f) - 1;
  tm.tm_year = ((fdate >> 9) & 0x7f) + 80;
  return mg_timegm(&tm);
}

static int ff_stat(const char *path, size_t *size, time_t *mtime) {
  FILINFO fi;
  if (path[0] == '\0') {
    if (size) *size = 0;
    if (mtime) *mtime = 0;
    return MG_FS_DIR;
  } else if (f_stat(path, &fi) == 0) {
    if (size) *size = (size_t) fi.fsize;
    if (mtime) *mtime = ff_time_to_epoch(fi.fdate, fi.ftime);
    return MG_FS_READ | MG_FS_WRITE | ((fi.fattrib & AM_DIR) ? MG_FS_DIR : 0);
  } else {
    return 0;
  }
}

static void ff_list(const char *dir, void (*fn)(const char *, void *),
                    void *userdata) {
  DIR d;
  FILINFO fi;
  if (f_opendir(&d, dir) == FR_OK) {
    while (f_readdir(&d, &fi) == FR_OK && fi.fname[0] != '\0') {
      if (!strcmp(fi.fname, ".") || !strcmp(fi.fname, "..")) continue;
      fn(fi.fname, userdata);
    }
    f_closedir(&d);
  }
}

static void *ff_open(const char *path, int flags) {
  FIL f;
  unsigned char mode = FA_READ;
  if (flags & MG_FS_WRITE) mode |= FA_WRITE | FA_OPEN_ALWAYS | FA_OPEN_APPEND;
  if (f_open(&f, path, mode) == 0) {
    FIL *fp = calloc(1, sizeof(*fp));
    memcpy(fp, &f, sizeof(*fp));
    return fp;
  } else {
    return NULL;
  }
}

static void ff_close(void *fp) {
  if (fp != NULL) {
    f_close((FIL *) fp);
    free(fp);
  }
}

static size_t ff_read(void *fp, void *buf, size_t len) {
  UINT n = 0, misalign = ((size_t) buf) & 3;
  if (misalign) {
    char aligned[4];
    f_read((FIL *) fp, aligned, len > misalign ? misalign : len, &n);
    memcpy(buf, aligned, n);
  } else {
    f_read((FIL *) fp, buf, len, &n);
  }
  return n;
}

static size_t ff_write(void *fp, const void *buf, size_t len) {
  UINT n = 0;
  return f_write((FIL *) fp, (char *) buf, len, &n) == FR_OK ? n : 0;
}

static size_t ff_seek(void *fp, size_t offset) {
  f_lseek((FIL *) fp, offset);
  return offset;
}

static bool ff_rename(const char *from, const char *to) {
  return f_rename(from, to) == FR_OK;
}

static bool ff_remove(const char *path) {
  return f_unlink(path) == FR_OK;
}

static bool ff_mkdir(const char *path) {
  return f_mkdir(path) == FR_OK;
}

struct mg_fs mg_fs_fat = {ff_stat,  ff_list, ff_open,   ff_close,  ff_read,
                          ff_write, ff_seek, ff_rename, ff_remove, ff_mkdir};
#endif

#ifdef MG_ENABLE_LINES
#line 1 "src/fs_packed.c"
#endif




struct packed_file {
  const char *data;
  size_t size;
  size_t pos;
};

const char *mg_unpack(const char *path, size_t *size, time_t *mtime);
const char *mg_unlist(size_t no);

#if MG_ENABLE_PACKED_FS
#else
const char *mg_unpack(const char *path, size_t *size, time_t *mtime) {
  (void) path, (void) size, (void) mtime;
  return NULL;
}
const char *mg_unlist(size_t no) {
  (void) no;
  return NULL;
}
#endif

static int is_dir_prefix(const char *prefix, size_t n, const char *path) {
  // MG_INFO(("[%.*s] [%s] %c", (int) n, prefix, path, path[n]));
  return n < strlen(path) && strncmp(prefix, path, n) == 0 &&
         (n == 0 || path[n] == '/' || path[n - 1] == '/');
}

static int packed_stat(const char *path, size_t *size, time_t *mtime) {
  const char *p;
  size_t i, n = strlen(path);
  if (mg_unpack(path, size, mtime)) return MG_FS_READ;  // Regular file
  // Scan all files. If `path` is a dir prefix for any of them, it's a dir
  for (i = 0; (p = mg_unlist(i)) != NULL; i++) {
    if (is_dir_prefix(path, n, p)) return MG_FS_DIR;
  }
  return 0;
}

static void packed_list(const char *dir, void (*fn)(const char *, void *),
                        void *userdata) {
  char buf[MG_PATH_MAX], tmp[sizeof(buf)];
  const char *path, *begin, *end;
  size_t i, n = strlen(dir);
  tmp[0] = '\0';  // Previously listed entry
  for (i = 0; (path = mg_unlist(i)) != NULL; i++) {
    if (!is_dir_prefix(dir, n, path)) continue;
    begin = &path[n + 1];
    end = strchr(begin, '/');
    if (end == NULL) end = begin + strlen(begin);
    mg_snprintf(buf, sizeof(buf), "%.*s", (int) (end - begin), begin);
    buf[sizeof(buf) - 1] = '\0';
    // If this entry has been already listed, skip
    // NOTE: we're assuming that file list is sorted alphabetically
    if (strcmp(buf, tmp) == 0) continue;
    fn(buf, userdata);  // Not yet listed, call user function
    strcpy(tmp, buf);   // And save this entry as listed
  }
}

static void *packed_open(const char *path, int flags) {
  size_t size = 0;
  const char *data = mg_unpack(path, &size, NULL);
  struct packed_file *fp = NULL;
  if (data == NULL) return NULL;
  if (flags & MG_FS_WRITE) return NULL;
  fp = (struct packed_file *) calloc(1, sizeof(*fp));
  fp->size = size;
  fp->data = data;
  return (void *) fp;
}

static void packed_close(void *fp) {
  if (fp != NULL) free(fp);
}

static size_t packed_read(void *fd, void *buf, size_t len) {
  struct packed_file *fp = (struct packed_file *) fd;
  if (fp->pos + len > fp->size) len = fp->size - fp->pos;
  memcpy(buf, &fp->data[fp->pos], len);
  fp->pos += len;
  return len;
}

static size_t packed_write(void *fd, const void *buf, size_t len) {
  (void) fd, (void) buf, (void) len;
  return 0;
}

static size_t packed_seek(void *fd, size_t offset) {
  struct packed_file *fp = (struct packed_file *) fd;
  fp->pos = offset;
  if (fp->pos > fp->size) fp->pos = fp->size;
  return fp->pos;
}

static bool packed_rename(const char *from, const char *to) {
  (void) from, (void) to;
  return false;
}

static bool packed_remove(const char *path) {
  (void) path;
  return false;
}

static bool packed_mkdir(const char *path) {
  (void) path;
  return false;
}

struct mg_fs mg_fs_packed = {
    packed_stat,  packed_list, packed_open,   packed_close,  packed_read,
    packed_write, packed_seek, packed_rename, packed_remove, packed_mkdir};

#ifdef MG_ENABLE_LINES
#line 1 "src/fs_posix.c"
#endif


#if MG_ENABLE_FILE

#ifndef MG_STAT_STRUCT
#define MG_STAT_STRUCT stat
#endif

#ifndef MG_STAT_FUNC
#define MG_STAT_FUNC stat
#endif

static int p_stat(const char *path, size_t *size, time_t *mtime) {
#if !defined(S_ISDIR)
  MG_ERROR(("stat() API is not supported. %p %p %p", path, size, mtime));
  return 0;
#else
#if MG_ARCH == MG_ARCH_WIN32
  struct _stati64 st;
  wchar_t tmp[MG_PATH_MAX];
  MultiByteToWideChar(CP_UTF8, 0, path, -1, tmp, sizeof(tmp) / sizeof(tmp[0]));
  if (_wstati64(tmp, &st) != 0) return 0;
#else
  struct MG_STAT_STRUCT st;
  if (MG_STAT_FUNC(path, &st) != 0) return 0;
#endif
  if (size) *size = (size_t) st.st_size;
  if (mtime) *mtime = st.st_mtime;
  return MG_FS_READ | MG_FS_WRITE | (S_ISDIR(st.st_mode) ? MG_FS_DIR : 0);
#endif
}

#if MG_ARCH == MG_ARCH_WIN32
struct dirent {
  char d_name[MAX_PATH];
};

typedef struct win32_dir {
  HANDLE handle;
  WIN32_FIND_DATAW info;
  struct dirent result;
} DIR;

int gettimeofday(struct timeval *tv, void *tz) {
  FILETIME ft;
  unsigned __int64 tmpres = 0;

  if (tv != NULL) {
    GetSystemTimeAsFileTime(&ft);
    tmpres |= ft.dwHighDateTime;
    tmpres <<= 32;
    tmpres |= ft.dwLowDateTime;
    tmpres /= 10;  // convert into microseconds
    tmpres -= (int64_t) 11644473600000000;
    tv->tv_sec = (long) (tmpres / 1000000UL);
    tv->tv_usec = (long) (tmpres % 1000000UL);
  }
  (void) tz;
  return 0;
}

static int to_wchar(const char *path, wchar_t *wbuf, size_t wbuf_len) {
  int ret;
  char buf[MAX_PATH * 2], buf2[MAX_PATH * 2], *p;
  strncpy(buf, path, sizeof(buf));
  buf[sizeof(buf) - 1] = '\0';
  // Trim trailing slashes. Leave backslash for paths like "X:\"
  p = buf + strlen(buf) - 1;
  while (p > buf && p[-1] != ':' && (p[0] == '\\' || p[0] == '/')) *p-- = '\0';
  memset(wbuf, 0, wbuf_len * sizeof(wchar_t));
  ret = MultiByteToWideChar(CP_UTF8, 0, buf, -1, wbuf, (int) wbuf_len);
  // Convert back to Unicode. If doubly-converted string does not match the
  // original, something is fishy, reject.
  WideCharToMultiByte(CP_UTF8, 0, wbuf, (int) wbuf_len, buf2, sizeof(buf2),
                      NULL, NULL);
  if (strcmp(buf, buf2) != 0) {
    wbuf[0] = L'\0';
    ret = 0;
  }
  return ret;
}

DIR *opendir(const char *name) {
  DIR *d = NULL;
  wchar_t wpath[MAX_PATH];
  DWORD attrs;

  if (name == NULL) {
    SetLastError(ERROR_BAD_ARGUMENTS);
  } else if ((d = (DIR *) calloc(1, sizeof(*d))) == NULL) {
    SetLastError(ERROR_NOT_ENOUGH_MEMORY);
  } else {
    to_wchar(name, wpath, sizeof(wpath) / sizeof(wpath[0]));
    attrs = GetFileAttributesW(wpath);
    if (attrs != 0Xffffffff && (attrs & FILE_ATTRIBUTE_DIRECTORY)) {
      (void) wcscat(wpath, L"\\*");
      d->handle = FindFirstFileW(wpath, &d->info);
      d->result.d_name[0] = '\0';
    } else {
      free(d);
      d = NULL;
    }
  }
  return d;
}

int closedir(DIR *d) {
  int result = 0;
  if (d != NULL) {
    if (d->handle != INVALID_HANDLE_VALUE)
      result = FindClose(d->handle) ? 0 : -1;
    free(d);
  } else {
    result = -1;
    SetLastError(ERROR_BAD_ARGUMENTS);
  }
  return result;
}

struct dirent *readdir(DIR *d) {
  struct dirent *result = NULL;
  if (d != NULL) {
    memset(&d->result, 0, sizeof(d->result));
    if (d->handle != INVALID_HANDLE_VALUE) {
      result = &d->result;
      WideCharToMultiByte(CP_UTF8, 0, d->info.cFileName, -1, result->d_name,
                          sizeof(result->d_name), NULL, NULL);
      if (!FindNextFileW(d->handle, &d->info)) {
        FindClose(d->handle);
        d->handle = INVALID_HANDLE_VALUE;
      }
    } else {
      SetLastError(ERROR_FILE_NOT_FOUND);
    }
  } else {
    SetLastError(ERROR_BAD_ARGUMENTS);
  }
  return result;
}
#endif

static void p_list(const char *dir, void (*fn)(const char *, void *),
                   void *userdata) {
#if MG_ENABLE_DIRLIST
  struct dirent *dp;
  DIR *dirp;
  if ((dirp = (opendir(dir))) == NULL) return;
  while ((dp = readdir(dirp)) != NULL) {
    if (!strcmp(dp->d_name, ".") || !strcmp(dp->d_name, "..")) continue;
    fn(dp->d_name, userdata);
  }
  closedir(dirp);
#else
  (void) dir, (void) fn, (void) userdata;
#endif
}

static void *p_open(const char *path, int flags) {
  const char *mode = flags == MG_FS_READ ? "rb" : "a+b";
#if MG_ARCH == MG_ARCH_WIN32
  wchar_t b1[MG_PATH_MAX], b2[10];
  MultiByteToWideChar(CP_UTF8, 0, path, -1, b1, sizeof(b1) / sizeof(b1[0]));
  MultiByteToWideChar(CP_UTF8, 0, mode, -1, b2, sizeof(b2) / sizeof(b2[0]));
  return (void *) _wfopen(b1, b2);
#else
  return (void *) fopen(path, mode);
#endif
}

static void p_close(void *fp) {
  fclose((FILE *) fp);
}

static size_t p_read(void *fp, void *buf, size_t len) {
  return fread(buf, 1, len, (FILE *) fp);
}

static size_t p_write(void *fp, const void *buf, size_t len) {
  return fwrite(buf, 1, len, (FILE *) fp);
}

static size_t p_seek(void *fp, size_t offset) {
#if (defined(_FILE_OFFSET_BITS) && _FILE_OFFSET_BITS == 64) ||  \
    (defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 200112L) || \
    (defined(_XOPEN_SOURCE) && _XOPEN_SOURCE >= 600)
  if (fseeko((FILE *) fp, (off_t) offset, SEEK_SET) != 0) (void) 0;
#else
  if (fseek((FILE *) fp, (long) offset, SEEK_SET) != 0) (void) 0;
#endif
  return (size_t) ftell((FILE *) fp);
}

static bool p_rename(const char *from, const char *to) {
  return rename(from, to) == 0;
}

static bool p_remove(const char *path) {
  return remove(path) == 0;
}

static bool p_mkdir(const char *path) {
  return mkdir(path, 0775) == 0;
}

#else

static int p_stat(const char *path, size_t *size, time_t *mtime) {
  (void) path, (void) size, (void) mtime;
  return 0;
}
static void p_list(const char *path, void (*fn)(const char *, void *),
                   void *userdata) {
  (void) path, (void) fn, (void) userdata;
}
static void *p_open(const char *path, int flags) {
  (void) path, (void) flags;
  return NULL;
}
static void p_close(void *fp) {
  (void) fp;
}
static size_t p_read(void *fd, void *buf, size_t len) {
  (void) fd, (void) buf, (void) len;
  return 0;
}
static size_t p_write(void *fd, const void *buf, size_t len) {
  (void) fd, (void) buf, (void) len;
  return 0;
}
static size_t p_seek(void *fd, size_t offset) {
  (void) fd, (void) offset;
  return (size_t) ~0;
}
static bool p_rename(const char *from, const char *to) {
  (void) from, (void) to;
  return false;
}
static bool p_remove(const char *path) {
  (void) path;
  return false;
}
static bool p_mkdir(const char *path) {
  (void) path;
  return false;
}
#endif

struct mg_fs mg_fs_posix = {p_stat,  p_list, p_open,   p_close,  p_read,
                            p_write, p_seek, p_rename, p_remove, p_mkdir};

#ifdef MG_ENABLE_LINES
#line 1 "src/http.c"
#endif












// Chunk deletion marker is the MSB in the "processed" counter
#define MG_DMARK ((size_t) 1 << (sizeof(size_t) * 8 - 1))

// Multipart POST example:
// --xyz
// Content-Disposition: form-data; name="val"
//
// abcdef
// --xyz
// Content-Disposition: form-data; name="foo"; filename="a.txt"
// Content-Type: text/plain
//
// hello world
//
// --xyz--
size_t mg_http_next_multipart(struct mg_str body, size_t ofs,
                              struct mg_http_part *part) {
  struct mg_str cd = mg_str_n("Content-Disposition", 19);
  const char *s = body.ptr;
  size_t b = ofs, h1, h2, b1, b2, max = body.len;

  // Init part params
  if (part != NULL) part->name = part->filename = part->body = mg_str_n(0, 0);

  // Skip boundary
  while (b + 2 < max && s[b] != '\r' && s[b + 1] != '\n') b++;
  if (b <= ofs || b + 2 >= max) return 0;
  // MG_INFO(("B: %zu %zu [%.*s]", ofs, b - ofs, (int) (b - ofs), s));

  // Skip headers
  h1 = h2 = b + 2;
  for (;;) {
    while (h2 + 2 < max && s[h2] != '\r' && s[h2 + 1] != '\n') h2++;
    if (h2 == h1) break;
    if (h2 + 2 >= max) return 0;
    // MG_INFO(("Header: [%.*s]", (int) (h2 - h1), &s[h1]));
    if (part != NULL && h1 + cd.len + 2 < h2 && s[h1 + cd.len] == ':' &&
        mg_ncasecmp(&s[h1], cd.ptr, cd.len) == 0) {
      struct mg_str v = mg_str_n(&s[h1 + cd.len + 2], h2 - (h1 + cd.len + 2));
      part->name = mg_http_get_header_var(v, mg_str_n("name", 4));
      part->filename = mg_http_get_header_var(v, mg_str_n("filename", 8));
    }
    h1 = h2 = h2 + 2;
  }
  b1 = b2 = h2 + 2;
  while (b2 + 2 + (b - ofs) + 2 < max && !(s[b2] == '\r' && s[b2 + 1] == '\n' &&
                                           memcmp(&s[b2 + 2], s, b - ofs) == 0))
    b2++;

  if (b2 + 2 >= max) return 0;
  if (part != NULL) part->body = mg_str_n(&s[b1], b2 - b1);
  // MG_INFO(("Body: [%.*s]", (int) (b2 - b1), &s[b1]));
  return b2 + 2;
}

void mg_http_bauth(struct mg_connection *c, const char *user,
                   const char *pass) {
  struct mg_str u = mg_str(user), p = mg_str(pass);
  size_t need = c->send.len + 36 + (u.len + p.len) * 2;
  if (c->send.size < need) mg_iobuf_resize(&c->send, need);
  if (c->send.size >= need) {
    int i, n = 0;
    char *buf = (char *) &c->send.buf[c->send.len + 21];
    memcpy(&buf[-21], "Authorization: Basic ", 21);  // DON'T use mg_send!
    for (i = 0; i < (int) u.len; i++) {
      n = mg_base64_update(((unsigned char *) u.ptr)[i], buf, n);
    }
    if (p.len > 0) {
      n = mg_base64_update(':', buf, n);
      for (i = 0; i < (int) p.len; i++) {
        n = mg_base64_update(((unsigned char *) p.ptr)[i], buf, n);
      }
    }
    n = mg_base64_final(buf, n);
    c->send.len += 21 + (size_t) n + 2;
    memcpy(&c->send.buf[c->send.len - 2], "\r\n", 2);
  } else {
    MG_ERROR(("%lu %s cannot resize iobuf %d->%d ", c->id, c->label,
              (int) c->send.size, (int) need));
  }
}

struct mg_str mg_http_var(struct mg_str buf, struct mg_str name) {
  struct mg_str k, v, result = mg_str_n(NULL, 0);
  while (mg_split(&buf, &k, &v, '&')) {
    if (name.len == k.len && mg_ncasecmp(name.ptr, k.ptr, k.len) == 0) {
      result = v;
      break;
    }
  }
  return result;
}

int mg_http_get_var(const struct mg_str *buf, const char *name, char *dst,
                    size_t dst_len) {
  int len;
  if (dst == NULL || dst_len == 0) {
    len = -2;  // Bad destination
  } else if (buf->ptr == NULL || name == NULL || buf->len == 0) {
    len = -1;  // Bad source
    dst[0] = '\0';
  } else {
    struct mg_str v = mg_http_var(*buf, mg_str(name));
    if (v.ptr == NULL) {
      len = -4;  // Name does not exist
    } else {
      len = mg_url_decode(v.ptr, v.len, dst, dst_len, 1);
      if (len < 0) len = -3;  // Failed to decode
    }
  }
  return len;
}

static bool isx(int c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
         (c >= 'A' && c <= 'F');
}

int mg_url_decode(const char *src, size_t src_len, char *dst, size_t dst_len,
                  int is_form_url_encoded) {
  size_t i, j;
  for (i = j = 0; i < src_len && j + 1 < dst_len; i++, j++) {
    if (src[i] == '%') {
      // Use `i + 2 < src_len`, not `i < src_len - 2`, note small src_len
      if (i + 2 < src_len && isx(src[i + 1]) && isx(src[i + 2])) {
        mg_unhex(src + i + 1, 2, (uint8_t *) &dst[j]);
        i += 2;
      } else {
        return -1;
      }
    } else if (is_form_url_encoded && src[i] == '+') {
      dst[j] = ' ';
    } else {
      dst[j] = src[i];
    }
  }
  if (j < dst_len) dst[j] = '\0';  // Null-terminate the destination
  return i >= src_len && j < dst_len ? (int) j : -1;
}

static bool isok(uint8_t c) {
  return c == '\n' || c == '\r' || c >= ' ';
}

int mg_http_get_request_len(const unsigned char *buf, size_t buf_len) {
  size_t i;
  for (i = 0; i < buf_len; i++) {
    if (!isok(buf[i])) return -1;
    if ((i > 0 && buf[i] == '\n' && buf[i - 1] == '\n') ||
        (i > 3 && buf[i] == '\n' && buf[i - 1] == '\r' && buf[i - 2] == '\n'))
      return (int) i + 1;
  }
  return 0;
}

static const char *skip(const char *s, const char *e, const char *d,
                        struct mg_str *v) {
  v->ptr = s;
  while (s < e && *s != '\n' && strchr(d, *s) == NULL) s++;
  v->len = (size_t) (s - v->ptr);
  while (s < e && strchr(d, *s) != NULL) s++;
  return s;
}

struct mg_str *mg_http_get_header(struct mg_http_message *h, const char *name) {
  size_t i, n = strlen(name), max = sizeof(h->headers) / sizeof(h->headers[0]);
  for (i = 0; i < max && h->headers[i].name.len > 0; i++) {
    struct mg_str *k = &h->headers[i].name, *v = &h->headers[i].value;
    if (n == k->len && mg_ncasecmp(k->ptr, name, n) == 0) return v;
  }
  return NULL;
}

static void mg_http_parse_headers(const char *s, const char *end,
                                  struct mg_http_header *h, int max_headers) {
  int i;
  for (i = 0; i < max_headers; i++) {
    struct mg_str k, v, tmp;
    const char *he = skip(s, end, "\n", &tmp);
    s = skip(s, he, ": \r\n", &k);
    s = skip(s, he, "\r\n", &v);
    if (k.len == tmp.len) continue;
    while (v.len > 0 && v.ptr[v.len - 1] == ' ') v.len--;  // Trim spaces
    if (k.len == 0) break;
    // MG_INFO(("--HH [%.*s] [%.*s] [%.*s]", (int) tmp.len - 1, tmp.ptr,
    //(int) k.len, k.ptr, (int) v.len, v.ptr));
    h[i].name = k;
    h[i].value = v;
  }
}

int mg_http_parse(const char *s, size_t len, struct mg_http_message *hm) {
  int is_response, req_len = mg_http_get_request_len((unsigned char *) s, len);
  const char *end = s == NULL ? NULL : s + req_len, *qs;  // Cannot add to NULL
  struct mg_str *cl;

  memset(hm, 0, sizeof(*hm));
  if (req_len <= 0) return req_len;

  hm->message.ptr = hm->head.ptr = s;
  hm->body.ptr = end;
  hm->head.len = (size_t) req_len;
  hm->chunk.ptr = end;
  hm->message.len = hm->body.len = (size_t) ~0;  // Set body length to infinite

  // Parse request line
  s = skip(s, end, " ", &hm->method);
  s = skip(s, end, " ", &hm->uri);
  s = skip(s, end, "\r\n", &hm->proto);

  // Sanity check. Allow protocol/reason to be empty
  if (hm->method.len == 0 || hm->uri.len == 0) return -1;

  // If URI contains '?' character, setup query string
  if ((qs = (const char *) memchr(hm->uri.ptr, '?', hm->uri.len)) != NULL) {
    hm->query.ptr = qs + 1;
    hm->query.len = (size_t) (&hm->uri.ptr[hm->uri.len] - (qs + 1));
    hm->uri.len = (size_t) (qs - hm->uri.ptr);
  }

  mg_http_parse_headers(s, end, hm->headers,
                        sizeof(hm->headers) / sizeof(hm->headers[0]));
  if ((cl = mg_http_get_header(hm, "Content-Length")) != NULL) {
    hm->body.len = (size_t) mg_to64(*cl);
    hm->message.len = (size_t) req_len + hm->body.len;
  }

  // mg_http_parse() is used to parse both HTTP requests and HTTP
  // responses. If HTTP response does not have Content-Length set, then
  // body is read until socket is closed, i.e. body.len is infinite (~0).
  //
  // For HTTP requests though, according to
  // http://tools.ietf.org/html/rfc7231#section-8.1.3,
  // only POST and PUT methods have defined body semantics.
  // Therefore, if Content-Length is not specified and methods are
  // not one of PUT or POST, set body length to 0.
  //
  // So, if it is HTTP request, and Content-Length is not set,
  // and method is not (PUT or POST) then reset body length to zero.
  is_response = mg_ncasecmp(hm->method.ptr, "HTTP/", 5) == 0;
  if (hm->body.len == (size_t) ~0 && !is_response &&
      mg_vcasecmp(&hm->method, "PUT") != 0 &&
      mg_vcasecmp(&hm->method, "POST") != 0) {
    hm->body.len = 0;
    hm->message.len = (size_t) req_len;
  }

  // The 204 (No content) responses also have 0 body length
  if (hm->body.len == (size_t) ~0 && is_response &&
      mg_vcasecmp(&hm->uri, "204") == 0) {
    hm->body.len = 0;
    hm->message.len = (size_t) req_len;
  }

  return req_len;
}

static void mg_http_vprintf_chunk(struct mg_connection *c, const char *fmt,
                                  va_list *ap) {
  size_t len = c->send.len;
  mg_send(c, "        \r\n", 10);
  mg_vxprintf(mg_pfn_iobuf, &c->send, fmt, ap);
  if (c->send.len >= len + 10) {
    mg_snprintf((char *) c->send.buf + len, 9, "%08lx", c->send.len - len - 10);
    c->send.buf[len + 8] = '\r';
    if (c->send.len == len + 10) c->is_resp = 0;  // Last chunk, reset marker
  }
  mg_send(c, "\r\n", 2);
}

void mg_http_printf_chunk(struct mg_connection *c, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  mg_http_vprintf_chunk(c, fmt, &ap);
  va_end(ap);
}

void mg_http_write_chunk(struct mg_connection *c, const char *buf, size_t len) {
  mg_printf(c, "%lx\r\n", (unsigned long) len);
  mg_send(c, buf, len);
  mg_send(c, "\r\n", 2);
  if (len == 0) c->is_resp = 0;
}

// clang-format off
static const char *mg_http_status_code_str(int status_code) {
  switch (status_code) {
    case 100: return "Continue";
    case 201: return "Created";
    case 202: return "Accepted";
    case 204: return "No Content";
    case 206: return "Partial Content";
    case 301: return "Moved Permanently";
    case 302: return "Found";
    case 304: return "Not Modified";
    case 400: return "Bad Request";
    case 401: return "Unauthorized";
    case 403: return "Forbidden";
    case 404: return "Not Found";
    case 418: return "I'm a teapot";
    case 500: return "Internal Server Error";
    case 501: return "Not Implemented";
    default: return "OK";
  }
}
// clang-format on

void mg_http_reply(struct mg_connection *c, int code, const char *headers,
                   const char *fmt, ...) {
  va_list ap;
  size_t len;
  mg_printf(c, "HTTP/1.1 %d %s\r\n%sContent-Length:           \r\n\r\n", code,
            mg_http_status_code_str(code), headers == NULL ? "" : headers);
  len = c->send.len;
  va_start(ap, fmt);
  mg_vxprintf(mg_pfn_iobuf, &c->send, fmt, &ap);
  va_end(ap);
  if (c->send.len > 15) {
    mg_snprintf((char *) &c->send.buf[len - 14], 11, "%010lu",
                (unsigned long) (c->send.len - len));
    c->is_resp = 0;
    c->send.buf[len - 4] = '\r';  // Change ending 0 to space
  }
  c->is_resp = 0;
}

static void http_cb(struct mg_connection *, int, void *, void *);
static void restore_http_cb(struct mg_connection *c) {
  mg_fs_close((struct mg_fd *) c->pfn_data);
  c->pfn_data = NULL;
  c->pfn = http_cb;
  c->is_resp = 0;
}

char *mg_http_etag(char *buf, size_t len, size_t size, time_t mtime);
char *mg_http_etag(char *buf, size_t len, size_t size, time_t mtime) {
  mg_snprintf(buf, len, "\"%lld.%lld\"", (int64_t) mtime, (int64_t) size);
  return buf;
}

static void static_cb(struct mg_connection *c, int ev, void *ev_data,
                      void *fn_data) {
  if (ev == MG_EV_WRITE || ev == MG_EV_POLL) {
    struct mg_fd *fd = (struct mg_fd *) fn_data;
    // Read to send IO buffer directly, avoid extra on-stack buffer
    size_t n, max = MG_IO_SIZE, space;
    size_t *cl = (size_t *) &c->label[(sizeof(c->label) - sizeof(size_t)) /
                                      sizeof(size_t) * sizeof(size_t)];
    if (c->send.size < max) mg_iobuf_resize(&c->send, max);
    if (c->send.len >= c->send.size) return;  // Rate limit
    if ((space = c->send.size - c->send.len) > *cl) space = *cl;
    n = fd->fs->rd(fd->fd, c->send.buf + c->send.len, space);
    c->send.len += n;
    *cl -= n;
    if (n == 0) restore_http_cb(c);
  } else if (ev == MG_EV_CLOSE) {
    restore_http_cb(c);
  }
  (void) ev_data;
}

// Known mime types. Keep it outside guess_content_type() function, since
// some environments don't like it defined there.
// clang-format off
static struct mg_str s_known_types[] = {
    MG_C_STR("html"), MG_C_STR("text/html; charset=utf-8"),
    MG_C_STR("htm"), MG_C_STR("text/html; charset=utf-8"),
    MG_C_STR("css"), MG_C_STR("text/css; charset=utf-8"),
    MG_C_STR("js"), MG_C_STR("text/javascript; charset=utf-8"),
    MG_C_STR("gif"), MG_C_STR("image/gif"),
    MG_C_STR("png"), MG_C_STR("image/png"),
    MG_C_STR("jpg"), MG_C_STR("image/jpeg"),
    MG_C_STR("jpeg"), MG_C_STR("image/jpeg"),
    MG_C_STR("woff"), MG_C_STR("font/woff"),
    MG_C_STR("ttf"), MG_C_STR("font/ttf"),
    MG_C_STR("svg"), MG_C_STR("image/svg+xml"),
    MG_C_STR("txt"), MG_C_STR("text/plain; charset=utf-8"),
    MG_C_STR("avi"), MG_C_STR("video/x-msvideo"),
    MG_C_STR("csv"), MG_C_STR("text/csv"),
    MG_C_STR("doc"), MG_C_STR("application/msword"),
    MG_C_STR("exe"), MG_C_STR("application/octet-stream"),
    MG_C_STR("gz"), MG_C_STR("application/gzip"),
    MG_C_STR("ico"), MG_C_STR("image/x-icon"),
    MG_C_STR("json"), MG_C_STR("application/json"),
    MG_C_STR("mov"), MG_C_STR("video/quicktime"),
    MG_C_STR("mp3"), MG_C_STR("audio/mpeg"),
    MG_C_STR("mp4"), MG_C_STR("video/mp4"),
    MG_C_STR("mpeg"), MG_C_STR("video/mpeg"),
    MG_C_STR("pdf"), MG_C_STR("application/pdf"),
    MG_C_STR("shtml"), MG_C_STR("text/html; charset=utf-8"),
    MG_C_STR("tgz"), MG_C_STR("application/tar-gz"),
    MG_C_STR("wav"), MG_C_STR("audio/wav"),
    MG_C_STR("webp"), MG_C_STR("image/webp"),
    MG_C_STR("zip"), MG_C_STR("application/zip"),
    MG_C_STR("3gp"), MG_C_STR("video/3gpp"),
    {0, 0},
};
// clang-format on

static struct mg_str guess_content_type(struct mg_str path, const char *extra) {
  struct mg_str k, v, s = mg_str(extra);
  size_t i = 0;

  // Shrink path to its extension only
  while (i < path.len && path.ptr[path.len - i - 1] != '.') i++;
  path.ptr += path.len - i;
  path.len = i;

  // Process user-provided mime type overrides, if any
  while (mg_commalist(&s, &k, &v)) {
    if (mg_strcmp(path, k) == 0) return v;
  }

  // Process built-in mime types
  for (i = 0; s_known_types[i].ptr != NULL; i += 2) {
    if (mg_strcmp(path, s_known_types[i]) == 0) return s_known_types[i + 1];
  }

  return mg_str("text/plain; charset=utf-8");
}

static int getrange(struct mg_str *s, int64_t *a, int64_t *b) {
  size_t i, numparsed = 0;
  // MG_INFO(("%.*s", (int) s->len, s->ptr));
  for (i = 0; i + 6 < s->len; i++) {
    if (memcmp(&s->ptr[i], "bytes=", 6) == 0) {
      struct mg_str p = mg_str_n(s->ptr + i + 6, s->len - i - 6);
      if (p.len > 0 && p.ptr[0] >= '0' && p.ptr[0] <= '9') numparsed++;
      *a = mg_to64(p);
      // MG_INFO(("PPP [%.*s] %d", (int) p.len, p.ptr, numparsed));
      while (p.len && p.ptr[0] >= '0' && p.ptr[0] <= '9') p.ptr++, p.len--;
      if (p.len && p.ptr[0] == '-') p.ptr++, p.len--;
      *b = mg_to64(p);
      if (p.len > 0 && p.ptr[0] >= '0' && p.ptr[0] <= '9') numparsed++;
      // MG_INFO(("PPP [%.*s] %d", (int) p.len, p.ptr, numparsed));
      break;
    }
  }
  return (int) numparsed;
}

void mg_http_serve_file(struct mg_connection *c, struct mg_http_message *hm,
                        const char *path,
                        const struct mg_http_serve_opts *opts) {
  char etag[64], tmp[MG_PATH_MAX];
  struct mg_fs *fs = opts->fs == NULL ? &mg_fs_posix : opts->fs;
  struct mg_fd *fd = path == NULL ? NULL : mg_fs_open(fs, path, MG_FS_READ);
  size_t size = 0;
  time_t mtime = 0;
  struct mg_str *inm = NULL;
  struct mg_str mime = guess_content_type(mg_str(path), opts->mime_types);
  bool gzip = false;

  // If file does not exist, we try to open file PATH.gz - and if such
  // pre-compressed .gz file exists, serve it with the Content-Encoding: gzip
  // Note - we ignore Accept-Encoding, cause we don't have a choice
  if (fd == NULL) {
    MG_DEBUG(("NULL [%s]", path));
    mg_snprintf(tmp, sizeof(tmp), "%s.gz", path);
    if ((fd = mg_fs_open(fs, tmp, MG_FS_READ)) != NULL) {
      gzip = true;
      path = tmp;
    } else if (opts->page404 != NULL) {
      // No precompressed file, serve 404
      fd = mg_fs_open(fs, opts->page404, MG_FS_READ);
      mime = guess_content_type(mg_str(path), opts->mime_types);
      path = opts->page404;
    }
  }

  if (fd == NULL || fs->st(path, &size, &mtime) == 0) {
    mg_http_reply(c, 404, opts->extra_headers, "Not found\n");
    mg_fs_close(fd);
    // NOTE: mg_http_etag() call should go first!
  } else if (mg_http_etag(etag, sizeof(etag), size, mtime) != NULL &&
             (inm = mg_http_get_header(hm, "If-None-Match")) != NULL &&
             mg_vcasecmp(inm, etag) == 0) {
    mg_fs_close(fd);
    mg_http_reply(c, 304, opts->extra_headers, "");
  } else {
    int n, status = 200;
    char range[100];
    int64_t r1 = 0, r2 = 0, cl = (int64_t) size;

    // Handle Range header
    struct mg_str *rh = mg_http_get_header(hm, "Range");
    range[0] = '\0';
    if (rh != NULL && (n = getrange(rh, &r1, &r2)) > 0 && r1 >= 0 && r2 >= 0) {
      // If range is specified like "400-", set second limit to content len
      if (n == 1) r2 = cl - 1;
      if (r1 > r2 || r2 >= cl) {
        status = 416;
        cl = 0;
        mg_snprintf(range, sizeof(range), "Content-Range: bytes */%lld\r\n",
                    (int64_t) size);
      } else {
        status = 206;
        cl = r2 - r1 + 1;
        mg_snprintf(range, sizeof(range),
                    "Content-Range: bytes %lld-%lld/%lld\r\n", r1, r1 + cl - 1,
                    (int64_t) size);
        fs->sk(fd->fd, (size_t) r1);
      }
    }
    mg_printf(c,
              "HTTP/1.1 %d %s\r\n"
              "Content-Type: %.*s\r\n"
              "Etag: %s\r\n"
              "Content-Length: %llu\r\n"
              "%s%s%s\r\n",
              status, mg_http_status_code_str(status), (int) mime.len, mime.ptr,
              etag, cl, gzip ? "Content-Encoding: gzip\r\n" : "", range,
              opts->extra_headers ? opts->extra_headers : "");
    if (mg_vcasecmp(&hm->method, "HEAD") == 0) {
      c->is_draining = 1;
      c->is_resp = 0;
      mg_fs_close(fd);
    } else {
      // Track to-be-sent content length at the end of c->label, aligned
      size_t *clp = (size_t *) &c->label[(sizeof(c->label) - sizeof(size_t)) /
                                         sizeof(size_t) * sizeof(size_t)];
      c->pfn = static_cb;
      c->pfn_data = fd;
      *clp = (size_t) cl;
    }
  }
}

struct printdirentrydata {
  struct mg_connection *c;
  struct mg_http_message *hm;
  const struct mg_http_serve_opts *opts;
  const char *dir;
};

static void printdirentry(const char *name, void *userdata) {
  struct printdirentrydata *d = (struct printdirentrydata *) userdata;
  struct mg_fs *fs = d->opts->fs == NULL ? &mg_fs_posix : d->opts->fs;
  size_t size = 0;
  time_t t = 0;
  char path[MG_PATH_MAX], sz[40], mod[40];
  int flags, n = 0;

  // MG_DEBUG(("[%s] [%s]", d->dir, name));
  if (mg_snprintf(path, sizeof(path), "%s%c%s", d->dir, '/', name) >
      sizeof(path)) {
    MG_ERROR(("%s truncated", name));
  } else if ((flags = fs->st(path, &size, &t)) == 0) {
    MG_ERROR(("%lu stat(%s): %d", d->c->id, path, errno));
  } else {
    const char *slash = flags & MG_FS_DIR ? "/" : "";
    if (flags & MG_FS_DIR) {
      mg_snprintf(sz, sizeof(sz), "%s", "[DIR]");
    } else {
      mg_snprintf(sz, sizeof(sz), "%lld", (uint64_t) size);
    }
#if defined(MG_HTTP_DIRLIST_TIME)
    char time_str[30];
    struct tm *time_info = localtime(&t);
    strftime(time_str, sizeof time_str, "%Y/%m/%d %H:%M:%S", time_info);
    mg_snprintf(mod, sizeof(mod), "%s", time_str);
#elif defined(MG_HTTP_DIRLIST_TIME_UTC)
    char time_str[30];
    struct tm *time_info = gmtime(&t);
    strftime(time_str, sizeof time_str, "%Y/%m/%d %H:%M:%S", time_info);
    mg_snprintf(mod, sizeof(mod), "%s", time_str);
#else
    mg_snprintf(mod, sizeof(mod), "%ld", (unsigned long) t);
#endif
    n = (int) mg_url_encode(name, strlen(name), path, sizeof(path));
    mg_printf(d->c,
              "  <tr><td><a href=\"%.*s%s\">%s%s</a></td>"
              "<td name=%lu>%s</td><td name=%lld>%s</td></tr>\n",
              n, path, slash, name, slash, (unsigned long) t, mod,
              flags & MG_FS_DIR ? (int64_t) -1 : (int64_t) size, sz);
  }
}

static void listdir(struct mg_connection *c, struct mg_http_message *hm,
                    const struct mg_http_serve_opts *opts, char *dir) {
  const char *sort_js_code =
      "<script>function srt(tb, sc, so, d) {"
      "var tr = Array.prototype.slice.call(tb.rows, 0),"
      "tr = tr.sort(function (a, b) { var c1 = a.cells[sc], c2 = b.cells[sc],"
      "n1 = c1.getAttribute('name'), n2 = c2.getAttribute('name'), "
      "t1 = a.cells[2].getAttribute('name'), "
      "t2 = b.cells[2].getAttribute('name'); "
      "return so * (t1 < 0 && t2 >= 0 ? -1 : t2 < 0 && t1 >= 0 ? 1 : "
      "n1 ? parseInt(n2) - parseInt(n1) : "
      "c1.textContent.trim().localeCompare(c2.textContent.trim())); });";
  const char *sort_js_code2 =
      "for (var i = 0; i < tr.length; i++) tb.appendChild(tr[i]); "
      "if (!d) window.location.hash = ('sc=' + sc + '&so=' + so); "
      "};"
      "window.onload = function() {"
      "var tb = document.getElementById('tb');"
      "var m = /sc=([012]).so=(1|-1)/.exec(window.location.hash) || [0, 2, 1];"
      "var sc = m[1], so = m[2]; document.onclick = function(ev) { "
      "var c = ev.target.rel; if (c) {if (c == sc) so *= -1; srt(tb, c, so); "
      "sc = c; ev.preventDefault();}};"
      "srt(tb, sc, so, true);"
      "}"
      "</script>";
  struct mg_fs *fs = opts->fs == NULL ? &mg_fs_posix : opts->fs;
  struct printdirentrydata d = {c, hm, opts, dir};
  char tmp[10], buf[MG_PATH_MAX];
  size_t off, n;
  int len = mg_url_decode(hm->uri.ptr, hm->uri.len, buf, sizeof(buf), 0);
  struct mg_str uri = len > 0 ? mg_str_n(buf, (size_t) len) : hm->uri;

  mg_printf(c,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html; charset=utf-8\r\n"
            "%s"
            "Content-Length:         \r\n\r\n",
            opts->extra_headers == NULL ? "" : opts->extra_headers);
  off = c->send.len;  // Start of body
  mg_printf(c,
            "<!DOCTYPE html><html><head><title>Index of %.*s</title>%s%s"
            "<style>th,td {text-align: left; padding-right: 1em; "
            "font-family: monospace; }</style></head>"
            "<body><h1>Index of %.*s</h1><table cellpadding=\"0\"><thead>"
            "<tr><th><a href=\"#\" rel=\"0\">Name</a></th><th>"
            "<a href=\"#\" rel=\"1\">Modified</a></th>"
            "<th><a href=\"#\" rel=\"2\">Size</a></th></tr>"
            "<tr><td colspan=\"3\"><hr></td></tr>"
            "</thead>"
            "<tbody id=\"tb\">\n",
            (int) uri.len, uri.ptr, sort_js_code, sort_js_code2, (int) uri.len,
            uri.ptr);
  mg_printf(c, "%s",
            "  <tr><td><a href=\"..\">..</a></td>"
            "<td name=-1></td><td name=-1>[DIR]</td></tr>\n");

  fs->ls(dir, printdirentry, &d);
  mg_printf(c,
            "</tbody><tfoot><tr><td colspan=\"3\"><hr></td></tr></tfoot>"
            "</table><address>Mongoose v.%s</address></body></html>\n",
            MG_VERSION);
  n = mg_snprintf(tmp, sizeof(tmp), "%lu", (unsigned long) (c->send.len - off));
  if (n > sizeof(tmp)) n = 0;
  memcpy(c->send.buf + off - 12, tmp, n);  // Set content length
  c->is_resp = 0;                          // Mark response end
}

// Resolve requested file into `path` and return its fs->st() result
static int uri_to_path2(struct mg_connection *c, struct mg_http_message *hm,
                        struct mg_fs *fs, struct mg_str url, struct mg_str dir,
                        char *path, size_t path_size) {
  int flags, tmp;
  // Append URI to the root_dir, and sanitize it
  size_t n = mg_snprintf(path, path_size, "%.*s", (int) dir.len, dir.ptr);
  if (n > path_size) n = path_size;
  path[path_size - 1] = '\0';
  if (n + 2 < path_size) path[n++] = '/', path[n] = '\0';
  mg_url_decode(hm->uri.ptr + url.len, hm->uri.len - url.len, path + n,
                path_size - n, 0);
  path[path_size - 1] = '\0';  // Double-check
  mg_remove_double_dots(path);
  n = strlen(path);
  while (n > 1 && path[n - 1] == '/') path[--n] = 0;  // Trim trailing slashes
  flags = mg_vcmp(&hm->uri, "/") == 0 ? MG_FS_DIR : fs->st(path, NULL, NULL);
  MG_VERBOSE(("%lu %.*s -> %s %d", c->id, (int) hm->uri.len, hm->uri.ptr, path,
              flags));
  if (flags == 0) {
    // Do nothing - let's caller decide
  } else if ((flags & MG_FS_DIR) && hm->uri.len > 0 &&
             hm->uri.ptr[hm->uri.len - 1] != '/') {
    mg_printf(c,
              "HTTP/1.1 301 Moved\r\n"
              "Location: %.*s/\r\n"
              "Content-Length: 0\r\n"
              "\r\n",
              (int) hm->uri.len, hm->uri.ptr);
    flags = -1;
  } else if (flags & MG_FS_DIR) {
    if (((mg_snprintf(path + n, path_size - n, "/" MG_HTTP_INDEX) > 0 &&
          (tmp = fs->st(path, NULL, NULL)) != 0) ||
         (mg_snprintf(path + n, path_size - n, "/index.shtml") > 0 &&
          (tmp = fs->st(path, NULL, NULL)) != 0))) {
      flags = tmp;
    } else if ((mg_snprintf(path + n, path_size - n, "/" MG_HTTP_INDEX ".gz") >
                    0 &&
                (tmp = fs->st(path, NULL, NULL)) !=
                    0)) {  // check for gzipped index
      flags = tmp;
      path[n + 1 + strlen(MG_HTTP_INDEX)] =
          '\0';  // Remove appended .gz in index file name
    } else {
      path[n] = '\0';  // Remove appended index file name
    }
  }
  return flags;
}

static int uri_to_path(struct mg_connection *c, struct mg_http_message *hm,
                       const struct mg_http_serve_opts *opts, char *path,
                       size_t path_size) {
  struct mg_fs *fs = opts->fs == NULL ? &mg_fs_posix : opts->fs;
  struct mg_str k, v, s = mg_str(opts->root_dir), u = {0, 0}, p = {0, 0};
  while (mg_commalist(&s, &k, &v)) {
    if (v.len == 0) v = k, k = mg_str("/");
    if (hm->uri.len < k.len) continue;
    if (mg_strcmp(k, mg_str_n(hm->uri.ptr, k.len)) != 0) continue;
    u = k, p = v;
  }
  return uri_to_path2(c, hm, fs, u, p, path, path_size);
}

void mg_http_serve_dir(struct mg_connection *c, struct mg_http_message *hm,
                       const struct mg_http_serve_opts *opts) {
  char path[MG_PATH_MAX];
  const char *sp = opts->ssi_pattern;
  int flags = uri_to_path(c, hm, opts, path, sizeof(path));
  if (flags < 0) {
    // Do nothing: the response has already been sent by uri_to_path()
  } else if (flags & MG_FS_DIR) {
    listdir(c, hm, opts, path);
  } else if (flags && sp != NULL &&
             mg_globmatch(sp, strlen(sp), path, strlen(path))) {
    mg_http_serve_ssi(c, opts->root_dir, path);
  } else {
    mg_http_serve_file(c, hm, path, opts);
  }
}

static bool mg_is_url_safe(int c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
         (c >= 'A' && c <= 'Z') || c == '.' || c == '_' || c == '-' || c == '~';
}

size_t mg_url_encode(const char *s, size_t sl, char *buf, size_t len) {
  size_t i, n = 0;
  for (i = 0; i < sl; i++) {
    int c = *(unsigned char *) &s[i];
    if (n + 4 >= len) return 0;
    if (mg_is_url_safe(c)) {
      buf[n++] = s[i];
    } else {
      buf[n++] = '%';
      mg_hex(&s[i], 1, &buf[n]);
      n += 2;
    }
  }
  return n;
}

void mg_http_creds(struct mg_http_message *hm, char *user, size_t userlen,
                   char *pass, size_t passlen) {
  struct mg_str *v = mg_http_get_header(hm, "Authorization");
  user[0] = pass[0] = '\0';
  if (v != NULL && v->len > 6 && memcmp(v->ptr, "Basic ", 6) == 0) {
    char buf[256];
    int n = mg_base64_decode(v->ptr + 6, (int) v->len - 6, buf);
    const char *p = (const char *) memchr(buf, ':', n > 0 ? (size_t) n : 0);
    if (p != NULL) {
      mg_snprintf(user, userlen, "%.*s", (int) (p - buf), buf);
      mg_snprintf(pass, passlen, "%.*s", n - (int) (p - buf) - 1, p + 1);
    }
  } else if (v != NULL && v->len > 7 && memcmp(v->ptr, "Bearer ", 7) == 0) {
    mg_snprintf(pass, passlen, "%.*s", (int) v->len - 7, v->ptr + 7);
  } else if ((v = mg_http_get_header(hm, "Cookie")) != NULL) {
    struct mg_str t = mg_http_get_header_var(*v, mg_str_n("access_token", 12));
    if (t.len > 0) mg_snprintf(pass, passlen, "%.*s", (int) t.len, t.ptr);
  } else {
    mg_http_get_var(&hm->query, "access_token", pass, passlen);
  }
}

static struct mg_str stripquotes(struct mg_str s) {
  return s.len > 1 && s.ptr[0] == '"' && s.ptr[s.len - 1] == '"'
             ? mg_str_n(s.ptr + 1, s.len - 2)
             : s;
}

struct mg_str mg_http_get_header_var(struct mg_str s, struct mg_str v) {
  size_t i;
  for (i = 0; v.len > 0 && i + v.len + 2 < s.len; i++) {
    if (s.ptr[i + v.len] == '=' && memcmp(&s.ptr[i], v.ptr, v.len) == 0) {
      const char *p = &s.ptr[i + v.len + 1], *b = p, *x = &s.ptr[s.len];
      int q = p < x && *p == '"' ? 1 : 0;
      while (p < x &&
             (q ? p == b || *p != '"' : *p != ';' && *p != ' ' && *p != ','))
        p++;
      // MG_INFO(("[%.*s] [%.*s] [%.*s]", (int) s.len, s.ptr, (int) v.len,
      // v.ptr, (int) (p - b), b));
      return stripquotes(mg_str_n(b, (size_t) (p - b + q)));
    }
  }
  return mg_str_n(NULL, 0);
}

bool mg_http_match_uri(const struct mg_http_message *hm, const char *glob) {
  return mg_match(hm->uri, mg_str(glob), NULL);
}

long mg_http_upload(struct mg_connection *c, struct mg_http_message *hm,
                    struct mg_fs *fs, const char *path, size_t max_size) {
  char buf[20] = "0";
  long res = 0, offset;
  mg_http_get_var(&hm->query, "offset", buf, sizeof(buf));
  offset = strtol(buf, NULL, 0);
  if (hm->body.len == 0) {
    mg_http_reply(c, 200, "", "%ld", res);  // Nothing to write
  } else {
    struct mg_fd *fd;
    size_t current_size = 0;
    MG_DEBUG(("%s -> %d bytes @ %ld", path, (int) hm->body.len, offset));
    if (offset == 0) fs->rm(path);  // If offset if 0, truncate file
    fs->st(path, &current_size, NULL);
    if (offset < 0) {
      mg_http_reply(c, 400, "", "offset required");
      res = -1;
    } else if (offset > 0 && current_size != (size_t) offset) {
      mg_http_reply(c, 400, "", "%s: offset mismatch", path);
      res = -2;
    } else if ((size_t) offset + hm->body.len > max_size) {
      mg_http_reply(c, 400, "", "%s: over max size of %lu", path,
                    (unsigned long) max_size);
      res = -3;
    } else if ((fd = mg_fs_open(fs, path, MG_FS_WRITE)) == NULL) {
      mg_http_reply(c, 400, "", "open(%s): %d", path, errno);
      res = -4;
    } else {
      res = offset + (long) fs->wr(fd->fd, hm->body.ptr, hm->body.len);
      mg_fs_close(fd);
      mg_http_reply(c, 200, "", "%ld", res);
    }
  }
  return res;
}

int mg_http_status(const struct mg_http_message *hm) {
  return atoi(hm->uri.ptr);
}

// If a server sends data to the client using chunked encoding, Mongoose strips
// off the chunking prefix (hex length and \r\n) and suffix (\r\n), appends the
// stripped data to the body, and fires the MG_EV_HTTP_CHUNK event.  When zero
// chunk is received, we fire MG_EV_HTTP_MSG, and the body already has all
// chunking prefixes/suffixes stripped.
//
// If a server sends data without chunked encoding, we also fire a series of
// MG_EV_HTTP_CHUNK events for every received piece of data, and then we fire
// MG_EV_HTTP_MSG event in the end.
//
// We track total processed length in the c->pfn_data, which is a void *
// pointer: we store a size_t value there.
static bool getchunk(struct mg_str s, size_t *prefixlen, size_t *datalen) {
  size_t i = 0, n;
  while (i < s.len && s.ptr[i] != '\r' && s.ptr[i] != '\n') i++;
  n = mg_unhexn(s.ptr, i);
  // MG_INFO(("%d %d", (int) (i + n + 4), (int) s.len));
  if (s.len < i + n + 4) return false;  // Chunk not yet fully buffered
  if (s.ptr[i] != '\r' || s.ptr[i + 1] != '\n') return false;
  if (s.ptr[i + n + 2] != '\r' || s.ptr[i + n + 3] != '\n') return false;
  *prefixlen = i + 2;
  *datalen = n;
  return true;
}

static bool mg_is_chunked(struct mg_http_message *hm) {
  const char *needle = "chunked";
  struct mg_str *te = mg_http_get_header(hm, "Transfer-Encoding");
  return te != NULL && mg_vcasecmp(te, needle) == 0;
}

void mg_http_delete_chunk(struct mg_connection *c, struct mg_http_message *hm) {
  size_t ofs = (size_t) (hm->chunk.ptr - (char *) c->recv.buf);
  mg_iobuf_del(&c->recv, ofs, hm->chunk.len);
  c->pfn_data = (void *) ((size_t) c->pfn_data | MG_DMARK);
}

static void deliver_chunked_chunks(struct mg_connection *c, size_t hlen,
                                   struct mg_http_message *hm, bool *next) {
  //  |  ... headers ... | HEXNUM\r\n ..data.. \r\n | ......
  //  +------------------+--------------------------+----
  //  |      hlen        |           chunk1         | ......
  char *buf = (char *) &c->recv.buf[hlen], *p = buf;
  size_t len = c->recv.len - hlen;
  size_t processed = ((size_t) c->pfn_data) & ~MG_DMARK;
  size_t mark, pl, dl, del = 0, ofs = 0;
  bool last = false;
  if (processed <= len) len -= processed, buf += processed;
  while (!last && getchunk(mg_str_n(buf + ofs, len - ofs), &pl, &dl)) {
    size_t saved = c->recv.len;
    memmove(p + processed, buf + ofs + pl, dl);
    // MG_INFO(("P2 [%.*s]", (int) (processed + dl), p));
    hm->chunk = mg_str_n(p + processed, dl);
    mg_call(c, MG_EV_HTTP_CHUNK, hm);
    ofs += pl + dl + 2, del += pl + 2;  // 2 is for \r\n suffix
    processed += dl;
    if (c->recv.len != saved) processed -= dl, buf -= dl;
    // mg_hexdump(c->recv.buf, hlen + processed);
    last = (dl == 0);
  }
  mg_iobuf_del(&c->recv, hlen + processed, del);
  mark = ((size_t) c->pfn_data) & MG_DMARK;
  c->pfn_data = (void *) (processed | mark);
  if (last) {
    hm->body.len = processed;
    hm->message.len = hlen + processed;
    c->pfn_data = NULL;
    if (mark) mg_iobuf_del(&c->recv, 0, hlen), *next = true;
    // MG_INFO(("LAST, mark: %lx", mark));
    // mg_hexdump(c->recv.buf, c->recv.len);
  }
}

static void deliver_normal_chunks(struct mg_connection *c, size_t hlen,
                                  struct mg_http_message *hm, bool *next) {
  size_t left, processed = ((size_t) c->pfn_data) & ~MG_DMARK;
  size_t deleted = ((size_t) c->pfn_data) & MG_DMARK;
  hm->chunk = mg_str_n((char *) &c->recv.buf[hlen], c->recv.len - hlen);
  if (processed <= hm->chunk.len && !deleted) {
    hm->chunk.len -= processed;
    hm->chunk.ptr += processed;
  }
  left = hm->body.len < processed ? 0 : hm->body.len - processed;
  if (hm->chunk.len > left) hm->chunk.len = left;
  if (hm->chunk.len > 0) mg_call(c, MG_EV_HTTP_CHUNK, hm);
  processed += hm->chunk.len;
  deleted = ((size_t) c->pfn_data) & MG_DMARK;  // Re-evaluate after user call
  if (processed >= hm->body.len) {              // Last, 0-len chunk
    hm->chunk.len = 0;                          // Reset length
    mg_call(c, MG_EV_HTTP_CHUNK, hm);           // Call user handler
    c->pfn_data = NULL;                         // Reset processed counter
    if (processed && deleted) mg_iobuf_del(&c->recv, 0, hlen), *next = true;
  } else {
    c->pfn_data = (void *) (processed | deleted);  // if it is set
  }
}

static void http_cb(struct mg_connection *c, int ev, void *evd, void *fnd) {
  if (ev == MG_EV_READ || ev == MG_EV_CLOSE) {
    struct mg_http_message hm;
    // mg_hexdump(c->recv.buf, c->recv.len);
    while (c->recv.buf != NULL && c->recv.len > 0) {
      bool next = false;
      int hlen = mg_http_parse((char *) c->recv.buf, c->recv.len, &hm);
      if (hlen < 0) {
        mg_error(c, "HTTP parse:\n%.*s", (int) c->recv.len, c->recv.buf);
        break;
      }
      if (c->is_resp) break;           // Response is still generated
      if (hlen == 0) break;            // Request is not buffered yet
      if (ev == MG_EV_CLOSE) {         // If client did not set Content-Length
        hm.message.len = c->recv.len;  // and closes now, deliver a MSG
        hm.body.len = hm.message.len - (size_t) (hm.body.ptr - hm.message.ptr);
      }
      if (mg_is_chunked(&hm)) {
        deliver_chunked_chunks(c, (size_t) hlen, &hm, &next);
      } else {
        deliver_normal_chunks(c, (size_t) hlen, &hm, &next);
      }
      if (next) continue;  // Chunks & request were deleted
      //  Chunk events are delivered. If we have full body, deliver MSG
      if (c->recv.len < hm.message.len) break;
      if (c->is_accepted) c->is_resp = 1;  // Start generating response
      mg_call(c, MG_EV_HTTP_MSG, &hm);     // User handler can clear is_resp
      mg_iobuf_del(&c->recv, 0, hm.message.len);
    }
  }
  (void) evd, (void) fnd;
}

static void mg_hfn(struct mg_connection *c, int ev, void *ev_data, void *fnd) {
  if (ev == MG_EV_HTTP_MSG) {
    struct mg_http_message *hm = (struct mg_http_message *) ev_data;
    if (mg_http_match_uri(hm, "/quit")) {
      mg_http_reply(c, 200, "", "ok\n");
      c->is_draining = 1;
      c->label[0] = 'X';
    } else if (mg_http_match_uri(hm, "/debug")) {
      int level = (int) mg_json_get_long(hm->body, "$.level", MG_LL_DEBUG);
      mg_log_set(level);
      mg_http_reply(c, 200, "", "Debug level set to %d\n", level);
    } else {
      mg_http_reply(c, 200, "", "hi\n");
    }
  } else if (ev == MG_EV_CLOSE) {
    if (c->label[0] == 'X') *(bool *) fnd = true;
  }
}

void mg_hello(const char *url) {
  struct mg_mgr mgr;
  bool done = false;
  mg_mgr_init(&mgr);
  if (mg_http_listen(&mgr, url, mg_hfn, &done) == NULL) done = true;
  while (done == false) mg_mgr_poll(&mgr, 100);
  mg_mgr_free(&mgr);
}

struct mg_connection *mg_http_connect(struct mg_mgr *mgr, const char *url,
                                      mg_event_handler_t fn, void *fn_data) {
  struct mg_connection *c = mg_connect(mgr, url, fn, fn_data);
  if (c != NULL) c->pfn = http_cb;
  return c;
}

struct mg_connection *mg_http_listen(struct mg_mgr *mgr, const char *url,
                                     mg_event_handler_t fn, void *fn_data) {
  struct mg_connection *c = mg_listen(mgr, url, fn, fn_data);
  if (c != NULL) c->pfn = http_cb;
  return c;
}

#ifdef MG_ENABLE_LINES
#line 1 "src/iobuf.c"
#endif




// Not using memset for zeroing memory, cause it can be dropped by compiler
// See https://github.com/cesanta/mongoose/pull/1265
static void zeromem(volatile unsigned char *buf, size_t len) {
  if (buf != NULL) {
    while (len--) *buf++ = 0;
  }
}

static size_t roundup(size_t size, size_t align) {
  return align == 0 ? size : (size + align - 1) / align * align;
}

int mg_iobuf_resize(struct mg_iobuf *io, size_t new_size) {
  int ok = 1;
  new_size = roundup(new_size, io->align);
  if (new_size == 0) {
    zeromem(io->buf, io->size);
    free(io->buf);
    io->buf = NULL;
    io->len = io->size = 0;
  } else if (new_size != io->size) {
    // NOTE(lsm): do not use realloc here. Use calloc/free only, to ease the
    // porting to some obscure platforms like FreeRTOS
    void *p = calloc(1, new_size);
    if (p != NULL) {
      size_t len = new_size < io->len ? new_size : io->len;
      if (len > 0 && io->buf != NULL) memmove(p, io->buf, len);
      zeromem(io->buf, io->size);
      free(io->buf);
      io->buf = (unsigned char *) p;
      io->size = new_size;
    } else {
      ok = 0;
      MG_ERROR(("%lld->%lld", (uint64_t) io->size, (uint64_t) new_size));
    }
  }
  return ok;
}

int mg_iobuf_init(struct mg_iobuf *io, size_t size, size_t align) {
  io->buf = NULL;
  io->align = align;
  io->size = io->len = 0;
  return mg_iobuf_resize(io, size);
}

size_t mg_iobuf_add(struct mg_iobuf *io, size_t ofs, const void *buf,
                    size_t len) {
  size_t new_size = roundup(io->len + len, io->align);
  mg_iobuf_resize(io, new_size);      // Attempt to resize
  if (new_size != io->size) len = 0;  // Resize failure, append nothing
  if (ofs < io->len) memmove(io->buf + ofs + len, io->buf + ofs, io->len - ofs);
  if (buf != NULL) memmove(io->buf + ofs, buf, len);
  if (ofs > io->len) io->len += ofs - io->len;
  io->len += len;
  return len;
}

size_t mg_iobuf_del(struct mg_iobuf *io, size_t ofs, size_t len) {
  if (ofs > io->len) ofs = io->len;
  if (ofs + len > io->len) len = io->len - ofs;
  if (io->buf) memmove(io->buf + ofs, io->buf + ofs + len, io->len - ofs - len);
  if (io->buf) zeromem(io->buf + io->len - len, len);
  io->len -= len;
  return len;
}

void mg_iobuf_free(struct mg_iobuf *io) {
  mg_iobuf_resize(io, 0);
}

#ifdef MG_ENABLE_LINES
#line 1 "src/json.c"
#endif




static const char *escapeseq(int esc) {
  return esc ? "\b\f\n\r\t\\\"" : "bfnrt\\\"";
}

static char json_esc(int c, int esc) {
  const char *p, *esc1 = escapeseq(esc), *esc2 = escapeseq(!esc);
  for (p = esc1; *p != '\0'; p++) {
    if (*p == c) return esc2[p - esc1];
  }
  return 0;
}

static int mg_pass_string(const char *s, int len) {
  int i;
  for (i = 0; i < len; i++) {
    if (s[i] == '\\' && i + 1 < len && json_esc(s[i + 1], 1)) {
      i++;
    } else if (s[i] == '\0') {
      return MG_JSON_INVALID;
    } else if (s[i] == '"') {
      return i;
    }
  }
  return MG_JSON_INVALID;
}

static double mg_atod(const char *p, int len, int *numlen) {
  double d = 0.0;
  int i = 0, sign = 1;

  // Sign
  if (i < len && *p == '-') {
    sign = -1, i++;
  } else if (i < len && *p == '+') {
    i++;
  }

  // Decimal
  for (; i < len && p[i] >= '0' && p[i] <= '9'; i++) {
    d *= 10.0;
    d += p[i] - '0';
  }
  d *= sign;

  // Fractional
  if (i < len && p[i] == '.') {
    double frac = 0.0, base = 0.1;
    i++;
    for (; i < len && p[i] >= '0' && p[i] <= '9'; i++) {
      frac += base * (p[i] - '0');
      base /= 10.0;
    }
    d += frac * sign;
  }

  // Exponential
  if (i < len && (p[i] == 'e' || p[i] == 'E')) {
    int j, exp = 0, minus = 0;
    i++;
    if (i < len && p[i] == '-') minus = 1, i++;
    if (i < len && p[i] == '+') i++;
    while (i < len && p[i] >= '0' && p[i] <= '9' && exp < 308)
      exp = exp * 10 + (p[i++] - '0');
    if (minus) exp = -exp;
    for (j = 0; j < exp; j++) d *= 10.0;
    for (j = 0; j < -exp; j++) d /= 10.0;
  }

  if (numlen != NULL) *numlen = i;
  return d;
}

int mg_json_get(struct mg_str json, const char *path, int *toklen) {
  const char *s = json.ptr;
  int len = (int) json.len;
  enum { S_VALUE, S_KEY, S_COLON, S_COMMA_OR_EOO } expecting = S_VALUE;
  unsigned char nesting[MG_JSON_MAX_DEPTH];
  int i = 0;             // Current offset in `s`
  int j = 0;             // Offset in `s` we're looking for (return value)
  int depth = 0;         // Current depth (nesting level)
  int ed = 0;            // Expected depth
  int pos = 1;           // Current position in `path`
  int ci = -1, ei = -1;  // Current and expected index in array

  if (toklen) *toklen = 0;
  if (path[0] != '$') return MG_JSON_INVALID;

#define MG_CHECKRET(x)                                  \
  do {                                                  \
    if (depth == ed && path[pos] == '\0' && ci == ei) { \
      if (toklen) *toklen = i - j + 1;                  \
      return j;                                         \
    }                                                   \
  } while (0)

// In the ascii table, the distance between `[` and `]` is 2.
// Ditto for `{` and `}`. Hence +2 in the code below.
#define MG_EOO(x)                                            \
  do {                                                       \
    if (depth == ed && ci != ei) return MG_JSON_NOT_FOUND;   \
    if (c != nesting[depth - 1] + 2) return MG_JSON_INVALID; \
    depth--;                                                 \
    MG_CHECKRET(x);                                          \
  } while (0)

  for (i = 0; i < len; i++) {
    unsigned char c = ((unsigned char *) s)[i];
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r') continue;
    switch (expecting) {
      case S_VALUE:
        // p("V %s [%.*s] %d %d %d %d\n", path, pos, path, depth, ed, ci, ei);
        if (depth == ed) j = i;
        if (c == '{') {
          if (depth >= (int) sizeof(nesting)) return MG_JSON_TOO_DEEP;
          if (depth == ed && path[pos] == '.' && ci == ei) {
            // If we start the object, reset array indices
            ed++, pos++, ci = ei = -1;
          }
          nesting[depth++] = c;
          expecting = S_KEY;
          break;
        } else if (c == '[') {
          if (depth >= (int) sizeof(nesting)) return MG_JSON_TOO_DEEP;
          if (depth == ed && path[pos] == '[' && ei == ci) {
            ed++, pos++, ci = 0;
            for (ei = 0; path[pos] != ']' && path[pos] != '\0'; pos++) {
              ei *= 10;
              ei += path[pos] - '0';
            }
            if (path[pos] != 0) pos++;
          }
          nesting[depth++] = c;
          break;
        } else if (c == ']' && depth > 0) {  // Empty array
          MG_EOO(']');
        } else if (c == 't' && i + 3 < len && memcmp(&s[i], "true", 4) == 0) {
          i += 3;
        } else if (c == 'n' && i + 3 < len && memcmp(&s[i], "null", 4) == 0) {
          i += 3;
        } else if (c == 'f' && i + 4 < len && memcmp(&s[i], "false", 5) == 0) {
          i += 4;
        } else if (c == '-' || ((c >= '0' && c <= '9'))) {
          int numlen = 0;
          mg_atod(&s[i], len - i, &numlen);
          i += numlen - 1;
        } else if (c == '"') {
          int n = mg_pass_string(&s[i + 1], len - i - 1);
          if (n < 0) return n;
          i += n + 1;
        } else {
          return MG_JSON_INVALID;
        }
        MG_CHECKRET('V');
        if (depth == ed && ei >= 0) ci++;
        expecting = S_COMMA_OR_EOO;
        break;

      case S_KEY:
        if (c == '"') {
          int n = mg_pass_string(&s[i + 1], len - i - 1);
          if (n < 0) return n;
          if (i + 1 + n >= len) return MG_JSON_NOT_FOUND;
          if (depth < ed) return MG_JSON_NOT_FOUND;
          if (depth == ed && path[pos - 1] != '.') return MG_JSON_NOT_FOUND;
          // printf("K %s [%.*s] [%.*s] %d %d %d\n", path, pos, path, n,
          //  &s[i + 1], n, depth, ed);
          // NOTE(cpq): in the check sequence below is important.
          // strncmp() must go first: it fails fast if the remaining length of
          // the path is smaller than `n`.
          if (depth == ed && path[pos - 1] == '.' &&
              strncmp(&s[i + 1], &path[pos], (size_t) n) == 0 &&
              (path[pos + n] == '\0' || path[pos + n] == '.' ||
               path[pos + n] == '[')) {
            pos += n;
          }
          i += n + 1;
          expecting = S_COLON;
        } else if (c == '}') {  // Empty object
          MG_EOO('}');
          expecting = S_COMMA_OR_EOO;
        } else {
          return MG_JSON_INVALID;
        }
        break;

      case S_COLON:
        if (c == ':') {
          expecting = S_VALUE;
        } else {
          return MG_JSON_INVALID;
        }
        break;

      case S_COMMA_OR_EOO:
        if (depth <= 0) {
          return MG_JSON_INVALID;
        } else if (c == ',') {
          expecting = (nesting[depth - 1] == '{') ? S_KEY : S_VALUE;
        } else if (c == ']' || c == '}') {
          MG_EOO('O');
          if (depth == ed && ei >= 0) ci++;
        } else {
          return MG_JSON_INVALID;
        }
        break;
    }
  }
  return MG_JSON_NOT_FOUND;
}

bool mg_json_get_num(struct mg_str json, const char *path, double *v) {
  int n, toklen, found = 0;
  if ((n = mg_json_get(json, path, &toklen)) >= 0 &&
      (json.ptr[n] == '-' || (json.ptr[n] >= '0' && json.ptr[n] <= '9'))) {
    if (v != NULL) *v = mg_atod(json.ptr + n, toklen, NULL);
    found = 1;
  }
  return found;
}

bool mg_json_get_bool(struct mg_str json, const char *path, bool *v) {
  int found = 0, off = mg_json_get(json, path, NULL);
  if (off >= 0 && (json.ptr[off] == 't' || json.ptr[off] == 'f')) {
    if (v != NULL) *v = json.ptr[off] == 't';
    found = 1;
  }
  return found;
}

static bool json_unescape(const char *s, size_t len, char *to, size_t n) {
  size_t i, j;
  for (i = 0, j = 0; i < len && j < n; i++, j++) {
    if (s[i] == '\\' && i + 5 < len && s[i + 1] == 'u') {
      //  \uXXXX escape. We could process a simple one-byte chars
      // \u00xx from the ASCII range. More complex chars would require
      // dragging in a UTF8 library, which is too much for us
      if (s[i + 2] != '0' || s[i + 3] != '0') return false;  // Give up
      ((unsigned char *) to)[j] = (unsigned char) mg_unhexn(s + i + 4, 2);

      i += 5;
    } else if (s[i] == '\\' && i + 1 < len) {
      char c = json_esc(s[i + 1], 0);
      if (c == 0) return false;
      to[j] = c;
      i++;
    } else {
      to[j] = s[i];
    }
  }
  if (j >= n) return false;
  if (n > 0) to[j] = '\0';
  return true;
}

char *mg_json_get_str(struct mg_str json, const char *path) {
  char *result = NULL;
  int len = 0, off = mg_json_get(json, path, &len);
  if (off >= 0 && len > 1 && json.ptr[off] == '"') {
    if ((result = (char *) calloc(1, (size_t) len)) != NULL &&
        !json_unescape(json.ptr + off + 1, (size_t) (len - 2), result,
                       (size_t) len)) {
      free(result);
      result = NULL;
    }
  }
  return result;
}

char *mg_json_get_b64(struct mg_str json, const char *path, int *slen) {
  char *result = NULL;
  int len = 0, off = mg_json_get(json, path, &len);
  if (off >= 0 && json.ptr[off] == '"' && len > 1 &&
      (result = (char *) calloc(1, (size_t) len)) != NULL) {
    int k = mg_base64_decode(json.ptr + off + 1, len - 2, result);
    if (slen != NULL) *slen = k;
  }
  return result;
}

char *mg_json_get_hex(struct mg_str json, const char *path, int *slen) {
  char *result = NULL;
  int len = 0, off = mg_json_get(json, path, &len);
  if (off >= 0 && json.ptr[off] == '"' && len > 1 &&
      (result = (char *) calloc(1, (size_t) len / 2)) != NULL) {
    mg_unhex(json.ptr + off + 1, (size_t) (len - 2), (uint8_t *) result);
    result[len / 2 - 1] = '\0';
    if (slen != NULL) *slen = len / 2 - 1;
  }
  return result;
}

long mg_json_get_long(struct mg_str json, const char *path, long dflt) {
  double dv;
  long result = dflt;
  if (mg_json_get_num(json, path, &dv)) result = (long) dv;
  return result;
}

#ifdef MG_ENABLE_LINES
#line 1 "src/log.c"
#endif





static void default_logger(char c, void *param) {
  putchar(c);
  (void) c, (void) param;
}

static int s_level = MG_LL_INFO;
static mg_pfn_t s_log_func = default_logger;
static void *s_log_func_param = NULL;

void mg_log_set_fn(mg_pfn_t fn, void *param) {
  s_log_func = fn;
  s_log_func_param = param;
}

static void logc(unsigned char c) {
  s_log_func((char) c, s_log_func_param);
}

static void logs(const char *buf, size_t len) {
  size_t i;
  for (i = 0; i < len; i++) logc(((unsigned char *) buf)[i]);
}

void mg_log_set(int log_level) {
  MG_DEBUG(("Setting log level to %d", log_level));
  s_level = log_level;
}

bool mg_log_prefix(int level, const char *file, int line, const char *fname) {
  if (level <= s_level) {
    const char *p = strrchr(file, '/');
    char buf[41];
    size_t n;
    if (p == NULL) p = strrchr(file, '\\');
    n = mg_snprintf(buf, sizeof(buf), "%llx %d %s:%d:%s", mg_millis(), level,
                    p == NULL ? file : p + 1, line, fname);
    if (n > sizeof(buf) - 2) n = sizeof(buf) - 2;
    while (n < sizeof(buf)) buf[n++] = ' ';
    logs(buf, n - 1);
    return true;
  } else {
    return false;
  }
}

void mg_log(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  mg_vxprintf(s_log_func, s_log_func_param, fmt, &ap);
  va_end(ap);
  logc((unsigned char) '\n');
}

static unsigned char nibble(unsigned c) {
  return (unsigned char) (c < 10 ? c + '0' : c + 'W');
}

#define ISPRINT(x) ((x) >= ' ' && (x) <= '~')
void mg_hexdump(const void *buf, size_t len) {
  const unsigned char *p = (const unsigned char *) buf;
  unsigned char ascii[16], alen = 0;
  size_t i;
  for (i = 0; i < len; i++) {
    if ((i % 16) == 0) {
      // Print buffered ascii chars
      if (i > 0) logs("  ", 2), logs((char *) ascii, 16), logc('\n'), alen = 0;
      // Print hex address, then \t
      logc(nibble((i >> 12) & 15)), logc(nibble((i >> 8) & 15)),
          logc(nibble((i >> 4) & 15)), logc('0'), logs("   ", 3);
    }
    logc(nibble(p[i] >> 4)), logc(nibble(p[i] & 15));  // Two nibbles, e.g. c5
    logc(' ');                                         // Space after hex number
    ascii[alen++] = ISPRINT(p[i]) ? p[i] : '.';        // Add to the ascii buf
  }
  while (alen < 16) logs("   ", 3), ascii[alen++] = ' ';
  logs("  ", 2), logs((char *) ascii, 16), logc('\n');
}

#ifdef MG_ENABLE_LINES
#line 1 "src/md5.c"
#endif



#if defined(MG_ENABLE_MD5) && MG_ENABLE_MD5

static void mg_byte_reverse(unsigned char *buf, unsigned longs) {
  if (MG_BIG_ENDIAN) {
    do {
      uint32_t t = (uint32_t) ((unsigned) buf[3] << 8 | buf[2]) << 16 |
                   ((unsigned) buf[1] << 8 | buf[0]);
      *(uint32_t *) buf = t;
      buf += 4;
    } while (--longs);
  } else {
    (void) buf, (void) longs;  // Little endian. Do nothing
  }
}

#define F1(x, y, z) (z ^ (x & (y ^ z)))
#define F2(x, y, z) F1(z, x, y)
#define F3(x, y, z) (x ^ y ^ z)
#define F4(x, y, z) (y ^ (x | ~z))

#define MD5STEP(f, w, x, y, z, data, s) \
  (w += f(x, y, z) + data, w = w << s | w >> (32 - s), w += x)

/*
 * Start MD5 accumulation.  Set bit count to 0 and buffer to mysterious
 * initialization constants.
 */
void mg_md5_init(mg_md5_ctx *ctx) {
  ctx->buf[0] = 0x67452301;
  ctx->buf[1] = 0xefcdab89;
  ctx->buf[2] = 0x98badcfe;
  ctx->buf[3] = 0x10325476;

  ctx->bits[0] = 0;
  ctx->bits[1] = 0;
}

static void mg_md5_transform(uint32_t buf[4], uint32_t const in[16]) {
  uint32_t a, b, c, d;

  a = buf[0];
  b = buf[1];
  c = buf[2];
  d = buf[3];

  MD5STEP(F1, a, b, c, d, in[0] + 0xd76aa478, 7);
  MD5STEP(F1, d, a, b, c, in[1] + 0xe8c7b756, 12);
  MD5STEP(F1, c, d, a, b, in[2] + 0x242070db, 17);
  MD5STEP(F1, b, c, d, a, in[3] + 0xc1bdceee, 22);
  MD5STEP(F1, a, b, c, d, in[4] + 0xf57c0faf, 7);
  MD5STEP(F1, d, a, b, c, in[5] + 0x4787c62a, 12);
  MD5STEP(F1, c, d, a, b, in[6] + 0xa8304613, 17);
  MD5STEP(F1, b, c, d, a, in[7] + 0xfd469501, 22);
  MD5STEP(F1, a, b, c, d, in[8] + 0x698098d8, 7);
  MD5STEP(F1, d, a, b, c, in[9] + 0x8b44f7af, 12);
  MD5STEP(F1, c, d, a, b, in[10] + 0xffff5bb1, 17);
  MD5STEP(F1, b, c, d, a, in[11] + 0x895cd7be, 22);
  MD5STEP(F1, a, b, c, d, in[12] + 0x6b901122, 7);
  MD5STEP(F1, d, a, b, c, in[13] + 0xfd987193, 12);
  MD5STEP(F1, c, d, a, b, in[14] + 0xa679438e, 17);
  MD5STEP(F1, b, c, d, a, in[15] + 0x49b40821, 22);

  MD5STEP(F2, a, b, c, d, in[1] + 0xf61e2562, 5);
  MD5STEP(F2, d, a, b, c, in[6] + 0xc040b340, 9);
  MD5STEP(F2, c, d, a, b, in[11] + 0x265e5a51, 14);
  MD5STEP(F2, b, c, d, a, in[0] + 0xe9b6c7aa, 20);
  MD5STEP(F2, a, b, c, d, in[5] + 0xd62f105d, 5);
  MD5STEP(F2, d, a, b, c, in[10] + 0x02441453, 9);
  MD5STEP(F2, c, d, a, b, in[15] + 0xd8a1e681, 14);
  MD5STEP(F2, b, c, d, a, in[4] + 0xe7d3fbc8, 20);
  MD5STEP(F2, a, b, c, d, in[9] + 0x21e1cde6, 5);
  MD5STEP(F2, d, a, b, c, in[14] + 0xc33707d6, 9);
  MD5STEP(F2, c, d, a, b, in[3] + 0xf4d50d87, 14);
  MD5STEP(F2, b, c, d, a, in[8] + 0x455a14ed, 20);
  MD5STEP(F2, a, b, c, d, in[13] + 0xa9e3e905, 5);
  MD5STEP(F2, d, a, b, c, in[2] + 0xfcefa3f8, 9);
  MD5STEP(F2, c, d, a, b, in[7] + 0x676f02d9, 14);
  MD5STEP(F2, b, c, d, a, in[12] + 0x8d2a4c8a, 20);

  MD5STEP(F3, a, b, c, d, in[5] + 0xfffa3942, 4);
  MD5STEP(F3, d, a, b, c, in[8] + 0x8771f681, 11);
  MD5STEP(F3, c, d, a, b, in[11] + 0x6d9d6122, 16);
  MD5STEP(F3, b, c, d, a, in[14] + 0xfde5380c, 23);
  MD5STEP(F3, a, b, c, d, in[1] + 0xa4beea44, 4);
  MD5STEP(F3, d, a, b, c, in[4] + 0x4bdecfa9, 11);
  MD5STEP(F3, c, d, a, b, in[7] + 0xf6bb4b60, 16);
  MD5STEP(F3, b, c, d, a, in[10] + 0xbebfbc70, 23);
  MD5STEP(F3, a, b, c, d, in[13] + 0x289b7ec6, 4);
  MD5STEP(F3, d, a, b, c, in[0] + 0xeaa127fa, 11);
  MD5STEP(F3, c, d, a, b, in[3] + 0xd4ef3085, 16);
  MD5STEP(F3, b, c, d, a, in[6] + 0x04881d05, 23);
  MD5STEP(F3, a, b, c, d, in[9] + 0xd9d4d039, 4);
  MD5STEP(F3, d, a, b, c, in[12] + 0xe6db99e5, 11);
  MD5STEP(F3, c, d, a, b, in[15] + 0x1fa27cf8, 16);
  MD5STEP(F3, b, c, d, a, in[2] + 0xc4ac5665, 23);

  MD5STEP(F4, a, b, c, d, in[0] + 0xf4292244, 6);
  MD5STEP(F4, d, a, b, c, in[7] + 0x432aff97, 10);
  MD5STEP(F4, c, d, a, b, in[14] + 0xab9423a7, 15);
  MD5STEP(F4, b, c, d, a, in[5] + 0xfc93a039, 21);
  MD5STEP(F4, a, b, c, d, in[12] + 0x655b59c3, 6);
  MD5STEP(F4, d, a, b, c, in[3] + 0x8f0ccc92, 10);
  MD5STEP(F4, c, d, a, b, in[10] + 0xffeff47d, 15);
  MD5STEP(F4, b, c, d, a, in[1] + 0x85845dd1, 21);
  MD5STEP(F4, a, b, c, d, in[8] + 0x6fa87e4f, 6);
  MD5STEP(F4, d, a, b, c, in[15] + 0xfe2ce6e0, 10);
  MD5STEP(F4, c, d, a, b, in[6] + 0xa3014314, 15);
  MD5STEP(F4, b, c, d, a, in[13] + 0x4e0811a1, 21);
  MD5STEP(F4, a, b, c, d, in[4] + 0xf7537e82, 6);
  MD5STEP(F4, d, a, b, c, in[11] + 0xbd3af235, 10);
  MD5STEP(F4, c, d, a, b, in[2] + 0x2ad7d2bb, 15);
  MD5STEP(F4, b, c, d, a, in[9] + 0xeb86d391, 21);

  buf[0] += a;
  buf[1] += b;
  buf[2] += c;
  buf[3] += d;
}

void mg_md5_update(mg_md5_ctx *ctx, const unsigned char *buf, size_t len) {
  uint32_t t;

  t = ctx->bits[0];
  if ((ctx->bits[0] = t + ((uint32_t) len << 3)) < t) ctx->bits[1]++;
  ctx->bits[1] += (uint32_t) len >> 29;

  t = (t >> 3) & 0x3f;

  if (t) {
    unsigned char *p = (unsigned char *) ctx->in + t;

    t = 64 - t;
    if (len < t) {
      memcpy(p, buf, len);
      return;
    }
    memcpy(p, buf, t);
    mg_byte_reverse(ctx->in, 16);
    mg_md5_transform(ctx->buf, (uint32_t *) ctx->in);
    buf += t;
    len -= t;
  }

  while (len >= 64) {
    memcpy(ctx->in, buf, 64);
    mg_byte_reverse(ctx->in, 16);
    mg_md5_transform(ctx->buf, (uint32_t *) ctx->in);
    buf += 64;
    len -= 64;
  }

  memcpy(ctx->in, buf, len);
}

void mg_md5_final(mg_md5_ctx *ctx, unsigned char digest[16]) {
  unsigned count;
  unsigned char *p;
  uint32_t *a;

  count = (ctx->bits[0] >> 3) & 0x3F;

  p = ctx->in + count;
  *p++ = 0x80;
  count = 64 - 1 - count;
  if (count < 8) {
    memset(p, 0, count);
    mg_byte_reverse(ctx->in, 16);
    mg_md5_transform(ctx->buf, (uint32_t *) ctx->in);
    memset(ctx->in, 0, 56);
  } else {
    memset(p, 0, count - 8);
  }
  mg_byte_reverse(ctx->in, 14);

  a = (uint32_t *) ctx->in;
  a[14] = ctx->bits[0];
  a[15] = ctx->bits[1];

  mg_md5_transform(ctx->buf, (uint32_t *) ctx->in);
  mg_byte_reverse((unsigned char *) ctx->buf, 4);
  memcpy(digest, ctx->buf, 16);
  memset((char *) ctx, 0, sizeof(*ctx));
}
#endif

#ifdef MG_ENABLE_LINES
#line 1 "src/mqtt.c"
#endif








#define MQTT_CLEAN_SESSION 0x02
#define MQTT_HAS_WILL 0x04
#define MQTT_WILL_RETAIN 0x20
#define MQTT_HAS_PASSWORD 0x40
#define MQTT_HAS_USER_NAME 0x80

void mg_mqtt_send_header(struct mg_connection *c, uint8_t cmd, uint8_t flags,
                         uint32_t len) {
  uint8_t buf[1 + sizeof(len)], *vlen = &buf[1];
  buf[0] = (uint8_t) ((cmd << 4) | flags);
  do {
    *vlen = len % 0x80;
    len /= 0x80;
    if (len > 0) *vlen |= 0x80;
    vlen++;
  } while (len > 0 && vlen < &buf[sizeof(buf)]);
  mg_send(c, buf, (size_t) (vlen - buf));
}

static void mg_send_u16(struct mg_connection *c, uint16_t value) {
  mg_send(c, &value, sizeof(value));
}

void mg_mqtt_login(struct mg_connection *c, const struct mg_mqtt_opts *opts) {
  char rnd[10], client_id[21], zero = 0;
  struct mg_str cid = opts->client_id;
  uint32_t total_len = 7 + 1 + 2 + 2;
  uint8_t hdr[8] = {0, 4, 'M', 'Q', 'T', 'T', opts->version, 0};

  if (cid.len == 0) {
    mg_random(rnd, sizeof(rnd));
    mg_hex(rnd, sizeof(rnd), client_id);
    client_id[sizeof(client_id) - 1] = '\0';
    cid = mg_str(client_id);
  }

  if (hdr[6] == 0) hdr[6] = 4;  // If version is not set, use 4 (3.1.1)
  c->is_mqtt5 = hdr[6] == 5;    // Set version 5 flag
  hdr[7] = (uint8_t) ((opts->will_qos & 3) << 3);  // Connection flags
  if (opts->user.len > 0) {
    total_len += 2 + (uint32_t) opts->user.len;
    hdr[7] |= MQTT_HAS_USER_NAME;
  }
  if (opts->pass.len > 0) {
    total_len += 2 + (uint32_t) opts->pass.len;
    hdr[7] |= MQTT_HAS_PASSWORD;
  }
  if (opts->will_topic.len > 0 && opts->will_message.len > 0) {
    total_len +=
        4 + (uint32_t) opts->will_topic.len + (uint32_t) opts->will_message.len;
    hdr[7] |= MQTT_HAS_WILL;
  }
  if (opts->clean || cid.len == 0) hdr[7] |= MQTT_CLEAN_SESSION;
  if (opts->will_retain) hdr[7] |= MQTT_WILL_RETAIN;
  total_len += (uint32_t) cid.len;
  if (c->is_mqtt5) total_len += 1U + (hdr[7] & MQTT_HAS_WILL ? 1U : 0);

  mg_mqtt_send_header(c, MQTT_CMD_CONNECT, 0, total_len);
  mg_send(c, hdr, sizeof(hdr));
  // keepalive == 0 means "do not disconnect us!"
  mg_send_u16(c, mg_htons((uint16_t) opts->keepalive));

  if (c->is_mqtt5) mg_send(c, &zero, sizeof(zero));  // V5 properties
  mg_send_u16(c, mg_htons((uint16_t) cid.len));
  mg_send(c, cid.ptr, cid.len);

  if (hdr[7] & MQTT_HAS_WILL) {
    if (c->is_mqtt5) mg_send(c, &zero, sizeof(zero));  // will props
    mg_send_u16(c, mg_htons((uint16_t) opts->will_topic.len));
    mg_send(c, opts->will_topic.ptr, opts->will_topic.len);
    mg_send_u16(c, mg_htons((uint16_t) opts->will_message.len));
    mg_send(c, opts->will_message.ptr, opts->will_message.len);
  }
  if (opts->user.len > 0) {
    mg_send_u16(c, mg_htons((uint16_t) opts->user.len));
    mg_send(c, opts->user.ptr, opts->user.len);
  }
  if (opts->pass.len > 0) {
    mg_send_u16(c, mg_htons((uint16_t) opts->pass.len));
    mg_send(c, opts->pass.ptr, opts->pass.len);
  }
}

void mg_mqtt_pub(struct mg_connection *c, struct mg_str topic,
                 struct mg_str data, int qos, bool retain) {
  uint8_t flags = (uint8_t) (((qos & 3) << 1) | (retain ? 1 : 0)), zero = 0;
  uint32_t len = 2 + (uint32_t) topic.len + (uint32_t) data.len;
  MG_DEBUG(("%lu [%.*s] -> [%.*s]", c->id, (int) topic.len, (char *) topic.ptr,
            (int) data.len, (char *) data.ptr));
  if (qos > 0) len += 2;
  if (c->is_mqtt5) len++;
  mg_mqtt_send_header(c, MQTT_CMD_PUBLISH, flags, len);
  mg_send_u16(c, mg_htons((uint16_t) topic.len));
  mg_send(c, topic.ptr, topic.len);
  if (qos > 0) {
    if (++c->mgr->mqtt_id == 0) ++c->mgr->mqtt_id;
    mg_send_u16(c, mg_htons(c->mgr->mqtt_id));
  }
  if (c->is_mqtt5) mg_send(c, &zero, sizeof(zero));
  mg_send(c, data.ptr, data.len);
}

void mg_mqtt_sub(struct mg_connection *c, struct mg_str topic, int qos) {
  uint8_t qos_ = qos & 3, zero = 0;
  uint32_t len = 2 + (uint32_t) topic.len + 2 + 1 + (c->is_mqtt5 ? 1 : 0);
  mg_mqtt_send_header(c, MQTT_CMD_SUBSCRIBE, 2, len);
  if (++c->mgr->mqtt_id == 0) ++c->mgr->mqtt_id;
  mg_send_u16(c, mg_htons(c->mgr->mqtt_id));
  if (c->is_mqtt5) mg_send(c, &zero, sizeof(zero));
  mg_send_u16(c, mg_htons((uint16_t) topic.len));
  mg_send(c, topic.ptr, topic.len);
  mg_send(c, &qos_, sizeof(qos_));
}

int mg_mqtt_parse(const uint8_t *buf, size_t len, uint8_t version,
                  struct mg_mqtt_message *m) {
  uint8_t lc = 0, *p, *end;
  uint32_t n = 0, len_len = 0;

  memset(m, 0, sizeof(*m));
  m->dgram.ptr = (char *) buf;
  if (len < 2) return MQTT_INCOMPLETE;
  m->cmd = (uint8_t) (buf[0] >> 4);
  m->qos = (buf[0] >> 1) & 3;

  n = len_len = 0;
  p = (uint8_t *) buf + 1;
  while ((size_t) (p - buf) < len) {
    lc = *((uint8_t *) p++);
    n += (uint32_t) ((lc & 0x7f) << 7 * len_len);
    len_len++;
    if (!(lc & 0x80)) break;
    if (len_len >= 4) return MQTT_MALFORMED;
  }
  end = p + n;
  if ((lc & 0x80) || (end > buf + len)) return MQTT_INCOMPLETE;
  m->dgram.len = (size_t) (end - buf);

  switch (m->cmd) {
    case MQTT_CMD_CONNACK:
      if (end - p < 2) return MQTT_MALFORMED;
      m->ack = p[1];
      break;
    case MQTT_CMD_PUBACK:
    case MQTT_CMD_PUBREC:
    case MQTT_CMD_PUBREL:
    case MQTT_CMD_PUBCOMP:
    case MQTT_CMD_SUBSCRIBE:
    case MQTT_CMD_SUBACK:
    case MQTT_CMD_UNSUBSCRIBE:
    case MQTT_CMD_UNSUBACK:
      if (p + 2 > end) return MQTT_MALFORMED;
      m->id = (uint16_t) ((((uint16_t) p[0]) << 8) | p[1]);
      p += 2;
      break;
    case MQTT_CMD_PUBLISH: {
      if (p + 2 > end) return MQTT_MALFORMED;
      m->topic.len = (uint16_t) ((((uint16_t) p[0]) << 8) | p[1]);
      m->topic.ptr = (char *) p + 2;
      p += 2 + m->topic.len;
      if (p > end) return MQTT_MALFORMED;
      if (m->qos > 0) {
        if (p + 2 > end) return MQTT_MALFORMED;
        m->id = (uint16_t) ((((uint16_t) p[0]) << 8) | p[1]);
        p += 2;
      }
      if (p >= end) return MQTT_MALFORMED;
      if (version == 5) p += 1 + p[0];  // Skip options
      if (p > end) return MQTT_MALFORMED;
      m->data.ptr = (char *) p;
      m->data.len = (size_t) (end - p);
      break;
    }
    default:
      break;
  }
  return MQTT_OK;
}

static void mqtt_cb(struct mg_connection *c, int ev, void *ev_data,
                    void *fn_data) {
  if (ev == MG_EV_READ) {
    for (;;) {
      uint8_t version = c->is_mqtt5 ? 5 : 4;
      struct mg_mqtt_message mm;
      int rc = mg_mqtt_parse(c->recv.buf, c->recv.len, version, &mm);
      if (rc == MQTT_MALFORMED) {
        MG_ERROR(("%lu MQTT malformed message", c->id));
        c->is_closing = 1;
        break;
      } else if (rc == MQTT_OK) {
        MG_VERBOSE(("%lu MQTT CMD %d len %d [%.*s]", c->id, mm.cmd,
                    (int) mm.dgram.len, (int) mm.data.len, mm.data.ptr));
        switch (mm.cmd) {
          case MQTT_CMD_CONNACK:
            mg_call(c, MG_EV_MQTT_OPEN, &mm.ack);
            if (mm.ack == 0) {
              MG_DEBUG(("%lu Connected", c->id));
            } else {
              MG_ERROR(("%lu MQTT auth failed, code %d", c->id, mm.ack));
              c->is_closing = 1;
            }
            break;
          case MQTT_CMD_PUBLISH: {
            MG_DEBUG(("%lu [%.*s] -> [%.*s]", c->id, (int) mm.topic.len,
                      mm.topic.ptr, (int) mm.data.len, mm.data.ptr));
            if (mm.qos > 0) {
              uint16_t id = mg_htons(mm.id);
              mg_mqtt_send_header(c, MQTT_CMD_PUBACK, 0, sizeof(id));
              mg_send(c, &id, sizeof(id));
            }
            mg_call(c, MG_EV_MQTT_MSG, &mm);
            break;
          }
        }
        mg_call(c, MG_EV_MQTT_CMD, &mm);
        mg_iobuf_del(&c->recv, 0, mm.dgram.len);
      } else {
        break;
      }
    }
  }
  (void) ev_data;
  (void) fn_data;
}

void mg_mqtt_ping(struct mg_connection *nc) {
  mg_mqtt_send_header(nc, MQTT_CMD_PINGREQ, 0, 0);
}

void mg_mqtt_pong(struct mg_connection *nc) {
  mg_mqtt_send_header(nc, MQTT_CMD_PINGRESP, 0, 0);
}

void mg_mqtt_disconnect(struct mg_connection *nc) {
  mg_mqtt_send_header(nc, MQTT_CMD_DISCONNECT, 0, 0);
}

struct mg_connection *mg_mqtt_connect(struct mg_mgr *mgr, const char *url,
                                      const struct mg_mqtt_opts *opts,
                                      mg_event_handler_t fn, void *fn_data) {
  struct mg_connection *c = mg_connect(mgr, url, fn, fn_data);
  if (c != NULL) {
    struct mg_mqtt_opts empty;
    memset(&empty, 0, sizeof(empty));
    mg_mqtt_login(c, opts == NULL ? &empty : opts);
    c->pfn = mqtt_cb;
  }
  return c;
}

struct mg_connection *mg_mqtt_listen(struct mg_mgr *mgr, const char *url,
                                     mg_event_handler_t fn, void *fn_data) {
  struct mg_connection *c = mg_listen(mgr, url, fn, fn_data);
  if (c != NULL) c->pfn = mqtt_cb, c->pfn_data = mgr;
  return c;
}

#ifdef MG_ENABLE_LINES
#line 1 "src/net.c"
#endif







size_t mg_vprintf(struct mg_connection *c, const char *fmt, va_list *ap) {
  size_t old = c->send.len;
  mg_vxprintf(mg_pfn_iobuf, &c->send, fmt, ap);
  return c->send.len - old;
}

size_t mg_printf(struct mg_connection *c, const char *fmt, ...) {
  size_t len = 0;
  va_list ap;
  va_start(ap, fmt);
  len = mg_vprintf(c, fmt, &ap);
  va_end(ap);
  return len;
}

char *mg_straddr(struct mg_addr *a, char *buf, size_t len) {
  char tmp[30];
  const char *fmt = a->is_ip6 ? "[%s]:%d" : "%s:%d";
  mg_ntoa(a, tmp, sizeof(tmp));
  mg_snprintf(buf, len, fmt, tmp, (int) mg_ntohs(a->port));
  return buf;
}

char *mg_ntoa(const struct mg_addr *addr, char *buf, size_t len) {
  if (addr->is_ip6) {
    uint16_t *p = (uint16_t *) addr->ip6;
    mg_snprintf(buf, len, "%x:%x:%x:%x:%x:%x:%x:%x", mg_htons(p[0]),
                mg_htons(p[1]), mg_htons(p[2]), mg_htons(p[3]), mg_htons(p[4]),
                mg_htons(p[5]), mg_htons(p[6]), mg_htons(p[7]));
  } else {
    uint8_t p[4];
    memcpy(p, &addr->ip, sizeof(p));
    mg_snprintf(buf, len, "%d.%d.%d.%d", (int) p[0], (int) p[1], (int) p[2],
                (int) p[3]);
  }
  return buf;
}

static bool mg_atonl(struct mg_str str, struct mg_addr *addr) {
  if (mg_vcasecmp(&str, "localhost") != 0) return false;
  addr->ip = mg_htonl(0x7f000001);
  addr->is_ip6 = false;
  return true;
}

static bool mg_atone(struct mg_str str, struct mg_addr *addr) {
  if (str.len > 0) return false;
  addr->ip = 0;
  addr->is_ip6 = false;
  return true;
}

static bool mg_aton4(struct mg_str str, struct mg_addr *addr) {
  uint8_t data[4] = {0, 0, 0, 0};
  size_t i, num_dots = 0;
  for (i = 0; i < str.len; i++) {
    if (str.ptr[i] >= '0' && str.ptr[i] <= '9') {
      int octet = data[num_dots] * 10 + (str.ptr[i] - '0');
      if (octet > 255) return false;
      data[num_dots] = (uint8_t) octet;
    } else if (str.ptr[i] == '.') {
      if (num_dots >= 3 || i == 0 || str.ptr[i - 1] == '.') return false;
      num_dots++;
    } else {
      return false;
    }
  }
  if (num_dots != 3 || str.ptr[i - 1] == '.') return false;
  memcpy(&addr->ip, data, sizeof(data));
  addr->is_ip6 = false;
  return true;
}

static bool mg_v4mapped(struct mg_str str, struct mg_addr *addr) {
  int i;
  if (str.len < 14) return false;
  if (str.ptr[0] != ':' || str.ptr[1] != ':' || str.ptr[6] != ':') return false;
  for (i = 2; i < 6; i++) {
    if (str.ptr[i] != 'f' && str.ptr[i] != 'F') return false;
  }
  if (!mg_aton4(mg_str_n(&str.ptr[7], str.len - 7), addr)) return false;
  memset(addr->ip6, 0, sizeof(addr->ip6));
  addr->ip6[10] = addr->ip6[11] = 255;
  memcpy(&addr->ip6[12], &addr->ip, 4);
  addr->is_ip6 = true;
  return true;
}

static bool mg_aton6(struct mg_str str, struct mg_addr *addr) {
  size_t i, j = 0, n = 0, dc = 42;
  if (str.len > 2 && str.ptr[0] == '[') str.ptr++, str.len -= 2;
  if (mg_v4mapped(str, addr)) return true;
  for (i = 0; i < str.len; i++) {
    if ((str.ptr[i] >= '0' && str.ptr[i] <= '9') ||
        (str.ptr[i] >= 'a' && str.ptr[i] <= 'f') ||
        (str.ptr[i] >= 'A' && str.ptr[i] <= 'F')) {
      unsigned long val;
      if (i > j + 3) return false;
      // MG_DEBUG(("%zu %zu [%.*s]", i, j, (int) (i - j + 1), &str.ptr[j]));
      val = mg_unhexn(&str.ptr[j], i - j + 1);
      addr->ip6[n] = (uint8_t) ((val >> 8) & 255);
      addr->ip6[n + 1] = (uint8_t) (val & 255);
    } else if (str.ptr[i] == ':') {
      j = i + 1;
      if (i > 0 && str.ptr[i - 1] == ':') {
        dc = n;  // Double colon
        if (i > 1 && str.ptr[i - 2] == ':') return false;
      } else if (i > 0) {
        n += 2;
      }
      if (n > 14) return false;
      addr->ip6[n] = addr->ip6[n + 1] = 0;  // For trailing ::
    } else {
      return false;
    }
  }
  if (n < 14 && dc == 42) return false;
  if (n < 14) {
    memmove(&addr->ip6[dc + (14 - n)], &addr->ip6[dc], n - dc + 2);
    memset(&addr->ip6[dc], 0, 14 - n);
  }
  addr->is_ip6 = true;
  return true;
}

bool mg_aton(struct mg_str str, struct mg_addr *addr) {
  // MG_INFO(("[%.*s]", (int) str.len, str.ptr));
  return mg_atone(str, addr) || mg_atonl(str, addr) || mg_aton4(str, addr) ||
         mg_aton6(str, addr);
}

struct mg_connection *mg_alloc_conn(struct mg_mgr *mgr) {
  struct mg_connection *c =
      (struct mg_connection *) calloc(1, sizeof(*c) + mgr->extraconnsize);
  if (c != NULL) {
    c->mgr = mgr;
    c->send.align = c->recv.align = MG_IO_SIZE;
    c->id = ++mgr->nextid;
  }
  return c;
}

void mg_close_conn(struct mg_connection *c) {
  mg_resolve_cancel(c);  // Close any pending DNS query
  LIST_DELETE(struct mg_connection, &c->mgr->conns, c);
  if (c == c->mgr->dns4.c) c->mgr->dns4.c = NULL;
  if (c == c->mgr->dns6.c) c->mgr->dns6.c = NULL;
  // Order of operations is important. `MG_EV_CLOSE` event must be fired
  // before we deallocate received data, see #1331
  mg_call(c, MG_EV_CLOSE, NULL);
  MG_DEBUG(("%lu %p closed", c->id, c->fd));

  mg_tls_free(c);
  mg_iobuf_free(&c->recv);
  mg_iobuf_free(&c->send);
  memset(c, 0, sizeof(*c));
  free(c);
}

struct mg_connection *mg_connect(struct mg_mgr *mgr, const char *url,
                                 mg_event_handler_t fn, void *fn_data) {
  struct mg_connection *c = NULL;
  if (url == NULL || url[0] == '\0') {
    MG_ERROR(("null url"));
  } else if ((c = mg_alloc_conn(mgr)) == NULL) {
    MG_ERROR(("OOM"));
  } else {
    LIST_ADD_HEAD(struct mg_connection, &mgr->conns, c);
    c->is_udp = (strncmp(url, "udp:", 4) == 0);
    c->fd = (void *) (size_t) -1;  // Set to INVALID_SOCKET
    c->fn = fn;
    c->is_client = true;
    c->fn_data = fn_data;
    MG_DEBUG(("%lu %p %s", c->id, c->fd, url));
    mg_call(c, MG_EV_OPEN, NULL);
    mg_resolve(c, url);
  }
  return c;
}

struct mg_connection *mg_listen(struct mg_mgr *mgr, const char *url,
                                mg_event_handler_t fn, void *fn_data) {
  struct mg_connection *c = NULL;
  if ((c = mg_alloc_conn(mgr)) == NULL) {
    MG_ERROR(("OOM %s", url));
  } else if (!mg_open_listener(c, url)) {
    MG_ERROR(("Failed: %s, errno %d", url, errno));
    free(c);
    c = NULL;
  } else {
    c->is_listening = 1;
    c->is_udp = strncmp(url, "udp:", 4) == 0;
    LIST_ADD_HEAD(struct mg_connection, &mgr->conns, c);
    c->fn = fn;
    c->fn_data = fn_data;
    mg_call(c, MG_EV_OPEN, NULL);
    MG_DEBUG(("%lu %p %s", c->id, c->fd, url));
  }
  return c;
}

struct mg_connection *mg_wrapfd(struct mg_mgr *mgr, int fd,
                                mg_event_handler_t fn, void *fn_data) {
  struct mg_connection *c = mg_alloc_conn(mgr);
  if (c != NULL) {
    c->fd = (void *) (size_t) fd;
    c->fn = fn;
    c->fn_data = fn_data;
    MG_EPOLL_ADD(c);
    mg_call(c, MG_EV_OPEN, NULL);
    LIST_ADD_HEAD(struct mg_connection, &mgr->conns, c);
  }
  return c;
}

struct mg_timer *mg_timer_add(struct mg_mgr *mgr, uint64_t milliseconds,
                              unsigned flags, void (*fn)(void *), void *arg) {
  struct mg_timer *t = (struct mg_timer *) calloc(1, sizeof(*t));
  if (t != NULL) {
    mg_timer_init(&mgr->timers, t, milliseconds, flags, fn, arg);
    t->id = mgr->timerid++;
  }
  return t;
}

void mg_mgr_free(struct mg_mgr *mgr) {
  struct mg_connection *c;
  struct mg_timer *tmp, *t = mgr->timers;
  while (t != NULL) tmp = t->next, free(t), t = tmp;
  mgr->timers = NULL;  // Important. Next call to poll won't touch timers
  for (c = mgr->conns; c != NULL; c = c->next) c->is_closing = 1;
  mg_mgr_poll(mgr, 0);
#if MG_ARCH == MG_ARCH_FREERTOS_TCP
  FreeRTOS_DeleteSocketSet(mgr->ss);
#endif
  MG_DEBUG(("All connections closed"));
#if MG_ENABLE_EPOLL
  if (mgr->epoll_fd >= 0) close(mgr->epoll_fd), mgr->epoll_fd = -1;
#endif
  free(mgr->priv);
}

void mg_mgr_init(struct mg_mgr *mgr) {
  memset(mgr, 0, sizeof(*mgr));
#if MG_ENABLE_EPOLL
  if ((mgr->epoll_fd = epoll_create1(0)) < 0) MG_ERROR(("epoll: %d", errno));
#else
  mgr->epoll_fd = -1;
#endif
#if MG_ARCH == MG_ARCH_WIN32 && MG_ENABLE_WINSOCK
  // clang-format off
  { WSADATA data; WSAStartup(MAKEWORD(2, 2), &data); }
  // clang-format on
#elif MG_ARCH == MG_ARCH_FREERTOS_TCP
  mgr->ss = FreeRTOS_CreateSocketSet();
#elif defined(__unix) || defined(__unix__) || defined(__APPLE__)
  // Ignore SIGPIPE signal, so if client cancels the request, it
  // won't kill the whole process.
  signal(SIGPIPE, SIG_IGN);
#endif
  mgr->dnstimeout = 3000;
  mgr->dns4.url = "udp://8.8.8.8:53";
  mgr->dns6.url = "udp://[2001:4860:4860::8888]:53";
}

#ifdef MG_ENABLE_LINES
#line 1 "src/rpc.c"
#endif


void mg_rpc_add(struct mg_rpc **head, struct mg_str method,
                void (*fn)(struct mg_rpc_req *), void *fn_data) {
  struct mg_rpc *rpc = (struct mg_rpc *) calloc(1, sizeof(*rpc));
  if (rpc != NULL) {
    rpc->method = mg_strdup(method), rpc->fn = fn, rpc->fn_data = fn_data;
    rpc->next = *head, *head = rpc;
  }
}

void mg_rpc_del(struct mg_rpc **head, void (*fn)(struct mg_rpc_req *)) {
  struct mg_rpc *r;
  while ((r = *head) != NULL) {
    if (r->fn == fn || fn == NULL) {
      *head = r->next;
      free((void *) r->method.ptr);
      free(r);
    } else {
      head = &(*head)->next;
    }
  }
}

static void mg_rpc_call(struct mg_rpc_req *r, struct mg_str method) {
  struct mg_rpc *h = r->head == NULL ? NULL : *r->head;
  while (h != NULL && !mg_match(method, h->method, NULL)) h = h->next;
  if (h != NULL) {
    r->rpc = h;
    h->fn(r);
  } else {
    mg_rpc_err(r, -32601, "\"%.*s not found\"", (int) method.len, method.ptr);
  }
}

void mg_rpc_process(struct mg_rpc_req *r) {
  int len, off = mg_json_get(r->frame, "$.method", &len);
  if (off > 0 && r->frame.ptr[off] == '"') {
    struct mg_str method = mg_str_n(&r->frame.ptr[off + 1], (size_t) len - 2);
    mg_rpc_call(r, method);
  } else if ((off = mg_json_get(r->frame, "$.result", &len)) > 0 ||
             (off = mg_json_get(r->frame, "$.error", &len)) > 0) {
    mg_rpc_call(r, mg_str(""));  // JSON response! call "" method handler
  } else {
    mg_rpc_err(r, -32700, "%.*Q", (int) r->frame.len, r->frame.ptr);  // Invalid
  }
}

void mg_rpc_vok(struct mg_rpc_req *r, const char *fmt, va_list *ap) {
  int len, off = mg_json_get(r->frame, "$.id", &len);
  if (off > 0) {
    mg_xprintf(r->pfn, r->pfn_data, "{%Q:%.*s,%Q:", "id", len,
               &r->frame.ptr[off], "result");
    mg_vxprintf(r->pfn, r->pfn_data, fmt == NULL ? "null" : fmt, ap);
    mg_xprintf(r->pfn, r->pfn_data, "}");
  }
}

void mg_rpc_ok(struct mg_rpc_req *r, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  mg_rpc_vok(r, fmt, &ap);
  va_end(ap);
}

void mg_rpc_verr(struct mg_rpc_req *r, int code, const char *fmt, va_list *ap) {
  int len, off = mg_json_get(r->frame, "$.id", &len);
  mg_xprintf(r->pfn, r->pfn_data, "{");
  if (off > 0) {
    mg_xprintf(r->pfn, r->pfn_data, "%Q:%.*s,", "id", len, &r->frame.ptr[off]);
  }
  mg_xprintf(r->pfn, r->pfn_data, "%Q:{%Q:%d,%Q:", "error", "code", code,
             "message");
  mg_vxprintf(r->pfn, r->pfn_data, fmt == NULL ? "null" : fmt, ap);
  mg_xprintf(r->pfn, r->pfn_data, "}}");
}

void mg_rpc_err(struct mg_rpc_req *r, int code, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  mg_rpc_verr(r, code, fmt, &ap);
  va_end(ap);
}

static size_t print_methods(mg_pfn_t pfn, void *pfn_data, va_list *ap) {
  struct mg_rpc *h, **head = (struct mg_rpc **) va_arg(*ap, void **);
  size_t len = 0;
  for (h = *head; h != NULL; h = h->next) {
    len += mg_xprintf(pfn, pfn_data, "%s%.*Q", h == *head ? "" : ",",
                      (int) h->method.len, h->method.ptr);
  }
  return len;
}

void mg_rpc_list(struct mg_rpc_req *r) {
  mg_rpc_ok(r, "[%M]", print_methods, r->head);
}

#ifdef MG_ENABLE_LINES
#line 1 "src/sha1.c"
#endif
/* Copyright(c) By Steve Reid <steve@edmweb.com> */
/* 100% Public Domain */



union char64long16 {
  unsigned char c[64];
  uint32_t l[16];
};

#define rol(value, bits) (((value) << (bits)) | ((value) >> (32 - (bits))))

static uint32_t blk0(union char64long16 *block, int i) {
  if (MG_BIG_ENDIAN) {
  } else {
    block->l[i] = (rol(block->l[i], 24) & 0xFF00FF00) |
                  (rol(block->l[i], 8) & 0x00FF00FF);
  }
  return block->l[i];
}

/* Avoid redefine warning (ARM /usr/include/sys/ucontext.h define R0~R4) */
#undef blk
#undef R0
#undef R1
#undef R2
#undef R3
#undef R4

#define blk(i)                                                               \
  (block->l[i & 15] = rol(block->l[(i + 13) & 15] ^ block->l[(i + 8) & 15] ^ \
                              block->l[(i + 2) & 15] ^ block->l[i & 15],     \
                          1))
#define R0(v, w, x, y, z, i)                                          \
  z += ((w & (x ^ y)) ^ y) + blk0(block, i) + 0x5A827999 + rol(v, 5); \
  w = rol(w, 30);
#define R1(v, w, x, y, z, i)                                  \
  z += ((w & (x ^ y)) ^ y) + blk(i) + 0x5A827999 + rol(v, 5); \
  w = rol(w, 30);
#define R2(v, w, x, y, z, i)                          \
  z += (w ^ x ^ y) + blk(i) + 0x6ED9EBA1 + rol(v, 5); \
  w = rol(w, 30);
#define R3(v, w, x, y, z, i)                                        \
  z += (((w | x) & y) | (w & x)) + blk(i) + 0x8F1BBCDC + rol(v, 5); \
  w = rol(w, 30);
#define R4(v, w, x, y, z, i)                          \
  z += (w ^ x ^ y) + blk(i) + 0xCA62C1D6 + rol(v, 5); \
  w = rol(w, 30);

static void mg_sha1_transform(uint32_t state[5],
                              const unsigned char buffer[64]) {
  uint32_t a, b, c, d, e;
  union char64long16 block[1];

  memcpy(block, buffer, 64);
  a = state[0];
  b = state[1];
  c = state[2];
  d = state[3];
  e = state[4];
  R0(a, b, c, d, e, 0);
  R0(e, a, b, c, d, 1);
  R0(d, e, a, b, c, 2);
  R0(c, d, e, a, b, 3);
  R0(b, c, d, e, a, 4);
  R0(a, b, c, d, e, 5);
  R0(e, a, b, c, d, 6);
  R0(d, e, a, b, c, 7);
  R0(c, d, e, a, b, 8);
  R0(b, c, d, e, a, 9);
  R0(a, b, c, d, e, 10);
  R0(e, a, b, c, d, 11);
  R0(d, e, a, b, c, 12);
  R0(c, d, e, a, b, 13);
  R0(b, c, d, e, a, 14);
  R0(a, b, c, d, e, 15);
  R1(e, a, b, c, d, 16);
  R1(d, e, a, b, c, 17);
  R1(c, d, e, a, b, 18);
  R1(b, c, d, e, a, 19);
  R2(a, b, c, d, e, 20);
  R2(e, a, b, c, d, 21);
  R2(d, e, a, b, c, 22);
  R2(c, d, e, a, b, 23);
  R2(b, c, d, e, a, 24);
  R2(a, b, c, d, e, 25);
  R2(e, a, b, c, d, 26);
  R2(d, e, a, b, c, 27);
  R2(c, d, e, a, b, 28);
  R2(b, c, d, e, a, 29);
  R2(a, b, c, d, e, 30);
  R2(e, a, b, c, d, 31);
  R2(d, e, a, b, c, 32);
  R2(c, d, e, a, b, 33);
  R2(b, c, d, e, a, 34);
  R2(a, b, c, d, e, 35);
  R2(e, a, b, c, d, 36);
  R2(d, e, a, b, c, 37);
  R2(c, d, e, a, b, 38);
  R2(b, c, d, e, a, 39);
  R3(a, b, c, d, e, 40);
  R3(e, a, b, c, d, 41);
  R3(d, e, a, b, c, 42);
  R3(c, d, e, a, b, 43);
  R3(b, c, d, e, a, 44);
  R3(a, b, c, d, e, 45);
  R3(e, a, b, c, d, 46);
  R3(d, e, a, b, c, 47);
  R3(c, d, e, a, b, 48);
  R3(b, c, d, e, a, 49);
  R3(a, b, c, d, e, 50);
  R3(e, a, b, c, d, 51);
  R3(d, e, a, b, c, 52);
  R3(c, d, e, a, b, 53);
  R3(b, c, d, e, a, 54);
  R3(a, b, c, d, e, 55);
  R3(e, a, b, c, d, 56);
  R3(d, e, a, b, c, 57);
  R3(c, d, e, a, b, 58);
  R3(b, c, d, e, a, 59);
  R4(a, b, c, d, e, 60);
  R4(e, a, b, c, d, 61);
  R4(d, e, a, b, c, 62);
  R4(c, d, e, a, b, 63);
  R4(b, c, d, e, a, 64);
  R4(a, b, c, d, e, 65);
  R4(e, a, b, c, d, 66);
  R4(d, e, a, b, c, 67);
  R4(c, d, e, a, b, 68);
  R4(b, c, d, e, a, 69);
  R4(a, b, c, d, e, 70);
  R4(e, a, b, c, d, 71);
  R4(d, e, a, b, c, 72);
  R4(c, d, e, a, b, 73);
  R4(b, c, d, e, a, 74);
  R4(a, b, c, d, e, 75);
  R4(e, a, b, c, d, 76);
  R4(d, e, a, b, c, 77);
  R4(c, d, e, a, b, 78);
  R4(b, c, d, e, a, 79);
  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
  /* Erase working structures. The order of operations is important,
   * used to ensure that compiler doesn't optimize those out. */
  memset(block, 0, sizeof(block));
  a = b = c = d = e = 0;
  (void) a;
  (void) b;
  (void) c;
  (void) d;
  (void) e;
}

void mg_sha1_init(mg_sha1_ctx *context) {
  context->state[0] = 0x67452301;
  context->state[1] = 0xEFCDAB89;
  context->state[2] = 0x98BADCFE;
  context->state[3] = 0x10325476;
  context->state[4] = 0xC3D2E1F0;
  context->count[0] = context->count[1] = 0;
}

void mg_sha1_update(mg_sha1_ctx *context, const unsigned char *data,
                    size_t len) {
  size_t i, j;

  j = context->count[0];
  if ((context->count[0] += (uint32_t) len << 3) < j) context->count[1]++;
  context->count[1] += (uint32_t) (len >> 29);
  j = (j >> 3) & 63;
  if ((j + len) > 63) {
    memcpy(&context->buffer[j], data, (i = 64 - j));
    mg_sha1_transform(context->state, context->buffer);
    for (; i + 63 < len; i += 64) {
      mg_sha1_transform(context->state, &data[i]);
    }
    j = 0;
  } else
    i = 0;
  memcpy(&context->buffer[j], &data[i], len - i);
}

void mg_sha1_final(unsigned char digest[20], mg_sha1_ctx *context) {
  unsigned i;
  unsigned char finalcount[8], c;

  for (i = 0; i < 8; i++) {
    finalcount[i] = (unsigned char) ((context->count[(i >= 4 ? 0 : 1)] >>
                                      ((3 - (i & 3)) * 8)) &
                                     255);
  }
  c = 0200;
  mg_sha1_update(context, &c, 1);
  while ((context->count[0] & 504) != 448) {
    c = 0000;
    mg_sha1_update(context, &c, 1);
  }
  mg_sha1_update(context, finalcount, 8);
  for (i = 0; i < 20; i++) {
    digest[i] =
        (unsigned char) ((context->state[i >> 2] >> ((3 - (i & 3)) * 8)) & 255);
  }
  memset(context, '\0', sizeof(*context));
  memset(&finalcount, '\0', sizeof(finalcount));
}

#ifdef MG_ENABLE_LINES
#line 1 "src/sntp.c"
#endif






#define SNTP_TIME_OFFSET 2208988800U  // (1970 - 1900) in seconds
#define SNTP_MAX_FRAC 4294967295.0    // 2 ** 32 - 1

static int64_t gettimestamp(const uint32_t *data) {
  uint32_t sec = mg_ntohl(data[0]), frac = mg_ntohl(data[1]);
  if (sec) sec -= SNTP_TIME_OFFSET;
  return ((int64_t) sec) * 1000 + (int64_t) (frac / SNTP_MAX_FRAC * 1000.0);
}

int64_t mg_sntp_parse(const unsigned char *buf, size_t len) {
  int64_t res = -1;
  int mode = len > 0 ? buf[0] & 7 : 0;
  int version = len > 0 ? (buf[0] >> 3) & 7 : 0;
  if (len < 48) {
    MG_ERROR(("%s", "corrupt packet"));
  } else if (mode != 4 && mode != 5) {
    MG_ERROR(("%s", "not a server reply"));
  } else if (buf[1] == 0) {
    MG_ERROR(("%s", "server sent a kiss of death"));
  } else if (version == 4 || version == 3) {
    // int64_t ref = gettimestamp((uint32_t *) &buf[16]);
    int64_t t0 = gettimestamp((uint32_t *) &buf[24]);
    int64_t t1 = gettimestamp((uint32_t *) &buf[32]);
    int64_t t2 = gettimestamp((uint32_t *) &buf[40]);
    int64_t t3 = (int64_t) mg_millis();
    int64_t delta = (t3 - t0) - (t2 - t1);
    MG_VERBOSE(("%lld %lld %lld %lld delta:%lld", t0, t1, t2, t3, delta));
    res = t2 + delta / 2;
  } else {
    MG_ERROR(("unexpected version: %d", version));
  }
  return res;
}

static void sntp_cb(struct mg_connection *c, int ev, void *evd, void *fnd) {
  if (ev == MG_EV_READ) {
    int64_t milliseconds = mg_sntp_parse(c->recv.buf, c->recv.len);
    if (milliseconds > 0) {
      MG_INFO(("%lu got time: %lld ms from epoch", c->id, milliseconds));
      mg_call(c, MG_EV_SNTP_TIME, (uint64_t *) &milliseconds);
      MG_VERBOSE(("%u.%u", (unsigned) (milliseconds / 1000),
                  (unsigned) (milliseconds % 1000)));
    }
    mg_iobuf_del(&c->recv, 0, c->recv.len);  // Free receive buffer
  } else if (ev == MG_EV_CONNECT) {
    mg_sntp_request(c);
  } else if (ev == MG_EV_CLOSE) {
  }
  (void) fnd;
  (void) evd;
}

void mg_sntp_request(struct mg_connection *c) {
  if (c->is_resolving) {
    MG_ERROR(("%lu wait until resolved", c->id));
  } else {
    int64_t now = (int64_t) mg_millis();  // Use int64_t, for vc98
    uint8_t buf[48] = {0};
    uint32_t *t = (uint32_t *) &buf[40];
    double frac = ((double) (now % 1000)) / 1000.0 * SNTP_MAX_FRAC;
    buf[0] = (0 << 6) | (4 << 3) | 3;
    t[0] = mg_htonl((uint32_t) (now / 1000) + SNTP_TIME_OFFSET);
    t[1] = mg_htonl((uint32_t) frac);
    mg_send(c, buf, sizeof(buf));
  }
}

struct mg_connection *mg_sntp_connect(struct mg_mgr *mgr, const char *url,
                                      mg_event_handler_t fn, void *fnd) {
  struct mg_connection *c = NULL;
  if (url == NULL) url = "udp://time.google.com:123";
  if ((c = mg_connect(mgr, url, fn, fnd)) != NULL) c->pfn = sntp_cb;
  return c;
}

#ifdef MG_ENABLE_LINES
#line 1 "src/sock.c"
#endif










#if MG_ENABLE_SOCKET
#if MG_ARCH == MG_ARCH_WIN32 && MG_ENABLE_WINSOCK
typedef unsigned long nfds_t;
#define MG_SOCK_ERRNO WSAGetLastError()
#if defined(_MSC_VER)
#pragma comment(lib, "ws2_32.lib")
#define alloca(a) _alloca(a)
#endif
#define poll(a, b, c) WSAPoll((a), (b), (c))
#ifndef SO_EXCLUSIVEADDRUSE
#define SO_EXCLUSIVEADDRUSE ((int) (~SO_REUSEADDR))
#endif
#elif MG_ARCH == MG_ARCH_FREERTOS_TCP
#define MG_SOCK_ERRNO errno
typedef Socket_t SOCKET;
#define INVALID_SOCKET FREERTOS_INVALID_SOCKET
#elif MG_ARCH == MG_ARCH_TIRTOS
#define MG_SOCK_ERRNO errno
#define closesocket(x) close(x)
#else
#define MG_SOCK_ERRNO errno
#ifndef closesocket
#define closesocket(x) close(x)
#endif
#define INVALID_SOCKET (-1)
typedef int SOCKET;
#endif

#define FD(c_) ((SOCKET) (size_t) (c_)->fd)
#define S2PTR(s_) ((void *) (size_t) (s_))

#ifndef MSG_NONBLOCKING
#define MSG_NONBLOCKING 0
#endif

#ifndef AF_INET6
#define AF_INET6 10
#endif

union usa {
  struct sockaddr sa;
  struct sockaddr_in sin;
#if MG_ENABLE_IPV6
  struct sockaddr_in6 sin6;
#endif
};

static socklen_t tousa(struct mg_addr *a, union usa *usa) {
  socklen_t len = sizeof(usa->sin);
  memset(usa, 0, sizeof(*usa));
  usa->sin.sin_family = AF_INET;
  usa->sin.sin_port = a->port;
  *(uint32_t *) &usa->sin.sin_addr = a->ip;
#if MG_ENABLE_IPV6
  if (a->is_ip6) {
    usa->sin.sin_family = AF_INET6;
    usa->sin6.sin6_port = a->port;
    memcpy(&usa->sin6.sin6_addr, a->ip6, sizeof(a->ip6));
    len = sizeof(usa->sin6);
  }
#endif
  return len;
}

static void tomgaddr(union usa *usa, struct mg_addr *a, bool is_ip6) {
  a->is_ip6 = is_ip6;
  a->port = usa->sin.sin_port;
  memcpy(&a->ip, &usa->sin.sin_addr, sizeof(a->ip));
#if MG_ENABLE_IPV6
  if (is_ip6) {
    memcpy(a->ip6, &usa->sin6.sin6_addr, sizeof(a->ip6));
    a->port = usa->sin6.sin6_port;
  }
#endif
}

static bool mg_sock_would_block(void) {
  int err = MG_SOCK_ERRNO;
  return err == EINPROGRESS || err == EWOULDBLOCK
#ifndef WINCE
         || err == EAGAIN || err == EINTR
#endif
#if MG_ARCH == MG_ARCH_WIN32 && MG_ENABLE_WINSOCK
         || err == WSAEINTR || err == WSAEWOULDBLOCK
#endif
      ;
}

static bool mg_sock_conn_reset(void) {
  int err = MG_SOCK_ERRNO;
#if MG_ARCH == MG_ARCH_WIN32 && MG_ENABLE_WINSOCK
  return err == WSAECONNRESET;
#else
  return err == EPIPE || err == ECONNRESET;
#endif
}

static void setlocaddr(SOCKET fd, struct mg_addr *addr) {
  union usa usa;
  socklen_t n = sizeof(usa);
  if (getsockname(fd, &usa.sa, &n) == 0) {
    tomgaddr(&usa, addr, n != sizeof(usa.sin));
  }
}

static void iolog(struct mg_connection *c, char *buf, long n, bool r) {
  if (n == MG_IO_WAIT) {
    // Do nothing
  } else if (n <= 0) {
    c->is_closing = 1;  // Termination. Don't call mg_error(): #1529
  } else if (n > 0) {
    if (c->is_hexdumping) {
      union usa usa;
      char t1[50], t2[50];
      socklen_t slen = sizeof(usa.sin);
      struct mg_addr a;
      memset(&usa, 0, sizeof(usa));
      memset(&a, 0, sizeof(a));
      if (getsockname(FD(c), &usa.sa, &slen) < 0) (void) 0;  // Ignore result
      tomgaddr(&usa, &a, c->rem.is_ip6);
      MG_INFO(("\n-- %lu %s %s %s %s %ld", c->id,
               mg_straddr(&a, t1, sizeof(t1)), r ? "<-" : "->",
               mg_straddr(&c->rem, t2, sizeof(t2)), c->label, n));

      mg_hexdump(buf, (size_t) n);
    }
    if (r) {
      c->recv.len += (size_t) n;
      mg_call(c, MG_EV_READ, &n);
    } else {
      mg_iobuf_del(&c->send, 0, (size_t) n);
      // if (c->send.len == 0) mg_iobuf_resize(&c->send, 0);
      if (c->send.len == 0) {
        MG_EPOLL_MOD(c, 0);
      }
      mg_call(c, MG_EV_WRITE, &n);
    }
  }
}

long mg_io_send(struct mg_connection *c, const void *buf, size_t len) {
  long n;
  if (c->is_udp) {
    union usa usa;
    socklen_t slen = tousa(&c->rem, &usa);
    n = sendto(FD(c), (char *) buf, len, 0, &usa.sa, slen);
    if (n > 0) setlocaddr(FD(c), &c->loc);
  } else {
    n = send(FD(c), (char *) buf, len, MSG_NONBLOCKING);
#if MG_ARCH == MG_ARCH_RTX
    if (n == BSD_EWOULDBLOCK) return MG_IO_WAIT;
#endif
  }
  if (n < 0 && mg_sock_would_block()) return MG_IO_WAIT;
  if (n < 0 && mg_sock_conn_reset()) return MG_IO_RESET;
  if (n <= 0) return MG_IO_ERR;
  return n;
}

bool mg_send(struct mg_connection *c, const void *buf, size_t len) {
  if (c->is_udp) {
    long n = mg_io_send(c, buf, len);
    MG_DEBUG(("%lu %p %d:%d %ld err %d", c->id, c->fd, (int) c->send.len,
              (int) c->recv.len, n, MG_SOCK_ERRNO));
    iolog(c, (char *) buf, n, false);
    return n > 0;
  } else {
    return mg_iobuf_add(&c->send, c->send.len, buf, len);
  }
}

static void mg_set_non_blocking_mode(SOCKET fd) {
#if defined(MG_CUSTOM_NONBLOCK)
  MG_CUSTOM_NONBLOCK(fd);
#elif MG_ARCH == MG_ARCH_WIN32 && MG_ENABLE_WINSOCK
  unsigned long on = 1;
  ioctlsocket(fd, FIONBIO, &on);
#elif MG_ARCH == MG_ARCH_RTX
  unsigned long on = 1;
  ioctlsocket(fd, FIONBIO, &on);
#elif MG_ARCH == MG_ARCH_FREERTOS_TCP
  const BaseType_t off = 0;
  if (setsockopt(fd, 0, FREERTOS_SO_RCVTIMEO, &off, sizeof(off)) != 0) (void) 0;
  if (setsockopt(fd, 0, FREERTOS_SO_SNDTIMEO, &off, sizeof(off)) != 0) (void) 0;
#elif MG_ARCH == MG_ARCH_FREERTOS_LWIP || MG_ARCH == MG_ARCH_RTX_LWIP
  lwip_fcntl(fd, F_SETFL, O_NONBLOCK);
#elif MG_ARCH == MG_ARCH_AZURERTOS
  fcntl(fd, F_SETFL, O_NONBLOCK);
#elif MG_ARCH == MG_ARCH_TIRTOS
  int val = 0;
  setsockopt(fd, SOL_SOCKET, SO_BLOCKING, &val, sizeof(val));
  // SPRU524J section 3.3.3 page 63, SO_SNDLOWAT
  int sz = sizeof(val);
  getsockopt(fd, SOL_SOCKET, SO_SNDBUF, &val, &sz);
  val /= 2;  // set send low-water mark at half send buffer size
  setsockopt(fd, SOL_SOCKET, SO_SNDLOWAT, &val, sizeof(val));
#else
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);  // Non-blocking mode
  fcntl(fd, F_SETFD, FD_CLOEXEC);                          // Set close-on-exec
#endif
}

bool mg_open_listener(struct mg_connection *c, const char *url) {
  SOCKET fd = INVALID_SOCKET;
  bool success = false;
  c->loc.port = mg_htons(mg_url_port(url));
  if (!mg_aton(mg_url_host(url), &c->loc)) {
    MG_ERROR(("invalid listening URL: %s", url));
  } else {
    union usa usa;
    socklen_t slen = tousa(&c->loc, &usa);
    int on = 1, af = c->loc.is_ip6 ? AF_INET6 : AF_INET;
    int type = strncmp(url, "udp:", 4) == 0 ? SOCK_DGRAM : SOCK_STREAM;
    int proto = type == SOCK_DGRAM ? IPPROTO_UDP : IPPROTO_TCP;
    (void) on;

    if ((fd = socket(af, type, proto)) == INVALID_SOCKET) {
      MG_ERROR(("socket: %d", MG_SOCK_ERRNO));
#if ((MG_ARCH == MG_ARCH_WIN32) || (MG_ARCH == MG_ARCH_UNIX) || \
     (defined(LWIP_SOCKET) && SO_REUSE == 1))
    } else if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (char *) &on,
                          sizeof(on)) != 0) {
      // 1. SO_RESUSEADDR is not enabled on Windows because the semantics of
      //    SO_REUSEADDR on UNIX and Windows is different. On Windows,
      //    SO_REUSEADDR allows to bind a socket to a port without error even
      //    if the port is already open by another program. This is not the
      //    behavior SO_REUSEADDR was designed for, and leads to hard-to-track
      //    failure scenarios. Therefore, SO_REUSEADDR was disabled on Windows
      //    unless SO_EXCLUSIVEADDRUSE is supported and set on a socket.
      // 2. In case of LWIP, SO_REUSEADDR should be explicitly enabled, by
      // defining
      //    SO_REUSE (in lwipopts.h), otherwise the code below will compile
      //    but won't work! (setsockopt will return EINVAL)
      MG_ERROR(("reuseaddr: %d", MG_SOCK_ERRNO));
#endif
#if MG_ARCH == MG_ARCH_WIN32 && !defined(SO_EXCLUSIVEADDRUSE) && !defined(WINCE)
    } else if (setsockopt(fd, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, (char *) &on,
                          sizeof(on)) != 0) {
      // "Using SO_REUSEADDR and SO_EXCLUSIVEADDRUSE"
      MG_ERROR(("exclusiveaddruse: %d", MG_SOCK_ERRNO));
#endif
    } else if (bind(fd, &usa.sa, slen) != 0) {
      MG_ERROR(("bind: %d", MG_SOCK_ERRNO));
    } else if ((type == SOCK_STREAM &&
                listen(fd, MG_SOCK_LISTEN_BACKLOG_SIZE) != 0)) {
      // NOTE(lsm): FreeRTOS uses backlog value as a connection limit
      // In case port was set to 0, get the real port number
      MG_ERROR(("listen: %d", MG_SOCK_ERRNO));
    } else {
      setlocaddr(fd, &c->loc);
      mg_set_non_blocking_mode(fd);
      c->fd = S2PTR(fd);
      MG_EPOLL_ADD(c);
      success = true;
    }
  }
  if (success == false && fd != INVALID_SOCKET) closesocket(fd);
  return success;
}

long mg_io_recv(struct mg_connection *c, void *buf, size_t len) {
  long n = 0;
  if (c->is_udp) {
    union usa usa;
    socklen_t slen = tousa(&c->rem, &usa);
    n = recvfrom(FD(c), (char *) buf, len, 0, &usa.sa, &slen);
    if (n > 0) tomgaddr(&usa, &c->rem, slen != sizeof(usa.sin));
  } else {
    n = recv(FD(c), (char *) buf, len, MSG_NONBLOCKING);
  }
  if (n < 0 && mg_sock_would_block()) return MG_IO_WAIT;
  if (n < 0 && mg_sock_conn_reset()) return MG_IO_RESET;
  if (n <= 0) return MG_IO_ERR;
  return n;
}

// NOTE(lsm): do only one iteration of reads, cause some systems
// (e.g. FreeRTOS stack) return 0 instead of -1/EWOULDBLOCK when no data
static void read_conn(struct mg_connection *c) {
  long n = -1;
  if (c->recv.len >= MG_MAX_RECV_SIZE) {
    mg_error(c, "max_recv_buf_size reached");
  } else if (c->recv.size <= c->recv.len &&
             !mg_iobuf_resize(&c->recv, c->recv.size + MG_IO_SIZE)) {
    mg_error(c, "oom");
  } else {
    char *buf = (char *) &c->recv.buf[c->recv.len];
    size_t len = c->recv.size - c->recv.len;
    n = c->is_tls ? mg_tls_recv(c, buf, len) : mg_io_recv(c, buf, len);
    MG_DEBUG(("%lu %p snd %ld/%ld rcv %ld/%ld n=%ld err=%d", c->id, c->fd,
              (long) c->send.len, (long) c->send.size, (long) c->recv.len,
              (long) c->recv.size, n, MG_SOCK_ERRNO));
    iolog(c, buf, n, true);
  }
}

static void write_conn(struct mg_connection *c) {
  char *buf = (char *) c->send.buf;
  size_t len = c->send.len;
  long n = c->is_tls ? mg_tls_send(c, buf, len) : mg_io_send(c, buf, len);
  MG_DEBUG(("%lu %p snd %ld/%ld rcv %ld/%ld n=%ld err=%d", c->id, c->fd,
            (long) c->send.len, (long) c->send.size, (long) c->recv.len,
            (long) c->recv.size, n, MG_SOCK_ERRNO));
  iolog(c, buf, n, false);
}

static void close_conn(struct mg_connection *c) {
  if (FD(c) != INVALID_SOCKET) {
#if MG_ENABLE_EPOLL
    epoll_ctl(c->mgr->epoll_fd, EPOLL_CTL_DEL, FD(c), NULL);
#endif
    closesocket(FD(c));
#if MG_ARCH == MG_ARCH_FREERTOS_TCP
    FreeRTOS_FD_CLR(c->fd, c->mgr->ss, eSELECT_ALL);
#endif
  }
  mg_close_conn(c);
}

static void connect_conn(struct mg_connection *c) {
  union usa usa;
  socklen_t n = sizeof(usa);
  // Use getpeername() to test whether we have connected
  if (getpeername(FD(c), &usa.sa, &n) == 0) {
    c->is_connecting = 0;
    mg_call(c, MG_EV_CONNECT, NULL);
    MG_EPOLL_MOD(c, 0);
    if (c->is_tls_hs) mg_tls_handshake(c);
  } else {
    mg_error(c, "socket error");
  }
}

static void setsockopts(struct mg_connection *c) {
#if MG_ARCH == MG_ARCH_FREERTOS_TCP || MG_ARCH == MG_ARCH_AZURERTOS || \
    MG_ARCH == MG_ARCH_TIRTOS
  (void) c;
#else
  int on = 1;
#if !defined(SOL_TCP)
#define SOL_TCP IPPROTO_TCP
#endif
  if (setsockopt(FD(c), SOL_TCP, TCP_NODELAY, (char *) &on, sizeof(on)) != 0)
    (void) 0;
  if (setsockopt(FD(c), SOL_SOCKET, SO_KEEPALIVE, (char *) &on, sizeof(on)) !=
      0)
    (void) 0;
#endif
}

void mg_connect_resolved(struct mg_connection *c) {
  int type = c->is_udp ? SOCK_DGRAM : SOCK_STREAM;
  int rc, af = c->rem.is_ip6 ? AF_INET6 : AF_INET;  // c->rem has resolved IP
  c->fd = S2PTR(socket(af, type, 0));               // Create outbound socket
  c->is_resolving = 0;                              // Clear resolving flag
  if (FD(c) == INVALID_SOCKET) {
    mg_error(c, "socket(): %d", MG_SOCK_ERRNO);
  } else if (c->is_udp) {
    MG_EPOLL_ADD(c);
#if MG_ARCH == MG_ARCH_TIRTOS
    union usa usa;  // TI-RTOS NDK requires binding to receive on UDP sockets
    socklen_t slen = tousa(&c->loc, &usa);
    if (bind(c->fd, &usa.sa, slen) != 0) MG_ERROR(("bind: %d", MG_SOCK_ERRNO));
#endif
    mg_call(c, MG_EV_RESOLVE, NULL);
    mg_call(c, MG_EV_CONNECT, NULL);
  } else {
    union usa usa;
    socklen_t slen = tousa(&c->rem, &usa);
    mg_set_non_blocking_mode(FD(c));
    setsockopts(c);
    MG_EPOLL_ADD(c);
    mg_call(c, MG_EV_RESOLVE, NULL);
    if ((rc = connect(FD(c), &usa.sa, slen)) == 0) {
      mg_call(c, MG_EV_CONNECT, NULL);
    } else if (mg_sock_would_block()) {
      MG_DEBUG(("%lu %p -> %x:%hu pend", c->id, c->fd, mg_ntohl(c->rem.ip),
                mg_ntohs(c->rem.port)));
      c->is_connecting = 1;
    } else {
      mg_error(c, "connect: %d", MG_SOCK_ERRNO);
    }
  }
  (void) rc;
}

static SOCKET raccept(SOCKET sock, union usa *usa, socklen_t len) {
  SOCKET s = INVALID_SOCKET;
  do {
    memset(usa, 0, sizeof(*usa));
    s = accept(sock, &usa->sa, &len);
  } while (s == INVALID_SOCKET && errno == EINTR);
  return s;
}

static void accept_conn(struct mg_mgr *mgr, struct mg_connection *lsn) {
  struct mg_connection *c = NULL;
  union usa usa;
  socklen_t sa_len = sizeof(usa);
  SOCKET fd = raccept(FD(lsn), &usa, sa_len);
  if (fd == INVALID_SOCKET) {
#if MG_ARCH == MG_ARCH_AZURERTOS
    // AzureRTOS, in non-block socket mode can mark listening socket readable
    // even it is not. See comment for 'select' func implementation in
    // nx_bsd.c That's not an error, just should try later
    if (MG_SOCK_ERRNO != EAGAIN)
#endif
      MG_ERROR(("%lu accept failed, errno %d", lsn->id, MG_SOCK_ERRNO));
#if (MG_ARCH != MG_ARCH_WIN32) && (MG_ARCH != MG_ARCH_FREERTOS_TCP) && \
    (MG_ARCH != MG_ARCH_TIRTOS) && !(MG_ENABLE_POLL)
  } else if ((long) fd >= FD_SETSIZE) {
    MG_ERROR(("%ld > %ld", (long) fd, (long) FD_SETSIZE));
    closesocket(fd);
#endif
  } else if ((c = mg_alloc_conn(mgr)) == NULL) {
    MG_ERROR(("%lu OOM", lsn->id));
    closesocket(fd);
  } else {
    // char buf[40];
    tomgaddr(&usa, &c->rem, sa_len != sizeof(usa.sin));
    // mg_straddr(&c->rem, buf, sizeof(buf));
    LIST_ADD_HEAD(struct mg_connection, &mgr->conns, c);
    c->fd = S2PTR(fd);
    MG_EPOLL_ADD(c);
    mg_set_non_blocking_mode(FD(c));
    setsockopts(c);
    c->is_accepted = 1;
    c->is_hexdumping = lsn->is_hexdumping;
    c->loc = lsn->loc;
    c->pfn = lsn->pfn;
    c->pfn_data = lsn->pfn_data;
    c->fn = lsn->fn;
    c->fn_data = lsn->fn_data;
    MG_DEBUG(("%lu %p accepted %x.%hu -> %x.%hu", c->id, c->fd,
              mg_ntohl(c->rem.ip), mg_ntohs(c->rem.port), mg_ntohl(c->loc.ip),
              mg_ntohs(c->loc.port)));
    mg_call(c, MG_EV_OPEN, NULL);
    mg_call(c, MG_EV_ACCEPT, NULL);
  }
}

static bool mg_socketpair(SOCKET sp[2], union usa usa[2], bool udp) {
  SOCKET sock;
  socklen_t n = sizeof(usa[0].sin);
  bool success = false;

  sock = sp[0] = sp[1] = INVALID_SOCKET;
  (void) memset(&usa[0], 0, sizeof(usa[0]));
  usa[0].sin.sin_family = AF_INET;
  *(uint32_t *) &usa->sin.sin_addr = mg_htonl(0x7f000001U);  // 127.0.0.1
  usa[1] = usa[0];

  if (udp && (sp[0] = socket(AF_INET, SOCK_DGRAM, 0)) != INVALID_SOCKET &&
      (sp[1] = socket(AF_INET, SOCK_DGRAM, 0)) != INVALID_SOCKET &&
      bind(sp[0], &usa[0].sa, n) == 0 && bind(sp[1], &usa[1].sa, n) == 0 &&
      getsockname(sp[0], &usa[0].sa, &n) == 0 &&
      getsockname(sp[1], &usa[1].sa, &n) == 0 &&
      connect(sp[0], &usa[1].sa, n) == 0 &&
      connect(sp[1], &usa[0].sa, n) == 0) {
    success = true;
  } else if (!udp &&
             (sock = socket(AF_INET, SOCK_STREAM, 0)) != INVALID_SOCKET &&
             bind(sock, &usa[0].sa, n) == 0 &&
             listen(sock, MG_SOCK_LISTEN_BACKLOG_SIZE) == 0 &&
             getsockname(sock, &usa[0].sa, &n) == 0 &&
             (sp[0] = socket(AF_INET, SOCK_STREAM, 0)) != INVALID_SOCKET &&
             connect(sp[0], &usa[0].sa, n) == 0 &&
             (sp[1] = raccept(sock, &usa[1], n)) != INVALID_SOCKET) {
    success = true;
  }
  if (success) {
    mg_set_non_blocking_mode(sp[1]);
  } else {
    if (sp[0] != INVALID_SOCKET) closesocket(sp[0]);
    if (sp[1] != INVALID_SOCKET) closesocket(sp[1]);
    sp[0] = sp[1] = INVALID_SOCKET;
  }
  if (sock != INVALID_SOCKET) closesocket(sock);
  return success;
}

int mg_mkpipe(struct mg_mgr *mgr, mg_event_handler_t fn, void *fn_data,
              bool udp) {
  union usa usa[2];
  SOCKET sp[2] = {INVALID_SOCKET, INVALID_SOCKET};
  struct mg_connection *c = NULL;
  if (!mg_socketpair(sp, usa, udp)) {
    MG_ERROR(("Cannot create socket pair"));
  } else if ((c = mg_wrapfd(mgr, (int) sp[1], fn, fn_data)) == NULL) {
    closesocket(sp[0]);
    closesocket(sp[1]);
    sp[0] = sp[1] = INVALID_SOCKET;
  } else {
    tomgaddr(&usa[0], &c->rem, false);
    MG_DEBUG(("%lu %p pipe %lu", c->id, c->fd, (unsigned long) sp[0]));
  }
  return (int) sp[0];
}

static bool can_read(const struct mg_connection *c) {
  return c->is_full == false;
}

static bool can_write(const struct mg_connection *c) {
  return c->is_connecting || (c->send.len > 0 && c->is_tls_hs == 0);
}

static bool skip_iotest(const struct mg_connection *c) {
  return (c->is_closing || c->is_resolving || FD(c) == INVALID_SOCKET) ||
         (can_read(c) == false && can_write(c) == false);
}

static void mg_iotest(struct mg_mgr *mgr, int ms) {
#if MG_ARCH == MG_ARCH_FREERTOS_TCP
  struct mg_connection *c;
  for (c = mgr->conns; c != NULL; c = c->next) {
    c->is_readable = c->is_writable = 0;
    if (skip_iotest(c)) continue;
    if (can_read(c))
      FreeRTOS_FD_SET(c->fd, mgr->ss, eSELECT_READ | eSELECT_EXCEPT);
    if (can_write(c)) FreeRTOS_FD_SET(c->fd, mgr->ss, eSELECT_WRITE);
  }
  FreeRTOS_select(mgr->ss, pdMS_TO_TICKS(ms));
  for (c = mgr->conns; c != NULL; c = c->next) {
    EventBits_t bits = FreeRTOS_FD_ISSET(c->fd, mgr->ss);
    c->is_readable = bits & (eSELECT_READ | eSELECT_EXCEPT) ? 1 : 0;
    c->is_writable = bits & eSELECT_WRITE ? 1 : 0;
    FreeRTOS_FD_CLR(c->fd, mgr->ss,
                    eSELECT_READ | eSELECT_EXCEPT | eSELECT_WRITE);
  }
#elif MG_ENABLE_EPOLL
  size_t max = 1;
  for (struct mg_connection *c = mgr->conns; c != NULL; c = c->next) {
    c->is_readable = c->is_writable = 0;
    if (mg_tls_pending(c) > 0) ms = 1, c->is_readable = 1;
    if (can_write(c)) MG_EPOLL_MOD(c, 1);
    max++;
  }
  struct epoll_event *evs = (struct epoll_event *) alloca(max * sizeof(evs[0]));
  int n = epoll_wait(mgr->epoll_fd, evs, (int) max, ms);
  for (int i = 0; i < n; i++) {
    struct mg_connection *c = (struct mg_connection *) evs[i].data.ptr;
    if (evs[i].events & EPOLLERR) {
      mg_error(c, "socket error");
    } else if (c->is_readable == 0) {
      bool rd = evs[i].events & (EPOLLIN | EPOLLHUP);
      bool wr = evs[i].events & EPOLLOUT;
      c->is_readable = can_read(c) && rd ? 1U : 0;
      c->is_writable = can_write(c) && wr ? 1U : 0;
    }
  }
  (void) skip_iotest;
#elif MG_ENABLE_POLL
  nfds_t n = 0;
  for (struct mg_connection *c = mgr->conns; c != NULL; c = c->next) n++;
  struct pollfd *fds = (struct pollfd *) alloca(n * sizeof(fds[0]));
  memset(fds, 0, n * sizeof(fds[0]));
  n = 0;
  for (struct mg_connection *c = mgr->conns; c != NULL; c = c->next) {
    c->is_readable = c->is_writable = 0;
    if (skip_iotest(c)) {
      // Socket not valid, ignore
    } else if (mg_tls_pending(c) > 0) {
      ms = 1;  // Don't wait if TLS is ready
    } else {
      fds[n].fd = FD(c);
      if (can_read(c)) fds[n].events |= POLLIN;
      if (can_write(c)) fds[n].events |= POLLOUT;
      n++;
    }
  }

  // MG_INFO(("poll n=%d ms=%d", (int) n, ms));
  if (poll(fds, n, ms) < 0) {
#if MG_ARCH == MG_ARCH_WIN32
    if (n == 0) Sleep(ms);  // On Windows, poll fails if no sockets
#endif
    memset(fds, 0, n * sizeof(fds[0]));
  }
  n = 0;
  for (struct mg_connection *c = mgr->conns; c != NULL; c = c->next) {
    if (skip_iotest(c)) {
      // Socket not valid, ignore
    } else if (mg_tls_pending(c) > 0) {
      c->is_readable = 1;
    } else {
      if (fds[n].revents & POLLERR) {
        mg_error(c, "socket error");
      } else {
        c->is_readable =
            (unsigned) (fds[n].revents & (POLLIN | POLLHUP) ? 1 : 0);
        c->is_writable = (unsigned) (fds[n].revents & POLLOUT ? 1 : 0);
      }
      n++;
    }
  }
#else
  struct timeval tv = {ms / 1000, (ms % 1000) * 1000}, tv_zero = {0, 0};
  struct mg_connection *c;
  fd_set rset, wset, eset;
  SOCKET maxfd = 0;
  int rc;

  FD_ZERO(&rset);
  FD_ZERO(&wset);
  FD_ZERO(&eset);
  for (c = mgr->conns; c != NULL; c = c->next) {
    c->is_readable = c->is_writable = 0;
    if (skip_iotest(c)) continue;
    FD_SET(FD(c), &eset);
    if (can_read(c)) FD_SET(FD(c), &rset);
    if (can_write(c)) FD_SET(FD(c), &wset);
    if (mg_tls_pending(c) > 0) tv = tv_zero;
    if (FD(c) > maxfd) maxfd = FD(c);
  }

  if ((rc = select((int) maxfd + 1, &rset, &wset, &eset, &tv)) < 0) {
#if MG_ARCH == MG_ARCH_WIN32
    if (maxfd == 0) Sleep(ms);  // On Windows, select fails if no sockets
#else
    MG_ERROR(("select: %d %d", rc, MG_SOCK_ERRNO));
#endif
    FD_ZERO(&rset);
    FD_ZERO(&wset);
    FD_ZERO(&eset);
  }

  for (c = mgr->conns; c != NULL; c = c->next) {
    if (FD(c) != INVALID_SOCKET && FD_ISSET(FD(c), &eset)) {
      mg_error(c, "socket error");
    } else {
      c->is_readable = FD(c) != INVALID_SOCKET && FD_ISSET(FD(c), &rset);
      c->is_writable = FD(c) != INVALID_SOCKET && FD_ISSET(FD(c), &wset);
      if (mg_tls_pending(c) > 0) c->is_readable = 1;
    }
  }
#endif
}

void mg_mgr_poll(struct mg_mgr *mgr, int ms) {
  struct mg_connection *c, *tmp;
  uint64_t now;

  mg_iotest(mgr, ms);
  now = mg_millis();
  mg_timer_poll(&mgr->timers, now);

  for (c = mgr->conns; c != NULL; c = tmp) {
    bool is_resp = c->is_resp;
    tmp = c->next;
    mg_call(c, MG_EV_POLL, &now);
    if (is_resp && !c->is_resp) {
      long n = 0;
      mg_call(c, MG_EV_READ, &n);
    }
    MG_VERBOSE(("%lu %c%c %c%c%c%c%c", c->id, c->is_readable ? 'r' : '-',
                c->is_writable ? 'w' : '-', c->is_tls ? 'T' : 't',
                c->is_connecting ? 'C' : 'c', c->is_tls_hs ? 'H' : 'h',
                c->is_resolving ? 'R' : 'r', c->is_closing ? 'C' : 'c'));
    if (c->is_resolving || c->is_closing) {
      // Do nothing
    } else if (c->is_listening && c->is_udp == 0) {
      if (c->is_readable) accept_conn(mgr, c);
    } else if (c->is_connecting) {
      if (c->is_readable || c->is_writable) connect_conn(c);
    } else if (c->is_tls_hs) {
      if ((c->is_readable || c->is_writable)) mg_tls_handshake(c);
    } else {
      if (c->is_readable) read_conn(c);
      if (c->is_writable) write_conn(c);
    }

    if (c->is_draining && c->send.len == 0) c->is_closing = 1;
    if (c->is_closing) close_conn(c);
  }
}
#endif

#ifdef MG_ENABLE_LINES
#line 1 "src/ssi.c"
#endif




#ifndef MG_MAX_SSI_DEPTH
#define MG_MAX_SSI_DEPTH 5
#endif

#ifndef MG_SSI_BUFSIZ
#define MG_SSI_BUFSIZ 1024
#endif

#if MG_ENABLE_SSI
static char *mg_ssi(const char *path, const char *root, int depth) {
  struct mg_iobuf b = {NULL, 0, 0, MG_IO_SIZE};
  FILE *fp = fopen(path, "rb");
  if (fp != NULL) {
    char buf[MG_SSI_BUFSIZ], arg[sizeof(buf)];
    int ch, intag = 0;
    size_t len = 0;
    buf[0] = arg[0] = '\0';
    while ((ch = fgetc(fp)) != EOF) {
      if (intag && ch == '>' && buf[len - 1] == '-' && buf[len - 2] == '-') {
        buf[len++] = (char) (ch & 0xff);
        buf[len] = '\0';
        if (sscanf(buf, "<!--#include file=\"%[^\"]", arg)) {
          char tmp[MG_PATH_MAX + MG_SSI_BUFSIZ + 10],
              *p = (char *) path + strlen(path), *data;
          while (p > path && p[-1] != MG_DIRSEP && p[-1] != '/') p--;
          mg_snprintf(tmp, sizeof(tmp), "%.*s%s", (int) (p - path), path, arg);
          if (depth < MG_MAX_SSI_DEPTH &&
              (data = mg_ssi(tmp, root, depth + 1)) != NULL) {
            mg_iobuf_add(&b, b.len, data, strlen(data));
            free(data);
          } else {
            MG_ERROR(("%s: file=%s error or too deep", path, arg));
          }
        } else if (sscanf(buf, "<!--#include virtual=\"%[^\"]", arg)) {
          char tmp[MG_PATH_MAX + MG_SSI_BUFSIZ + 10], *data;
          mg_snprintf(tmp, sizeof(tmp), "%s%s", root, arg);
          if (depth < MG_MAX_SSI_DEPTH &&
              (data = mg_ssi(tmp, root, depth + 1)) != NULL) {
            mg_iobuf_add(&b, b.len, data, strlen(data));
            free(data);
          } else {
            MG_ERROR(("%s: virtual=%s error or too deep", path, arg));
          }
        } else {
          // Unknown SSI tag
          MG_ERROR(("Unknown SSI tag: %.*s", (int) len, buf));
          mg_iobuf_add(&b, b.len, buf, len);
        }
        intag = 0;
        len = 0;
      } else if (ch == '<') {
        intag = 1;
        if (len > 0) mg_iobuf_add(&b, b.len, buf, len);
        len = 0;
        buf[len++] = (char) (ch & 0xff);
      } else if (intag) {
        if (len == 5 && strncmp(buf, "<!--#", 5) != 0) {
          intag = 0;
        } else if (len >= sizeof(buf) - 2) {
          MG_ERROR(("%s: SSI tag is too large", path));
          len = 0;
        }
        buf[len++] = (char) (ch & 0xff);
      } else {
        buf[len++] = (char) (ch & 0xff);
        if (len >= sizeof(buf)) {
          mg_iobuf_add(&b, b.len, buf, len);
          len = 0;
        }
      }
    }
    if (len > 0) mg_iobuf_add(&b, b.len, buf, len);
    if (b.len > 0) mg_iobuf_add(&b, b.len, "", 1);  // nul-terminate
    fclose(fp);
  }
  (void) depth;
  (void) root;
  return (char *) b.buf;
}

void mg_http_serve_ssi(struct mg_connection *c, const char *root,
                       const char *fullpath) {
  const char *headers = "Content-Type: text/html; charset=utf-8\r\n";
  char *data = mg_ssi(fullpath, root, 0);
  mg_http_reply(c, 200, headers, "%s", data == NULL ? "" : data);
  free(data);
}
#else
void mg_http_serve_ssi(struct mg_connection *c, const char *root,
                       const char *fullpath) {
  mg_http_reply(c, 501, NULL, "SSI not enabled");
  (void) root, (void) fullpath;
}
#endif

#ifdef MG_ENABLE_LINES
#line 1 "src/str.c"
#endif


struct mg_str mg_str_s(const char *s) {
  struct mg_str str = {s, s == NULL ? 0 : strlen(s)};
  return str;
}

struct mg_str mg_str_n(const char *s, size_t n) {
  struct mg_str str = {s, n};
  return str;
}

int mg_lower(const char *s) {
  int c = *s;
  if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
  return c;
}

int mg_ncasecmp(const char *s1, const char *s2, size_t len) {
  int diff = 0;
  if (len > 0) do {
      diff = mg_lower(s1++) - mg_lower(s2++);
    } while (diff == 0 && s1[-1] != '\0' && --len > 0);
  return diff;
}

int mg_casecmp(const char *s1, const char *s2) {
  return mg_ncasecmp(s1, s2, (size_t) ~0);
}

int mg_vcmp(const struct mg_str *s1, const char *s2) {
  size_t n2 = strlen(s2), n1 = s1->len;
  int r = strncmp(s1->ptr, s2, (n1 < n2) ? n1 : n2);
  if (r == 0) return (int) (n1 - n2);
  return r;
}

int mg_vcasecmp(const struct mg_str *str1, const char *str2) {
  size_t n2 = strlen(str2), n1 = str1->len;
  int r = mg_ncasecmp(str1->ptr, str2, (n1 < n2) ? n1 : n2);
  if (r == 0) return (int) (n1 - n2);
  return r;
}

struct mg_str mg_strdup(const struct mg_str s) {
  struct mg_str r = {NULL, 0};
  if (s.len > 0 && s.ptr != NULL) {
    char *sc = (char *) calloc(1, s.len + 1);
    if (sc != NULL) {
      memcpy(sc, s.ptr, s.len);
      sc[s.len] = '\0';
      r.ptr = sc;
      r.len = s.len;
    }
  }
  return r;
}

int mg_strcmp(const struct mg_str str1, const struct mg_str str2) {
  size_t i = 0;
  while (i < str1.len && i < str2.len) {
    int c1 = str1.ptr[i];
    int c2 = str2.ptr[i];
    if (c1 < c2) return -1;
    if (c1 > c2) return 1;
    i++;
  }
  if (i < str1.len) return 1;
  if (i < str2.len) return -1;
  return 0;
}

const char *mg_strstr(const struct mg_str haystack,
                      const struct mg_str needle) {
  size_t i;
  if (needle.len > haystack.len) return NULL;
  for (i = 0; i <= haystack.len - needle.len; i++) {
    if (memcmp(haystack.ptr + i, needle.ptr, needle.len) == 0) {
      return haystack.ptr + i;
    }
  }
  return NULL;
}

static bool is_space(int c) {
  return c == ' ' || c == '\r' || c == '\n' || c == '\t';
}

struct mg_str mg_strstrip(struct mg_str s) {
  while (s.len > 0 && is_space((int) *s.ptr)) s.ptr++, s.len--;
  while (s.len > 0 && is_space((int) *(s.ptr + s.len - 1))) s.len--;
  return s;
}

bool mg_match(struct mg_str s, struct mg_str p, struct mg_str *caps) {
  size_t i = 0, j = 0, ni = 0, nj = 0;
  if (caps) caps->ptr = NULL, caps->len = 0;
  while (i < p.len || j < s.len) {
    if (i < p.len && j < s.len && (p.ptr[i] == '?' || s.ptr[j] == p.ptr[i])) {
      if (caps == NULL) {
      } else if (p.ptr[i] == '?') {
        caps->ptr = &s.ptr[j], caps->len = 1;     // Finalize `?` cap
        caps++, caps->ptr = NULL, caps->len = 0;  // Init next cap
      } else if (caps->ptr != NULL && caps->len == 0) {
        caps->len = (size_t) (&s.ptr[j] - caps->ptr);  // Finalize current cap
        caps++, caps->len = 0, caps->ptr = NULL;       // Init next cap
      }
      i++, j++;
    } else if (i < p.len && (p.ptr[i] == '*' || p.ptr[i] == '#')) {
      if (caps && !caps->ptr) caps->len = 0, caps->ptr = &s.ptr[j];  // Init cap
      ni = i++, nj = j + 1;
    } else if (nj > 0 && nj <= s.len && (p.ptr[ni] == '#' || s.ptr[j] != '/')) {
      i = ni, j = nj;
      if (caps && caps->ptr == NULL && caps->len == 0) {
        caps--, caps->len = 0;  // Restart previous cap
      }
    } else {
      return false;
    }
  }
  if (caps && caps->ptr && caps->len == 0) {
    caps->len = (size_t) (&s.ptr[j] - caps->ptr);
  }
  return true;
}

bool mg_globmatch(const char *s1, size_t n1, const char *s2, size_t n2) {
  return mg_match(mg_str_n(s2, n2), mg_str_n(s1, n1), NULL);
}

static size_t mg_nce(const char *s, size_t n, size_t ofs, size_t *koff,
                     size_t *klen, size_t *voff, size_t *vlen, char delim) {
  size_t kvlen, kl;
  for (kvlen = 0; ofs + kvlen < n && s[ofs + kvlen] != delim;) kvlen++;
  for (kl = 0; kl < kvlen && s[ofs + kl] != '=';) kl++;
  if (koff != NULL) *koff = ofs;
  if (klen != NULL) *klen = kl;
  if (voff != NULL) *voff = kl < kvlen ? ofs + kl + 1 : 0;
  if (vlen != NULL) *vlen = kl < kvlen ? kvlen - kl - 1 : 0;
  ofs += kvlen + 1;
  return ofs > n ? n : ofs;
}

bool mg_split(struct mg_str *s, struct mg_str *k, struct mg_str *v, char sep) {
  size_t koff = 0, klen = 0, voff = 0, vlen = 0, off = 0;
  if (s->ptr == NULL || s->len == 0) return 0;
  off = mg_nce(s->ptr, s->len, 0, &koff, &klen, &voff, &vlen, sep);
  if (k != NULL) *k = mg_str_n(s->ptr + koff, klen);
  if (v != NULL) *v = mg_str_n(s->ptr + voff, vlen);
  *s = mg_str_n(s->ptr + off, s->len - off);
  return off > 0;
}

bool mg_commalist(struct mg_str *s, struct mg_str *k, struct mg_str *v) {
  return mg_split(s, k, v, ',');
}

char *mg_hex(const void *buf, size_t len, char *to) {
  const unsigned char *p = (const unsigned char *) buf;
  const char *hex = "0123456789abcdef";
  size_t i = 0;
  for (; len--; p++) {
    to[i++] = hex[p[0] >> 4];
    to[i++] = hex[p[0] & 0x0f];
  }
  to[i] = '\0';
  return to;
}

static unsigned char mg_unhex_nimble(unsigned char c) {
  return (c >= '0' && c <= '9')   ? (unsigned char) (c - '0')
         : (c >= 'A' && c <= 'F') ? (unsigned char) (c - '7')
                                  : (unsigned char) (c - 'W');
}

unsigned long mg_unhexn(const char *s, size_t len) {
  unsigned long i = 0, v = 0;
  for (i = 0; i < len; i++) v <<= 4, v |= mg_unhex_nimble(((uint8_t *) s)[i]);
  return v;
}

void mg_unhex(const char *buf, size_t len, unsigned char *to) {
  size_t i;
  for (i = 0; i < len; i += 2) {
    to[i >> 1] = (unsigned char) mg_unhexn(&buf[i], 2);
  }
}

uint64_t mg_tou64(struct mg_str str) {
  uint64_t result = 0;
  size_t i = 0;
  while (i < str.len && (str.ptr[i] == ' ' || str.ptr[i] == '\t')) i++;
  while (i < str.len && str.ptr[i] >= '0' && str.ptr[i] <= '9') {
    result *= 10;
    result += (unsigned) (str.ptr[i] - '0');
    i++;
  }
  return result;
}

int64_t mg_to64(struct mg_str str) {
  int64_t result = 0, neg = 1, max = 922337203685477570 /* INT64_MAX/10-10 */;
  size_t i = 0;
  while (i < str.len && (str.ptr[i] == ' ' || str.ptr[i] == '\t')) i++;
  if (i < str.len && str.ptr[i] == '-') neg = -1, i++;
  while (i < str.len && str.ptr[i] >= '0' && str.ptr[i] <= '9') {
    if (result > max) return 0;
    result *= 10;
    result += (str.ptr[i] - '0');
    i++;
  }
  return result * neg;
}

char *mg_remove_double_dots(char *s) {
  char *saved = s, *p = s;
  while (*s != '\0') {
    *p++ = *s++;
    if (s[-1] == '/' || s[-1] == '\\') {
      while (s[0] != '\0') {
        if (s[0] == '/' || s[0] == '\\') {
          s++;
        } else if (s[0] == '.' && s[1] == '.' &&
                   (s[2] == '/' || s[2] == '\\')) {
          s += 2;
        } else {
          break;
        }
      }
    }
  }
  *p = '\0';
  return saved;
}

#ifdef MG_ENABLE_LINES
#line 1 "src/timer.c"
#endif



#define MG_TIMER_CALLED 4

void mg_timer_init(struct mg_timer **head, struct mg_timer *t, uint64_t ms,
                   unsigned flags, void (*fn)(void *), void *arg) {
  t->id = 0, t->period_ms = ms, t->expire = 0;
  t->flags = flags, t->fn = fn, t->arg = arg, t->next = *head;
  *head = t;
}

void mg_timer_free(struct mg_timer **head, struct mg_timer *t) {
  while (*head && *head != t) head = &(*head)->next;
  if (*head) *head = t->next;
}

// t: expiration time, prd: period, now: current time. Return true if expired
bool mg_timer_expired(uint64_t *t, uint64_t prd, uint64_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

void mg_timer_poll(struct mg_timer **head, uint64_t now_ms) {
  struct mg_timer *t, *tmp;
  for (t = *head; t != NULL; t = tmp) {
    bool once = t->expire == 0 && (t->flags & MG_TIMER_RUN_NOW) &&
                !(t->flags & MG_TIMER_CALLED);  // Handle MG_TIMER_NOW only once
    bool expired = mg_timer_expired(&t->expire, t->period_ms, now_ms);
    tmp = t->next;
    if (!once && !expired) continue;
    if ((t->flags & MG_TIMER_REPEAT) || !(t->flags & MG_TIMER_CALLED)) {
      t->fn(t->arg);
    }
    t->flags |= MG_TIMER_CALLED;
  }
}

#ifdef MG_ENABLE_LINES
#line 1 "src/tls_dummy.c"
#endif


#if !MG_ENABLE_MBEDTLS && !MG_ENABLE_OPENSSL && !MG_ENABLE_CUSTOM_TLS
void mg_tls_init(struct mg_connection *c, const struct mg_tls_opts *opts) {
  (void) opts;
  mg_error(c, "TLS is not enabled");
}
void mg_tls_handshake(struct mg_connection *c) {
  (void) c;
}
void mg_tls_free(struct mg_connection *c) {
  (void) c;
}
long mg_tls_recv(struct mg_connection *c, void *buf, size_t len) {
  return c == NULL || buf == NULL || len == 0 ? 0 : -1;
}
long mg_tls_send(struct mg_connection *c, const void *buf, size_t len) {
  return c == NULL || buf == NULL || len == 0 ? 0 : -1;
}
size_t mg_tls_pending(struct mg_connection *c) {
  (void) c;
  return 0;
}
#endif

#ifdef MG_ENABLE_LINES
#line 1 "src/tls_mbed.c"
#endif




#if MG_ENABLE_MBEDTLS

#if defined(MBEDTLS_VERSION_NUMBER) && MBEDTLS_VERSION_NUMBER >= 0x03000000
#define MGRNG , rng_get, NULL
#else
#define MGRNG
#endif

void mg_tls_free(struct mg_connection *c) {
  struct mg_tls *tls = (struct mg_tls *) c->tls;
  if (tls != NULL) {
    free(tls->cafile);
    mbedtls_ssl_free(&tls->ssl);
    mbedtls_pk_free(&tls->pk);
    mbedtls_x509_crt_free(&tls->ca);
    mbedtls_x509_crt_free(&tls->cert);
    mbedtls_ssl_config_free(&tls->conf);
    free(tls);
    c->tls = NULL;
  }
}

static int mg_net_send(void *ctx, const unsigned char *buf, size_t len) {
  long n = mg_io_send((struct mg_connection *) ctx, buf, len);
  MG_VERBOSE(("%lu n=%ld", ((struct mg_connection *) ctx)->id, n));
  if (n == MG_IO_WAIT) return MBEDTLS_ERR_SSL_WANT_WRITE;
  if (n == MG_IO_RESET) return MBEDTLS_ERR_NET_CONN_RESET;
  if (n == MG_IO_ERR) return MBEDTLS_ERR_NET_SEND_FAILED;
  return (int) n;
}

static int mg_net_recv(void *ctx, unsigned char *buf, size_t len) {
  long n = mg_io_recv((struct mg_connection *) ctx, buf, len);
  MG_VERBOSE(("%lu n=%ld", ((struct mg_connection *) ctx)->id, n));
  if (n == MG_IO_WAIT) return MBEDTLS_ERR_SSL_WANT_WRITE;
  if (n == MG_IO_RESET) return MBEDTLS_ERR_NET_CONN_RESET;
  if (n == MG_IO_ERR) return MBEDTLS_ERR_NET_RECV_FAILED;
  return (int) n;
}

void mg_tls_handshake(struct mg_connection *c) {
  struct mg_tls *tls = (struct mg_tls *) c->tls;
  int rc = mbedtls_ssl_handshake(&tls->ssl);
  if (rc == 0) {  // Success
    MG_DEBUG(("%lu success", c->id));
    c->is_tls_hs = 0;
    mg_call(c, MG_EV_TLS_HS, NULL);
  } else if (rc == MBEDTLS_ERR_SSL_WANT_READ ||
             rc == MBEDTLS_ERR_SSL_WANT_WRITE) {  // Still pending
    MG_VERBOSE(("%lu pending, %d%d %d (-%#x)", c->id, c->is_connecting,
                c->is_tls_hs, rc, -rc));
  } else {
    mg_error(c, "TLS handshake: -%#x", -rc);  // Error
  }
}

static int mbed_rng(void *ctx, unsigned char *buf, size_t len) {
  mg_random(buf, len);
  (void) ctx;
  return 0;
}

static void debug_cb(void *c, int lev, const char *s, int n, const char *s2) {
  n = (int) strlen(s2) - 1;
  MG_VERBOSE(("%lu %d %.*s", ((struct mg_connection *) c)->id, lev, n, s2));
  (void) s;
}

#if defined(MBEDTLS_VERSION_NUMBER) && MBEDTLS_VERSION_NUMBER >= 0x03000000
static int rng_get(void *p_rng, unsigned char *buf, size_t len) {
  (void) p_rng;
  mg_random(buf, len);
  return 0;
}
#endif

static struct mg_str mg_loadfile(struct mg_fs *fs, const char *path) {
  size_t n = 0;
  if (path[0] == '-') return mg_str(path);
  char *p = mg_file_read(fs, path, &n);
  return mg_str_n(p, n);
}

void mg_tls_init(struct mg_connection *c, const struct mg_tls_opts *opts) {
  struct mg_fs *fs = opts->fs == NULL ? &mg_fs_posix : opts->fs;
  struct mg_tls *tls = (struct mg_tls *) calloc(1, sizeof(*tls));
  int rc = 0;
  c->tls = tls;
  if (c->tls == NULL) {
    mg_error(c, "TLS OOM");
    goto fail;
  }
  MG_DEBUG(("%lu Setting TLS", c->id));
  mbedtls_ssl_init(&tls->ssl);
  mbedtls_ssl_config_init(&tls->conf);
  mbedtls_x509_crt_init(&tls->ca);
  mbedtls_x509_crt_init(&tls->cert);
  mbedtls_pk_init(&tls->pk);
  mbedtls_ssl_conf_dbg(&tls->conf, debug_cb, c);
#if defined(MG_MBEDTLS_DEBUG_LEVEL)
  mbedtls_debug_set_threshold(MG_MBEDTLS_DEBUG_LEVEL);
#endif
  if ((rc = mbedtls_ssl_config_defaults(
           &tls->conf,
           c->is_client ? MBEDTLS_SSL_IS_CLIENT : MBEDTLS_SSL_IS_SERVER,
           MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT)) != 0) {
    mg_error(c, "tls defaults %#x", -rc);
    goto fail;
  }
  mbedtls_ssl_conf_rng(&tls->conf, mbed_rng, c);
  if (opts->ca == NULL || strcmp(opts->ca, "*") == 0) {
    mbedtls_ssl_conf_authmode(&tls->conf, MBEDTLS_SSL_VERIFY_NONE);
  } else if (opts->ca != NULL && opts->ca[0] != '\0') {
#if defined(MBEDTLS_X509_CA_CHAIN_ON_DISK)
    tls->cafile = strdup(opts->ca);
    rc = mbedtls_ssl_conf_ca_chain_file(&tls->conf, tls->cafile, NULL);
    if (rc != 0) {
      mg_error(c, "parse on-disk chain(%s) err %#x", tls->cafile, -rc);
      goto fail;
    }
#else
    struct mg_str s = mg_loadfile(fs, opts->ca);
    rc = mbedtls_x509_crt_parse(&tls->ca, (uint8_t *) s.ptr, s.len + 1);
    if (opts->ca[0] != '-') free((char *) s.ptr);
    if (rc != 0) {
      mg_error(c, "parse(%s) err %#x", opts->ca, -rc);
      goto fail;
    }
    mbedtls_ssl_conf_ca_chain(&tls->conf, &tls->ca, NULL);
#endif
    if (opts->srvname.len > 0) {
      char *x = mg_mprintf("%.*s", (int) opts->srvname.len, opts->srvname.ptr);
      mbedtls_ssl_set_hostname(&tls->ssl, x);
      free(x);
    }
    mbedtls_ssl_conf_authmode(&tls->conf, MBEDTLS_SSL_VERIFY_REQUIRED);
  }
  if (opts->cert != NULL && opts->cert[0] != '\0') {
    struct mg_str s = mg_loadfile(fs, opts->cert);
    const char *key = opts->certkey == NULL ? opts->cert : opts->certkey;
    rc = mbedtls_x509_crt_parse(&tls->cert, (uint8_t *) s.ptr, s.len + 1);
    if (opts->cert[0] != '-') free((char *) s.ptr);
    if (rc != 0) {
      mg_error(c, "parse(%s) err %#x", opts->cert, -rc);
      goto fail;
    }
    s = mg_loadfile(fs, key);
    rc = mbedtls_pk_parse_key(&tls->pk, (uint8_t *) s.ptr, s.len + 1, NULL,
                              0 MGRNG);
    if (key[0] != '-') free((char *) s.ptr);
    if (rc != 0) {
      mg_error(c, "tls key(%s) %#x", key, -rc);
      goto fail;
    }
    rc = mbedtls_ssl_conf_own_cert(&tls->conf, &tls->cert, &tls->pk);
    if (rc != 0) {
      mg_error(c, "own cert %#x", -rc);
      goto fail;
    }
  }
  if ((rc = mbedtls_ssl_setup(&tls->ssl, &tls->conf)) != 0) {
    mg_error(c, "setup err %#x", -rc);
    goto fail;
  }
  c->tls = tls;
  c->is_tls = 1;
  c->is_tls_hs = 1;
  mbedtls_ssl_set_bio(&tls->ssl, c, mg_net_send, mg_net_recv, 0);
  if (c->is_client && c->is_resolving == 0 && c->is_connecting == 0) {
    mg_tls_handshake(c);
  }
  return;
fail:
  mg_tls_free(c);
}

size_t mg_tls_pending(struct mg_connection *c) {
  struct mg_tls *tls = (struct mg_tls *) c->tls;
  return tls == NULL ? 0 : mbedtls_ssl_get_bytes_avail(&tls->ssl);
}

long mg_tls_recv(struct mg_connection *c, void *buf, size_t len) {
  struct mg_tls *tls = (struct mg_tls *) c->tls;
  long n = mbedtls_ssl_read(&tls->ssl, (unsigned char *) buf, len);
  if (n == MBEDTLS_ERR_SSL_WANT_READ || n == MBEDTLS_ERR_SSL_WANT_WRITE)
    return MG_IO_WAIT;
  if (n <= 0) return MG_IO_ERR;
  return n;
}

long mg_tls_send(struct mg_connection *c, const void *buf, size_t len) {
  struct mg_tls *tls = (struct mg_tls *) c->tls;
  long n = mbedtls_ssl_write(&tls->ssl, (unsigned char *) buf, len);
  if (n == MBEDTLS_ERR_SSL_WANT_READ || n == MBEDTLS_ERR_SSL_WANT_WRITE)
    return MG_IO_WAIT;
  if (n <= 0) return MG_IO_ERR;
  return n;
}
#endif

#ifdef MG_ENABLE_LINES
#line 1 "src/tls_openssl.c"
#endif



#if MG_ENABLE_OPENSSL
static int mg_tls_err(struct mg_tls *tls, int res) {
  int err = SSL_get_error(tls->ssl, res);
  // We've just fetched the last error from the queue.
  // Now we need to clear the error queue. If we do not, then the following
  // can happen (actually reported):
  //  - A new connection is accept()-ed with cert error (e.g. self-signed cert)
  //  - Since all accept()-ed connections share listener's context,
  //  - *ALL* SSL accepted connection report read error on the next poll cycle.
  //    Thus a single errored connection can close all the rest, unrelated ones.
  // Clearing the error keeps the shared SSL_CTX in an OK state.

  if (err != 0) ERR_print_errors_fp(stderr);
  ERR_clear_error();
  if (err == SSL_ERROR_WANT_READ) return 0;
  if (err == SSL_ERROR_WANT_WRITE) return 0;
  return err;
}

void mg_tls_init(struct mg_connection *c, const struct mg_tls_opts *opts) {
  struct mg_tls *tls = (struct mg_tls *) calloc(1, sizeof(*tls));
  const char *id = "mongoose";
  static unsigned char s_initialised = 0;
  int rc;

  if (tls == NULL) {
    mg_error(c, "TLS OOM");
    goto fail;
  }

  if (!s_initialised) {
    SSL_library_init();
    s_initialised++;
  }
  MG_DEBUG(("%lu Setting TLS, CA: %s, cert: %s, key: %s", c->id,
            opts->ca == NULL ? "null" : opts->ca,
            opts->cert == NULL ? "null" : opts->cert,
            opts->certkey == NULL ? "null" : opts->certkey));
  tls->ctx = c->is_client ? SSL_CTX_new(SSLv23_client_method())
                          : SSL_CTX_new(SSLv23_server_method());
  if ((tls->ssl = SSL_new(tls->ctx)) == NULL) {
    mg_error(c, "SSL_new");
    goto fail;
  }
  SSL_set_session_id_context(tls->ssl, (const uint8_t *) id,
                             (unsigned) strlen(id));
  // Disable deprecated protocols
  SSL_set_options(tls->ssl, SSL_OP_NO_SSLv2);
  SSL_set_options(tls->ssl, SSL_OP_NO_SSLv3);
  SSL_set_options(tls->ssl, SSL_OP_NO_TLSv1);
  SSL_set_options(tls->ssl, SSL_OP_NO_TLSv1_1);
#ifdef MG_ENABLE_OPENSSL_NO_COMPRESSION
  SSL_set_options(tls->ssl, SSL_OP_NO_COMPRESSION);
#endif
#ifdef MG_ENABLE_OPENSSL_CIPHER_SERVER_PREFERENCE
  SSL_set_options(tls->ssl, SSL_OP_CIPHER_SERVER_PREFERENCE);
#endif

  if (opts->ca != NULL && opts->ca[0] != '\0') {
    SSL_set_verify(tls->ssl, SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT,
                   NULL);
    if ((rc = SSL_CTX_load_verify_locations(tls->ctx, opts->ca, NULL)) != 1) {
      mg_error(c, "load('%s') %d err %d", opts->ca, rc, mg_tls_err(tls, rc));
      goto fail;
    }
  }
  if (opts->cert != NULL && opts->cert[0] != '\0') {
    const char *key = opts->certkey;
    if (key == NULL) key = opts->cert;
    if ((rc = SSL_use_certificate_file(tls->ssl, opts->cert, 1)) != 1) {
      mg_error(c, "Invalid SSL cert, err %d", mg_tls_err(tls, rc));
      goto fail;
    } else if ((rc = SSL_use_PrivateKey_file(tls->ssl, key, 1)) != 1) {
      mg_error(c, "Invalid SSL key, err %d", mg_tls_err(tls, rc));
      goto fail;
#if OPENSSL_VERSION_NUMBER > 0x10100000L
    } else if ((rc = SSL_use_certificate_chain_file(tls->ssl, opts->cert)) !=
               1) {
      mg_error(c, "Invalid chain, err %d", mg_tls_err(tls, rc));
      goto fail;
#endif
    } else {
      SSL_set_mode(tls->ssl, SSL_MODE_ACCEPT_MOVING_WRITE_BUFFER);
#if OPENSSL_VERSION_NUMBER > 0x10002000L
      SSL_set_ecdh_auto(tls->ssl, 1);
#endif
    }
  }
  if (opts->ciphers != NULL) SSL_set_cipher_list(tls->ssl, opts->ciphers);
#if OPENSSL_VERSION_NUMBER >= 0x10100000L
  if (opts->srvname.len > 0) {
    char *s = mg_mprintf("%.*s", (int) opts->srvname.len, opts->srvname.ptr);
    SSL_set1_host(tls->ssl, s);
    free(s);
  }
#endif
  c->tls = tls;
  c->is_tls = 1;
  c->is_tls_hs = 1;
  if (c->is_client && c->is_resolving == 0 && c->is_connecting == 0) {
    mg_tls_handshake(c);
  }
  MG_DEBUG(("%lu SSL %s OK", c->id, c->is_accepted ? "accept" : "client"));
  return;
fail:
  c->is_closing = 1;
  free(tls);
}

void mg_tls_handshake(struct mg_connection *c) {
  struct mg_tls *tls = (struct mg_tls *) c->tls;
  int rc;
  SSL_set_fd(tls->ssl, (int) (size_t) c->fd);
  rc = c->is_client ? SSL_connect(tls->ssl) : SSL_accept(tls->ssl);
  if (rc == 1) {
    MG_DEBUG(("%lu success", c->id));
    c->is_tls_hs = 0;
    mg_call(c, MG_EV_TLS_HS, NULL);
  } else {
    int code = mg_tls_err(tls, rc);
    if (code != 0) mg_error(c, "tls hs: rc %d, err %d", rc, code);
  }
}

void mg_tls_free(struct mg_connection *c) {
  struct mg_tls *tls = (struct mg_tls *) c->tls;
  if (tls == NULL) return;
  SSL_free(tls->ssl);
  SSL_CTX_free(tls->ctx);
  free(tls);
  c->tls = NULL;
}

size_t mg_tls_pending(struct mg_connection *c) {
  struct mg_tls *tls = (struct mg_tls *) c->tls;
  return tls == NULL ? 0 : (size_t) SSL_pending(tls->ssl);
}

long mg_tls_recv(struct mg_connection *c, void *buf, size_t len) {
  struct mg_tls *tls = (struct mg_tls *) c->tls;
  int n = SSL_read(tls->ssl, buf, (int) len);
  if (n < 0 && mg_tls_err(tls, n) == 0) return MG_IO_WAIT;
  if (n <= 0) return MG_IO_ERR;
  return n;
}

long mg_tls_send(struct mg_connection *c, const void *buf, size_t len) {
  struct mg_tls *tls = (struct mg_tls *) c->tls;
  int n = SSL_write(tls->ssl, buf, (int) len);
  if (n < 0 && mg_tls_err(tls, n) == 0) return MG_IO_WAIT;
  if (n <= 0) return MG_IO_ERR;
  return n;
}
#endif

#ifdef MG_ENABLE_LINES
#line 1 "src/url.c"
#endif


struct url {
  size_t key, user, pass, host, port, uri, end;
};

int mg_url_is_ssl(const char *url) {
  return strncmp(url, "wss:", 4) == 0 || strncmp(url, "https:", 6) == 0 ||
         strncmp(url, "mqtts:", 6) == 0 || strncmp(url, "ssl:", 4) == 0 ||
         strncmp(url, "tls:", 4) == 0;
}

static struct url urlparse(const char *url) {
  size_t i;
  struct url u;
  memset(&u, 0, sizeof(u));
  for (i = 0; url[i] != '\0'; i++) {
    if (url[i] == '/' && i > 0 && u.host == 0 && url[i - 1] == '/') {
      u.host = i + 1;
      u.port = 0;
    } else if (url[i] == ']') {
      u.port = 0;  // IPv6 URLs, like http://[::1]/bar
    } else if (url[i] == ':' && u.port == 0 && u.uri == 0) {
      u.port = i + 1;
    } else if (url[i] == '@' && u.user == 0 && u.pass == 0 && u.uri == 0) {
      u.user = u.host;
      u.pass = u.port;
      u.host = i + 1;
      u.port = 0;
    } else if (url[i] == '/' && u.host && u.uri == 0) {
      u.uri = i;
    }
  }
  u.end = i;
#if 0
  printf("[%s] %d %d %d %d %d\n", url, u.user, u.pass, u.host, u.port, u.uri);
#endif
  return u;
}

struct mg_str mg_url_host(const char *url) {
  struct url u = urlparse(url);
  size_t n = u.port  ? u.port - u.host - 1
             : u.uri ? u.uri - u.host
                     : u.end - u.host;
  struct mg_str s = mg_str_n(url + u.host, n);
  return s;
}

const char *mg_url_uri(const char *url) {
  struct url u = urlparse(url);
  return u.uri ? url + u.uri : "/";
}

unsigned short mg_url_port(const char *url) {
  struct url u = urlparse(url);
  unsigned short port = 0;
  if (strncmp(url, "http:", 5) == 0 || strncmp(url, "ws:", 3) == 0) port = 80;
  if (strncmp(url, "wss:", 4) == 0 || strncmp(url, "https:", 6) == 0)
    port = 443;
  if (strncmp(url, "mqtt:", 5) == 0) port = 1883;
  if (strncmp(url, "mqtts:", 6) == 0) port = 8883;
  if (u.port) port = (unsigned short) atoi(url + u.port);
  return port;
}

struct mg_str mg_url_user(const char *url) {
  struct url u = urlparse(url);
  struct mg_str s = mg_str("");
  if (u.user && (u.pass || u.host)) {
    size_t n = u.pass ? u.pass - u.user - 1 : u.host - u.user - 1;
    s = mg_str_n(url + u.user, n);
  }
  return s;
}

struct mg_str mg_url_pass(const char *url) {
  struct url u = urlparse(url);
  struct mg_str s = mg_str_n("", 0UL);
  if (u.pass && u.host) {
    size_t n = u.host - u.pass - 1;
    s = mg_str_n(url + u.pass, n);
  }
  return s;
}

#ifdef MG_ENABLE_LINES
#line 1 "src/util.c"
#endif


#if MG_ENABLE_CUSTOM_RANDOM
#else
void mg_random(void *buf, size_t len) {
  bool done = false;
  unsigned char *p = (unsigned char *) buf;
#if MG_ARCH == MG_ARCH_ESP32
  while (len--) *p++ = (unsigned char) (esp_random() & 255);
  done = true;
#elif MG_ARCH == MG_ARCH_WIN32
#elif MG_ARCH == MG_ARCH_UNIX
  FILE *fp = fopen("/dev/urandom", "rb");
  if (fp != NULL) {
    if (fread(buf, 1, len, fp) == len) done = true;
    fclose(fp);
  }
#endif
  // If everything above did not work, fallback to a pseudo random generator
  while (!done && len--) *p++ = (unsigned char) (rand() & 255);
}
#endif

char *mg_random_str(char *buf, size_t len) {
  size_t i;
  mg_random(buf, len);
  for (i = 0; i < len; i++) {
    uint8_t c = ((uint8_t *) buf)[i] % 62U;
    buf[i] = i == len - 1 ? (char) '\0'            // 0-terminate last byte
             : c < 26     ? (char) ('a' + c)       // lowercase
             : c < 52     ? (char) ('A' + c - 26)  // uppercase
                          : (char) ('0' + c - 52);     // numeric
  }
  return buf;
}

uint32_t mg_ntohl(uint32_t net) {
  uint8_t data[4] = {0, 0, 0, 0};
  memcpy(&data, &net, sizeof(data));
  return (((uint32_t) data[3]) << 0) | (((uint32_t) data[2]) << 8) |
         (((uint32_t) data[1]) << 16) | (((uint32_t) data[0]) << 24);
}

uint16_t mg_ntohs(uint16_t net) {
  uint8_t data[2] = {0, 0};
  memcpy(&data, &net, sizeof(data));
  return (uint16_t) ((uint16_t) data[1] | (((uint16_t) data[0]) << 8));
}

uint32_t mg_crc32(uint32_t crc, const char *buf, size_t len) {
  int i;
  crc = ~crc;
  while (len--) {
    crc ^= *(unsigned char *) buf++;
    for (i = 0; i < 8; i++) crc = crc & 1 ? (crc >> 1) ^ 0xedb88320 : crc >> 1;
  }
  return ~crc;
}

static int isbyte(int n) {
  return n >= 0 && n <= 255;
}

static int parse_net(const char *spec, uint32_t *net, uint32_t *mask) {
  int n, a, b, c, d, slash = 32, len = 0;
  if ((sscanf(spec, "%d.%d.%d.%d/%d%n", &a, &b, &c, &d, &slash, &n) == 5 ||
       sscanf(spec, "%d.%d.%d.%d%n", &a, &b, &c, &d, &n) == 4) &&
      isbyte(a) && isbyte(b) && isbyte(c) && isbyte(d) && slash >= 0 &&
      slash < 33) {
    len = n;
    *net = ((uint32_t) a << 24) | ((uint32_t) b << 16) | ((uint32_t) c << 8) |
           (uint32_t) d;
    *mask = slash ? (uint32_t) (0xffffffffU << (32 - slash)) : (uint32_t) 0;
  }
  return len;
}

int mg_check_ip_acl(struct mg_str acl, uint32_t remote_ip) {
  struct mg_str k, v;
  int allowed = acl.len == 0 ? '+' : '-';  // If any ACL is set, deny by default
  while (mg_commalist(&acl, &k, &v)) {
    uint32_t net, mask;
    if (k.ptr[0] != '+' && k.ptr[0] != '-') return -1;
    if (parse_net(&k.ptr[1], &net, &mask) == 0) return -2;
    if ((mg_ntohl(remote_ip) & mask) == net) allowed = k.ptr[0];
  }
  return allowed == '+';
}

#if MG_ENABLE_CUSTOM_MILLIS
#else
uint64_t mg_millis(void) {
#if MG_ARCH == MG_ARCH_WIN32
  return GetTickCount();
#elif MG_ARCH == MG_ARCH_RP2040
  return time_us_64() / 1000;
#elif MG_ARCH == MG_ARCH_ESP32
  return esp_timer_get_time() / 1000;
#elif MG_ARCH == MG_ARCH_ESP8266
  return xTaskGetTickCount() * portTICK_PERIOD_MS;
#elif MG_ARCH == MG_ARCH_FREERTOS_TCP || MG_ARCH == MG_ARCH_FREERTOS_LWIP
  return xTaskGetTickCount() * portTICK_PERIOD_MS;
#elif MG_ARCH == MG_ARCH_AZURERTOS
  return tx_time_get() * (1000 /* MS per SEC */ / TX_TIMER_TICKS_PER_SECOND);
#elif MG_ARCH == MG_ARCH_TIRTOS
  return (uint64_t) Clock_getTicks();
#elif MG_ARCH == MG_ARCH_ZEPHYR
  return (uint64_t) k_uptime_get();
#elif MG_ARCH == MG_ARCH_UNIX && defined(__APPLE__)
  // Apple CLOCK_MONOTONIC_RAW is equivalent to CLOCK_BOOTTIME on linux
  // Apple CLOCK_UPTIME_RAW is equivalent to CLOCK_MONOTONIC_RAW on linux
  return clock_gettime_nsec_np(CLOCK_UPTIME_RAW) / 1000000;
#elif MG_ARCH == MG_ARCH_UNIX
  struct timespec ts = {0, 0};
  // See #1615 - prefer monotonic clock
#if defined(CLOCK_MONOTONIC_RAW)
  // Raw hardware-based time that is not subject to NTP adjustment
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
#elif defined(CLOCK_MONOTONIC)
  // Affected by the incremental adjustments performed by adjtime and NTP
  clock_gettime(CLOCK_MONOTONIC, &ts);
#else
  // Affected by discontinuous jumps in the system time and by the incremental
  // adjustments performed by adjtime and NTP
  clock_gettime(CLOCK_REALTIME, &ts);
#endif
  return ((uint64_t) ts.tv_sec * 1000 + (uint64_t) ts.tv_nsec / 1000000);
#elif defined(ARDUINO)
  return (uint64_t) millis();
#else
  return (uint64_t) (time(NULL) * 1000);
#endif
}
#endif


#ifdef MG_ENABLE_LINES
#line 1 "src/ws.c"
#endif










struct ws_msg {
  uint8_t flags;
  size_t header_len;
  size_t data_len;
};

size_t mg_ws_vprintf(struct mg_connection *c, int op, const char *fmt,
                     va_list *ap) {
  size_t len = c->send.len;
  size_t n = mg_vxprintf(mg_pfn_iobuf, &c->send, fmt, ap);
  mg_ws_wrap(c, c->send.len - len, op);
  return n;
}

size_t mg_ws_printf(struct mg_connection *c, int op, const char *fmt, ...) {
  size_t len = 0;
  va_list ap;
  va_start(ap, fmt);
  len = mg_ws_vprintf(c, op, fmt, &ap);
  va_end(ap);
  return len;
}

static void ws_handshake(struct mg_connection *c, const struct mg_str *wskey,
                         const struct mg_str *wsproto, const char *fmt,
                         va_list *ap) {
  const char *magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  unsigned char sha[20], b64_sha[30];

  mg_sha1_ctx sha_ctx;
  mg_sha1_init(&sha_ctx);
  mg_sha1_update(&sha_ctx, (unsigned char *) wskey->ptr, wskey->len);
  mg_sha1_update(&sha_ctx, (unsigned char *) magic, 36);
  mg_sha1_final(sha, &sha_ctx);
  mg_base64_encode(sha, sizeof(sha), (char *) b64_sha);
  mg_xprintf(mg_pfn_iobuf, &c->send,
             "HTTP/1.1 101 Switching Protocols\r\n"
             "Upgrade: websocket\r\n"
             "Connection: Upgrade\r\n"
             "Sec-WebSocket-Accept: %s\r\n",
             b64_sha);
  if (fmt != NULL) mg_vxprintf(mg_pfn_iobuf, &c->send, fmt, ap);
  if (wsproto != NULL) {
    mg_printf(c, "Sec-WebSocket-Protocol: %.*s\r\n", (int) wsproto->len,
              wsproto->ptr);
  }
  mg_send(c, "\r\n", 2);
}

static uint32_t be32(const uint8_t *p) {
  return (((uint32_t) p[3]) << 0) | (((uint32_t) p[2]) << 8) |
         (((uint32_t) p[1]) << 16) | (((uint32_t) p[0]) << 24);
}

static size_t ws_process(uint8_t *buf, size_t len, struct ws_msg *msg) {
  size_t i, n = 0, mask_len = 0;
  memset(msg, 0, sizeof(*msg));
  if (len >= 2) {
    n = buf[1] & 0x7f;                // Frame length
    mask_len = buf[1] & 128 ? 4 : 0;  // last bit is a mask bit
    msg->flags = buf[0];
    if (n < 126 && len >= mask_len) {
      msg->data_len = n;
      msg->header_len = 2 + mask_len;
    } else if (n == 126 && len >= 4 + mask_len) {
      msg->header_len = 4 + mask_len;
      msg->data_len = (((size_t) buf[2]) << 8) | buf[3];
    } else if (len >= 10 + mask_len) {
      msg->header_len = 10 + mask_len;
      msg->data_len =
          (size_t) (((uint64_t) be32(buf + 2) << 32) + be32(buf + 6));
    }
  }
  // Sanity check, and integer overflow protection for the boundary check below
  // data_len should not be larger than 1 Gb
  if (msg->data_len > 1024 * 1024 * 1024) return 0;
  if (msg->header_len + msg->data_len > len) return 0;
  if (mask_len > 0) {
    uint8_t *p = buf + msg->header_len, *m = p - mask_len;
    for (i = 0; i < msg->data_len; i++) p[i] ^= m[i & 3];
  }
  return msg->header_len + msg->data_len;
}

static size_t mkhdr(size_t len, int op, bool is_client, uint8_t *buf) {
  size_t n = 0;
  buf[0] = (uint8_t) (op | 128);
  if (len < 126) {
    buf[1] = (unsigned char) len;
    n = 2;
  } else if (len < 65536) {
    uint16_t tmp = mg_htons((uint16_t) len);
    buf[1] = 126;
    memcpy(&buf[2], &tmp, sizeof(tmp));
    n = 4;
  } else {
    uint32_t tmp;
    buf[1] = 127;
    tmp = mg_htonl((uint32_t) (((uint64_t) len) >> 32));
    memcpy(&buf[2], &tmp, sizeof(tmp));
    tmp = mg_htonl((uint32_t) (len & 0xffffffffU));
    memcpy(&buf[6], &tmp, sizeof(tmp));
    n = 10;
  }
  if (is_client) {
    buf[1] |= 1 << 7;  // Set masking flag
    mg_random(&buf[n], 4);
    n += 4;
  }
  return n;
}

static void mg_ws_mask(struct mg_connection *c, size_t len) {
  if (c->is_client && c->send.buf != NULL) {
    size_t i;
    uint8_t *p = c->send.buf + c->send.len - len, *mask = p - 4;
    for (i = 0; i < len; i++) p[i] ^= mask[i & 3];
  }
}

size_t mg_ws_send(struct mg_connection *c, const void *buf, size_t len,
                  int op) {
  uint8_t header[14];
  size_t header_len = mkhdr(len, op, c->is_client, header);
  mg_send(c, header, header_len);
  MG_VERBOSE(("WS out: %d [%.*s]", (int) len, (int) len, buf));
  mg_send(c, buf, len);
  mg_ws_mask(c, len);
  return header_len + len;
}

static bool mg_ws_client_handshake(struct mg_connection *c) {
  int n = mg_http_get_request_len(c->recv.buf, c->recv.len);
  if (n < 0) {
    mg_error(c, "not http");  // Some just, not an HTTP request
  } else if (n > 0) {
    if (n < 15 || memcmp(c->recv.buf + 9, "101", 3) != 0) {
      mg_error(c, "handshake error");
    } else {
      struct mg_http_message hm;
      mg_http_parse((char *) c->recv.buf, c->recv.len, &hm);
      c->is_websocket = 1;
      mg_call(c, MG_EV_WS_OPEN, &hm);
    }
    mg_iobuf_del(&c->recv, 0, (size_t) n);
  } else {
    return true;  // Request is not yet received, quit event handler
  }
  return false;  // Continue event handler
}

static void mg_ws_cb(struct mg_connection *c, int ev, void *ev_data,
                     void *fn_data) {
  struct ws_msg msg;
  size_t ofs = (size_t) c->pfn_data;

  // assert(ofs < c->recv.len);
  if (ev == MG_EV_READ) {
    if (c->is_client && !c->is_websocket && mg_ws_client_handshake(c)) return;

    while (ws_process(c->recv.buf + ofs, c->recv.len - ofs, &msg) > 0) {
      char *s = (char *) c->recv.buf + ofs + msg.header_len;
      struct mg_ws_message m = {{s, msg.data_len}, msg.flags};
      size_t len = msg.header_len + msg.data_len;
      uint8_t final = msg.flags & 128, op = msg.flags & 15;
      // MG_VERBOSE ("fin %d op %d len %d [%.*s]", final, op,
      //                       (int) m.data.len, (int) m.data.len, m.data.ptr));
      switch (op) {
        case WEBSOCKET_OP_CONTINUE:
          mg_call(c, MG_EV_WS_CTL, &m);
          break;
        case WEBSOCKET_OP_PING:
          MG_DEBUG(("%s", "WS PONG"));
          mg_ws_send(c, s, msg.data_len, WEBSOCKET_OP_PONG);
          mg_call(c, MG_EV_WS_CTL, &m);
          break;
        case WEBSOCKET_OP_PONG:
          mg_call(c, MG_EV_WS_CTL, &m);
          break;
        case WEBSOCKET_OP_TEXT:
        case WEBSOCKET_OP_BINARY:
          if (final) mg_call(c, MG_EV_WS_MSG, &m);
          break;
        case WEBSOCKET_OP_CLOSE:
          MG_DEBUG(("%lu WS CLOSE", c->id));
          mg_call(c, MG_EV_WS_CTL, &m);
          // Echo the payload of the received CLOSE message back to the sender
          mg_ws_send(c, m.data.ptr, m.data.len, WEBSOCKET_OP_CLOSE);
          c->is_draining = 1;
          break;
        default:
          // Per RFC6455, close conn when an unknown op is recvd
          mg_error(c, "unknown WS op %d", op);
          break;
      }

      // Handle fragmented frames: strip header, keep in c->recv
      if (final == 0 || op == 0) {
        if (op) ofs++, len--, msg.header_len--;       // First frame
        mg_iobuf_del(&c->recv, ofs, msg.header_len);  // Strip header
        len -= msg.header_len;
        ofs += len;
        c->pfn_data = (void *) ofs;
        // MG_INFO(("FRAG %d [%.*s]", (int) ofs, (int) ofs, c->recv.buf));
      }
      // Remove non-fragmented frame
      if (final && op) mg_iobuf_del(&c->recv, ofs, len);
      // Last chunk of the fragmented frame
      if (final && !op) {
        m.flags = c->recv.buf[0];
        m.data = mg_str_n((char *) &c->recv.buf[1], (size_t) (ofs - 1));
        mg_call(c, MG_EV_WS_MSG, &m);
        mg_iobuf_del(&c->recv, 0, ofs);
        ofs = 0;
        c->pfn_data = NULL;
      }
    }
  }
  (void) fn_data;
  (void) ev_data;
}

struct mg_connection *mg_ws_connect(struct mg_mgr *mgr, const char *url,
                                    mg_event_handler_t fn, void *fn_data,
                                    const char *fmt, ...) {
  struct mg_connection *c = mg_connect(mgr, url, fn, fn_data);
  if (c != NULL) {
    char nonce[16], key[30];
    struct mg_str host = mg_url_host(url);
    mg_random(nonce, sizeof(nonce));
    mg_base64_encode((unsigned char *) nonce, sizeof(nonce), key);
    mg_xprintf(mg_pfn_iobuf, &c->send,
               "GET %s HTTP/1.1\r\n"
               "Upgrade: websocket\r\n"
               "Host: %.*s\r\n"
               "Connection: Upgrade\r\n"
               "Sec-WebSocket-Version: 13\r\n"
               "Sec-WebSocket-Key: %s\r\n",
               mg_url_uri(url), (int) host.len, host.ptr, key);
    if (fmt != NULL) {
      va_list ap;
      va_start(ap, fmt);
      mg_vxprintf(mg_pfn_iobuf, &c->send, fmt, &ap);
      va_end(ap);
    }
    mg_xprintf(mg_pfn_iobuf, &c->send, "\r\n");
    c->pfn = mg_ws_cb;
    c->pfn_data = NULL;
  }
  return c;
}

void mg_ws_upgrade(struct mg_connection *c, struct mg_http_message *hm,
                   const char *fmt, ...) {
  struct mg_str *wskey = mg_http_get_header(hm, "Sec-WebSocket-Key");
  c->pfn = mg_ws_cb;
  c->pfn_data = NULL;
  if (wskey == NULL) {
    mg_http_reply(c, 426, "", "WS upgrade expected\n");
    c->is_draining = 1;
  } else {
    struct mg_str *wsproto = mg_http_get_header(hm, "Sec-WebSocket-Protocol");
    va_list ap;
    va_start(ap, fmt);
    ws_handshake(c, wskey, wsproto, fmt, &ap);
    va_end(ap);
    c->is_websocket = 1;
    c->is_resp = 0;
    mg_call(c, MG_EV_WS_OPEN, hm);
  }
}

size_t mg_ws_wrap(struct mg_connection *c, size_t len, int op) {
  uint8_t header[14], *p;
  size_t header_len = mkhdr(len, op, c->is_client, header);

  // NOTE: order of operations is important!
  mg_iobuf_add(&c->send, c->send.len, NULL, header_len);
  p = &c->send.buf[c->send.len - len];         // p points to data
  memmove(p, p - header_len, len);             // Shift data
  memcpy(p - header_len, header, header_len);  // Prepend header
  mg_ws_mask(c, len);                          // Mask data

  return c->send.len;
}

#ifdef MG_ENABLE_LINES
#line 1 "mip/driver_enc28j60.c"
#endif


#if MG_ENABLE_MIP

// Instruction set
enum { OP_RCR, OP_RBM, OP_WCR, OP_WBM, OP_BFS, OP_BFC, OP_SRC };

static uint8_t rd(struct mip_spi *spi, uint8_t op, uint8_t addr) {
  spi->begin(spi->spi);
  spi->txn(spi->spi, (uint8_t) ((op << 5) | (addr & 0x1f)));
  uint8_t value = spi->txn(spi->spi, 255);
  if (addr & 0x80) value = spi->txn(spi->spi, 255);
  spi->end(spi->spi);
  return value;
}

static bool mip_driver_enc28j60_init(uint8_t *mac, void *data) {
  (void) mac, (void) data;
  rd((struct mip_spi *) data, OP_SRC, 0x1f);
  return false;
}

static size_t mip_driver_enc28j60_tx(const void *buf, size_t len, void *data) {
  (void) buf, (void) len, (void) data;
  return 0;
}

static size_t mip_driver_enc28j60_rx(void *buf, size_t len, void *data) {
  (void) buf, (void) len, (void) data;
  return 0;
}

static bool mip_driver_enc28j60_up(void *data) {
  (void) data;
  return false;
}

struct mip_driver mip_driver_enc28j60 = {
    mip_driver_enc28j60_init, mip_driver_enc28j60_tx, mip_driver_enc28j60_rx,
    mip_driver_enc28j60_up, NULL};
#endif

#ifdef MG_ENABLE_LINES
#line 1 "mip/driver_stm32.c"
#endif


#if MG_ENABLE_MIP
struct stm32_eth {
  volatile uint32_t MACCR, MACFFR, MACHTHR, MACHTLR, MACMIIAR, MACMIIDR, MACFCR,
      MACVLANTR, RESERVED0[2], MACRWUFFR, MACPMTCSR, RESERVED1, MACDBGR, MACSR,
      MACIMR, MACA0HR, MACA0LR, MACA1HR, MACA1LR, MACA2HR, MACA2LR, MACA3HR,
      MACA3LR, RESERVED2[40], MMCCR, MMCRIR, MMCTIR, MMCRIMR, MMCTIMR,
      RESERVED3[14], MMCTGFSCCR, MMCTGFMSCCR, RESERVED4[5], MMCTGFCR,
      RESERVED5[10], MMCRFCECR, MMCRFAECR, RESERVED6[10], MMCRGUFCR,
      RESERVED7[334], PTPTSCR, PTPSSIR, PTPTSHR, PTPTSLR, PTPTSHUR, PTPTSLUR,
      PTPTSAR, PTPTTHR, PTPTTLR, RESERVED8, PTPTSSR, PTPPPSCR, RESERVED9[564],
      DMABMR, DMATPDR, DMARPDR, DMARDLAR, DMATDLAR, DMASR, DMAOMR, DMAIER,
      DMAMFBOCR, DMARSWTR, RESERVED10[8], DMACHTDR, DMACHRDR, DMACHTBAR,
      DMACHRBAR;
};
#define ETH ((struct stm32_eth *) (uintptr_t) 0x40028000)

#define BIT(x) ((uint32_t) 1 << (x))
#define ETH_PKT_SIZE 1540  // Max frame size
#define ETH_DESC_CNT 4     // Descriptors count
#define ETH_DS 4           // Descriptor size (words)

static uint32_t s_rxdesc[ETH_DESC_CNT][ETH_DS];      // RX descriptors
static uint32_t s_txdesc[ETH_DESC_CNT][ETH_DS];      // TX descriptors
static uint8_t s_rxbuf[ETH_DESC_CNT][ETH_PKT_SIZE];  // RX ethernet buffers
static uint8_t s_txbuf[ETH_DESC_CNT][ETH_PKT_SIZE];  // TX ethernet buffers
static void (*s_rx)(void *, size_t, void *);         // Recv callback
static void *s_rxdata;                               // Recv callback data
enum { PHY_ADDR = 0, PHY_BCR = 0, PHY_BSR = 1 };     // PHY constants

static inline void spin(volatile uint32_t count) {
  while (count--) (void) 0;
}

static uint32_t eth_read_phy(uint8_t addr, uint8_t reg) {
  ETH->MACMIIAR &= (7 << 2);
  ETH->MACMIIAR |= ((uint32_t) addr << 11) | ((uint32_t) reg << 6);
  ETH->MACMIIAR |= BIT(0);
  while (ETH->MACMIIAR & BIT(0)) spin(1);
  return ETH->MACMIIDR;
}

static void eth_write_phy(uint8_t addr, uint8_t reg, uint32_t val) {
  ETH->MACMIIDR = val;
  ETH->MACMIIAR &= (7 << 2);
  ETH->MACMIIAR |= ((uint32_t) addr << 11) | ((uint32_t) reg << 6) | BIT(1);
  ETH->MACMIIAR |= BIT(0);
  while (ETH->MACMIIAR & BIT(0)) spin(1);
}

static uint32_t get_hclk(void) {
  struct rcc {
    volatile uint32_t CR, PLLCFGR, CFGR;
  } *RCC = (struct rcc *) 0x40023800;
  uint32_t clk = 0, hsi = 16000000 /* 16 MHz */, hse = 8000000 /* 8MHz */;

  if (RCC->CFGR & (1 << 2)) {
    clk = hse;
  } else if (RCC->CFGR & (1 << 3)) {
    uint32_t vco, m, n, p;
    m = (RCC->PLLCFGR & (0x3f << 0)) >> 0;
    n = (RCC->PLLCFGR & (0x1ff << 6)) >> 6;
    p = (((RCC->PLLCFGR & (3 << 16)) >> 16) + 1) * 2;
    clk = (RCC->PLLCFGR & (1 << 22)) ? hse : hsi;
    vco = (uint32_t) ((uint64_t) clk * n / m);
    clk = vco / p;
  } else {
    clk = hsi;
  }
  uint32_t hpre = (RCC->CFGR & (15 << 4)) >> 4;
  if (hpre < 8) return clk;

  uint8_t ahbptab[8] = {1, 2, 3, 4, 6, 7, 8, 9};  // log2(div)
  return ((uint32_t) clk) >> ahbptab[hpre - 8];
}

//  Guess CR from HCLK. MDC clock is generated from HCLK (AHB); as per 802.3,
//  it must not exceed 2.5MHz As the AHB clock can be (and usually is) derived
//  from the HSI (internal RC), and it can go above specs, the datasheets
//  specify a range of frequencies and activate one of a series of dividers to
//  keep the MDC clock safely below 2.5MHz. We guess a divider setting based on
//  HCLK with a +5% drift. If the user uses a different clock from our
//  defaults, needs to set the macros on top Valid for STM32F74xxx/75xxx
//  (38.8.1) and STM32F42xxx/43xxx (33.8.1) (both 4.5% worst case drift)
static int guess_mdc_cr(void) {
  uint8_t crs[] = {2, 3, 0, 1, 4, 5};          // ETH->MACMIIAR::CR values
  uint8_t div[] = {16, 26, 42, 62, 102, 124};  // Respective HCLK dividers
  uint32_t hclk = get_hclk();                  // Guess system HCLK
  int result = -1;                             // Invalid CR value
  if (hclk < 25000000) {
    MG_ERROR(("HCLK too low"));
  } else {
    for (int i = 0; i < 6; i++) {
      if (hclk / div[i] <= 2375000UL /* 2.5MHz - 5% */) {
        result = crs[i];
        break;
      }
    }
    if (result < 0) MG_ERROR(("HCLK too high"));
  }
  MG_DEBUG(("HCLK: %u, CR: %d", hclk, result));
  return result;
}

static bool mip_driver_stm32_init(uint8_t *mac, void *userdata) {
  struct mip_driver_stm32 *d = (struct mip_driver_stm32 *) userdata;
  // Init RX descriptors
  for (int i = 0; i < ETH_DESC_CNT; i++) {
    s_rxdesc[i][0] = BIT(31);                            // Own
    s_rxdesc[i][1] = sizeof(s_rxbuf[i]) | BIT(14);       // 2nd address chained
    s_rxdesc[i][2] = (uint32_t) (uintptr_t) s_rxbuf[i];  // Point to data buffer
    s_rxdesc[i][3] =
        (uint32_t) (uintptr_t) s_rxdesc[(i + 1) % ETH_DESC_CNT];  // Chain
  }

  // Init TX descriptors
  for (int i = 0; i < ETH_DESC_CNT; i++) {
    s_txdesc[i][2] = (uint32_t) (uintptr_t) s_txbuf[i];  // Buf pointer
    s_txdesc[i][3] =
        (uint32_t) (uintptr_t) s_txdesc[(i + 1) % ETH_DESC_CNT];  // Chain
  }

  ETH->DMABMR |= BIT(0);                        // Software reset
  while ((ETH->DMABMR & BIT(0)) != 0) spin(1);  // Wait until done

  // Set MDC clock divider. If user told us the value, use it. Otherwise, guess
  int cr = (d == NULL || d->mdc_cr < 0) ? guess_mdc_cr() : d->mdc_cr;
  ETH->MACMIIAR = ((uint32_t)cr & 3) << 2;

  // NOTE(cpq): we do not use extended descriptor bit 7, and do not use
  // hardware checksum. Therefore, descriptor size is 4, not 8
  // ETH->DMABMR = BIT(13) | BIT(16) | BIT(22) | BIT(23) | BIT(25);
  ETH->MACIMR = BIT(3) | BIT(9);                    // Mask timestamp & PMT IT
  ETH->MACFCR = BIT(7);                             // Disable zero quarta pause
  ETH->MACFFR = BIT(31);                            // Receive all
  eth_write_phy(PHY_ADDR, PHY_BCR, BIT(15));        // Reset PHY
  eth_write_phy(PHY_ADDR, PHY_BCR, BIT(12));        // Set autonegotiation
  ETH->DMARDLAR = (uint32_t) (uintptr_t) s_rxdesc;  // RX descriptors
  ETH->DMATDLAR = (uint32_t) (uintptr_t) s_txdesc;  // RX descriptors
  ETH->DMAIER = BIT(6) | BIT(16);                   // RIE, NISE
  ETH->MACCR = BIT(2) | BIT(3) | BIT(11) | BIT(14);    // RE, TE, Duplex, Fast
  ETH->DMAOMR = BIT(1) | BIT(13) | BIT(21) | BIT(25);  // SR, ST, TSF, RSF

  // MAC address filtering
  ETH->MACA0HR = ((uint32_t) mac[5] << 8U) | mac[4];
  ETH->MACA0LR = (uint32_t) (mac[3] << 24) | ((uint32_t) mac[2] << 16) |
                 ((uint32_t) mac[1] << 8) | mac[0];
  return true;
}

static void mip_driver_stm32_setrx(void (*rx)(void *, size_t, void *),
                                   void *rxdata) {
  s_rx = rx;
  s_rxdata = rxdata;
}

static uint32_t s_txno;
static size_t mip_driver_stm32_tx(const void *buf, size_t len, void *userdata) {
  if (len > sizeof(s_txbuf[s_txno])) {
    printf("%s: frame too big, %ld\n", __func__, (long) len);
    len = 0;  // Frame is too big
  } else if ((s_txdesc[s_txno][0] & BIT(31))) {
    printf("%s: no free descr\n", __func__);
    len = 0;  // All descriptors are busy, fail
  } else {
    memcpy(s_txbuf[s_txno], buf, len);     // Copy data
    s_txdesc[s_txno][1] = (uint32_t) len;  // Set data len
    s_txdesc[s_txno][0] = BIT(20) | BIT(28) | BIT(29) | BIT(30);  // Chain,FS,LS
    s_txdesc[s_txno][0] |= BIT(31);  // Set OWN bit - let DMA take over
    if (++s_txno >= ETH_DESC_CNT) s_txno = 0;
  }
  uint32_t sr = ETH->DMASR;
  if (sr & BIT(2)) ETH->DMASR = BIT(2), ETH->DMATPDR = 0;  // Resume
  if (sr & BIT(5)) ETH->DMASR = BIT(5), ETH->DMATPDR = 0;  // if busy
  if (len == 0) printf("E: D0 %lx SR %lx\n", (long) s_txdesc[0][0], (long) sr);
  return len;
  (void) userdata;
}

static bool mip_driver_stm32_up(void *userdata) {
  uint32_t bsr = eth_read_phy(PHY_ADDR, PHY_BSR);
  (void) userdata;
  return bsr & BIT(2) ? 1 : 0;
}

void ETH_IRQHandler(void);
void ETH_IRQHandler(void) {
  qp_mark(QP_IRQTRIGGERED, 0);
  volatile uint32_t sr = ETH->DMASR;
  if (sr & BIT(6)) {  // Frame received, loop
    for (uint32_t i = 0; i < ETH_DESC_CNT; i++) {
      if (s_rxdesc[i][0] & BIT(31)) continue;
      uint32_t len = ((s_rxdesc[i][0] >> 16) & (BIT(14) - 1));
      //    printf("%lx %lu %lx %lx\n", i, len, s_rxdesc[i][0], sr);
      if (s_rx != NULL) s_rx(s_rxbuf[i], len > 4 ? len - 4 : len, s_rxdata);
      s_rxdesc[i][0] = BIT(31);
    }
  }
  if (sr & BIT(7)) ETH->DMARPDR = 0;     // Resume RX
  ETH->DMASR = sr & ~(BIT(2) | BIT(7));  // Clear status
}

struct mip_driver mip_driver_stm32 = {
    mip_driver_stm32_init, mip_driver_stm32_tx, NULL, mip_driver_stm32_up,
    mip_driver_stm32_setrx};
#endif

#ifdef MG_ENABLE_LINES
#line 1 "mip/driver_w5500.c"
#endif


#if MG_ENABLE_MIP

enum { W5500_CR = 0, W5500_S0 = 1, W5500_TX0 = 2, W5500_RX0 = 3 };

static void w5500_txn(struct mip_spi *s, uint8_t block, uint16_t addr, bool wr,
                      void *buf, size_t len) {
  uint8_t *p = (uint8_t *) buf;
  uint8_t cmd[] = {(uint8_t) (addr >> 8), (uint8_t) (addr & 255),
                   (uint8_t) ((block << 3) | (wr ? 4 : 0))};
  s->begin(s->spi);
  for (size_t i = 0; i < sizeof(cmd); i++) s->txn(s->spi, cmd[i]);
  for (size_t i = 0; i < len; i++) {
    uint8_t r = s->txn(s->spi, p[i]);
    if (!wr) p[i] = r;
  }
  s->end(s->spi);
}

// clang-format off
static  void w5500_wn(struct mip_spi *s, uint8_t block, uint16_t addr, void *buf, size_t len) { w5500_txn(s, block, addr, true, buf, len); }
static  void w5500_w1(struct mip_spi *s, uint8_t block, uint16_t addr, uint8_t val) { w5500_wn(s, block, addr, &val, 1); }
static  void w5500_w2(struct mip_spi *s, uint8_t block, uint16_t addr, uint16_t val) { uint8_t buf[2] = {(uint8_t) (val >> 8), (uint8_t) (val & 255)}; w5500_wn(s, block, addr, buf, sizeof(buf)); }
static  void w5500_rn(struct mip_spi *s, uint8_t block, uint16_t addr, void *buf, size_t len) { w5500_txn(s, block, addr, false, buf, len); }
static  uint8_t w5500_r1(struct mip_spi *s, uint8_t block, uint16_t addr) { uint8_t r = 0; w5500_rn(s, block, addr, &r, 1); return r; }
static  uint16_t w5500_r2(struct mip_spi *s, uint8_t block, uint16_t addr) { uint8_t buf[2] = {0, 0}; w5500_rn(s, block, addr, buf, sizeof(buf)); return (uint16_t) ((buf[0] << 8) | buf[1]); }
// clang-format on

static size_t w5500_rx(void *buf, size_t buflen, void *data) {
  struct mip_spi *s = (struct mip_spi *) data;
  uint16_t r = 0, n = 0, len = (uint16_t) buflen, n2;     // Read recv len
  while ((n2 = w5500_r2(s, W5500_S0, 0x26)) > n) n = n2;  // Until it is stable
  // printf("RSR: %d\n", (int) n);
  if (n > 0) {
    uint16_t ptr = w5500_r2(s, W5500_S0, 0x28);  // Get read pointer
    n = w5500_r2(s, W5500_RX0, ptr);             // Read frame length
    if (n <= len + 2 && n > 1) {
      r = (uint16_t) (n - 2);
      w5500_rn(s, W5500_RX0, (uint16_t) (ptr + 2), buf, r);
    }
    w5500_w2(s, W5500_S0, 0x28, (uint16_t) (ptr + n));  // Advance read pointer
    w5500_w1(s, W5500_S0, 1, 0x40);                     // Sock0 CR -> RECV
    // printf("  RX_RD: tot=%u n=%u r=%u\n", n2, n, r);
  }
  return r;
}

static size_t w5500_tx(const void *buf, size_t buflen, void *data) {
  struct mip_spi *s = (struct mip_spi *) data;
  uint16_t n = 0, len = (uint16_t) buflen;
  while (n < len) n = w5500_r2(s, W5500_S0, 0x20);      // Wait for space
  uint16_t ptr = w5500_r2(s, W5500_S0, 0x24);           // Get write pointer
  w5500_wn(s, W5500_TX0, ptr, (void *) buf, len);       // Write data
  w5500_w2(s, W5500_S0, 0x24, (uint16_t) (ptr + len));  // Advance write pointer
  w5500_w1(s, W5500_S0, 1, 0x20);                       // Sock0 CR -> SEND
  for (int i = 0; i < 40; i++) {
    uint8_t ir = w5500_r1(s, W5500_S0, 2);  // Read S0 IR
    if (ir == 0) continue;
    // printf("IR %d, len=%d, free=%d, ptr %d\n", ir, (int) len, (int) n, ptr);
    w5500_w1(s, W5500_S0, 2, ir);  // Write S0 IR: clear it!
    if (ir & 8) len = 0;           // Timeout. Report error
    if (ir & (16 | 8)) break;      // Stop on SEND_OK or timeout
  }
  return len;
}

static bool w5500_init(uint8_t *mac, void *data) {
  struct mip_spi *s = (struct mip_spi *) data;
  s->end(s->spi);
  w5500_w1(s, W5500_CR, 0, 0x80);     // Reset chip: CR -> 0x80
  w5500_w1(s, W5500_CR, 0x2e, 0);     // CR PHYCFGR -> reset
  w5500_w1(s, W5500_CR, 0x2e, 0xf8);  // CR PHYCFGR -> set
  // w5500_wn(s, W5500_CR, 9, s->mac, 6);      // Set source MAC
  w5500_w1(s, W5500_S0, 0x1e, 16);          // Sock0 RX buf size
  w5500_w1(s, W5500_S0, 0x1f, 16);          // Sock0 TX buf size
  w5500_w1(s, W5500_S0, 0, 4);              // Sock0 MR -> MACRAW
  w5500_w1(s, W5500_S0, 1, 1);              // Sock0 CR -> OPEN
  return w5500_r1(s, W5500_S0, 3) == 0x42;  // Sock0 SR == MACRAW
  (void) mac;
}

static bool w5500_up(void *data) {
  uint8_t phycfgr = w5500_r1((struct mip_spi *) data, W5500_CR, 0x2e);
  return phycfgr & 1;  // Bit 0 of PHYCFGR is LNK (0 - down, 1 - up)
}

struct mip_driver mip_driver_w5500 = {w5500_init, w5500_tx, w5500_rx, w5500_up,
                                      NULL};
#endif

#ifdef MG_ENABLE_LINES
#line 1 "mip/mip.c"
#endif


#if MG_ENABLE_MIP

#define MIP_ETHEMERAL_PORT 49152
#define U16(ptr) ((((uint16_t) (ptr)[0]) << 8) | (ptr)[1])
#define PDIFF(a, b) ((size_t) (((char *) (b)) - ((char *) (a))))

#ifndef MIP_ARP_ENTRIES
#define MIP_ARP_ENTRIES 5  // Number of ARP cache entries. Maximum 21
#endif

#ifndef MIP_QSIZE
#define MIP_QSIZE (16 * 1024)  // Queue size
#endif

#ifndef MIP_TCP_KEEPALIVE_MS
#define MIP_TCP_KEEPALIVE_MS 45000  // TCP keep-alive period, ms
#endif

#define MIP_ARP_CS (2 + 12 * MIP_ARP_ENTRIES)  // ARP cache size
#define MIP_TCP_ACK_MS 150                     // Timeout for ACKing

struct connstate {
  uint32_t seq, ack;           // TCP seq/ack counters
  uint64_t timer;              // TCP keep-alive / ACK timer
  uint8_t mac[6];              // Peer MAC address
  uint8_t ttype;               // Timer type. 0: ack, 1: keep-alive
#define MIP_TTYPE_KEEPALIVE 0  // Connection is idle for long, send keepalive
#define MIP_TTYPE_ACK 1        // Peer sent us data, we have to ack it soon
  uint8_t tmiss;               // Number of keep-alive misses
  struct mg_iobuf raw;         // For TLS only. Incoming raw data
};

struct str {
  uint8_t *buf;
  size_t len;
};

// Receive queue - single producer, single consumer queue.  Interrupt-based
// drivers copy received frames to the queue in interrupt context. mip_poll()
// function runs in event loop context, reads from the queue
struct queue {
  uint8_t *buf;
  size_t len;
  volatile size_t tail, head;
};

// Network interface
struct mip_if {
  uint8_t mac[6];             // MAC address. Must be set to a valid MAC
  uint32_t ip, mask, gw;      // IP address, mask, default gateway. Can be 0
  struct str rx;              // Output (TX) buffer
  struct str tx;              // Input (RX) buffer
  bool use_dhcp;              // Enable DCHP
  struct mip_driver *driver;  // Low level driver
  void *driver_data;          // Driver-specific data
  struct mg_mgr *mgr;         // Mongoose event manager

  // Internal state, user can use it but should not change it
  uint64_t now;                   // Current time
  uint64_t timer_1000ms;          // 1000 ms timer: for DHCP and link state
  uint64_t lease_expire;          // Lease expiration time
  uint8_t arp_cache[MIP_ARP_CS];  // Each entry is 12 bytes
  uint16_t eport;                 // Next ephemeral port
  uint16_t dropped;               // Number of dropped frames
  uint8_t state;                  // Current state
#define MIP_STATE_DOWN 0          // Interface is down
#define MIP_STATE_UP 1            // Interface is up
#define MIP_STATE_READY 2         // Interface is up and has IP
  struct queue queue;             // Receive queue
};

#pragma pack(push, 1)

struct lcp {
  uint8_t addr, ctrl, proto[2], code, id, len[2];
};

struct eth {
  uint8_t dst[6];  // Destination MAC address
  uint8_t src[6];  // Source MAC address
  uint16_t type;   // Ethernet type
};

struct ip {
  uint8_t ver;    // Version
  uint8_t tos;    // Unused
  uint16_t len;   // Length
  uint16_t id;    // Unused
  uint16_t frag;  // Fragmentation
  uint8_t ttl;    // Time to live
  uint8_t proto;  // Upper level protocol
  uint16_t csum;  // Checksum
  uint32_t src;   // Source IP
  uint32_t dst;   // Destination IP
};

struct ip6 {
  uint8_t ver;      // Version
  uint8_t opts[3];  // Options
  uint16_t len;     // Length
  uint8_t proto;    // Upper level protocol
  uint8_t ttl;      // Time to live
  uint8_t src[16];  // Source IP
  uint8_t dst[16];  // Destination IP
};

struct icmp {
  uint8_t type;
  uint8_t code;
  uint16_t csum;
};

struct arp {
  uint16_t fmt;    // Format of hardware address
  uint16_t pro;    // Format of protocol address
  uint8_t hlen;    // Length of hardware address
  uint8_t plen;    // Length of protocol address
  uint16_t op;     // Operation
  uint8_t sha[6];  // Sender hardware address
  uint32_t spa;    // Sender protocol address
  uint8_t tha[6];  // Target hardware address
  uint32_t tpa;    // Target protocol address
};

struct tcp {
  uint16_t sport;  // Source port
  uint16_t dport;  // Destination port
  uint32_t seq;    // Sequence number
  uint32_t ack;    // Acknowledgement number
  uint8_t off;     // Data offset
  uint8_t flags;   // TCP flags
#define TH_FIN 0x01
#define TH_SYN 0x02
#define TH_RST 0x04
#define TH_PUSH 0x08
#define TH_ACK 0x10
#define TH_URG 0x20
#define TH_ECE 0x40
#define TH_CWR 0x80
  uint16_t win;   // Window
  uint16_t csum;  // Checksum
  uint16_t urp;   // Urgent pointer
};

struct udp {
  uint16_t sport;  // Source port
  uint16_t dport;  // Destination port
  uint16_t len;    // UDP length
  uint16_t csum;   // UDP checksum
};

struct dhcp {
  uint8_t op, htype, hlen, hops;
  uint32_t xid;
  uint16_t secs, flags;
  uint32_t ciaddr, yiaddr, siaddr, giaddr;
  uint8_t hwaddr[208];
  uint32_t magic;
  uint8_t options[32];
};

#pragma pack(pop)

struct pkt {
  struct str raw;  // Raw packet data
  struct str pay;  // Payload data
  struct eth *eth;
  struct llc *llc;
  struct arp *arp;
  struct ip *ip;
  struct ip6 *ip6;
  struct icmp *icmp;
  struct tcp *tcp;
  struct udp *udp;
  struct dhcp *dhcp;
};

static void q_copyin(struct queue *q, const uint8_t *buf, size_t len,
                     size_t head) {
  size_t left = q->len - head;
  memcpy(&q->buf[head], buf, left < len ? left : len);
  if (left < len) memcpy(q->buf, &buf[left], len - left);
}

static void q_copyout(struct queue *q, uint8_t *buf, size_t len, size_t tail) {
  size_t left = q->len - tail;
  memcpy(buf, &q->buf[tail], left < len ? left : len);
  if (left < len) memcpy(&buf[left], q->buf, len - left);
}

static bool q_write(struct queue *q, const void *buf, size_t len) {
  bool success = false;
  size_t left = (q->len - q->head + q->tail - 1) % q->len;
  if (len + sizeof(size_t) <= left) {
    q_copyin(q, (uint8_t *) &len, sizeof(len), q->head);
    q_copyin(q, (uint8_t *) buf, len, (q->head + sizeof(size_t)) % q->len);
    q->head = (q->head + sizeof(len) + len) % q->len;
    success = true;
  }
  return success;
}

static inline size_t q_space(struct queue *q) {
  return q->tail > q->head ? q->tail - q->head : q->tail + (q->len - q->head);
}

static inline size_t q_avail(struct queue *q) {
  size_t n = 0;
  if (q->tail != q->head) q_copyout(q, (uint8_t *) &n, sizeof(n), q->tail);
  return n;
}

static size_t q_read(struct queue *q, void *buf) {
  size_t n = q_avail(q);
  if (n > 0) {
    q_copyout(q, (uint8_t *) buf, n, (q->tail + sizeof(n)) % q->len);
    q->tail = (q->tail + sizeof(n) + n) % q->len;
  }
  return n;
}

static struct str mkstr(void *buf, size_t len) {
  struct str str = {(uint8_t *) buf, len};
  return str;
}

static void mkpay(struct pkt *pkt, void *p) {
  pkt->pay = mkstr(p, (size_t) (&pkt->raw.buf[pkt->raw.len] - (uint8_t *) p));
}

static uint32_t csumup(uint32_t sum, const void *buf, size_t len) {
  const uint8_t *p = (const uint8_t *) buf;
  for (size_t i = 0; i < len; i++) sum += i & 1 ? p[i] : (uint32_t) (p[i] << 8);
  return sum;
}

static uint16_t csumfin(uint32_t sum) {
  while (sum >> 16) sum = (sum & 0xffff) + (sum >> 16);
  return mg_htons(~sum & 0xffff);
}

static uint16_t ipcsum(const void *buf, size_t len) {
  uint32_t sum = csumup(0, buf, len);
  return csumfin(sum);
}

// ARP cache is organised as a doubly linked list. A successful cache lookup
// moves an entry to the head of the list. New entries are added by replacing
// the last entry in the list with a new IP/MAC.
// ARP cache format: | prev | next | Entry0 | Entry1 | .... | EntryN |
// ARP entry format: | prev | next | IP (4bytes) | MAC (6bytes) |
// prev and next are 1-byte offsets in the cache, so cache size is max 256 bytes
// ARP entry size is 12 bytes
static void arp_cache_init(uint8_t *p, int n, int size) {
  for (int i = 0; i < n; i++) p[2 + i * size] = (uint8_t) (2 + (i - 1) * size);
  for (int i = 0; i < n; i++) p[3 + i * size] = (uint8_t) (2 + (i + 1) * size);
  p[0] = p[2] = (uint8_t) (2 + (n - 1) * size);
  p[1] = p[3 + (n - 1) * size] = 2;
}

#if 0
static inline void arp_cache_dump(const uint8_t *p) {
  MG_INFO(("ARP cache:"));
  for (uint8_t i = 0, j = p[1]; i < MIP_ARP_ENTRIES; i++, j = p[j + 1]) {
    MG_INFO(("  %d.%d.%d.%d -> %x:%x:%x:%x:%x:%x", p[j + 2], p[j + 3], p[j + 4],
             p[j + 5], p[j + 6], p[j + 7], p[j + 8], p[j + 9], p[j + 10],
             p[j + 11]));
  }
}
#endif

static uint8_t *arp_cache_find(struct mip_if *ifp, uint32_t ip) {
  uint8_t *p = ifp->arp_cache;
  if (ip == 0 || ip == 0xffffffffU) return NULL;
  for (uint8_t i = 0, j = p[1]; i < MIP_ARP_ENTRIES; i++, j = p[j + 1]) {
    if (memcmp(p + j + 2, &ip, sizeof(ip)) == 0) {
      p[1] = j, p[0] = p[j];  // Found entry! Point list head to us
      // MG_DEBUG(("ARP find: %#lx @ %x:%x:%x:%x:%x:%x", (long) ip, p[j + 6],
      //          p[j + 7], p[j + 8], p[j + 9], p[j + 10], p[j + 11]));
      return p + j + 6;  // And return MAC address
    }
  }
  return NULL;
}

static void arp_cache_add(struct mip_if *ifp, uint32_t ip, uint8_t mac[6]) {
  uint8_t *p = ifp->arp_cache;
  if (ip == 0 || ip == ~0U) return;             // Bad IP
  if (arp_cache_find(ifp, ip) != NULL) return;  // Already exists, do nothing
  memcpy(p + p[0] + 2, &ip, sizeof(ip));  // Replace last entry: IP address
  memcpy(p + p[0] + 6, mac, 6);           // And MAC address
  p[1] = p[0], p[0] = p[p[1]];            // Point list head to us
  MG_DEBUG(("ARP cache: added %#lx @ %x:%x:%x:%x:%x:%x", (long) mg_htonl(ip),
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]));
}

static size_t ether_output(struct mip_if *ifp, size_t len) {
  // size_t min = 64;  // Pad short frames to 64 bytes (minimum Ethernet size)
  // if (len < min) memset(ifp->tx.buf + len, 0, min - len), len = min;
  // mg_hexdump(ifp->tx.buf, len);
  return ifp->driver->tx(ifp->tx.buf, len, ifp->driver_data);
}

static void arp_ask(struct mip_if *ifp, uint32_t ip) {
  struct eth *eth = (struct eth *) ifp->tx.buf;
  struct arp *arp = (struct arp *) (eth + 1);
  memset(eth->dst, 255, sizeof(eth->dst));
  memcpy(eth->src, ifp->mac, sizeof(eth->src));
  eth->type = mg_htons(0x806);
  memset(arp, 0, sizeof(*arp));
  arp->fmt = mg_htons(1), arp->pro = mg_htons(0x800), arp->hlen = 6,
  arp->plen = 4;
  arp->op = mg_htons(1), arp->tpa = ip, arp->spa = ifp->ip;
  memcpy(arp->sha, ifp->mac, sizeof(arp->sha));
  ether_output(ifp, PDIFF(eth, arp + 1));
}

static size_t mg_print_ipv4(mg_pfn_t fn, void *fn_data, va_list *ap) {
  uint32_t ip = mg_ntohl(va_arg(*ap, uint32_t));
  return mg_xprintf(fn, fn_data, "%d.%d.%d.%d", ip >> 24, (ip >> 16) & 255,
                    (ip >> 8) & 255, ip & 255);
}

static void onstatechange(struct mip_if *ifp) {
  if (ifp->state == MIP_STATE_READY) {
    MG_INFO(("READY, IP: %M", mg_print_ipv4, ifp->ip));
    MG_INFO(("       GW: %M", mg_print_ipv4, ifp->gw));
    MG_INFO(("       Lease: %lld sec", (ifp->lease_expire - ifp->now) / 1000));
    arp_ask(ifp, ifp->gw);
  } else if (ifp->state == MIP_STATE_UP) {
    MG_ERROR(("Link up"));
  } else if (ifp->state == MIP_STATE_DOWN) {
    MG_ERROR(("Link down"));
  }
}

static struct ip *tx_ip(struct mip_if *ifp, uint8_t proto, uint32_t ip_src,
                        uint32_t ip_dst, size_t plen) {
  struct eth *eth = (struct eth *) ifp->tx.buf;
  struct ip *ip = (struct ip *) (eth + 1);
  uint8_t *mac = arp_cache_find(ifp, ip_dst);  // Dst IP in ARP cache ?
  if (!mac && (ip_dst & ifp->mask)) arp_ask(ifp, ip_dst);  // Same net, lookup
  if (!mac) mac = arp_cache_find(ifp, ifp->gw);            // Use gateway MAC
  if (mac) memcpy(eth->dst, mac, sizeof(eth->dst));        // Found? Use it
  if (!mac) memset(eth->dst, 255, sizeof(eth->dst));       // No? Use broadcast
  memcpy(eth->src, ifp->mac, sizeof(eth->src));  // TODO(cpq): ARP lookup
  eth->type = mg_htons(0x800);
  memset(ip, 0, sizeof(*ip));
  ip->ver = 0x45;   // Version 4, header length 5 words
  ip->frag = 0x40;  // Don't fragment
  ip->len = mg_htons((uint16_t) (sizeof(*ip) + plen));
  ip->ttl = 64;
  ip->proto = proto;
  ip->src = ip_src;
  ip->dst = ip_dst;
  ip->csum = ipcsum(ip, sizeof(*ip));
  return ip;
}

static void tx_udp(struct mip_if *ifp, uint32_t ip_src, uint16_t sport,
                   uint32_t ip_dst, uint16_t dport, const void *buf,
                   size_t len) {
  struct ip *ip = tx_ip(ifp, 17, ip_src, ip_dst, len + sizeof(struct udp));
  struct udp *udp = (struct udp *) (ip + 1);
  // MG_DEBUG(("UDP XX LEN %d %d", (int) len, (int) ifp->tx.len));
  udp->sport = sport;
  udp->dport = dport;
  udp->len = mg_htons((uint16_t) (sizeof(*udp) + len));
  udp->csum = 0;
  uint32_t cs = csumup(0, udp, sizeof(*udp));
  cs = csumup(cs, buf, len);
  cs = csumup(cs, &ip->src, sizeof(ip->src));
  cs = csumup(cs, &ip->dst, sizeof(ip->dst));
  cs += (uint32_t) (ip->proto + sizeof(*udp) + len);
  udp->csum = csumfin(cs);
  memmove(udp + 1, buf, len);
  // MG_DEBUG(("UDP LEN %d %d", (int) len, (int) ifp->frame_len));
  ether_output(ifp, sizeof(struct eth) + sizeof(*ip) + sizeof(*udp) + len);
}

static void tx_dhcp(struct mip_if *ifp, uint32_t src, uint32_t dst,
                    uint8_t *opts, size_t optslen) {
  struct dhcp dhcp = {1, 1, 6, 0, 0, 0, 0, 0, 0, 0, 0, {0}, 0, {0}};
  dhcp.magic = mg_htonl(0x63825363);
  memcpy(&dhcp.hwaddr, ifp->mac, sizeof(ifp->mac));
  memcpy(&dhcp.xid, ifp->mac + 2, sizeof(dhcp.xid));
  memcpy(&dhcp.options, opts, optslen);
  tx_udp(ifp, src, mg_htons(68), dst, mg_htons(67), &dhcp, sizeof(dhcp));
}

static void tx_dhcp_request(struct mip_if *ifp, uint32_t src, uint32_t dst) {
  uint8_t opts[] = {
      53, 1, 3,                 // Type: DHCP request
      55, 2, 1,   3,            // GW and mask
      12, 3, 'm', 'i', 'p',     // Host name: "mip"
      54, 4, 0,   0,   0,   0,  // DHCP server ID
      50, 4, 0,   0,   0,   0,  // Requested IP
      255                       // End of options
  };
  memcpy(opts + 14, &dst, sizeof(dst));
  memcpy(opts + 20, &src, sizeof(src));
  tx_dhcp(ifp, src, dst, opts, sizeof(opts));
}

static void tx_dhcp_discover(struct mip_if *ifp) {
  uint8_t opts[] = {
      53, 1, 1,     // Type: DHCP discover
      55, 2, 1, 3,  // Parameters: ip, mask
      255           // End of options
  };
  tx_dhcp(ifp, 0, 0xffffffff, opts, sizeof(opts));
  MG_DEBUG(("DHCP discover sent"));
}

static void rx_arp(struct mip_if *ifp, struct pkt *pkt) {
  if (pkt->arp->op == mg_htons(1) && pkt->arp->tpa == ifp->ip) {
    // ARP request. Make a response, then send
    struct eth *eth = (struct eth *) ifp->tx.buf;
    struct arp *arp = (struct arp *) (eth + 1);
    MG_DEBUG(("ARP op %d %#x %#x", mg_htons(arp->op), arp->spa, arp->tpa));
    memcpy(eth->dst, pkt->eth->src, sizeof(eth->dst));
    memcpy(eth->src, ifp->mac, sizeof(eth->src));
    eth->type = mg_htons(0x806);
    *arp = *pkt->arp;
    arp->op = mg_htons(2);
    memcpy(arp->tha, pkt->arp->sha, sizeof(pkt->arp->tha));
    memcpy(arp->sha, ifp->mac, sizeof(pkt->arp->sha));
    arp->tpa = pkt->arp->spa;
    arp->spa = ifp->ip;
    MG_DEBUG(("ARP response: we're %#lx", (long) mg_ntohl(ifp->ip)));
    ether_output(ifp, PDIFF(eth, arp + 1));
  } else if (pkt->arp->op == mg_htons(2)) {
    if (memcmp(pkt->arp->tha, ifp->mac, sizeof(pkt->arp->tha)) != 0) return;
    // MG_INFO(("ARP RESPONSE"));
    arp_cache_add(ifp, pkt->arp->spa, pkt->arp->sha);
  }
}

static void rx_icmp(struct mip_if *ifp, struct pkt *pkt) {
  // MG_DEBUG(("ICMP %d", (int) len));
  if (pkt->icmp->type == 8 && pkt->ip != NULL && pkt->ip->dst == ifp->ip) {
    size_t hlen = sizeof(struct eth) + sizeof(struct ip) + sizeof(struct icmp);
    size_t space = ifp->tx.len - hlen, plen = pkt->pay.len;
    if (plen > space) plen = space;
    struct ip *ip =
        tx_ip(ifp, 1, ifp->ip, pkt->ip->src, sizeof(struct icmp) + plen);
    struct icmp *icmp = (struct icmp *) (ip + 1);
    memset(icmp, 0, sizeof(*icmp));        // Set csum to 0
    memcpy(icmp + 1, pkt->pay.buf, plen);  // Copy RX payload to TX
    icmp->csum = ipcsum(icmp, sizeof(*icmp) + plen);
    ether_output(ifp, hlen + plen);
  }
}

static void rx_dhcp(struct mip_if *ifp, struct pkt *pkt) {
  uint32_t ip = 0, gw = 0, mask = 0;
  uint8_t *p = pkt->dhcp->options, *end = &pkt->raw.buf[pkt->raw.len];
  if (end < (uint8_t *) (pkt->dhcp + 1)) return;
  while (p + 1 < end && p[0] != 255) {  // Parse options
    if (p[0] == 1 && p[1] == sizeof(ifp->mask) && p + 6 < end) {  // Mask
      memcpy(&mask, p + 2, sizeof(mask));
    } else if (p[0] == 3 && p[1] == sizeof(ifp->gw) && p + 6 < end) {  // GW
      memcpy(&gw, p + 2, sizeof(gw));
      ip = pkt->dhcp->yiaddr;
    } else if (p[0] == 51 && p[1] == 4 && p + 6 < end) {  // Lease
      uint32_t lease = 0;
      memcpy(&lease, p + 2, sizeof(lease));
      ifp->lease_expire = ifp->now + mg_ntohl(lease) * 1000;
    }
    p += p[1] + 2;
  }
  if (ip && mask && gw && ifp->ip == 0) {
    arp_cache_add(ifp, pkt->dhcp->siaddr, ((struct eth *) pkt->raw.buf)->src);
    ifp->ip = ip, ifp->gw = gw, ifp->mask = mask;
    ifp->state = MIP_STATE_READY;
    onstatechange(ifp);
    tx_dhcp_request(ifp, ip, pkt->dhcp->siaddr);
  }
}

static struct mg_connection *getpeer(struct mg_mgr *mgr, struct pkt *pkt,
                                     bool lsn) {
  struct mg_connection *c = NULL;
  for (c = mgr->conns; c != NULL; c = c->next) {
    if (c->is_udp && pkt->udp && c->loc.port == pkt->udp->dport) break;
    if (!c->is_udp && pkt->tcp && c->loc.port == pkt->tcp->dport &&
        lsn == c->is_listening && (lsn || c->rem.port == pkt->tcp->sport))
      break;
  }
  return c;
}

static void rx_udp(struct mip_if *ifp, struct pkt *pkt) {
  struct mg_connection *c = getpeer(ifp->mgr, pkt, true);
  if (c == NULL) {
    // No UDP listener on this port. Should send ICMP, but keep silent.
  } else if (c != NULL) {
    c->rem.port = pkt->udp->sport;
    c->rem.ip = pkt->ip->src;
    if (c->recv.len >= MG_MAX_RECV_SIZE) {
      mg_error(c, "max_recv_buf_size reached");
    } else if (c->recv.size - c->recv.len < pkt->pay.len &&
               !mg_iobuf_resize(&c->recv, c->recv.len + pkt->pay.len)) {
      mg_error(c, "oom");
    } else {
      memcpy(&c->recv.buf[c->recv.len], pkt->pay.buf, pkt->pay.len);
      c->recv.len += pkt->pay.len;
      mg_call(c, MG_EV_READ, &pkt->pay.len);
    }
  }
}

static size_t tx_tcp(struct mip_if *ifp, uint32_t dst_ip, uint8_t flags,
                     uint16_t sport, uint16_t dport, uint32_t seq, uint32_t ack,
                     const void *buf, size_t len) {
  struct ip *ip = tx_ip(ifp, 6, ifp->ip, dst_ip, sizeof(struct tcp) + len);
  struct tcp *tcp = (struct tcp *) (ip + 1);
  memset(tcp, 0, sizeof(*tcp));
  memmove(tcp + 1, buf, len);
  tcp->sport = sport;
  tcp->dport = dport;
  tcp->seq = seq;
  tcp->ack = ack;
  tcp->flags = flags;
  tcp->win = mg_htons(8192);
  tcp->off = (uint8_t) (sizeof(*tcp) / 4 << 4);
  uint32_t cs = 0;
  uint16_t n = (uint16_t) (sizeof(*tcp) + len);
  uint8_t pseudo[] = {0, ip->proto, (uint8_t) (n >> 8), (uint8_t) (n & 255)};
  cs = csumup(cs, tcp, n);
  cs = csumup(cs, &ip->src, sizeof(ip->src));
  cs = csumup(cs, &ip->dst, sizeof(ip->dst));
  cs = csumup(cs, pseudo, sizeof(pseudo));
  tcp->csum = csumfin(cs);
  return ether_output(ifp, PDIFF(ifp->tx.buf, tcp + 1) + len);
}

static size_t tx_tcp_pkt(struct mip_if *ifp, struct pkt *pkt, uint8_t flags,
                         uint32_t seq, const void *buf, size_t len) {
  uint32_t delta = (pkt->tcp->flags & (TH_SYN | TH_FIN)) ? 1 : 0;
  return tx_tcp(ifp, pkt->ip->src, flags, pkt->tcp->dport, pkt->tcp->sport, seq,
                mg_htonl(mg_ntohl(pkt->tcp->seq) + delta), buf, len);
}

static struct mg_connection *accept_conn(struct mg_connection *lsn,
                                         struct pkt *pkt) {
  struct mg_connection *c = mg_alloc_conn(lsn->mgr);
  struct connstate *s = (struct connstate *) (c + 1);
  s->seq = mg_ntohl(pkt->tcp->ack), s->ack = mg_ntohl(pkt->tcp->seq);
  s->timer = ((struct mip_if *) c->mgr->priv)->now + MIP_TCP_KEEPALIVE_MS;
  c->rem.ip = pkt->ip->src;
  c->rem.port = pkt->tcp->sport;
  MG_DEBUG(("%lu accepted %lx:%hx", c->id, mg_ntohl(c->rem.ip), c->rem.port));
  LIST_ADD_HEAD(struct mg_connection, &lsn->mgr->conns, c);
  c->is_accepted = 1;
  c->is_hexdumping = lsn->is_hexdumping;
  c->pfn = lsn->pfn;
  c->loc = lsn->loc;
  c->pfn_data = lsn->pfn_data;
  c->fn = lsn->fn;
  c->fn_data = lsn->fn_data;
  mg_call(c, MG_EV_OPEN, NULL);
  mg_call(c, MG_EV_ACCEPT, NULL);
  return c;
}

static void settmout(struct mg_connection *c, uint8_t type) {
  struct mip_if *ifp = (struct mip_if *) c->mgr->priv;
  struct connstate *s = (struct connstate *) (c + 1);
  unsigned n = type == MIP_TTYPE_ACK ? MIP_TCP_ACK_MS : MIP_TCP_KEEPALIVE_MS;
  s->timer = ifp->now + n;
  s->ttype = type;
  MG_VERBOSE(("%lu %d -> %llx", c->id, type, s->timer));
}

long mg_io_send(struct mg_connection *c, const void *buf, size_t len) {
  struct mip_if *ifp = (struct mip_if *) c->mgr->priv;
  struct connstate *s = (struct connstate *) (c + 1);
  size_t max_headers_len = 14 + 24 /* max IP */ + 60 /* max TCP */;
  if (len + max_headers_len > ifp->tx.len) len = ifp->tx.len - max_headers_len;
  if (tx_tcp(ifp, c->rem.ip, TH_PUSH | TH_ACK, c->loc.port, c->rem.port,
             mg_htonl(s->seq), mg_htonl(s->ack), buf, len) > 0) {
    s->seq += (uint32_t) len;
    if (s->ttype == MIP_TTYPE_KEEPALIVE) settmout(c, MIP_TTYPE_KEEPALIVE);
  } else {
    return MG_IO_ERR;
  }
  return (long) len;
}

long mg_io_recv(struct mg_connection *c, void *buf, size_t len) {
  struct connstate *s = (struct connstate *) (c + 1);
  if (s->raw.len == 0) return MG_IO_WAIT;
  if (len > s->raw.len) len = s->raw.len;
  memcpy(buf, s->raw.buf, len);
  mg_iobuf_del(&s->raw, 0, len);
  MG_DEBUG(("%lu", len));
  return (long) len;
}

static void read_conn(struct mg_connection *c, struct pkt *pkt) {
  struct connstate *s = (struct connstate *) (c + 1);
  struct mg_iobuf *io = c->is_tls ? &s->raw : &c->recv;
  uint32_t seq = mg_ntohl(pkt->tcp->seq);
  s->raw.align = c->recv.align;
  if (pkt->tcp->flags & TH_FIN) {
    s->ack = mg_htonl(pkt->tcp->seq) + 1, s->seq = mg_htonl(pkt->tcp->ack);
    c->is_closing = 1;
  } else if (pkt->pay.len == 0) {
    // TODO(cpq): handle this peer's ACK
  } else if (seq != s->ack) {
    // TODO(cpq): peer sent us SEQ which we don't expect. Retransmit rather
    // than close this connection
    mg_error(c, "SEQ != ACK: %x %x", seq, s->ack);
  } else if (io->size - io->len < pkt->pay.len &&
             !mg_iobuf_resize(io, io->len + pkt->pay.len)) {
    mg_error(c, "oom");
  } else {
    // Copy TCP payload into the IO buffer. If the connection is plain text, we
    // copy to c->recv. If the connection is TLS, this data is encrypted,
    // therefore we copy that encrypted data to the s->raw iobuffer instead,
    // and then call mg_tls_recv() to decrypt it. NOTE: mg_tls_recv() will
    // call back mg_io_recv() which grabs raw data from s->raw
    memcpy(&io->buf[io->len], pkt->pay.buf, pkt->pay.len);
    io->len += pkt->pay.len;

    MG_DEBUG(("%lu SEQ %x -> %x", c->id, mg_htonl(pkt->tcp->seq), s->ack));
    s->ack = (uint32_t) (mg_htonl(pkt->tcp->seq) + pkt->pay.len);
#if 1
    // Send ACK immediately
    MG_DEBUG(("  imm ACK", c->id, mg_htonl(pkt->tcp->seq), s->ack));
    tx_tcp((struct mip_if *) c->mgr->priv, c->rem.ip, TH_ACK, c->loc.port,
           c->rem.port, mg_htonl(s->seq), mg_htonl(s->ack), "", 0);
#else
    // Advance ACK counter and setup a timer to send an ACK back
    settmout(c, MIP_TTYPE_ACK);
#endif

    if (c->is_tls) {
      // TLS connection. Make room for decrypted data in c->recv
      io = &c->recv;
      if (io->size - io->len < pkt->pay.len &&
          !mg_iobuf_resize(io, io->len + pkt->pay.len)) {
        mg_error(c, "oom");
      } else {
        // Decrypt data directly into c->recv
        long n = mg_tls_recv(c, &io->buf[io->len], io->size - io->len);
        if (n == MG_IO_ERR) {
          mg_error(c, "TLS recv error");
        } else if (n > 0) {
          // Decrypted successfully - trigger MG_EV_READ
          io->len += (size_t) n;
          mg_call(c, MG_EV_READ, &n);
        }
      }
    } else {
      // Plain text connection, data is already in c->recv, trigger MG_EV_READ
      mg_call(c, MG_EV_READ, &pkt->pay.len);
    }
  }
}

static void rx_tcp(struct mip_if *ifp, struct pkt *pkt) {
  struct mg_connection *c = getpeer(ifp->mgr, pkt, false);
  struct connstate *s = (struct connstate *) (c + 1);

  if (c != NULL && s->ttype == MIP_TTYPE_KEEPALIVE) {
    s->tmiss = 0;                      // Reset missed keep-alive counter
    settmout(c, MIP_TTYPE_KEEPALIVE);  // Advance keep-alive timer
  }
#if 0
  MG_INFO(("%lu %hhu %d", c ? c->id : 0, pkt->tcp->flags, (int) pkt->pay.len));
#endif
  if (c != NULL && c->is_connecting && pkt->tcp->flags & (TH_SYN | TH_ACK)) {
    s->seq = mg_ntohl(pkt->tcp->ack), s->ack = mg_ntohl(pkt->tcp->seq) + 1;
    tx_tcp_pkt(ifp, pkt, TH_ACK, pkt->tcp->ack, NULL, 0);
    c->is_connecting = 0;             // Client connected
    mg_call(c, MG_EV_CONNECT, NULL);  // Let user know
  } else if (c != NULL && c->is_connecting) {
    tx_tcp_pkt(ifp, pkt, TH_RST | TH_ACK, pkt->tcp->ack, NULL, 0);
  } else if (c != NULL) {
#if 0
    MG_DEBUG(("%lu %d %lx:%hu -> %lx:%hu", c->id, (int) pkt->raw.len,
              mg_ntohl(pkt->ip->src), mg_ntohs(pkt->tcp->sport),
              mg_ntohl(pkt->ip->dst), mg_ntohs(pkt->tcp->dport)));
    mg_hexdump(pkt->pay.buf, pkt->pay.len);
#endif
    read_conn(c, pkt);
  } else if ((c = getpeer(ifp->mgr, pkt, true)) == NULL) {
    tx_tcp_pkt(ifp, pkt, TH_RST | TH_ACK, pkt->tcp->ack, NULL, 0);
  } else if (pkt->tcp->flags & TH_SYN) {
    // Use peer's source port as ISN, in order to recognise the handshake
    uint32_t isn = mg_htonl((uint32_t) mg_ntohs(pkt->tcp->sport));
    tx_tcp_pkt(ifp, pkt, TH_SYN | TH_ACK, isn, NULL, 0);
  } else if (pkt->tcp->flags & TH_FIN) {
    tx_tcp_pkt(ifp, pkt, TH_FIN | TH_ACK, pkt->tcp->ack, NULL, 0);
  } else if (mg_htonl(pkt->tcp->ack) == mg_htons(pkt->tcp->sport) + 1U) {
    accept_conn(c, pkt);
  } else {
    // MG_DEBUG(("dropped silently.."));
  }
}

static void rx_ip(struct mip_if *ifp, struct pkt *pkt) {
  //  MG_DEBUG(("IP %d", (int) pkt->pay.len));
  if (pkt->ip->proto == 1) {
    pkt->icmp = (struct icmp *) (pkt->ip + 1);
    if (pkt->pay.len < sizeof(*pkt->icmp)) return;
    mkpay(pkt, pkt->icmp + 1);
    rx_icmp(ifp, pkt);
  } else if (pkt->ip->proto == 17) {
    pkt->udp = (struct udp *) (pkt->ip + 1);
    if (pkt->pay.len < sizeof(*pkt->udp)) return;
    // MG_DEBUG(("  UDP %u %u -> %u", len, mg_htons(udp->sport),
    // mg_htons(udp->dport)));
    mkpay(pkt, pkt->udp + 1);
    if (pkt->udp->dport == mg_htons(68)) {
      pkt->dhcp = (struct dhcp *) (pkt->udp + 1);
      mkpay(pkt, pkt->dhcp + 1);
      rx_dhcp(ifp, pkt);
    } else {
      rx_udp(ifp, pkt);
    }
  } else if (pkt->ip->proto == 6) {
    pkt->tcp = (struct tcp *) (pkt->ip + 1);
    if (pkt->pay.len < sizeof(*pkt->tcp)) return;
    mkpay(pkt, pkt->tcp + 1);
    uint16_t iplen = mg_ntohs(pkt->ip->len);
    uint16_t off = (uint16_t) (sizeof(*pkt->ip) + ((pkt->tcp->off >> 4) * 4U));
    if (iplen >= off) pkt->pay.len = (size_t) (iplen - off);
    rx_tcp(ifp, pkt);
  }
}

static void rx_ip6(struct mip_if *ifp, struct pkt *pkt) {
  // MG_DEBUG(("IP %d", (int) len));
  if (pkt->ip6->proto == 1 || pkt->ip6->proto == 58) {
    pkt->icmp = (struct icmp *) (pkt->ip6 + 1);
    if (pkt->pay.len < sizeof(*pkt->icmp)) return;
    mkpay(pkt, pkt->icmp + 1);
    rx_icmp(ifp, pkt);
  } else if (pkt->ip6->proto == 17) {
    pkt->udp = (struct udp *) (pkt->ip6 + 1);
    if (pkt->pay.len < sizeof(*pkt->udp)) return;
    // MG_DEBUG(("  UDP %u %u -> %u", len, mg_htons(udp->sport),
    // mg_htons(udp->dport)));
    mkpay(pkt, pkt->udp + 1);
  }
}

static void mip_rx(struct mip_if *ifp, void *buf, size_t len) {
  const uint8_t broadcast[] = {255, 255, 255, 255, 255, 255};
  // struct pkt pkt = {.raw = {.buf = (uint8_t *) buf, .len = len}};
  struct pkt pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.raw.buf = (uint8_t *) buf;
  pkt.raw.len = len;
  pkt.eth = (struct eth *) buf;
  if (pkt.raw.len < sizeof(*pkt.eth)) return;  // Truncated - runt?
  if (memcmp(pkt.eth->dst, ifp->mac, sizeof(pkt.eth->dst)) != 0 &&
      memcmp(pkt.eth->dst, broadcast, sizeof(pkt.eth->dst)) != 0) {
    // Not for us. Drop silently
  } else if (pkt.eth->type == mg_htons(0x806)) {
    pkt.arp = (struct arp *) (pkt.eth + 1);
    if (sizeof(*pkt.eth) + sizeof(*pkt.arp) > pkt.raw.len) return;  // Truncated
    rx_arp(ifp, &pkt);
  } else if (pkt.eth->type == mg_htons(0x86dd)) {
    pkt.ip6 = (struct ip6 *) (pkt.eth + 1);
    if (pkt.raw.len < sizeof(*pkt.eth) + sizeof(*pkt.ip6)) return;  // Truncated
    if ((pkt.ip6->ver >> 4) != 0x6) return;                         // Not IP
    mkpay(&pkt, pkt.ip6 + 1);
    rx_ip6(ifp, &pkt);
  } else if (pkt.eth->type == mg_htons(0x800)) {
    pkt.ip = (struct ip *) (pkt.eth + 1);
    if (pkt.raw.len < sizeof(*pkt.eth) + sizeof(*pkt.ip)) return;  // Truncated
    if ((pkt.ip->ver >> 4) != 4) return;                           // Not IP
    // MG_DEBUG(("%u %u", mg_ntohs(pkt.ip->len) + 14, pkt.raw.len));
    if ((size_t) mg_ntohs(pkt.ip->len) + 14 < pkt.raw.len) {
      pkt.raw.len -= pkt.raw.len - (mg_ntohs(pkt.ip->len) + 14);
    }
    mkpay(&pkt, pkt.ip + 1);
    rx_ip(ifp, &pkt);
  } else {
    MG_DEBUG(("  Unknown eth type %x", mg_htons(pkt.eth->type)));
  }
}

static void mip_poll(struct mip_if *ifp, uint64_t uptime_ms) {
  if (ifp == NULL || ifp->driver == NULL) return;
  bool expired_1000ms = mg_timer_expired(&ifp->timer_1000ms, 1000, uptime_ms);
  ifp->now = uptime_ms;

  // Handle physical interface up/down status
  if (expired_1000ms && ifp->driver->up) {
    bool up = ifp->driver->up(ifp->driver_data);
    bool current = ifp->state != MIP_STATE_DOWN;
    if (up != current) {
      ifp->state = up == false     ? MIP_STATE_DOWN
                   : ifp->use_dhcp ? MIP_STATE_UP
                                   : MIP_STATE_READY;
      if (!up && ifp->use_dhcp) ifp->ip = 0;
      onstatechange(ifp);
    }
  }
  if (ifp->state == MIP_STATE_DOWN) return;
  // if (expired_1000ms) arp_cache_dump(ifp->arp_cache);

  if (ifp->ip == 0 && expired_1000ms) {
    tx_dhcp_discover(ifp);  // If IP not configured, send DHCP
  } else if (ifp->use_dhcp == false && expired_1000ms &&
             arp_cache_find(ifp, ifp->gw) == NULL) {
    arp_ask(ifp, ifp->gw);  // If GW's MAC address in not in ARP cache
  }

  // Read data from the network
  for (;;) {
    size_t len = ifp->queue.len > 0 ? q_read(&ifp->queue, ifp->rx.buf)
                                    : ifp->driver->rx(ifp->rx.buf, ifp->rx.len,
                                                      ifp->driver_data);
    if (len == 0) break;
    qp_mark(QP_FRAMEPOPPED, (int) q_space(&ifp->queue));
    mip_rx(ifp, ifp->rx.buf, len);
    qp_mark(QP_FRAMEDONE, (int) q_space(&ifp->queue));
  }

  // Process timeouts
  for (struct mg_connection *c = ifp->mgr->conns; c != NULL; c = c->next) {
    if (c->is_udp || c->is_listening) continue;
    if (c->is_connecting || c->is_resolving) continue;
    struct connstate *s = (struct connstate *) (c + 1);
    if (uptime_ms > s->timer) {
      if (s->ttype == MIP_TTYPE_ACK) {
        MG_DEBUG(("%lu ack %x %x", c->id, s->seq, s->ack));
        tx_tcp(ifp, c->rem.ip, TH_ACK, c->loc.port, c->rem.port,
               mg_htonl(s->seq), mg_htonl(s->ack), "", 0);
      } else {
        MG_DEBUG(("%lu keepalive", c->id));
        tx_tcp(ifp, c->rem.ip, TH_ACK, c->loc.port, c->rem.port,
               mg_htonl(s->seq - 1), mg_htonl(s->ack), "", 0);
        if (s->tmiss++ > 2) mg_error(c, "keepalive");
      }
      settmout(c, MIP_TTYPE_KEEPALIVE);
    }
  }
#ifdef MIP_QPROFILE
  qp_log();
#endif
}

// This function executes in interrupt context, thus it should copy data
// somewhere fast. Note that newlib's malloc is not thread safe, thus use
// our lock-free queue with preallocated buffer to copy data and return asap
static void on_rx(void *buf, size_t len, void *userdata) {
  struct mip_if *ifp = (struct mip_if *) userdata;
  if (q_write(&ifp->queue, buf, len)) {
    qp_mark(QP_FRAMEPUSHED, (int) q_space(&ifp->queue));
  } else {
    ifp->dropped++;
    qp_mark(QP_FRAMEDROPPED, ifp->dropped);
    MG_ERROR(("dropped %d", (int) len));
  }
}

static void if_init(struct mip_if *ifp, struct mg_mgr *mgr,
                    struct mip_cfg *ipcfg, struct mip_driver *driver,
                    void *driver_data, size_t maxpktsize, size_t qlen) {
  memcpy(ifp->mac, ipcfg->mac, sizeof(ifp->mac));
  ifp->use_dhcp = ipcfg->ip == 0;
  ifp->ip = ipcfg->ip, ifp->mask = ipcfg->mask, ifp->gw = ipcfg->gw;
  ifp->rx.buf = (uint8_t *) (ifp + 1), ifp->rx.len = maxpktsize;
  ifp->tx.buf = ifp->rx.buf + maxpktsize, ifp->tx.len = maxpktsize;
  ifp->driver = driver;
  ifp->driver_data = driver_data;
  ifp->mgr = mgr;
  ifp->queue.buf = ifp->tx.buf + maxpktsize;
  ifp->queue.len = qlen;
  ifp->timer_1000ms = mg_millis();
  arp_cache_init(ifp->arp_cache, MIP_ARP_ENTRIES, 12);
  if (driver->setrx) driver->setrx(on_rx, ifp);
  mgr->priv = ifp;
  mgr->extraconnsize = sizeof(struct connstate);
#ifdef MIP_QPROFILE
  qp_init();
#endif
}

void mip_init(struct mg_mgr *mgr, struct mip_cfg *ipcfg,
              struct mip_driver *driver, void *driver_data) {
  if (driver->init && !driver->init(ipcfg->mac, driver_data)) {
    MG_ERROR(("driver init failed"));
  } else {
    size_t maxpktsize = 1540, qlen = driver->setrx ? MIP_QSIZE : 0;
    struct mip_if *ifp =
        (struct mip_if *) calloc(1, sizeof(*ifp) + 2 * maxpktsize + qlen);
    if_init(ifp, mgr, ipcfg, driver, driver_data, maxpktsize, qlen);
  }
}

int mg_mkpipe(struct mg_mgr *m, mg_event_handler_t fn, void *d, bool udp) {
  (void) m, (void) fn, (void) d, (void) udp;
  MG_ERROR(("Not implemented"));
  return -1;
}

#if 0
static uint16_t mkeport(void) {
  uint16_t a = 0, b = mg_millis() & 0xffffU, c = MIP_ETHEMERAL_PORT;
  mg_random(&a, sizeof(a));
  c += (a ^ b) % (0xffffU - MIP_ETHEMERAL_PORT);
  return c;
}
#endif

void mg_connect_resolved(struct mg_connection *c) {
  struct mip_if *ifp = (struct mip_if *) c->mgr->priv;
  c->is_resolving = 0;
  if (ifp->eport < MIP_ETHEMERAL_PORT) ifp->eport = MIP_ETHEMERAL_PORT;
  c->loc.ip = ifp->ip;
  c->loc.port = mg_htons(ifp->eport++);
  MG_DEBUG(("%lu %08lx:%hu->%08lx:%hu", c->id, mg_ntohl(c->loc.ip),
            mg_ntohs(c->loc.port), mg_ntohl(c->rem.ip), mg_ntohs(c->rem.port)));
  mg_call(c, MG_EV_RESOLVE, NULL);
  if (c->is_udp) {
    mg_call(c, MG_EV_CONNECT, NULL);
  } else {
    uint32_t isn = mg_htonl((uint32_t) mg_ntohs(c->loc.port));
    tx_tcp(ifp, c->rem.ip, TH_SYN, c->loc.port, c->rem.port, isn, 0, NULL, 0);
    c->is_connecting = 1;
  }
}

bool mg_open_listener(struct mg_connection *c, const char *url) {
  c->loc.port = mg_htons(mg_url_port(url));
  return true;
}

static void write_conn(struct mg_connection *c) {
  long len = c->is_tls ? mg_tls_send(c, c->send.buf, c->send.len)
                       : mg_io_send(c, c->send.buf, c->send.len);
  if (len > 0) {
    mg_iobuf_del(&c->send, 0, (size_t) len);
    mg_call(c, MG_EV_WRITE, &len);
  }
}

static void close_conn(struct mg_connection *c) {
  struct connstate *s = (struct connstate *) (c + 1);
  mg_iobuf_free(&s->raw);  // For TLS connections, release raw data
  if (c->is_udp == false && c->is_listening == false) {   // For TCP conns,
    struct mip_if *ifp = (struct mip_if *) c->mgr->priv;  // send TCP FIN
    tx_tcp(ifp, c->rem.ip, TH_FIN | TH_ACK, c->loc.port, c->rem.port,
           mg_htonl(s->seq), mg_htonl(s->ack), NULL, 0);
  }
  mg_close_conn(c);
}

static bool can_write(struct mg_connection *c) {
  return c->is_connecting == 0 && c->is_resolving == 0 && c->send.len > 0 &&
         c->is_tls_hs == 0;
}

void mg_mgr_poll(struct mg_mgr *mgr, int ms) {
  struct mg_connection *c, *tmp;
  uint64_t now = mg_millis();
  mip_poll((struct mip_if *) mgr->priv, now);
  mg_timer_poll(&mgr->timers, now);
  for (c = mgr->conns; c != NULL; c = tmp) {
    tmp = c->next;
    if (c->is_tls_hs) mg_tls_handshake(c);
    if (can_write(c)) write_conn(c);
    if (c->is_draining && c->send.len == 0) c->is_closing = 1;
    if (c->is_closing) close_conn(c);
  }
  (void) ms;
}

bool mg_send(struct mg_connection *c, const void *buf, size_t len) {
  struct mip_if *ifp = (struct mip_if *) c->mgr->priv;
  bool res = false;
  if (ifp->ip == 0 || ifp->state != MIP_STATE_READY) {
    mg_error(c, "net down");
  } else if (c->is_udp) {
    tx_udp(ifp, ifp->ip, c->loc.port, c->rem.ip, c->rem.port, buf, len);
    res = true;
  } else {
    res = mg_iobuf_add(&c->send, c->send.len, buf, len);
  }
  return res;
}

#ifdef MIP_QPROFILE

#pragma pack(push, 1)
struct qpentry {
  uint32_t timestamp;
  uint16_t type;
  uint16_t len;
};
#pragma pack(pop)

static struct queue qp;

// This is called from IRQ and main contexts; two producers, single consumer
// TODO(scaprile): avoid concurrency issues (2 queues ?)
void qp_mark(unsigned int type, int len) {
  static bool ovf = false;
  static uint16_t irq_ctr = 0, drop_ctr = 0;
  struct qpentry e = {.timestamp = (uint32_t) mg_millis(),
                      .type = (uint16_t) type,
                      .len = (uint16_t) len};
  if (type == QP_IRQTRIGGERED) e.len = ++irq_ctr;
  if (ovf) {
    e.type = (uint16_t) QP_QUEUEOVF;
    e.len = drop_ctr;
  }
  ovf = !q_write(&qp, &e, sizeof(e));
}

void qp_log(void) {
  struct qpentry e;
  const char *titles[] = {"IRQ ", "PUSH", "POP ", "DONE", "DROP", "OVFL"};
  for (int i = 0; i < 10 && q_read(&qp, &e); i++) {
    MG_INFO(("%lx %s %u", e.timestamp, titles[e.type], e.len));
  }
}

void qp_init(void) {
  qp.len = 500 * (sizeof(size_t) + sizeof(struct qpentry));
  qp.buf = calloc(1, qp.len);  // THERE IS NO FREE
}
#endif  // MIP_QPROFILE

#endif  // MG_ENABLE_MIP
