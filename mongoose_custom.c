#include "mip.h"
#include "mongoose.h"

#define MIP_ETHEMERAL_PORT 49152

static void mg_mip_ev_handler(struct mip_ev *mev) {
  struct mg_mgr *mgr = mev->ifp->evdata;
  if (mev->event == MIP_UP) {
    char buf[40];
    struct mg_addr addr = {.ip = mev->ifp->ip};
    MG_INFO(("IP: %s", mg_ntoa(&addr, buf, sizeof(buf))));
  } else if (mev->event == MIP_DOWN) {
    MG_ERROR(("Network down"));
  } else if (mev->event == MIP_UDP) {
    struct mg_connection *c = NULL;
    for (c = mgr->conns; c != NULL; c = c->next) {
      if (c->is_udp && c->loc.port == mev->dst_port) break;
    }
    if (c == NULL) {
      // printf("NO UDP listener on port %hu\n", );
      // MG_DEBUG(("%d %p %u", mev->event, mev->buf, mev->len));
    } else if (c != NULL) {
      c->rem.port = mev->src_port;
      c->rem.ip = mev->src_ip;
      if (c->recv.len >= MG_MAX_RECV_BUF_SIZE) {
        mg_error(c, "max_recv_buf_size reached");
      } else if (c->recv.size - c->recv.len < mev->len &&
                 !mg_iobuf_resize(&c->recv, c->recv.len + mev->len)) {
        mg_error(c, "oom");
      } else {
        memcpy(&c->recv.buf[c->recv.len], mev->buf, mev->len);
        c->recv.len += mev->len;
        struct mg_str evd = mg_str_n((char *) mev->buf, mev->len);
        mg_call(c, MG_EV_READ, &evd);
      }
    }
  }
}

void mg_attach_mip(struct mg_mgr *mgr, struct mip_if *ifp) {
  mgr->userdata = ifp;
  ifp->ev = mg_mip_ev_handler;
  ifp->evdata = mgr;
}

void mg_connect_resolved(struct mg_connection *c) {
  struct mip_if *ifp = (struct mip_if *) c->mgr->userdata;
  if (ifp->eport < MIP_ETHEMERAL_PORT) ifp->eport = MIP_ETHEMERAL_PORT;
  if (c->is_udp) {
    c->loc.ip = ifp->ip;
    c->loc.port = mg_htons(ifp->eport++);
    MG_INFO(("%lu %08lx.%hu->%08lx.%hu", c->id, mg_ntohl(c->loc.ip),
             mg_ntohs(c->loc.port), mg_ntohl(c->rem.ip),
             mg_ntohs(c->rem.port)));
  } else {
    mg_error(c, "Not implemented");
  }
  c->is_resolving = 0;
}

bool mg_open_listener(struct mg_connection *c, const char *url) {
  c->loc.port = mg_htons(mg_url_port(url));
  return true;
}

void mg_mgr_poll(struct mg_mgr *mgr, int ms) {
  // struct mip_if *ifp = (struct mip_if *) mgr->userdata;
  struct mg_connection *c, *tmp;
  uint64_t now = mg_millis();
  mip_poll(mgr->userdata, now);
  mg_timer_poll(&mgr->timers, now);
  for (c = mgr->conns; c != NULL; c = tmp) {
    tmp = c->next;
    mg_call(c, MG_EV_POLL, &now);
#if 0
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
      while (c->is_tls && read_conn(c) > 0) (void) 0;  // Read buffered TLS data
    }
#endif
    if (c->is_draining && c->send.len == 0) c->is_closing = 1;
    if (c->is_closing) mg_close_conn(c);
  }
  (void) ms;
}

bool mg_send(struct mg_connection *c, const void *buf, size_t len) {
  struct mip_if *ifp = (struct mip_if *) c->mgr->userdata;
  bool res = false;
  if (ifp->ip == 0) {
    mg_error(c, "net down");
  } else if (c->is_udp) {
    mip_tx_udp(ifp, ifp->ip, c->loc.port, c->rem.ip, c->rem.port, buf, len);
    res = true;
  } else {
    mg_error(c, "TCP Not implemented");
  }
  return res;
}

int mg_mkpipe(struct mg_mgr *mgr, mg_event_handler_t fn, void *fn_data) {
  (void) mgr, (void) fn, (void) fn_data;
  return -1;
}
