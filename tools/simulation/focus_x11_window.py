#!/usr/bin/env python3
"""Find, raise and fullscreen an X11 window; print its numeric XID."""

import argparse
import sys
import time

from Xlib import X, display, protocol


def window_title(window, utf8_atom, name_atom):
    try:
        prop = window.get_full_property(name_atom, utf8_atom)
        if prop is not None:
            value = prop.value
            if isinstance(value, bytes):
                return value.decode("utf-8", errors="replace")
            return str(value)
        return window.get_wm_name() or ""
    except Exception:
        return ""


def send_wm_state(dpy, root, window, action, first, second=0):
    event = protocol.event.ClientMessage(
        window=window,
        client_type=dpy.intern_atom("_NET_WM_STATE"),
        data=(32, [action, first, second, 1, 0]),
    )
    root.send_event(
        event,
        event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--title", default="Gazebo")
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--no-fullscreen", action="store_true")
    args = parser.parse_args()

    dpy = display.Display()
    root = dpy.screen().root
    client_list_atom = dpy.intern_atom("_NET_CLIENT_LIST")
    utf8_atom = dpy.intern_atom("UTF8_STRING")
    name_atom = dpy.intern_atom("_NET_WM_NAME")
    target = None
    deadline = time.monotonic() + max(args.timeout, 0.0)
    while time.monotonic() <= deadline:
        prop = root.get_full_property(client_list_atom, X.AnyPropertyType)
        for xid in [] if prop is None else prop.value:
            window = dpy.create_resource_object("window", int(xid))
            if args.title.lower() in window_title(window, utf8_atom, name_atom).lower():
                target = window
                break
        if target is not None:
            break
        time.sleep(0.2)

    if target is None:
        raise SystemExit("X11 window not found: %s" % args.title)

    active_atom = dpy.intern_atom("_NET_ACTIVE_WINDOW")
    active_event = protocol.event.ClientMessage(
        window=target,
        client_type=active_atom,
        data=(32, [1, X.CurrentTime, 0, 0, 0]),
    )
    root.send_event(
        active_event,
        event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask,
    )
    send_wm_state(
        dpy, root, target, 1,
        dpy.intern_atom("_NET_WM_STATE_MAXIMIZED_VERT"),
        dpy.intern_atom("_NET_WM_STATE_MAXIMIZED_HORZ"),
    )
    if not args.no_fullscreen:
        send_wm_state(
            dpy, root, target, 1,
            dpy.intern_atom("_NET_WM_STATE_FULLSCREEN"),
        )
    try:
        target.configure(stack_mode=X.Above)
    except Exception:
        pass
    dpy.sync()
    time.sleep(0.5)
    print(int(target.id))


if __name__ == "__main__":
    main()
