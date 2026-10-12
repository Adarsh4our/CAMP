#!/usr/bin/env python3
"""
arrange_windows.py — Multi-Window Tiling Manager for CAMP Operator Dashboard

Automatically detects top-level X11 client windows via _NET_CLIENT_LIST
(bypassing window manager frame decoration wrappers) and tiles them:
  1. Window 1: RViz Campus Map & Camera  -> Left Half   (0, 0, W/2, H)
  2. Window 2: RViz Livox Mid-360 LiDAR  -> Top-Right   (W/2, 0, W/2, H/2)
  3. Window 3: Gazebo Harmonic Sim       -> Bottom-Right(W/2, H/2, W/2, H/2)
"""

import ctypes
from ctypes import util
import os
import sys
import time

x11 = ctypes.cdll.LoadLibrary(util.find_library('X11'))


class Display(ctypes.Structure):
    pass


x11.XOpenDisplay.restype = ctypes.POINTER(Display)
x11.XDefaultRootWindow.restype = ctypes.c_ulong
x11.XInternAtom.restype = ctypes.c_ulong
x11.XDefaultScreen.restype = ctypes.c_int
x11.XDisplayWidth.restype = ctypes.c_int
x11.XDisplayHeight.restype = ctypes.c_int


def get_client_windows(display, root):
    """Retrieve actual client window IDs from the EWMH _NET_CLIENT_LIST property."""
    atom = x11.XInternAtom(display, b'_NET_CLIENT_LIST', False)
    actual_type = ctypes.c_ulong()
    actual_format = ctypes.c_int()
    nitems = ctypes.c_ulong()
    bytes_after = ctypes.c_ulong()
    prop_ptr = ctypes.POINTER(ctypes.c_ulong)()

    status = x11.XGetWindowProperty(
        display, root, atom, 0, 1024, False, 0,
        ctypes.byref(actual_type), ctypes.byref(actual_format),
        ctypes.byref(nitems), ctypes.byref(bytes_after),
        ctypes.byref(prop_ptr)
    )
    if status != 0 or not prop_ptr:
        return []

    windows = [prop_ptr[i] for i in range(nitems.value)]
    x11.XFree(prop_ptr)
    return windows


def get_window_title(display, window):
    """Fetch window name/title string."""
    name_return = ctypes.c_char_p()
    x11.XFetchName(display, window, ctypes.byref(name_return))
    if name_return.value:
        name = name_return.value.decode('utf-8', 'ignore')
        x11.XFree(name_return)
        return name
    return ''


def arrange():
    display = x11.XOpenDisplay(None)
    if not display:
        print("[arrange_windows] Cannot open X display. Exiting.")
        return

    screen = x11.XDefaultScreen(display)
    screen_w = x11.XDisplayWidth(display, screen)
    screen_h = x11.XDisplayHeight(display, screen)

    # Standard fallback to 1920x1080 if display dims read unusual
    if screen_w < 640 or screen_h < 480:
        screen_w, screen_h = 1920, 1080

    half_w = screen_w // 2
    half_h = screen_h // 2

    root = x11.XDefaultRootWindow(display)
    print(f"[arrange_windows] Display detected: {screen_w}x{screen_h}. Monitoring for simulation windows...")

    moved_gazebo = False
    moved_map_rviz = False
    moved_lidar_rviz = False

    # Poll for windows for up to 35 seconds
    start_time = time.time()
    while time.time() - start_time < 35.0:
        clients = get_client_windows(display, root)
        for win in clients:
            title = get_window_title(display, win)
            if not title:
                continue

            # 1. Gazebo Simulation Window -> Bottom-Right
            if ('Gazebo Sim' in title or 'gz-sim-gui' in title) and not moved_gazebo:
                print(f"[arrange_windows] Found Gazebo ({hex(win)}). Moving to Bottom-Right: ({half_w}, {half_h}, {half_w}, {half_h})")
                x11.XMoveResizeWindow(display, win, half_w, half_h, half_w, half_h)
                x11.XFlush(display)
                moved_gazebo = True

            # 2. Campus Map RViz -> Left Half
            elif 'campus_nav.rviz' in title and not moved_map_rviz:
                print(f"[arrange_windows] Found Campus Map RViz ({hex(win)}). Moving to Left-Half: (0, 0, {half_w}, {screen_h})")
                x11.XMoveResizeWindow(display, win, 0, 0, half_w, screen_h)
                x11.XFlush(display)
                moved_map_rviz = True

            # 3. Livox Mid-360 LiDAR RViz -> Top-Right
            elif 'livox_mid360.rviz' in title and not moved_lidar_rviz:
                print(f"[arrange_windows] Found LiDAR RViz ({hex(win)}). Moving to Top-Right: ({half_w}, 0, {half_w}, {half_h})")
                x11.XMoveResizeWindow(display, win, half_w, 0, half_w, half_h)
                x11.XFlush(display)
                moved_lidar_rviz = True

        if moved_gazebo and moved_map_rviz and moved_lidar_rviz:
            print("[arrange_windows] All 3 operator windows arranged successfully!")
            break

        time.sleep(0.5)

    x11.XCloseDisplay(display)


if __name__ == '__main__':
    arrange()
