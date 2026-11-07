#!/usr/bin/env python3
import argparse
import math
import os
import sys
import time

import numpy as np
import mujoco

import mujoco_model as km


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Demo:
    CTRL_DT = 0.02          # control period (s)

    def __init__(self, model, data, viewer=None, writer=None, renderer=None, fps=30,
                 debug=False):
        self.m, self.d = model, data
        self.viewer, self.writer, self.renderer = viewer, writer, renderer
        self.fps, self.debug = fps, debug
        self.camera = "wide"
        self._next_frame = 0.0
        self._next_sync = 0.0
        self._wall0 = time.time()

        J = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
        A = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
        B = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
        self.q_x, self.q_y, self.q_yaw = (model.jnt_qposadr[J(n)] for n in ("base_x", "base_y", "base_yaw"))
        self.q_tray = model.jnt_qposadr[J("tray_joint")]
        self.j_star = J("star_joint")
        self.q_star = model.jnt_qposadr[self.j_star]
        self.dof_star = model.jnt_dofadr[self.j_star]
        self.q_tab = model.jnt_qposadr[J("tab_joint")]
        self.a_x, self.a_y, self.a_yaw = A("act_x"), A("act_y"), A("act_yaw")
        self.a_tray = A("act_tray")
        self.a_wheel = [(A(f"act_{n}_wheel"), math.radians(a)) for n, a in km.WHEELS]
        self.b_apples = [B(f"apple_{k + 1}") for k in range(km.N_APPLES)]
        self.cmd = np.zeros(3)          # commanded world-frame (vx, vy, wz)

        # star-wheel ratchet state: wheel may only move inside [n*pitch-eps, (n+1)*pitch+eps]
        self.star_n = 0
        self._set_star_window()

    # ------------------------------------------------------------ star wheel
    def _set_star_window(self):
        lo = self.star_n * km.STAR_PITCH - km.STAR_EPS
        hi = (self.star_n + 1) * km.STAR_PITCH + km.STAR_EPS
        self.m.jnt_range[self.j_star] = (lo, hi)
        self.m.jnt_limited[self.j_star] = 1

    def _star_step(self):
        th = self.d.qpos[self.q_star]
        # ratchet: once the wheel is within 15 deg of the next position it locks in
        if th >= (self.star_n + 1) * km.STAR_PITCH - math.radians(15):
            self.star_n += 1
            self._set_star_window()
            print(f"      * star wheel stepped -> {self.star_n} item(s) released "
                  f"(t={self.d.time:.2f}s)")
        # detent: stable at every multiple of the pitch
        self.d.qfrc_applied[self.dof_star] = -km.DETENT_K * math.sin(km.STAR_SPOKES * th)

    # ------------------------------------------------------------------ stepping
    def _step(self):
        self._star_step()
        mujoco.mj_step(self.m, self.d)
        t = self.d.time
        if self.viewer is not None and t >= self._next_sync:
            self._next_sync = t + 1 / 60.0
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.viewer.cam.fixedcamid = mujoco.mj_name2id(
                self.m, mujoco.mjtObj.mjOBJ_CAMERA, self.camera)
            self.viewer.sync()
            lag = (self._wall0 + t) - time.time()      # keep real-time pace
            if lag > 0:
                time.sleep(lag)
        if self.writer is not None and t >= self._next_frame:
            self._next_frame = t + 1.0 / self.fps
            self.renderer.update_scene(self.d, camera=self.camera)
            self.writer.append_data(self.renderer.render())

    def advance(self, seconds):
        end = self.d.time + seconds
        while self.d.time < end:
            if self.viewer is not None and not self.viewer.is_running():
                raise KeyboardInterrupt
            self._step()

    # ------------------------------------------------------------------ base
    def pose(self):
        return (self.d.qpos[self.q_x], self.d.qpos[self.q_y], self.d.qpos[self.q_yaw])

    def set_base_cmd(self, vx, vy, wz):
        """World-frame velocity command; wheels are spun with matching kinematics."""
        self.d.ctrl[self.a_x], self.d.ctrl[self.a_y], self.d.ctrl[self.a_yaw] = vx, vy, wz
        yaw = self.d.qpos[self.q_yaw]
        c, s = math.cos(yaw), math.sin(yaw)
        vbx, vby = c * vx + s * vy, -s * vx + c * vy        # body frame
        for act, a in self.a_wheel:
            v_t = -vbx * math.sin(a) + vby * math.cos(a) + km.WHEEL_OFFSET * wz
            self.d.ctrl[act] = -v_t / km.WHEEL_RADIUS

    def drive_to(self, tx, ty, tyaw=0.0, kp=2.0, vmax=0.25, wmax=0.8, a_max=0.4,
                 tol=0.003, yaw_tol=0.004, timeout=60.0):
        t_end, still = self.d.time + timeout, 0.0
        while self.d.time < t_end:
            x, y, yaw = self.pose()
            ex, ey, eyaw = tx - x, ty - y, wrap(tyaw - yaw)
            vd = np.array([kp * ex, kp * ey])
            n = np.linalg.norm(vd)
            if n > vmax:
                vd *= vmax / n
            wd = float(np.clip(2.0 * eyaw, -wmax, wmax))
            want = np.array([vd[0], vd[1], wd])
            dv = want - self.cmd
            lim = a_max * self.CTRL_DT                      # gentle: an apple is on the tray
            nv = np.linalg.norm(dv[:2])
            if nv > lim:
                dv[:2] *= lim / nv
            dv[2] = np.clip(dv[2], -3 * lim, 3 * lim)
            self.cmd = self.cmd + dv
            self.set_base_cmd(*self.cmd)
            self.advance(self.CTRL_DT)
            if abs(ex) < tol and abs(ey) < tol and abs(eyaw) < yaw_tol \
                    and np.linalg.norm(self.cmd) < 0.01:
                still += self.CTRL_DT
                if still > 0.5:
                    break
            else:
                still = 0.0
        self.cmd[:] = 0
        self.set_base_cmd(0, 0, 0)
        self.advance(0.3)

    # ------------------------------------------------------------------ tray
    def ramp(self, act, start, end, speed):
        """Smooth (ease-in/ease-out) move so the apple does not slip on the tray."""
        n = max(1, int(1.5 * abs(end - start) / speed / self.CTRL_DT))
        for i in range(1, n + 1):
            u = i / n
            u = u * u * (3 - 2 * u)
            self.d.ctrl[act] = start + (end - start) * u
            self.advance(self.CTRL_DT)
            if self.debug and i % 10 == 0:
                print(f"      q_tray={self.d.qpos[self.q_tray]:.3f}  "
                      f"star={math.degrees(self.d.qpos[self.q_star]):6.1f} deg  "
                      f"tab={math.degrees(self.d.qpos[self.q_tab]):5.1f} deg")
        self.d.ctrl[act] = end

    # ------------------------------------------------------------------ result
    def apples_on_tray(self):
        x, y, yaw = self.pose()
        c, s = math.cos(yaw), math.sin(yaw)
        q = self.d.qpos[self.q_tray]
        on = []
        for k, b in enumerate(self.b_apples):
            ap = self.d.xpos[b]
            dx, dy = ap[0] - x, ap[1] - y
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            if ap[2] > km.TRAY_FLOOR_TOP and (q - 0.11) < lx < (q + 0.12) and abs(ly) < 0.08:
                on.append(f"apple_{k + 1}")
        return on


def run(demo, shelf_x, shelf_y):
    stop_x = shelf_x - km.ROBOT_STANDOFF
    start = demo.pose()

    print("0/5 Letting the apples settle in the hopper ...")
    demo.camera = "wide"
    demo.advance(2.0)

    print(f"1/5 Driving to ({stop_x:.3f}, {shelf_y:.3f}) ...")
    demo.drive_to(stop_x, shelf_y, 0.0)

    print("2/5 Extending tray - tab pushes a spoke, star wheel steps ...")
    demo.camera = "close"
    demo.ramp(demo.a_tray, 0.0, km.TRAY_PUSH_END, speed=0.06)

    print("3/5 Waiting for the apple to drop onto the tray ...")
    demo.advance(2.0)
    print(f"      on tray: {demo.apples_on_tray()}  (wheel released {demo.star_n})")

    print("4/5 Retracting tray to home (tab folds under the locked spoke) ...")
    demo.ramp(demo.a_tray, km.TRAY_PUSH_END, 0.0, speed=0.06)
    demo.advance(1.5)

    print("5/5 Driving back to the start pose ...")
    demo.camera = "wide"
    demo.drive_to(start[0], start[1], 0.0)
    demo.advance(1.0)

    on = demo.apples_on_tray()
    ok = len(on) == 1 and demo.star_n == 1
    print(f"Result: {on} on the tray, wheel released {demo.star_n} -> "
          f"{'SUCCESS - exactly one item picked' if ok else 'CHECK: expected exactly one item'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--shelf-x", type=float, default=1.5, help="star-wheel axle x (world)")
    ap.add_argument("--shelf-y", type=float, default=0.0)
    ap.add_argument("--record", metavar="FILE.mp4", help="render headless to a video file")
    ap.add_argument("--headless", action="store_true", help="no window, no video")
    ap.add_argument("--debug", action="store_true", help="print tray / wheel / tab angles")
    ap.add_argument("--save-xml", metavar="FILE.xml", help="also write the generated MJCF")
    args = ap.parse_args()

    xml = km.build_xml(args.shelf_x, args.shelf_y)
    if args.save_xml:
        with open(args.save_xml, "w") as fh:
            fh.write(xml)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    if args.record:
        import imageio
        os.environ.setdefault("MUJOCO_GL", "egl")
        renderer = mujoco.Renderer(model, height=720, width=1280)
        writer = imageio.get_writer(args.record, fps=30)
        demo = Demo(model, data, writer=writer, renderer=renderer, debug=args.debug)
        try:
            run(demo, args.shelf_x, args.shelf_y)
        finally:
            writer.close()
            print("video saved:", args.record)
    elif args.headless:
        demo = Demo(model, data, debug=args.debug)
        sys.exit(0 if run(demo, args.shelf_x, args.shelf_y) else 1)
    else:
        from mujoco import viewer as mj_viewer
        with mj_viewer.launch_passive(model, data) as viewer:
            demo = Demo(model, data, viewer=viewer, debug=args.debug)
            try:
                run(demo, args.shelf_x, args.shelf_y)
                print("Done - close the window to exit.")
                while viewer.is_running():
                    demo.advance(0.05)
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
