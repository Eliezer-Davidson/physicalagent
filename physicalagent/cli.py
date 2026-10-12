"""physicalagent — command-line entry point.

    physicalagent doctor                 preflight checks, exit non-zero if not ready
    physicalagent status                 one-shot telemetry dump
    physicalagent run "<task>"           give Gemini control to accomplish a task
    physicalagent run "<task>" --dry     build everything, print the tool surface, DON'T move

Thin glue over the tested modules. `run` refuses to start motion unless doctor
passes (unless --force), and always leaves the robot halted on exit.
"""
from __future__ import annotations

import argparse
import os
import sys

from dotenv import load_dotenv

from physicalagent import config


SYSTEM_PROMPT = (
    "You are the reasoning brain of a small four-legged robot (a HiWonder ROSPug) "
    "on a carpeted floor. You perceive through a forward camera and a 360-degree "
    "lidar, and you act through movement tools. Distances are in centimetres, "
    "angles in degrees (+left / -right).\n\n"
    "HOW TO JUDGE YOUR OWN PROGRESS — this matters more than anything else:\n"
    "The robot's wheel-free odometry is UNRELIABLE for distance on carpet; it can "
    "report anywhere from half to two-and-a-half times the distance actually "
    "travelled. Never trust odom_estimate_cm. Judge real progress by watching "
    "front_clearance_m and the lidar sectors shrink as you approach something, and "
    "by looking with the camera. Rotation is TIMED and calibrated, accurate to a few "
    "degrees; odom_observed_deg on a turn is a rough cross-check that usually reads "
    "low, so ignore it. To face something precisely, turn a little, look, repeat.\n\n"
    "You can walk up to about 150 cm in a single call. Top speed is fixed at the "
    "hardware maximum, so asking for a longer distance simply walks for longer. "
    "Prefer a few decisive moves over many tiny ones, re-checking your surroundings "
    "between them.\n\n"
    "You remember earlier tasks in this session, so 'continue what you were doing' "
    "is meaningful — use that context.\n\n"
    "SAFETY: sense before you move and check clearance before walking forward. The "
    "lidar's rear sector is partially blind, so be careful reversing. Every tool "
    "reports what the safety layer actually allowed, which may be less than you "
    "asked for — read it and adapt. If motion is refused (low battery, sagging "
    "pack, paused deadman), do not fight it: explain and stop. Some action "
    "sequences are dynamic and can tip the robot, so ensure clear space first. "
    "When a task is done, give a short plain-language report of what you did."
)


def _load_env():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_dotenv(os.path.join(root, ".env"))


def _host():
    # No literal address lives in this repo. ROSPUG_HOST first, then the mDNS
    # name the robot advertises -- a site IP as a shipped default is both a
    # portability bug and something nobody outside that site should read.
    return os.environ.get("ROSPUG_HOST") or config.ROSPUG.host


def _connect():
    from physicalagent.robots.transport import RosbridgeTransport
    t = RosbridgeTransport(_host())
    t.connect()
    return t


def _frame_fetcher():
    import urllib.request
    url = f"http://{_host()}:8080/snapshot?topic=/csi_camera/image_raw&quality=90"
    return lambda: urllib.request.urlopen(url, timeout=8).read()


def resolve_profile(name: str | None):
    """Pick the robot profile: --robot, else $PHYSICALAGENT_ROBOT, else rospug."""
    from physicalagent.profiles import get_profile
    chosen = name or os.environ.get("PHYSICALAGENT_ROBOT") or "rospug"
    return get_profile(chosen)


def motion_unlocked_for(profile) -> bool:
    """Whether motion is unlocked for THIS robot.

    The profile is the source of truth: motion_unlocked=False (the default)
    means no human has yet watched this particular robot move under our
    control. $PHYSICALAGENT_MOTION_UNLOCKED is still honoured, but only in the
    conservative direction -- it can LOCK a robot, never unlock one.

    That asymmetry is deliberate. `./robot` exports the variable set to 1
    unconditionally, so a two-way override would mean every newly registered
    profile is unlocked on its very first run by a launcher script that knows
    nothing about it -- exactly the failure the per-robot flag exists to
    prevent. Setting it to 0 to be extra careful still works, and for the
    ROSPug (motion_unlocked=True, launched with the variable at 1) the
    resulting value is identical to what config.MOTION_UNLOCKED produced.
    """
    env = os.environ.get("PHYSICALAGENT_MOTION_UNLOCKED")
    if env is not None and env.strip() != "1":
        return False
    return bool(profile.motion_unlocked)


def build_guard(profile=None):
    """The one place the SafetyGuard is configured, so every entry point agrees.

    The action whitelist and the motion-unlock decision both come from the
    profile rather than from module-level globals. Constructing the guard
    inline per command is how `do_action` came to be silently dead in the
    console -- the adapter's list was opened up while the guard's stayed empty.
    """
    from physicalagent.safety import SafetyGuard
    if profile is None:
        profile = resolve_profile(None)
    return SafetyGuard(profile=profile.safety,
                       motion_unlocked=motion_unlocked_for(profile))


def tonypi_tier_for(args=None):
    """Resolve the TonyPi servo tier: --tier flag, else $PHYSICALAGENT_TONYPI_TIER,
    else LOCKED. Threaded the same conservative way as motion_unlocked -- a robot
    starts timid and is widened only by an explicit choice. LOCKED is the default;
    FULL is the deliberate override of HiWonder's conservative range/speed limits.
    An unknown name raises (ServoTier.resolve) rather than silently relaxing."""
    from physicalagent.robots.tonypi_servo import ServoTier
    name = getattr(args, "tier", None) if args is not None else None
    name = name or os.environ.get("PHYSICALAGENT_TONYPI_TIER")
    return ServoTier.resolve(name)


def minihexa_tier_for(args=None):
    """Resolve the MiniHexa motion tier: --tier flag, else
    $PHYSICALAGENT_MINIHEXA_TIER, else LOCKED. Threaded the same conservative way
    as the TonyPi tier -- the robot starts timid and is widened only by an
    explicit choice. LOCKED is the default; FULL is the deliberate override of
    HiWonder's conservative range/speed limits up to the firmware/electrical
    maximum. An unknown name raises (MotionTier.resolve) rather than silently
    relaxing the clamp."""
    from physicalagent.robots.minihexa_motion import MotionTier
    name = getattr(args, "tier", None) if args is not None else None
    name = name or os.environ.get("PHYSICALAGENT_MINIHEXA_TIER")
    return MotionTier.resolve(name)


def minihexa_link_kind_for(args=None):
    """Which MiniHexa transport to use: --link flag, else
    $PHYSICALAGENT_MINIHEXA_LINK, else serial. Serial is the full-control default
    (the BLE firmware cannot do per-joint or action-stop)."""
    kind = getattr(args, "link", None) if args is not None else None
    return kind or os.environ.get("PHYSICALAGENT_MINIHEXA_LINK") or "serial"


def tier_for(profile, args=None):
    """The motion tier for a tiered robot, dispatched by profile: TonyPi ->
    ServoTier, MiniHexa -> MotionTier, everything else (rospug) -> None. Kept
    separate so the two tier enums never cross wires -- passing a ServoTier to the
    MiniHexa clamp (or vice versa) would miss the tier band entirely."""
    if profile.name == "tonypi":
        return tonypi_tier_for(args)
    if profile.name == "minihexa":
        return minihexa_tier_for(args)
    return None


def link_kind_override_for(profile, args=None):
    """The transport override to pass to link_for, or None to use the profile's
    default. Only the MiniHexa exposes a --link choice; every other robot uses its
    profile's own LinkSpec.kind."""
    if profile.name == "minihexa":
        return minihexa_link_kind_for(args)
    return None


def link_for(profile, kind=None):
    """Build the profile's link object, chosen by LinkSpec.kind (or an explicit
    `kind` override, used by the MiniHexa --link flag to switch transport).

    rosbridge uses the shared _connect() (roslibpy); http_rpc builds a TonyPi
    JSON-RPC link; serial / ble build the MiniHexa links. The http_rpc, serial and
    ble links all open NO I/O at construction (the serial/ble links open lazily on
    first use), so this is safe to call before any I/O decision -- and safe on the
    --dry path, which never touches the link at all."""
    kind = kind or profile.link.kind
    if kind == "rosbridge":
        return _connect()
    if kind == "http_rpc":
        from physicalagent import config
        from physicalagent.robots.tonypi_link import HttpTonyLink
        return HttpTonyLink(host=profile.link.host or config.TONYPI.host,
                            rpc_port=profile.link.port or config.TONYPI.rpc_port,
                            video_port=config.TONYPI.video_port,
                            snapshot_path=config.TONYPI.snapshot_path)
    if kind == "serial":
        from physicalagent import config
        from physicalagent.robots.minihexa_link import SerialMiniHexaLink
        return SerialMiniHexaLink(device=profile.link.device or config.MINIHEXA.device,
                                  baud=profile.link.baud or config.MINIHEXA.baud)
    if kind == "ble":
        from physicalagent import config
        from physicalagent.robots.minihexa_link import BleMiniHexaLink
        return BleMiniHexaLink(address=config.MINIHEXA.ble_address,
                               write_char=config.MINIHEXA.ble_write_char,
                               notify_char=config.MINIHEXA.ble_notify_char)
    raise ValueError(f"profile {profile.name!r}: no link builder for kind {kind!r}")


def build_adapter(profile, link, guard, *, clock=None, frame_fetcher=None, tier=None):
    """Turn a resolved profile into its adapter, via the profile's own
    adapter_factory. This is the wiring the research found missing: the cmd_*
    bodies used to hard-code RospugAdapter, so `--robot tonypi` could never build
    a TonyPiAdapter. `tier` is the TonyPi servo tier (ignored by robots that have
    no tiers); `frame_fetcher` is used by the ROSPug's HTTP snapshot path and
    ignored by TonyPi (whose frames come from the link)."""
    factory = profile.adapter_factory
    if factory is None:
        raise ValueError(
            f"profile {profile.name!r} has no adapter_factory -- cannot build it")
    return factory(link, guard, clock=clock, robot_profile=profile,
                   frame_fetcher=frame_fetcher, tier=tier)


def cmd_doctor(_args) -> int:
    from physicalagent.doctor import diagnose
    profile = resolve_profile(getattr(_args, "robot", None))
    t = _connect()
    rep = diagnose(t, _frame_fetcher(), profile=profile.safety)
    print(rep.summary())
    t.terminate()
    return 0 if rep.ok else 1


def _rosbridge_link(host: str, port: int = 9090):
    """(call_service, transport) if rosbridge answers, else (None, None).

    Guarded at every step. The probe exists to leave a capture behind for a
    robot nobody has characterised yet, so a machine that is not ROS at all, a
    rosbridge that refuses mid-handshake, or a missing roslibpy must all
    degrade to "no ROS link" -- never take the capture down with a traceback.
    """
    try:
        from physicalagent.probe.net import scan_ports
        found = scan_ports(host, ports=((port, "rosbridge WebSocket"),))
        if not found or not found[0].open:
            return None, None
        from physicalagent.robots.transport import RosbridgeTransport
        link = RosbridgeTransport(host, port=port)
        link.connect()
        return link.call_service, link
    except Exception as exc:
        print(f"  (rosbridge port is open but the link failed: {exc})", file=sys.stderr)
        return None, None


def cmd_probe(args) -> int:
    """Capture everything about a newly-plugged-in robot."""
    import datetime as _dt
    import json
    from physicalagent.probe import run_probe

    robot = args.robot or "unknown"
    host = args.host or _host()
    stamp = _dt.datetime.now().strftime("%Y-%m-%d_%H%M%S")
    base = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "artifacts", "probes")

    # The graph dumper was written, tested and then never called: cmd_probe
    # passed no ros_call, so every capture recorded {"skipped": "no ROS link"}
    # while the probe printed "rosbridge is up -- dump the graph next". Scan
    # for 9090 first and, if it answers, hand run_probe a real rosapi caller.
    ros_call, link = _rosbridge_link(host)
    try:
        out = run_probe(host=host, robot=robot, base_dir=base, stamp=stamp,
                        ros_call=ros_call)
    finally:
        if link is not None:
            try:
                link.terminate()
            except Exception:
                pass
    print(f"capture written to {out}")

    manifest = json.loads((out / "manifest.json").read_text())
    conclusions = manifest.get("conclusions", [])
    if conclusions:
        for conclusion in conclusions:
            print(f"  {conclusion}")
    else:
        print("  (no conclusions drawn)")
    return 0


def cmd_status(_args) -> int:
    import time
    profile = resolve_profile(getattr(_args, "robot", None))
    link = link_for(profile, link_kind_override_for(profile, _args))
    rosbridge = profile.link.kind == "rosbridge"
    if rosbridge:
        _subscribe_all(link)
        time.sleep(3)
    a = build_adapter(profile, link, build_guard(profile),
                      frame_fetcher=_frame_fetcher() if rosbridge else None,
                      tier=tier_for(profile, _args))
    print(a.telemetry())
    if hasattr(link, "terminate"):
        link.terminate()
    return 0


def cmd_run(args) -> int:
    _load_env()
    # Argument errors are reported before environment errors: an unknown robot
    # name is the operator's typo, and telling them about a missing key first
    # sends them to the wrong problem. It also keeps the offline suite runnable
    # with no key in the environment at all.
    profile = resolve_profile(getattr(args, "robot", None))
    key = os.environ.get("GEMINI_API_KEY", "").strip()
    if not key:
        print("GEMINI_API_KEY not set (put it in .env)", file=sys.stderr)
        return 2

    from physicalagent.brain.er import Brain, GeminiClient
    from physicalagent.doctor import diagnose
    from physicalagent.robots.transport import MonotonicClock
    import threading
    import time

    rosbridge = profile.link.kind == "rosbridge"

    link = link_for(profile, link_kind_override_for(profile, args))
    if rosbridge:
        _subscribe_all(link)
        time.sleep(3)

    guard = build_guard(profile)
    # The adapter comes from the profile's factory, not a hard-coded class -- so
    # `--robot tonypi` builds a TonyPiAdapter here, not a RospugAdapter, and
    # `--robot minihexa` builds a MiniHexaAdapter. tier_for picks the right tier
    # enum per robot (ServoTier vs MotionTier), or None for the untiered rospug.
    adapter = build_adapter(profile, link, guard, clock=MonotonicClock(),
                            frame_fetcher=_frame_fetcher() if rosbridge else None,
                            tier=tier_for(profile, args))

    if args.dry:
        print("TOOL SURFACE Gemini would receive:\n")
        for d in adapter.gemini_declarations():
            print(f"  {d['name']:18s} {d['description'][:70]}...")
        _terminate(link)
        return 0

    # ROS preflight applies only to the rosbridge robot; TonyPi has no cmd_vel/
    # deadman/lidar to diagnose (its safety is the servo tier + settle-to-stance).
    if rosbridge:
        rep = diagnose(link, _frame_fetcher(), settle_s=0, profile=profile.safety)
        print(rep.summary())
        if not rep.ok and not args.force:
            print("\nrefusing to run — fix preflight or pass --force", file=sys.stderr)
            _terminate(link)
            return 1

    # keep supervision alive while Gemini drives (ROS deadman, or TonyPi camera)
    beating = threading.Event(); beating.set()
    _start_keepalive(profile, link, adapter, beating)

    prompt = SYSTEM_PROMPT if rosbridge else profile.embodiment_prompt
    brain = Brain(adapter, GeminiClient(key), prompt, max_hops=args.max_hops)
    print(f"\n=== running task: {args.task!r} ===\n")
    try:
        result = brain.run(args.task)
        print("\n=== trace ===")
        for e in result.trace:
            mark = "ok " if e.ok else "ERR"
            detail = f" -> {e.result}" if e.ok and e.result else (
                "" if e.ok else f" -> {e.error or ''}")
            print(f"  [{mark}] {e.name}({e.args}){detail}")
        print(f"\n=== {'COMPLETE' if result.complete else 'STOPPED (hop cap)'} in {result.hops} hops ===")
        if result.final_text:
            print(f"\n{result.final_text}")
    finally:
        beating.clear()
        adapter.stop_all()
        time.sleep(0.5)
        _terminate(link)
    return 0


def _terminate(link) -> None:
    """Best-effort link teardown. The TonyPi JSON-RPC link has no terminate();
    only the rosbridge transport does, so guard on the method."""
    if hasattr(link, "terminate"):
        try:
            link.terminate()
        except Exception:
            pass


def _start_keepalive(profile, link, adapter, beating) -> None:
    """Spawn the supervision keepalive thread for the driving session.

    rosbridge: publish the ROS deadman heartbeat so the robot-side node stays
    ALIVE. http_rpc (TonyPi): load the camera-only vision function (9 = lab_adjust,
    NO motion) and feed its ~7s deadman on a timer so `look` gets fresh frames.
    The periodic thread lives HERE, never in the adapter, so the adapter stays
    deterministic and offline-testable.
    """
    import threading
    import time
    if profile.link.kind == "rosbridge":
        def _beat():
            while beating.is_set():
                link.publish("/physicalagent/heartbeat", "std_msgs/Bool", {"data": True})
                time.sleep(0.2)
        threading.Thread(target=_beat, daemon=True).start()
        return

    # Only the TonyPi (http_rpc) has a camera-function deadman to feed. The
    # MiniHexa (serial/ble) has no camera and no host-fed watchdog -- its firmware
    # ignores silence entirely, so there is nothing to keep alive. Its safety is
    # the bounded burst (walk sends its own stop), not a heartbeat.
    if profile.link.kind != "http_rpc":
        return

    from physicalagent import config
    try:
        adapter.start_camera()
    except Exception as exc:
        print(f"  (camera keepalive failed to start: {exc})", file=sys.stderr)

    def _beat():
        while beating.is_set():
            try:
                adapter.heartbeat()
            except Exception:
                pass
            time.sleep(config.TONYPI.heartbeat_interval_s)
    threading.Thread(target=_beat, daemon=True).start()


def cmd_shell(args) -> int:
    """Interactive console: the operator types tasks, Gemini drives, live trace."""
    from physicalagent.brain.er import Brain, GeminiClient
    from physicalagent.doctor import diagnose
    from physicalagent.robots.transport import MonotonicClock
    from physicalagent.shell import run_shell, enable_line_editing
    import threading
    import time

    _load_env()
    # Argument errors before environment errors -- see cmd_run.
    profile = resolve_profile(getattr(args, "robot", None))
    key = os.environ.get("GEMINI_API_KEY")
    if not key:
        print("GEMINI_API_KEY not set (put it in .env)", file=sys.stderr)
        return 2
    rosbridge = profile.link.kind == "rosbridge"

    link = link_for(profile, link_kind_override_for(profile, args))
    if rosbridge:
        _subscribe_all(link)
        time.sleep(3)

    guard = build_guard(profile)
    # Adapter via the profile factory -- TonyPi builds a TonyPiAdapter, MiniHexa a
    # MiniHexaAdapter; tier_for picks the right tier enum per robot.
    adapter = build_adapter(profile, link, guard, clock=MonotonicClock(),
                            frame_fetcher=_frame_fetcher() if rosbridge else None,
                            tier=tier_for(profile, args))

    if rosbridge:
        rep = diagnose(link, _frame_fetcher(), settle_s=0, profile=profile.safety)
        print(rep.summary())
        if not rep.ok and not args.force:
            print("\nrefusing to start — fix preflight or pass --force", file=sys.stderr)
            _terminate(link)
            return 1

    beating = threading.Event(); beating.set()
    _start_keepalive(profile, link, adapter, beating)

    if rosbridge:
        print(f"\nlive camera: http://{_host()}:8080/stream?topic=/csi_camera/image_raw")
        caps = guard.profile.velocity_limits(guard.motion_unlocked)
        print(f"caps: {'UNLOCKED' if guard.motion_unlocked else 'LOCKED'} "
              f"{caps.max_abs_x:.2f} m/s")
    elif profile.link.kind == "http_rpc":
        from physicalagent import config
        print(f"\nlive camera: http://{profile.link.host}:"
              f"{config.TONYPI.video_port}{config.TONYPI.snapshot_path}")
        print(f"servo tier: {tonypi_tier_for(args).name}")
    else:
        # MiniHexa: no camera; report the chosen transport and motion tier.
        print(f"\ntransport: {link.kind}   motion tier: {minihexa_tier_for(args).name}")
        if not link.supports_per_joint:
            print("note: BLE — per-joint servo control and action-stop are unavailable")

    prompt = SYSTEM_PROMPT if rosbridge else profile.embodiment_prompt
    brain = Brain(adapter, GeminiClient(key), prompt, max_hops=args.max_hops)
    enable_line_editing()
    try:
        return run_shell(brain, adapter)
    finally:
        beating.clear()
        adapter.stop_all()
        time.sleep(0.5)
        _terminate(link)


def _subscribe_all(t):
    from physicalagent import config
    t.subscribe(config.ROSPUG.topic_battery, "std_msgs/UInt16", "batt")
    t.subscribe("/physicalagent/deadman_status", "std_msgs/String", "dead")
    t.subscribe("/scan_raw", "sensor_msgs/LaserScan", "scan")
    t.subscribe("/odom_raw", "nav_msgs/Odometry", "odom")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="physicalagent",
                                description="Give Gemini Robotics-ER control of a HiWonder robot.")
    sub = p.add_subparsers(dest="cmd", required=True)

    def _with_robot(sp):
        sp.add_argument("--robot", default=None,
                        help="which robot profile to use (default: $PHYSICALAGENT_ROBOT or rospug)")
        sp.add_argument("--tier", default=None,
                        help="TonyPi/MiniHexa motion tier: locked (default), normal, or full. "
                             "full overrides HiWonder's conservative range/speed limits "
                             "up to the servo's electrical max. Also "
                             "$PHYSICALAGENT_TONYPI_TIER / $PHYSICALAGENT_MINIHEXA_TIER.")
        sp.add_argument("--link", default="serial", choices=("serial", "ble"),
                        help="MiniHexa transport: serial (default, full control) or ble "
                             "(reduced set — no per-joint, no action-stop). Ignored by "
                             "robots that are not the MiniHexa.")
        return sp

    _with_robot(sub.add_parser("doctor", help="preflight checks")).set_defaults(func=cmd_doctor)
    _with_robot(sub.add_parser("status", help="telemetry dump")).set_defaults(func=cmd_status)
    pr = _with_robot(sub.add_parser("probe", help="capture what is plugged in"))
    pr.add_argument("--host", default=None, help="IP to port-scan (default: ROSPUG_HOST)")
    pr.set_defaults(func=cmd_probe)
    r = _with_robot(sub.add_parser("run", help="run a task under Gemini control"))
    r.add_argument("task", help="natural-language task, e.g. 'look around and describe the room'")
    r.add_argument("--dry", action="store_true", help="print the tool surface, do not move")
    r.add_argument("--force", action="store_true", help="run even if preflight fails")
    r.add_argument("--max-hops", type=int, default=60, dest="max_hops")
    r.set_defaults(func=cmd_run)
    s = _with_robot(sub.add_parser("shell", help="interactive console — type tasks, watch it work"))
    s.add_argument("--force", action="store_true", help="start even if preflight fails")
    s.add_argument("--max-hops", type=int, default=60, dest="max_hops")
    s.set_defaults(func=cmd_shell)
    return p


def main(argv=None) -> int:
    from physicalagent.profiles import UnknownRobotError

    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except UnknownRobotError as exc:
        # resolve_profile() raises UnknownRobotError naming the available
        # robots; a typo'd --robot is the expected failure mode, so it gets a
        # clean one-line message instead of a raw traceback. This is
        # deliberately narrower than bare KeyError -- catching every KeyError
        # here would also swallow (and mislabel as "unknown robot") a genuine
        # bug like an unchecked dict key raised mid-command, discarding the
        # traceback needed to debug it.
        print(exc.args[0], file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
