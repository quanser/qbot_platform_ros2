#!/usr/bin/env python3
"""Set up the QBot Platform warehouse arena in Quanser Interactive Labs (QLabs).

Run by qbot_platform_mapless_nav_bringup_launch.py when platform:=virtual, after QLabs is started.
It stops any running RT models, rebuilds the arena (walls, cones, flooring, cameras), spawns the
QBot, starts the workspace and virtual driver RT models with quarc_run, then exits with status 0 so
the launch file can start the driver and lidar nodes. QLabs must already be running with the
workspace open.

The RT models are the rt-linux_x86_64 builds shipped in this package's rt_models directory and are
started by path through quarc_run.
"""

import argparse
import math
import os
import subprocess
import sys
import time

# Arena layout in metres. Each wall is ((x, y), yaw in degrees).
CONES = [
    [-0.6, 0.3, 0.1],
    [-0.7, 0.8, 0.1],
    [1.2, -1.0, 0.1],
    [-1.0, -1.5, 0.1],
    [1.0, 1.3, 0.1],
    [1.1, 1.2, 0.1],
]

# Top, bottom and left walls of the 4 x 4 m box. Horizontal walls use yaw 90, vertical walls 0.
PERIMETER_WALLS = [
    ([-1.5, 2.0], 90), ([-0.5, 2.0], 90), ([0.5, 2.0], 90), ([1.5, 2.0], 90),
    ([-1.5, -2.0], 90), ([-0.5, -2.0], 90), ([0.5, -2.0], 90), ([1.5, -2.0], 90),
    ([-2.0, -1.5], 0), ([-2.0, -0.5], 0), ([-2.0, 0.5], 0), ([-2.0, 1.5], 0),
]

# The right wall is a zigzag from corner to corner with alternating +-22 degree legs.
ZIGZAG_WALLS = [
    ([2.2, -1.5, 0.1], -22),
    ([2.2, -0.5, 0.1], 22),
    ([2.2, 0.5, 0.1], -22),
    ([2.2, 1.5, 0.1], 22),
]

# QBot platform flooring tiles: (actor number, location, yaw in radians, configuration).
FLOOR_TILES = [
    (0, [-0.6, 0.6, 0.0], -math.pi / 2, 5),
    (1, [0.6, 1.8, 0.0], -math.pi / 2, 0),
    (2, [1.8, -0.6, 0.0], math.pi, 0),
    (3, [-0.6, -1.8, 0.0], math.pi / 2, 0),
    (4, [-1.8, 0.6, 0.0], 0.0, 0),
    (5, [-0.6, 0.6, 0.0], 0.0, 1),
    (6, [0.6, 0.6, 0.0], -math.pi / 2, 1),
    (7, [0.6, -0.6, 0.0], math.pi, 1),
    (8, [-0.6, -0.6, 0.0], math.pi / 2, 1),
]


def default_model_dir():
    """The installed rt_models directory, or the source one when run from the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(get_package_share_directory("qbot_hardware"), "rt_models")
    except Exception:  # noqa: BLE001 - not in a sourced ROS environment
        here = os.path.dirname(os.path.abspath(__file__))
        installed = os.path.join(here, "..", "..", "share", "qbot_hardware", "rt_models")
        return installed if os.path.isdir(installed) else os.path.join(here, "..", "rt_models")


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--host", default="localhost", help="Host running QLabs.")
    parser.add_argument(
        "--wait-sec",
        type=float,
        default=90.0,
        help="Seconds to keep retrying the QLabs connection, for when QLabs is still starting.",
    )
    parser.add_argument(
        "--post-connect-sec",
        type=float,
        default=5.0,
        help="Seconds to wait after a connection that needed retries, for the module to finish loading.",
    )
    parser.add_argument("--actor-number", type=int, default=0, help="QBot actor number to spawn.")
    parser.add_argument("--x", type=float, default=0.0, help="Spawn x position (m).")
    parser.add_argument("--y", type=float, default=-0.1, help="Spawn y position (m).")
    parser.add_argument("--z", type=float, default=0.1, help="Spawn z position (m).")
    parser.add_argument("--yaw-deg", type=float, default=90.0, help="Spawn yaw (degrees).")
    parser.add_argument(
        "--camera",
        choices=("top", "trailing"),
        default="top",
        help="Viewpoint after setup: a fixed top view of the arena or a camera trailing the QBot.",
    )
    parser.add_argument(
        "--keep-world",
        action="store_true",
        help="Do not destroy previously spawned actors and do not spawn the arena. Only the QBot is spawned.",
    )
    parser.add_argument(
        "--no-rt-model",
        action="store_true",
        help="Do not stop or start the RT models (they are already running).",
    )
    parser.add_argument("--rt-target", default="tcpip://localhost:17000", help="QUARC target URI for quarc_run.")
    parser.add_argument(
        "--model-suffix",
        default="rt-linux_x86_64",
        help="Build target of the virtual RT models (a Linux laptop). The physical QBot Platform uses "
        "rt-linux_qbot_platform and is not started by this script.",
    )
    parser.add_argument(
        "--model-dir",
        default=None,
        help="Directory holding the RT models. Default: the package's rt_models directory.",
    )
    parser.add_argument("--workspace-model", default=None, help="Default QBotPlatform_Workspace.<model-suffix>.")
    parser.add_argument("--driver-model", default=None, help="Default qbot_platform_driver_virtual.<model-suffix>.")
    parser.add_argument(
        "--driver-model-uri",
        default="tcpip://localhost:17098",
        help="URI passed to the virtual driver model with -uri.",
    )
    parser.add_argument(
        "--settle-sec",
        type=float,
        default=2.0,
        help="Seconds to wait after starting the RT models before exiting.",
    )
    # Launch appends ROS arguments to some process types; ignore anything we do not know.
    args, _ = parser.parse_known_args(argv)
    args.workspace_model = args.workspace_model or f"QBotPlatform_Workspace.{args.model_suffix}"
    args.driver_model = args.driver_model or f"qbot_platform_driver_virtual.{args.model_suffix}"
    args.model_dir = args.model_dir or default_model_dir()
    return args


def stop_rt_models(args):
    """Stop any running RT models. quarc_run reports an error when none are running, so ignore it."""
    print("qlabs_setup: stopping any pre-existing RT models.")
    subprocess.run(["quarc_run", "-q", "-Q", "-t", args.rt_target, args.driver_model], check=False)
    time.sleep(4.0)
    subprocess.run(["quarc_run", "-q", "-Q", "-t", args.rt_target, f"*.{args.model_suffix}"], check=False)


def model_path(args, model):
    """Full path of a shipped model, or the bare name (resolved by QUARC) if it is not in model_dir."""
    path = os.path.abspath(os.path.join(args.model_dir, model))
    return path if os.path.isfile(path) else model


def start_rt_models(args):
    print("qlabs_setup: starting RT models.")
    workspace = model_path(args, args.workspace_model)
    driver = model_path(args, args.driver_model)
    commands = [
        ["quarc_run", "-D", "-r", "-t", args.rt_target, workspace],
        ["quarc_run", "-D", "-r", "-t", args.rt_target, driver, "-uri", args.driver_model_uri],
    ]
    for cmd in commands:
        result = subprocess.run(cmd, check=False)
        if result.returncode != 0:
            print(f"qlabs_setup: '{' '.join(cmd)}' failed with exit code {result.returncode}.", file=sys.stderr)
            return False
    return True


def spawn_arena(qlabs, qvl):
    print("qlabs_setup: spawning the arena.")
    cone = qvl.traffic_cone.QLabsTrafficCone(qlabs)
    for location in CONES:
        cone.spawn(location=location, rotation=[0, 0, 0], scale=[1, 1, 1], configuration=0)

    wall = qvl.walls.QLabsWalls(qlabs)
    for (x, y), yaw in PERIMETER_WALLS:
        wall.spawn_degrees(location=[x, y, 0.1], rotation=[0, 0, yaw])
        wall.set_enable_dynamics(True)
    for location, yaw in ZIGZAG_WALLS:
        wall.spawn_degrees(location=location, rotation=[0, 0, yaw])
        wall.set_enable_dynamics(True)

    floor = qvl.qbot_platform_flooring.QLabsQBotPlatformFlooring(qlabs)
    for actor_number, location, yaw, configuration in FLOOR_TILES:
        floor.spawn_id(
            actorNumber=actor_number,
            location=location,
            rotation=[0, 0, yaw],
            scale=[1, 1, 1],
            configuration=configuration,
            waitForConfirmation=False,
        )


def main(argv=None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    # Launch captures the output through a pipe. Keep the messages in order with quarc_run's.
    sys.stdout.reconfigure(line_buffering=True)

    try:
        import qvl.free_camera
        import qvl.qbot_platform
        import qvl.qbot_platform_flooring
        import qvl.qlabs
        import qvl.traffic_cone
        import qvl.walls
    except ImportError as exc:
        print(f"qlabs_setup: Quanser QLabs Python API (qvl) is not available: {exc}", file=sys.stderr)
        return 2

    if not args.no_rt_model:
        try:
            stop_rt_models(args)
        except FileNotFoundError:
            print("qlabs_setup: quarc_run was not found on PATH.", file=sys.stderr)
            return 5

    qlabs = qvl.qlabs.QuanserInteractiveLabs()
    print(f"qlabs_setup: connecting to QLabs on '{args.host}'...")
    deadline = time.monotonic() + max(args.wait_sec, 0.0)
    retried = False
    while not qlabs.open(args.host):
        if time.monotonic() >= deadline:
            print("qlabs_setup: could not connect to QLabs. Is it running and is the workspace open?", file=sys.stderr)
            return 3
        retried = True
        time.sleep(2.0)
    if retried:
        print(f"qlabs_setup: connected. Waiting {args.post_connect_sec:.1f}s for the module to load.")
        time.sleep(max(args.post_connect_sec, 0.0))

    try:
        if not args.keep_world:
            qlabs.destroy_all_spawned_actors()

        try:
            print("qlabs_setup: spawning the QBot Platform.")
            qbot = qvl.qbot_platform.QLabsQBotPlatform(qlabs)
            qbot.spawn_id_degrees(
                actorNumber=args.actor_number,
                location=[args.x, args.y, args.z],
                rotation=[0, 0, args.yaw_deg],
                scale=[1, 1, 1],
                configuration=1,
                waitForConfirmation=False,
            )
            qbot.possess(qbot.VIEWPOINT_TRAILING)

            if not args.keep_world:
                spawn_arena(qlabs, qvl)

            if args.camera == "top":
                top_camera = qvl.free_camera.QLabsFreeCamera(qlabs)
                top_camera.spawn_degrees(location=[0.0, 0.0, 4], rotation=[0, 90.0, 90.0])
                top_camera.possess()
                time.sleep(3)
                top_camera.set_camera_properties(90, False, 2.9, 10.0)
        except Exception as exc:  # noqa: BLE001 - report any SDK failure to the launch file
            print(f"qlabs_setup: spawning in QLabs failed: {exc}", file=sys.stderr)
            return 4
    finally:
        qlabs.close()

    if not args.no_rt_model:
        try:
            if not start_rt_models(args):
                return 5
        except FileNotFoundError:
            print("qlabs_setup: quarc_run was not found on PATH.", file=sys.stderr)
            return 5
        print(f"qlabs_setup: RT models started. Waiting {args.settle_sec:.1f}s to settle.")
        time.sleep(max(args.settle_sec, 0.0))

    print("qlabs_setup: completed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
