#!/usr/bin/env python3
"""Native Jazzy calibration using memo, with an optional RViz image layer."""
import argparse
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
ARTIFACTS = WORKSPACE / "data/calibration"
GUI_IMAGE = os.environ.get("CALIBRATION_GUI_IMAGE", "memo-calibration:jazzy")


def execute(command):
    subprocess.run([str(value) for value in command], check=True)


def runtime(args, command, *, overlay=True, gui=False, record=False):
    install = ARTIFACTS / ("gui-install" if gui else "install")
    script = 'set -e; unset OPAL_PREFIX; source /opt/ros/jazzy/setup.bash; '
    if overlay:
        script += 'source "$1/local_setup.bash"; shift; '
    script += 'exec "$@"'
    wrapped = ["bash", "-c", script, "calibration"] + ([str(install)] if overlay else []) + list(command)
    domain = args.domain_id if args.domain_id is not None else (None if record else 77)
    env = ["env", "-u", "CYCLONEDDS_URI"]
    if domain is not None:
        env += [f"ROS_DOMAIN_ID={domain}"]
    if gui:
        docker = ["docker", "run", "--rm", "--init", "--network", "host", "--user", f"{os.getuid()}:{os.getgid()}",
                  "--env", "HOME=/tmp", "--volume", f"{WORKSPACE}:{WORKSPACE}", "--workdir", str(WORKSPACE)]
        # The build command uses the GUI image without needing a display.
        if args.command == "capture":
            if not os.environ.get("DISPLAY"):
                raise ValueError("DISPLAY is required for RViz; omit --gui for headless capture")
            docker += ["--env", f"DISPLAY={os.environ['DISPLAY']}", "--env", "QT_X11_NO_MITSHM=1",
                       "--env", "LIBGL_ALWAYS_SOFTWARE=1", "--volume", "/tmp/.X11-unix:/tmp/.X11-unix:ro"]
            authority = Path(os.environ.get("XAUTHORITY", str(Path.home() / ".Xauthority")))
            if authority.is_file():
                docker += ["--volume", f"{authority}:/tmp/calibration.xauth:ro", "--env", "XAUTHORITY=/tmp/calibration.xauth"]
        if sys.stdin.isatty():
            docker += ["-it"]
        execute(docker + [GUI_IMAGE] + env + wrapped)
    elif os.environ.get("ROS_DISTRO") == "jazzy":
        execute(env + wrapped)
    else:
        docker = ["docker", "exec"]
        if sys.stdin.isatty():
            docker += ["-it"]
        execute(docker + [args.memo_container] + env + wrapped)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, default=ARTIFACTS / "vlp16")
    parser.add_argument("--memo-container", default="memo_ws")
    parser.add_argument("--domain-id", type=int, help="Default: 77 for offline work; memo's existing domain for recording")
    sub = parser.add_subparsers(dest="command", required=True)
    build = sub.add_parser("build")
    build.add_argument("--gui", action="store_true", help="Build the optional RViz image and panel")
    build.add_argument("--base-image", default="memo_ws:latest")
    init = sub.add_parser("init", help="Create measured board parameters; never overwrite a session")
    init.add_argument("--board-width-mm", type=float, required=True)
    init.add_argument("--board-height-mm", type=float, required=True)
    init.add_argument("--landscape", action="store_true")
    init.add_argument("--image-topic", default="/camera/rgb/image_raw")
    init.add_argument("--compressed-topic", help="Decode this JPEG topic into --image-topic during capture")
    init.add_argument("--info-topic", default="/camera/rgb/camera_info")
    init.add_argument("--points-topic", default="/velodyne_points")
    init.add_argument("--lidar-frame", default="velodyne")
    sub.add_parser("record")
    play = sub.add_parser("play")
    play.add_argument("bag", type=Path)
    play.add_argument("--rate", type=float, default=0.5)
    play.add_argument("--start-offset", type=float, default=0.0, help="Seconds after the recording starts")
    play.add_argument("--duration", type=float, help="Replay only this many seconds of recorded time")
    play.add_argument("--no-pause", action="store_true", help="Start immediately instead of waiting for space")
    capture = sub.add_parser("capture")
    capture.add_argument("--gui", action="store_true")
    capture.add_argument("--live", action="store_true", help="Use wall time; also set --domain-id to the sensor domain")
    for name in ("background", "sample", "discard", "optimise"):
        sub.add_parser(name)
    assess = sub.add_parser("assess")
    assess.add_argument("csv", type=Path)
    export = sub.add_parser("export", help="Print a static-transform command; does not publish it")
    export.add_argument("result", type=Path)
    export.add_argument("--camera-frame", required=True)
    export.add_argument("--lidar-frame", required=True)
    args = parser.parse_args()
    session = args.session.resolve()
    if args.domain_id is not None and not 0 <= args.domain_id <= 232:
        raise ValueError("Use a DDS domain in 0..232")
    if args.command == "build":
        ARTIFACTS.mkdir(parents=True, exist_ok=True)
        if args.gui:
            execute(["docker", "build", "--build-arg", f"BASE_IMAGE={args.base_image}", "-t", GUI_IMAGE,
                     "-f", PACKAGE / "docker/Dockerfile.memo", PACKAGE])
        prefix = "gui-" if args.gui else ""
        runtime(args, ["env", "COLCON_DEFAULTS_FILE=/dev/null", "CMAKE_BUILD_PARALLEL_LEVEL=2", "MAKEFLAGS=-j2",
                       "colcon", "--log-base", str(ARTIFACTS / f"{prefix}log"), "build", "--base-paths", str(PACKAGE),
                       "--build-base", str(ARTIFACTS / f"{prefix}build"), "--install-base", str(ARTIFACTS / f"{prefix}install"),
                       "--symlink-install", "--cmake-args", f"-DBUILD_RVIZ_PANEL={'ON' if args.gui else 'OFF'}",
                       "-DBUILD_TESTING=ON", "-DCMAKE_BUILD_TYPE=Release",
                       "-DMPI_C_COMPILER=/usr/bin/mpicc", "-DMPI_CXX_COMPILER=/usr/bin/mpicxx"],
                overlay=False, gui=args.gui)
        return
    if args.command == "init":
        width, height = (8, 5) if args.landscape else (5, 8)
        if (not all(math.isfinite(v) for v in (args.board_width_mm, args.board_height_mm)) or
                args.board_width_mm + 1e-6 < (width + 1) * 83.63 or
                args.board_height_mm + 1e-6 < (height + 1) * 83.63):
            raise ValueError("Backing board must contain the 501.78 x 752.67 mm grid (swap for landscape)")
        if (session / "params.yaml").exists():
            raise ValueError("Session already exists; choose a new --session")
        params = {"camera_topic": args.image_topic, "camera_info": args.info_topic, "lidar_topic": args.points_topic,
                  "bounds": {"x_min": 0.5, "x_max": 4.0, "y_min": -2.0, "y_max": 2.0,
                             "z_min": -2.0, "z_max": 2.0, "k": 50, "z": 1.0, "voxel_res": 0.4},
                  "chessboard": {"pattern_size": {"width": width, "height": height}, "square_length": 83.63,
                                 "board_dimension": {"width": args.board_width_mm, "height": args.board_height_mm},
                                 "translation_error": {"x": 0.0, "y": 0.0}}}
        session.mkdir(parents=True, exist_ok=True)
        # JSON is a YAML subset and is accepted by the ROS parameter parser.
        (session / "params.yaml").write_text(json.dumps({"/**": {"ros__parameters": params}}, indent=2) + "\n")
        displays = []
        for kind, name, topic in (("Image", "Camera", args.image_topic), ("Image", "Board detection", "/camera_features"),
                                  ("PointCloud2", "ROI", "/experimental_region"),
                                  ("PointCloud2", "Board", "/chessboard")):
            displays.append({"Class": f"rviz_default_plugins/{kind}", "Name": name, "Enabled": True,
                             "Topic": {"Value": topic, "Reliability Policy": "Best Effort"},
                             "Size (Pixels)": 3, "Color Transformer": "Intensity"})
        rviz = {"Panels": [{"Class": "rviz_common/Displays", "Name": "Displays"},
                            {"Class": "cam_lidar_calibration::CamLidarPanel", "Name": "Calibration"}],
                "Visualization Manager": {"Class": "", "Global Options": {"Fixed Frame": args.lidar_frame}, "Displays": displays}}
        (session / "capture.rviz").write_text(json.dumps(rviz, indent=2) + "\n")
        (session / "input.json").write_text(json.dumps({"compressed_topic": args.compressed_topic}, indent=2) + "\n")
        print(f"Created {session}; verify topic names, ROI, and print-center offsets before capture")
        return
    if args.command == "export":
        result = json.loads(args.result.read_text())
        if result["direction"] != "camera_optical_to_lidar" or args.camera_frame == args.lidar_frame:
            raise ValueError("Need camera_optical_to_lidar data and distinct actual sensor frames")
        command = ["ros2", "run", "tf2_ros", "static_transform_publisher"]
        for key, value in zip(("x", "y", "z", "qx", "qy", "qz", "qw"), result["translation_m"] + result["quaternion_xyzw"]):
            command += [f"--{key}", str(value)]
        print(shlex.join(command + ["--frame-id", args.lidar_frame, "--child-frame-id", args.camera_frame]))
        return
    if args.command == "play":
        if (args.rate <= 0 or args.start_offset < 0 or (args.duration is not None and args.duration <= 0)
                or not (args.bag.resolve() / "metadata.yaml").is_file()):
            raise ValueError("Need a completed Jazzy bag directory and positive playback rate")
        params = json.loads((session / "params.yaml").read_text())["/**"]["ros__parameters"]
        source = json.loads((session / "input.json").read_text()).get("compressed_topic") if (session / "input.json").exists() else None
        controls = ["--start-offset", str(args.start_offset)]
        if not args.no_pause:
            controls += ["--start-paused"]
        if args.duration is not None:
            controls += ["--playback-duration", str(args.duration)]
        runtime(args, ["ros2", "bag", "play", str(args.bag.resolve()), "--clock", "--rate", str(args.rate)] + controls + [
                       "--topics", source or params["camera_topic"], params["camera_info"], params["lidar_topic"]])
    elif args.command == "assess":
        runtime(args, ["python3", str(PACKAGE / "scripts/assess_jazzy.py"), str(args.csv.resolve())])
    elif args.command in ("background", "sample", "discard"):
        operation = {"background": 3, "sample": 1, "discard": 2}[args.command]
        runtime(args, ["ros2", "service", "call", "/optimiser", "cam_lidar_calibration/srv/Optimise",
                       json.dumps({"operation": operation})])
        print("Request acknowledged; watch capture logs for completion/failure before issuing another request.")
    elif args.command == "optimise":
        runtime(args, ["ros2", "action", "send_goal", "/run_optimise", "cam_lidar_calibration/action/RunOptimise", "{}"])
    else:
        params = json.loads((session / "params.yaml").read_text())["/**"]["ros__parameters"]
        if args.command == "record":
            source = json.loads((session / "input.json").read_text()).get("compressed_topic") if (session / "input.json").exists() else None
            runtime(args, ["ros2", "bag", "record", "--storage", "sqlite3", "--output", str(session / "jazzy_bag"),
                           source or params["camera_topic"], params["camera_info"], params["lidar_topic"]], overlay=False, record=True)
        elif args.command == "capture":
            source = json.loads((session / "input.json").read_text()).get("compressed_topic") if (session / "input.json").exists() else None
            runtime(args, ["ros2", "launch", "cam_lidar_calibration", "run_optimiser.launch.py",
                           f"params_file:={session}/params.yaml", f"rviz_config:={session}/capture.rviz",
                           f"output_dir:={session}/captures", f"gui:={str(args.gui).lower()}",
                           f"decode_compressed:={str(bool(source)).lower()}",
                           f"compressed_topic:={source or '/unused'}", f"image_topic:={params['camera_topic']}",
                           f"use_sim_time:={str(not args.live).lower()}"], gui=args.gui)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        sys.exit(f"calibration: {error}")
    except KeyboardInterrupt:
        sys.exit(130)
