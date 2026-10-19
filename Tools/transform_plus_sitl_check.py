#!/usr/bin/env python3
"""Run an isolated, headless Gazebo flight; keep console and ULog evidence."""
import datetime
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time
import pexpect

repo = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--passage-smoke', action='store_true', help='Load passage world without arming or flying')
args = parser.parse_args()
build = repo / "build/px4_sitl_default"
run = repo / "build" / ("transform_plus_check_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
run.mkdir()
env = os.environ.copy()
env.update(PX4_SYS_AUTOSTART="5101", PX4_SIM_MODEL="gz_transform_quad_plus",
           HEADLESS="1", GZ_PARTITION="transform_plus_validation_" + str(os.getpid()),
           PX4_GZ_MODELS=str(repo / "Tools/simulation/gz/models"),
           PX4_GZ_WORLDS=str(repo / "Tools/simulation/gz/worlds"),
           PX4_GZ_PLUGINS=str(build / "src/modules/simulation/gz_plugins"),
           PX4_GZ_SERVER_CONFIG=str(repo / "src/modules/simulation/gz_bridge/server.config"))
env["GZ_SIM_RESOURCE_PATH"] = env["PX4_GZ_MODELS"] + ":" + env["PX4_GZ_WORLDS"]
env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = env["PX4_GZ_PLUGINS"]
env["GZ_SIM_SERVER_CONFIG_PATH"] = env["PX4_GZ_SERVER_CONFIG"]
if args.passage_smoke:
    env.update(PX4_SYS_AUTOSTART='5102', PX4_SIM_MODEL='gz_transform_quad_plus_collision')
print("Evidence:", run, flush=True)
with (run / "console.txt").open("w") as log:
    child = pexpect.spawn(str(build / "bin/px4"), ["-w", str(run), str(build / "etc")],
                          env=env, encoding="utf-8", timeout=90)
    child.logfile = log

    def command(text):
        child.sendline(text)
        child.expect_exact(text + "\r\n")
        child.expect(r"pxh> ")

    def settle(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                child.read_nonblocking(65536, timeout=min(1, end - time.monotonic()))
            except pexpect.TIMEOUT:
                pass

    try:
        child.expect(r"pxh> ")
        command("transform_control status")
        if args.passage_smoke:
            settle(5)
            for query in [['gz', 'model', '--list'],
                          ['gz', 'model', '-m', 'transform_quad_plus_collision_0', '--pose']]:
                result = subprocess.run(query, env=env, text=True, capture_output=True, timeout=15, check=True)
                log.write(result.stdout)
                print(result.stdout, flush=True)
            command('listener vehicle_attitude 1')
            command('gz_bridge status')
            child.sendline('shutdown')
            child.expect(pexpect.EOF, timeout=20)
            raise SystemExit(0)
        command("param set NAV_RCL_ACT 0")
        command("param set COM_RC_IN_MODE 4")
        command("param set COM_DL_LOSS_T 300")
        command("param set MIS_TAKEOFF_ALT 3")
        settle(15)
        command("commander takeoff")
        settle(25)
        command("listener vehicle_land_detected 1")
        command("listener vehicle_local_position 1")
        for cycle in range(3):
            print("Cycle", cycle + 1, "fold", flush=True)
            command("transform_control fold")
            settle(10)
            command("param show CA_ROTOR0_PX")
            command("param show CA_ROTOR0_PY")
            command("listener vehicle_local_position 1")
            print("Cycle", cycle + 1, "open", flush=True)
            command("transform_control open")
            settle(10)
        command("commander land")
        settle(20)
        command("listener vehicle_land_detected 1")
        command("commander status")
        child.sendline("shutdown")
        child.expect(pexpect.EOF, timeout=20)
    finally:
        if child.isalive():
            child.sendline("shutdown")
            try:
                child.expect(pexpect.EOF, timeout=15)
            except (pexpect.TIMEOUT, pexpect.EOF):
                child.terminate(force=True)
        # Gazebo is launched by rcS in the PX4 session; stop that owned group.
        try:
            os.killpg(child.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
for flight_log in sorted((run / "log").rglob("*.ulg")):
    subprocess.run(["python3", str(repo / "Tools/analyze_transform_plus_log.py"), str(flight_log)], check=True)
print("Finished:", run, flush=True)
