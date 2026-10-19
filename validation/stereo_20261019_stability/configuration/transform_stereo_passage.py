#!/usr/bin/env python3
"""Marked-gate stereo prototype. Default is perception only; --execute flies SITL."""
import argparse
from collections import deque
import json
import math
import struct
from pathlib import Path
import time

import numpy as np
from transform_stereo_perception import required_progress, fit_heading_offset, lateral_allowance
from transform_stereo_vision_worker import VisionWorker


class Feedback:
    def __init__(self, node):
        from gz.msgs10.model_pb2 import Model
        from gz.msgs10.contacts_pb2 import Contacts
        self.joints = None
        self.joint_time = 0
        self.gate_contacts = 0
        self.contact_messages = 0
        self.node = node
        self.servo_commands = [None]*4
        self.servo_times = [0.]*4
        self.callbacks = []
        node.subscribe(Model, '/transform/joints', self.on_joints)
        node.subscribe(Contacts, '/transform/contacts', self.on_contacts)
        from gz.msgs10.double_pb2 import Double
        for i in range(4):
            def receive(message, i=i):
                self.servo_commands[i] = message.data
                self.servo_times[i] = time.monotonic()
            self.callbacks.append(receive)
            node.subscribe(Double, f'/model/transform_quad_plus_stereo_0/servo_{i}', receive)

    def on_joints(self, msg):
        angles = {j.name: j.axis1.position for j in msg.joint}
        if all(f'arm{i}_Joint' in angles for i in range(1, 5)):
            self.joints = np.array([angles[f'arm{i}_Joint'] for i in range(1, 5)])
            self.joint_time = time.monotonic()

    def on_contacts(self, msg):
        self.contact_messages += 1
        for contact in msg.contact:
            if 'gate_' in contact.collision1.name or 'gate_' in contact.collision2.name:
                self.gate_contacts += 1

    def at_progress(self, progress):
        return (self.joints is not None and time.monotonic()-self.joint_time < .5
                and np.max(np.abs(self.joints+math.pi/3*progress)) < .025)


class Vehicle:
    def __init__(self, endpoint):
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.link = mavutil.mavlink_connection(endpoint, source_system=245)
        heartbeat = None
        deadline = time.monotonic()+20
        while time.monotonic() < deadline and heartbeat is None:
            if endpoint.startswith('udpout:'):
                self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            heartbeat = self.link.wait_heartbeat(timeout=1)
        if heartbeat is None:
            raise RuntimeError('No PX4 heartbeat')
        self.last_heartbeat = 0
        self.system = self.link.target_system
        self.component = self.link.target_component
        self.position = None
        self.velocity = np.zeros(3)
        self.attitude = None
        self.position_time = self.attitude_time = 0
        self.custom_mode = 0
        self.armed = False
        for message, hz in [(32, 30), (30, 30)]:
            self.command(511, message, 1e6/hz)
        self.link.mav.param_request_read_send(self.system, self.component, b'SYS_AUTOSTART', -1)
        self.autostart = None

    def command(self, command, *args):
        self.link.mav.command_long_send(self.system, self.component, command, 0,
                                        *(list(args)+[0]*(7-len(args))))

    def poll(self):
        if time.monotonic()-self.last_heartbeat > 1:
            self.link.mav.heartbeat_send(self.mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                         self.mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_heartbeat = time.monotonic()
        while True:
            msg = self.link.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_srcSystem() != self.system:
                continue
            kind = msg.get_type()
            if kind == 'LOCAL_POSITION_NED':
                self.position = np.array([msg.x, msg.y, msg.z])
                self.velocity = np.array([msg.vx, msg.vy, msg.vz])
                self.position_time = time.monotonic()
            elif kind == 'ATTITUDE':
                self.attitude = np.array([msg.roll, msg.pitch, msg.yaw])
                self.attitude_time = time.monotonic()
            elif kind == 'HEARTBEAT':
                self.custom_mode = msg.custom_mode
                self.armed = bool(msg.base_mode & 128)
            elif kind == 'PARAM_VALUE' and msg.param_id == 'SYS_AUTOSTART':
                # PX4 encodes INT32 parameters bytewise in the float wire field.
                self.autostart = (struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
                                  if msg.param_type == 6 else round(msg.param_value))

    def setpoint(self, xyz, yaw):
        # Position and yaw active; ignore velocity, acceleration and yaw rate.
        self.link.mav.set_position_target_local_ned_send(
            0, self.system, self.component, 1, 0b100111111000,
            *map(float, xyz), 0, 0, 0, 0, 0, 0, float(yaw), 0)

    def fold(self, progress):
        self.command(187, progress, 0, 0, 0, 0, 0, 100)


def body_rotation(attitude):
    roll, pitch, yaw = attitude
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cp*cy, sr*sp*cy-cr*sy, cr*sp*cy+sr*sy],
                     [cp*sy, sr*sp*sy+cr*cy, cr*sp*sy-sr*cy],
                     [-sp, sr*cp, cr*cp]])


def run(args):
    from gz.transport13 import Node
    node = Node()
    feedback = Feedback(node)
    vehicle = Vehicle(args.endpoint) if args.execute else None
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    if args.contact_probe:
        deadline = time.monotonic()+args.duration
        while time.monotonic() < deadline and not feedback.gate_contacts:
            time.sleep(.05)
        result = dict(success=feedback.gate_contacts > 0, mode='contact_probe',
                      gate_contacts=feedback.gate_contacts, contact_messages=feedback.contact_messages)
        (output / 'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result), flush=True)
        return 0 if result['success'] else 1
    state, started, entered = 'WAIT', time.monotonic(), time.monotonic()
    observations = deque(maxlen=15)
    target = yaw = gate_position = gate_direction = None
    commanded_position = None
    progress, passed = 0.0, 0
    planned_width = None
    cross_allowance = args.margin
    cross_paused = False
    cross_center_since = None
    last_cross_gate = None
    last_cross_time = 0
    last_gate = 0
    last_fold = 0
    last_telemetry = 0
    heading_offset = 0.0
    alignment_valid = False
    alignment_positions, alignment_vectors = deque(maxlen=300), deque(maxlen=300)
    cross_errors = deque(maxlen=5)
    last_cross_error = 0.
    converged_since = None
    outcome = {'success': False, 'mode': 'fold_check' if args.fold_check else ('flight' if args.execute else 'perception')}
    last_tick = time.monotonic()
    max_loop_interval = 0.
    previous_loop = last_tick
    vision = VisionWorker(output, args.image_interval)
    with (output / 'events.jsonl').open('w') as log:
        def event(kind, **fields):
            entry = dict(time=time.monotonic()-started, event=kind, state=state, **fields)
            log.write(json.dumps(entry)+'\n')
            log.flush()
            if kind not in ('measurement', 'telemetry'):
                print(json.dumps(entry), flush=True)

        def transition(new):
            nonlocal state, entered, converged_since
            state, entered, converged_since = new, time.monotonic(), None
            observations.clear()
            if new in ('ACQUIRE', 'CROSS'):
                alignment_positions.clear()
                alignment_vectors.clear()
                cross_errors.clear()
            event('state')

        try:
            while time.monotonic()-started < args.duration:
                now = time.monotonic()
                elapsed = (now-entered)*args.sim_speed
                max_loop_interval = max(max_loop_interval, now-previous_loop)
                previous_loop = now
                if vehicle:
                    vehicle.poll()
                    if now-last_telemetry > .2 and vehicle.position is not None:
                        event('telemetry', position=vehicle.position.tolist(), velocity=vehicle.velocity.tolist(),
                              attitude=vehicle.attitude.tolist() if vehicle.attitude is not None else None,
                              joints=feedback.joints.tolist() if feedback.joints is not None else None,
                              servo_commands=feedback.servo_commands.copy(),
                              gate_contacts=feedback.gate_contacts)
                        last_telemetry = now
                gate = vision.next_gate(allow_clipped=state in ('VERIFY', 'CROSS'))
                if gate:
                    last_gate = now
                    event('measurement', **gate.as_dict())
                    if not vehicle:
                        observations.append(gate)
                        if len(observations) >= args.samples:
                            outcome.update(success=True, samples=len(observations),
                                           median_width=float(np.median([g.width for g in observations])))
                            break
                    elif vehicle.position is not None and vehicle.attitude is not None:
                        # Camera midpoint FLU (.08, 0, .08) -> body FRD (.08, 0, -.08).
                        relative = body_rotation(vehicle.attitude) @ np.array(
                            [gate.forward+.08, gate.right, gate.down-.08])
                        if state == 'APPROACH' and gate.forward > 1.1:
                            alignment_positions.append(vehicle.position.copy())
                            alignment_vectors.append(relative.copy())
                            alignment = fit_heading_offset(alignment_positions, alignment_vectors)
                            if alignment is not None:
                                heading_offset, residual = alignment
                                if not alignment_valid:
                                    event('heading_alignment', offset=heading_offset, residual=residual)
                                alignment_valid = True
                        rotation = body_rotation([0, 0, heading_offset])
                        center = vehicle.position + rotation @ relative
                        if state == 'VERIFY' and not gate.full_height:
                            # Close-range strips can be vertically clipped.
                            # Retain the full-height measurement from the stop.
                            center[2] = gate_position[2]
                        if (state == 'CROSS' and planned_width is not None
                                and abs(gate.width-planned_width) < .03 and gate.forward < 2):
                            last_cross_gate, last_cross_time = gate, now
                            cross_errors.append(gate.lateral_error)
                            last_cross_error = float(np.median(cross_errors))
                            # Image-center error alone hides sideways body offset
                            # when the camera is yawed. Servo in the measured gate
                            # plane, rather than relying on the distant pose fit.
                            normal = vehicle.attitude[2]+gate.normal_yaw+heading_offset
                            tangent = np.array([-math.sin(normal), math.cos(normal), 0.])
                            correction = .15*(last_cross_error-np.dot(target-vehicle.position, tangent))*tangent
                            if gate.full_height:
                                correction[2] = .15*(center[2]-gate_position[2])
                            gate_position += correction
                            target += correction
                        if state in ('ACQUIRE', 'MEASURE', 'VERIFY'):
                            observations.append((gate, center, vehicle.attitude[2]+gate.normal_yaw+heading_offset))
                if not vehicle:
                    time.sleep(.02)
                    continue
                if vehicle.autostart not in (None, 5103):
                    raise RuntimeError('Flight allowed only for SYS_AUTOSTART=5103')
                if state == 'WAIT':
                    if (vehicle.autostart == 5103 and vehicle.position is not None and vehicle.attitude is not None):
                        target = vehicle.position.copy()
                        commanded_position = target.copy()
                        target[2] -= args.altitude
                        yaw = float(vehicle.attitude[2])
                        last_tick = now
                        transition('PRIME')
                    elif elapsed > 20:
                        raise RuntimeError('Missing telemetry or 5103 identity')
                else:
                    if now-vehicle.position_time > 1 or now-vehicle.attitude_time > 1:
                        raise RuntimeError('Stale PX4 telemetry')
                    delta = target-commanded_position
                    horizontal = np.linalg.norm(delta[:2])
                    speed = args.cross_speed if state == 'CROSS' else args.speed
                    step = .05*args.sim_speed
                    if horizontal > speed*step:
                        delta[:2] *= speed*step/horizontal
                    delta[2] = np.clip(delta[2], -.3*step, .3*step)
                    commanded_position += delta
                    vehicle.setpoint(commanded_position, yaw)
                    if now-last_tick > .5:
                        raise RuntimeError('Offboard loop exceeded 500 ms')
                    last_tick = now
                    if state == 'PRIME' and elapsed > 2:
                        vehicle.link.mav.set_mode_send(vehicle.system, 1, 6 << 16)
                        vehicle.command(400, 1)
                        transition('TAKEOFF')
                    elif state == 'TAKEOFF':
                        if elapsed > 15 and not vehicle.armed:
                            raise RuntimeError('Arming failed')
                        if np.linalg.norm(vehicle.position-target) < .10 and np.linalg.norm(vehicle.velocity) < .08:
                            if args.fold_check:
                                progress = .875
                                vehicle.fold(progress)
                                last_fold = now
                                transition('FOLD')
                            else:
                                transition('ACQUIRE')
                    elif state in ('ACQUIRE', 'MEASURE') and len(observations) >= 12:
                        gates = [o[0] for o in observations]
                        if np.ptp([g.width for g in gates]) > .03:
                            observations.popleft()
                            continue
                        gate_position = np.median([o[1] for o in observations], axis=0)
                        normal = float(np.median([o[2] for o in observations]))
                        gate_direction = np.array([math.cos(normal), math.sin(normal), 0.])
                        if state == 'ACQUIRE':
                            yaw = normal-heading_offset
                            target = gate_position-1.4*gate_direction
                            transition('APPROACH')
                        else:
                            # Refine the stop with the closer stereo measurement;
                            # a distant pose estimate is not accurate enough to cross.
                            refined_stop = gate_position-1.4*gate_direction
                            if np.linalg.norm(vehicle.position-refined_stop) > .025:
                                target = refined_stop
                                yaw = normal-heading_offset
                                transition('APPROACH')
                                continue
                            width = float(np.median([g.width for g in gates]))
                            planned_width = width
                            if not alignment_valid:
                                raise RuntimeError('Visual/odometry heading alignment not established')
                            if min(g.height for g in gates) < .35:
                                raise RuntimeError('Insufficient measured gate height')
                            uncertainty = max(g.uncertainty for g in gates)
                            progress = required_progress(width, args.margin, uncertainty)
                            if progress is not None:
                                cross_allowance = lateral_allowance(width, progress, uncertainty)
                            event('plan', width=width, uncertainty=uncertainty, progress=progress,
                                  gate_position=gate_position.tolist(), direction=gate_direction.tolist(),
                                  heading_offset=heading_offset, lateral_allowance=cross_allowance)
                            if progress is None:
                                raise RuntimeError('Insufficient width with configured margin')
                            vehicle.fold(progress)
                            last_fold = now
                            transition('FOLD')
                    elif state == 'APPROACH':
                        if np.linalg.norm(vehicle.position-target) < .04 and np.linalg.norm(vehicle.velocity) < .04:
                            transition('MEASURE')
                    elif state == 'FOLD':
                        if now-last_fold > 1:
                            vehicle.fold(progress)
                            last_fold = now
                        if feedback.at_progress(progress) and np.linalg.norm(vehicle.velocity) < .04:
                            if converged_since is None:
                                converged_since = now
                            if (now-converged_since)*args.sim_speed > 2:
                                if args.fold_check:
                                    event('fold_verified', progress=progress, joints=feedback.joints.tolist(),
                                          servo_commands=feedback.servo_commands.copy())
                                    vehicle.fold(0)
                                    transition('OPEN')
                                else:
                                    transition('VERIFY')
                        else:
                            converged_since = None
                        if elapsed > 25 and not feedback.at_progress(progress):
                            raise RuntimeError(f'Folding timeout: commands={feedback.servo_commands}, '
                                               f'joints={feedback.joints}')
                        if elapsed > 60:
                            raise RuntimeError(f'Settling timeout: velocity={vehicle.velocity.tolist()}')
                    elif state == 'VERIFY':
                        # Folding can move the body while the local hold target
                        # remains fixed. Re-observe and center after the mechanism
                        # has settled instead of trusting the pre-fold gate pose.
                        if len(observations) >= 12:
                            gates = [o[0] for o in observations]
                            if abs(float(np.median([g.width for g in gates]))-planned_width) > .025:
                                raise RuntimeError('Gate identity/width changed after folding')
                            gate_position = np.median([o[1] for o in observations], axis=0)
                            normal = float(np.median([o[2] for o in observations]))
                            gate_direction = np.array([math.cos(normal), math.sin(normal), 0.])
                            # After folding, inspect closer to reduce disparity
                            # quantization in the gate-plane lateral estimate.
                            target = gate_position-.95*gate_direction
                            yaw = normal-heading_offset
                            centered = (abs(float(np.median([g.lateral_error for g in gates]))) < .012
                                        and np.linalg.norm(vehicle.position-target) < .025
                                        and np.linalg.norm(vehicle.velocity) < .035
                                        and feedback.at_progress(progress))
                            if centered:
                                if converged_since is None:
                                    converged_since = now
                                if (now-converged_since)*args.sim_speed > 2:
                                    transition('CROSS')
                            else:
                                converged_since = None
                        if elapsed > 35:
                            raise RuntimeError('Post-fold visual centering timeout')
                    elif state == 'CROSS':
                        # Freeze the measured gate pose before its edges leave the image.
                        # Use PX4 odometry for this short traverse; never use Gazebo world pose.
                        if not feedback.at_progress(progress):
                            raise RuntimeError('Lost joint tracking while crossing')
                        along = float(np.dot(vehicle.position-gate_position, gate_direction))
                        cross_track = vehicle.position-gate_position-along*gate_direction
                        visible = last_cross_gate is not None and now-last_cross_time < .3
                        # Stop while both edges will remain visible after braking.
                        # At <0.75 m the remaining image width is needed to finish
                        # the traverse; starting another stop can lose both edges.
                        can_stop = visible and last_cross_gate.forward > .75
                        if not cross_paused and can_stop and abs(last_cross_error) > cross_allowance*.5:
                            cross_paused = True
                            cross_center_since = None
                            target -= np.dot(target-vehicle.position, gate_direction)*gate_direction
                            event('cross_pause', right=last_cross_gate.right, lateral_error=last_cross_error,
                                  distance=last_cross_gate.forward)
                        if cross_paused:
                            if not visible:
                                raise RuntimeError('Lost gate during near-door centering')
                            yaw += np.clip(.5*last_cross_gate.normal_yaw, -.1, .1)*step
                            if (abs(last_cross_error) < .008 and np.linalg.norm(vehicle.velocity) < .04
                                    and abs(last_cross_gate.normal_yaw) < math.radians(2)):
                                if cross_center_since is None:
                                    cross_center_since = now
                                if (now-cross_center_since)*args.sim_speed > 1:
                                    cross_paused = False
                                    event('cross_resume', right=last_cross_gate.right, lateral_error=last_cross_error)
                            else:
                                cross_center_since = None
                        if ((np.linalg.norm(cross_track[:2]) > cross_allowance and not (cross_paused and can_stop))
                                or abs(cross_track[2]) > .10):
                            raise RuntimeError(f'Cross-track error exceeds allowance: {cross_track.tolist()}')
                        if not cross_paused:
                            target = target+gate_direction*args.cross_speed*step
                        if float(np.dot(target-gate_position, gate_direction)) > .8:
                            target = gate_position+.8*gate_direction
                        if along > .7 and np.linalg.norm(vehicle.position-target) < .05:
                            passed += 1
                            event('passed', count=passed, contacts=feedback.gate_contacts)
                            vehicle.fold(0)
                            transition('OPEN')
                    elif state == 'OPEN':
                        if feedback.at_progress(0):
                            if args.fold_check or passed >= args.gates:
                                vehicle.command(21)
                                transition('LAND')
                            else:
                                transition('ACQUIRE')
                    elif state == 'LAND':
                        if not vehicle.armed:
                            outcome.update(success=feedback.gate_contacts == 0, gates=passed,
                                           gate_contacts=feedback.gate_contacts,
                                           contact_messages=feedback.contact_messages)
                            break
                    if state not in ('PRIME', 'TAKEOFF', 'LAND') and vehicle.custom_mode >> 16 & 255 != 6:
                        raise RuntimeError('PX4 left Offboard mode')
                    if not args.fold_check and state in ('APPROACH', 'MEASURE', 'FOLD', 'VERIFY') and now-last_gate > 2:
                        raise RuntimeError('Lost gate perception before crossing')
                    if state not in ('WAIT', 'PRIME', 'TAKEOFF', 'LAND') and max(abs(vehicle.attitude[:2])) > math.radians(8):
                        raise RuntimeError('Excessive roll/pitch')
                    if feedback.gate_contacts:
                        raise RuntimeError('Gate contact detected')
                time.sleep(max(0, .05-(time.monotonic()-now)))
            else:
                raise RuntimeError('Run timeout')
        except (Exception, KeyboardInterrupt) as exc:
            outcome['error'] = str(exc) or 'Interrupted'
            outcome['abort_feedback'] = dict(
                state=state, joints=feedback.joints.tolist() if feedback.joints is not None else None,
                servo_commands=feedback.servo_commands.copy(),
                velocity=vehicle.velocity.tolist() if vehicle else None,
                attitude=vehicle.attitude.tolist() if vehicle and vehicle.attitude is not None else None)
            event('abort', reason=outcome['error'])
            if vehicle and vehicle.armed:
                # PX4 takes over landing; do not unfold near a gate on an abort.
                vehicle.command(21)
                deadline = time.monotonic()+20/args.sim_speed
                while vehicle.armed and time.monotonic() < deadline:
                    vehicle.poll()
                    time.sleep(.1)
        finally:
            vision.close()
            outcome['max_loop_interval'] = max_loop_interval
            outcome['servo_commands'] = feedback.servo_commands
            outcome['joints'] = feedback.joints.tolist() if feedback.joints is not None else None
            outcome['gate_contacts'] = feedback.gate_contacts
            outcome['contact_messages'] = feedback.contact_messages
            (output / 'result.json').write_text(json.dumps(outcome, indent=2)+'\n')
    print(json.dumps(outcome), flush=True)
    return 0 if outcome['success'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Arm and fly the isolated 5103 SITL')
    parser.add_argument('--fold-check', action='store_true', help='Hover folding diagnostic, with --execute')
    parser.add_argument('--contact-probe', action='store_true', help='Positive contact test only; does not fly')
    parser.add_argument('--endpoint', default='udpin:127.0.0.1:14640')
    parser.add_argument('--output', default='build/transform_stereo_latest')
    parser.add_argument('--duration', type=float, default=240)
    parser.add_argument('--sim-speed', type=float, default=1)
    parser.add_argument('--samples', type=int, default=12)
    parser.add_argument('--altitude', type=float, default=1.6)
    parser.add_argument('--margin', type=float, default=.04, help='Per-side planning margin, m')
    parser.add_argument('--speed', type=float, default=.10)
    parser.add_argument('--cross-speed', type=float, default=.20)
    parser.add_argument('--image-interval', type=float, default=1.0,
                        help='Save a stereo image pair every N wall seconds; 0 disables the sequence')
    parser.add_argument('--gates', type=int, default=2)
    args = parser.parse_args()
    if args.fold_check and not args.execute:
        parser.error('--fold-check requires --execute')
    if (not 0 < args.speed <= .2 or not 0 < args.cross_speed <= .2 or not 0 < args.sim_speed <= 1
            or args.margin < .02 or args.gates < 1 or not 1 <= args.samples <= 15
            or not math.isfinite(args.image_interval) or args.image_interval < 0):
        parser.error('Invalid speed, margin, gate count or sample count')
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
