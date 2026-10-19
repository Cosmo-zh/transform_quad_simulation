#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <parameters/param.h>

#include "TransformGeometry.hpp"

#include <float.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

extern "C" __EXPORT int transform_control_main(int argc, char *argv[]);

namespace transform_control
{
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task = -1;
static px4::atomic<int> command_target{-1};
static px4::atomic<int> transform_mode{0};
static px4::atomic<int> manual_source{0};
static px4::atomic<int> transform_limit_milli{1000};
static px4::atomic<int> actuator_output_mode{0};
static px4::atomic<int> transition_time_ms{12000};
static px4::atomic<int> geometry_frame{static_cast<int>(GeometryFrame::X)};

static constexpr hrt_abstime kLoopIntervalUs = 20000;
static constexpr hrt_abstime kGeometryUpdateIntervalUs = 100000;

static constexpr float kMinTransitionTimeSec = 3.0f;
static constexpr float kMaxTransitionTimeSec = 30.0f;
static constexpr float kProgressEpsilon = 0.001f;
static constexpr float kServoOpen = 0.0f;
static constexpr float kServoFolded = 1.0f;
static constexpr int kExternalTransformActionIndex = 99;
static constexpr int kExternalTransformProgressIndex = 100;
static constexpr float kExternalFoldCommandThreshold = 0.05f;
static constexpr float kRcAuxEnableThreshold = 0.5f;
static constexpr hrt_abstime kRcAuxTimeoutUs = 500000;

enum class FoldState {
	Open,
	Folding,
	Folded,
	Unfolding,
	Partial
};

enum class TransformMode {
	Normal = 0,
	ServoOnly = 1,
	GeometryOnly = 2
};

enum class ManualSource {
	Button = 0,
	RcAux = 1
};

enum class ActuatorOutputMode {
	Progress = 0,
	PwmOpenHigh = 1,
	PwmOpenLow = 2
};

struct RotorParamHandles {
	param_t x[kRotorCount];
	param_t y[kRotorCount];
};

static float lerp(float from, float to, float progress)
{
	return from + (to - from) * progress;
}

static const char *geometry_name(GeometryFrame frame)
{
	return frame == GeometryFrame::Plus ? "plus (motor 1 front)" : "x";
}

static const char *state_name(FoldState state)
{
	switch (state) {
	case FoldState::Open: return "open";
	case FoldState::Folding: return "folding";
	case FoldState::Folded: return "folded";
	case FoldState::Unfolding: return "unfolding";
	case FoldState::Partial: return "partial";
	}

	return "unknown";
}

static const char *mode_name(TransformMode mode)
{
	switch (mode) {
	case TransformMode::Normal: return "normal";
	case TransformMode::ServoOnly: return "servo_only";
	case TransformMode::GeometryOnly: return "geometry_only";
	}

	return "unknown";
}

static const char *source_name(ManualSource source)
{
	switch (source) {
	case ManualSource::Button: return "button";
	case ManualSource::RcAux: return "rc_aux";
	}

	return "unknown";
}

static const char *output_mode_name(ActuatorOutputMode mode)
{
	switch (mode) {
	case ActuatorOutputMode::Progress: return "progress";
	case ActuatorOutputMode::PwmOpenHigh: return "pwm_open_high";
	case ActuatorOutputMode::PwmOpenLow: return "pwm_open_low";
	}

	return "unknown";
}

static float constrain_progress(float progress)
{
	if (!PX4_ISFINITE(progress)) {
		return 0.0f;
	}

	if (progress < 0.0f) {
		return 0.0f;
	}

	if (progress > 1.0f) {
		return 1.0f;
	}

	return progress;
}

static int progress_limit_milli_from_float(float progress)
{
	return static_cast<int>(lroundf(constrain_progress(progress) * 1000.0f));
}

static float progress_limit()
{
	return static_cast<float>(transform_limit_milli.load()) * 0.001f;
}

static int transition_time_ms_from_float(float transition_time_sec)
{
	return static_cast<int>(lroundf(transition_time_sec * 1000.0f));
}

static float transition_time_sec()
{
	return static_cast<float>(transition_time_ms.load()) * 0.001f;
}

static float progress_step()
{
	return static_cast<float>(kLoopIntervalUs) * 1.0e-6f / transition_time_sec();
}

static float apply_progress_limit(float progress)
{
	const float limited_progress = constrain_progress(progress);
	const float limit = progress_limit();
	return limited_progress > limit ? limit : limited_progress;
}

static FoldState state_from_progress(float progress, float target_progress)
{
	if (progress <= kProgressEpsilon && target_progress <= kProgressEpsilon) {
		return FoldState::Open;
	}

	if (progress >= 1.0f - kProgressEpsilon && target_progress >= 1.0f - kProgressEpsilon) {
		return FoldState::Folded;
	}

	if (fabsf(progress - target_progress) <= kProgressEpsilon) {
		return FoldState::Partial;
	}

	return target_progress > progress ? FoldState::Folding : FoldState::Unfolding;
}

static float target_from_manual_input(const manual_control_setpoint_s &rc_input)
{
	const ManualSource source = static_cast<ManualSource>(manual_source.load());

	if (source == ManualSource::RcAux) {
		const bool enabled = PX4_ISFINITE(rc_input.aux2) && rc_input.aux2 > kRcAuxEnableThreshold;

		if (!enabled || !PX4_ISFINITE(rc_input.aux1)) {
			return 0.0f;
		}

		return apply_progress_limit((rc_input.aux1 + 1.0f) * 0.5f);
	}

	const bool command_fold = (rc_input.buttons & (1 << 0)) != 0;
	return command_fold ? apply_progress_limit(1.0f) : 0.0f;
}

static float actuator_output_from_progress(float progress)
{
	const float constrained_progress = constrain_progress(progress);

	switch (static_cast<ActuatorOutputMode>(actuator_output_mode.load())) {
	case ActuatorOutputMode::PwmOpenHigh:
		return 1.0f - 2.0f * constrained_progress;

	case ActuatorOutputMode::PwmOpenLow:
		return -1.0f + 2.0f * constrained_progress;

	case ActuatorOutputMode::Progress:
		break;
	}

	return lerp(kServoOpen, kServoFolded, constrained_progress);
}

static bool find_rotor_params(RotorParamHandles &handles)
{
	char param_name[16];

	for (int i = 0; i < kRotorCount; ++i) {
		snprintf(param_name, sizeof(param_name), "CA_ROTOR%d_PX", i);
		handles.x[i] = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "CA_ROTOR%d_PY", i);
		handles.y[i] = param_find(param_name);

		if (handles.x[i] == PARAM_INVALID || handles.y[i] == PARAM_INVALID) {
			PX4_ERR("missing CA_ROTOR%d_PX/PY", i);
			return false;
		}
	}

	return true;
}

static void update_rotor_geometry(const RotorParamHandles &handles, float progress, GeometryFrame frame)
{
	// CA geometry is runtime state during morphing; avoid scheduling parameter
	// autosave for every interpolation step.
	param_control_autosave(false);

	for (int i = 0; i < kRotorCount; ++i) {
		const RotorPosition position = rotor_position(i, progress, frame);

		param_set_no_notification(handles.x[i], &position.x);
		param_set_no_notification(handles.y[i], &position.y);
	}

	param_control_autosave(true);
	param_notify_changes();
}

static void publish_actuator_setpoint(uORB::Publication<vehicle_command_s> &vehicle_command_pub, float progress)
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;
	command.param1 = actuator_output_from_progress(progress);
	command.param2 = NAN;
	command.param3 = NAN;
	command.param4 = NAN;
	command.param5 = NAN;
	command.param6 = NAN;
	command.param7 = 0.0f;
	command.target_system = UINT8_MAX;
	command.target_component = 0;
	command.source_system = 1;
	command.source_component = 1;
	command.from_external = false;

	// Function 301 (Peripheral via Actuator Set 1) drives the folding actuator.
	// SITL uses progress mode (0..1); hardware PWM can use [-1..1] output modes.
	vehicle_command_pub.publish(command);
}

static bool update_external_transform_command(uORB::Subscription &vehicle_command_sub)
{
	vehicle_command_s command{};
	bool updated = false;

	while (vehicle_command_sub.update(&command)) {
		if (!PX4_ISFINITE(command.param7) || command.param7 < 98.5f || command.param7 > 100.5f) {
			continue;
		}

		const int actuator_set_index = static_cast<int>(lroundf(command.param7));

		if (command.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR
		    && command.from_external
		    && (actuator_set_index == kExternalTransformActionIndex
			|| actuator_set_index == kExternalTransformProgressIndex)
		    && PX4_ISFINITE(command.param1)) {
			const bool fold = fabsf(command.param1) > kExternalFoldCommandThreshold;
			if (actuator_set_index == kExternalTransformProgressIndex) {
				if (command.param1 < 0.0f || command.param1 > 1.0f) {
					continue;
				}

				command_target.store(progress_limit_milli_from_float(command.param1));

			} else {
				command_target.store(fold ? 1000 : 0);
			}

			PX4_INFO("external target progress: %.3f", (double)command_target.load() * 0.001);
			updated = true;
		}
	}

	return updated;
}

static float update_progress(float current_progress, float target_progress)
{
	if (fabsf(current_progress - target_progress) <= kProgressEpsilon) {
		return target_progress;
	}

	if (current_progress < target_progress) {
		current_progress += progress_step();
		return current_progress > target_progress ? target_progress : current_progress;
	}

	current_progress -= progress_step();
	return current_progress < target_progress ? target_progress : current_progress;
}

int transform_thread_main(int argc, char *argv[])
{
	thread_running = true;
	const GeometryFrame frame = static_cast<GeometryFrame>(geometry_frame.load());

	uORB::Subscription manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_s> vehicle_command_pub{ORB_ID(vehicle_command)};

	RotorParamHandles rotor_params{};

	if (!find_rotor_params(rotor_params)) {
		thread_running = false;
		return PX4_ERROR;
	}

	float current_progress = 0.0f;
	float target_progress = 0.0f;
	FoldState current_state = FoldState::Open;
	hrt_abstime last_geometry_update = 0;
	hrt_abstime last_manual_input_update = 0;

	update_rotor_geometry(rotor_params, current_progress, frame);
	publish_actuator_setpoint(vehicle_command_pub, current_progress);
	PX4_INFO("geometry: %s", geometry_name(frame));

	PX4_INFO("started: state=%s mode=%s source=%s output=%s limit=%.3f transition=%.1fs",
		 state_name(current_state), mode_name(static_cast<TransformMode>(transform_mode.load())),
		 source_name(static_cast<ManualSource>(manual_source.load())),
		 output_mode_name(static_cast<ActuatorOutputMode>(actuator_output_mode.load())), (double)progress_limit(),
		 (double)transition_time_sec());

	while (!thread_should_exit) {
		update_external_transform_command(vehicle_command_sub);

		const int commanded_target = command_target.load();

		if (commanded_target >= 0) {
			target_progress = apply_progress_limit(static_cast<float>(commanded_target) * 0.001f);

		} else {
			manual_control_setpoint_s rc_input{};
			const ManualSource source = static_cast<ManualSource>(manual_source.load());
			const bool manual_updated = manual_control_sub.update(&rc_input);

			if (manual_updated) {
				last_manual_input_update = hrt_absolute_time();
			}

			if (source == ManualSource::RcAux && hrt_elapsed_time(&last_manual_input_update) > kRcAuxTimeoutUs) {
				target_progress = 0.0f;

			} else if (manual_updated && rc_input.valid) {
				target_progress = target_from_manual_input(rc_input);

			} else if (manual_updated && source == ManualSource::RcAux) {
				target_progress = 0.0f;
			}
		}

		const float previous_progress = current_progress;
		current_progress = update_progress(current_progress, target_progress);

		const FoldState new_state = state_from_progress(current_progress, target_progress);

		if (new_state != current_state) {
			current_state = new_state;
			PX4_INFO("state=%s progress=%.2f", state_name(current_state), (double)current_progress);
		}

		const TransformMode mode = static_cast<TransformMode>(transform_mode.load());

		if (mode != TransformMode::GeometryOnly) {
			publish_actuator_setpoint(vehicle_command_pub, current_progress);
		}

		const bool progress_changed = fabsf(current_progress - previous_progress) > FLT_EPSILON;
		const bool reached_target = fabsf(current_progress - target_progress) <= kProgressEpsilon;
		const hrt_abstime now = hrt_absolute_time();

		if (mode != TransformMode::ServoOnly
		    && progress_changed
		    && (hrt_elapsed_time(&last_geometry_update) >= kGeometryUpdateIntervalUs || reached_target)) {
			update_rotor_geometry(rotor_params, current_progress, frame);
			last_geometry_update = now;
		}

		px4_usleep(kLoopIntervalUs);
	}

	if (static_cast<TransformMode>(transform_mode.load()) != TransformMode::GeometryOnly) {
		publish_actuator_setpoint(vehicle_command_pub, current_progress);
	}

	thread_running = false;
	PX4_INFO("stopped: state=%s progress=%.2f", state_name(current_state), (double)current_progress);
	return PX4_OK;
}
}

int transform_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (transform_control::thread_running) {
			return 0;
		}

		transform_control::thread_should_exit = false;
		transform_control::daemon_task = px4_task_spawn_cmd("transform_control",
						    SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2500,
						    transform_control::transform_thread_main, nullptr);
		return transform_control::daemon_task < 0 ? PX4_ERROR : PX4_OK;
	}

	if (!strcmp(argv[1], "geometry")) {
		if (argc >= 3) {
			if (transform_control::thread_running) {
				PX4_ERR("geometry must be configured before starting the module");
				return PX4_ERROR;
			}

			if (!strcmp(argv[2], "x")) {
				transform_control::geometry_frame.store(static_cast<int>(transform_control::GeometryFrame::X));

			} else if (!strcmp(argv[2], "plus")) {
				transform_control::geometry_frame.store(static_cast<int>(transform_control::GeometryFrame::Plus));

			} else {
				PX4_ERR("unknown geometry: %s (use x or plus)", argv[2]);
				return PX4_ERROR;
			}
		}

		PX4_INFO("geometry: %s", transform_control::geometry_name(
				 static_cast<transform_control::GeometryFrame>(transform_control::geometry_frame.load())));
		return PX4_OK;
	}

	if (!strcmp(argv[1], "stop")) {
		transform_control::thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "fold")) {
		transform_control::command_target.store(1000);
		PX4_INFO("command target: fold");
		return 0;
	}

	if (!strcmp(argv[1], "target")) {
		char *end = nullptr;
		const float progress = argc >= 3 ? strtof(argv[2], &end) : NAN;

		if (!PX4_ISFINITE(progress) || end == argv[2] || *end != '\0' || progress < 0.0f || progress > 1.0f) {
			PX4_ERR("usage: transform_control target <0..1>");
			return PX4_ERROR;
		}

		transform_control::command_target.store(transform_control::progress_limit_milli_from_float(progress));
		PX4_INFO("command target progress: %.3f (subject to limit)", (double)progress);
		return PX4_OK;
	}

	if (!strcmp(argv[1], "open") || !strcmp(argv[1], "unfold")) {
		transform_control::command_target.store(0);
		PX4_INFO("command target: open");
		return 0;
	}

	if (!strcmp(argv[1], "manual")) {
		transform_control::command_target.store(-1);
		PX4_INFO("command target: manual input source %s",
			 transform_control::source_name(
				 static_cast<transform_control::ManualSource>(transform_control::manual_source.load())));
		return 0;
	}

	if (!strcmp(argv[1], "source")) {
		if (argc < 3) {
			PX4_INFO("source: %s",
				 transform_control::source_name(
					 static_cast<transform_control::ManualSource>(transform_control::manual_source.load())));
			return 0;
		}

		if (!strcmp(argv[2], "button")) {
			transform_control::manual_source.store(static_cast<int>(transform_control::ManualSource::Button));

		} else if (!strcmp(argv[2], "rc_aux")) {
			transform_control::manual_source.store(static_cast<int>(transform_control::ManualSource::RcAux));

		} else {
			PX4_ERR("unknown source: %s", argv[2]);
			return 1;
		}

		transform_control::command_target.store(-1);
		PX4_INFO("source: %s",
			 transform_control::source_name(
				 static_cast<transform_control::ManualSource>(transform_control::manual_source.load())));
		return 0;
	}

	if (!strcmp(argv[1], "limit")) {
		if (argc < 3) {
			PX4_INFO("limit: %.3f", (double)transform_control::progress_limit());
			return 0;
		}

		char *end = nullptr;
		const float limit = strtof(argv[2], &end);

		if (end == argv[2] || !PX4_ISFINITE(limit) || limit < 0.0f || limit > 1.0f) {
			PX4_ERR("limit must be in range 0..1");
			return 1;
		}

		transform_control::transform_limit_milli.store(transform_control::progress_limit_milli_from_float(limit));
		PX4_INFO("limit: %.3f", (double)transform_control::progress_limit());
		return 0;
	}

	if (!strcmp(argv[1], "transition")) {
		if (argc < 3) {
			PX4_INFO("transition: %.1fs", (double)transform_control::transition_time_sec());
			return 0;
		}

		char *end = nullptr;
		const float transition_time_sec = strtof(argv[2], &end);

		if (end == argv[2] || !PX4_ISFINITE(transition_time_sec)
		    || transition_time_sec < transform_control::kMinTransitionTimeSec
		    || transition_time_sec > transform_control::kMaxTransitionTimeSec) {
			PX4_ERR("transition must be in range %.1f..%.1f seconds",
				(double)transform_control::kMinTransitionTimeSec,
				(double)transform_control::kMaxTransitionTimeSec);
			return 1;
		}

		transform_control::transition_time_ms.store(
			transform_control::transition_time_ms_from_float(transition_time_sec));
		PX4_INFO("transition: %.1fs", (double)transform_control::transition_time_sec());
		return 0;
	}

	if (!strcmp(argv[1], "output")) {
		if (argc < 3) {
			PX4_INFO("output: %s",
				 transform_control::output_mode_name(
					 static_cast<transform_control::ActuatorOutputMode>(transform_control::actuator_output_mode.load())));
			return 0;
		}

		if (!strcmp(argv[2], "progress")) {
			transform_control::actuator_output_mode.store(static_cast<int>(transform_control::ActuatorOutputMode::Progress));

		} else if (!strcmp(argv[2], "pwm_open_high")) {
			transform_control::actuator_output_mode.store(static_cast<int>(transform_control::ActuatorOutputMode::PwmOpenHigh));

		} else if (!strcmp(argv[2], "pwm_open_low")) {
			transform_control::actuator_output_mode.store(static_cast<int>(transform_control::ActuatorOutputMode::PwmOpenLow));

		} else {
			PX4_ERR("unknown output: %s", argv[2]);
			return 1;
		}

		PX4_INFO("output: %s",
			 transform_control::output_mode_name(
				 static_cast<transform_control::ActuatorOutputMode>(transform_control::actuator_output_mode.load())));
		return 0;
	}

	if (!strcmp(argv[1], "mode")) {
		if (argc < 3) {
			PX4_INFO("mode: %s",
				 transform_control::mode_name(
					 static_cast<transform_control::TransformMode>(transform_control::transform_mode.load())));
			return 0;
		}

		if (!strcmp(argv[2], "normal")) {
			transform_control::transform_mode.store(static_cast<int>(transform_control::TransformMode::Normal));

		} else if (!strcmp(argv[2], "servo_only")) {
			transform_control::transform_mode.store(static_cast<int>(transform_control::TransformMode::ServoOnly));

		} else if (!strcmp(argv[2], "geometry_only")) {
			transform_control::transform_mode.store(static_cast<int>(transform_control::TransformMode::GeometryOnly));

		} else {
			PX4_ERR("unknown mode: %s", argv[2]);
			return 1;
		}

		PX4_INFO("mode: %s",
			 transform_control::mode_name(
				 static_cast<transform_control::TransformMode>(transform_control::transform_mode.load())));
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		PX4_INFO("geometry: %s", transform_control::geometry_name(
				 static_cast<transform_control::GeometryFrame>(transform_control::geometry_frame.load())));
		const int command_target = transform_control::command_target.load();
		const transform_control::TransformMode mode =
			static_cast<transform_control::TransformMode>(transform_control::transform_mode.load());
		const transform_control::ManualSource source =
			static_cast<transform_control::ManualSource>(transform_control::manual_source.load());
		const transform_control::ActuatorOutputMode output_mode =
			static_cast<transform_control::ActuatorOutputMode>(transform_control::actuator_output_mode.load());

		if (command_target >= 0) {
			PX4_INFO("%s, mode: %s, source: %s, output: %s, limit: %.3f, transition: %.1fs, command target progress: %.3f",
				 transform_control::thread_running ? "running" : "stopped",
				 transform_control::mode_name(mode),
				 transform_control::source_name(source),
				 transform_control::output_mode_name(output_mode),
				 (double)transform_control::progress_limit(),
				 (double)transform_control::transition_time_sec(),
				 (double)command_target * 0.001);

		} else {
			PX4_INFO("%s, mode: %s, source: %s, output: %s, limit: %.3f, transition: %.1fs, command target: manual",
				 transform_control::thread_running ? "running" : "stopped",
				 transform_control::mode_name(mode),
				 transform_control::source_name(source),
				 transform_control::output_mode_name(output_mode),
				 (double)transform_control::progress_limit(),
				 (double)transform_control::transition_time_sec());
		}

		return 0;
	}

	return 1;
}
