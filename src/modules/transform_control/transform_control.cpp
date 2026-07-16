#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <parameters/param.h>

#include <atomic>
#include <float.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

extern "C" __EXPORT int transform_control_main(int argc, char *argv[]);

namespace transform_control
{
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task = -1;
static std::atomic<int> command_target{-1};
static std::atomic<int> transform_mode{0};

static constexpr int kRotorCount = 4;
static constexpr hrt_abstime kLoopIntervalUs = 20000;
static constexpr hrt_abstime kGeometryUpdateIntervalUs = 100000;

static constexpr float kTransitionTimeSec = 12.0f;
static constexpr float kLoopRateHz = 50.0f;
static constexpr float kProgressStep = 1.0f / (kTransitionTimeSec * kLoopRateHz);
static constexpr float kProgressEpsilon = 0.001f;
static constexpr float kServoOpen = 0.0f;
static constexpr float kServoFolded = 1.0f;
static constexpr float kFoldAngleRad = 1.047197551f;
static constexpr int kExternalTransformActionIndex = 99;
static constexpr float kExternalFoldCommandThreshold = 0.05f;

// PX/PY are PX4 Control Allocation coordinates, not raw Gazebo SDF model coordinates.
static constexpr float kOpenX[kRotorCount] = {0.099f, -0.099f, 0.099f, -0.099f};
static constexpr float kOpenY[kRotorCount] = {0.099f, -0.099f, -0.099f, 0.099f};
static constexpr float kHingeX[kRotorCount] = {0.040679492f, -0.040679492f, 0.037985917f, -0.037985917f};
static constexpr float kHingeY[kRotorCount] = {0.037985917f, -0.037985917f, -0.040679492f, 0.040679492f};

enum class FoldState {
	Open,
	Folding,
	Folded,
	Unfolding
};

enum class TransformMode {
	Normal = 0,
	ServoOnly = 1,
	GeometryOnly = 2
};

struct RotorParamHandles {
	param_t x[kRotorCount];
	param_t y[kRotorCount];
};

static float lerp(float from, float to, float progress)
{
	return from + (to - from) * progress;
}

static const char *state_name(FoldState state)
{
	switch (state) {
	case FoldState::Open: return "open";
	case FoldState::Folding: return "folding";
	case FoldState::Folded: return "folded";
	case FoldState::Unfolding: return "unfolding";
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

static FoldState state_from_progress(float progress, float target_progress)
{
	if (progress <= kProgressEpsilon && target_progress <= kProgressEpsilon) {
		return FoldState::Open;
	}

	if (progress >= 1.0f - kProgressEpsilon && target_progress >= 1.0f - kProgressEpsilon) {
		return FoldState::Folded;
	}

	return target_progress > progress ? FoldState::Folding : FoldState::Unfolding;
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

static void rotor_position(int rotor_index, float progress, float &x, float &y)
{
	const float angle = kFoldAngleRad * progress;
	const float cos_angle = cosf(angle);
	const float sin_angle = sinf(angle);
	const float dx = kOpenX[rotor_index] - kHingeX[rotor_index];
	const float dy = kOpenY[rotor_index] - kHingeY[rotor_index];

	x = kHingeX[rotor_index] + cos_angle * dx - sin_angle * dy;
	y = kHingeY[rotor_index] + sin_angle * dx + cos_angle * dy;
}

static void update_rotor_geometry(const RotorParamHandles &handles, float progress)
{
	// CA geometry is runtime state during morphing; avoid scheduling parameter
	// autosave for every interpolation step.
	param_control_autosave(false);

	for (int i = 0; i < kRotorCount; ++i) {
		float current_x = 0.0f;
		float current_y = 0.0f;
		rotor_position(i, progress, current_x, current_y);

		param_set_no_notification(handles.x[i], &current_x);
		param_set_no_notification(handles.y[i], &current_y);
	}

	param_control_autosave(true);
	param_notify_changes();
}

static void publish_actuator_setpoint(uORB::Publication<vehicle_command_s> &vehicle_command_pub, float progress)
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;
	command.param1 = lerp(kServoOpen, kServoFolded, progress);
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

	// SIM_GZ_SV_FUNC1..4 map to function 301
	// (Peripheral via Actuator Set 1), so Actuator 1 drives all four
	// Gazebo arm joints synchronously.
	vehicle_command_pub.publish(command);
}

static bool update_external_transform_command(uORB::Subscription &vehicle_command_sub)
{
	vehicle_command_s command{};
	bool updated = false;

	while (vehicle_command_sub.update(&command)) {
		const int actuator_set_index = static_cast<int>(lroundf(command.param7));

		if (command.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR
		    && command.from_external
		    && actuator_set_index == kExternalTransformActionIndex) {
			const bool fold = fabsf(command.param1) > kExternalFoldCommandThreshold;
			command_target.store(fold ? 1 : 0);
			PX4_INFO("external command target: %s", fold ? "fold" : "open");
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
		current_progress += kProgressStep;
		return current_progress > target_progress ? target_progress : current_progress;
	}

	current_progress -= kProgressStep;
	return current_progress < target_progress ? target_progress : current_progress;
}

int transform_thread_main(int argc, char *argv[])
{
	thread_running = true;

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

	update_rotor_geometry(rotor_params, current_progress);
	publish_actuator_setpoint(vehicle_command_pub, current_progress);

	PX4_INFO("started: state=%s mode=%s transition=%.1fs",
		 state_name(current_state), mode_name(static_cast<TransformMode>(transform_mode.load())),
		 (double)kTransitionTimeSec);

	while (!thread_should_exit) {
		update_external_transform_command(vehicle_command_sub);

		const int commanded_target = command_target.load();

		if (commanded_target >= 0) {
			target_progress = commanded_target > 0 ? 1.0f : 0.0f;

		} else {
			manual_control_setpoint_s rc_input{};

			if (manual_control_sub.update(&rc_input) && rc_input.valid) {
				const bool command_fold = (rc_input.buttons & (1 << 0)) != 0;
				target_progress = command_fold ? 1.0f : 0.0f;
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
			update_rotor_geometry(rotor_params, current_progress);
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

	if (!strcmp(argv[1], "stop")) {
		transform_control::thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "fold")) {
		transform_control::command_target.store(1);
		PX4_INFO("command target: fold");
		return 0;
	}

	if (!strcmp(argv[1], "open") || !strcmp(argv[1], "unfold")) {
		transform_control::command_target.store(0);
		PX4_INFO("command target: open");
		return 0;
	}

	if (!strcmp(argv[1], "manual")) {
		transform_control::command_target.store(-1);
		PX4_INFO("command target: manual_control_setpoint.buttons bit0");
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
		const int command_target = transform_control::command_target.load();
		const transform_control::TransformMode mode =
			static_cast<transform_control::TransformMode>(transform_control::transform_mode.load());

		if (command_target >= 0) {
			PX4_INFO("%s, mode: %s, command target: %s",
				 transform_control::thread_running ? "running" : "stopped",
				 transform_control::mode_name(mode),
				 command_target > 0 ? "fold" : "open");

		} else {
			PX4_INFO("%s, mode: %s, command target: manual_control_setpoint.buttons bit0",
				 transform_control::thread_running ? "running" : "stopped",
				 transform_control::mode_name(mode));
		}

		return 0;
	}

	return 1;
}
