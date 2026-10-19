#pragma once

#include <math.h>

namespace transform_control
{
static constexpr int kRotorCount = 4;

enum class GeometryFrame {
	X = 0,
	Plus = 1
};

struct RotorPosition {
	float x;
	float y;
};

inline RotorPosition rotor_position(int rotor_index, float progress, GeometryFrame frame)
{
	// These coordinates describe the original X frame in PX4 body axes.
	static constexpr float open_x[kRotorCount] = {0.099f, -0.099f, 0.099f, -0.099f};
	static constexpr float open_y[kRotorCount] = {0.099f, -0.099f, -0.099f, 0.099f};
	static constexpr float hinge_x[kRotorCount] = {0.040679492f, -0.040679492f, 0.037985917f, -0.037985917f};
	static constexpr float hinge_y[kRotorCount] = {0.037985917f, -0.037985917f, -0.040679492f, 0.040679492f};
	static constexpr float fold_angle_rad = 1.047197551f;
	static constexpr float cos_45 = 0.7071067811865475f;

	const float angle = fold_angle_rad * progress;
	const float cos_angle = cosf(angle);
	const float sin_angle = sinf(angle);
	const float dx = open_x[rotor_index] - hinge_x[rotor_index];
	const float dy = open_y[rotor_index] - hinge_y[rotor_index];
	RotorPosition position{
		hinge_x[rotor_index] + cos_angle * dx - sin_angle * dy,
		hinge_y[rotor_index] + sin_angle * dx + cos_angle * dy
	};

	if (frame == GeometryFrame::Plus) {
		// Rotate the entire mechanism by -45 deg: motor 1 becomes the front motor.
		const float x = cos_45 * (position.x + position.y);
		position.y = cos_45 * (position.y - position.x);
		position.x = x;
	}

	return position;
}
}
