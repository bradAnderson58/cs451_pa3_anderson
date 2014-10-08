
#include <cassert>

#include "Character.hpp"
#include "Skeleton.hpp"
#include "Quaternion.h"
using namespace mathtool;

namespace Character {

Vector3d rotate_by_yaw(Vector3d const &in, float const yaw) {
	float c = cosf(yaw);
	float s = sinf(yaw);
	return Vector3d(s * in[2] + c * in[1], 0, c * in[2] - s * in[1]);
}

void Control::clear() {
	desired_velocity = Vector3d(0.0f, 0.0f, 0.0f);
	desired_turning = 0;
	jump = false;
}

void Control::apply_to(State &state, float timestep) const {
	state.position = state.position+ rotate_by_yaw(desired_velocity * timestep, state.orientation);
	state.orientation += desired_turning * timestep;
	state.jumpping = jump;
}

void State::clear() {
	jumpping = false;
	position = Vector3d(0.0f, 0.0f, 0.0f);
	orientation = 0;
}

void State::apply_to(Pose &pose) const {
	Quaternion rot = Quaternion::get(orientation, Vector3d(0.0f, 1.0f, 0.0f));
	pose.root_position = rot.rotate(pose.root_position) + position;
	pose.root_orientation = (rot  * pose.root_orientation);
}

void StateDelta::clear() {
	position = Vector3d(0.0f, 0.0f, 0.0f);
	orientation = 0;
	landed = jumpped = false;
}

void StateDelta::apply_to(State &state) const {
	if (landed) state.jumpping = false;
	if (jumpped) state.jumpping = true;
	state.position = state.position+ rotate_by_yaw(position, state.orientation);
	state.orientation += orientation;
}

void Angles::clear(unsigned int DOF, Library::Skeleton const *skel) {
	angles.clear();
	angles.resize(DOF, 0.0);
	skeleton = skel;
}

void Angles::to_pose(Pose &to) const {
	assert(skeleton);
	skeleton->build_pose(&(angles[0]), to);
}

void Pose::clear(unsigned int bones) {
	root_position = Vector3d(0.0f,0.0f,0.0f);
	root_orientation = Quaternion();

	bone_orientations.resize(bones);

	skeleton = NULL;	

	for (unsigned int i = 0; i < bones; ++i) {
		bone_orientations[i] = Quaternion();
	}
}

void Pose::to_angles(Angles &into) const {
	assert(skeleton);
	into.clear(skeleton->frame_size, skeleton);
	skeleton->get_angles(*this, &into.angles[0]);
}

Pose &Pose::operator=(Pose const &other) {
	root_position = other.root_position;
	root_orientation = other.root_orientation;
	bone_orientations = other.bone_orientations;
	skeleton = other.skeleton;
	return *this;
}

}
