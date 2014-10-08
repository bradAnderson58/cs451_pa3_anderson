#ifndef CHARACTER_POSE_UTILS_HPP
#define CHARACTER_POSE_UTILS_HPP

#include "Character.hpp"
#include <Vector.h>
#include <Quaternion.h>
#include <vector>
using namespace mathtool;

namespace Character {

class WorldBones {
public:
	vector< Vector3d > bases;
	vector< Vector3d > tips;
	vector< Quaternion > orientations;
};

void get_world_bones(Pose const &pose, WorldBones &out);

float world_distance(WorldBones &a, WorldBones &b);

void lower_to_ground(Pose const &pose);

};

#endif //CHARACTER_POSE_UTILS_HPP
