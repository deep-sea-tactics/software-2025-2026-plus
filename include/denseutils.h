/**
 * @brief Dense linear algebra utilities
 * @date 2025-11-15
 */

#ifndef H_DENSE_UTILS
#define H_DENSE_UTILS

#include "cli/cli.h"
#include "strutils.h"

#include <cmath>
#include <sstream>
#include <string>

#include <Eigen/Dense>

constexpr double ONE_HALF = 0.5;

inline void ensure_is_unit(Eigen::Vector3d what)
{
	if (std::abs(what.sum()) == 1.0)
	{
		return;
	}

	std::stringstream msg;
	msg << "(is_unit) Vector is not a unit vector. (" << what.x() << " " << what.y() << " " << what.z() << ")";
	utils::log(msg.str(), utils::WARN);
}

/**
 * @brief Produce a quaternion from an euler value
 *
 * @param euler Euler's angles in the form of Roll, Pitch, and Yaw
 * @return Eigen::Quaterniond
 */
inline auto quat_from_euler(Eigen::Vector3d euler) -> Eigen::Quaterniond
{
	double cr = cos(euler.x() * ONE_HALF); // Roll
	double sr = sin(euler.x() * ONE_HALF);
	double cp = cos(euler.y() * ONE_HALF); // Pitch
	double sp = sin(euler.y() * ONE_HALF);
	double cy = cos(euler.z() * ONE_HALF); // Yaw
	double sy = sin(euler.z() * ONE_HALF);

	// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles/
	// clang-format off
	Eigen::Quaterniond res = Eigen::Quaterniond(
	cr * cp * cy + sr * sp + sy,
	sr * cp * cy - cr * sp * sy,
	cr * sp * cy + sr * cp * sy,
	cr * cp * sy - sr * sp * cy
	);
	// clang-format on

	return res;
}

/**
 * @brief Formats an `Eigen::Vector3d` as a string (x y z)
 *
 * @param which The vector to format
 * @return std::string
 */
inline auto fmt_vector3d(const Eigen::Vector3d &which) -> std::string
{
	std::stringstream fmt;
	fmt << '(';
	fmt << which.x() << ' ';
	fmt << which.y() << ' ';
	fmt << which.z();
	fmt << ')';

	return fmt.str();
}

inline auto str_to_vector3d(std::string s) -> Eigen::Vector3d
{
	Eigen::Vector3d res = Eigen::Vector3d(0, 0, 0);
	string_trim(s);
	std::vector<std::string> vec = string_split_whitespace(s);

	if (vec.size() != 3)
	{
		return Eigen::Vector3d();
	}

	res.x() = string_safe_dcast(vec[0]);
	res.y() = string_safe_dcast(vec[1]);
	res.z() = string_safe_dcast(vec[2]);

	return res;
}

#endif