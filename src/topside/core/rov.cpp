#include <algorithm>

#include "rov.h"
#include "cache/cache_manager.h"
#include "denseutils.h"

#include <Eigen/Dense>
#include <stdexcept>

/**
 * @brief Iterates through thrusters and optimizes each individual one using `rov_t::optimize_thruster`
 *
 * @param target_translational_N Passed to `rov_t::optimize_thruster`
 * @param target_rotational_Nm Passed to `rov_t::optimize_thruster`
 */
void rov_t::optimize_throttle_config(Eigen::Vector3d target_translational_N, Eigen::Vector3d target_rotational_Nm)
{
	for (auto [id, p_thruster] : m_thrusters)
	{
		optimize_thruster(p_thruster, target_translational_N, target_rotational_Nm);
	}
}

/**
 * @brief Calculates the force and torque that `which` produces, then uses `Eigen::MatrixBase::cross` and `Eigen::MatrixBase::dot` to calculate how "effective"
 * the thruster is at achieving `target_translational` and `target_rotational`
 *
 * @param which Pointer to the thruster and is used to get thruster data and store the result (`m_target_congruence`)
 * @param target_translational_N Normalized and used to compare against the thruster in a linear fashion
 * @param target_rotational_Nm Normalized and used to compare against the thruster in a rotational fashion
 */
void rov_t::optimize_thruster(std::shared_ptr<abstract_thruster_t> which, Eigen::Vector3d &target_translational_N, Eigen::Vector3d &target_rotational_Nm)
{
	Eigen::Vector3d &look = which->get_look();
	Eigen::Vector3d &pos = which->get_pos();

	double final_congruence = 0.0;

	// Thruster lookat target
	const Eigen::Vector3d lookat = target_translational_N.normalized();

	const double dot = lookat.dot(look);

	final_congruence = dot;

	const Eigen::Vector3d calc_torque = pos.cross(look);
	const Eigen::Vector3d calc_torque_direction = calc_torque.normalized();

	const Eigen::Vector3d lookat_torque = target_rotational_Nm.normalized();

	const double torque_dot = lookat_torque.dot(calc_torque_direction);

	final_congruence += torque_dot;
	final_congruence = std::clamp(final_congruence, -1.0, 1.0);

	which->get_target_congruence() = final_congruence;

	// For as disgusting as this down here is, I'm going to keep it in case it becomes useful again

	/*
	std::cout << "Congruence of the thruster:\n" << final_congruence << std::endl;
	std::cout << std::endl;
	std::cout << "Position of the thruster:\n" << pos << std::endl;
	std::cout << std::endl;
	std::cout << "Look of the thruster:\n" << look << std::endl;
	std::cout << std::endl;
	std::cout << "Calculated torque production:\n" << calc_torque << std::endl;
	std::cout << std::endl;
	*/
}

void rov_t::load_from_cache()
{
	cache_manager_t rov_cache = cache_manager_t("topside_rov_body");
	rov_cache.load_cache();
	std::vector<std::string> thruster_keys = rov_cache.get_all_keys_with_scope("thruster");
	if (thruster_keys.empty())
	{
		utils::log("No ROV configured.", utils::MSG_TYPE::ERROR);
	}

	std::vector<std::string> ids = {};

	for (const auto &key : thruster_keys)
	{
		std::vector<std::string> nested_scopes = nest_get_scopes(key);

		if (nested_scopes.empty())
		{
			return;
		}

		std::string id = nested_scopes[1]; // 0.1.2...
		if (std::find(ids.begin(), ids.end(), id) == ids.end() || ids.empty() == true)
		{
			utils::log("Found a thruster: " + id);
			ids.push_back(id);
		}
	}

	for (const auto &id : ids)
	{
		std::string nested = nest_scopes({"thruster", id});

		std::string positions = rov_cache.read_buf_or(nest_scopes({nested, "position"}), "<undefined>");
		std::string facings = rov_cache.read_buf_or(nest_scopes({nested, "facing"}), "<undefined>");

		if (positions == "<undefined>")
		{
			throw std::runtime_error("Failure to load ROV; non-existent key " + nest_scopes({nested, "position"}));
		}

		if (facings == "<undefined>")
		{
			throw std::runtime_error("Failure to load ROV; non-existent key " + nest_scopes({nested, "facing"}));
		}

		Eigen::Vector3d position_m = str_to_vector3d(positions);
		Eigen::Vector3d facing = str_to_vector3d(facings);

		utils::log("Registered thruster \"" + id + "\" with position \"" + positions + "\" and facing \"" + facings + "\"");

		create_thruster(position_m, facing, 1.0, id);
	}
}