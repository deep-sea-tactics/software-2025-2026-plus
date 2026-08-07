#ifndef H_ROV
#define H_ROV

#include <memory>

#include "denseutils.h"
#include "cli/cli.h"
#include "geometry/rectprism.h"

#include "amp_distribution.h"
#include "thruster/thruster.h"

class rov_t
{
public:
	explicit rov_t() {}
	~rov_t() {}

	void load_from_cache();

	/**
	 * @brief Create a thruster object
	 *
	 * @param pos Position of the thruster
	 * @param look Look vector of the thruster (thrusters are really simple!)
	 */
	void create_thruster(const Eigen::Vector3d pos_m, const Eigen::Vector3d look, const double force_N, const std::string id)
	{
		ensure_is_unit(look);

		std::shared_ptr<abstract_thruster_t> new_thruster = std::make_shared<abstract_thruster_t>(pos_m, look, force_N);
		m_thrusters[id] = new_thruster;
	}

	/**
	 * @brief Sum unbalanced torque produced by thrusters
	 *
	 * @param factor_throttle Whether or not to consider the current throttles of every thruster
	 * @return Eigen::Vector3d Nm unbalanced torque
	 */
	auto calculate_unbalanced_torque(const bool factor_throttle = false) -> Eigen::Vector3d
	{
		if (m_thrusters.size() == 0)
		{
			utils::log("(rov) No thrusters, cannot calculate torque.", utils::MSG_TYPE::WARN);
		}

		Eigen::Vector3d sum = Eigen::Vector3d(0, 0, 0);

		for (auto [id, thruster] : m_thrusters)
		{
			Eigen::Vector3d &look = thruster->get_look();
			Eigen::Vector3d &pos = thruster->get_pos();

			Eigen::Vector3d tau_Nm = pos.cross(look);

			if (factor_throttle)
			{
				tau_Nm *= thruster->get_output();
			}

			sum += tau_Nm;
		}

		return sum;
	}

	/**
	 * @brief Optimize the values of thrusters to control the ROV's unbalanced force output direction and torque
	 *
	 * @param target_translational_N Target force direction for linear forces
	 * @param target_rotational_Nm  Target force direction for rotational (torque) forces
	 */
	void optimize_throttle_config(Eigen::Vector3d target_translational_N, Eigen::Vector3d target_rotational_Nm);

	/**
	 * @brief Obtain a reference to the internal thruster map of this ROV
	 *
	 * @return thrusters_t&
	 */
	auto get_thrusters() -> thrusters_t & { return m_thrusters; }

private:
	thrusters_t m_thrusters = {};
	std::shared_ptr<amp_distributor_t> m_amp_distributor;

	/**
	 * @brief The geometric shape of the ROV, not the physical shape. Utilities
	 * for geometric properties such as the centroid.
	 *
	 */
	std::shared_ptr<rectprism_t> m_shape;

	/**
	 * @brief Optimize a specific thruster towards a target.
	 *
	 * @param which
	 * @param target
	 */
	void optimize_thruster(std::shared_ptr<abstract_thruster_t> which, Eigen::Vector3d &target, Eigen::Vector3d &target_rotational);
};

#endif