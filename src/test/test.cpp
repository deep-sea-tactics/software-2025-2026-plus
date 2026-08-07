#include <chrono>
#include <cmath>
#include <string>
#include <thread>

#define FLAG_DYVER_TEST

#include "topside/core/amp_distribution.h"

#include "networking/iosock.h"
#include "networking/networking_key.h"

#include "networking/dyver/client.h"
#include "networking/dyver/server.h"
#include "cache/cache_manager.h"

#include "DSS.h"

#include "topside/core/rov.h"

#include "utils.h"
#include "strutils.h"

#include "test.h"

#include "robot/driver/esc/BESCR3_T200.h"

static const test_t TEST_TEST = test_t("test_test", __LINE__,
	[]()
	{
		if (1 + 1 == 2)
		{
			return true;
		}
		else
		{
			return false;
		}
	});

static const test_t TEST_AMP_DISTRIBUTOR = test_t("test_amp_distributor", __LINE__,
	[]()
	{
		amp_distributor_t distributor = amp_distributor_t(10.0);
		std::shared_ptr<dynamic_amp_request_t> req_a = distributor.invoke_request(5.0, AMP_REQUEST_PRIORITY::ALWAYS_FULFILL);
		std::shared_ptr<dynamic_amp_request_t> req_b = distributor.invoke_request(3.0, AMP_REQUEST_PRIORITY::DISTRIBUTE);
		std::shared_ptr<dynamic_amp_request_t> req_c = distributor.invoke_request(2.0, AMP_REQUEST_PRIORITY::DISTRIBUTE);

		distributor.compute();

		/*
		// Debug purposes
		std::cout << req_a->get_allowance() << std::endl;
		std::cout << req_b->get_allowance() << std::endl;
		std::cout << req_c->get_allowance() << std::endl;
		*/

		if (req_a->get_allowance() != 5.0)
			return false;
		if (req_b->get_allowance() != 3.0)
			return false;
		if (req_c->get_allowance() != 2.0)
			return false;

		std::shared_ptr<dynamic_amp_request_t> req_d = distributor.invoke_request(6.0, AMP_REQUEST_PRIORITY::DISTRIBUTE);

		/*
		std::cout << req_d->get_allowance() << std::endl;
		std::cout << req_a->get_allowance() << std::endl;
		std::cout << req_b->get_allowance() << std::endl;
		std::cout << req_c->get_allowance() << std::endl;
		*/

		if (req_a->get_allowance() != 5.0)
			return false;
		if (approx_eq(req_b->get_allowance(), 1.6667, 0.1) == false)
			return false;
		if (approx_eq(req_c->get_allowance(), 1.6667, 0.1) == false)
			return false;
		if (approx_eq(req_d->get_allowance(), 1.6667, 0.1) == false)
			return false;

		return true;
	});

static const test_t TEST_PWM_THROTTLE = test_t("test_pwm_throttle", __LINE__,
	[]()
	{
		utils::linear_percentage_t throttle = utils::linear_percentage_t(1000.0, 1100.0);
		double p = throttle.to_percentage(1050.0);

		if (p != 0.5)
			return false;

		p = throttle.to_percentage(1075.0);

		if (p != 0.75)
			return false;

		double v = throttle.to_value(p);

		if (v != 1075.0)
			return false;

		double sgn_p = throttle.sgn_to_percentage(1025.0);

		if (sgn_p != -0.5)
			return false;

		sgn_p = throttle.sgn_to_percentage(1075.0);

		if (sgn_p != 0.5)
			return false;

		double sgn_v = throttle.sgn_to_value(-0.5);

		if (sgn_v != 1025.0)
			return false;

		sgn_v = throttle.sgn_to_value(0.5);

		if (sgn_v != 1075.0)
			return false;

		return true;
	});

static const test_t TEST_ABSTRACT_ROV = test_t("test_abstract_rov", __LINE__,
	[]()
	{
		rov_t rov = rov_t();
		const double FORCE = 28.63542; // Roughly approximation of a BlueRobotics T200 Thruster maximmum output

		// A terribly impractical ROV... but mathematically simple and easy to test
		rov.create_thruster({4.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, FORCE, "EastUp");
		rov.create_thruster({-4.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, FORCE, "WestUp");
		rov.create_thruster({0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}, FORCE, "ForwardForward");
		rov.create_thruster({0.0, 0.0, -1.0}, {0.0, 0.0, 1.0}, FORCE, "BackForward");

		rov.optimize_throttle_config({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});

		if (rov.get_thrusters()["EastUp"]->get_target_congruence() != 1.0)
			return false;

		if (rov.get_thrusters()["WestUp"]->get_target_congruence() != -1.0)
			return false;

		if (rov.get_thrusters()["ForwardForward"]->get_target_congruence() != 0.0 || rov.get_thrusters()["BackForward"]->get_target_congruence() != 0.0)
			return false;

		rov.optimize_throttle_config({0.0, 1.0, 0.0}, {0.0, 0.0, 0.0});

		if (rov.get_thrusters()["EastUp"]->get_target_congruence() != 1.0)
			return false;

		if (rov.get_thrusters()["WestUp"]->get_target_congruence() != 1.0)
			return false;

		if (rov.get_thrusters()["ForwardForward"]->get_target_congruence() != 0.0 || rov.get_thrusters()["BackForward"]->get_target_congruence() != 0.0)
			return false;

		rov.optimize_throttle_config({0.0, 0.0, 1.0}, {0.0, 0.0, 0.0});

		if (rov.get_thrusters()["EastUp"]->get_target_congruence() != 0.0 || rov.get_thrusters()["WestUp"]->get_target_congruence() != 0.0)
			return false;

		if (rov.get_thrusters()["ForwardForward"]->get_target_congruence() != 1.0)
			return false;

		if (rov.get_thrusters()["BackForward"]->get_target_congruence() != 1.0)
			return false;

		rov.create_thruster({1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, FORCE, "HorribleDesignChoice");
		if (rov.calculate_unbalanced_torque().y() != -1.0)
		{
			return false;
		}

		return true;
	});

static const test_t TEST_DELEGATE = test_t("test_delegate", __LINE__,
	[]()
	{
		bool res = false;
		std::shared_ptr<delegate_t<std::function<void()>>> del = std::make_shared<delegate_t<std::function<void()>>>();
		del->connect([&res]() { res = true; });
		del->call();

		return res;
	});

static const test_t TEST_IOSOCK = test_t("test_iosock", __LINE__,
	[]()
	{
		iosock_t peer_a;
		iosock_t peer_b;

		peer_a.init(PORT_PLAINTEXT, PORT_PLAINTEXT, true, true, "127.0.0.1", "127.0.0.5");
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		peer_b.init(PORT_PLAINTEXT, PORT_PLAINTEXT, true, true, "127.0.0.5", "127.0.0.1");
		std::this_thread::sleep_for(std::chrono::seconds(2));

		std::string tosendb = "Meow!";
		std::string tosenda = "Woof!";
		std::string recieved = "";

		peer_b.get_onrx()->connect(
			[&recieved](std::string msg)
			{
				recieved = msg;
				std::cout << "Peer b Recieved: " << msg << std::endl;
			});

		peer_a << tosendb;

		std::this_thread::sleep_for(std::chrono::seconds(1));

		if (recieved != tosendb)
		{
			return false;
		}

		peer_a.get_onrx()->connect(
			[&recieved](std::string msg)
			{
				recieved = msg;
				std::cout << "Peer a recieved: " << msg << std::endl;
			});

		peer_b << tosenda;

		std::this_thread::sleep_for(std::chrono::seconds(1));

		if (recieved != tosenda)
		{
			return false;
		}

		peer_a.kill();
		peer_b.kill();

		std::this_thread::sleep_for(std::chrono::seconds(1));

		return true;
	});

static const test_t TEST_CLIENT_SERVER = test_t("test_client_server", __LINE__,
	[]()
	{
		client_t client;
		server_t server;

		server.init();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		client.init();

		std::this_thread::sleep_for(std::chrono::seconds(3));

		bool res = server.verify_connection();

		std::this_thread::sleep_for(std::chrono::seconds(3));

		server.kill();
		client.kill();

		return res;
	});

static const test_t TEST_CACHE_MANAGER = test_t("test_cache_manager", __LINE__,
	[]()
	{
		std::string nest_res = nest_scopes({"thruster", "test"});
		if (nest_res != "thruster.test")
		{
			return false;
		}

		cache_manager_t manager = cache_manager_t("test");
		manager.write_buf("test_k", "test_p");
		manager.write_buf("test_k2", "Multiword cache");

		for (auto &[k, p] : *manager.get_cache())
		{
			std::cout << k << " : " << p << std::endl;
		}

		manager.rebuild_cache();
		manager.load_cache();

		if (manager.read_buf("test_k") != "test_p")
		{
			utils::log("\"test_k\" does not equal \"test_p\". It instead equals: " + manager.read_buf("test_k"), utils::MSG_TYPE::ERROR);
			return false;
		}

		if (manager.read_buf("test_k2") != "Multiword cache")
		{
			utils::log("\"test_k2\" does not equal \"Multiword cache\". It instead equals: " + manager.read_buf("test_k2"), utils::MSG_TYPE::ERROR);
			return false;
		}

		manager.write_buf("test.nested1", "quite nested");
		manager.write_buf("test.nested2", "birds");

		manager.rebuild_cache();
		manager.load_cache();

		// Note: Scope is different from the beginning of the string, so the first two entries are omitted
		std::vector<std::string> all_test = manager.read_all_with_scope("test");

		for (const auto &element : all_test)
		{
			std::cout << element << std::endl;
		}
		if (all_test[0] != "quite nested")
		{
			return false;
		}

		if (all_test[1] != "birds")
		{
			return false;
		}

		return true;
	});

static const test_t TEST_UTILS = test_t("test_utils", __LINE__,
	[]()
	{
		std::string s = "Please split	my\nstring.";
		std::vector<std::string> split_res = string_split_whitespace(s);
		const std::vector<std::string> split_comp = {"Please", "split", "my", "string."};
		if (split_res != split_comp)
		{
			return false;
		}

		for (const auto &s : split_res)
		{
			std::cout << s << " ";
		}
		std::cout << "\n\n";

		std::uint8_t b1 = 0b00000000;
		std::uint8_t b2 = 0b00000000;

		// 0000000000000000
		// 0

		if (utils::tc_tw(b1, b2) != 0b0000000000000000)
		{
			return false;
		}

		b1 = 0b11111111;
		b2 = 0b11111111;

		// 1111111111111111
		// -1

		if (utils::tc_tw(b1, b2) != -0b0000000000000001)
		{
			return false;
		}

		b1 = 0b11111110;
		b2 = 0b11111111;

		// 1111111111111110

		if (utils::tc_tw(b1, b2) != -0b0000000000000010)
		{
			return false;
		}

		return true;
	});

static const test_t TEST_CLOSEST_VALUE_TO = test_t("test_closest_value_to", __LINE__,
	[]()
	{
		std::vector<double> test_values = {-2.0, -1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
		double res = utils::closest_value_to(2.6, test_values);

		std::cout << res << std::endl;

		if (res != 3.0)
		{
			return false;
		}

		res = utils::closest_value_to(1.1, test_values);
		std::cout << res << std::endl;

		if (res != 1.0)
		{
			return false;
		}

		res = utils::closest_value_to(-2.2, test_values);
		std::cout << res << std::endl;

		if (res != -2.0)
		{
			return false;
		}

		return true;
	});

static const test_t TEST_BESCR3_T200_MATHS = test_t("test_bescr3_t200_maths", __LINE__,
	[]()
	{
		t200_data_t data = t200_data_t();
		data.load(VOLTAGE_MODE::V10);

		int pwm_us = data.get_pwm_us_from_force_kgf_approx(-2.31);
		std::cout << "pwm from -2.31 kfg: " << pwm_us << std::endl;

		if (pwm_us != 1100)
		{
			return false;
		}

		double current_A = data.get_current_A_from_pwm_us_approx(1100);
		std::cout << current_A << std::endl;

		if (current_A != 13.620)
		{
			return false;
		}

		return true;
	});

auto main() -> int
{
	std::cout << "Commencing Dyver Tests" << std::endl;

	console_clear();

	int passed_tests = 0;
	int failed_tests = 0;

	TEST_TEST.run(&passed_tests, &failed_tests);
	TEST_AMP_DISTRIBUTOR.run(&passed_tests, &failed_tests);
	TEST_PWM_THROTTLE.run(&passed_tests, &failed_tests);
	TEST_ABSTRACT_ROV.run(&passed_tests, &failed_tests);
	TEST_DELEGATE.run(&passed_tests, &failed_tests);
	// TEST_IOSOCK.run(&passed_tests, &failed_tests);
	// TEST_CLIENT_SERVER.run(&passed_tests, &failed_tests);
	TEST_CACHE_MANAGER.run(&passed_tests, &failed_tests);
	TEST_UTILS.run(&passed_tests, &failed_tests);
	TEST_CLOSEST_VALUE_TO.run(&passed_tests, &failed_tests);
	TEST_BESCR3_T200_MATHS.run(&passed_tests, &failed_tests);

	std::cout << passed_tests << " tests passed" << std::endl;
	std::cout << failed_tests << " tests failed" << std::endl;

	return 0;
}