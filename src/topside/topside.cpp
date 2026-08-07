#include "dss/runtime.h"
#include "dss/DSS.h"

#include <memory>
#include <iostream>

#include "networking/dyver/server.h"

#include <cstring>

#include "app.h"
#include "cli/cli.h"
#include "topside/core/rov.h"

void command_line(DSS::cli_t *p_cli) { p_cli->init(); }

auto main(int argc, char **argv) -> int
{
	(void)argc;
	(void)argv;

	// This sucks, but I'm too lazy to do anything better
	bool headless = false;
	if (argc > 1)
	{
		if (strcmp(argv[1], "--headless") == 0)
		{
			headless = true;
		}

		if (strcmp(argv[1], "--help") == 0)
		{
			std::cout << "Usage: DyverTopside [OPTION]\n--headless\n\tRun CLI without the GUI\n\n--help\n\tShow this menu\n";
			return 0;
		}
	}

	// DSS
	DSS::environment_t env = DSS::environment_t();

	env.init();
	std::shared_ptr<DSS::executor_t> main_executor = env.main_executor();

	if (main_executor == nullptr)
	{
		return 1;
	}

	std::shared_ptr<rov_t> rov = std::make_shared<rov_t>();
	rov->load_from_cache();
	utils::log("Initialization completed.");
	std::this_thread::sleep_for(std::chrono::seconds(1));

	server_t server = server_t();

	cli_t cli = cli_t("Dyver Tospide CLI");
	cli.get_on_input()->connect(
		[&server, rov](std::string got)
		{
			if (got == "server-verify")
			{
				server.verify_connection();
			}

			if (got == "server-start")
			{
				server.init();
			}

			if (got == "server-kill")
			{
				server.kill();
			}

			if (got == "rov-reload")
			{
				rov->load_from_cache();
			}
		});

	cli.init(headless);

	if (headless == false)
	{
		// Dyver
		app_t app = app_t(main_executor);

		// Run Dyver
		app.run();
		std::cout << std::endl;

		// Ew
		std::terminate();
	}

	return 0;
}