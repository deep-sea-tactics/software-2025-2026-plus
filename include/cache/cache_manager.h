#ifndef CACHE_MANAGER_H
#define CACHE_MANAGER_H

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <stdlib.h>

#include "strutils.h"

// clang-format off
const std::vector<std::filesystem::path> CACHE_PATHS = 
{
    "{HOME}/.dyver/cache",
    ".dyver/cache",
    "."
};
// clang-format on

const std::string LEX_DELIM = "\n";
const std::string LEX_KP = "=";
const std::string LEX_COMMENT = "#";
const std::string LEX_NEST_DELIM = ".";

const int KP_SIZE = 2;

inline auto nest_scopes(const std::vector<std::string> &args) -> std::string
{
	std::stringstream res;
	std::string sres = "";
	if (args.empty() == true)
	{
		return "";
	}

	for (const auto &s : args)
	{
		res << s << ".";
	}
	sres = res.str();
	sres.pop_back();

	return sres;
}

inline auto nest_get_scopes(std::string nested_key) -> std::vector<std::string>
{
	std::vector<std::string> tokens = string_split(nested_key, LEX_NEST_DELIM);
	return tokens;
}

inline auto nest_get_last_scope(std::string nested_key) -> std::string
{
	std::vector<std::string> tokens = nest_get_scopes(nested_key);
	if (tokens.empty())
	{
		return std::string();
	}

	return tokens.back();
}

class cache_manager_t
{
public:
	explicit cache_manager_t(const std::string name)
	{
		m_name = name;
		m_cache_buffer = std::make_shared<std::map<std::string, std::string>>();
	}
	~cache_manager_t()
	{
		if (m_file != nullptr)
		{
			if (m_file->is_open())
			{
				m_file->close();
			}
		}
	}

	auto get_cache() -> std::shared_ptr<std::map<std::string, std::string>> { return m_cache_buffer; }

	void load_file(bool truncate = false, bool clear_buffer = true)
	{
		if (clear_buffer == true)
		{
			m_cache_buffer->clear();
		}
		m_file = std::make_shared<std::fstream>();
		bool success = false;

		const char *home_var = getenv("HOME");

		for (auto path : CACHE_PATHS)
		{
			if (home_var != nullptr && path.generic_string().find("{HOME}") != std::string::npos)
			{
				std::string path_string = path.generic_string();
				string_replace(path_string, "{HOME}", home_var);
				path = std::filesystem::path(path_string);
			}

			std::filesystem::create_directories(path);
			path.append(m_name + ".cache");

			if (std::filesystem::exists(path) == false)
			{
				std::ofstream f;
				f.open(path);
				f.close();
			}
			if (truncate == false)
			{
				m_file->open(path);
			}
			else if (truncate == true)
			{
				m_file->open(path, std::ios::in | std::ios::out | std::ios::trunc);
			}

			if (m_file->is_open())
			{
				success = true;
				break;
			}
		}

		if (success == false)
		{
			throw std::runtime_error("Unable to load cache for " + m_name);
			return;
		}
	}

	void load_cache()
	{
		load_file();

		std::stringstream buf = std::stringstream();
		buf << m_file->rdbuf();
		parse_cache(buf.str());
		m_file->close();
		m_is_open = true;
	}

	void write_buf(std::string k, std::string p) { (*m_cache_buffer)[k] = p; }

	auto read_buf_or(std::string k, std::string def) -> std::string
	{
		if (is_open() == false)
		{
			return def;
		}
		if (m_cache_buffer->contains(k))
		{
			return read_buf(k);
		}

		write_buf(k, def);

		return def;
	}

	auto read_buf(std::string k) -> std::string
	{
		if (is_open() == false)
		{
			return std::string();
		}
		return (*m_cache_buffer)[k];
	}

	auto get_all_keys_with_scope(std::string k_scope_name) -> std::vector<std::string>
	{
		if (k_scope_name.ends_with(LEX_NEST_DELIM) == false)
		{
			k_scope_name += LEX_NEST_DELIM;
		}

		if (is_open() == false)
		{
			return {};
		}
		std::vector<std::string> res = {};

		if (m_cache_buffer->empty() == true)
		{
			return {};
		}

		for (auto &[k, p] : *m_cache_buffer)
		{
			if (k.starts_with(k_scope_name) == false)
			{
				continue;
			}
			res.push_back(k);
		}

		return res;
	}

	auto read_all_with_scope(std::string k_scope_name) -> std::vector<std::string>
	{
		std::vector<std::string> res = {};
		for (const auto &k : get_all_keys_with_scope(k_scope_name))
		{
			res.push_back(read_buf_or(k, "<undefined>"));
		}

		return res;
	}

	void rebuild_cache()
	{
		load_file(true, false);

		for (const auto &[k, p] : *m_cache_buffer)
		{
			std::stringstream statement = std::stringstream();

			statement << k << LEX_KP << p << "\n";

			*m_file << statement.str();
		}
		m_file->close();
	}

	auto is_open() -> bool { return (m_is_open); }

private:
	void parse_cache(std::string buf)
	{
		std::vector<std::string> split = string_split(buf, LEX_DELIM);
		std::size_t line_n = 0;

		for (const auto &line : split)
		{
			++line_n;
			if (line.empty() == true)
			{
				continue;
			}

			if (line.starts_with(LEX_COMMENT))
			{
				continue;
			}

			std::vector<std::string> tokens = string_split(line, LEX_KP);
			if (tokens.size() != KP_SIZE)
			{
				std::cout << "(cache_manager_t) Malformed key pair at line " + std::to_string(line_n) + ": (" + line + ")" << std::endl;
				return;
			}

			string_trim(tokens[0]);
			string_trim(tokens[1]);

			if (m_cache_buffer->contains(tokens[0]))
			{
				std::cout << "(cache_manager_t) Duplicate key at line " + std::to_string(line_n) << std::endl;
				return;
			}

			(*m_cache_buffer)[tokens[0]] = tokens[1];
		}
	}

	std::shared_ptr<std::fstream> m_file = nullptr;

	std::string m_name = "";
	std::shared_ptr<std::map<std::string, std::string>> m_cache_buffer = nullptr;
	bool m_is_open = false;
};

#endif // CACHE_MANAGER_H