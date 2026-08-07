#ifndef H_UTILS
#define H_UTILS

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <string>
#include <bitset>
#include <vector>

namespace utils
{
/**
 * @brief Convert metric "milliU" to U
 *
 */
constexpr double MILLIU_T_U = (1.0e-3);

/**
 * @brief Convert metric centiU to U
 *
 */
constexpr double CENTIU_T_U = (1.0e-2);

/**
 * @brief Convert metric nanoU to U
 *
 */
constexpr double NANOU_T_U = (1.0e-9);

/**
 * @brief Convert metric kiloU to U
 *
 */
constexpr double KILOU_T_U = (1.0e3);

/**
 * @brief Convert a kilogram of force to a Newton of force
 *
 */
constexpr double kgf_T_N = 9.80665;

constexpr double DEG_TO_RAD = (M_PI / 180);

/**
 * @brief Returns the square of n
 *
 * @param n The number to take the square of
 * @return double Representing the result of the square operation
 */
inline auto square(const double n) -> double { return n * n; }

struct linear_percentage_t
{
	double m_max;
	double m_min;

	linear_percentage_t(const double min, const double max)
	{
		m_max = max;
		m_min = min;
	}

	auto to_percentage(const double v) -> double
	{
		double diff = (m_max - m_min);
		double inbetween = (v - m_min);

		return (inbetween / diff);
	}

	auto sgn_to_percentage(const double v) -> double
	{
		double diff = (m_max - m_min);

		double mid = diff / 2.0;

		double inbetween = (v - m_min);

		double res;
		linear_percentage_t pc = linear_percentage_t(mid, diff);

		// Gross, but I'm lazy

		if (inbetween < mid)
		{
			pc.m_min = 0.0;
			pc.m_max = mid;
			res = pc.to_percentage(inbetween) * -1.0;

			return res;
		}

		res = pc.to_percentage(inbetween);

		return res;
	}

	auto to_value(const double p) -> double
	{
		double diff = (m_max - m_min);
		return m_min + (p * diff);
	}

	auto sgn_to_value(const double p) -> double
	{
		double diff = (m_max - m_min);
		double mid = diff / 2.0;

		linear_percentage_t pc = linear_percentage_t(mid, diff);

		if (p < 0.0)
		{
			pc.m_min = 0.0;
			pc.m_max = mid;

			return m_min + pc.to_value(std::abs(p));
		}

		return m_min + pc.to_value(p);
	}
};

inline auto from_yn(std::string s) -> bool
{
	if (s.find("yes") != std::string::npos)
	{
		return true;
	}

	return false;
}

inline auto to_yn(bool v) -> std::string
{
	if (v == true)
	{
		return "yes";
	}

	return "no";
}

/**
 * @brief Two's complement two words
 *
 * @param b1 Word 1
 * @param b2 Word 2
 * @return std::int16_t Resulting number
 */
inline auto tc_tw(std::uint8_t b1, std::uint8_t b2) -> std::int16_t
{
	std::bitset<8> bs1 = std::bitset<8>(b1);
	std::bitset<8> bs2 = std::bitset<8>(b2);

	// Um... at least it works..
	std::bitset<16> resbs = std::bitset<16>(bs2.to_string() + bs1.to_string());

	if (resbs.any() == false)
	{
		return 0;
	}

	if (resbs[0] == 0)
	{
		return (std::int16_t)resbs.to_ulong();
	}

	resbs.flip();

	return -((std::int16_t)resbs.to_ulong() + 1);
}

template <typename N> inline auto closest_value_to(N number, std::vector<N> numbers) -> N
{
	if (numbers.empty() == true)
	{
		return 0;
	}

	std::sort(numbers.begin(), numbers.end(),
		[=](N a, N b)
		{
			N adiff = std::abs((number - a));
			N bdiff = std::abs((number - b));

			return (adiff >= bdiff);
		});

	return numbers.back();
}

template <typename N> inline auto sort_greatest_to_least(std::vector<N> &numbers) { std::sort(numbers.begin(), numbers.end(), std::greater<N>()); }
template <typename N> inline auto sort_least_to_greatest(std::vector<N> &numbers) { std::sort(numbers.begin(), numbers.end(), std::less<N>()); }
} // namespace utils

#endif