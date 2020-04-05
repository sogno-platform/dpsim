#pragma once

#if defined(__GNUC__) && !defined(__clang__)
  #include <cxxabi.h>
#endif

#include <cps/Definitions.h>
#include <cps/Logger.h>

namespace CPS {
namespace Utils {

	template<typename T>
	String className(T *ptr, const String &prefix = "CPS::") {
		const char *mangled;
		mangled = typeid(*ptr).name();
#if !defined(__GNUC__) || defined(__clang__)
		String ret(mangled);
		return ret;
#else
		Int status = 1;
		const char *unmangled;
		unmangled = abi::__cxa_demangle(mangled, NULL, NULL, &status);

		if (status) {
			return mangled;
		}
		else {
			String ret(unmangled);
			free((void *) unmangled);
			if (ret.find(prefix) == 0)
				return ret.substr(prefix.size());
			else
				return ret;
		}
#endif
	}

	struct Rgb {
		double r, g, b;

		static Rgb gradient(double x) {
			if (x > 1.0) x = 1.0;

			Rgb c =  {2.0 * x, 2.0 * (1 - x), 0.0 };

			if (c.r > 1.0) c.r = 1.0;
			if (c.g > 1.0) c.g = 1.0;

			return c;
		}

		String hex() {
			return fmt::format("#{:02x}{:02x}{:02x}",
				(unsigned) (r * 255),
				(unsigned) (g * 255),
				(unsigned) (b * 255)
			);
		}
	};
}
}
