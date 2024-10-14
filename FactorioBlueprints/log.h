#pragma once

#include <iostream>

#ifndef LOG
#define LOG(level, ...)  \
	if (LOG_##level##_ENABLED) { \
		std::cout << __VA_ARGS__; \
	}
#endif
