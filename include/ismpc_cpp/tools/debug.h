#pragma once

#include <iostream>

#ifdef DEBUG
#define PRINT(msg) std::cout << msg << std::endl
#else
#define PRINT(msg)
#endif
