#pragma once
#include <string>

#define ERR_HEADER ("[" + std::string(__FILE__).substr(std::string(__FILE__).find_last_of("/")+1) + ":" + std::to_string(uint(__LINE__)) + "] ")