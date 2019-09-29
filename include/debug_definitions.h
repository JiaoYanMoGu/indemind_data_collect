#ifndef DEBUG_DEFINITIONS_H
#define DEBUG_DEFINITIONS_H

#include <iostream>
#define RED "\x1B[31m"
#define GRN "\x1B[32m"
#define YEL "\x1B[33m"
#define BLUE "\x1B[34m"
#define RESET "\x1B[0m"

#define INFO(str) do { std::cout << __FILE__ << " " << __FUNCTION__<< ": Line " << __LINE__ << ": " << GRN << str \
              << RESET << std::endl; } while(0)
#define WARN(str) do { std::cout << __FILE__ << " " << __FUNCTION__<< ": Line " << __LINE__ << ": " << YEL << str \
              << RESET << std::endl; } while(0)
#define ERROR(str) do { std::cout << __FILE__ << " " << __FUNCTION__<< ": Line " << __LINE__ << ": " << RED << str \
              << RESET << std::endl; } while(0)
#define DBG(str) do { std::cout << __FILE__ << " " << __FUNCTION__<< ": Line " << __LINE__ << ": " << BLUE << str \
              << RESET << std::endl; } while(0)

#endif
