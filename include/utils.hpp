/************************************
* @file message.hpp
* @details Message formats
* @author Arghadeep Mazumder
* @version 1.0.0
* @copyright -
************************************/
#pragma once

#include "colors.hpp"
#include <iostream>

#define ENDLINE (std::cout << std::endl)

// -------- WARNING --------
/**
* @brief
* @tparam T
* @param t
* @details
*/
template <typename T> void WARNING(T t) {
 std::cout << BOLD << RED << t << std::endl;
}

/**
* @brief
* @tparam T
* @tparam Args
* @param t
* @param args
* @details
*/
template <typename T, typename... Args> void WARNING(T t, Args... args) {
 std::cout << BOLD << RED << t << std::endl;
 WARNING(args...);
}

// -------- STATUS --------
/**
* @brief
* @tparam T
* @tparam C
* @param t
* @param c
* @details
*/
template <typename T, typename C> void STATUS(T t, C c) {
 std::cout << c << t << std::endl;
}

/**
* @brief
* @tparam T
* @tparam C
* @tparam Args
* @param t
* @param c
* @param args
* @details
*/
template <typename T, typename C, typename... Args>
void STATUS(T t, C c, Args... args) {
 std::cout << c << t << std::endl;
 STATUS(c, args...);
}

// -------- MESSAGE --------
/**
* @brief
* @tparam T
* @param t
* @details
*/
template <typename T> void MESSAGE(T t) { std::cout << CYAN << t << std::endl; }

/**
* @brief
* @tparam T
* @tparam Args
* @param t
* @param args
* @details
*/
template <typename T, typename... Args> void MESSAGE(T t, Args... args) {
 std::cout << CYAN << t;
 MESSAGE(args...);
}

// -------- PRINT --------
/**
* @brief
* @tparam T
* @param t
* @details
*/
template <typename T> void PRINT(T t) {
 std::cout << MAGENTA << t << " " << RESET;
}

/**
* @brief
* @tparam T
* @tparam Args
* @param t
* @param args
* @details
*/
template <typename T, typename... Args> void PRINT(T t, Args... args) {
 std::cout << MAGENTA << t << " ";
 PRINT(args...);
 std::cout << std::endl;
}

/**
* @brief
* @tparam T
* @param t
* @details
*/
template <typename T> void PRINT_DATA(T t) { std::cout << GREEN << t << ", "; }

/**
* @brief Function to print data in an array or vector
* @tparam T
* @tparam Args
* @param t
* @param args
* @details Output format 1, 2, 3,
*          Usage: PRINT_DATA(val)
*/
template <typename T, typename... Args> void PRINT_DATA(T t, Args... args) {
 std::cout << GREEN << t << ", ";
 PRINT_DATA(args...);
}

/**
*
* @tparam T
* @param vec
*/
template <typename T> void PRINT_VECTOR(const T &vec) {
 std::cout << GREEN << "Elements: ";
 for (const auto &elem : vec)
   std::cout << elem << " ";
 std::cout << RESET << std::endl;
}