/**
 * @file main.cpp
 * @author shlchen (3079625093@qq.com)
 * @version 0.1
 * @date 2022-03-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "include/graph.h"

void print(ns_db::Vex &vex) {
  std::cout << vex._symbol <<' ';
}

int main(int argc, char const *argv[]) {
  ns_db::UDGraphInfo info[] = {
      ns_db::UDGraphInfo('1', '2'),
      ns_db::UDGraphInfo('1', '3'),
      ns_db::UDGraphInfo('2', '4'),
      ns_db::UDGraphInfo('2', '5'),
      ns_db::UDGraphInfo('4', '8'),
      ns_db::UDGraphInfo('5', '8'),
      ns_db::UDGraphInfo('3', '6'),
      ns_db::UDGraphInfo('3', '7'),
      ns_db::UDGraphInfo('6', '7')};
  ns_db::UDGraph graph(info, 9);
  std::cout << graph << std::endl;
  /**
   * @brief output
   * {'pos': 0, 'symbol': 1, 'links': [(1)-(2)-]}
   * {'pos': 1, 'symbol': 2, 'links': [(0)-(3)-(4)-]}
   * {'pos': 2, 'symbol': 3, 'links': [(0)-(6)-(7)-]}
   * {'pos': 3, 'symbol': 4, 'links': [(1)-(5)-]}
   * {'pos': 4, 'symbol': 5, 'links': [(1)-(5)-]}
   * {'pos': 5, 'symbol': 8, 'links': [(3)-(4)-]}
   * {'pos': 6, 'symbol': 6, 'links': [(2)-(7)-]}
   * {'pos': 7, 'symbol': 7, 'links': [(2)-(6)-]}
   */
  graph.traverseDFS(print);
  /**
   * @brief output
   * 1 2 4 8 5 3 6 7
   */
  std::cout << std::endl;

  return 0;
}
