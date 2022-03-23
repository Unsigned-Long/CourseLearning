/**
 * @file main.cpp
 * @author shlchen (3079625093@qq.com)
 * @version 0.1
 * @date 2022-03-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "include/cirqueue.h"

void print(int &elem) {
  std::cout << elem << ' ';
  return;
}

int main(int argc, char const *argv[]) {
  try {
    // 构造最大长度为4的队列
    ns_db::CirQueue cq(4);
    // 4个元素依次入队, 并遍历输出队列
    cq.push(12).push(23).push(34).push(13).traverse(print);
    std::cout << std::endl;
    // 输出该队列的状态信息
    std::cout << cq << std::endl
              << std::endl;
    /**
     * @brief output
     * 12 23 34 13
     * {'maxSize': 5, 'enableSize': 4, 'front': 0, 'rear': 4}
     */

    int top;
    // 元素出队
    cq.pop(top).traverse(print);
    std::cout << "the top elem is: " << top << std::endl;
    // 输出该队列的状态信息
    std::cout << cq << std::endl
              << std::endl;
    /**
     * @brief output
     * the top elem is: 12
     * {'maxSize': 5, 'enableSize': 4, 'front': 1, 'rear': 4}
     */

    cq.push(43);
    cq.traverse(print);
    // 输出该队列的状态信息
    std::cout << cq << std::endl
              << std::endl
              << std::endl;
    /**
     * @brief output
     * 23 34 13 43
     * {'maxSize': 5, 'enableSize': 4, 'front': 1, 'rear': 0}
     */

    // 元素出队
    cq.pop(top);
    std::cout << "the top elem is: " << top << std::endl;
    cq.traverse(print);
    // 输出该队列的状态信息
    std::cout << cq << std::endl;
    std::cout << std::endl
              << std::endl;

    /**
     * @brief output
     * the top elem is: 23
     * 34 13 43
     * {'maxSize': 5, 'enableSize': 4, 'front': 2, 'rear': 0}
     */
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }
  return 0;
}
