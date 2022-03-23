/**
 * @file main.cpp
 * @author shlchen (3079625093@qq.com)
 * @version 0.1
 * @date 2022-03-23
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "include/linklist.h"

void print(int &elem) {
  std::cout << elem << ' ';
  return;
}

int main(int argc, char const *argv[]) {
  try {
    int array[] = {1, 2, 3, 4};
    // 构造一个含有四个元素的链表
    ns_db::LinkList ls(array, 4);

    // 打印链表
    ls.traverse(print);
    /**
     * @brief
     * 1 2 3 4
     */
    std::cout << std::endl;

    int elem;
    // 将链表下标为"3"的元素删除(下标从"0"开始), 并打印链表
    ls.erase(elem, 3).traverse(print);
    std::cout << std::endl;
    /**
     * @brief 输出结果, 其中'node-deconstructor'表示节点被删除, 内存被释放
     * 'node-deconstructor'
     * 1 2 3
     */

    // 先后在链表的两个位置插入两个元素, 并打印链表
    ls.insert(12, 2).insert(34, 0).traverse(print);
    std::cout << std::endl;
    /**
     * @brief
     * 34 1 2 12 3
     */

    /**
     * @brief 输出结果, 其中'node-deconstructor'表示节点被删除, 内存被释放
     * 'node-deconstructor'
     * 'node-deconstructor'
     * 'node-deconstructor'
     * 'node-deconstructor'
     * 'node-deconstructor'
     * 'node-deconstructor'
     */

  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }
  return 0;
}
