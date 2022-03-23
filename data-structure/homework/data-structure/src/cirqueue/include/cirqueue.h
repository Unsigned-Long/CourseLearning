/**
 * @file cirqueue.h
 * @author shlchen (3079625093@qq.com)
 * @version 0.1
 * @date 2022-03-22
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <iostream>

namespace ns_db {
/**
 * @brief 定义宏,用于在线性链表中方便的抛出异常
 */
#define THROW(where, what) \
  throw std::runtime_error(std::string("[ error from 'CirQueue'-'") + #where + "' ] " + what)

  /**
   * @brief 用 "elem_type" 来表示元素的类型, 如果必要, 可很方便的将非模板类扩展成模板类
   */
  using elem_type = int;
  /**
   * @brief 表示遍历的函数类型, 传入一个元素, 没有返回值
   */
  using traverse_fun = void (*)(elem_type &);

  /**
   * @brief 表示循环队列的数据结构
   */
  class CirQueue {
  public:
    friend std::ostream &operator<<(std::ostream &os, const CirQueue &cq);

  private:
    // 最大元素个数, 可存储的元素个数为maxSize - 1
    const std::size_t _maxSize;
    // 头节点的编号
    std::size_t _front;
    // 尾节点的编号
    std::size_t _rear;
    // 节点数组
    elem_type *_data;

  public:
    /**
     * @brief 构造可存储maxSize个元素的循环队列
     *
     * @param maxSize 循环队列可存储的元素个数
     */
    CirQueue(std::size_t maxSize = 10);

    /**
     * @brief 得到循环队列的头部元素
     *
     * @param elem 头部元素
     * @return CirQueue& 该循环队列
     */
    CirQueue &front(elem_type &elem);

    /**
     * @brief 判断循环队列是否为空
     *
     * @return true 为空
     * @return false 不为空
     */
    bool empty() const;

    /**
     * @brief 判断循环队列是否为满
     *
     * @return true 为满
     * @return false 不为满
     */
    bool full() const;

    /**
     * @brief 往循环队列中压入一个元素
     *
     * @param elem 压入的元素
     * @return CirQueue& 该循环队列
     */
    CirQueue &push(const elem_type &elem);

    /**
     * @brief 往循环队列中压出一个元素
     *
     * @param elem 出队的元素
     * @return CirQueue& 该循环队列
     */
    CirQueue &pop(elem_type &elem);

    /**
     * @brief 遍历循环队列
     *
     * @param fun 访问函数
     * @return CirQueue& 该循环队列
     */
    CirQueue &traverse(traverse_fun fun);

    ~CirQueue();
  };

  /**
   * @brief 输出循环队列的具体细节
   */
  std::ostream &operator<<(std::ostream &os, const CirQueue &cq);
} // namespace ns_db
