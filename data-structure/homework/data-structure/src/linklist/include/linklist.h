/**
 * @file linklist.h
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
 * @brief 定义宏,用于在线性链表中方便的抛出异常. 比如当插入元素时, 如果传入的位置不符合,
 * 则会抛出 "[ error from 'LinkList'-'insert' ] the param 'pos' is invalid" 的异常
 */
#define THROW(where, what) \
  throw std::runtime_error(std::string("[ error from 'LinkList'-'") + #where + "' ] " + what)

  /**
   * @brief 用 "elem_type" 来表示元素的类型, 如果必要, 可很方便的将非模板类扩展成模板类
   */
  using elem_type = int;
  /**
   * @brief 表示遍历的函数类型, 传入一个元素, 没有返回值
   */
  using traverse_fun = void (*)(elem_type &);

  /**
   * @brief 用来表示线性链表节点的数据结构
   */
  struct Node {
  public:
    // 数据
    elem_type _elem;
    // 指针
    Node *_next;

  public:
    /**
     * @brief 构造函数
     *
     * @param elem 数据
     * @param next 指针
     */
    Node(const elem_type &elem, Node *next);

    /**
     * @brief 析构函数: 非必要, 这里主要是为了在其中输出一些内容来检查内存是否被释放干净
     */
    ~Node();
  };

  /**
   * @brief 表示链表的数据结构
   */
  class LinkList {
  private:
    // 头指针
    Node *_head;

  public:
    /**
     * @brief 构造函数, 构造一个空的链表
     */
    LinkList();

    /**
     * @brief 构造函数, 构造一个含有指定个数的链表
     *
     * @param size 链表的初始长度
     */
    LinkList(std::size_t size);
    /**
     * @brief 构造函数, 构造一个含有指定元素的链表
     *
     * @param elems 元素的值
     * @param size 长度
     */
    LinkList(elem_type elems[], std::size_t size);

    /**
     * @brief 判断链表是否为空
     *
     * @return true 链表为空
     * @return false 链表不为空
     */
    bool empty() const;

    /**
     * @brief 往链表的指定位置插入一个元素
     *
     * @param elem 元素的值
     * @param pos 位置
     * @return LinkList& 链表本身
     */
    LinkList &insert(const elem_type &elem, std::size_t pos);

    /**
     * @brief 遍历链表
     *
     * @param fun 访问函数
     * @return LinkList& 链表本身
     */
    LinkList &traverse(traverse_fun fun);

    /**
     * @brief 删除链表的指定位置元素
     *
     * @param elem 元素的值
     * @param pos 位置
     * @return LinkList& 链表本身
     */
    LinkList &erase(elem_type &elem, std::size_t pos);

    /**
     * @brief 析构函数, 删除所有申请的内存
     */
    ~LinkList();
  };

} // namespace ns_db
