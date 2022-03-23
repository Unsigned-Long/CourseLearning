/**
 * @file linklist.cpp
 * @author shlchen (3079625093@qq.com)
 * @version 0.1
 * @date 2022-03-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "include/linklist.h"

namespace ns_db {
  Node::Node(const elem_type &elem, Node *next)
      : _elem(elem), _next(next) {}

  Node::~Node() {
    std::cout << "'node-deconstructor'" << std::endl;
  }

  LinkList::LinkList()
      : _head(new Node(elem_type(), nullptr)) {
  }

  LinkList::LinkList(std::size_t size)
      : _head(new Node(elem_type(), nullptr)) {
    for (int i = 0; i != size; ++i) {
      this->insert(elem_type(), 0);
    }
  }

  LinkList::LinkList(elem_type elems[], std::size_t size)
      : _head(new Node(elem_type(), nullptr)) {
    for (int i = size - 1; i >= 0; --i) {
      this->insert(elems[i], 0);
    }
  }

  bool LinkList::empty() const {
    return this->_head->_next == nullptr;
  }

  LinkList &LinkList::insert(const elem_type &elem, std::size_t pos) {
    std::size_t count = 0;
    // "ptr" 指向待插入位置之前的节点
    Node *ptr = this->_head;
    while (count != pos && ptr != nullptr) {
      ptr = ptr->_next;
      ++count;
    }
    if (ptr == nullptr) {
      THROW(insert, "the param 'pos' is invalid");
    }
    Node *newNode = new Node(elem, ptr->_next);
    ptr->_next = newNode;
    return *this;
  }

  LinkList &LinkList::erase(elem_type &elem, std::size_t pos) {
    std::size_t count = 0;
    Node *ptr = this->_head;
    // "ptr" 指向待删除位置之前的节点
    while (count != pos && ptr != nullptr) {
      ptr = ptr->_next;
      ++count;
    }
    if (ptr->_next == nullptr) {
      THROW(erase, "the param 'pos' is invalid");
    }
    elem = ptr->_next->_elem;
    delete ptr->_next;
    ptr->_next = nullptr;
    return *this;
  }

  LinkList &LinkList::traverse(traverse_fun fun) {
    // "ptr" 指向待访问的节点
    Node *ptr = this->_head->_next;
    while (ptr != nullptr) {
      fun(ptr->_elem);
      ptr = ptr->_next;
    }
    return *this;
  }

  LinkList::~LinkList() {
    // "ptr" 指向待删除的节点
    Node *ptr = this->_head->_next;
    delete this->_head;
    while (ptr != nullptr) {
      Node *temp = ptr;
      ptr = ptr->_next;
      delete temp;
    }
  }
} // namespace ns_db
