/**
 * @file cirqueue.cpp
 * @author shlchen (3079625093@qq.com)
 * @version 0.1
 * @date 2022-03-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "include/cirqueue.h"

namespace ns_db {
  CirQueue::CirQueue(std::size_t maxSize)
      : _maxSize(maxSize + 1), _front(0), _rear(0), _data(new elem_type[maxSize + 1]) {
  }

  bool CirQueue::empty() const {
    // 循环队列空的条件
    return this->_front == this->_rear;
  }

  bool CirQueue::full() const {
    // 循环队列满的条件
    return (this->_rear + 1) % this->_maxSize == this->_front;
  }

  CirQueue &CirQueue::front(elem_type &elem) {
    // 如果队列为空, 则抛出异常
    if (this->empty()) {
      THROW(front, "the CirQueue is empty");
    }
    elem = this->_data[this->_front];
    return *this;
  }

  CirQueue &CirQueue::push(const elem_type &elem) {
    // 如果队列为满, 则抛出异常
    if (this->full()) {
      THROW(push, "the CirQueue is full");
    }
    this->_data[this->_rear] = elem;
    this->_rear = (this->_rear + 1) % this->_maxSize;
    return *this;
  }

  CirQueue &CirQueue::pop(elem_type &elem) {
    // 如果队列为空, 则抛出异常
    if (this->empty()) {
      THROW(front, "the CirQueue is empty");
    }
    elem = this->_data[this->_front];
    this->_front = (this->_front + 1) % this->_maxSize;
    return *this;
  }
  CirQueue &CirQueue ::traverse(traverse_fun fun) {
    for (int i = this->_front; i % this->_maxSize != this->_rear; ++i) {
      fun(this->_data[i]);
    }
    return *this;
  }

  CirQueue ::~CirQueue() {
    delete[] this->_data;
  }

  std::ostream &operator<<(std::ostream &os, const CirQueue &cq) {
    os << '{';
    os << "'maxSize': " << cq._maxSize << ", ";
    os << "'enableSize': " << cq._maxSize - 1 << ", ";
    os << "'front': " << cq._front << ", ";
    os << "'rear': " << cq._rear;
    os << '}';
    return os;
  }
} // namespace ns_db
