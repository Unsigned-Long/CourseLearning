#pragma once

#include <iostream>

namespace ns_ds {
template <typename DataType>
class LinkList {
 public:
  using data_type = DataType;
  using traverse_fun = void (*)(data_type &);

 private:
  struct Node {
    data_type _data;
    Node *_next;

    Node(const data_type &data, Node *next) : _data(data), _next(next) {}
  };

  Node *_head;

 public:
  LinkList() : _head(new Node(data_type(), nullptr)) {}

  ~LinkList() { this->clear(); }

  void clear() {
    Node *ptr = this->_head->_next;
    while (ptr != nullptr) {
      Node *temp = ptr->_next;
      delete ptr;
      ptr = temp;
    }
    this->_head->_next = nullptr;
  }

  bool empty() const { return this->_head->_next == nullptr; }

  int length() const {
    Node *ptr = this->_head;
    int len = 0;
    while (ptr->_next != nullptr) {
      ++len, ptr = ptr->_next;
    }
    return len;
  }

  bool getElem(int pos, data_type &elem) {
    if (pos < 0 || this->empty()) {
      return false;
    }
    int temp = -1;
    Node *ptr = this->_head;
    while ((temp != pos) && (ptr->_next != nullptr)) {
      ++temp, ptr = ptr->_next;
    }
    if (temp == pos) {
      elem = ptr->_data;
      return true;
    } else {
      return false;
    }
  }

  bool locateElem(const data_type &elem, int &pos) {
    Node *ptr = this->_head;
    int temp = -1;
    while (ptr->_next != nullptr) {
      ptr = ptr->_next;
      ++temp;
      if (ptr->_data == elem) {
        pos = temp;
        return true;
      }
    }
    return false;
  }

  bool priorElem(const data_type &curElem, data_type &priorElem) {
    Node *ptr = this->_head;
    while (ptr->_next != nullptr) {
      if (ptr->_next->_data == curElem) {
        if (ptr == this->_head) {
          return false;
        } else {
          priorElem = ptr->_data;
          return true;
        }
      }
      ptr = ptr->_next;
    }
    return false;
  }

  bool nextElem(const data_type &curElem, data_type &nextElem) {
    Node *ptr = this->_head->_next;
    while (ptr != nullptr) {
      if (ptr->_data == curElem && ptr->_next != nullptr) {
        nextElem = ptr->_next->_data;
        return true;
      }
      ptr = ptr->_next;
    }
    return false;
  }

  bool insertElem(int pos, const data_type &elem) {
    if (pos < 0) {
      return false;
    }
    Node *ptr = this->_head;
    int temp = 0;
    while (ptr != nullptr) {
      if (temp == pos) {
        Node *node = new Node(elem, ptr->_next);
        ptr->_next = node;
        return true;
      }
      ++temp;
      ptr = ptr->_next;
    }
    return false;
  }

  bool deleteElem(int pos) {
    if (pos < 0) {
      return false;
    }
    Node *ptr = this->_head;
    int temp = 0;
    while (ptr->_next != nullptr) {
      if (temp == pos) {
        Node *node = ptr->_next;
        ptr->_next = ptr->_next->_next;
        delete node;
        return true;
      }
      ++temp;
      ptr = ptr->_next;
    }
    return false;
  }

  void traverse(const traverse_fun &fun) {
    Node *ptr = this->_head;
    while (ptr->_next != nullptr) {
      ptr = ptr->_next;
      fun(ptr->_data);
    }
    return;
  }

  void status() {
    std::cout << "LinkList: {'len': " << this->length();
    std::cout << ", 'elem(s)': [";
    Node *ptr = this->_head;
    if (ptr->_next != nullptr) {
      std::cout << ptr->_next->_data;
      ptr = ptr->_next;
      while (ptr->_next != nullptr) {
        std::cout << ", " << ptr->_next->_data;
        ptr = ptr->_next;
      }
    }
    std::cout << "]}\n";

    return;
  }

 private:
  LinkList(const LinkList &) = delete;
  LinkList(LinkList &&) = delete;
  LinkList &operator=(const LinkList &) = delete;
  LinkList &operator=(LinkList &&) = delete;
};

}  // namespace ns_ds
