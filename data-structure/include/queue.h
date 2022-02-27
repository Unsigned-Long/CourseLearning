#pragma once

#include "linklist.h"

namespace ns_ds {
  template <typename DataType>
  class Queue {
  public:
    using data_type = DataType;
    using traverse_fun = void (*)(data_type &);

  private:
    ns_ds::LinkList<data_type> _ls;

  public:
    Queue() = default;
    ~Queue() = default;

    void clear() {
      this->_ls.clear();
    }

    bool empty() {
      return this->_ls.empty();
    }

    int length() {
      return this->_ls.length();
    }

    bool head(data_type &head) {
      return this->_ls.getElem(0, head);
    }

    bool push(const data_type &elem) {
      return this->_ls.insertElem(this->_ls.length(), elem);
    }

    bool pop() {
      return this->_ls.deleteElem(0);
    }

    void traverse(const traverse_fun &fun) {
      return this->_ls.traverse(fun);
    }

    void status() {
      std::cout << "Queue: {'len': " << this->length();
      std::cout << ", 'elem(s)': [";
      if (!this->_ls.empty()) {
        data_type temp;
        int count = 0;
        this->_ls.getElem(count, temp);
        std::cout << temp;
        while (++count != this->_ls.length()) {
          this->_ls.getElem(count, temp);
          std::cout << ", " << temp;
        }
      }
      std::cout << "]}\n";

      return;
    }
  };
} // namespace ns_ds
