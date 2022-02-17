#pragma once

#include "linklist.h"

namespace ns_ds {
  template <typename DataType>
  class Stack {
  public:
    using data_type = DataType;
    using traverse_fun = void (*)(data_type &);

  private:
    ns_ds::LinkList<data_type> _ls;

  public:
    Stack() = default;
    ~Stack() = default;

    void clear() {
      this->_ls.clear();
    }

    bool empty() {
      return this->_ls.empty();
    }

    int length() {
      return this->_ls.length();
    }

    bool top(data_type &elem) {
      return this->_ls.getElem(this->_ls.length() - 1, elem);
    }

    bool push(const data_type &elem) {
      return this->_ls.insertElem(this->_ls.length(), elem);
    }

    bool pop() {
      return this->_ls.deleteElem(this->_ls.length() - 1);
    }

    void traverse(const traverse_fun &fun) {
      return this->_ls.traverse(fun);
    }

    void status() {
      std::cout << "Stack: {'len': " << this->length();
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

} // namespace ns_db
