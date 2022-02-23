#pragma once

#include <iostream>

namespace ns_ds {
  template <typename ElemType>
  class SqeList {
  public:
    using elem_type = ElemType;

    using traverse_fun = void (*)(elem_type &);

  private:
    elem_type *_head;

    int _len;
    int _max_len;

  public:
    SqeList(int init_len = 10)
        : _head(new elem_type[init_len]), _len(0), _max_len(init_len) {}

    SqeList(const elem_type *elems, int len)
        : _head(new elem_type[len]), _len(len), _max_len(len) {
      for (int i = 0; i != len; ++i) {
        this->_head[i] = elems[i];
      }
    }

    ~SqeList() { delete[] this->_head; }

    void clear() { this->_len = 0; }

    bool empty() const { return this->_len == 0; }

    int length() const { return this->_len; }

    bool getElem(int pos, elem_type &elem) {
      if (pos < 0 || pos >= this->_len) {
        return false;
      }
      elem = this->_head[pos];
      return true;
    }

    bool locateElem(const elem_type &elem, int &pos) {
      for (int i = 0; i != this->_len; ++i) {
        if (this->_head[i] == elem) {
          pos = i;
          return true;
        }
      }
      return false;
    }

    bool priorElem(const elem_type &curElem, elem_type &priorElem) {
      for (int i = 0; i < this->_len - 1; ++i) {
        if (this->_head[i + 1] == curElem) {
          priorElem = this->_head[i];
          return true;
        }
      }
      return false;
    }

    bool nextElem(const elem_type &curElem, elem_type &nextElem) {
      for (int i = 0; i < this->_len - 1; ++i) {
        if (this->_head[i] == curElem) {
          nextElem = this->_head[i + 1];
          return true;
        }
      }
      return false;
    }

    bool insertElem(int pos, const elem_type &elem) {
      if (pos < 0 || pos > this->_len) {
        return false;
      }
      if (this->_len == this->_max_len) {
        elem_type *old = this->_head;
        this->_max_len *= 2;
        this->_head = new elem_type[this->_max_len];
        for (int i = 0; i <= this->_len; ++i) {
          if (i < pos) {
            this->_head[i] = old[i];
          } else if (i == pos) {
            this->_head[i] = elem;
          } else {
            this->_head[i] = old[i - 1];
          }
        }
        delete[] old;
      } else {
        for (int i = this->_len; i >= pos; --i) {
          if (i > pos) {
            this->_head[i] = this->_head[i - 1];
          } else if (i == pos) {
            this->_head[i] = elem;
          }
        }
      }
      ++this->_len;
      return true;
    }

    bool deleteElem(int pos) {
      if (pos < 0 || pos >= this->_len) {
        return false;
      }
      for (int i = pos; i < this->_len - 1; ++i) {
        this->_head[i] = this->_head[i + 1];
      }
      --this->_len;
      return true;
    }

    void traverse(const traverse_fun &fun) {
      for (int i = 0; i != this->_len; ++i) {
        fun(this->_head[i]);
      }
      return;
    }

    void status() {
      std::cout << "SeqList: {'len': " << this->_len;
      std::cout << ", 'max-len': " << this->_max_len;
      std::cout << ", 'elem(s)': [";
      if (this->_len != 0) {
        for (int i = 0; i != this->_len - 1; ++i) {
          std::cout << this->_head[i] << ", ";
        }
        std::cout << this->_head[this->_len - 1];
      }
      std::cout << "]}\n";

      return;
    }

  private:
    SqeList(const SqeList &) = delete;
    SqeList(SqeList &&) = delete;
    SqeList &operator=(const SqeList &) = delete;
    SqeList &operator=(SqeList &&) = delete;
  };

} // namespace ns_ds
