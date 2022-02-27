#pragma once

#include <iostream>
#include <sstream>
#include <string>

namespace ns_ds {
  template <typename DataType>
  class BinaryTree {
  public:
    using data_type = DataType;
    using traverse_fun = void (*)(data_type &data);

  public:
    enum class Order {
      PRE_ORDER,
      IN_ORDER,
      POST_ORDER
    };

  private:
    struct Node {
      data_type _data;
      BinaryTree *_lChild;
      BinaryTree *_rChild;

      Node(BinaryTree *lChild, BinaryTree *rChild, const data_type &data)
          : _lChild(lChild), _rChild(rChild), _data(data) {}
    };

  private:
    Node *_root;

  public:
    BinaryTree()
        : _root(nullptr) {}

    BinaryTree(const data_type &data)
        : _root(new Node(nullptr, nullptr, data)) {}

    bool empty() const {
      return this->_root == nullptr;
    }

    BinaryTree *setData(const data_type &data) {
      if (this->empty()) {
        this->_root = new Node(nullptr, nullptr, data);
      } else {
        this->_root->_data = data;
      }
      return this;
    }

    BinaryTree *insertLeftChild(const data_type &data) {
      BinaryTree *newBinaryTree = new BinaryTree(data);
      newBinaryTree->_root->_lChild = this->_root->_lChild;
      this->_root->_lChild = newBinaryTree;
      return newBinaryTree;
    }

    BinaryTree *insertRightChild(const data_type &data) {
      BinaryTree *newBinaryTree = new BinaryTree(data);
      newBinaryTree->_root->_rChild = this->_root->_rChild;
      this->_root->_rChild = newBinaryTree;
      return newBinaryTree;
    }

    void traverse(const traverse_fun &fun, Order order = Order::PRE_ORDER) {
      if (this->empty()) {
        return;
      }
      switch (order) {
      case Order::PRE_ORDER: {
        fun(this->_root->_data);
        if (this->_root->_lChild != nullptr) {
          this->_root->_lChild->traverse(fun, order);
        }
        if (this->_root->_rChild != nullptr) {
          this->_root->_rChild->traverse(fun, order);
        }
      } break;
      case Order::IN_ORDER: {
        if (this->_root->_lChild != nullptr) {
          this->_root->_lChild->traverse(fun, order);
        }
        fun(this->_root->_data);
        if (this->_root->_rChild != nullptr) {
          this->_root->_rChild->traverse(fun, order);
        }
      } break;
      case Order::POST_ORDER: {
        if (this->_root->_lChild != nullptr) {
          this->_root->_lChild->traverse(fun, order);
        }
        if (this->_root->_rChild != nullptr) {
          this->_root->_rChild->traverse(fun, order);
        }
        fun(this->_root->_data);
      }
        fun(this->_root->_data);
        break;
      default:
        break;
      }
      return;
    }

    std::string toPlantUML() {
      std::stringstream stream;
      stream << "@startmindmap BinaryTree\n";
      stream << "skinparam DefaultFontName \"Ubuntu Mono\"\n";
      stream << "scale 20\n";
      this->plantUML(1, stream);
      stream << "@endmindmap\n";
      return stream.str();
    }

    ~BinaryTree() {
      if (this->empty()) {
        return;
      } else {
        delete this->_root->_lChild;
        delete this->_root->_rChild;
        delete this->_root;
      }
    }

  protected:
    void plantUML(std::size_t index, std::stringstream &stream) {
      if (this->empty()) {
        return;
      }
      stream << std::string(index, '*') << " " << this->_root->_data << "\n";
      if (this->_root->_rChild == nullptr && this->_root->_lChild == nullptr) {
        return;
      }
      if (this->_root->_rChild != nullptr) {
        this->_root->_rChild->plantUML(index + 1, stream);
      } else {
        stream << std::string(index + 1, '*') << " ^\n";
      }
      if (this->_root->_lChild != nullptr) {
        this->_root->_lChild->plantUML(index + 1, stream);
      } else {
        stream << std::string(index + 1, '*') << " ^\n";
      }
      return;
    }
  };
} // namespace ns_ds
