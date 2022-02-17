#include "linklist.h"
#include "seqlist.h"
#include "stack.h"
void test_seqlist() {
  ns_ds::SqeList<int> l(2);
  l.status();

  l.insertElem(0, 12);
  l.insertElem(0, 13);
  l.status();

  l.insertElem(0, 16);
  l.status();

  l.deleteElem(0);
  l.status();

  int elem;
  if (l.getElem(0, elem)) {
    std::cout << "the elem I get at '0' is: " << elem << std::endl;
  }

  int pos;
  if (l.locateElem(12, pos)) {
    std::cout << "the elem '12' is at: " << pos << std::endl;
  }

  if (l.priorElem(12, elem)) {
    std::cout << "12 has prior elem equals to " << elem << std::endl;
  } else {
    std::cout << "12 hasn't prior elem" << std::endl;
  }

  if (l.nextElem(12, elem)) {
    std::cout << "12 has next elem equals to " << elem << std::endl;
  } else {
    std::cout << "12 hasn't next elem" << std::endl;
  }

  l.traverse([](int& val) { val *= 2; });
  l.status();

  l.clear();
  l.status();
  return;
}

void test_linklist() {
  ns_ds::LinkList<int> l;
  l.status();

  l.insertElem(0, 12);
  l.insertElem(0, 13);
  l.status();

  l.insertElem(0, 16);
  l.status();

  l.deleteElem(0);
  l.status();

  int elem;
  if (l.getElem(0, elem)) {
    std::cout << "the elem I get at '0' is: " << elem << std::endl;
  }

  int pos;
  if (l.locateElem(12, pos)) {
    std::cout << "the elem '12' is at: " << pos << std::endl;
  }

  if (l.priorElem(12, elem)) {
    std::cout << "12 has prior elem equals to " << elem << std::endl;
  } else {
    std::cout << "12 hasn't prior elem" << std::endl;
  }

  if (l.nextElem(12, elem)) {
    std::cout << "12 has next elem equals to " << elem << std::endl;
  } else {
    std::cout << "12 hasn't next elem" << std::endl;
  }

  l.traverse([](int& val) { val *= 2; });
  l.status();

  l.clear();
  l.status();
}

void test_stack(){
  ns_ds::Stack<int> s;
  s.push(12);
  s.push(2);
  s.push(1);
  s.push(9);
  s.status();

  s.pop();
  s.pop();
  s.status();
  int top;
  s.top(top);
  std::cout << "now the top elem is: " << top << '\n';

  s.traverse([](int &val) { val *= 2; });
  s.status();

  s.clear();
  s.status();

  std::cout << "now the stack is empty? (" << std::boolalpha << s.empty() << ")\n";

}

int main(int argc, char const* argv[]) {
  std::cout << "[ test for 'SeqList' ]\n\n";
  test_seqlist();
  std::cout << "\n[ test for 'LinkList' ]\n\n";
  test_linklist();
  std::cout << "\n[ test for 'Stack' ]\n\n";
  test_stack();
  return 0;
}
