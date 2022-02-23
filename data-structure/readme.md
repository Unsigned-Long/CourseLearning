# Data Structure

___Author : csl___

___E-Mail : 3079625093@qq.com___

[TOC]

## OverView

this project is based on the course 'Data Structure'. it has realized some classic data structures such as '___sequential list___', '___link list___' and so on. 

## Code

the source files are here:

+ [___include___](./include/)
  
  + [seqlist.h](./include/seqlist.h) the sequential list source code
  
  + [linklist.h](./include/linklist.h) the link list source code

___total: 2 files___

## Items

### 1. sequential list

test case

```cpp
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
```

output

```cpp
[ test for 'SeqList' ]

SeqList: {'len': 0, 'max-len': 2, 'elem(s)': []}
SeqList: {'len': 2, 'max-len': 2, 'elem(s)': [13, 12]}
SeqList: {'len': 3, 'max-len': 4, 'elem(s)': [16, 13, 12]}
SeqList: {'len': 2, 'max-len': 4, 'elem(s)': [13, 12]}
the elem I get at '0' is: 13
the elem '12' is at: 1
12 has prior elem equals to 13
12 hasn't next elem
SeqList: {'len': 2, 'max-len': 4, 'elem(s)': [26, 24]}
SeqList: {'len': 0, 'max-len': 4, 'elem(s)': []}
```

### 2. link list

test case

```cpp
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
```

output

```cpp
[ test for 'LinkList' ]

LinkList: {'len': 0, 'elem(s)': []}
LinkList: {'len': 2, 'elem(s)': [13, 12]}
LinkList: {'len': 3, 'elem(s)': [16, 13, 12]}
LinkList: {'len': 2, 'elem(s)': [13, 12]}
the elem I get at '0' is: 13
the elem '12' is at: 1
12 has prior elem equals to 13
12 hasn't next elem
LinkList: {'len': 2, 'elem(s)': [26, 24]}
LinkList: {'len': 0, 'elem(s)': []}
```

### 3. stack

test case

```cpp
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
```

output

```cpp
[ test for 'Stack' ]

Stack: {'len': 4, 'elem(s)': [12, 2, 1, 9]}
Stack: {'len': 2, 'elem(s)': [12, 2]}
now the top elem is: 2
Stack: {'len': 2, 'elem(s)': [24, 4]}
Stack: {'len': 0, 'elem(s)': []}
now the stack is empty? (true)
```

### 4. queue

test case

```cpp
void test_queue() {
  ns_ds::Queue<int> q;
  q.push(12);
  q.push(2);
  q.push(1);
  q.push(9);
  q.status();

  q.pop();
  q.pop();
  q.status();
  int head;
  q.head(head);
  std::cout << "now the head elem is: " << head << '\n';

  q.traverse([](int &val) { val *= 2; });
  q.status();

  q.clear();
  q.status();

  std::cout << "now the queue is empty? (" << std::boolalpha << q.empty() << ")\n";
}
```

outptu

```cpp
[ test for 'Queue' ]

Queue: {'len': 4, 'elem(s)': [12, 2, 1, 9]}
Queue: {'len': 2, 'elem(s)': [1, 9]}
now the head elem is: 1
Queue: {'len': 2, 'elem(s)': [2, 18]}
Queue: {'len': 0, 'elem(s)': []}
now the queue is empty? (true)
```

### 5. BM && KMP

#### a. BM

test case

```cpp
void test_BF() {
  const char *chr1 = "startendengok";
  ns_ds::char_stream str1(chr1, 13);
  str1.status();
  const char *chr2 = "eng";
  ns_ds::char_stream str2(chr2, 3);
  str2.status();
  std::cout << "the pos is at: " << ns_ds::matchBF(str1, str2, 3) << std::endl;
}
```

outptu

```cpp

[ test for 'BF' ]

SeqList: {'len': 13, 'max-len': 13, 'elem(s)': [s, t, a, r, t, e, n, d, e, n, g, o, k]}
SeqList: {'len': 3, 'max-len': 3, 'elem(s)': [e, n, g]}
the pos is at: 8
```

#### b. KMP

test case

```cpp
void test_KMP() {
  const char *chr1 = "startendengok";
  ns_ds::char_stream str1(chr1, 13);
  str1.status();
  const char *chr2 = "eng";
  ns_ds::char_stream str2(chr2, 3);
  str2.status();
  std::cout << "the pos is at: " << ns_ds::matchKMP(str1, str2, 3) << std::endl;
}
```

```cpp
[ test for 'KMP' ]

SeqList: {'len': 13, 'max-len': 13, 'elem(s)': [s, t, a, r, t, e, n, d, e, n, g, o, k]}
SeqList: {'len': 3, 'max-len': 3, 'elem(s)': [e, n, g]}
the pos is at: 8
```



