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

outptu

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

