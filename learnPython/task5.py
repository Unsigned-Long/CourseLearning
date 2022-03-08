# coding:utf-8

names = ["tom", "jack", "bob", "mike"]

print(names)

for name in names:
    print("Hello, ", name.title(), '!', sep='')

transports = ["bicycle", "car", "bus", "highway"]

for t in transports:
    print("I would like to own a ", t.title(), '!', sep='')
