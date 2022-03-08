# coding:utf-8

import random
people = [
    {
        "first_name": "shoulong",
        "last_name": "chen",
        "age": 21,
    },
    {
        "first_name": "shou",
        "last_name": "zhang",
        "age": 22,
    },
    {
        "first_name": "long",
        "last_name": "wang",
        "age": 20,
    },
    {
        "first_name": "xian",
        "last_name": "cao",
        "age": 19,
    },
]

for elem in people:
    print("people : ")
    for key, value in elem.items():
        print(key, ':', value)
    print("------")


info = {}

for i in range(0, 10):
    info[str(random.random()*10)] = random.random()*100

for key, val in info.items():
    print("key :", key, "value :", val)
