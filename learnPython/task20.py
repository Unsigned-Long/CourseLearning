# coding:utf-8

import random
names = []

while True:
    name = input("please add a name.(type 'q' if you want to quit it.)")
    if name == 'q':
        break
    else:
        names.append(name)
        print("We have append the name.")

print(names)


while True:
    age = int(input("please input your age : (type -1 if you want to quit it.) "))
    if age == -1:
        break
    elif age < 3:
        print("free")
    elif age <= 12:
        print("10 dolars")
    else:
        print("15 dolars")

while True:
    print(random.random()*100)
