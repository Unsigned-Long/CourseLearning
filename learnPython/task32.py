# coding:utf-8

filename = "Zen_of_Python.txt"

with open(filename) as file:
    text = file.read()
    print(text)
print()

with open(filename) as file:
    for line in file:
        print(line.strip())
print()

with open(filename) as file:
    lines = file.readlines()
for line in lines:
    line = line.replace("Python", "CPP")
    print(line.strip())
