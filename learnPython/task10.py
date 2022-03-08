# coding:utf-8

from typing import List


for val in range(1, 21):
    print(val)

ls = list(range(1, 1000001, 1))

# for val in ls:
#     print(val)

print("min :", min(ls))
print("max :", max(ls))

print("sum :", sum(ls))

odds = list(range(1, 20, 2))

for val in odds:
    print(val)

ls2 = [val for val in range(3, 31, 3)]

for val in ls2:
    print(val)

ls3 = [val**3 for val in range(1, 11, 1)]
for val in ls3:
    print(val)
