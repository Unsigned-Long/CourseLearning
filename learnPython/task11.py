# coding:utf-8

foods = ['pizza', 'falafel', 'carrot cake', 'cannoli', 'ice cream']

print("The first three items in the list are:")
print(foods[:3])

print("Three items from the middle of the list are:")
print(foods[1:4])

print("The last three items in the list are:")
print(foods[-3:])

foods2 = foods[:]
foods.append("drinks")

foods2.append("rice")

print("My favorite foods are:")
for food in foods:
    print(food)

print("My friend's favorite foods are:")
for food in foods2:
    print(food)
