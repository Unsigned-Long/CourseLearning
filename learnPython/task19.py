# coding:utf-8

car_name = input("Please input the car's name that you want : ")
print("Let me see if I can find you a", car_name)

num = int(input("The number of the people is "))
if num > 8:
    print("Sorry, we don't have more tables!")

val = int(input("Please input a number : "))
if val % 10 == 0:
    print(val, "is right.")
else:
    print(val, "is not right.")
