# coding:utf-8

# Alien Game

alien_color = "red"

if alien_color == "green":
    print("You get five points!")

if alien_color == "green":
    print("You get five points!")
else:
    print("You get ten points!")

if alien_color == "green":
    print("You get five points!")
elif alien_color == "yellow":
    print("You get ten points!")
else:
    print("You get fifteen points!")

people = ("baby", "kid", "chrid", "teenage", "adult", "old")
age = 21
if age < 2:
    type = people[0]
elif age < 4:
    type = people[1]
elif age < 13:
    type = people[2]
elif age < 20:
    type = people[3]
elif age < 65:
    type = people[4]
else:
    type = people[-1]
print("You are a(an) ", type, "!", sep='')

favorite_fruits = ["apple", "banan", "petch"]

if "apple" in favorite_fruits:
    print("apple is in your list!".title())
if "banan" in favorite_fruits:
    print("banan is in your list!".title())
if "petch" in favorite_fruits:
    print("petch is in your list!".title())
