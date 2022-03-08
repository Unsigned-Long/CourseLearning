# coding:utf-8

friends = ["mike", "jack", "bob", "jimi"]

for friend in friends:
    print(friend.title(), "is my friend!")

friend_not = friends.pop(1)
print(friend_not, "isn't my friend!")
print(friends)

friends.append("lily")
print(friends)

friends.insert(0, "mono")
print(friends)

while len(friends) > 2:
    friends.pop()
print(friends)

del friends[0]
del friends[0]
print(friends)
