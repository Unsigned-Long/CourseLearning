# coding:utf-8

def show_users(users: list) -> None:
    for user in users:
        print("Hello,", user.title())


def make_great(users: list) -> None:
    for i in range(0, len(users)):
        users[i] = "The Great "+users[i]


users = ["tom", "bob", "jack", "lily", "admin"]

show_users(users)

make_great(users)

show_users(users)
