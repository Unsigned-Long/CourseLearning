# coding:utf-8

users = ["tom", "bob", "jack", "lily", "admin"]

# users.clear()

if users:
    for user in users:
        if user == "admin":
            print("Hello admin, would you like to see a status report?")
        else:
            print("Hello,", user.title(), "thank you for logging in again.")
else:
    print("We need to find some users!")


current_users = users[:-1]+["mary"]
new_users = ["mike", "jimy", "Jack", "csl", "lily"]
for user in new_users:
    if user.lower() in current_users:
        print("the name [", user, "] is already exit.", sep='')
    else:
        current_users.append(user.lower())
        print("successfully!")


nums = list(range(1, 10))

for num in nums:
    if num == 1:
        print("1st")
    elif num == 2:
        print("2nd")
    elif num == 3:
        print("3rd")
    else:
        print(str(num)+"th")
