# coding:utf-8

first_name = input("Please input your first name:\n")
last_name = input("Please input your last name:\n")

full_name = first_name+' '+last_name
with open("./guest.txt", 'w') as file:
    file.write(full_name)

print("Your info has been recorded. Just check the file 'guest.txt'.")

while True:
    name = input("Please input your name('q' to quit):\n")
    if name == 'q':
        break
    print("hello,", name)
    with open("./guest_book.txt", 'a') as file:
        file.write(name)
        file.write('\n')
