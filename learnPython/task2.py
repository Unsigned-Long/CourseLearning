# coding:utf-8

# get the first and last name from the input
first_name = input("Please input your first name :\n")
last_name = input("Please input your last name :\n")

# combine the names and output
full_name = first_name.strip().title()+' '+last_name.strip().title()
message = " Would you like to learn some Python today?"
print("Hello, ", full_name, '!', message, sep='')


