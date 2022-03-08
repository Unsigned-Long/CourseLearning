# coding:utf-8

sandwich_orders = ["1", "2", "3", "4", "1", "5", "10", "1", "3", "1"]
finished_sandwiches = []
while sandwich_orders:
    index = sandwich_orders.pop()
    print("sandwich index :", index)
    finished_sandwiches.append(index)

print("finihed sandwiches :", finished_sandwiches)

while "1" in finished_sandwiches:
    finished_sandwiches.remove("1")

print(finished_sandwiches)

while True:
    msg = input("please input the index of the sandwich : ")
    if msg in finished_sandwiches:
        print("fine, yours is finished.")
    else:
        print("sorry, we can't find your order, maybe it's cooking or you didn't submit your order!")
