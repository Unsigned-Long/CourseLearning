# coding:utf-8

import json

filename = "./fav_num.txt"


def write_fav_num() -> None:
    try:
        num = float(input("Please input your favorite number: "))
    except ValueError:
        print("Please right format number!")
    else:
        with open(filename, 'w') as file:
            json.dump(num, file)
        print("We has write it to a file with 'Json' format.")
    return None


if __name__ == "__main__":
    write_fav_num()
    with open(filename, 'r') as file:
        num = json.load(file)
        msg = input(str(num) + " is your favorite num?(y|n)")
        if msg.lower() == 'n':
            write_fav_num()
        else:
            print("I know your favorite number! It’s", num, end = ".\n")
