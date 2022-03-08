# coding:utf-8

def add() -> None:
    print("Please input two numbs to add:")
    try:
        num1 = int(input("num 1 : "))
        num2 = int(input("num 2 : "))
    except ValueError:
        print("Wrong num type!")
    else:
        print("result :", str(num1), '+', str(num2), '=', num1+num2)
    return None


def readFile(filename: str) -> None:
    try:
        with open(filename) as file:
            msg = file.read()
    except FileNotFoundError:
        print("can't find file named", filename)
        pass
    else:
        print(msg)
    return None


def countWord(filename: str, word: str) -> None:
    try:
        with open(filename) as file:
            msg = file.read()
    except FileNotFoundError:
        print("can't find file named", filename)
        pass
    else:
        count = msg.lower().count(word)
        print("the world '", word, "' appears ", count, " times", sep='')


if __name__ == "__main__":
    add()
    readFile("./guestq.txt")
    countWord("./the_power_of_music.txt", "the")
