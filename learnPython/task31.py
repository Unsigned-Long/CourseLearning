# coding:utf-8

import random


class Die():

    def __init__(self, sider: int = 6) -> None:
        self.sider = sider
        return None

    def roll_die(self) -> None:
        print("The result is", random.randint(1, self.sider))
        return None


if __name__ == "__main__":
    d_6 = Die()
    for i in range(0, 10):
        d_6.roll_die()
    d_10 = Die(10)
    d_20 = Die(20)

    d_10.roll_die()
    d_20.roll_die()
