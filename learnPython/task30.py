# coding:utf-8
from task29 import Restaurant
from task28 import Users


class IceCreamStand(Restaurant):

    def __init__(self, restaurant_name: str, cuisine: str) -> None:
        super().__init__(restaurant_name, cuisine)
        self.flavors = ["red", "blue", "green"]
        return None

    def display_flavors(self) -> None:
        print("Here are the flavors :")
        for flavor in self.flavors:
            print(flavor, end=' ')
        print()
        return None


class Privileges():
    def __init__(self, type: int) -> None:
        if type == 0:
            self.priv = "can add post"
        elif type == 1:
            self.priv = "can delete post"
        else:
            self.priv = "can ban user"
        return None

    def show_privileges(self) -> None:
        print(self.priv)


class Admin(Users):

    def __init__(self, first_name: str, last_name: str) -> None:
        super().__init__(first_name, last_name)
        self.privileges = Privileges(2)
        return None

    def show_privileges(self) -> None:
        print("the privileges of", self.first_name +
              " "+self.last_name, "is:")
        self.privileges.show_privileges()

        return None


if __name__ == "__main__":
    ice = IceCreamStand("Mary's Horse", "909")
    ice.display_flavors()
    a1 = Admin("shoulong", "chen")
    a1.show_privileges()
