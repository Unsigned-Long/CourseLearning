# coding:utf-8

class Restaurant():
    """a class for desc a restaurant"""

    def __init__(self, restaurant_name: str, cuisine: str) -> None:
        self.restaurant_name = restaurant_name
        self.cuisine = cuisine
        return None

    def desc_restaurant(self) -> None:
        print("\nthe restaurant calls", self.restaurant_name)
        print("the cuisine of the restaurant is", self.cuisine)
        return None

    def open_restaurant(self) -> None:
        print(self.restaurant_name, "is running!")
        return None


if __name__ == "__main__":

    my_restaurant = Restaurant("CPP_RES", "213")
    my_restaurant.desc_restaurant()
    my_restaurant.open_restaurant()

    her_restaurant = Restaurant("Mike's Lunch", "111")
    her_restaurant.desc_restaurant()

    his_restaurant = Restaurant("turkilies", "1113")
    his_restaurant.desc_restaurant()


class Users():
    """a desc for a user"""

    def __init__(self, first_name: str, last_name: str) -> None:
        self.first_name = first_name
        self.last_name = last_name
        return None

    def desc(self) -> None:
        print("My name is", ' '.join(
            [self.first_name.title(), self.last_name.title()])
        )
        return None

    def greet_user(self) -> None:
        print("Hello,", self.first_name+" "+self.last_name)


if __name__ == "__main__":
    u1 = Users("shoulong", "chen")
    u1.greet_user()
    u1.desc()
