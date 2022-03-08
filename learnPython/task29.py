# coding:utf-8

class Restaurant():
    """a class for desc a restaurant"""

    def __init__(self, restaurant_name: str, cuisine: str) -> None:
        self.restaurant_name = restaurant_name
        self.cuisine = cuisine
        self.number_served = 0
        return None

    def desc_restaurant(self) -> None:
        print("\nthe restaurant calls", self.restaurant_name)
        print("the cuisine of the restaurant is", self.cuisine)
        print("the number of serverd is", self.number_served)
        return None

    def open_restaurant(self) -> None:
        print(self.restaurant_name, "is running!")
        return None

    def set_number_served(self, num: int) -> bool:
        if num > self.number_served:
            self.number_served = num
            return True
        else:
            return False

    def increment_number_served(self) -> None:
        self.number_served += 1
        return None

    def reset_number_served(self) -> None:
        self.number_served = 0
        return None


if __name__ == "__main__":

    my_rest = Restaurant("Chen's Dinner", "dda")

    my_rest.desc_restaurant()
    my_rest.set_number_served(-2)
    my_rest.desc_restaurant()
    my_rest.set_number_served(2)
    my_rest.desc_restaurant()

    my_rest.increment_number_served()
    my_rest.increment_number_served()
    my_rest.desc_restaurant()
    my_rest.reset_number_served()
    my_rest.desc_restaurant()
