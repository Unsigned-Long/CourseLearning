# coding:utf-8

def make_shirt(text, size="L"):
    print("The size of this shirt is", size, ",and the text is '"+text+"'.")


make_shirt("Hello, Python!", "L")

make_shirt(size="M", text="Hello, CPP!")

make_shirt("Hello, Java!")


def describe_city(city_name, country="China"):
    print(city_name, "is in", country)

describe_city("Shanghai")

describe_city("York","America")

describe_city("Dali")
