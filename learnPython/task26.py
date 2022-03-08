# coding:utf-8


def show_friends(*friends) -> None:
    for friend in friends:
        print(friend)


show_friends("lily", "tom", "mike")
show_friends("lily", "tom")
show_friends("lily")


def build_profile(first_name: str, last_name: str, **other_info) -> dict:
    info = {}
    info["first_name"] = first_name.title()
    info["last_name"] = last_name.title()
    for key, val in other_info.items():
        info[key] = val
    return info


csl = build_profile("shoulong", "chen", age=21, sex="f", height=1.75)

print(csl)


def make_car(maker: str, type: str, **other_info) -> dict:
    info = {}
    info["maker"] = maker
    info["type"] = type
    for key, val in other_info.items():
        info[key] = val
    return info

car = make_car('subaru', 'outback', color='blue', tow_package=True)

print(car)
