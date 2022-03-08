# coding:utf-8

from typing import List


def city_country(city_name: str, country_name: str) -> str:
    full_name = city_name.title()+", "+country_name.title()
    return full_name


msg = city_country("dali", "china")
print(msg)
msg = city_country("shanghai", "china")
print(msg)
msg = city_country("kunming", "china")
print(msg)


def make_album(singer_name: str, work_name: str, songs_num: int = 0) -> dict:
    temp = {
        "singer_name": singer_name,
        "work_name": work_name,
    }
    if songs_num != 0:
        temp["song_num"] = songs_num
    return temp


info = make_album("csl", "Hello, world!")
info2 = make_album("csl", "Hello, world!", 100)

print(info)
print(info2)


while True:
    pass
