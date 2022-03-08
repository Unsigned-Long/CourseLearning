# coding:utf-8

keywords = {
    "in": "get elem in a list",
    "for": "a loop",
    "if": "judge something",
    "not": "not something",
    "and": "all",
    "or": "either",
}

for key, value in keywords.items():
    print(key+" : "+value)

for key in keywords.keys():
    print(key+" : "+keywords[key])

rivers = {
    "changjiang": "china",
    "huanghe": "china",
    "heilongjiang": "china",
}
for river in rivers:
    print("the ", river.title(), " runs throngh ", rivers[river], '.', sep='')

people = ["mike", "jimy", "Jack", "csl", "lily"]

already = {
    "mike": True,
    "lily": True,
}

for key in people:
    if key in already.keys():
        print(key.title(), ",Thanks for coming!")
    else:
        print(key.title(), ",Please coming!")
