# coding:utf-8

def read(filename: str):
    with open(filename) as file:
        lines = file.readlines()
    v1 = []
    v2 = []
    v3 = []
    for line in lines:
        vec = line.strip().split(',')
        v1.append(float(vec[0]))
        v2.append(float(vec[1]))
        v3.append(float(vec[2]))
    return (v1, v2, v3)
