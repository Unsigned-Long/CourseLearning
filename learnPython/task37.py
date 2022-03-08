# coding:utf-8

import unittest


class Employee():
    def __init__(self, name: str, age: int, salary: float) -> None:
        self.name = name
        self.age = age
        self.salary = salary
        return None

    def give_raise(self, val: float = -1.0) -> None:
        if val == -1.0:
            self.salary += 5000.0
        else:
            self.salary += val
        return None


class TestEmployee(unittest.TestCase):
    def setUp(self) -> None:
        self.em = Employee("csl", 1, 1.0)
        return None

    def test_give_default_raise(self) -> None:
        self.em.give_raise()
        self.assertEqual(5001.0, self.em.salary)
        return None

    def test_give_custom_raise(self) -> None:
        self.em.give_raise(20.0)
        self.assertEqual(21.0, self.em.salary)
        return None


if __name__ == "__main__":
    unittest.main()