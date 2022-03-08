# coding:utf-8

import unittest


def get_city_country(city: str, country: str, population: int = -1) -> str:
    if population == -1:
        return city.title()+", "+country.title()
    else:
        return city.title()+", "+country.title()+" - population "+str(population)


class TestCityCountry(unittest.TestCase):

    def test_get_city_country(self):
        result = get_city_country("beijing", "china")
        print(result)
        self.assertEqual("Beijing, China", result)

# City, Country – population xxx eg. Santiago, Chile – population 5000000
    def test_get_city_country_population(self):
        result = get_city_country("beijing", "china", population=14)
        print(result)
        self.assertEqual("Beijing, China - population 14", result)



if __name__ == "__main__":
    unittest.main()
