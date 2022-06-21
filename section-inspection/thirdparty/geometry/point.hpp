#ifndef POINT_HPP
#define POINT_HPP

/**
 * @file point.hpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2021-12-06
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "utility.hpp"
#include <functional>

namespace ns_geo {
#pragma region geometry
  /**
   * @brief the types of the geometry
   */
  enum class GeoType {
    // for geometry without reference
    POINT2,
    POINT3,
    LINE2,
    LINE3,
    LINESTRING2,
    LINESTRING3,
    POLYGON,
    RECTANGLE,
    TRIANGLE2,
    TRIANGLE3,
    CIRCLE,
    // for geometry with reference
    REF_POINT2,
    REF_POINT3,
    REF_LINE2,
    REF_LINE3,
    REF_LINESTRING2,
    REF_LINESTRING3,
    REF_POLYGON,
    REF_RECTANGLE,
    REF_TRIANGLE2,
    REF_TRIANGLE3
  };

  /**
   * @brief overload the operator '<<' for 'GeoType'
   *
   * @param os the ostream
   * @param geoType the GeoType object
   * @return std::ostream&
   */
  static std::ostream &operator<<(std::ostream &os, GeoType geoType) {
    switch (geoType) {
    case GeoType::POINT2:
      os << "POINT2";
      break;
    case GeoType::POINT3:
      os << "POINT3";
      break;
    case GeoType::LINE2:
      os << "LINE2";
      break;
    case GeoType::LINE3:
      os << "LINE3";
      break;
    case GeoType::LINESTRING2:
      os << "LINESTRING2";
      break;
    case GeoType::LINESTRING3:
      os << "LINESTRING3";
      break;
    case GeoType::POLYGON:
      os << "POLYGON";
      break;
    case GeoType::RECTANGLE:
      os << "RECTANGLE";
      break;
    case GeoType::TRIANGLE2:
      os << "TRIANGLE2";
      break;
    case GeoType::TRIANGLE3:
      os << "TRIANGLE3";
      break;
    case GeoType::CIRCLE:
      os << "CIRCLE";
      break;
    case GeoType::REF_POINT2:
      os << "REF-POINT2";
      break;
    case GeoType::REF_POINT3:
      os << "REF-POINT3";
      break;
    case GeoType::REF_LINE2:
      os << "REF-LINE2";
      break;
    case GeoType::REF_LINE3:
      os << "REF-LINE3";
      break;
    case GeoType::REF_LINESTRING2:
      os << "REF-LINESTRING2";
      break;
    case GeoType::REF_LINESTRING3:
      os << "REF-LINESTRING3";
      break;
    case GeoType::REF_POLYGON:
      os << "REF-POLYGON";
      break;
    case GeoType::REF_RECTANGLE:
      os << "REF-RECTANGLE";
      break;
    case GeoType::REF_TRIANGLE2:
      os << "REF-TRIANGLE2";
      break;
    case GeoType::REF_TRIANGLE3:
      os << "REF-TRIANGLE3";
      break;
    default:
      break;
    }
    return os;
  }

#pragma endregion

  /**
   * @brief the base geometry class
   */
  class Geometry {
  public:
    virtual ~Geometry() = default;

    /**
     * @brief the type of the geometry
     *
     * @return GeoType
     */
    [[nodiscard]] virtual GeoType type() const = 0;
  };

#pragma region Point2

  /**
   * @brief a sample template class to describe the 2-dime points
   */
  template <typename Ty = float>
  class Point2 : protected Geometry {
  public:
    using value_type = Ty;
    using ary_type = std::array<Ty, 2>;
    using self_type = Point2<value_type>;

  public:
    value_type x;
    value_type y;

  public:
    /**
     * @brief constructors
     */
    Point2() = default;
    Point2(value_type x, value_type y) : x(x), y(y) {}
    explicit Point2(const ary_type &p) : x(p[0]), y(p[1]) {}
    explicit Point2(const value_type p[2]) : x(p[0]), y(p[1]) {}

    explicit operator ary_type() const { return ary_type{this->x, this->y}; }

    /**
     * @brief Exchange the X and Y coordinates of the point based on current point
     *
     * @return self_type
     */
    inline self_type transposed() const { return self_type(this->y, this->x); }

    /**
     * @brief Exchange the X and Y coordinates of the point
     *
     * @return self_type&
     */
    inline self_type &transpose() {
      auto temp = this->x;
      this->x = this->y;
      this->y = temp;
      return *this;
    }

    [[nodiscard]] inline ns_geo::GeoType type() const override {
      return GeoType::POINT2;
    }
  };
  /**
   * @brief overload operator "<<" for Point2<_Ty>
   */
  template <typename Ty>
  std::ostream &operator<<(std::ostream &os, const Point2<Ty> &p) {
    os << '[' << p.x << ", " << p.y << ']';
    return os;
  }

#pragma endregion

#pragma region Point3

  /**
   * @brief a sample template class to describe the 3-dime points
   */
  template <typename Ty = float>
  class Point3 : protected Geometry {
  public:
    using value_type = Ty;
    using ary_type = std::array<Ty, 3>;
    using self_type = Point3<value_type>;

  public:
    value_type x;
    value_type y;
    value_type z;

  public:
    /**
     * @brief constructors
     */
    Point3() = default;
    Point3(value_type x, value_type y, value_type z) : x(x), y(y), z(z) {}
    explicit Point3(const ary_type &p) : x(p[0]), y(p[1]), z(p[2]) {}
    explicit Point3(const value_type p[3]) : x(p[0]), y(p[1]), z(p[2]) {}

    explicit operator ary_type() const { return ary_type{this->x, this->y, this->z}; }

    [[nodiscard]] inline ns_geo::GeoType type() const override {
      return GeoType::POINT3;
    }
  };
  /**
   * @brief overload operator "<<" for Point3
   */
  template <typename Ty>
  std::ostream &operator<<(std::ostream &os, const Point3<Ty> &p) {
    os << '[' << p.x << ", " << p.y << ", " << p.z << ']';
    return os;
  }
#pragma endregion

#pragma region PointSet2

  /**
   * @brief Container for storage point2<_Ty>
   *
   */
  template <typename Ty>
  class PointSet2 : public std::vector<Point2<Ty>> {
  public:
    using value_type = Ty;
    using point_type = Point2<value_type>;
    using container_type = std::vector<point_type>;
    /**
     * @brief using container_type's constructors
     */
    using container_type::container_type;
    using self_type = PointSet2<value_type>;

    using selector = std::function<bool(const point_type &)>;

  public:
    /**
     * @brief write points to the file
     *
     * @param filePath the path of the file
     * @param mode the ios mode
     */
    void write(const std::string &filePath, std::ios_base::openmode mode = std::ios::out | std::ios::binary) const {
      std::ofstream file(filePath, mode);
      if (!file.is_open())
        throw std::ios_base::failure("File Open Failed");
      if (std::ios::binary == (mode & std::ios::binary)) {
        for (const auto &p : *this)
          file.write((const char *)(&p), sizeof(point_type));
      } else
        for (const auto &point : *this)
          file << point.x << ',' << point.y << '\n';
    }
    /**
     * @brief read points from the file
     *
     * @param filePath the path of the file
     * @param mode the ios mode
     */
    void read(const std::string &filePath,
              std::ios_base::openmode mode = std::ios::in | std::ios::binary) {
      std::ifstream file(filePath, mode);
      if (!file.is_open())
        throw std::ios_base::failure("File Open Failed");
      if (std::ios::binary == (mode & std::ios::binary)) {
        point_type p;
        file.seekg(0, std::ios::end);
        auto size = file.tellg() / sizeof(point_type);
        file.seekg(0, std::ios::beg);
        if (!this->empty()) {
          this->clear();
        }
        this->resize(size);
        int count = 0;
        while (!file.eof() && count < size) {
          file.read((char *)(&p), sizeof(point_type));
          this->at(count) = p;
          ++count;
        }
      } else {
        Point2<value_type> point;
        std::string str;
        while (!file.eof()) {
          std::getline(file, str);
          if (str.empty())
            continue;
          auto iter = std::find(str.cbegin(), str.cend(), ',');
          point.x = static_cast<value_type>(std::stod(std::string(str.cbegin(), iter)));
          point.y = static_cast<value_type>(std::stod(std::string(++iter, str.cend())));
          this->push_back(point);
        }
      }
    }

    /**
     * @brief generate eligible point randomly
     *
     * @param num the number
     * @param x_min the min x value
     * @param x_max the max x value
     * @param y_min the min y value
     * @param y_max the max y value
     * @param select the selector
     * @return self_type
     */
    static self_type randomGenerator(std::size_t num,
                                     Ty x_min, Ty x_max,
                                     Ty y_min, Ty y_max,
                                     const selector &select = nullptr) {
      std::uniform_real_distribution<> u_x(static_cast<float>(x_min), static_cast<float>(x_max));
      std::uniform_real_distribution<> u_y(static_cast<float>(y_min), static_cast<float>(y_max));
      self_type ps(num);
      int count = 0;
      while (count != num) {
        point_type p(static_cast<Ty>(u_x(engine)), static_cast<Ty>(u_y(engine)));
        if (select == nullptr || select(p))
          ps.at(count++) = p;
      }
      return ps;
    }
  };

#pragma endregion

#pragma region PointSet3
  /**
   * @brief Container for storage point3<_Ty>
   *
   */
  template <typename Ty>
  class PointSet3 : public std::vector<Point3<Ty>> {
  public:
    using value_type = Ty;
    using point_type = Point3<value_type>;
    using container_type = std::vector<point_type>;
    /**
     * @brief using container_type's constructors
     */
    using container_type::container_type;
    using self_type = PointSet3<value_type>;

    using selector = std::function<bool(const point_type &)>;

  public:
    /**
     * @brief write points to the file
     *
     * @param filePath the path of the file
     * @param mode the ios mode
     */
    void write(const std::string &filePath,
               std::ios_base::openmode mode = std::ios::out | std::ios::binary) const {
      std::ofstream file(filePath, mode);
      if (!file.is_open())
        throw std::ios_base::failure("File Open Failed");
      if (std::ios::binary == (mode & std::ios::binary)) {
        for (const auto &p : *this)
          file.write((const char *)(&p), sizeof(point_type));
      } else
        for (const auto &point : *this)
          file << point.x << ',' << point.y << ',' << point.z << '\n';
    }

    /**
     * @brief read points from the file
     *
     * @param filePath the path of the file
     * @param mode the ios mode
     */
    void read(const std::string &filePath,
              std::ios_base::openmode mode = std::ios::in | std::ios::binary) {
      std::ifstream file(filePath, mode);
      if (!file.is_open())
        throw std::ios_base::failure("File Open Failed");
      if (std::ios::binary == (mode & std::ios::binary)) {
        point_type p;
        file.seekg(0, std::ios::end);
        auto size = file.tellg() / sizeof(point_type);
        file.seekg(0, std::ios::beg);
        int count = 0;
        if (!this->empty()) {
          this->clear();
        }
        this->resize(size);
        while (!file.eof() && count < size) {
          file.read((char *)(&p), sizeof(point_type));
          this->at(count) = p;
          ++count;
        }
      } else {
        Point3<value_type> point;
        std::string str;
        while (!file.eof()) {
          std::getline(file, str);
          if (str.empty())
            continue;
          auto iter = std::find(str.cbegin(), str.cend(), ',');
          point.x = static_cast<value_type>(std::stod(std::string(str.cbegin(), iter)));
          auto iter2 = std::find(++iter, str.cend(), ',');
          point.y = static_cast<value_type>(std::stod(std::string(iter, iter2)));
          point.z = static_cast<value_type>(std::stod(std::string(++iter2, str.cend())));
          this->push_back(point);
        }
      }
    }

    /**
     * @brief generate eligible point randomly
     *
     * @param num the number
     * @param x_min the min x value
     * @param x_max the max x value
     * @param y_min the min y value
     * @param y_max the max y value
     * @param z_min the min z value
     * @param z_max the max z value
     * @param select the selector
     * @return self_type
     */
    static self_type randomGenerator(std::size_t num,
                                     Ty x_min, Ty x_max,
                                     Ty y_min, Ty y_max,
                                     Ty z_min, Ty z_max,
                                     const selector &select = nullptr) {
      std::uniform_real_distribution<> u_x(static_cast<float>(x_min),
                                           static_cast<float>(x_max));
      std::uniform_real_distribution<> u_y(static_cast<float>(y_min),
                                           static_cast<float>(y_max));
      std::uniform_real_distribution<> u_z(static_cast<float>(z_min),
                                           static_cast<float>(z_max));
      self_type ps(num);
      int count = 0;
      while (count != num) {
        point_type p(static_cast<Ty>(u_x(engine)),
                     static_cast<Ty>(u_y(engine)),
                     static_cast<Ty>(u_z(engine)));
        if (select == nullptr || select(p))
          ps.at(count++) = p;
      }
      return ps;
    }
  };
#pragma endregion

#pragma region RefPoint2
  template <typename Ty = float>
  class RefPoint2 : public Point2<Ty> {
  public:
    using id_type = uint;
    using value_type = Ty;
    using ary_type = std::array<Ty, 2>;
    using self_type = RefPoint2<value_type>;

  public:
    id_type id{};

  public:
    /**
     * @brief constructors
     */
    RefPoint2() = default;
    RefPoint2(id_type id, value_type x, value_type y)
        : id(id), Point2<Ty>(x, y) {}
    RefPoint2(id_type id, const ary_type &p) : id(id), Point2<Ty>(p[0], p[1]) {}
    RefPoint2(id_type id, const value_type p[2])
        : id(id), Point2<Ty>(p[0], p[1]) {}

    [[nodiscard]] inline ns_geo::GeoType type() const override {
      return GeoType::REF_POINT2;
    }
  };
  /**
   * @brief overload operator "<<" for RefPoint2<_Ty>
   */
  template <typename Ty>
  std::ostream &operator<<(std::ostream &os, const RefPoint2<Ty> &p) {
    os << '{' << p.id << ": " << '[' << p.x << ", " << p.y << ']' << '}';
    return os;
  }
#pragma endregion

#pragma region RefPoint3
  template <typename Ty = float>
  class RefPoint3 : public Point3<Ty> {
  public:
    using id_type = uint;
    using value_type = Ty;
    using ary_type = std::array<Ty, 3>;
    using self_type = RefPoint3<value_type>;

  public:
    id_type id{};

  public:
    /**
     * @brief constructors
     */
    RefPoint3() = default;
    RefPoint3(id_type id, value_type x, value_type y, value_type z)
        : id(id), Point3<Ty>(x, y, z) {}
    RefPoint3(id_type id, const ary_type &p)
        : id(id), Point3<Ty>(p[0], p[1], p[2]) {}
    RefPoint3(id_type id, const value_type p[3])
        : id(id), Point3<Ty>(p[0], p[1], p[2]) {}

    [[nodiscard]] inline ns_geo::GeoType type() const override {
      return GeoType::REF_POINT3;
    }
  };
  /**
   * @brief overload operator "<<" for RefPoint3<_Ty>
   */
  template <typename Ty>
  std::ostream &operator<<(std::ostream &os, const RefPoint3<Ty> &p) {
    os << '{' << p.id << ": " << '[' << p.x << ", " << p.y << ", " << p.z
       << ']' << '}';
    return os;
  }

#pragma endregion

#pragma region RefPointSet2

  template <typename Ty, typename Hash = std::hash<uint>,
            typename Pred = std::equal_to<uint>>
  class RefPointSet2 : public std::unordered_map<uint, RefPoint2<Ty>, Hash, Pred> {
  public:
    using value_type = Ty;
    using id_type = uint;
    using refpoint_type = RefPoint2<value_type>;
    using container_type =
        std::unordered_map<id_type, refpoint_type, Hash, Pred>;
    /**
     * @brief using container_type's constructors
     */
    using container_type::container_type;
    using selector = std::function<bool(const refpoint_type &)>;

    using self_type = RefPointSet2<value_type>;

  public:
    /**
     * @brief insert a reference point to the refpointset
     *
     * @param p the reference point
     * @return auto
     */
    auto insert(const refpoint_type &p) {
      return container_type::insert(std::make_pair(p.id, p));
    }
    /**
     * @brief write points to the file
     *
     * @param filePath the path of the file
     * @param mode the ios mode
     */
    void write(const std::string &filePath, std::ios_base::openmode mode = std::ios::out | std::ios::binary) const {
      std::ofstream file(filePath, mode);
      if (!file.is_open())
        throw std::ios_base::failure("File Open Failed");
      if (std::ios::binary == (mode & std::ios::binary))
        for (const auto &[id, refPt] : *this)
          file.write((const char *)(&refPt), sizeof(refpoint_type));
      else
        for (const auto &[id, refPt] : *this)
          file << id << ',' << refPt.x << ',' << refPt.y << '\n';
    }
    /**
     * @brief read points from the file
     *
     * @param filePath the path of the file
     * @param mode the ios mode
     */
    void read(const std::string &filePath,
              std::ios_base::openmode mode = std::ios::in | std::ios::binary) {
      std::ifstream file(filePath, mode);
      if (!file.is_open())
        throw std::ios_base::failure("File Open Failed");
      if (std::ios::binary == (mode & std::ios::binary)) {
        refpoint_type refPt;
        file.seekg(0, std::ios::end);
        auto size = file.tellg() / sizeof(refpoint_type);
        file.seekg(0, std::ios::beg);
        if (!this->empty()) {
          this->clear();
        }
        int count = 0;
        while (!file.eof() && count < size) {
          file.read((char *)(&refPt), sizeof(refpoint_type));
          this->insert(refPt);
          ++count;
        }
      } else {
        refpoint_type refPt;
        std::string str;
        while (!file.eof()) {
          std::getline(file, str);
          if (str.empty())
            continue;
          auto iter = std::find(str.cbegin(), str.cend(), ',');
          const_cast<id_type &>(refPt.id) = static_cast<id_type>(std::stoi(std::string(str.cbegin(), iter)));
          auto iter2 = std::find(++iter, str.cend(), ',');
          refPt.x = static_cast<value_type>(std::stod(std::string(iter, iter2)));
          refPt.y = static_cast<value_type>(
              std::stod(std::string(++iter2, str.cend())));
          this->insert(refPt);
        }
      }
    }

  public:
    /**
     * @brief Create a RefLine2<_Ty> object
     *
     * @param pid1 the id of the 1st point
     * @param pid2 the id of the 2nd point
     * @return RefLine2<value_type>
     */
    RefLine2<value_type> createRefLine2(id_type pid1, id_type pid2) const {
      return RefLine2<value_type>(pid1, pid2, this);
    }

    /**
     * @brief Create a RefRectangle<_Ty> object
     *
     * @param topLeftID the id of the top-left point
     * @param bottomRightID the id of the bottom-right point
     * @return RefRectangle<value_type>
     */
    RefRectangle<value_type> createRefRectangle(id_type topLeftID, id_type bottomRightID) const {
      return RefRectangle<value_type>(topLeftID, bottomRightID, this);
    }

    /**
     * @brief Create a RefTriangle2<_Ty> object
     *
     * @param pid1 the id of the 1st point
     * @param pid2 the id of the 2nd point
     * @param pid3 the id of the 3rd point
     * @return RefTriangle2<value_type>
     */
    RefTriangle2<value_type> createRefTriangle2(id_type pid1, id_type pid2, id_type pid3) const {
      return RefTriangle2<value_type>(pid1, pid2, pid3, this);
    }

    /**
     * @brief Create a RefPolygon<_Ty> object
     *
     * @param pidList the id list for points
     * @return RefPolygon<value_type>
     */
    RefPolygon<value_type> createRefPolygon(const std::initializer_list<id_type> &pidList) const {
      return RefPolygon<value_type>(pidList, this);
    }

    /**
     * @brief Create a RefLineString2<_Ty> object
     *
     * @param pidList the id list for points
     * @return RefLineString2<value_type>
     */
    RefLineString2<value_type> createRefLineString2(const std::initializer_list<id_type> &pidList) const {
      return RefLineString2<value_type>(pidList, this);
    }

    /**
     * @brief generate eligible point randomly
     *
     * @param num the number
     * @param x_min the min x value
     * @param x_max the max x value
     * @param y_min the min y value
     * @param y_max the max y value
     * @param select the selector
     * @return self_type
     */
    static self_type randomGenerator(std::size_t num,
                                     Ty x_min, Ty x_max,
                                     Ty y_min, Ty y_max,
                                     const selector &select = nullptr) {
      std::uniform_real_distribution<> u_x(static_cast<float>(x_min),
                                           static_cast<float>(x_max));
      std::uniform_real_distribution<> u_y(static_cast<float>(y_min),
                                           static_cast<float>(y_max));
      self_type ps;
      int count = 0;
      while (count != num) {
        refpoint_type p(count, static_cast<Ty>(u_x(engine)), static_cast<Ty>(u_y(engine)));
        if (select == nullptr || select(p))
          ps.insert(p), ++count;
      }
      return ps;
    }

  public:
    /**
     * @brief dangerous function has been deleted
     */
    refpoint_type &operator[](const id_type &id) = delete;
  };

#pragma endregion

#pragma region RefPointSet3
  template <typename Ty, typename Hash = std::hash<uint>,
            typename Pred = std::equal_to<uint>>
  class RefPointSet3
      : public std::unordered_map<uint, RefPoint3<Ty>, Hash, Pred> {
  public:
    using value_type = Ty;
    using id_type = uint;
    using refpoint_type = RefPoint3<value_type>;
    using container_type =
        std::unordered_map<id_type, refpoint_type, Hash, Pred>;
    /**
     * @brief using container_type's constructors
     */
    using container_type::container_type;

    using selector = std::function<bool(const refpoint_type &)>;

    using self_type = RefPointSet3<value_type>;

  public:
    /**
     * @brief insert a reference point to the refpointset
     *
     * @param p the reference point
     * @return auto
     */
    auto insert(const refpoint_type &p) {
      return container_type::insert(std::make_pair(p.id, p));
    }
    /**
     * @brief write points to the file
     *
     * @param filePath the path of the file
     * @param mode the ios mode
     */
    void write(const std::string &filePath,
               std::ios_base::openmode mode = std::ios::out | std::ios::binary) const {
      std::ofstream file(filePath, mode);
      if (!file.is_open())
        throw std::ios_base::failure("File Open Failed");
      if (std::ios::binary == (mode & std::ios::binary)) {
        for (const auto &[id, refPt] : *this)
          file.write((const char *)(&refPt), sizeof(refpoint_type));
      } else
        for (const auto &[id, refPt] : *this)
          file << refPt.id << ',' << refPt.x << ','
               << refPt.y << ',' << refPt.z << '\n';
    }
    /**
     * @brief read points from the file
     *
     * @param filePath the path of the file
     * @param mode the ios mode
     */
    void read(const std::string &filePath,
              std::ios_base::openmode mode = std::ios::in | std::ios::binary) {
      std::ifstream file(filePath, mode);
      if (!file.is_open())
        throw std::ios_base::failure("File Open Failed");
      if (std::ios::binary == (mode & std::ios::binary)) {
        refpoint_type refPt;
        file.seekg(0, std::ios::end);
        auto size = file.tellg() / sizeof(refpoint_type);
        file.seekg(0, std::ios::beg);
        if (!this->empty()) {
          this->clear();
        }
        int count = 0;
        while (!file.eof() && count < size) {
          file.read((char *)(&refPt), sizeof(refpoint_type));
          this->insert(refPt);
          ++count;
        }
      } else {
        refpoint_type refPt;
        std::string str;
        while (!file.eof()) {
          std::getline(file, str);
          if (str.empty())
            continue;
          auto iter = std::find(str.cbegin(), str.cend(), ',');
          const_cast<uint &>(refPt.id) = static_cast<uint>(std::stoi(std::string(str.cbegin(), iter)));
          auto iter2 = std::find(++iter, str.cend(), ',');
          refPt.x = static_cast<value_type>(std::stod(std::string(iter, iter2)));
          auto iter3 = std::find(++iter2, str.cend(), ',');
          refPt.y = static_cast<value_type>(std::stod(std::string(iter2, iter3)));
          refPt.z = static_cast<value_type>(std::stod(std::string(++iter3, str.cend())));
          this->insert(refPt);
        }
      }
    }

  public:
    /**
     * @brief Create a RefLine3<_Ty> object
     *
     * @param pid1 the id of the 1st point
     * @param pid2 the id of the 2nd point
     * @return RefLine3<value_type>
     */
    RefLine3<value_type> createRefLine3(id_type pid1, id_type pid2) const {
      return RefLine3<value_type>(pid1, pid2, this);
    }
    /**
     * @brief Create a RefTriangle3<_Ty> object
     *
     * @param pid1 the id of the 1st point
     * @param pid2 the id of the 2nd point
     * @param pid3 the id of the 3rd point
     * @return RefTriangle3<value_type>
     */
    RefTriangle3<value_type> createRefTriangle3(id_type pid1,
                                                id_type pid2,
                                                id_type pid3) const {
      return RefTriangle3<value_type>(pid1, pid2, pid3, this);
    }

    /**
     * @brief Create a RefLineString3<_Ty> object
     *
     * @param pidList the id list for points
     * @return RefLineString3<value_type>
     */
    RefLineString3<value_type> createRefLineString3(
        const std::initializer_list<id_type> &pidList) const {
      return RefLineString3<value_type>(pidList, this);
    }

    /**
     * @brief generate eligible point randomly
     *
     * @param num the number
     * @param x_min the min x value
     * @param x_max the max x value
     * @param y_min the min y value
     * @param y_max the max y value
     * @param z_min the min z value
     * @param z_max the max z value
     * @param select the selector
     * @return self_type
     */
    static self_type randomGenerator(std::size_t num,
                                     Ty x_min, Ty x_max,
                                     Ty y_min, Ty y_max,
                                     Ty z_min, Ty z_max,
                                     const selector &select = nullptr) {
      std::uniform_real_distribution<> u_x(static_cast<float>(x_min),
                                           static_cast<float>(x_max));
      std::uniform_real_distribution<> u_y(static_cast<float>(y_min),
                                           static_cast<float>(y_max));
      std::uniform_real_distribution<> u_z(static_cast<float>(z_min),
                                           static_cast<float>(z_max));

      self_type ps;
      int count = 0;
      while (count != num) {
        refpoint_type p(count, static_cast<Ty>(u_x(engine)),
                        static_cast<Ty>(u_y(engine)),
                        static_cast<Ty>(u_z(engine)));

        if (select == nullptr || select(p))
          ps.insert(p), ++count;
      }
      return ps;
    }

  public:
    /**
     * @brief dangerous function has been deleted
     */
    refpoint_type &operator[](const id_type &id) = delete;
  };

#pragma endregion
} // namespace ns_geo

#endif