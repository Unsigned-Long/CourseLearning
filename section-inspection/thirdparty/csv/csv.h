#pragma once

/**
 * @file csv-v2.h
 * @author shlchen (3079625093@qq.com)
 * @brief
 * @version 0.2
 * @date 2022-05-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <algorithm>
#include <array>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace ns_csv {

#define THROW_EXCEPTION(where, msg) \
  throw std::runtime_error(std::string("[ error from 'libcsv'-'") + #where + "' ] " + msg)

  namespace ns_priv {

#pragma region help functions
    /**
     * @brief a function to split a string to some string elements according the splitor
     *
     * @param str the string to be splited
     * @param splitor the splitor char
     * @param ignoreEmpty whether ignoring the empty string element or not
     *
     * @return the splited string vector
     */
    static std::vector<std::string> __split__(const std::string &str, char splitor, bool ignoreEmpty = false) {
      std::vector<std::string> vec;
      auto iter = str.cbegin();
      while (true) {
        auto pos = std::find(iter, str.cend(), splitor);
        auto elem = std::string(iter, pos);
        if ((!ignoreEmpty) || (ignoreEmpty && !elem.empty()))
          vec.push_back(elem);
        if (pos == str.cend())
          break;
        iter = ++pos;
      }
      return vec;
    }

    /**
     * @brief a function to combine some string elements to a string according the splitor
     *
     * @param str the string to be splited
     * @param splitor the splitor char
     * @param ignoreEmpty whether ignoring the empty string element or not
     *
     * @return the combined string
     */
    static std::string __combine__(const std::vector<std::string> &strVec, char splitor, bool ignoreEmpty = false) {
      std::stringstream stream;
      for (const auto &elem : strVec) {
        if (elem.empty() && ignoreEmpty) {
          continue;
        }
        stream << elem << splitor;
      }
      std::string str = stream.str();
      str.pop_back();
      return str;
    }

    template <typename ArgvType>
    void __print__(std::ofstream &ofs, char splitor, const ArgvType &argv) {
      ofs << argv << '\n';
      return;
    }

    /**
     * @brief print the argvs with template param list
     *
     * @param ofs the output file stream
     * @param splitor the splitor
     * @param argv one of the argvs
     * @param argvs the else argvs
     */
    template <typename ArgvType, typename... ArgvsType>
    void __print__(std::ofstream &ofs, char splitor, const ArgvType &argv, const ArgvsType &...argvs) {
      ofs << argv << splitor;
      return __print__(ofs, splitor, argvs...);
    }

    static std::stringstream &operator>>(std::stringstream &os, char *str) {
      // don't define, 'os >> str' will escape the space
      strcpy(str, os.str().c_str());
      return os;
    }

    static std::stringstream &operator>>(std::stringstream &os, std::string &str) {
      // don't define, 'os >> str' will escape the space
      str = os.str();
      return os;
    }

// Get the type of a struct member variable
#define STRUCT_MEM_TYPE(TYPE, MEMBER) decltype(((struct TYPE *)0)->MEMBER)

    // Boxing a structure member variable to obtain its type and offset
    template <typename MemType, std::size_t MemOffset>
    struct STRUCT_MEM_PACK {
      using mem_type = MemType;
      static constexpr std::size_t mem_offset = MemOffset;
    };

    // Template functions for assigning values to structural objects
    template <typename StructType>
    void __str_vec_to_obj__(const std::vector<std::string> &strVec, StructType &obj, std::size_t strIdx = 0) {
    }

    // Template functions for assigning values to structural objects
    template <typename StructType, typename MemPack, typename... MemPacks>
    void __str_vec_to_obj__(const std::vector<std::string> &strVec, StructType &obj, std::size_t strIdx = 0) {
      std::stringstream stream;
      typename MemPack::mem_type elem{};
      if (!strVec.at(strIdx).empty()) {
        // if 'cur str' is empty, use defalut to agssign the member
        stream << strVec.at(strIdx);
        stream >> elem;
      }
      // get reference of the member
      *((typename MemPack::mem_type *)((char *)(&obj) + MemPack::mem_offset)) = elem;
      __str_vec_to_obj__<StructType, MemPacks...>(strVec, obj, strIdx + 1);
    }

    template <typename StructType>
    void __obj_to_str_vec__(std::vector<std::string> &strVec, const StructType &obj, std::size_t strIdx = 0) {
    }

    template <typename StructType, typename MemPack, typename... MemPacks>
    void __obj_to_str_vec__(std::vector<std::string> &strVec, const StructType &obj, std::size_t strIdx = 0) {
      std::stringstream stream;
      // get reference of the member
      stream << *((typename MemPack::mem_type *)((char *)(&obj) + MemPack::mem_offset));
      stream >> strVec.at(strIdx);
      __obj_to_str_vec__<StructType, MemPacks...>(strVec, obj, strIdx + 1);
    }

#pragma endregion

#pragma region macros

// Auxiliary macro used to box a structure type and one of its member variables
#define CSV_STRUCT_MEM(TYPE, MEMBER) \
  ns_csv::ns_priv::STRUCT_MEM_PACK<STRUCT_MEM_TYPE(TYPE, MEMBER), offsetof(TYPE, MEMBER)>

// a macro launcher
#define _MACRO_SELF(X) X
#define _CSV_MACRO_VAR_ARGS_IMPL_COUNT(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, N, ...) N
#define _CSV_COUNT_MACRO_VAR_ARGS(...) _MACRO_SELF(_CSV_MACRO_VAR_ARGS_IMPL_COUNT(__VA_ARGS__, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0))

#define _CSV_MACRO_COMBINE_2(MACRO, ARGS_COUNT) MACRO##ARGS_COUNT
#define _CSV_MACRO_COMBINE_1(MACRO, ARGS_COUNT) _CSV_MACRO_COMBINE_2(MACRO, ARGS_COUNT)
#define _CSV_MACRO_COMBINE(MACRO, ARGS_COUNT) _CSV_MACRO_COMBINE_1(MACRO, ARGS_COUNT)

#define _CSV_MACRO_LAUNCHER(MACRO, ...) \
  _MACRO_SELF(_CSV_MACRO_COMBINE(MACRO, _CSV_COUNT_MACRO_VAR_ARGS(__VA_ARGS__))(__VA_ARGS__))

#define _CSV_STRUCT_2(TYPE, MEMBER) CSV_STRUCT_MEM(TYPE, MEMBER)
#define _CSV_STRUCT_3(TYPE, MEMBER, ...) CSV_STRUCT_MEM(TYPE, MEMBER), _CSV_STRUCT_2(TYPE, __VA_ARGS__)
#define _CSV_STRUCT_4(TYPE, MEMBER, ...) CSV_STRUCT_MEM(TYPE, MEMBER), _CSV_STRUCT_3(TYPE, __VA_ARGS__)
#define _CSV_STRUCT_5(TYPE, MEMBER, ...) CSV_STRUCT_MEM(TYPE, MEMBER), _CSV_STRUCT_4(TYPE, __VA_ARGS__)
#define _CSV_STRUCT_6(TYPE, MEMBER, ...) CSV_STRUCT_MEM(TYPE, MEMBER), _CSV_STRUCT_5(TYPE, __VA_ARGS__)
#define _CSV_STRUCT_7(TYPE, MEMBER, ...) CSV_STRUCT_MEM(TYPE, MEMBER), _CSV_STRUCT_6(TYPE, __VA_ARGS__)
#define _CSV_STRUCT_8(TYPE, MEMBER, ...) CSV_STRUCT_MEM(TYPE, MEMBER), _CSV_STRUCT_7(TYPE, __VA_ARGS__)
#define _CSV_STRUCT_9(TYPE, MEMBER, ...) CSV_STRUCT_MEM(TYPE, MEMBER), _CSV_STRUCT_8(TYPE, __VA_ARGS__)
#define _CSV_STRUCT_10(TYPE, MEMBER, ...) CSV_STRUCT_MEM(TYPE, MEMBER), _CSV_STRUCT_9(TYPE, __VA_ARGS__)

// Select which macro to call according to the number of parameters
// If the structure has more than 9 member variables, you need to directly use macro 'CSV_STRUCT_MEM'
#define CSV_STRUCT(TYPE, ...) \
  TYPE, _CSV_MACRO_LAUNCHER(_CSV_STRUCT_, TYPE, __VA_ARGS__)

#pragma endregion

  } // namespace ns_priv

#pragma region csv reader

  namespace ns_priv {
    class Reader {
    public:
      Reader(std::ifstream *ifs) : _ifs(ifs) {}

      virtual ~Reader() {}

      /**
       * @brief get next std::string vector and assign to the elems
       */
      template <typename... ElemTypes>
      bool readLine(char splitor = ',', ElemTypes &...elems) {
        std::string str;
        if (std::getline(*(this->_ifs), str)) {
          auto strVec = ns_priv::__split__(str, splitor, false);
          strVec.resize(sizeof...(ElemTypes));
          this->parse(strVec, 0, elems...);
          return true;
        }
        return false;
      }

    protected:
      template <typename ElemType, typename... ElemTypes>
      void parse(const std::vector<std::string> &strVec, std::size_t index,
                 ElemType &elem, ElemTypes &...elems) {
        std::stringstream stream;
        if (!strVec.at(index).empty()) {
          stream << strVec.at(index);
          stream >> elem;
        } else {
          // empty
          elem = ElemType{};
        }
        return this->parse(strVec, index + 1, elems...);
      }

      void parse(const std::vector<std::string> &strVec, std::size_t index) {
        return;
      }

    protected:
      /**
       * @brief the input file stream
       */
      std::ifstream *_ifs;

    private:
      Reader() = delete;
      Reader(const Reader &) = delete;
      Reader(Reader &&) = delete;
      Reader &operator=(const Reader &) = delete;
      Reader &operator=(Reader &&) = delete;
    };

    class StreamReader : public Reader {

    public:
      StreamReader(std::ifstream &ifs) : Reader(&ifs) {
        if (!this->_ifs->is_open()) {
          THROW_EXCEPTION(CSVReader, "the file stream may be invalid");
        }
      }

      virtual ~StreamReader() override {}
    };

    class FileReader : public Reader {

    public:
      FileReader(const std::string &filename) : Reader(new std::ifstream(filename)) {
        if (!this->_ifs->is_open()) {
          THROW_EXCEPTION(CSVReader, "the file name may be invalid");
        }
      }

      virtual ~FileReader() override {
        delete this->_ifs;
      }
    };

  } // namespace ns_priv

  class CSVReader {
  public:
    using Ptr = std::shared_ptr<ns_priv::Reader>;

  public:
    /**
     * @brief create a file reader pointer
     *
     * @param filename the name of the csv file
     * @return Ptr
     */
    static Ptr create(const std::string &filename) {
      return std::make_shared<ns_priv::FileReader>(filename);
    }

    /**
     * @brief create a input file stream reader pointer
     *
     * @param ifs input file stream
     * @return Ptr
     */
    static Ptr create(std::ifstream &ifs) {
      return std::make_shared<ns_priv::StreamReader>(ifs);
    }

  public:
    /**
     * @brief read all items in the ifstream
     *
     * @param ifs the input fstream
     * @param splitor the splitor
     *
     * @return std::vector<itemType> data
     */
    template <typename StructType, typename... MemPacks>
    static std::vector<StructType> read(std::ifstream &ifs, char splitor) {
      std::vector<StructType> data;
      std::string strLine;
      StructType obj{};
      while (std::getline(ifs, strLine)) {
        auto strVec = ns_csv::ns_priv::__split__(strLine, splitor, false);
        strVec.resize(sizeof...(MemPacks));
        ns_priv::__str_vec_to_obj__<StructType, MemPacks...>(strVec, obj);
        data.push_back(obj);
      }
      return data;
    }

    /**
     * @brief read all items in the ifstream with header
     *
     * @param ifs the input fstream
     * @param splitor the splitor
     *
     * @return std::vector<itemType> data
     */
    template <typename StructType, typename... MemPacks>
    static auto readWithHeader(std::ifstream &ifs, char splitor) {
      std::string strLine;
      // header
      std::getline(ifs, strLine);
      auto vec = ns_csv::ns_priv::__split__(strLine, splitor, false);
      vec.resize(sizeof...(MemPacks));
      std::array<std::string, sizeof...(MemPacks)> header;
      for (int i = 0; i != header.size(); ++i) {
        header.at(i) = vec.at(i);
      }
      // content
      std::vector<StructType> data = read<StructType, MemPacks...>(ifs, splitor);
      return std::make_pair(header, data);
    }

    /**
     * @brief read some items in the ifstream
     *
     * @param ifs the input fstream
     * @param splitor the splitor
     * @param itemNum the number of the items to read
     *
     * @return std::vector<itemType> data
     */
    template <typename StructType, typename... MemPacks>
    static std::vector<StructType> read(std::ifstream &ifs, char splitor, std::size_t itemNum) {
      std::vector<StructType> data;
      std::string strLine;
      StructType obj{};
      int itemCount = 0;
      while (itemCount++ < itemNum) {
        if (std::getline(ifs, strLine)) {
          auto strVec = ns_csv::ns_priv::__split__(strLine, splitor, false);
          strVec.resize(sizeof...(MemPacks));
          ns_priv::__str_vec_to_obj__<StructType, MemPacks...>(strVec, obj);
          data.push_back(obj);
        } else {
          break;
        }
      }
      return data;
    }

    /**
     * @brief read some items in the ifstream with header
     *
     * @param ifs the input fstream
     * @param splitor the splitor
     * @param itemNum the number of the items to read
     *
     * @return std::vector<itemType> data
     */
    template <typename StructType, typename... MemPacks>
    static auto readWithHeader(std::ifstream &ifs, char splitor, std::size_t itemNum) {
      std::string strLine;
      // header
      std::getline(ifs, strLine);
      auto vec = ns_csv::ns_priv::__split__(strLine, splitor, false);
      vec.resize(sizeof...(MemPacks));
      std::array<std::string, sizeof...(MemPacks)> header;
      for (int i = 0; i != header.size(); ++i) {
        header.at(i) = vec.at(i);
      }
      // content
      std::vector<StructType> data = read<StructType, MemPacks...>(ifs, splitor, itemNum);
      return std::make_pair(header, data);
    }

    /**
     * @brief read all items in the file
     *
     * @param fileName the file name
     * @param splitor the splitor
     *
     * @return std::vector<itemType> data
     */
    template <typename StructType, typename... MemPacks>
    static std::vector<StructType> read(const std::string &fileName, char splitor) {
      std::ifstream ifs(fileName);
      auto data = CSVReader::read<StructType, MemPacks...>(ifs, splitor);
      ifs.close();
      return data;
    }

    /**
     * @brief read all items in the file with header
     *
     * @param fileName the file name
     * @param splitor the splitor
     *
     * @return std::vector<itemType> data
     */
    template <typename StructType, typename... MemPacks>
    static auto readWithHeader(const std::string &fileName, char splitor) {
      std::ifstream ifs(fileName);
      auto data = CSVReader::readWithHeader<StructType, MemPacks...>(ifs, splitor);
      ifs.close();
      return data;
    }
  };

#pragma endregion

#pragma region csv writer
  namespace ns_priv {
    class Writer {
    public:
      Writer(std::ofstream *ofs) : _ofs(ofs) {}

      virtual ~Writer() {}

      /**
       * @brief use variable template parameters to write any num arguements
       */
      template <typename... Types>
      void writeLine(char splitor, const Types &...argvs) {
        ns_priv::__print__(*(this->_ofs), splitor, argvs...);
        return;
      }

      void setPrecision(std::size_t prec) {
        *(this->_ofs) << std::fixed << std::setprecision(prec);
        return;
      }

    protected:
      /**
       * @brief the output file stream
       */
      std::ofstream *_ofs;

    private:
      Writer() = delete;
      Writer(const Writer &) = delete;
      Writer(Writer &&) = delete;
      Writer &operator=(const Writer &) = delete;
      Writer &operator=(Writer &&) = delete;
    };

    class StreamWriter : public Writer {
    public:
      StreamWriter(std::ofstream &ofs) : Writer(&ofs) {
        if (!this->_ofs->is_open()) {
          THROW_EXCEPTION(CSVWriter, "the file stream may be invalid");
        }
      }

      virtual ~StreamWriter() override {}
    };

    class FileWriter : public Writer {
    public:
      FileWriter(const std::string &filename) : Writer(new std::ofstream(filename)) {
        if (!this->_ofs->is_open()) {
          THROW_EXCEPTION(CSVWriter, "the file name may be invalid");
        }
      }

      virtual ~FileWriter() override {
        delete this->_ofs;
      }
    };

  } // namespace ns_priv

  class CSVWriter {
  public:
    using Ptr = std::shared_ptr<ns_priv::Writer>;

  public:
    /**
     * @brief create a file writer pointer
     *
     * @param filename the name of the csv file
     * @return Ptr
     */
    static Ptr create(const std::string &filename) {
      return std::make_shared<ns_priv::FileWriter>(filename);
    }

    /**
     * @brief create a output file stream writer pointer
     *
     * @param ifs output file stream
     * @return Ptr
     */
    static Ptr create(std::ofstream &ofs) {
      return std::make_shared<ns_priv::StreamWriter>(ofs);
    }

  public:
    /**
     * @brief write data to a csv file
     *
     * @param ofs the out fstream
     * @param splitor the splitor
     * @param data the data array
     */
    template <typename StructType, typename... MemPacks>
    static void write(std::ofstream &ofs, char splitor, const std::vector<StructType> &data) {
      std::vector<std::string> strVec(sizeof...(MemPacks));
      for (const auto &elem : data) {
        ns_priv::__obj_to_str_vec__<StructType, MemPacks...>(strVec, elem);
        ofs << ns_priv::__combine__(strVec, splitor, false) << '\n';
      }
    }

    /**
     * @brief write data to a csv file
     *
     * @param ofs the out fstream
     * @param splitor the splitor
     * @param header the header labels
     * @param data the data array
     */
    template <typename StructType, typename... MemPacks>
    static void writeWithHeader(std::ofstream &ofs, char splitor,
                                const std::array<std::string, sizeof...(MemPacks)> &header,
                                const std::vector<StructType> &data) {
      std::vector<std::string> strVec(sizeof...(MemPacks));
      for (int i = 0; i != header.size(); ++i) {
        strVec.at(i) = header.at(i);
      }
      ofs << ns_priv::__combine__(strVec, splitor, false) << '\n';
      CSVWriter::write<StructType, MemPacks...>(ofs, splitor, data);
    }

    /**
     * @brief write data to a csv file
     *
     * @param fileName the file name
     * @param splitor the splitor
     * @param data the data array
     */
    template <typename StructType, typename... MemPacks>
    static void write(const std::string &fileName, char splitor, const std::vector<StructType> &data) {
      std::ofstream ofs(fileName);
      CSVWriter::write<StructType, MemPacks...>(ofs, splitor, data);
      ofs.close();
    }

    /**
     * @brief write data to a csv file with header
     *
     * @param fileName the file name
     * @param splitor the splitor
     * @param header the header labels
     * @param data the data array
     */
    template <typename StructType, typename... MemPacks>
    static void writeWithHeader(const std::string &fileName, char splitor,
                                const std::array<std::string, sizeof...(MemPacks)> &header,
                                const std::vector<StructType> &data) {
      std::ofstream ofs(fileName);
      CSVWriter::writeWithHeader<StructType, MemPacks...>(ofs, splitor, header, data);
      ofs.close();
    }
  };

#pragma endregion

#undef THROW_EXCEPTION
} // namespace ns_csv
