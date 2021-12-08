
# PyGEO
> author : csl  
> e-mail : 3079625093@qq.com

---
## ___1. OverView___
### ___1) Part One: Legendre___
#### this class is designed to calculate the normalized spherical harmonic function.

#### This class provides four methods for calculating Legendre function:   
  +   ___[1] standard forward column method___  
  +   ___[2] standard forward row method___  
  +   ___[3] cross order recursive method___  
  +   ___[4] Belikov column method___   
#### And each method has two implementation methods: recursion and loop.
### ___2) Part Two: Gravity Anomality___
#### Because the material density distribution in the actual earth is very uneven, there is always a deviation between the actual observed gravity value and the theoretical normal gravity value. After excluding the influence of various interference factors, the change of gravity caused only by the uneven material density distribution is called gravity anomaly. 
## ___2. Result Display___

### ___1) Part One: Legendre___

>___Calculate Result___  

<img src="./images/excel/result.png" width="80%">

>___RunTime___  

<img src="./images/excel/time.png" width="30%">

>___Other Images___   

<img src="./images/pcl/img1.png" width="50%"><img src="./images/pcl/img2.png" width="50%"><img src="./images/pcl/img3.png" width="50%"><img src="./images/pcl/img4.png" width="50%">

### ___2) Part Two: Gravity Anomality___

>___data___

```cpp
# BGI DATA EXTRACTION
# REQUEST : latitude =38.30000000/41.80000000  longitude =-84.90000000/-80.50000000  country=102
# source	lat(deg)	long(deg)	alt(m) 	g(mgal)
31020015 40.32310000 -84.89990000 320.5 980105.99
31020015 40.36630000 -84.89990000 297.5 980113.56
31020015 40.39660000 -84.89990000 285.9 980116.49
31020489 39.05830000 -84.89990000 150.5 980000.00
31020489 41.07660000 -84.89990000 233.9 980209.80
31020552 40.51280000 -84.89880000 262.7 980136.51
31020552 40.41830000 -84.89710000 281.0 980118.76
31020015 40.41810000 -84.89660000 281.1 980118.82
...
```

>___statisic info___
```cpp
min or pos 38.3316 -80.6483
max or pos 39.5832 -83.4367
min originGA -197.08183
max originGA -38.51461
mean originGA -92.90380520595969
stdevp originGA 17.148267188068175

min sp pos 41.1525 -83.881
max sp pos 39.5217 -83.4882
min spayialGA -57.3588
max spayialGA 54.5752
mean spayialGA -8.65432242915571
stdevp spayialGA 18.781959150775112

min sb pos 41.1525 -83.881
max sb pos 39.5832 -83.4367
min simBouGA -82.07751
max simBouGA 20.76423
mean simBouGA -39.205662196903305
stdevp simBouGA 16.15846141380393
```

>___Area___   

<img src="./images/arcgis/map.png" width="100%">


>___Python___

<img src="./images/python/location.png" width="50%"><img src="./images/python/Altitude.png" width="50%"><img src="./images/python/Gravity.png" width="50%"><img src="./images/python/OriginGA.png" width="50%"><img src="./images/python/SpatialCorrection.png" width="50%"><img src="./images/python/LayerCorrection.png" width="50%"><img src="./images/python/SpatialGA.png" width="50%"><img src="./images/python/SimpleBouguerGA.png" width="50%">

>___ArcGIS___   


<img src="./images/arcgis/obverSat.png" width="50%"><img src="./images/arcgis/gravity.png" width="50%"><img src="./images/arcgis/altitude.png" width="50%"><img src="./images/arcgis/layerCorr.png" width="50%"><img src="./images/arcgis/originGA.png" width="50%"><img src="./images/arcgis/simpleBouguer.png" width="50%"><img src="./images/arcgis/SpatialCorr.png" width="50%"><img src="./images/arcgis/spatialGA.png" width="50%">

### ___3) Part Three: Gravity Calculation___

>___ArcGIS___   


<img src="./images/arcgis/global_disPot.png" width="50%"><img src="./images/arcgis/global_geoidHeight.png" width="50%"><img src="./images/arcgis/global_gravityAnomal.png" width="50%"><img src="./images/arcgis/global_gravityDis.png" width="50%"><img src="./images/arcgis/taiwan_disPot.png" width="50%"><img src="./images/arcgis/taiwan_geoidHeight.png" width="50%"><img src="./images/arcgis/taiwan_gravityAnomal.png" width="50%"><img src="./images/arcgis/taiwan_gravityDis.png" width="50%">


## ___3. Third Party___
> ___PCL library___   
> ___colorPrj[self-build]___   
## ___4. Code Structure___
### ___0) file structure___

```cpp

include/
├── gravityAnomal.h
├── legendre.h
├── parameter.hpp
└── surpharmonic.h
src/
├── gravityAnomal.cpp
├── legendre.cpp
└── surpharmonic.cpp
CMakeLists.txt 
main.cpp 
readme.md 
thirdparty/
├── include
│   └── colorPrj.h
└── lib
    └── libcolorPrj.so
data/
├── file_structure.txt
├── inter
│   ├── altFile_inter.txt
│   ├── gravityFile_inter.txt
│   ├── layerCorrFile_inter.txt
│   ├── originalGAFile_inter.txt
│   ├── simpleBouguerGAFile_inter.txt
│   ├── spatialCorrFile_inter.txt
│   └── spatialGAFile_inter.txt
└── origin
    ├── altFile.txt
    ├── data_usa_ohio_dat_110213051620.txt.dat
    ├── gravityFile.txt
    ├── interploation
    ├── layerCorrFile.txt
    ├── originalGAFile.txt
    ├── simpleBouguerGAFile.txt
    ├── spatialCorrFile.txt
    └── spatialGAFile.txt
pyDrawer/
├── location.py
├── otherData.py
├── __pycache__
│   └── reader.cpython-38.pyc
└── reader.py
```

### ___1) Part One: Legendre___

<img src="./images/xmind/legendre.png" width="100%">  

### ___header file___
>___legendre.h___
```cpp
#pragma region class IndexOrder
    /**
     * \brief this class serves the Legendre class
     */
    class IndexOrder
    {
    public:
        int _n;
        int _m;
        IndexOrder(int index_n, int order_m) : _n(index_n), _m(order_m) {}
        ~IndexOrder() {}
        // overload the operator '=='
        bool operator==(const IndexOrder &i) const;
        // hash function to get the hash value for each object
        static std::size_t indexorder_hash(const IndexOrder &i);
    };
#pragma endregion

#pragma region class Legendre
    /**
     *  \class Legendre [static class]
     *  \brief this class is designed to calculate the normalized spherical harmonic function.
     *         This class provides four methods for calculating Legendre function: 
     *              [1] standard forward column method
     *              [2] standard forward row method
     *              [3] cross order recursive method 
     *              [4] Belikov column method
     */
    class Legendre
    {
    public:
        using value_type = long double;
        /**
         * \brief the function pointer
         */
        using legendre_fun = value_type (*)(int index_n, int order_m, float angle);
        using indexorder_hashfun = std::size_t (*)(const IndexOrder &i);
        using legendre_map = std::unordered_map<IndexOrder, value_type, indexorder_hashfun>;
        // [P_0_0] [P_1_0] [P_1_1] [cos_angle] [sin_angle] [cot_angle] [return value] [is reture]
        using check_tuple = std::tuple<value_type, value_type, value_type, value_type, value_type, value_type, value_type, bool>;

    private:
        Legendre() = default;
        ~Legendre() {}
        /**
         * \brief two middle functions in the belikov method
         */
        static value_type _belikov_denorm(int index_n, int order_m, float angle);
        static value_type _belikov_denorm(int index_n, int order_m);
        /**
         * \brief function to check and initialize the necessary things before starting the algorithm
         * \return [P_0_0] [P_1_0] [P_1_1] [cos_angle] [sin_angle] [cot_angle] [return value] [is reture]
         */
        static check_tuple _check_init(int index_n, int order_m, float radian_angle);

    public:
        /**
         * \brief loop methods
         * \attention the angle is passed by the radian form
         */
        // [1] standard forward column method
        static value_type forwardColumn(int index_n, int order_m, float radian_angle);
        static value_type forwardColumn(const IndexOrder &io, float radian_angle);
        // [2] standard forward row method
        static value_type forwardRow(int index_n, int order_m, float radian_angle);
        static value_type forwardRow(const IndexOrder &io, float radian_angle);
        // [3] cross order recursive method
        static value_type crossOrder(int index_n, int order_m, float radian_angle);
        static value_type crossOrder(const IndexOrder &io, float radian_angle);
        // [4] Belikov column method
        static value_type belikov(int index_n, int order_m, float radian_angle);
        static value_type belikov(const IndexOrder &io, float radian_angle);
        /**
         *  accuracy estimation
         * \attention the angle is passed by the radian form
         */
        static value_type accuracy_estimation(int index_n, float radian_angle, legendre_fun fun);
        /**
         * \brief calculate all elememts from [n = 0,m = 0] to [n = end_n,m = end_n]
         *        based on the standard forward column method
         *        It's faster than you calculate one by one
         * \attention the angle is passed by the radian form
         * \return a map contains all results
         */
        static legendre_map legendre_dataset(int end_n, float radian_angle);
    };
#pragma endregion
```

>___surpharmonic.h___

```cpp
#pragma region class SurPharmonic
    /**
     * \brief this is a display-class for visualation based on pcl library
     */
    class SurPharmonic
    {
    public:
        /**
         * \brief the function pointer
         * \param val the float value
         * \return float value
         */
        using tri_fun = float (*)(float val);

    private:
        SurPharmonic() = default;
        ~SurPharmonic() {}
        /**
         * \brief project the value range to rgb color space
         * \param val the value to project
         * \param min the low boundary of the value range
         * \param max the high boundary of the value range
         * \param tri_fun the trigonometric function
         * \param colorModel the display color type
         * \param classify the color grades
         */
        static void harmonic(int index_n, int order_m, float min, float max, SurPharmonic::tri_fun tri_fun,
                             const ns_clp::Color::ColorType &colorModel = ns_clp::Color::gray,
                             int classify = 0);

    public:
        // the R_nm function to display the SurPharmonic with pcl window [color]
        static void harmonic_Rnm(int index_n, int order_m,
                                 const ns_clp::Color::ColorType &colorModel = ns_clp::Color::gray,
                                 int classify = 0, float min = -3.5, float max = 3.5);
        static void harmonic_Rnm(const IndexOrder &i,
                                 const ns_clp::Color::ColorType &colorModel = ns_clp::Color::gray,
                                 int classify = 0, float min = -3.5, float max = 3.5);

        // the S_nm function to display the SurPharmonic with pcl window [color]
        static void harmonic_Snm(int index_n, int order_m,
                                 const ns_clp::Color::ColorType &colorModel = ns_clp::Color::gray,
                                 int classify = 0, float min = -3.5, float max = 3.5);
        static void harmonic_Snm(const IndexOrder &i,
                                 const ns_clp::Color::ColorType &colorModel = ns_clp::Color::gray,
                                 int classify = 0, float min = -3.5, float max = 3.5);
    };
#pragma endregion
```

### ___2) Part Two: Gravity Anomality___

<img src="./images/xmind/gravityAnomality.png" width="100%"> 

>___garvityAnomal.h___

```cpp
#pragma region class BGIReader
    /**
     * \brief a class serves for the \class GravityAnomal
     */ 
    class BGIReader
    {
    public:
        struct DataItem
        {
            int _source;
            // latitude unit[degree]
            double _lat;
            // Longitude unit[degree]
            double _long;
            // unit[m]
            double _alt;
            // Gravity observation unit[mGal]
            double _g;

            DataItem() = default;
        };

    private:
        // the header string for the BGI file
        std::string _header;
        // the data organized by DataItem
        std::list<DataItem> _data;

    public:
        BGIReader() = delete;
        /**
         * \brief pass a file name and init the class object
         */ 
        BGIReader(const std::string &fileName) { this->readData(fileName); }

    public:
        // get the header
        inline const std::string &header() const { return this->_header; }
        // get the data items
        inline const std::list<DataItem> &data() const { return this->_data; }

    private:
        // read the data and save them
        void readData(const std::string &fileName);
        // split a string into some small strings based on the splitor
        static std::vector<std::string> split(const std::string &str, char splitor);
    };
#pragma endregion

#pragma region class GravityAnomal
    /**
     * \brief the main class to calculate the gravity anomality
     */ 
    class GravityAnomal
    {
    public:
        struct GAItem
        {
            const BGIReader::DataItem *_dataItem;
            // [g - gamma] unit[mGal]
            double _originalGA;
            // [deta_1_g] unit[mGal]
            double _spatialCorr;
            // [deta_2_g] unit[mGal]
            double _layerCorr;

            GAItem() = default;
            // calculate and get the spatial gravity anomality
            double spatialGA() const { return this->_originalGA + this->_spatialCorr; }
            // calculate and get the simple Bouguer gravity anomality
            double simpleBouguerGA() const { return this->_originalGA + this->_spatialCorr + this->_layerCorr; }
        };

    private:
        // the data
        std::list<GAItem> _data;
        // the reference ellipsoid
        const ns_pygeo::params::Ellipsoid *_ellipsoid;

    public:
        GravityAnomal() = delete;
        /**
         * \brief construct a gravity anomality calculator
         * \param reader a BGIReader  pointer contains the data
         * \param ellipsoid the reference ellipsoid
         */
        GravityAnomal(const BGIReader *reader, const ns_pygeo::params::Ellipsoid *ellipsoid)
            : _ellipsoid(ellipsoid) { this->init(reader); };

    public:
        const std::list<GAItem> &data() const { return this->_data; }

    private:
        // calculate the values of  different gravity anomality correction
        void init(const BGIReader *reader);
        // calculate the spatial correction
        static double spatialCorrection(double alt);
        // calculate the layer correction
        static double layerCorrection(double alt);
        /**
         * \brief calculate the original gravity anomality
         * \param g the Gravity observations
         * \param ellipsoid the reference ellipsoid
         * \param radian_lat the latitude [radian]
         */
        static double originalGA(double g, const ns_pygeo::params::Ellipsoid *ellipsoid, double radian_lat);
    };
#pragma endregion

#pragma region operators overload
    // operator overload [BGIReader::DataItem]
    std::ostream &operator<<(std::ostream &os, const BGIReader::DataItem &item);

    // operator overload [GravityAnomal::GAItem]
    std::ostream &operator<<(std::ostream &os, const GravityAnomal::GAItem &item);
#pragma endregion

```

>___parameter.hpp___

```cpp
#pragma region namespace params
    namespace params
    {
        constexpr double PI = 3.1415926535;

        constexpr double G = 6.67259E-11;

        class Ellipsoid
        {
        private:
            // semimajor axis of the ellipsoid [m]
            const double _a;
            // flattening of the ellipsoid [1]
            const double _f;
            // geocentric gravitational constant of the earth(including the atmosphere) [m^3 * s^(-2)]
            const double _GM;
            // angular velocity of the earth [rad * s^(-1)]
            const double _omega;

        public:
            Ellipsoid() = delete;
            Ellipsoid(const double a, const double f, const double GM, const double omega)
                : _a(a), _f(f), _GM(GM), _omega(omega) {}

        public:
            inline double a() const { return this->_a; }
            inline double f() const { return this->_f; }
            inline double GM() const { return this->_GM; }
            inline double omega() const { return this->_omega; }
            // semiminor axis of the ellipsoid [m]
            inline double b() const { return this->_a * (1.0 - this->_f); }
            // first eccentricity [1]
            inline double e_fir() const { return std::sqrt(_a * _a - b() * b()) / _a; }
            // first eccentricity squared [1]
            inline double e_fir_2() const { return (_a * _a - b() * b()) / (_a * _a); }
            // second eccentricity [1]
            inline double e_sed() const { return std::sqrt(_a * _a - b() * b()) / b(); }
            // second eccentricity squared [1]
            inline double e_sed_2() const { return (_a * _a - b() * b()) / (b() * b()); }
            // linear eccentricity [1]
            inline double E() const { return std::sqrt(_a * _a - b() * b()); }
            // polar radius of curvature [m]
            inline double c() const { return _a * _a / b(); }
            // normal potential at the ellipsoid [m^2 * s^(-2)]
            inline double U_0() const { return _GM / E() * std::atan(E() / b()) + _omega * _omega * _a * _a / 3.0; }
            // normal gravity at the equator [m * s^(-2)]
            inline double gamma_a() const
            {
                double m = _omega * _omega * _a * _a * b() / _GM;
                return _GM / (_a * b()) * (1.0 - 1.5 * m - 3.0 * e_sed_2() * m / 14.0);
            }
            // normal gravity at the pole [m * s^(-2)]
            inline double gamma_b() const
            {
                double m = _omega * _omega * _a * _a * b() / _GM;
                return _GM / (_a * _a) * (1.0 + m + 3.0 * e_sed_2() * m / 7.0);
            }
            // m = std::pow(omega * a, 2) * b / GM
            inline double m() const { return _omega * _omega * _a * _a * b() / _GM; }
            // mass of the earth (includes atmosphere) [kg]
            inline double M() const { return _GM / G; }
            // the normal gravity [phi radian]
            inline double normalGravity(double phi) const
            {
                auto v1 = _a * std::cos(phi) * std::cos(phi);
                auto v2 = b() * std::sin(phi) * std::sin(phi);
                return (v1 * gamma_a() + v2 * gamma_b()) / std::sqrt(v1 * _a + v2 * b());
            }
            // the normal gravity [phi radian]
            inline double gamma(double radian_phi) const { return normalGravity(radian_phi); }

        private:
            inline double q(double u) const
            {
                return 0.5 * ((1.0 + 3.0 * (u * u) / (E() * E()) * std::atan(E() / u) - 3.0 * u / E()));
            }
            inline double q_prime(double u) const { return 3.0 * (1.0 + (u * u) / (E() * E())) * (1.0 - u * std::atan(E() / u) / E()) - 1.0; }
        };

        // the WGS 84 reference ellipsoid
        const Ellipsoid WGS_84(6378137.000, 1.0 / 298.257223563, 3986004.418E08, 7292115.000E-11);

    } // namespace params
#pragma endregion
```

### ___3) Part Three: GFC Gravity Calculation___

<img src="./images/xmind/GRSGravity.png" width="100%">

>___gravityField.h___
```cpp
    /**
     * \brief a cpp class to handle the GFC data calculation
     */
    class GFCHandler
    {
    public:
        using parma2_getfun = int (*)(int n);

    public:
        /**
         * \brief the type and members of each data item
         */
        struct DataItem
        {
            ns_pygeo::IndexOrder _nm;

            double _C;
            double _S;

            DataItem() = default;
        };
        /**
         * \brief the members of each calculation result
         */
        struct CalResult
        {
            ns_point::Point3d _p;
            double _disturbancePotential;
            double _geoidHeight;
            double _gravityAnomal;
            double _gravityDisturbance;
        };

    private:
        /**
         * \brief members for reading
         */
        std::string _header;
        std::vector<DataItem> _data;
        /**
         * \brief member for recording calculating result
         */
        std::vector<CalResult> _result;

    public:
        GFCHandler() = delete;

        GFCHandler(const std::string &filename) { this->readData(filename); }

        /**
         *  \attention the unit is [degree]
         *  \param the calculation range, max index and reference ellipsoid
         */
        void calculation(double lat_lower, double lat_upper, double lat_sep,
                         double long_lower, double long_upper, double long_sep,
                         int end_n, const ns_pygeo::params::Ellipsoid &e);

        /**
         * \brief write the calculation result to the file
         */
        void writeResult(const std::string &filename);

        /**
         * \brief a help function
         * \param sphCoor the Spherical coordinates
         * \param end_n the index
         * \param e the reference Ellipsoid
         * \param param1 it's for [GM / r]||[GM / r ^ 2]||[a]
         * \param param2 a function to get [1]||[n - 1]||[n + 1]
         * \param param3 it's for [a / r]||[(a / r) ^ 2]
         * \param param4 it'is for [Cnm * Cos(m * lambda) + Snm * sin(m * lambda)] * Pnm(Cos)
         */
        double _helpFunction(const ns_point::Point3d &sphCoor, int end_n, const ns_pygeo::params::Ellipsoid &e,
                             double param1, parma2_getfun param2, double params3, const std::vector<double> &param4);

        // just used during designe the algorithm
        double __helpFunction(const ns_point::Point3d &sphCoor, int end_n, const ns_pygeo::params::Ellipsoid &e,
                              double param1, parma2_getfun param2, double params3, Legendre::legendre_map *dt);

        // return function pointer for different problem
        inline parma2_getfun _disturbancePotential() { return nullptr; }

        inline parma2_getfun _geoidHeight() { return nullptr; }

        inline parma2_getfun _gravityAnomal()
        {
            return [](int n) -> int
            {
                return n - 1;
            };
        }

        inline parma2_getfun _gravityDisturbance()
        {
            return [](int n) -> int
            {
                return n + 1;
            };
        }

        // read the gfc data
        void readData(const std::string &filename);

        // get param 'Cu' when calculation
        double getCU(int n, int m, const ns_pygeo::params::Ellipsoid &e);
    };
    /**
     * \brief a function to split a string to some string elements according the splitor
     * \param str the string to be splited
     * \param splitor the splitor char
     * \param ignoreEmpty whether ignoring the empty string element or not
     */
    std::vector<std::string> split(const std::string &str, char splitor, bool ignoreEmpty = true);

    /**
     * \brief a function to output info to the std::ostream
     * \param str the target string
     * \param ceil whether output the ceil line or not
     * \param floor whether output the floor line or not
     * \param symbol the char type to construct the lin
     * \param os the output stream type
     */
    void outputFormat(const std::string &str, bool ceil = false, bool floor = true,
                      char symbol = '-', std::ostream &os = std::cout);
```