-- lakes
INSERT INTO lakes VALUES(
101,
'蓝湖',
st_geomfromtext('MULTIPOLYGON(((52 18, 66 23, 73 9, 48 6, 52 18),(59 18, 67 18,67 13, 59 13, 59 18)))',101)
);

INSERT INTO lakes VALUES(
0,
'图外其他湖泊'
);
SELECT * FROM lakes;

-- loads
INSERT INTO road_segments VALUES(
102,
'路5',
NULL,
2,
st_geomfromtext('LINESTRING(0 18, 10 21, 16 23, 28 26, 44 31)',101)
);

INSERT INTO road_segments VALUES(
103,
'路5',
'主街',
4,
st_geomfromtext('LINESTRING(44 31, 56 34, 70 38)',101)
);

INSERT INTO road_segments VALUES(
104,
'路5',
NULL,
2,
st_geomfromtext('LINESTRING(70 38, 72 48)',101)
);

INSERT INTO road_segments VALUES(
105,
'主街',
NULL,
4,
st_geomfromtext('LINESTRING(70 38, 84 42)',101)
);

INSERT INTO road_segments VALUES(
106,
'绿森林边路',
NULL,
1,
st_geomfromtext('LINESTRING(28 26, 28 0)',101)
);

-- divided_routes
INSERT INTO divided_routes VALUES(
119,
'路75',
4,
st_geomfromtext('MULTILINESTRING((10 48, 10 21, 10 0),(16 0, 16 23, 16 48))',101)
);

-- bridges
INSERT INTO bridges VALUES(
110,
'卡姆桥',
102,
103,
st_pointfromtext('POINT(44 31)',101)
);

-- rivers
INSERT INTO streams VALUES(
111,
'卡姆河',
0,
101,
st_geomfromtext('LINESTRING(38 48, 44 41, 41 36, 44 31, 52 18)',101)
);
INSERT INTO streams VALUES(
112,
NULL,
101,
0,
st_geomfromtext('LINESTRING(76 0, 78 4, 73 9)',101)
);

-- buildings
INSERT INTO buildings VALUES(
113,
'主街123号',
st_geomfromtext('POINT(52 30)',101),
st_geomfromtext('POLYGON((50 31, 54 31, 54 29, 50 29, 50 31))',101)
);
INSERT INTO buildings VALUES(
114,
'主街215号',
st_geomfromtext('POINT(64 33)',101),
st_geomfromtext('POLYGON((66 34, 62 34, 62 32, 66 32, 66 34))',101)
);

-- ponds
INSERT INTO ponds VALUES(
120,
NULL,
'思道哥池塘',
st_geomfromtext('MULTIPOLYGON(((24 44, 22 42, 24 40, 24 44)),((26 44, 26 40, 28 42, 26 44)))',101)
);

-- island
INSERT INTO island VALUES(
109,
'鹅岛',
101,
st_geomfromtext('MULTIPOLYGON(((67 13, 67 18, 59 18, 59 13, 67 13)))',101)
);

-- zone
INSERT INTO zone VALUES(
117,
'阿斯顿',
st_geomfromtext('MULTIPOLYGON(((62 48, 84 48, 84 30, 56 30, 56 34, 62 48)))',101)
);
INSERT INTO zone VALUES(
118,
'绿森林',
st_geomfromtext('MULTIPOLYGON(((28 26, 28 0, 84 0, 84 42, 28 26),
                (52 18, 66 23, 73 9, 48 6, 52 18),
                (59 18, 67 18, 67 13, 59 13, 59 18)))',101)
);