-- check the version of the postgis
SELECT postgis_full_version();

-- check the WGS84/UTM14 spatial reference system info
SELECT srtext FROM spatial_ref_sys WHERE srid = 32214;
SELECT srtext FROM spatial_ref_sys WHERE srid = 4326;

-- create the lake table
CREATE TABLE lakes(
fid     INTEGER NOT NULL PRIMARY key,
name    VARCHAR(64),
shore   geometry
);

-- create the road table
CREATE TABLE road_segments(
fid         INTEGER NOT NULL PRIMARY key,
name        VARCHAR(64),
aliases     VARCHAR(64),
num_lanes   INTEGER,
centerline  geometry
);

-- create the divided routes table
CREATE TABLE divided_routes(
fid         INTEGER NOT NULL PRIMARY key,
name        VARCHAR(64),
num_lanes   INTEGER,
centerlines geometry
);

-- create bridges table 
CREATE TABLE bridges(
fid         INTEGER NOT NULL PRIMARY KEY,
name        VARCHAR(64),
roadseg1id  INTEGER REFERENCES road_segments,
roadseg2id  INTEGER REFERENCES road_segments,
position    geometry
);

-- create stream table  
CREATE TABLE streams(
fid         INTEGER NOT NULL PRIMARY KEY,
name        VARCHAR(64),
fromlakeid  INTEGER REFERENCES lakes,
tolakeid I  NTEGER REFERENCES lakes,
centerline  geometry
);

-- create buildings table
CREATE TABLE buildings(
fid         INTEGER NOT NULL PRIMARY KEY,
address     VARCHAR(64),
position    geometry,
footprint   geometry
);

-- create ponds table
CREATE TABLE ponds(
fid     INTEGER NOT NULL PRIMARY KEY,
name    VARCHAR(64),
type    VARCHAR(64),
shores  geometry
);

-- create island table
CREATE TABLE island(
fid         INTEGER NOT NULL PRIMARY KEY,
name        VARCHAR(64),
lakeid      INTEGER REFERENCES lakes,
boundary    geometry
);

-- create zone table
CREATE TABLE zone(
fid         INTEGER NOT NULL PRIMARY KEY,
name        VARCHAR(64),
boundary    geometry
);