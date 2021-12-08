-- create table landuse(
--     id INTEGER,
--     the_geom geometry
-- );

-- INSERT into landuse VALUES(
--     12,
--     st_geomfromtext('MULTIPOLYGON(((0 0, 4 0, 4 4, 0 4, 0 0)),((5 0, 7 0, 7 7, 5 7,5 0)))')
-- );

-- select st_astext(st_intersection(the_geom,'POLYGON((2 0, 6 0, 6 6, 2 6, 2 0))')) from landuse where id=12;

select st_length(st_geomfromewkt('LINESTRING(0 0, 1 1)')) as length;

select st_length('LINESTRING(0 0, 1 1)'::geometry) as length;

select st_length('LINESTRING(0 0, 1 1)') as length;
