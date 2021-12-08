SELECT f_table_name FROM geometry_columns;

SELECT S.auth_srid FROM spatial_ref_sys S, geometry_columns G WHERE G.f_table_name = 'streams' AND S.srid=G.srid;

SELECT st_geometrytype(centerlines) FROM divided_routes WHERE name = '路75';

SELECT st_astext(boundary) FROM island WHERE name = '鹅岛';

SELECT st_isempty(centerline) FROM road_segments WHERE name = '路5' AND aliases = '主街';

SELECT st_issimple(shore) FROM lakes WHERE name = '蓝湖';

SELECT st_astext(st_boundary(boundary)) FROM island WHERE name = '鹅岛';

SELECT st_astext(st_envelope(boundary)) from island WHERE name = '鹅岛';

SELECT st_x(position), st_y(position) FROM bridges WHERE name = '卡姆桥';

SELECT st_astext(st_startpoint(centerline)),st_astext(st_endpoint(centerline)) FROM road_segments WHERE fid = 102;

SELECT st_isclosed(st_boundary(boundary)) FROM island WHERE name = '鹅岛';

SELECT st_length(centerline) FROM road_segments WHERE fid = 106;

SELECT st_numpoints(centerline) FROM road_segments WHERE fid = 102;

SELECT st_astext(st_pointn(centerline,1)) FROM road_segments WHERE fid = 102;

SELECT st_astext(st_centroid(boundary)) FROM island WHERE name = '鹅岛';

SELECT st_contains(boundary,st_pointonsurface(boundary)) FROM island WHERE name = '鹅岛';

SELECT st_area(boundary) FROM island WHERE name = '鹅岛';

SELECT st_numinteriorring(shore) FROM lakes WHERE name = '蓝湖';

SELECT st_numgeometries(centerlines) FROM divided_routes WHERE name = '路75';

SELECT st_astext(st_geometryn(centerlines,2)) FROM divided_routes WHERE name = '路75';

SELECT st_length(centerlines) FROM divided_routes WHERE name = '路75';

SELECT st_equals(boundary,st_polyfromtext('POLYGON((67 13, 67 18, 59 18, 59 13, 67 13))',101)) FROM island  WHERE name = '鹅岛';

SELECT st_disjoint(centerlines,boundary) FROM divided_routes d,zone z WHERE d.name = '路75' AND z.name = '阿斯顿';

SELECT st_touches(s.centerline, l.shore) FROM streams s, lakes l WHERE s.name = '卡姆河' AND l.name = '蓝湖';

SELECT st_within(z.boundary, b.footprint) FROM zone z, buildings b WHERE z.name = '阿斯顿' AND b.address = '主街215号';

SELECT st_crosses(r.centerline, d.centerlines) FROM road_segments r, divided_routes d WHERE r.fid = 102 AND d.name = '路75';

SELECT st_relate(z.boundary, i.boundary,'TTTTTTTTT') FROM zone z, island i WHERE z.name = '阿斯顿' AND i.name = '鹅岛';

SELECT st_distance(position, boundary) FROM bridges b, zone z WHERE b.name = '卡姆桥' AND z.name = '阿斯顿';

SELECT st_astext(st_intersection(s.centerline, l.shore)) FROM streams s, lakes l WHERE s.name = '卡姆河' AND l.name = '蓝湖';

SELECT st_astext(st_union(l.shore, i.boundary)) FROM lakes l, island i WHERE l.name = '蓝湖' AND i.name = '鹅岛';

SELECT st_astext(st_symdifference(l.shore, i.boundary)) FROM lakes l, island i WHERE l.name = '蓝湖' AND i.name = '鹅岛';

SELECT st_astext(st_convexhull(shore)) FROM lakes WHERE name = '蓝湖';