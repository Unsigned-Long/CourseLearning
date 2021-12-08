SELECT Z1.name, S1.name FROM zone Z1, streams S1 WHERE st_distance(Z1.boundary, S1.centerline) < 
ALL(SELECT st_distance(Z2.boundary, S1.centerline) FROM zone Z2 WHERE Z1.name <> Z2.name);

SELECT RS1, name FROM road_segments RS1 WHERE st_length(RS1.centerline) > 
ANY(SELECT st_length(RS2.centerline) FROM road_segments RS2 WHERE RS2.name <> '主街');

SELECT name, shores FROM ponds UNION SELECT name, shore FROM lakes;

SELECT * FROM bridges;