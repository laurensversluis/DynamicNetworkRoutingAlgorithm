SET search_path TO public,topology;
DROP TABLE IF EXISTS test_topo_vertices_pgr CASCADE;
SELECT pgr_createTopology('test_topo', 0.001, 'geom', 'id', 'source', 'target','true');

DROP TABLE IF EXISTS tree;

CREATE TABLE tree AS (
	SELECT c.id1 AS id, c.cost::integer
	FROM pgr_drivingdistance('SELECT id, source, target, cost FROM "2044".test_topo', 1, 2000, FALSE, FALSE) AS c,
	network AS n

DROP TABLE IF EXISTS tree;	
CREATE TABLE tree AS (
SELECT 
	network.id, 
	min(nodes.cost)::integer, 
	network.geom 
FROM 
	network, 
	(SELECT id1, cost FROM pgr_drivingdistance('SELECT id, source, target, cost FROM network', 10888, 5000000000, FALSE, FALSE)) AS nodes 
	WHERE network.source = nodes.id1 OR network.target = nodes.id1 
	GROUP BY network.id);
	
---- WITH SOURCE/TARGET JOIN
WITH RECURSIVE route(id, source, target, cost) AS (
		SELECT 
			prev_edge.id AS id,
			prev_edge.source,
			prev_edge.target,
			prev_edge.cost
			-- ARRAY[prev_edge.id] AS edge_list			
		FROM tree AS prev_edge
		WHERE prev_edge.id = 240
	UNION ALL
		SELECT 
			next_edge.id AS id,
			next_edge.source,
			next_edge.target,
			next_edge.cost
			-- array_append(route.edge_list, next_edge.id) AS edge_list
		FROM 
			(SELECT next_edge.* 
			FROM tree AS next_edge, route
			WHERE 
				route.target = next_edge.target OR
				route.source = next_edge.source OR
				route.target = next_edge.source OR
				route.source = next_edge.target
			ORDER BY next_edge.cost ASC
			LIMIT 1) AS next_edge
	WHERE 
		next_edge.cost <> 0
)
SELECT id, cost FROM route LIMIT 1000;	


----- WITH GEOM JOIN
WITH RECURSIVE route(id, cost, geom) AS (
		SELECT 
			prev_edge.id AS id,
			prev_edge.cost,
			prev_edge.geom
		FROM tree AS prev_edge
		WHERE prev_edge.id = 240
	UNION ALL
		SELECT 
			next_edge.id AS id,
			next_edge.cost,
			next_edge.geom
		FROM 
			(SELECT next_edge.* 
			FROM tree AS next_edge, route
			WHERE 
				ST_Touches(route.geom, next_edge.geom)
			ORDER BY next_edge.cost ASC
			LIMIT 1) AS next_edge
	WHERE 
		next_edge.cost <> 0
)
SELECT id, cost,geom FROM route ORDER BY id LIMIT 1000;	