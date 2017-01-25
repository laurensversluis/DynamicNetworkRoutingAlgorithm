-- Build required topology 
SET search_path TO "2044", public, topology;
DROP TABLE IF EXISTS network_topo_vertices_pgr CASCADE;
SELECT pgr_createTopology('netwerk_topo', 0.001, 'geom', 'id', 'source', 'target','true');

-- Build processed table
DROP TABLE IF EXISTS network_processed CASCADE;
CREATE TEMP TABLE network_processed AS (
	SELECT id, geom 
	FROM netwerk_topo);

	-- Create column for choice based on used radius
ALTER TABLE network_processed
ADD COLUMN choice_n integer;

-- Function that insert choice information into processed table based on specific destination segment	
DO $$
DECLARE r record;

BEGIN

SET search_path TO "2044", public, topology;

-- Create temporary tree 
DROP TABLE IF EXISTS network_tree_1;
CREATE TABLE network_tree_1 AS (
		SELECT 
			netwerk_topo.id AS id, 
			min(nodes.cost)::integer AS cost, 
			netwerk_topo.geom AS geom 
		FROM 
			netwerk_topo, 
			(SELECT 
				id1, 
				cost 
			FROM pgr_drivingdistance('SELECT id, source, target, cost FROM netwerk_topo', 1, 5000, FALSE, FALSE)) AS nodes 
		WHERE netwerk_topo.source = nodes.id1 OR netwerk_topo.target = nodes.id1 
		GROUP BY netwerk_topo.id);
		
ALTER TABLE network_tree_1
ADD CONSTRAINT network_tree_1_pk  PRIMARY KEY (id);
CREATE INDEX tree_btree ON network_tree_1 USING btree(cost);
CREATE INDEX tree_gist ON network_tree_1 USING gist(geom);

-- Create cursor based on tree originating from specified destination segment	
FOR r IN (SELECT id FROM netwerk_topo WHERE id < 100)
		
LOOP		
	----- Routing algorithm
UPDATE network_processed AS p
	SET choice_n = choice_n + 1
	FROM (	
		WITH RECURSIVE route(id, geom) AS (
					SELECT 
						prev_edge.id AS id,
						prev_edge.geom
					FROM network_tree_1 AS prev_edge
					WHERE prev_edge.id = r.id
					UNION ALL
					SELECT 
						next_edge.id AS id,
						next_edge.geom
					FROM 
						(SELECT next_edge.* 
						FROM network_tree_1 AS next_edge
						INNER JOIN route
						ON ST_Touches(next_edge.geom, route.geom)
						ORDER BY next_edge.cost ASC
						FETCH FIRST 1 ROWS ONLY) AS next_edge
					WHERE 
						next_edge.cost <> 0
			)
			-- Increment choice column with route
			SELECT id FROM route) AS route
	WHERE p.id = route.id;
	
END LOOP;
END$$;