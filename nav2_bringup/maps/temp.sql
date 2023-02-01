SELECT
n.id, e.id
FROM edges as e
JOIN nodes as n ON n.geometry IN e.geometry
