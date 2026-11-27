# M4: the drone plans its own search route

M3 explored the building and then flew a **hand-written 9-stop tour** that only fits this one building.
In M4 the drone works the tour out **from the map it just built**, so it works in any building:

1. **Rooms from the map.** The 2D slam_toolbox map is shrunk by 1 m so doorways close; each blob
   left is a room or corridor core, which is then grown back. This building gives 3 areas
   (south-west room, south-east room, north loop corridor).
2. **What still needs looking at.** While exploring, the node records the flight path. Every
   floor cell within 5 m with a clear line of sight from that path counts as already seen.
3. **Fewest stops.** Greedy set cover picks the stops that see the most unseen floor
   (line of sight, 5 m), until 95% is covered. Redundant stops are dropped, and two nearby
   stops are merged into one where possible. Stops keep 0.8 m from walls.
4. **Shortest order (TSP).** Real flying distances between stops come from Dijkstra on the map
   (0.5 m wall clearance). The order start -> every stop once -> home is solved exactly
   (Held-Karp dynamic programming, up to 12 stops; 2-opt above that).
5. **Fly it.** The stops are sent to Nav2 one by one (2 s hover at each so the victim detector
   gets a few looks). RViz shows the route (blue line) and numbered stops (orange, green when reached).

Offline result on the M3 map (`route_preview_m3_map.png`): **4 stops, 49 m**, against the M3
fixed tour's **9 stops, 65 m** from the same start, so the search leg is about 24% shorter with
5 fewer stop-and-hover points.

| file | what |
|---|---|
| `drone_search/drone_search/route_core.py` | planner maths (numpy only, unit-tested) |
| `drone_search/drone_search/search_route_node.py` | ROS 2 node: records path, plans on `/search_route/start`, flies with Nav2 |
| `drone_search/config/search_route.yaml` | parameters (see radius, cover goal, clearances) |
| `drone_search/test/test_route_core.py` | 5 offline tests incl. the real M3 map |
| `demo/run_m4_demo.bat` | one-click demo (`run_m4_demo.bat gui` for recordings) |

Outputs (in `media\m4`): `search_route.txt` + `search_route.png` (areas, stops, route),
`m4_route.log`, `m4_times.txt` (explore / route / total seconds), `victims.csv`, 2D + 3D maps.

**Gazebo run (Nov 27, 2026):** exploration already saw 91% of the floor, so the drone planned just 2 stops (one per south room): 35.3 m vs 59.9 m for the old tour from the same start (41% shorter). Route flown in 116 s, all 4 victims found (0.05-0.18 m error), no false pins. Mission 250 s in total.
