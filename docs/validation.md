# FAR Planner validation record

This companion to the [algorithm guide](algorithm.md) records regression cases, implementation findings, SDK/source revisions, measurements, and remaining integration failures on the refactor branches. Historical counts and scoped experiments are retained alongside their methods.

## Current verification summary

| Check | Current evidence |
| --- | --- |
| Builds and dependency resolution | All five SDKs; current official rosdep rules resolve all manifests |
| ROS 2 core | 121 behavioral cases, 127 reported checks on Humble/Jazzy/Lyrical |
| ROS 2 graph utilities | 12 decoder and 16 boundary checks on each SDK |
| ROS 1 workspace | 276 reported checks on both Melodic and Noetic |
| Native sanitizers | 113 C++ cases, three planner-startup cases, five boundary process cases; leak detection enabled |
| Rendered controls and stock layouts | Plugin interactions on all SDKs; 15 stock-layout cases with the fixture conditions below |
| Recording replay | Three goals on all five SDKs; separate sanitized replay passes |
| Open integration case | Barrier introduced after initial mapping: four original 110-second arrival waits fail; an additional run also fails after a 300-second diagnostic wait |

These results apply to the source revisions and workloads documented below. The open barrier case keeps integration validation incomplete.

## Initialization and scheduling checks

`Polygon` and `CTNode` represent local topology. `NavNode` stores persistent geometry, multiple adjacency lists, vote histories, coverage flags, and search state. `FARUtil`, `DynamicGraph`, `ContourGraph`, and `MapHandler` also hold substantial static state. Containers often copy shared pointers, so a copied node vector is not an immutable graph snapshot.

On ROS 2, graph-update and planning wall timers use `main_run_freq`, and `main()` calls `rclcpp::spin(node)`. On ROS 1, `ros::spin()` services callbacks continuously; graph updates and planning use separate ROS timers at `main_run_freq`, while a 50 ms wall timer retries queued observations even when simulated time is paused. Both entry points serialize planner callbacks. A TF listener may use its own thread. Introducing parallel planner callbacks requires a separate ownership and synchronization design.

Module configuration is loaded and validated before each module's `Init()` derives grid dimensions, angle thresholds, and other processing state. `DynamicGraph::Init()` supplies the terrain planner's parameters. Node factories fill IDs and flags before publishing nodes to their owning containers; contour construction links non-pillar vertices into closed rings before surface analysis. Preserve these ordering contracts when moving initialization or callbacks. The static review found constructor-initialization warnings for these paths, but no confirmed read before initialization in the current entry points.

On ROS 2, shutdown during `Init()` is normal cancellation: the entry point checks the context before creating an executor and does not report a startup failure after a signal has shut it down. The ROS 2 `test/test_startup_shutdown.py` sends SIGINT and SIGTERM during initialization and separately verifies that a zero graph frequency still fails with a diagnostic. Both signal cases failed before the fix. At that stage, Humble, Jazzy, and Lyrical each pass 118 behavioral cases (124 reported checks including CTest wrappers); all three startup cases also pass under ASan/UBSan with leak detection.

Boundary startup cancellation also skips graph generation, preserving an existing output file. A FIFO-controlled trajectory read verifies SIGINT/SIGTERM on ROS 2 and SIGINT on ROS 1; an empty trajectory still exits with an input error. ROS 1 checks `isShuttingDown()` while keeping its owning `NodeHandle` alive through error classification. These cases pass across all five SDKs. ROS 2 graph-decoder construction is inside its cancellation handler so a stopped context cannot escape through executor creation.

## Waypoint projection checks

For a convex waypoint with no obstacles inside its local crop, viewpoint extension now returns the computed projected point instead of silently returning the unextended navigation point. The ROS 1 helper also uses its supplied node argument consistently. A focused regression covers this branch; it does not measure navigation performance.

## TF and timestamp contract

A measurement in frame `sensor` at time `t` must use `T_world_sensor(t)`, with target frame first and source frame second in the lookup. A nonzero input timestamp must not silently become a latest-transform query. TF time zero explicitly requests the latest available transform; a bounded wait can accommodate delayed TF delivery. See the [official tf2 time tutorial](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Cpp.rst). A [TF message filter](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Tutorials/Intermediate/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter.rst) can queue stamped inputs until their transforms are available.

The timestamped-transform stage fixes the following confirmed defects:

- Cloud and odometry lookups use the original ROS measurement timestamp, including nanoseconds before PCL rounds timestamps to microseconds. Identity-frame clouds also receive that timestamp in PCL metadata, so both transform paths report the same measurement time. Stamped measurements never request the latest pose.
- On ROS 1, the stamped TF quaternion and translation now feed `pcl::transformPointCloud` directly, preserving the float quaternion/translation conversion used by the previous wrapper. This removes the `pcl_ros` dependency. Existing timestamp, rotation, intensity, metadata, failure-atomicity, and delayed-TF tests cover the replacement.
- `ProcessCloud` builds a candidate and commits it only after a successful transform. Its callers return on failure before changing map state. Goal transform failures also propagate and prevent goal replacement.
- Empty frame IDs are rejected; leading slashes are normalized consistently. Zero-stamped sensor clouds and odometry are rejected. Zero remains an explicit latest-transform request for operator goals.
- RViz goal clicks use the current fixed frame for their XY coordinates. The tool transforms its latest stamped odometry pose into that frame before copying the robot height; if the transform is unavailable, it does not publish the goal. Before any odometry arrives, the goal height defaults to zero. Each click publishes one goal and then the resume command, avoiding a duplicate planner goal reset.
- TF queries do not block the planner executor. Bounded odometry, scan, local-terrain, and terrain queues wait for their stamped TF and, where needed, matching odometry without silently using the latest transform.
- Before PCL conversion, PointCloud2 inputs require a complete native-endian binary layout with distinct, nonoverlapping float32 XYZ fields. Terrain and local-terrain clouds also require a distinct float32 intensity field for free/obstacle classification, and points with nonfinite intensity are removed; scans may omit intensity. Malformed inputs are rejected before entering the TF queues. A zero-width, zero-row-step local cloud with no data remains an explicit clear measurement.

The default launch supplies raw `/registered_scan` returns to `/scan_cloud` for ray tracing, and height-cost `/terrain_map` points to `/terrain_local_cloud` for local occupancy. ROS 2 previously swapped these streams. In the indoor recording, all raw intensities were zero, so 4,640 elevated returns were classified as free when used as local terrain. The corrected wiring matches ROS 1. The live ROS 2 `test/test_terrain_input_topics.py` launches the installed entry point, supplies raw reflectivity and terrain height costs separately, and verifies an occupied wall cell and its subsequent free-terrain replacement.

The cross-stream timing stage adds the following policies:

- Each odometry, scan, local-terrain, and terrain stream must advance strictly in timestamp order. Rejected observations do not advance the accepted stamp. Odometry also requires finite coordinates and a nonzero finite quaternion; valid samples wait in order for delayed stamped TF.
- Scans require exact or interpolated world-frame odometry at their timestamp. History retains at most 2,000 samples and 10 seconds; both interpolation samples must be within `sensor_time_tolerance`, with no extrapolation. Scans can wait briefly for the later sample. Rays start at the timestamped sensor-frame TF origin when available, or at interpolated odometry for world-frame scans.
- Terrain and local-terrain clouds require exact or interpolated odometry at their header stamp. Each interpolation sample must be within `sensor_time_tolerance` of that stamp; no pose is extrapolated. A cloud already more than the tolerance behind the latest accepted odometry is rejected on arrival. Once queued, newer odometry does not invalidate a cloud with a valid pose still in history. Terrain and local-terrain queues hold at most two clouds each and expire entries by steady time while waiting for later odometry or stamped TF. Terrain map cropping and height analysis use the measurement-time pose.
- Dynamic-obstacle clearing uses the closest processed scan within `sensor_time_tolerance` of the terrain stamp and restores that scan’s measurement-time sensor origin before tracing rays. Equal-distance ties prefer the earlier scan. In dynamic mode, terrain waits up to 250 ms from its arrival when no compatible scan has been processed or one is queued for odometry or TF. This also lets a scan arriving after terrain contribute to the same update. A second queued terrain releases the older one immediately, so high-rate terrain does not wait itself out of the two-entry queue. The normal terrain queue expiry still applies. Up to 32 processed scans are retained; scans too old for later terrain updates are pruned. If none matches after the wait, the map update proceeds without scan-based clearing. Existing dynamic exclusions continue to decay.
- Dynamic obstacle exclusions default to **15 seconds** (`util/dynamic_obs_dacay_time`, or `Util/dynamic_obs_dacay_time` on ROS 1). Their lifetime must exceed upstream terrain-point retention, periodic pruning delay, and delivery/scheduling lag. The supplied terrain pipeline retains points for 10 seconds and prunes voxels periodically; the moving-gate recording still published obsolete points 11.5 seconds after the last raw return. The previous 10-second exclusion could expire first and let those points back into the persistent map. Fifteen seconds provides margin for that pipeline; configure a longer interval for longer upstream retention. Exclusions remain temporary, so increasing this window also delays global-map acceptance at previously cleared positions. A terrain cloud's new header stamp does not imply that every retained point was just observed.
- `sensor_time_tolerance` defaults to **0.2 seconds** and must be in `(0, 10]`. Configure it in the planner parameters (`/far_planner/sensor_time_tolerance` on ROS 1) to match the upstream publication rate and motion/error budget. A larger value permits greater temporal mismatch; it does not improve synchronization.
- All inputs must use the planner's ROS clock. Zero sensor stamps and stamps ahead of that clock by more than the tolerance are rejected. A backward clock jump clears the ROS 1 TF listener immediately, along with sensor history, map, graph, and goal state; otherwise a new-epoch sensor could use an old cached TF before the listener receives another TF message. The planner publishes a stop waypoint and waits for fresh odometry/terrain. Explicit graph reset also clears cached observations. The TF provider must supply transforms for the new epoch.

`test/test_frame_transforms.cpp` covers moving transforms, interpolation, nanosecond precision, unavailable/past/future TF, zero stamps, frame normalization, and unchanged inputs on failure. `test/test_sensor_timing.cpp` covers bounded odometry and scan histories, nearest-scan selection, ordered/expiring queues, pending-scan matching, interpolation limits, skew, and clock-rewind detection. `test/test_local_terrain_callback.cpp` checks measurement-time odometry matching, stale local-cloud rejection, waiting for later odometry, and an empty clear without a TF. The ROS 2 `test/test_terrain_tf.py` exercises timestamped cloud conversion, atomic rejection, delayed TF for odometry, scans, and terrain after newer odometry, terrain-time cropping, invalid/out-of-order odometry, duplicate terrain, skew rejection, older-scan selection after a newer scan, queued-scan TF pairing, scan arrival after terrain, and simulated-time reset. The ROS 1 `test/test_scan_tf_ros1.py` exercises delayed odometry and TF plus scan arrival after terrain in the running node; `test/test_tf_epoch_ros1.py` checks that a clock rewind invalidates old TF before a new TF message arrives.

Remaining limits are explicit:

- Odometry, scans, local-terrain, and terrain retry only within their bounded queue lifetimes; TF arriving after expiry cannot recover that observation. The 0.2-second tolerance is a configurable policy, not a verified setting for every robot.
- World-frame scan clouds have no separate sensor frame, so their rays still start at the odometry reference point. Sensor-frame scans use their timestamped TF origin. A rigid cloud transform does not deskew a spinning scan; upstream per-point timing and motion compensation are required.
- Local terrain rejects clouds already stale on arrival and waits for exact or nearby bracketing odometry at its own stamp before replacing the local obstacle view. Its obstacle view then persists until a newer valid cloud or explicit empty clear arrives. A scan arriving after the 250 ms terrain wait cannot retroactively affect that update. This wait adds up to 250 ms of terrain latency when no compatible scan arrives. Aggregated terrain maps still need a documented upstream frame/time convention; they cannot be made simultaneous with a scan by assigning one header stamp. Graph publication time is not an aggregate observation time.
- These tests do not establish end-to-end navigation correctness. Validate synchronized bag replay and motion on the actual robot before adopting the refactor branches.

## Search correctness stage

Both graph traversals now use their reset reachability flags as settled-node markers, removing the duplicate closed-ID hash set. A regression repeats queries after bridge removal/reconnection, source changes, and coverage changes, checking reachability, distances, and parent clearing. Searches require the current graph to include all connected nodes. Idle planning now refreshes that snapshot before each search: an import can otherwise add nodes between graph-update and planning ticks. The original hash-set implementation reproduced a stale 2 m imported-node distance after source motion required 3 m; the callback regression passes after the refresh independently of the flag optimization.

In seven alternating Release process runs per variant, each batch performs 500 paired general/covered searches with graph setup and warmup excluded. Timing binaries use standard allocation functions; separate binaries count ordinary `operator new` calls. Final costs, parents, and reachability match in all four synthetic workloads. The actual before/after cores also match across 167,666 node/mode results from all 328 frozen complete graphs; general costs match the captured single-layer search exactly. That comparison uses no terminal goal, so covered-goal restrictions remain checked separately by the core regressions.

| Search workload | Nodes | Before / after median ms | Ordinary C++ allocation calls, before / after |
| --- | ---: | ---: | ---: |
| Small grid | 64 | 5.056 / 3.432 | 70,000 / 4,000 |
| Dense grid | 375 | 85.883 / 58.478 | 386,500 / 8,500 |
| Disconnected, partial coverage | 375 | 11.711 / 7.327 | 140,000 / 4,500 |
| Large dense grid | 1,024 | 238.333 / 175.382 | 1,037,500 / 10,000 |

This measures the search method and attributed ordinary `operator new` calls, not all process allocations or whole-planner throughput. Search-only median time falls 26.4–37.4% in these workloads; the idle-callback snapshot copy is outside the benchmark. Final suites pass on all five SDKs: 127 reported ROS 2 planner checks per SDK, 276 ROS 1 workspace checks per SDK, and 113 native C++ sanitizer cases. The sanitized 50.643-second replay on `c67fd9b` reaches all three goals, with 113 consistent graphs, 103 finite waypoints, and normal exit without sanitizer findings.

Both graph queues now break exactly equal-cost ties by ascending node ID. Previously, the default pair comparison used `shared_ptr` address order: the same symmetric diamond graph selected a different route when its two branch nodes exchanged allocation addresses, despite unchanged IDs, geometry, and costs. The regression fails before the change in both traversals and passes afterward for reversed addresses and neighbor order. An unequal-cost control still selects the cheaper branch when its ID is higher. This improves reproducibility for a fixed graph; it does not make sensor-driven graph construction deterministic or establish a navigation fix. At that tie-ordering stage, suites report 125 checks on each ROS 2 SDK, 272 on each ROS 1 SDK, and 111 native C++ sanitizer cases, all passing. The sanitized 50.643-second replay on `7775349` reaches all three goals, with 113 consistent graphs, 103 finite waypoints, and normal exit without sanitizer findings.

The two visibility-graph traversals and terrain A* now push an immutable cost with each queue entry. Every distance improvement adds a new entry; an older entry is skipped when popped. This removes both mutable comparator keys and the separate open-set bookkeeping. The general graph traversal retains its slope-adjusted goal cost, while the covered traversal retains its coverage and free-goal restrictions. Terrain A* still uses Euclidean step costs and a Euclidean heuristic. Each query now clears search scores and parent links only for grid cells touched by the previous query; new nodes start with infinite scores. A failed query followed by successful queries has regression coverage. The reduced reset work is not yet a measured planner throughput gain.

A five-node geometric graph demonstrated the old failure: the cost to node 1 settled at 45 instead of the 44.89018 route through node 3. `test/test_search.cpp` now checks both graph traversals against an independent quadratic shortest-path reference over 40 deterministic graphs with covered/uncovered nodes. It also checks an obstacle-grid terrain route. These tests establish search behavior for the covered cases, not the whole navigation stack.

Goal snapping now marks only the closest navigation point after scanning all candidates. Previously, each improving candidate retained `is_goal`, making a discarded point appear static after later goal resets. The new regression fails on the old behavior and checks selection plus replacement.

New temporary goals in single-layer mode now choose terrain height by physical vertical distance to the observed mean height in each map cell of the same XY column. Voxel-step counts can select a farther surface: with ground at −0.7 m and 1.3 m, a 0.5 m query previously chose −0.7 m, even though 1.3 m is closer. An occupied query cell also no longer hides a closer surface in the adjacent cell. Cell means are accumulated in double precision. The current cell is considered first, followed by the first occupied cell below and above; a cell-face distance bound stops a direction that cannot improve the result. Exact ties retain the first candidate, including the lower surface when the query cell is empty. Missing or out-of-map observations leave the ground-height query unchanged and report no association.

The two original cases failed before the fix. The map regression also compares 960 query positions against every occupied cell's mean across 40 deterministic sparse columns, including empty columns, grid faces, out-of-range queries, and height offsets of ±1,000 m. A public `GraphPlanner::UpdateGoal` regression verifies the resulting height in single-layer mode and preserves the requested Z in multi-layer mode; these tests do not establish full multi-layer navigation performance.

Goal refinement receives the current odometry position explicitly. It previously dereferenced a pointer populated only by an earlier graph traversal, so a goal arriving between graph initialization and the first planning tick could crash, including after reset. The indoor simulator exposed this sequence; a focused regression checks refinement before any traversal and again after reset.

`PathToGoal` reports arrival within `g_planner/converge_distance` of either the active adjusted goal or its initialization anchor (`origin_goal_pos_`). For a new node, that anchor is the requested position; when reusing a nearby navigation node, it is already the snapped node's position. Free-grid and polygon refinement can move the active XY target; `goal_adjust_radius` sizes the free-grid search but does not bound the later polygon projection. An arrival status therefore does not guarantee that the original requested XYZ was reached. The garage validation below checks that separately.

A controlled production-core probe, without TF or sensor input, confirms that distinction: a request 0.75 m from an existing navigation node snaps to it under the 0.8 m threshold. Arrival is true with the robot 0.146 m from that anchor and 0.604 m from the request. ASan/UBSan with leak detection reports no findings. This characterizes the current contract; it does not reconstruct or establish the cause of the earlier garage miss.

When local terrain occupies the goal grid, free-space classification requires a cell marked free without an overlapping inflated obstacle. Previously, one overlapping free sample could admit the goal to the covered traversal even when no free cell existed; a nearby navigation node could also bypass that blocked terrain. The navigation-node fallback now applies only when the goal grid has no terrain cells. A route-level regression fails on the old behavior and checks blocked terrain, obstacle removal, and the no-terrain fallback.

In multi-layer mode, goal-grid projection now rejects obstacle points outside `goal.z ± kTolerZ`. Free points describe ground, so their height is compared after adding `vehicle_height`. This derived tolerance is `floor_height - 2 * voxel_dim`; its boundary remains inclusive, matching the node-layer rule. Out-of-band samples neither occupy nor free a goal-grid cell; the existing same-layer navigation-node fallback still applies when no relevant terrain remains. Single-layer mode retains its XY projection.

A `MapHandler` → `GraphPlanner` regression starts the map on a ramp and verifies that coarse vertical extraction can include lower-floor samples at an upper-floor goal. It checks rejection of that free evidence, exclusion of the lower obstacle, preservation of a real goal-height obstacle, tolerance boundaries, and the single-layer policy. The old projection fails the cross-height cases. This validates goal classification, not full multi-layer navigation.

Planner reset now clears the selected goal before dropping its pointer. Temporary goal removal also erases the node ID index entry, allowing the node to be released instead of accumulating across goal changes; lifecycle regressions failed before these fixes.

Terrain A* now rejects a diagonal step if either adjacent cardinal cell is occupied. A two-obstacle corner regression failed on the old path, and the updated obstacle-grid route has an independently checked shortest length of 6 m under the corner-safe rule.

Terrain search now stores closed membership on each existing grid node, eliminating the per-query hash table. Every closed node has already entered `searched_cells_`, so the next query resets its flag together with its score and parent; unsuccessful and out-of-range queries also start with that reset. The deterministic reference test compares reachability and shortest cost for 320 queries across 80 changing obstacle grids and alternating map centers, including same-cell queries and occupied endpoints.

The optional `terrain_search_benchmark` target links the actual planner core. From a built, sourced workspace, run `cmake --build build/far_planner --target terrain_search_benchmark -j2` and `build/far_planner/terrain_search_benchmark 200` (ROS 1: `catkin_make terrain_search_benchmark`, then `devel/lib/far_planner/terrain_search_benchmark 200`, with a running ROS master). It reports query batches on an 80² grid, including search reset and path-length accounting; map projection, grid construction, ROS startup, and four warmup queries are untimed. Twenty alternating Release runs of the baseline core (`b4e0e6f`) and this change on native Jazzy, pinned to CPU 2 of an i9-12900K, produced these median/p95 batch times for 200 queries:

| Scenario | Hash set median/p95 (ms) | Node flag median/p95 (ms) |
| --- | --- | --- |
| Nearby endpoint | 0.164 / 0.170 | 0.160 / 0.172 |
| Clear corridor | 2.553 / 2.589 | 2.256 / 2.308 |
| Detour around a wall | 43.217 / 43.551 | 33.243 / 33.490 |
| Blocked corridor | 40.653 / 41.312 | 31.334 / 34.514 |

Every batch matched in acceptance, total path length, and returned point count. Nearby-query timing changed little; these synthetic cases measure terrain queries rather than complete planner latency. The heap, heuristic, neighbor order, occupied-endpoint rule, and diagonal corner checks are unchanged.

## Graph lifecycle stage

`EraseNodeFromStack` now uses stable remove-and-erase for all matching pointers, including the first element and adjacent copies. Merged-node removal uses the iterator returned by `erase`, so removing the first or consecutive nodes does not decrement past `begin()`.

A full graph reset now clears reciprocal navigation edges, potential connections, trajectory links, search-parent links, and navigation-to-contour references before releasing graph storage. Full contour reset clears the reciprocal vertex links. `test/test_graph_lifecycle.cpp` verifies first/last/consecutive erasure and that linked navigation and contour vertices are actually destroyed after reset; both ownership tests failed before the fix.

Merged-node and temporary-goal deletion also clear both sides of terrain-feasibility vote histories through the node ID map. These histories can exist for rejected pairs with no visibility edge, so adjacency alone cannot find them. Histories between surviving nodes remain intact. Regressions cover terrain-only neighbors and 32 goal replacements; the previous implementation retained stale node IDs after each removal.

Normal contour replacement now clears reciprocal front/back links before dropping the old topology. Navigation nodes may still hold an old vertex until rematching later in the same serialized update, but that vertex no longer retains its old polygon neighbors. A lifecycle test confirms retired vertices are released after the last navigation reference is removed. Long-running heap profiling is still needed to rule out other ownership growth.

Planner destruction also resets its goal, navigation graph, and contour graph, breaking their ownership cycles without publishing a reset waypoint. A live indoor replay previously left about 0.96 MB in linked graph allocations at shutdown. The lifecycle regression checks that destruction releases linked navigation and contour nodes even without an explicit reset or full initialization.

The boundary extractor likewise breaks reciprocal node links before graph replacement and destruction. Normal generation and output-error teardown each leaked 16,072 bytes in 175 allocations before the fix; both process regressions now pass under ASan/UBSan with leak detection. The bundled 25-node graph is byte-identical before and after. The extractor is noncopyable to keep graph ownership unambiguous. Current boundary suites report 16 checks on each ROS 2 SDK; the decoder suite reports 12.

A reference audit removed two unused private contour helpers and five private declarations with no callers or implementations. The unused `AddConnect` helper was the only production writer of `CTNode::connect_nodes`; removing both reduces native 64-bit `sizeof(CTNode)` from 144 to 120 bytes. Contour topology uses the reciprocal `front`/`back` ring, and its lifecycle regression retains the weak-reference release assertions. Active height checks use `IsEdgeOverlapInHeight`; the discarded `IsOverlapRange` helper had an incorrect OR predicate but no runtime callers. This measures object storage, not whole-planner memory or throughput. At that cleanup stage, core/workspace suites report 124 checks on each ROS 2 SDK and 270 on each ROS 1 SDK, with no failures; the native 110-case ASan/UBSan suite also passes. On source `56b72f4`, the sanitized 50.643-second recording replay again reaches all three goals, with 113 consistent graphs, 102 finite waypoints, and normal exit without sanitizer findings.

A follow-up reference audit removes the unused `NavNode::node_type` field, its `NodeType` enum, and the unused `LiDARMODEL` constants. Native 64-bit `sizeof(NavNode)` changes from 816 to 808 bytes; `CTNode` remains 120 bytes. These measurements cover object storage only. The default-state test removes its assertion for the deleted field; the remaining state and ownership checks are retained. All five SDK suites pass again (125 reported ROS 2 planner checks per SDK, 272 ROS 1 workspace checks per SDK), as do all 111 native C++ sanitizer cases. The sanitized 50.643-second replay on `4ae38ea` reaches all three goals, with 113 consistent graphs, 101 finite waypoints, and normal exit without sanitizer findings.

Pillar classification now uses the closed polygon perimeter, including the last-to-first edge. A regression triangle has a 7 m open chain and a 12 m perimeter, crossing a 10 m pillar threshold. The pillar fallback also sets both surface directions to its sentinel value.

Default planner `Point3D` values initialize XYZ and intensity to zero. Polygon, contour-node, and navigation-node scalar fields also have explicit neutral defaults; their factories no longer clear newly empty containers or restate neutral flags. The boundary extractor and graph decoder use coordinate-only points with zero-initialized XYZ. Contour pixels construct a complete point directly, so their intensity is defined before PCL conversion. `Point3D::operator!=` is the complement of its tolerant equality, including at the tolerance boundary and for nonfinite coordinates. The unused `point_hash` and `point_comp` helpers were removed because exact float hashing was inconsistent with tolerant equality; remaining Boost hash users include their header directly.

## Trajectory sampling stage

Trajectory sampling runs on every enabled graph-extraction call, even when the contour matcher has no new vertices. The previous empty-contour return skipped both initial trajectory creation and a bridge point requested by `IsInterNavpointNecessary`. Sampling uses that existing distance/connectivity policy and its recorded free-odometry position. Extraction clears its output each time, and graph-update freezing skips extraction at the caller.

Terrain initialization accepts nonempty nearby free or obstacle observations after frame/time validation and map processing. The previous obstacle-only gate kept the planner idle in observed, obstacle-free space. Empty terrain alone still does not initialize planning.

Unit regressions cover initial sampling, a requested bridge position, and no repeated sample while stationary. Live ROS 2 tests cover free-only terrain and terrain with an obstacle below the contour threshold. They check accepted free observations, one stable trajectory node across graph ticks, and a direct goal waypoint. ROS 1 also checks free-only startup through graph and waypoint publication.

## Boundary input stage

The boundary extractor reads ASCII PLY through a bounded, checked parser. The vertex element must come first and declare scalar `x`, `y`, `z`, and `poly_index` properties; their column order may vary. It accepts at most one million vertices with finite coordinates and integer-valued polygon IDs, and rejects missing or duplicate properties and truncated rows before modifying the polygon stack. Parsed vertices feed polygon grouping directly, without a temporary PCL cloud. Segment intersection uses a local 2D point type, so the extractor has no OpenCV dependency. The trajectory reader requires finite X, Y, and Z values; extra trajectory fields remain allowed for the sample format. File streams close automatically, and the executable reports an input error without saving a partial graph. Parser and intersection tests cover malformed input and segment edge cases. Process smoke checks verify the sample still produces 25 nodes and malformed input exits without a graph.

Boundary configuration normalizes `world_frame`, requires a positive finite `visual_scale_ratio` and a nonnegative finite `height_tolz`, and resolves relative filenames against `folder_path` with or without a trailing slash. Direct execution defaults to the `far_planner/data` directory, matching the launch file; ROS 1 reads overrides from the node's private parameters. Absolute filenames remain unchanged. Invalid settings fail before reading inputs or writing a graph.

The extractor builds graph and free-position markers only while their topics have subscribers. Its one-second loop checks again for viewers that connect later. Completed graph markers move into the outgoing array instead of copying their point lists.

## Saved graph decoder stage

The decoder now parses the complete saved graph before publishing it. Each line requires the 15 scalar fields and three `|` separators; the four connection groups may be empty. IDs use the message's full `uint32` range, including the final token on a line without a trailing space. Invalid coordinates, duplicate IDs, missing fields, and references to absent nodes reject the file without publishing a partial graph. Parser tests read both bundled `.vgh` files and exercise malformed input.

The decoder and boundary extractor now share `visibility_graph_msg/graph_file_writer.h` for saved graphs. It writes float coordinates with `max_digits10` precision, stages output in the destination directory, checks write/flush/sync/close, and renames only on success. Existing file permissions survive replacement; new files honor the process umask, matching the previous save behavior. Decoder live tests exposed the old one-bit float round-trip error and an injected-write test verifies failure cleanup. The boundary sample still produces 25 nodes, with a formerly rounded Z coordinate now round-tripping exactly. Atomic rename does not claim durability across a whole-machine power loss.

Boundary visibility-edge generation now visits only `j < i`; for `n` vertices it checks `n(n−1)/2` unordered pairs instead of looping over `n²` positions and skipping half. Candidate order and graph output stay the same; whole-extractor latency has not been measured.

Incoming graph messages are built before replacing the last accepted graph. Replacing or destroying a decoder graph clears its strong connection links, including a graph loaded from a file. This prevents reciprocal edges from retaining old graph nodes on repeated updates. A successful file load becomes the decoder's current graph, so save and request operations use it; its robot ID is zero because the file format has no robot ID.

Incoming decoded graph messages now validate their world frame, finite node positions and surface directions, direction count and type, unique IDs, and every connection target before touching the persistent graph. This prevents a missing `surface_dirs[1]` access and avoids partial graph updates on malformed input. The callback also reads the shared message by reference instead of copying the full node and edge arrays.

Graph publishers set the graph header and each node header to the same world frame and publication timestamp. The stamp describes when the accumulated graph snapshot was published; its geometry can include older observations. Incoming graphs must declare the expected node count, and every node frame must match the graph frame. The receiver does not silently transform mismatched node coordinates.

The graph decoder now applies the same validator before replacing its displayed and saved graph. A bad frame, geometry, duplicate ID, size, or connection leaves the previous graph intact; an explicitly empty, correctly framed graph clears it. The shared validator lives in `visibility_graph_msg` so both consumers use the same rules. The decoder refreshes stored graph markers when a viewer connects and about every two seconds while viewers remain; it does no marker construction without a viewer.

Decoder startup strips leading `/` from `world_frame` and rejects an empty or whitespace-containing frame and a nonpositive or nonfinite `visual_scale_ratio`. Invalid settings exit with a diagnostic instead of silently rejecting every graph or publishing unusable markers.

## Graph exchange spatial index stage

`GraphMsger` now packs only searchable nodes into its KD-tree cloud and keeps a parallel pointer for each point. Search returns the exact node pointer, avoiding both empty/ghost slots and loss of integer ID precision through a float intensity field. PCL returns squared distances, so the matcher compares them with the squared linear radius. Tests cover skipped nodes, a large ID, radius semantics, empty rebuilds, and invalid queries.

Each valid graph import builds a local index from the current authoritative `DynamicGraph` nodes before matching and releases the index's node references when the callback returns. Repeated imports between planner ticks reuse previously imported nodes, and matching sees updated node positions. Nodes within one incoming message still match against the graph that existed before that import. Publication no longer copies the graph pointer vector or rebuilds an index that it does not query. Regressions cover repeated callbacks, node identity, all four connection types, movement after publication, and node release after graph reset.

Matching selects the nearest live node with the same navigation-point and boundary flags. A closer incompatible node no longer hides a compatible neighbor inside `dist_margin`. The usual single-nearest query returns immediately for a compatible candidate; otherwise, an unsorted radius query finds the nearest acceptable node while retaining the strict distance limit. Tests cover both flags, coincident node types, radius boundaries, and comparison with an independent linear search.

Graph serialization moves each completed node into the outgoing message instead of copying its connection vectors. The decoder iterates validated connection IDs in the incoming message directly and reserves its node and ID-index containers up front. This removes four temporary connection arrays and their per-node copies; planner throughput has not been measured from this change.

The graph decoder preserves all three coordinates of both surface directions when receiving and republishing a graph. The Z coordinate carries the `-1` pillar sentinel and may also contain a nonzero direction component. A live message round-trip test checks both cases.

Before encoding a graph snapshot, the planner selects eligible nodes with at least one visibility, polygon, contour, or trajectory edge. It then emits connections only when their target is in that selection. This preserves nodes connected only by non-visibility edges and keeps every edge list consistent when a neighbor is isolated, unfinished, excluded by type, or absent from the supplied graph. Graph-message regressions check both cases and receiver validation.

The graph wire format uses 32-bit node IDs and counts and a 16-bit robot ID. Startup rejects a robot ID outside 0–65535. Before publication, encoding rejects duplicate or unrepresentable selected node IDs and leaves the outgoing snapshot empty; the publisher skips that snapshot instead of silently narrowing IDs. Boundary tests cover the largest valid node ID, oversized IDs, duplicate IDs, and robot ID limits.

## Scan sensor origin stage

When a scan arrives in a sensor frame, the cloud transform also supplies the sensor's world-frame translation at that same scan timestamp. Scan ray tracing starts from that translation, accounting for the scanner's offset from the odometry body. A scan already in the world frame still uses the interpolated odometry position because its original sensor frame is unavailable. Failed TF leaves both the accepted cloud and ray origin unchanged. Moving-TF regression tests cover the returned origin, interpolation, identity fallback, failure, and rotation while preserving intensity and cloud metadata.

Scans that arrive before their later odometry sample now wait in a timestamp-ordered queue. When odometry advances, the planner interpolates the scan position from nearby samples, transforms the scan, then stores it with its stamped sensor origin for terrain-time ray tracing. The queue holds at most 32 scans and discards scans that have waited longer than twice the configured tolerance (at least one second), measured by steady time so delayed replay stamps remain valid; clock rewind and graph reset clear it. Duplicate and out-of-order scans are rejected. Unit tests cover queue ordering, bounds, expiry, and reset; a ROS 2 callback test verifies a scan received before its later odometry is eventually ray traced. If scan TF is missing, the head scan stays queued and later scans retain timestamp order. Both versions schedule nonblocking TF retries every 50 ms. Invalid scans are removed, and scans waiting beyond the steady-time limit expire with a diagnostic. The live ROS 1 rostest and ROS 2 callback test deliver TF after a scan and verify that processing resumes without another scan or odometry callback.

Scan ray tracing now checks for a zero-length ray without squaring integer grid offsets, which could overflow for a distant endpoint. All three dominant-axis loops share the same cell-marking and stop rule. `test/test_map_handler.cpp` checks their voxel outputs and a large offset that previously made the squared distance overflow. The loop still stops at the grid boundary or the first scan-occupied cell.

Terrain uses the same nonblocking retry schedule but keeps at most two clouds because terrain messages can be large. It waits for a bracketing odometry sample, uses that interpolated pose for map cropping and terrain-height analysis, and retries a missing stamped TF. A cloud already stale relative to odometry is rejected on arrival. Later odometry does not discard queued terrain that still has a valid measurement-time pose; queue expiry, history bounds, and strictly increasing cloud stamps limit how long it can wait. Terrain is processed in stamp order; clock rewind and graph reset clear the queue.

Odometry keeps at most 200 small messages while waiting for their stamped world-frame TF. Samples are processed in order, and each accepted pose releases scans and terrain waiting on that time. The queue expires samples after twice the configured tolerance (at least one steady-time second) and clears on clock rewind or graph reset; later samples can advance after an expired head. Both versions retry every 50 ms and when new odometry advances the queues.

Local terrain keeps at most two clouds and follows the same stamped-TF retry and expiry policy. It requires exact or nearby bracketing odometry at its own stamp, with stale clouds rejected on arrival; newer odometry alone does not discard a cloud awaiting TF. Once transformed, its obstacle/free cloud replaces the previous local-terrain view. Accepted local terrain remains available to graph updates until replaced or reset. Each replacement clears only cells occupied by the previous local cloud before projecting the new one; a correctly stamped empty cloud clears stale obstacles without needing a TF for its empty frame. The debug map is converted only while subscribed and cached until the occupancy revision or grid center changes; each publication still receives a current timestamp. A terrain-search regression checks that a cleared obstacle stops blocking the direct route and that recentering clears occupancy. Accepted stamped local clouds, including explicitly empty messages, advance a local revision; repeated graph ticks reuse the projected occupancy, while a new revision or grid recentering rebuilds it immediately before subsequent graph checks. The debug map remains available to subscribers on repeated ticks.

ROS 1 now services callbacks continuously with a single-threaded `ros::spin()`. Graph updates and planning retain separate timers at `main_run_freq`; pending sensor observations retry on a 50 ms wall timer, including while simulated time is paused. The incoming odometry queue holds at most 200 messages to absorb bursts during graph work. Previously, callbacks ran only at the default 2.5 Hz graph rate, and the five-message subscription discarded most 200 Hz odometry between iterations. A terrain observation delayed by 120 ms could then lack a nearby earlier pose despite satisfying the 0.2-second policy. `test/test_callback_cadence_ros1.py` exercises nominal 200 Hz odometry and 5 Hz delayed terrain at the default graph rate: the old code accepted zero terrain updates, while the fix preserves sensor processing between graph ticks. The current ROS 1 workspace results are listed under [SDK compatibility](#ros-1-sdk-compatibility); the recording replay below also fails before this fix and passes afterward.

## Invariants to preserve

- Every accepted geometric value used by the planner is in the configured world frame. Failed or unavailable transforms must not mutate accepted map, odometry, or goal state.
- Node IDs remain stable and resolve to live nodes; deleting a node removes its adjacency, vote, and lookup references.
- Undirected graph operations maintain reciprocal edges. Known-free and attemptable graph policies remain distinct.
- Search costs are nonnegative. Priority-queue keys must remain valid after a distance improvement; the parent chain must terminate at the start.
- Geometry thresholds, boundary restrictions, and momentum behavior are preserved unless a separately tested behavior change is intentional.
- Optimization must not remove defensive copies where an operation mutates the container being traversed.

## Step-by-step simplification and efficiency work

| Step | Change | Required evidence |
| --- | --- | --- |
| 1 — foundation | Branch-specific dependencies/builds, portable setup, MIT notices, plugin cleanup | Native Jazzy/Lyrical/Noetic builds; launch and plugin smoke checks |
| 2 — frame/time correctness | Timestamped TF queries and explicit failure propagation, then cross-stream timing policy | Moving transforms at distinct times; unavailable/past/future TF; zero stamps; simulated-time jumps; unchanged state on failure |
| 3 — search correctness | Audit mutable priority-queue keys in both graph traversals and terrain search; consolidate only identical mechanics | A graph where a queued node's best cost decreases; reference shortest-path comparisons; covered-space constraints |
| 4 — graph lifecycle | Replace invalid `erase(it--)` loops; audit ownership cycles, ID maps, and adjacency cleanup | Remove first/last/consecutive nodes; repeated resets; sanitizer checks and live-node accounting |
| 5 — file boundaries | Validate graph/boundary/trajectory input and fail without partial state updates | Truncated/malformed files, missing fields, large IDs, round trips, invalid coordinates |
| 6 — measured hot paths | Remove proven unnecessary copies/allocations; bound pair iteration; avoid avoidable rebuilds | Identical graph/path results; allocation counts; median/p95 stage timing over representative graph sizes |

Confirmed input and graph-index defects have regression coverage: trajectory Y/Z validation, bounded PLY parsing with a required vertex count, compact KD-tree points, exact node identity, radius units, finite point-cloud conversion, and empty KD-tree queries. Empty cloud updates no longer insert a synthetic point at the world origin. Obstacle and free-grid updates now filter only cells touched by the incoming cloud, removing two full-grid passes per update; each touched cell is still filtered once. Dynamic-obstacle removal likewise checks only cells named by its incoming cloud. A sparse visited-cell index drives occupancy visualization and map reset without traversing the full grid. `test/test_map_handler.cpp` verifies repeated updates, removals, occupancy enumeration, and reset. These are code-level complexity improvements, not end-to-end speed measurements. Cloud filter and crop helpers now swap their completed output buffers into place, removing an extra full-cloud copy on those paths. The map obstacle update likewise swaps its validated points into the caller cloud, and terrain-neighbor filtering erases rejected cell indices in place instead of copying the whole set; neither change has a measured end-to-end speedup. Cloud decay now compacts surviving points in place, retains their order, and uses one ROS-time reading for tagging and expiry instead of allocating an index list and filtering into a second cloud. Measured hot-path changes and live allocation profiling are recorded below.

Terrain-height analysis first searches from the robot for a ground seed, then floods connected ground. The two phases now use separate byte visited markers, so resetting the queue at the seed cannot hide adjacent cells queued during the search. A two-cell regression covers that failure. The contiguous markers also replace per-cell hash-set insertions; end-to-end speed has not been measured.

The near-node pair loop now visits only `j < i`, preserving each pair and its evaluation order. For `n` nearby nodes, index visits fall from `n²` to `n(n−1)/2`; connection checks already ran only for that lower triangle, so this saves skipped loop work rather than changing graph decisions.

The standalone benchmarks also support Melodic's CMake 3.10. From the repository root in a sourced ROS environment, configure them outside the source tree:

```sh
bench_source="$PWD/benchmarks"
mkdir -p /tmp/far-cloud-decay-build
cd /tmp/far-cloud-decay-build
cmake "$bench_source" -DCMAKE_BUILD_TYPE=Release
cmake --build . -- -j2
./cloud_decay
```

The build provides the standalone targets named below. All eight build and run on Melodic and Noetic. Use `-- -j2` to pass parallelism to Make on both SDKs; CMake 3.10 does not support the newer `-S`/`-B` command form. The optional project also filters PCL 1.8's empty compiler definition, as the planner build does.

Contour edge selection now tracks the two highest qualifying vote scores in one pass instead of allocating and sorting a score vector, then scanning that vector for each candidate. Equal scores retain the original strict-rank rule: every candidate tied at the second slot qualifies. A missing vote record logs an error and follows the edge-removal path instead of dereferencing `end()`. Randomized rank comparisons and a missing-vote regression cover both cases. Run `cmake --build /tmp/far-cloud-decay-build --target contour_vote_rank` and `/tmp/far-cloud-decay-build/contour_vote_rank` after configuring `benchmarks/`. On an i9-12900K with GCC 13.3, the isolated benchmark's median batch times were:

| Vote records | Calls per batch | Sorted rank (ms) | Top-two scan (ms) |
| ---: | ---: | ---: | ---: |
| 4 | 20,000 | 2.35 | 1.69 |
| 16 | 10,000 | 3.66 | 3.39 |
| 64 | 3,000 | 6.44 | 4.88 |
| 256 | 1,000 | 13.65 | 6.49 |

The benchmark checks identical selected IDs for 32 generated vote maps at each size and alternates implementation order across 20 timed samples. It excludes edge mutation, ROS callbacks, and contour extraction, so it does not establish whole-planner latency gains.

Repeated `FillContourConnect` calls on an existing pair now skip construction of two unused vote histories. `test/test_graph_lifecycle.cpp` verifies that repeated fills preserve both histories and links. Run `cmake --build /tmp/far-cloud-decay-build --target contour_vote_fill` and `/tmp/far-cloud-decay-build/contour_vote_fill` for the isolated benchmark. In a local Release run, 100,000 no-op calls made 400,000 allocations before the change and none after it; median batch time was 4.01 ms versus 0.41 ms over 20 alternating samples. This measures only that branch of vote insertion, not planner throughput.

Debug point clouds now convert to ROS messages only when their topics have subscribers. Map-grid marker construction and its center queries also wait for a subscriber; dynamic scan-ray debug output skips the full scan-grid traversal when unused. Terrain searches copy paths for visualization and build their marker only while subscribed, discarding queued paths if subscribers leave. Mapping, route search, and raycasting still run for navigation. The existing live terrain callback test subscribes to obstacle and ray debug clouds, covering those active publication paths. No end-to-end latency measurement is available for this change.

The scan handler uses the input scan directly when the free cloud is empty, avoiding an unnecessary full-cloud copy; it still copies before overlap removal when free points exist. Empty dynamic-obstacle extraction and an uninitialized debug-grid query now clear their output clouds. `test/test_map_handler.cpp` covers the empty-output cases and scan input preservation.

The scan grid retains its dense reset. Run `cmake --build /tmp/far-cloud-decay-build --target scan_grid_reset` and `/tmp/far-cloud-decay-build/scan_grid_reset` after configuring `benchmarks/`. In a local Release probe of 1.32 million cells, clearing 100,000 touched cells took about 31 µs in index order or 117 µs in random order, versus 18–21 µs for the dense reset. A 1,000-cell sparse reset was faster, but its benefit would also need to pay for tracking each write. The probe excludes that bookkeeping and the rest of the planner.

Goal reprojection now reuses a byte visited grid and FIFO across calls, clearing only visited cells after each search. It retains the left/up/right/down tie order and accepts only cells whose value is exactly `FREE_BIT`; the regression covers ties, mixed obstacle/free cells, repeated searches, and an invalid seed. The old search allocated a hash node per visited cell. After configuring the standalone build above, run `cmake --build /tmp/far-cloud-decay-build --target goal_grid_search`, then `/tmp/far-cloud-decay-build/goal_grid_search`. On an i9-12900K, local Release median batch times (old/new, 20 alternating samples) were 0.072/0.016 ms for 200 one-step searches on a 101² grid, 0.041/0.008 ms for 200 one-step searches on a 301² grid, and 15.43/3.07 ms for three full 301² searches with no free cell. The benchmark verifies matching result indices. It excludes polygon checks, terrain updates, and end-to-end planner timing; retained marker capacity grows to the largest grid size and queue capacity to the widest explored search.

Goal and local-terrain obstacle inflation now clip each square to grid bounds before visiting cells, preserving the prior X/Y visit order and skipping distant points. Grid and terrain regressions cover edge cells and extreme coordinates. After configuring the standalone build above, run `cmake --build /tmp/far-cloud-decay-build --target grid_inflation`, then `/tmp/far-cloud-decay-build/grid_inflation` to compare equal occupancy. On a local Release run with 50,000 synthetic in-grid centers and a 101² grid, median painting time changed from 1.45 to 0.40 ms at radius 1 and from 39.33 to 9.32 ms at radius 8 (20 alternating samples). Reset, point conversion, and whole-planner timing are excluded.

Contour filtering now reads each polygon in place, moves retained polygons after the hierarchy pass, and converts contour coordinates without copying the source polygon. This avoids copying every polygon during adjacency cleanup and copying the whole collection when a polygon is removed. Image inflation uses a fixed three-offset array, and two unused contour buffers are gone. `test/test_contour_detector.cpp` checks that a nested rectangular observation still produces the same four world-frame corners. Run `cmake --build /tmp/far-cloud-decay-build --target contour_compaction && /tmp/far-cloud-decay-build/contour_compaction` for the isolated collection benchmark: with 512 contours of 256 vertices and 171 removed, local Release median/p95 times on an i9-12900K were 0.249/0.345 ms for the previous copy path and 0.0029/0.0046 ms for in-place moves (40 alternating samples, identical ordered results). Input cloning, OpenCV extraction, geometry checks, and whole-planner latency are excluded.

Hierarchy descendant collection and collinear wall cleanup now use iteration, so deeply nested contours or long wall runs do not grow the C++ call stack. A zero-length wall edge is ignored before normalization. Hierarchy removal uses one byte per contour index instead of hash-set entries; retained contours keep their original order. The unused contour-copy helper is removed, and debug contour rounding reads its source by reference. The contour tests cover a 100,000-level synthetic hierarchy and wall run, premarked descendants, and stable filtering order. These stress tests do not represent typical image contours.

Contour free-odometry sampling now runs on each update while the robot is inside a current obstacle polygon. A failed search no longer permanently suppresses later searches when the contour changes. The two-update regression starts with no free sample, shrinks the polygon, and checks that the free reference moves outside it. Repeated unsuccessful searches can add sampling work while the robot remains enclosed.

Terrain-height downsampling now reads the original free cloud and writes a separate filtered cloud, avoiding a full prefilter copy while preserving the caller's input. Inflation walks neighboring grid cells directly in the same order, instead of constructing a temporary vector for every filtered point. Map and planner tests cover behavior; whole-planner latency has not been measured for these changes.

Native Jazzy allocation profiling identified repeated terrain-neighbor radius searches as a hot path: these searches flatten Z, so all vertical cells in an XY column request the same height bounds. `ObsNeighborCloudWithTerrain` now reuses those bounds within one call, preserving each cell's vertical acceptance test and the downward extension. The temporary cache is released after the call; new terrain and robot movement always query fresh evidence. An empty terrain index clears the candidate sets directly. A public map regression passes before and after the change and covers vertical acceptance, changed heights at an unchanged pose, extended cells, and clipped map edges.

Build the optional actual-filter benchmark with `cmake --build build/far_planner --target terrain_neighbors_benchmark`, then run `build/far_planner/terrain_neighbors_benchmark 100` in the sourced workspace. Noetic uses `catkin_make terrain_neighbors_benchmark` and `devel/lib/far_planner/terrain_neighbors_benchmark 100`. Each call checks both output sets against a linear reference over the emitted terrain samples; input preparation and verification are untimed. On an i9-12900K with GCC 13.3 and PCL 1.14, 20 alternating Release runs pinned to CPU 2 measured these 100-call batch times:

| Neighborhood | Cells / XY columns | Previous median / p95 (ms) | Reuse median / p95 (ms) |
| --- | ---: | ---: | ---: |
| One level | 169 / 169 | 27.524 / 28.280 | 28.803 / 29.667 |
| Five levels | 845 / 169 | 123.225 / 127.674 | 30.850 / 31.742 |
| Map corner, five levels | 245 / 49 | 33.504 / 34.662 | 7.854 / 8.094 |
| No terrain | 845 / 169 | 1.298 / 1.336 | 0.650 / 0.669 |

The normal five-level filter takes about 75% less time; the one-level control is about 5% slower because it pays cache overhead without reusing a query. Heaptrack attributed 1,244,239 allocations before and 519,305 after to the filter across these four benchmark cases, including setup and warmups (58% fewer). These are isolated filter measurements, not whole-planner latency or a long-running memory bound.

Potential costs are point-cloud conversion/downsampling and neighborhood reconstruction, image processing, roughly quadratic nearby-node pair checks, graph serialization/KD-tree rebuilds, and graph traversals. Complexity is a prioritization aid, not a measured bottleneck. Existing `/runtime` and `/planning_time` measurements cover different scopes and units; normalize benchmark reporting before comparing them. Record point count, contour/graph size, edge count, configuration, CPU, ROS version, and correctness results with each performance claim.

`Grid` computes an index's three coordinates when requested instead of storing them for every cell. The constructor rejects invalid dimensions, nonpositive resolutions, and cell counts beyond its `int` index range before allocation. World obstacle/free cloud grids now use sparse storage and allocate clouds only for cells receiving points. Empty reads do not create cells; reset and obstacle removal release empty entries. Per-update sets replace three dense marker arrays. Terrain-height and scan grids remain dense because their local dimensions are much smaller. Startup now validates timer frequency and geometry settings before dividing or converting to integer grid and image sizes. Invalid values produce a named startup error; oversized derived dimensions are rejected before allocation. Contour resize bookkeeping uses the same rounded output size as OpenCV, including fractional ratios. Grid position conversion now keeps extreme or nonfinite coordinates outside the index range without an overflowing integer cast. Ray traces use double precision for squared distances and stop immediately for invalid starts. Goal-grid searches require an in-range seed; scan inflation skips points too distant to affect its local grid, caps angular expansion by the grid dimensions, and visits only the part of each inflated box inside the grid. The boundary regression checks the same occupied cells; whole-planner latency has not been measured for this change.

Startup now rejects nonpositive graph and goal vote sizes, nonpositive graph filter and finalization windows, negative clear or momentum thresholds, and negative or nonfinite goal convergence distance. Each error names its parameter. An empty odometry edge-vote history is treated as having no latest positive vote instead of reading past the deque; the regression previously crashed.

Cloud overlap and new-obstacle extraction now tag a combined working cloud before voxel filtering. Source and reference intensities stay unchanged; `RemoveOverlapCloud` no longer makes a second copy of the reference when its legacy copy flag is true. New obstacle, dynamic obstacle, scan-grid, map-cell, terrain-height, and terrain debug outputs use PCL's cloud append API so `width` and `height` match their points. Finite-point filtering and height cropping also resize the cloud itself after compaction, keeping those dimensions consistent. `test/test_overlap_cloud.cpp` and the cloud/map/scan tests check results, input preservation, and cloud dimensions. Planner throughput from this change has not been measured.

An empty neighborhood free cloud now clears the previous terrain-height output, traversability flags, extended terrain-neighbor indices, and height KD-tree. Map reset clears the same state; planner reset also clears its cached sensor and debug clouds and releases the new-obstacle KD-tree index. Reset invalidates the old map origin, so the next accepted odometry recenters the empty grid before new clouds enter it. Until then, map getters clear their output containers instead of returning stale caller data. Map regressions verify old heights disappear and a replay starting far from the prior origin can populate its new neighborhood.

`ClusterFilterCloud` now releases its temporary FLANN index after replacing the point cloud; a subsequent empty input also releases any retained index. Its regression checks the filtered point order and that searches cannot use an index built for the old point set.

In a four-second Noetic desktop-container smoke test, the planner completed initialization with its code-default 5 km map span and reached 101,620 KB peak RSS before receiving sensor data. This is an empty-map startup observation, not a bound on memory after mapping.

The standalone build above provides the isolated retention-stage benchmark at `/tmp/far-cloud-decay-build/cloud_decay`. `benchmarks/COLCON_IGNORE` keeps this optional build out of normal workspace builds. On an i9-12900K with GCC 13.3 and PCL 1.14, half the points expired; runs alternated implementation order. Median/p95 milliseconds were:

| Points | `ExtractIndices` | In-place compaction | Runs |
| ---: | ---: | ---: | ---: |
| 100,000 | 1.02 / 1.20 | 0.075 / 0.100 | 30 |
| 1,000,000 | 16.62 / 17.70 | 2.24 / 2.47 | 12 |

`kNavClearDist` is derived as `robot_dim / 2 + voxel_dim`. The graph connection angle and direction filter margin follow `util/accept_max_align_angle` and `util/angle_noise` after degree-to-radian conversion (ROS 1 uses `Util/` and `Graph/` prefixes). The former `nav_clear_dist`, `graph/connect_angle_thred`, and `graph/dirs_filter_margin` parameter reads were overwritten before use and have been removed. The `g_planner/free_counter_thred` setting was also unused and has been removed from both shipped configurations and `GraphPlannerParams` (ROS 1 uses `GPlanner/`).

The optional `grid_position` target in the same benchmark build compares five million in-range conversions with the previous unchecked formula: `cmake --build /tmp/far-cloud-decay-build --target grid_position` and `/tmp/far-cloud-decay-build/grid_position`. A local Release run measured about 6.6 ms for the old formula and 11.4 ms for the checked conversion. This synthetic check excludes TF, PCL, and map updates; it measures the cost of the new safety checks, not planner throughput.

The benchmark measures only retention, excluding cloud copies used to prepare the input, ROS callbacks, and planning. Representative planner stage latency remains unmeasured.

## Indoor navigation and replay validation

A native Jazzy Release run used the shipped default planner parameters and the upstream [Jazzy simulator snapshot](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/tree/8313dfed10533787582be8a6044483fe3299622f), its supplied indoor environment, terrain analysis, local planner, and vehicle simulator. Gazebo ran with headless rendering; RViz and joystick nodes were omitted. Sequential goals in `map` were sent after graph initialization, with a one-second dwell after each reported arrival:

| Goal XY (m) | Travel time (s) | Final XY error (m) |
| --- | ---: | ---: |
| (4, 14) | 9.19 | 0.029 |
| (10, −15) | 18.20 | 0.049 |
| (0, 0) | 13.40 | 0.041 |

The recorded trajectory covered 72.6 m. Odometry bracketed 221 of 222 messages in each scan/terrain stream, with a maximum bracket gap of 11.32 ms. The first cloud in each stream preceded the first recorded odometry sample by 5.9 ms and therefore had no earlier recorded pose for interpolation. The nearest sampled wall point was 0.557 m from the vehicle center; this point-cloud comparison is not a contact-safety proof.

The 50.6-second input recording was replayed with ROS simulated time under AddressSanitizer and UndefinedBehaviorSanitizer, with leak detection enabled. The harness held the first operator goal until a graph was published; sensor and TF timestamps were unchanged. All three goals reported arrival, 113 graph snapshots passed frame/ID/connection checks, and 102 waypoints were finite and in `map`. The planner exited normally without sanitizer findings after the shutdown fix. The separate C++ sanitizer suite passed 101 tests. The build used `-fsanitize=address,undefined -fno-omit-frame-pointer -DEIGEN_MALLOC_ALREADY_ALIGNED=1` to match the system PCL/Eigen allocation alignment.

After correcting the scan/terrain wiring, the installed Jazzy launch ran with `is_static_env=false` for five cycles of the same route: all 15 goals reported arrival, with a maximum final XY error of 0.133 m. During 222 seconds of observation it published 1,074 nonempty scan-ray debug clouds and 538 graph snapshots. After the first 60 seconds, sampled planner RSS stayed between 122.1 and 129.5 MiB; the published graph peaked at 91 nodes and ended at 80. This short repeat run exercises dynamic-mode processing in a static world; it does not establish moving-obstacle performance or long-duration memory stability.

The indoor runs above cover one environment on native Jazzy; the garage case below adds a ramp route. Broader moving-obstacle cases, additional multi-layer routes, stock RViz layout behavior, other distributions' simulator runs, and representative robot recordings remain to be checked. Simulator assets, recordings, and diagnostic output stay outside the repository.

## Rendered RViz interaction regression

The optional [RViz interaction test](../tests/rviz_interaction/README.md) loads the installed Goalpoint and TeleopPanel plugins into RViz's real render panel and drives Qt mouse events. Its standalone CMake project is excluded from normal colcon builds. The runner owns a temporary X display and an isolated ROS environment, redirects ROS/Qt state to its artifact directory, and stops its child processes on failure or timeout. ROS 1 uses a separate local master; ROS 2 uses a dedicated localhost domain.

The same test passes on Humble, Jazzy, Lyrical/Qt6, Noetic/Qt5, and Melodic/Qt5 with software OpenGL. Each run checks six goal/resume pairs on ROS 1 and seven on ROS 2, five file dialogs (including cancellation and an existing `.vgh` suffix), four toggle messages, graph reset, panel resume, joystick release/idle behavior, and a one-pixel widget. Missing TF and invalid goal topics reject the goal and recover after valid input. A moving-TF case produces a 2.25 m goal using odometry-time evidence; latest-TF lookup would incorrectly produce 6.25 m. All goal coordinates and joystick axes must be finite.

These checks exercise a rendered RViz test window, rather than the stock application layout or hardware graphics. File dialogs verify published paths, without starting the graph decoder or writing a graph. They complement the planner replay and simulator runs. JSON results, ROS logs, and screenshots remain outside the repository. The runner also rejects missing/invalid final reports and nonzero exits; a timeout check verifies that a child ignoring SIGTERM is killed along with its process group.

TeleopPanel uses typed member-pointer connections for buttons, binary checkboxes, the joystick, and its output timer, giving [compile-time signal/slot checks](https://doc.qt.io/qt-6/signalsandslots-syntaxes.html). The checkboxes use `toggled(bool)` for their Bool commands; the existing integer slots accept that value while retaining the plugin's Qt meta-object interface. The rendered regression verifies the resulting publications on all five supported ROS distributions.

On ROS 2, the panel now shares RViz's clock after initialization, as the Goalpoint tool already does. Its constructor retains a node-clock fallback for use before RViz supplies a context. A regression switches the running RViz context to paused simulation time at 200 seconds and checks goal, resume, and manual joystick stamps. The previous panel failed this check because its independent publisher node kept system time. The ROS 1 panel uses the global ROS clock.

The stock RViz application also loads the shipped default, Matterport, and boundary layouts on all five SDKs under an isolated Xvfb display with software OpenGL. All 15 cases stay running, exit normally on SIGINT, and report no ROS error/fatal or plugin-load errors. Representative default-layout screenshots show the planner control panels and Goalpoint toolbar; no sensor publishers were supplied. The fixture initializes Lyrical's private preferences file and disables the ROS 1 SDK's modal lifecycle notice. Initial attempts without those conditions are retained separately; they do not establish a project plugin defect. Hardware graphics remain unvalidated.

## Dependency resolution

The ROS 2 OpenCV manifest key is `libopencv-dev`, matching ROS 1. The former `opencv4` key is absent from the current rosdep rules and made the README dependency command return an error on Humble, Jazzy, and Lyrical even with OpenCV installed. After the correction, `rosdep check --from-paths src --ignore-src` passes on all five SDKs using freshly downloaded official rules in a private cache. This check installs or upgrades no packages.

## ROS 1 SDK compatibility

Release builds and fresh full-workspace test runs pass in the official `osrf/ros:melodic-desktop-full` and `osrf/ros:noetic-desktop-full` images, with no manually started ROS master. Each reports 276 checks, zero errors/failures/skips. The core C++ suite contains 113 tests. Earlier totals of 264/265 preceded its rostest wrapper and the boundary process regressions. Container runs use `ROS_IP=127.0.0.1` when networking is disabled.

| SDK | GCC | CMake | PCL | Qt | ROS Python |
| --- | --- | --- | --- | --- | --- |
| Melodic / Ubuntu 18.04 | 7.5.0 | 3.10.2 | 1.8.1 | 5.9.5 | 2.7.17 |
| Noetic / Ubuntu 20.04 | 9.4.0 | 3.16.3 | 1.10.0 | 5.12.8 | 3.8.10 |

Melodic exposed two library differences: PCL 1.8 exports a whitespace-only compiler definition, and its cloud `swap()` leaves the destination header unchanged. The ROS 1 build discards empty options; successful cloud transforms now set the target frame and measurement stamp after swapping points. Existing moving/rotating-TF and frame-normalization regressions failed before the header fix. The measurement-time test also checks a stale input header stamp and preservation of its sequence number.

The ROS 1 core suite now runs through `add_rostest_gtest`, which supplies the master required by its node handles. Python tests use catkin-rewritten shebangs and SDK-compatible standard-library APIs, following [ROS Python version rules](https://reps.openrobotics.org/rep-0151/). Python 2 test deadlines use POSIX elapsed time instead of the unavailable `time.monotonic`; no planner clock policy changes. The rendered RViz regression also passes on Melodic/Qt5.9. These are build, behavioral, and plugin checks; they do not add a Melodic closed-loop simulator run.

## Build dependency audit

The planner requests PCL `common`, `filters`, and `kdtree` before loading the ROS conversion package, rather than discovering every installed PCL module. Filters retain their own transitive dependencies. The unused PCL search header is removed. On ROS 1, direct PCL cloud transformation also removes the `pcl_ros` manifest/CMake dependency and the runtime `pcl_ros_tf`, segmentation, features, and ML libraries.

| Build | PCL/VTK-related link-command entries, before / after | Direct ELF dependencies, before / after |
| --- | ---: | ---: |
| Native Jazzy | 67 / 15 | 28 / 28 |
| Lyrical desktop container | 72 / 14 | 27 / 27 |
| Noetic desktop-full container | 166 / 102 | 18 / 16 |

Noetic's conversion package still exports upstream I/O/VTK link paths; these exports are retained. ELF linkers already discarded unused direct libraries on the tested ROS 2 builds, so their shorter link commands are not a measured runtime-memory improvement. OpenCV image codecs and GUI support remain required by image saving and visualization. GDCM dictionary initialization in the allocation trace comes through the installed OpenCV image-codec library.

In eight alternating Noetic startups per build, sampled one second after process creation with the default configuration and no sensor input, median RSS changed from 99.60 to 79.78 MiB; mapped shared-library files fell from 217 to 213 in every run. All processes exited normally. This measures startup only. At that dependency-audit stage, Humble/Jazzy/Lyrical each reported 120 planner checks, Noetic reported 264 workspace checks, and the native C++ sanitizer suite passed 110 tests. Current counts are listed in the verification summary. The Noetic recording replay after the dependency change still produces three arrivals, 110 consistent graphs, 99 finite waypoints, and a normal exit.

## Cross-distribution recording replay

The same 50.643-second indoor input recording was replayed with simulated time and the corrected topic semantics: `/registered_scan` feeds scan ray tracing, `/terrain_map` feeds local terrain, and `/terrain_map_ext` feeds global terrain. The first operator goal waited for graph readiness; sensor and TF stamps were preserved. Each of three goals required a new nonarrival-to-arrival sequence and a requested-XY error below 0.6 m. Graph checks covered finite coordinates, frames, unique IDs, and resolved connections; waypoints were finite in `map`. All planners exited normally.

| Build | Arrivals | Valid graph snapshots | Finite waypoints |
| --- | ---: | ---: | ---: |
| Native Jazzy, `bdf582d` | 3 | 114 | 103 |
| Humble desktop container, same planner source | 3 | 113 | 100 |
| Lyrical desktop container, `5fb9a58` | 3 | 113 | 103 |
| Noetic desktop-full container, callback scheduling fix | 3 | 110 | 99 |
| Melodic desktop-full container, ROS 1 planner source at `d211f4d` | 3 | 110 | 99 |
| Native Jazzy ASan/UBSan, same planner source | 3 | 113 | 102 |

Humble used a SQLite bag with compatible metadata; all 18,363 serialized payloads, timestamps, and ordering matched the original MCAP recording. ROS 1 used [Rosbags](https://ternaris.gitlab.io/rosbags/) 0.11.9 to convert serialization and retain latched static TF. Every converted message passed a CDR round-trip check; one byte-level difference decoded to the same message values. Noetic previously produced no graph because measurement-time terrain matching failed at the default callback cadence. The sanitizer replay enabled leak detection and exited without AddressSanitizer, UndefinedBehaviorSanitizer, or LeakSanitizer findings. These are recorded-input checks, not closed-loop navigation on every distribution or nontrivial sensor-frame TF coverage.

Melodic consumed the same converted ROS 1 bag at the default 2.5 Hz graph/planning rate. Its Python 2 observer retained the same graph, waypoint, arrival-transition, and requested-XY checks. The planner production sources matched the ROS 1 refactor branch at `d211f4d`; the process exited with code zero and no error/fatal log entries.

## Repeated navigation with live allocation profiling

Native Jazzy `bdf582d` completed 20 cycles of the three indoor goals (60 arrivals) over 877.25 seconds with `is_static_env=false`, `is_multi_layer=false`, and the default 15-second dynamic exclusion. The profiled executable used Release optimization with debug symbols (`-O3 -g -DNDEBUG`), and heaptrack recorded only the planner process. Each goal required a new arrival sequence, a current path endpoint, independent actual motion, and less than 0.6 m error to the active goal. Maximum active-goal error was 0.148 m. Refinement moved some targets: 19 completions exceeded 0.6 m error to the original request, with a maximum of 0.842 m. This workload checks the documented adjusted-goal API; it does not establish exact requested-point arrival.

All 2,176 graph snapshots passed consistency checks, and 1,995 waypoints were finite in `map`. The observer received 4,348 scan-ray debug messages, including nonempty rays confirming that dynamic processing was active. The graph peaked at 86 nodes and ended at 82. Heaptrack reported 36,925,513 allocations and a peak tracked heap of 42.24 MB. Median sampled live heap was 30.995 MiB in seconds 60–180, 32.214 MiB in 420–540, and 32.354 MiB in 780–875. The late increase was about 0.14 MiB; sampled RSS after 60 seconds ranged from 141.81 to 151.52 MiB, including profiler overhead.

The trace flushed cleanly and passed its compression integrity check. Its 5.71 MB end-of-process not-freed total matched the short startup/navigation baseline at the reported precision; leading stacks were global GDCM dictionaries and LTTng initialization. This total is not an unreachable-leak result. The separate sanitized replay above checks that distinction for its shorter workload. The finite run and its slowly changing heap medians do not prove bounded memory for arbitrary routes, moving environments, or runtimes. Diagnostic traces and plots remain outside the repository.

## Garage ramp navigation validation

A native Jazzy Release run on `b5782bc` used the same upstream simulator snapshot and its supplied `garage` world. FAR used the default parameters with `is_multi_layer=true`; live parameters confirmed `voxel_dim=0.15`, `vehicle_height=0.75`, `map_handler/floor_height=2.5`, and `is_static_env=true`. Extended terrain analysis enabled `checkTerrainConn`. The vehicle started at `(0, 74)` with initial terrain Z of 0.1 m. RViz and joystick nodes were omitted; the installed FAR launch supplied the normal scan/terrain topic wiring.

All five sequential goals reported arrival during 236.9 seconds of observation. Each goal required a new false-to-true arrival sequence and an independent XYZ error below 0.6 m, followed by a one-second dwell:

| Requested XYZ (m) | Time from command (s) | XYZ error (m) |
| --- | --- | --- |
| `(15, 78, 0.85)` | 8.79 | 0.062 |
| `(50, 78, 1.70)` | 18.20 | 0.078 |
| `(50, 30, 4.15)` | 89.80 | 0.075 |
| `(50, 78, 1.70)` | 50.60 | 0.094 |
| `(0, 74, 0.85)` | 57.00 | 0.261 |

Sampled vehicle Z ranged from 0.850 to 4.376 m, exercising a 3.526 m height change and return. The observer checked all 574 graph snapshots for finite positions, matching sizes, unique IDs, and resolved connection IDs; the graph peaked at 603 nodes and ended at 598. All 563 observed waypoints were finite and in `map`. The route included backtracking between ramp lanes before reaching the upper goal, so it is not a shortest-route benchmark.

The recording's three cloud streams used `map` and strictly increasing stamps. Odometry bracketed all 1,147 messages in each terrain stream and 1,148 of 1,149 scans; the largest pose bracket was 5.429 ms. One scan had no bracket in the recording. This checks input availability for interpolation, not acceptance of every message or nontrivial sensor-frame TF chains.

An earlier pilot target at `(40, 74, 1.70)` was refined and reported arrival 3.07 m from the requested point. It failed the independent requested-goal acceptance check and is not counted as a successful ramp traversal. The table uses open-lane targets that passed that check. The simulator integrates planar motion and sets vehicle Z from terrain observations, rather than contact dynamics; this run covers one selected garage route on native Jazzy, not general multi-layer navigation or collision safety. Assets, launch wrappers, recordings, and plots remain outside the repository.

## Live Lyrical planner validation

The official Lyrical desktop container ran planner source `552a559` against the native Jazzy simulator at `8313dfed10533787582be8a6044483fe3299622f`. The processes used a private localhost domain, shared IPC, and the same user ID. Parameters were checked through the running planner's service.

| Scenario | Goals reached | Graph snapshots | Finite waypoints | Maximum requested-XYZ error |
| --- | ---: | ---: | ---: | ---: |
| Indoor, dynamic mode | 3 | 109 | 102 | 0.039 m |
| Garage ascent and return, multi-layer mode | 5 | 396 | 385 | 0.121 m |

Every goal required a new false-to-true arrival sequence and an independent requested-XYZ error below 0.6 m. Graph checks covered node headers, finite geometry, unique IDs, sizes, and resolved connections. Indoor processing produced 218 nonempty scan-ray messages. The garage follow-up also recorded active path endpoints, rechecked position after a one-second dwell, and observed a 3.329 m vehicle-height span and 3.667 m maximum graph-height span. Every first arrival observation was already inside the acceptance bound. Both FAR planners and their launch parents exited with code zero. The external simulator logs contain shutdown-time signal/abort errors, so these runs do not prove clean simulator-child teardown.

The first garage attempt stopped at a return-leg arrival observation 0.604 m from the request, after three successful goals. That failed result remains separate; the passing follow-up does not establish repeatability or explain the earlier miss. A further instrumented repeat reached its 110-second ascent limit before the return leg; it did not reproduce the original miss and is not counted as a passing route. A Humble/Jazzy live attempt could not connect the two SDKs before graph publication and is not counted as planner validation. Use a matching simulator SDK for that check: [ROS 2 does not guarantee cross-distribution communication](https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases.rst#cross-distribution-communications). Logs and observer reports remain outside the repository.

## SDK-matched indoor validation

Humble, Noetic, and Melodic ran the planner and upstream simulator inside their matching SDK containers, with dynamic processing enabled, `main_run_freq=2.5`, and `sensor_time_tolerance=0.2`. Humble used planner source `dc8b719` and simulator source `158e67b31b644ed1e8b06eb1d7f70e183cc62591`; Noetic used `51e3568` and `bf0cba71365271ebff09831a05afd78578150300`; Melodic used `8785613` and `9f7479ef90ff43e37fef586dc13680b13c89f316`. The simulator C++ sources were unchanged. Private headless wrappers omitted RViz/joystick/image rendering, retained a camera model for pose updates, and removed an unavailable, unused Rotors world plugin.

| SDK | Goals reached | Graph snapshots | Finite waypoints | Nonempty scan-ray messages | Maximum requested-XYZ error |
| --- | ---: | ---: | ---: | ---: | ---: |
| Humble | 3 | 107 | 100 | 260 | 0.285 m |
| Noetic | 3 | 106 | 99 | 213 | 0.162 m |
| Melodic | 3 | 106 | 99 | 213 | 0.213 m |

Each goal required a new false-to-true arrival sequence and independent XYZ error below 0.6 m. Graph and waypoint validation used the same frame, finite-geometry, ID, size, and connection checks as the Lyrical runs. Humble used the upstream wall-clock configuration; both ROS 1 observers and nodes shared simulated ROS time, configured before node initialization.

Both ROS 1 launches and masters exited with code zero and no ROS error/fatal log entries. Humble's FAR planner and graph decoder also stopped normally with no error/fatal entries in their launch log. Several external Humble simulator executables aborted during shutdown; the launcher's zero exit does not establish clean child-process teardown. That external-stack limitation remains separate from the passed planner navigation checks. Simulation assets, container recipes, logs, and observers remain outside this repository.

## SDK-matched ROS 1 garage validation

Noetic planner source `8285435` and matching simulator source `bf0cba71365271ebff09831a05afd78578150300` completed the five garage goals listed above in 124.05 wall seconds, using simulated ROS time, `is_multi_layer=true`, and `is_static_env=true`. The private headless wrapper enabled terrain connectivity, started at `(0, 74, 0.85)`, and retained the camera placeholder used for indoor validation. Simulator C++ sources were unchanged; the existing garage mesh was mounted read-only.

All goals required a fresh false-to-true arrival sequence, requested-XYZ error below 0.6 m, and a one-second dwell inside that bound, with a 180-second budget per goal. Maximum error was 0.484 m. Vehicle height spanned 3.300 m; graph height spanned 3.701 m. All 270 graph snapshots passed the frame/geometry/ID/connection checks, and 259 waypoints were finite. The launch and owned ROS master exited with code zero without ROS error/fatal log entries. This extends multi-layer integration evidence to ROS 1; it is not a cross-SDK speed comparison or a shortest-route claim.

## Dynamic terrain retention validation

Live ROS 1 and ROS 2 retention regressions create an obstacle, clear it with a farther raw return, and continue publishing stale terrain through 13 seconds after clearing. They verify that the obstacle stays excluded and can enter the map again after the configured interval expires. The old defaults failed both regressions; the ROS 2 regression also passes with AddressSanitizer and UndefinedBehaviorSanitizer, including leak detection and clean shutdown.

A subsequent 878-second dynamic-mode run on the committed scan/terrain wiring (`9f06cbf`, with the earlier 10-second exclusion) completed 20 route cycles and all 60 goals. Across elapsed windows 60–300, 300–600, and 600–878 seconds, sampled RSS medians were 124.37, 125.46, and 126.52 MiB; the overall range after 60 seconds was 121.39–130.62 MiB. The published graph peaked at 92 nodes and ended at 83. These are process RSS observations, including allocator and middleware behavior, rather than a proof of bounded live heap.

A separate moving-barrier experiment placed a 2.6 × 0.5 × 1.5 m box across the corridor, held it while the vehicle approached, then withdrew it along the corridor and into the adjacent room. With the 10-second exclusion, obsolete gate points reappeared after the upstream terrain outlived the exclusion, and the goal was not reached within 70 seconds. With 15 seconds, the old location stayed clear and all three goals reported arrival (14.44, 18.60, and 13.40 seconds from each command). This exercises one controlled obstacle withdrawal, not general moving-obstacle avoidance.

A later Jazzy run on `a7ee644` placed the same barrier after initial mapping and requested two route cycles. It and an instrumented repeat both exceeded the 110-second arrival wait after the controlled withdrawal, with zero arrivals; neither is counted as passing repeated-barrier navigation. They produced 338/335 consistent graphs and 325/326 finite waypoints, observed the held barrier, and stopped the planners normally. The repeat retained its input recording and path/map traces. Replaying its inputs through the unchanged planner produced 648 map/ray updates and showed the old barrier front face clearing. Late points in the wider spatial box matched the static side wall (within 0.083 m of the supplied mesh preview). That box count does not establish a clearing defect. The vehicle's long detour remains an unresolved integration case; the earlier successful single-withdrawal result does not cover it.

An additional diagnostic using the same dynamic-mode settings on `9cc0f54` retained the original failed 110-second arrival wait and continued for another 190 seconds. It still completed no goals after 300 seconds of arrival waiting (about 320 seconds after the command, including controlled barrier motion). All 808 graphs and 801 waypoints passed the observer's consistency/finite checks, and the planner and launch parent exited normally. This rules out explaining that run solely by the original observation budget; it does not establish the cause of its repeated detours.

A controlled native pipeline probe held odometry stationary while adding a box and then supplying visible clearing rays. With the shipped 2.5 Hz graph rate, the occupied phase produced 128 map obstacle points and four finalized graph corners; both returned to zero during clearing while extended terrain still carried the old box samples. The process exited normally. Separately, all 326 paired path/waypoint samples in the recorded failed route had forward components toward their selected navigation segment; the maximum XY deviation was 41.2 degrees. These checks do not explain the long detour, but do not establish a visible-clearing retirement or opposite-direction waypoint defect.

Two single-run controls used the same pre-cleanup planner executable (`bbcb37909d5f…`), settings, occupied barrier pose, and final barrier pose:

| Withdrawal/control | Goals reached | Maximum requested-XYZ error | Consistent graphs / finite waypoints |
| --- | ---: | ---: | ---: |
| Immediate relocation; approximately 16-second phase retained | 3 | 0.045 m | 147 / 110 |
| Gradual withdrawal; vehicle held during motion, then resumed | 3 | 0.042 m | 166 / 152 |

The held vehicle moved 0.001 m during withdrawal. These diagnostic controls change the observation/motion history; their starting poses also differ slightly between independent runs. They do not pass or replace the original autonomous gradual-withdrawal workload.

A full-trail replay audit selected candidate old-obstacle points more than 0.25 m from the static mesh preview and outside the expanded final box, in an interior region that avoids cropping potential ray-cell centers. Across 320 late paired map/ray frames, none of those points matched a 0.15 m clearing-ray voxel. Sampled failed-run frames retained 117–129 such points; the held control's same geometry mask retained at most 11. This approximate mask does not identify objects. The evidence is consistent with retained geometry that was not observed clear, while the route-selection contribution remains unresolved.

A fresh original-workload run after the contour-storage cleanup (`56b72f4`) still exceeded the 110-second arrival wait after gradual withdrawal, with zero arrivals. Its 334 graph snapshots and 326 waypoints passed the consistency/finite checks; the planner and launch parent exited normally. The storage cleanup is therefore not counted as a navigation fix.

The original workload also times out after the same 110-second arrival wait on the stable-ID search ordering (`7775349`), with zero arrivals, 334 consistent graphs, and 327 finite waypoints. The planner and launch parent exit normally. Stable-ID tie-breaking improves the focused reproducibility case but is not counted as a fix for this navigation workload.

A private diagnostic build of `ef04c4d` replayed the failed inputs while capturing the complete internal search graph, goal-connection decisions, and chosen paths. An independent quadratic reference matched reachability and reachable-node costs in all 167,666 node/mode comparisons across 328 snapshots (up to 375 nodes); maximum numerical cost difference was 0.000030 m. All 328 direct odometry-to-goal checks passed their eligibility, convexity, and direction gates but failed polygon visibility. Of the paths, 301 were freshly selected and 27 reused momentum. A persistent corridor anchor's goal segment crossed retained contours around Y=9–10 m, including while its connection was actively rechecked. These observations narrow the case toward retained geometry and re-observation; they do not establish a clearing-policy defect.

The diagnostic uses recorded odometry, so it is not a closed-loop navigation pass or an exact reproduction of the original chosen paths. It produced 648 map and ray frames with normal planner/player exits. Compared by publication order with the earlier replay, all 648 ray point multisets and 645 map point multisets matched; debug publication stamps differed by at most 0.0252 seconds. Instrumentation and callback phase remain limits on replay equivalence.

A subsequent physical re-observation control on `2592af4` retained the ordinary gradual withdrawal, then temporarily routed the local planner back along the vehicle's travelled breadcrumbs and held it for eight seconds before resuming FAR waypoints. Obstacle checks and FAR's original goal/map/graph state remained active; all 64 captured planning snapshots during the control retained goal ID 26 and anchor `(4, 14, 0.75)`. In the corridor interior, all 99 retained map points matched newly observed clearing-ray voxels, and the count became zero (including all 15 frames in the final three seconds of the hold). The original direct goal connection reopened.

That diagnostic completed all three goals with maximum requested-XYZ error 0.106 m, 226 consistent graphs, 212 finite waypoints, and normal planner/launch-parent exits. It shows recovery after physical re-observation without resetting the goal or clearing occupied cells artificially. It changes vehicle guidance and therefore does not pass the original autonomous workload or establish that the normal policy selects an appropriate re-observation pose.

## Source migration notes

The planner and RViz headers are internal source interfaces; these packages install executables/plugins rather than an exported planner library. Forks that call their C++ helpers must update alongside this refactor. All in-repository callers compile on the five supported SDKs.

| Previous use | Required update |
| --- | --- |
| `TransformPCLFrame` / `TransformPoint3DFrame` without time | Pass the original ROS measurement stamp (`rclcpp::Time` or `ros::Time`). Handle TF failure before changing state; sensor clouds require a positive stamp. |
| `RemoveNanInfPoints(cloud)` | Pass `true` for terrain height-cost intensity validation, `false` for XYZ-only scans. |
| `ReEvaluateGoalPosition(goal, adjust_height)` | Pass the current odometry position between these arguments; refinement no longer relies on an earlier traversal. |
| `SetLocalTerrainObsCloud(cloud)` | Supply a monotonically changing revision when the accepted local terrain changes, including clears; unchanged revisions reuse cached projection. |
| `GraphPlannerParams::free_thred` | Remove the unused field and its `free_counter_thred` configuration. Other removed parameter reads are listed above. |
| `ResetCloudIntensityWithTime`, `VoteRankInVotes`, `RemoveIndicesFromCloud` | Use `StackCloudByTime` for retention, `far_planner::TopTwoScores` for contour selection, or an explicit PCL filter for caller-owned index removal. These unused wrappers are removed. |
| `IdxStack`, `cur_scan_cloud_` | Use `std::vector<std::size_t>` for indices; retain stamped scan history in code that needs scan/terrain pairing. |
| `CTNode::connect_nodes` | Remove references to this unused member; use `front`/`back` for contour-ring traversal. |
| `NavNode::node_type`, `NodeType`, `LiDARMODEL::*` | Remove references to these unused declarations; they did not participate in planning or graph serialization. |
| `point_hash` / `point_comp` | Define an exact or explicitly quantized key and matching equality; tolerant point equality cannot safely back the old exact-float hash. |
| `nodeptr_gcomp`, `nodeptr_fgcomp`, `TNodeptr_fcomp` | Queue immutable `(cost, node)` entries and skip stale entries; changing a queued node's cost must not change heap ordering. |
| Old RViz header paths | Use `goalpoint_rviz_plugin/goalpoint_tool.hpp` and `teleop_rviz_plugin/{teleop_panel,drive_widget}.hpp`. Keep Qt5 on Humble/Jazzy/ROS 1 and Qt6 on Lyrical. |

`GraphExtractor` now owns graph teardown and is noncopyable; retain it by reference or unique ownership. Graph message schemas, `.vgh` layout, and default ROS topic names remain compatible.

## Branch validation baseline

The foundation builds on native Jazzy and on official Noetic and Lyrical desktop containers. Jazzy and Lyrical report 232 checks, zero failures, and 58 skips; these are predominantly lint/generated-code checks. Noetic initially registers zero behavioral tests. Headless planner startup, decoder remapping, and generation of a 25-node example boundary graph pass on all three tested distributions. These checks do not establish end-to-end navigation correctness; the refactor must add behavioral regressions and representative replay/simulation validation.
