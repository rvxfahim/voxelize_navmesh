# Point Cloud Voxelizer: Map Subtraction Analysis

This document summarizes the discussion regarding the map subtraction logic in the `voxelizer_node` (ROS 2).

## 1. Grid Alignment Between Changing Leaf Sizes
The incoming live LiDAR scans are downsampled using `leaf_size` (e.g., 0.1m), and then compared to the static map using `subtraction_leaf_size`. 
**Issue:** Is there a mismatch because of the two distinct downsampling scales?
**Conclusion:** No. There is no grid mismatch. Both the downsampled LiDAR points and the overall static map are mathematically projected (snapped) onto the integer voxel network created by `subtraction_leaf_size` immediately prior to the subtraction operation. The initial geometric downsampling with `leaf_size` does not affect the alignment of the final grid.

## 2. Chebyshev Radius: Who gets inflated?
*   **Conceptually:** It is the walls and obstacles of the static map that are being inflated to handle noise and minor alignment errors.
*   **Algorithmically (Current Implementation):** The real code loops through every single live scan point and projects an "inflated" Chebyshev search radius *from that scan point outwards* to hunt for intersection with a static map voxel.

## 3. Performance Tradeoff: Search Radius vs Pre-Inflating the Map
Currently, the node performs $O((2R+1)^3)$ map hash lookups per voxel to determine if the live point is "dynamic" (the foreground). Pre-inflating the overall map into a thick hash set once at startup would reduce this to exactly $O(1)$ lookup per scan point.

### Why was `O(R^3)` Chosen Originally?
1.  **Memory constraints:** Inflating a million-point map to a thick shell using an `unordered_set` creates a massive RAM blow-out and ruins CPU cache locality.
2.  **Fast Path (Early Exit):** Most points hit the map. The search loop breaks immediately upon connection. 
3.  **Live Parameter Tuning (The Core Reason):** By searching outwards from the scan point, the `subtraction_radius` can be dynamically altered mid-drive (`ros2 param set`). Re-inflating a massive map hash set at runtime would create seconds-long executor stalls, dropping active scans.

### The Verdict on Pre-Inflation
If you abandon live `subtraction_radius` tuning, **pre-inflating the static map would be computationally much faster**.
*   The current method pays the maximum latency toll precisely when searching for true foreground objects, as those points mandate all $125$ (at radius 2) hash checks miss before being declared "dynamic."
*   If your goal prioritizes CPU speed, lowering LiDAR latency, and isolating dense foregrounds, and you possess adequate system RAM, altering `mapCallback` to parse the map, build an inflated `std::unordered_set<VoxelKey>`, and simply doing a single $O(1)$ lookup for each live point is the far superior architectural choice.