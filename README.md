
# Autonomous Warehouse Shelf Attachment System (ROS 2)

An advanced ROS 2-based autonomous robotics application designed to enable a warehouse mobile robot (RB1) to safely navigate to, detect, center itself under, and mechanically lift a heavy cargo shelf using laser reflectivity signatures (Sensor Fusion) and precise coordinate frame transformation (TF2).

---

## 🛠️ Key Technical Skills Demonstrated
*   **Frameworks & Languages:** ROS 2 (Humble/Jazzy), C++17
*   **Perception & Sensor Fusion:** `sensor_msgs/LaserScan` processing using high-intensity reflectivity arrays (8000+ intensity scores) for feature extraction.
*   **Kinematics & Control:** Odometry-based precise angular dead-reckoning, angle normalization across $[-\pi, \pi]$ transitions, and discrete structural state-machines.
*   **Distributed Architecture:** Multi-threaded execution (`MultiThreadedExecutor`), Reentrant Callback Groups to eliminate deadlock conditions, custom ROS 2 Service Server/Client paradigms (`GoToLoading.srv`), and `tf2_ros` dynamic coordinate frame broadcasting.

---

## 🚀 System Architecture & Pipeline

The system is split into two modular phases to ensure high reliability and repeatable execution in dynamic environment profiles.


```

[Phase 1: Pre-Approach] ──> Drives to Wall ──> Normalizes Angle ──> Executes Exact Pivot (e.g. -90°)
│
[Phase 2: Final Approach] <── Triggers /approach_shelf Service <─────────────┘
│
├──> Parsers High-Reflectivity Laser Signatures (Shelf Legs)
├──> Dynamically Broadcasts 'cart_frame' (TF2) at Calculated Midpoint
└──> Drives Straight Under Shelf (Distance-Tracking) ──> Triggers `/elevator_up`

```

### Phase 1: Pre-Approach Node (`pre_approach_node`)
*   **Task:** Moves the robot safely from an arbitrary staging area to a precise loading zone directly facing the cargo shelf.
*   **Implementation:** Subscribes to `/scan` to sense geometry and tracks absolute heading via `/odom`.
*   **Key Engineering Problem Solved:** Developed an explicit **Angle Normalization Function** that prevents catastrophic wrapping errors when crossing the $180^\circ$ / $-180^\circ$ threshold, guaranteeing perfect alignment regardless of initialization state.

### Phase 2: Perception & Final Approach Server (`approach_service_node`)
*   **Task:** Uses active sensory perception to isolate the shelf components, generate a localized target space, and execute the physical attachment payload.
*   **Perception Strategy:** Instead of raw distance ranges which are highly prone to ambient environmental clutter, this system filters the **Intensity Array** data. The target pillars utilize reflective coating registering a raw signal magnitude $\ge 8000.0$.
*   **TF2 Integration:** Isolates the twin high-intensity clusters, extracts their midpoints via Euclidean projection, and actively broadcasts a new transform frame named `cart_frame`.
*   **Mechanical Load:** Drives straight through the calculated frame utilizing explicit odometric tracking before outputting a programmatic execution callback to the industrial elevator platform (`/elevator_up`).

---

## 📦 Package Structure & Launch Controls

```text
attach_shelf/
├── CMakeLists.txt
├── package.xml
├── srv/
│   └── GoToLoading.srv            # Custom Request/Response Service Interface
├── src/
│   ├── pre_approach_v2.cpp        # Pre-approach control node & service client
│   └── approach_service_server.cpp# Perception pipeline, TF broadcaster & motion server
└── launch/
    └── attach_to_shelf.launch.py  # Orchestrates multi-threaded servers and configurations

```

### Launch Configurations

**Execute Only the Pre-Approach and Framework Transforms:**

```bash
ros2 launch attach_shelf attach_to_shelf.launch.py obstacle:=0.4 degrees:=-90 final_approach:=false

```

**Execute the Full Pipeline (Nav, Align, Perception, Approach, Lift):**

```bash
ros2 launch attach_shelf attach_to_shelf.launch.py obstacle:=0.4 degrees:=-90 final_approach:=true

```

---

---

