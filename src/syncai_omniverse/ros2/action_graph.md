# Isaac Sim → ROS2 Bridge (Action Graph) 筆記

以 `clock_publisher.py` 為例，拆解 Isaac Sim 如何透過 **OmniGraph (Action Graph)** 把模擬資料橋接到 ROS2。

---

## 1. `clock_publisher.py` 在做什麼

把 Isaac Sim 的模擬時鐘 (`sim time`) 發佈到 ROS2 的 `/clock` topic (型別 `rosgraph_msgs/Clock`)，每個 playback tick 都發一次。

**為什麼需要它**：nav2、rviz2 若設 `use_sim_time=true`，會讀 `/clock` 當作時間來源。沒有這個 publisher，TF 時戳 (從 0 開始) 對不上 wall clock (~1.7e9)，整包資料都會被當成 `TF_OLD_DATA` 丟掉。

---

## 2. OmniGraph 的核心概念

Isaac Sim ↔ ROS2 的 bridge，本質上是在 USD stage 底下建一張 **計算圖 (computation graph)**：

- 每個 **node** 是一個 C++ 編好的功能方塊 (由 Isaac Sim 或 ROS2 bridge extension 提供)
- 節點之間有兩種線：
  - **Execution flow** (`execIn` / `execOut`)：觸發順序，類似程式呼叫
  - **Data flow** (`inputs:*` / `outputs:*`)：資料從上游 output pin 流到下游 input pin
- 用 Python 透過 `og.Controller.edit(...)` 把節點串起來，不用自己寫 C++

**Evaluator 類型**：
- `"execution"` → **Action Graph**：由 execution pulse 驅動（事件驅動）
- `"push"` / `"pull"` → 每 frame 自動計算（不在本檔案使用）

---

## 3. 逐段對照 `clock_publisher.py`

```python
og.Controller.edit(
    {"graph_path": graph_path, "evaluator_name": "execution"},
    ...
)
```
在 `/ClockActionGraph` 下建立 (或覆寫) 一張 Action Graph。

### 3.1 `CREATE_NODES` — 建三個節點

| Node 名稱 | Node type | 來源 extension | 職責 |
|---|---|---|---|
| `OnTick` | `omni.graph.action.OnPlaybackTick` | `omni.graph.action` | **事件源**：每個 playback tick 發一次 execution pulse |
| `ReadSimTime` | `isaacsim.core.nodes.IsaacReadSimulationTime` | `isaacsim.core.nodes` | **資料源**：回傳當下的 `simulationTime` (秒) |
| `PublishClock` | `isaacsim.ros2.bridge.ROS2PublishClock` | `isaacsim.ros2.bridge` | **橋接出口**：組成 `rosgraph_msgs/Clock` 並透過 DDS 發出 |

> 真正「講 ROS2 話」的只有 `PublishClock`。`rclpy` / DDS 的 publisher 物件被封在這個 C++ 節點內部。

### 3.2 `CONNECT` — 把節點串起來

```python
("OnTick.outputs:tick",                 "PublishClock.inputs:execIn"),
("ReadSimTime.outputs:simulationTime",  "PublishClock.inputs:timeStamp"),
```

對應兩種 flow：

```
 [OnTick] ──── tick (execution) ────► execIn  ┐
                                              ├──► [PublishClock] ──► /clock (DDS)
 [ReadSimTime] ── simulationTime (data) ───► timeStamp ┘
```

- **執行線**：`OnTick.tick` → `PublishClock.execIn`，告訴 PublishClock「現在該發一筆」
- **資料線**：`ReadSimTime.simulationTime` → `PublishClock.timeStamp`，把當下的 sim 秒數塞進 msg

`ReadSimTime` 沒有 execution input — 它是 pure data producer，下游問就回答 (lazy pull)。

### 3.3 `SET_VALUES` — 節點靜態屬性

```python
("PublishClock.inputs:topicName", topic),   # 預設 "/clock"
```

topic 名稱是常數、不是每 tick 會變的資料，所以用 `SET_VALUES` 而不是 `CONNECT`。

---

## 4. 為什麼 `/clock` 不掛 namespace

`/clock` 是 ROS2 全系統共用的 singleton，所有 `use_sim_time=true` 的 node 都會訂它，所以故意 **不** 加 robot namespace。

其他 topic (如 `odom`、`scan`、`cmd_vel`) 就要走 namespace，透過 `_ns.py` 處理。

---

## 5. 整體 bridge 流程

```
Isaac Sim physics step
         │
         ▼
   playback tick  ──────►  OnPlaybackTick  (emits execution pulse)
                                 │
                 ┌───────────────┘
                 ▼
           ROS2PublishClock
                 ▲
                 │ (pulls value on demand)
      IsaacReadSimulationTime
                 │
                 ▼
        ROS2 DDS middleware  ──►  /clock topic  ──►  nav2, rviz2, …
```

---

## 6. 常見踩坑

1. **Action Graph 是 event-driven**
   沒有 `OnPlaybackTick` (或 `OnPhysicsStep`) 觸發，整張圖就是死的。
   > 相關：`OnPhysicsStep` 必須設 `pipeline_stage = GRAPH_PIPELINE_STAGE_ONDEMAND`，否則節點靜默不觸發。

2. **Extension 要先 enable**
   `isaacsim.ros2.bridge` 沒開，`ROS2PublishClock` 這種 node type 不存在，`og.Controller.edit` 會炸。

3. **Kit event subscription 要存成全域變數**
   否則會被 Python GC 靜默回收，事件就不會進來。

4. **每個 Publish/Subscribe node 都封了一個 DDS 物件**
   所以 `lidar_publisher.py`、`odom_publisher.py`、`cmd_vel_subscriber.py` 結構都相似：
   - 換 node type (`ROS2PublishLaserScan` / `ROS2PublishOdometry` / `ROS2SubscribeTwist`)
   - 執行來源可能從 `OnPlaybackTick` 換成 `OnPhysicsStep`
   - 資料來源換成對應的 sensor / articulation node
   - 概念完全一樣

---

## 7. 這個 module 的其他檔案

| 檔案 | 對應的 bridge 模式 |
|---|---|
| `clock_publisher.py` | sim time → `/clock` (本檔案範例) |
| `tf_publisher.py` | TF tree → `/tf`、`/tf_static` |
| `odom_publisher.py` | articulation state → `/odom` |
| `lidar_publisher.py` | RTX lidar → `/scan` |
| `cmd_vel_subscriber.py` | `/cmd_vel` → articulation controller (反向，ROS2 → Isaac) |
| `_ns.py` | namespace 工具 |
