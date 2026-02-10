Below is the **complete, consolidated firmware technical specification**, rewritten from first principles using the final architecture you converged on.

This version removes fragile assumptions (cluster ownership, hand size heuristics, door separation assumptions) and instead uses a **deterministic region-flow state model**, which remains reliable when:

* door and human temporarily merge into one sensed body,
* hand reaches forward before crossing,
* door opens from either side,
* door closes automatically,
* door remains open permanently,
* multiple people pass sequentially.

This document is written as **engineering firmware documentation**, not as an AI prompt.

---

# Firmware Technical Specification

## ESP32-C6 + VL53L5CX Doorway IN/OUT Counter

---

## 1. System Overview

The system is a doorway people counter using:

* ESP32-C6 microcontroller
* VL53L5CX Time-of-Flight sensor operating in 8×8 mode

The device is mounted above a doorway facing downward. The firmware detects directional movement through the doorway and generates **IN** and **OUT** events.

The firmware shall:

1. Detect directional crossings reliably.
2. Reject door motion and partial entry.
3. Operate correctly when door and human temporarily form a single sensed body.
4. Automatically determine usable sensing zones during calibration.
5. Use per-zone thresholds derived from floor distance.
6. Publish events via MQTT.
7. Buffer events when offline.
8. Persist calibration data in flash memory.
9. Operate continuously without manual intervention.

---

## 2. Hardware Configuration

### 2.1 Microcontroller

ESP32-C6

---

### 2.2 Distance Sensor

VL53L5CX

* Interface: I²C
* Resolution: 8×8 (64 zones)
* Continuous ranging mode

Input per frame:

```
distance_mm[64]
```

---

### 2.3 RGB LED (Common Anode)

Electrical behavior:

```
GPIO LOW  = LED ON
GPIO HIGH = LED OFF
```

GPIO naming:

| GPIO  | Color |
| ----- | ----- |
| LED_3 | Red   |
| LED_2 | Green |
| LED_1 | Blue  |

---

### 2.4 Calibration Trigger

GPIO:

```
IO9
```

Calibration mode entered when IO9 remains LOW for ≥ 5 seconds.

---

## 3. Device Identification

Firmware constants:

```
DEVICE_UID
BLE_BROADCAST_NAME
```

Included in all MQTT payloads.

---

## 4. Network Configuration

### 4.1 WiFi

```
SSID: EncryptedAir
PASSWORD: BlueSky@786
```

Requirements:

* Automatic connection at boot.
* Automatic reconnection on disconnect.

---

### 4.2 MQTT

Configurable parameters:

```
MQTT_HOST
MQTT_PORT
MQTT_USERNAME
MQTT_PASSWORD
MQTT_TOPIC
```

Requirements:

* MQTT connects after WiFi connection.
* Automatic reconnection.

---

## 5. System States

Top-level firmware state machine:

```
BOOT
WIFI_CONNECT
MQTT_CONNECT
NORMAL_OPERATION
CALIBRATION_MODE
```

Transitions:

```
BOOT → WIFI_CONNECT → MQTT_CONNECT → NORMAL_OPERATION
NORMAL_OPERATION ↔ CALIBRATION_MODE
```

---

## 6. LED Status Indication

Priority order:

1. Calibration mode
2. Error state
3. Network state

### LED Behavior

| Condition             | LED          |
| --------------------- | ------------ |
| Calibration mode      | Red blinking |
| WiFi + MQTT connected | Green ON     |
| WiFi connected only   | Orange       |
| WiFi disconnected     | Red ON       |

---

## 7. Sensor Processing Pipeline

Each sensor frame shall be processed in the following order:

```
1. Acquire distance frame
2. Apply static zone mask
3. Apply per-zone threshold
4. Determine active zones
5. Determine region occupancy
6. Update directional state machines
7. Generate event if sequence completed
```

---

## 8. Calibration Mode

Calibration determines:

* Zones of Interest (ZOI)
* Per-zone floor distance
* Per-zone detection threshold

Calibration results are stored in flash memory.

---

### 8.1 Calibration Procedure

#### Step 1 — Stabilization (10 seconds)

Sensor runs normally; data ignored.

---

#### Step 2 — Zone Analysis (15 seconds)

For each zone:

Compute:

```
mean_distance[z]
variance[z]
```

Zone classification:

##### VALID_WALK_ZONE

* Stable eliminating wall/pillar reflections
* Distance consistent with floor plane

##### STATIC_BLOCKED_ZONE

* Persistent short distance
* Wall, pillar, or door frame

##### UNUSABLE_ZONE

* Excessive noise

Result:

```
zone_mask[64]
```

---

#### Step 3 — Floor Distance Measurement (15 seconds)

For each VALID_WALK_ZONE:

```
floor_distance[z] = running_average(distance[z])
```

---

#### Step 4 — Per-Zone Threshold Calculation

For each VALID_WALK_ZONE:

```
threshold_candidate[z] = floor_distance[z] / 2
```

Threshold is continuously averaged:

```
threshold[z] = running_average(threshold_candidate[z])
```

Purpose:

* Ignore crawling persons
* Ignore floor-level objects

---

### 8.2 Stored Calibration Data

Stored in flash:

```
zone_mask[64]
floor_distance[64]
threshold[64]
```

Loaded automatically at boot.

---

## 9. Region Model

The doorway sensing area is divided into three logical regions:

```
A | M | B
```

Where:

* A = outside region
* M = doorway middle region
* B = inside region

Regions are defined using zone masks.

---

## 10. Active Zone Determination

A zone is active when:

```
zone_mask[z] == VALID_WALK_ZONE
AND distance_mm[z] < threshold[z]
```

---

## 11. Direction Detection Model

Direction detection is based on **ordered region occupancy flow**, not clustering or object separation.

This ensures reliability even when door and human are sensed as one body.

---

### 11.1 Region Occupancy

For each frame:

```
A_ACTIVE = any active zone in region A
M_ACTIVE = any active zone in region M
B_ACTIVE = any active zone in region B
```

---

### 11.2 IN Event State Machine

Valid IN sequence:

```
IDLE
 → A_ACTIVE
 → A_M_ACTIVE
 → M_B_ACTIVE
 → B_ACTIVE
 → CLEAR
 → EVENT_IN
```

Definitions:

* A_M_ACTIVE: A and M active simultaneously
* M_B_ACTIVE: M and B active simultaneously
* CLEAR: all regions inactive for N frames

Transitions allowed only forward.

Any reverse transition resets state to IDLE.

---

### 11.3 OUT Event State Machine

Mirror sequence:

```
IDLE
 → B_ACTIVE
 → M_B_ACTIVE
 → A_M_ACTIVE
 → A_ACTIVE
 → CLEAR
 → EVENT_OUT
```

---

### 11.4 State Stability Requirement

Each state must remain valid for a minimum number of consecutive frames before transition.

This prevents noise-induced transitions.

This is state confirmation, not time-based filtering.

---

### 11.5 Invalid Cases

No event generated when:

* Sequence incomplete.
* Direction reverses.
* Occupancy disappears before completion.
* Door motion alone activates regions.

---

## 12. Door Interaction Handling

Door motion is automatically rejected because:

* Door movement does not produce valid ordered A→M→B or B→M→A sequences.
* Door activity typically starts in only one region.
* Event requires full ordered progression and clearing.

No explicit door detection or clustering is required.

---

## 13. Event Publishing

MQTT payload format:

```json
{
  "device_uid": "...",
  "ble_name": "...",
  "timestamp": "...",
  "direction": "IN"
}
```

or

```
"direction": "OUT"
```

---

## 14. Event Buffering

### 14.1 Buffer Conditions

Events buffered when:

* WiFi disconnected
* MQTT disconnected
* Publish failure occurs

---

### 14.2 Buffer Structure

FIFO queue:

```
MAX_EVENTS = 100
```

Oldest event discarded when full.

---

### 14.3 Deferred Publishing

When connection restored:

```
publish buffered events sequentially
remove after successful publish
```

---

## 15. Persistence Requirements

Flash (NVS) storage must retain:

```
zone_mask
floor_distance
threshold
calibration parameters
```

Data must survive:

* power cycle
* firmware updates

---

## End of Firmware Specification

---

If you want next, the next practical step would be a **timing and frame-rate appendix**, defining minimum VL53 frame rate and state confirmation frame counts so detection remains consistent across different mounting heights.
