# HamBot World State Protocol

This document describes the JSON packet streamed from the overhead perception
server to all connected robot clients.

---

## Transport

| Property        | Value                        |
|-----------------|------------------------------|
| Protocol        | TCP                          |
| Default port    | `9999`                       |
| Message format  | One JSON object per line (`\n` delimited) |
| Typical rate    | ~30 packets/second           |

Each message is a single line of JSON followed by a newline character.
The receiver reads line by line — no length headers or framing needed.

---

## Packet Structure

```json
{
    "timestamp": 1740339600.123,
    "frame_id":  4821,
    "fps":       29.8,
    "robots": {
        "42": {
            "detected":        true,
            "x":               45.2,
            "y":              -12.8,
            "heading_deg":    127.4,
            "heading_current": true,
            "heading_age":     0,
            "source":         "aruco",
            "confidence":      0.94
        }
    }
}
```

---

## Field Reference

### Top-level fields

| Field       | Type    | Description                                      |
|-------------|---------|--------------------------------------------------|
| `timestamp` | float   | Unix time (seconds) when the frame was captured  |
| `frame_id`  | int     | Monotonically increasing frame counter           |
| `fps`       | float   | Server-side capture rate                         |
| `robots`    | object  | Map of detected robots, keyed by ArUco marker ID |

### Per-robot fields (`robots["<id>"]`)

| Field             | Type    | Description                                                    |
|-------------------|---------|----------------------------------------------------------------|
| `detected`        | bool    | Whether the robot was detected this frame                      |
| `x`               | float   | World-frame X position (cm)                                    |
| `y`               | float   | World-frame Y position (cm)                                    |
| `heading_deg`     | float   | Heading in degrees. 0° = right, 90° = up, CCW positive        |
| `heading_current` | bool    | `true` = fresh ArUco heading. `false` = held from prior frame |
| `heading_age`     | int     | Frames since the last fresh ArUco heading reading              |
| `source`          | string  | Detection method: `"aruco"`, `"hsv"`, or `"lost"`             |
| `confidence`      | float   | Detection confidence, 0.0–1.0                                  |

---

## Robot Identification

Each robot is identified by its ArUco marker ID. Your robot knows its own ID
at startup. To read your own state:

```python
MY_ARUCO_ID = "42"   # set this to your robot's marker ID

state, age = receiver.get()
if state is not None:
    me = state["robots"].get(MY_ARUCO_ID)
    if me and me["detected"]:
        x, y = me["x"], me["y"]
```

Your robot can also read the position of any other robot in the field using
their ArUco ID in the same way.

---

## Detection Sources

The `source` field indicates how the robot's position was determined:

| Value    | Meaning                                                         |
|----------|-----------------------------------------------------------------|
| `"aruco"` | Position and heading from ArUco marker — most accurate        |
| `"hsv"`   | Position from color detection fallback — heading may be held  |
| `"lost"`  | Robot not detected this frame — last known values may be held |

When `heading_current` is `false`, the heading value is carried forward from
the most recent ArUco detection. Use `heading_age` to decide how much to
trust it — values above ~10 frames should be treated with caution.

---

## Notes

- Position values are in **centimeters** in the world frame, with the origin
  at the center of the field.
- The packet always includes an entry for every robot the server is configured
  to track, even if `detected` is `false`.
- The protocol is intentionally minimal. Fields may be added in future
  versions — write your parsing code defensively using `.get()` with defaults.

---

## Version

This document reflects the current protocol as of April 2026.
The packet structure is expected to evolve as the system develops.
Check this file and the server repo's changelog for updates.
