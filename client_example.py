from world_state_receiver import WorldStateReceiver
import time

# use ip of your server
receiver = WorldStateReceiver(server_ip="192.168.1.165")

try:
    while True:
        time.sleep(1.0) # simple control as to how often to pull state
        state, age = receiver.get()

        if state is None:
            time.sleep(0.01)
            continue

        if age > 0.5:
            # Data is stale - take a safe action
            continue

        # Use the world state
        robot = state["robots"]["5"]   # your robot's ArUco ID
        for robot_id, robot in state["robots"].items():
            if robot["detected"]:
                print(f"Robot {robot_id}: Source: {robot['source']} Position: ({robot['x']:.1f}, {robot['y']:.1f})")

finally:
    receiver.stop()
