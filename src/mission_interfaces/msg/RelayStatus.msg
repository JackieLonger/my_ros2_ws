# Relay mission status published by each drone for monitoring
uint8 active_drone_id
string phase                # IDLE / RELAY_TAKEOFF / GOTO_HANDOFF / EXECUTING / INTERRUPTED / RETURNING
uint32 current_index
uint32 total_waypoints
float64 current_lat
float64 current_lon
