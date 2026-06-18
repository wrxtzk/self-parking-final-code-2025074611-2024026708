# Self-Parking Simulation Final Project Code

Course: Data Structure and Algorithms, Spring 2026

Team:
- 2025074611, 진성우
- 2024026708, 담조곤

## Files

- `student_planner.py`: rule-based parking planner and Stanley controller.
- `my_agent.py`: launcher for the simulator IPC client.
- `ipc_client.py`: JSONL TCP client used to communicate with the simulator.

## Validation Results

| Map | Source | Result | Time | Score |
| --- | --- | --- | --- | --- |
| Default Lot | saved replay seed 55330173 | Success | 84.0 s | 80.5 / 100 |
| Crowded Lot | offline run seed 0 | Success | 65.2 s | 85.0 / 100 |
| Full House Lot | offline run seed 0 | Collision with occupied slot | 50.9 s | 0.0 / 100 |

## Run

Place these files in the `self-parking-user-algorithms` workspace and start the simulator first.

```bash
python my_agent.py --host 127.0.0.1 --port 55556
```

The simulator repository should run `demo_self_parking_sim.py` and listen on the same host and port.
