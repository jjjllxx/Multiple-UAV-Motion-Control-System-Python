# Multiple-UAV-Motion-Control-System-Python
Python version for multiple UAV motion control system

Original Matlab Version: [Multiple-UAV-motion-control-system](https://github.com/jjjllxx/Multiple-UAV-Motion-Control-System.git)

This repo converts most of the functions in original repo from Matlab to Python.  
I cleaned up and refactored the code for better readability.

Ensure you have Python 3.8+ installed. Then run:
``` sh
pip install -r requirements.txt
python main.py
```

Working Process:
1. Select start point and end point for each UAV.
2. Use RRT algorithm to plan path.
3. Convert path to trajectory.
4. Detect collision for all the trajectories. Slow down low-priority UAVs if a collision is detected.
5. Show the results.

Configurable Items:
1. Number of UAVs.
2. RRT type.
3. RRT step size.
4. World size.
5. Create a random world or known world.

Planned trajectories:

<img src="resources/traj1.png" alt="traj1" width="390"/> <img src="resources/traj2.png" alt="traj2" width="390"/>

Time-based trajectory in xyz:
![traj-xyz](resources/traj-xyz.png)

Comparsion of new and old traj2(traj2 is slowed to avoid collision with traj1):
![comparison](resources/comparison.png)

UAV1 and UAV2 moving as planned trajectories:
![trajectories](resources/trajectories.gif)
