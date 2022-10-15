# Linear MPC for Lane Keeping and Obstacle Avoidance on Low Curvature Roads - Paper Application
Related Paper: [Linear MPC for Lane Keeping and Obstacle Avoidance on Low Curvature Roads](https://people.kth.se/~kallej/papers/vehicle_itsc13turri.pdf)

## Detailed Report
- See [Detailed Report](https://AtaberkOKLU.github.io/Linear-MPC-Lanekeeping-ObstacleAvoidance/Report/Report.pdf)

- Controller: Linear MPC Dual Mode
- Parameters: 
	- Sampling Rate: 10 Hz
	- Simulation Duration: 20 secs
	- Number of Horizon: 30
- Constraints:
	- Model Specific Constraints
	- Car Mechanical Constraints
	- Model Performance Constraints
	
## Test Results

### Three Obstacle Test
- Speed: 50 km/h

#### Position

<p align="center">
	<picture>
		<source media="(prefers-color-scheme: dark)" srcset="https://AtaberkOKLU.github.io/Linear-MPC-Lanekeeping-ObstacleAvoidance/Tests/ObstacleAvoidance/obs3_dark.png">
		<source media="(prefers-color-scheme: light)" srcset="https://AtaberkOKLU.github.io/Linear-MPC-Lanekeeping-ObstacleAvoidance/Tests/ObstacleAvoidance/obs3.svg">
		<img alt="FlowCharts" src="https://AtaberkOKLU.github.io/Linear-MPC-Lanekeeping-ObstacleAvoidance/Tests/ObstacleAvoidance/obs3.svg">
	</picture>
</p>

#### States

<p align="center">
	<picture>
		<source media="(prefers-color-scheme: dark)" srcset="https://AtaberkOKLU.github.io/Linear-MPC-Lanekeeping-ObstacleAvoidance/Tests/ObstacleAvoidance/obs3_states_dark.png">
		<source media="(prefers-color-scheme: light)" srcset="https://AtaberkOKLU.github.io/Linear-MPC-Lanekeeping-ObstacleAvoidance/Tests/ObstacleAvoidance/obs3_states.svg">
		<img alt="FlowCharts" src="https://AtaberkOKLU.github.io/Linear-MPC-Lanekeeping-ObstacleAvoidance/Tests/ObstacleAvoidance/obs3_states.svg">
	</picture>
</p>
