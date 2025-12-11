Development documentation 

Implementation of a distributed Particle Swarm Optimization within a multi agent system to solve the economic dispatch. 
Every agent provides initial schedule in the beginning aligned with their own device constraints. 
Initial schedules get send around and are combined to construct the search space for the agent internal Particle Swarm Optimization. 
Every agent performs particle swarm optimization to find its own best schedule in combination with the given, during the PSO fixed other schedules. 
All personal best schedules including their costs get send around and saved by every agent. 
When all expected schedules are received, each agent finds the cheapest schedule of all and sets this as the new global best schedule and cost.
For this schedule, the PSO finds the agents personal best schedule again and the process restarts. 

The main issue during the implementation was the handling of the async processes and the alignment of the different schedules found.
As soon as there was no direct communication between the agents anymore, the different agents were searching its own personal best on different schedules. 
Therefore, the msg collection of all schedules and costs in a dict was implemented, so at each iteration, after each own PSO the agents wait to compare all schedules of the other agents. 
Thereby, the same global bests are selected and all agents optimize in the same search space. 