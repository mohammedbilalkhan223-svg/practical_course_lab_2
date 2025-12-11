Development documentation 

1. Process steps 
    
Implementation of a distributed Particle Swarm Optimization within a multi agent system to solve the economic dispatch. 
Every agent provides initial schedule in the beginning aligned with their own device constraints. 
Initial schedules get send around and are combined to construct the search space for the agent internal Particle Swarm 
Optimization. 
Every agent performs particle swarm optimization to find its own best schedule in combination with the given, during the 
PSO fixed other schedules. 
All personal best schedules including their costs get send around and saved by every agent. 
When all expected schedules are received, each agent finds the cheapest schedule of all and sets this as the new global
best schedule and cost.
For this schedule, the PSO finds the agents personal best schedule again and the process restarts. 

2. Difficulties encounters 

The main issue during the implementation was the handling of the async processes and the alignment of the different 
schedules found.
As soon as there was no direct communication between the agents anymore, the different agents were searching its own
personal best on different schedules. 
Therefore, the msg collection of all schedules and costs in a dict was implemented, so at each iteration, after each 
own PSO the agents wait to compare all schedules of the other agents. 
Thereby, the same global bests are selected and all agents optimize in the same search space. 
Another problem we faced was the selection of the final schedule. The first approach was to get the final schedule and cost 
approved by all, but this resulted in message handling mismatches and some agents did not receive the new global best yet 
while others already wanted to approve the new global best. 
This was fixed by introducing a best_schedule found future, which is set when in all received global best schedules 
in one iteration no improvement was found. As soon as a new message with a better schedule / lower costs was received, 
the future was reset and the new iteration process was started.

3. Drawbacks of the implementation

In general, the PSO implementation in its current implementation and hyperparameter setting does not find an optimal schedule
or even a schedule without deviation. To enable this, a more effective search for the personal optimal and possible a better 
initial schedule setting is needed to nudge the PSO into the area of the best schedule. Furthermore, a wider PSO could be 
implemented by not directly setting the "best schedule found" future to true and ending the process 
but checking for a better solution in this schedule again for a limited number of iterations. As it is a metaheuristic, the best solution is not guaranteed by 
doing the process once. By adding a counter for the best schedule found and re-evaluating the current setup, the metaheuristic is 
more likely to find the optimum. 

The PSO implementation does not fulfil the requirements for redispatch as it is too slow in finding a common best schedule. 
To enable redispatch a timelimit for the PSO could be set, accepting not optimal solutions in a faster processing time.
This is not implemented in the solution yet. 


Sometimes, the solver does not converge towards a solution, especially with a large number of agents after the Observer Notification
the system does not come to an end. This could be tried to be avoided by only setting the "best schedule found" future to True
when all agents found no better schedule. This means, another message needs to be implemented saying, that agent... received
no cost improvement and when all agents sent this message, the future is set to true. Thereby, it is ensured that no optimization process 
or message still is in the backlog and is received delayed, starting a new process. This message backlog could be the reason 
why for larger scenarios the process is stuck after the final observer communication. this drawback is not resolved yet. 

4. Learnings 

In the next implementation I would try a different version of the schedule combination. Instead of taking the same 
schedule as the global best, it could be tried to combine all individual best schedules to one new schedule. The thought 
is, that thereby all devices have an influence on the updated schedule and a broader total search space can be analysed. 

Furthermore, a more in depth analysis of the hyperparameter would probably lead to better results but was not possible within
the limited time. 






