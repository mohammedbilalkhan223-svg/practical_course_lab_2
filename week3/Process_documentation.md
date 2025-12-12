# Development Documentation

## 1. Process Overview

This project implements a **distributed Particle Swarm Optimization (PSO)** embedded in a **multi-agent system (MAS)** 
to solve the Economic Dispatch (ED) problem.

### 1.1 Initial Schedule Generation
- Each agent represents a device with its own technical and operational constraints.
- At the beginning of the optimization, every agent independently computes an **initial feasible schedule** that 
respects its device constraints.
- These initial schedules are exchanged among the agents (via the neighbors) so that every agent has all initial schedules of all other known agents.

### 1.2 Construction of the Search Space
- The received initial schedules are combined to include the other devices in the cost function of the agent’s internal PSO.
- For each agent, the schedules of the *other* devices are treated as fixed during its local PSO run.

### 1.3 Local PSO Optimization
- Each agent runs a **local PSO** to search for its own best schedule, assuming the other agents’ schedules remain fixed.
- The result is the agent’s **personal best schedule** and the associated cost.

### 1.4 Exchange of Personal Best Schedules
- After the local PSO run, all agents send their personal best schedule combination of all devices and corresponding 
costs to the other agents.
- Each agent maintains a **dictionary of received schedule combination per agent and the corresponding costs**, indexed by agent ID.

### 1.5 Global Best Selection
- Once an agent has received all expected schedules for the current iteration, it:
  - Compares all collected costs for the schedule combinations (including its own).
  - Selects the **cheapest cost** as the new **global best cost** and updates the global best schedule.

### 1.6 Iterative Refinement
- For the newly selected global best schedule, each agent runs PSO again to update its personal best in the context of this global best.
- This process (local PSO → exchange → global best update) is repeated until **no further improvement** is found.


## 2. Difficulties encountered

### 2.1 Asynchronous execution and schedule alignment
- A major challenge was the handling of **asynchronous processes** and the **alignment of schedules** across agents.
- When agents were not perfectly synchronized, some agents started optimizing for outdated schedules while others had already moved on.
- This led to inconsistent search spaces and agents effectively optimizing different versions of the global schedule.

**Mitigation:**
- A **message collection mechanism** using a dictionary was implemented:
  - After each local PSO run, an agent waits until it has received schedules and costs from **all other agents**.
  - Only then does it update the global best and proceed to the next iteration.
- This ensures that all agents use the **same global best schedule** and thus operate within a consistent search space.
- A further improvement could be to include the iteration where the schedule is coming from.

### 2.2 Final schedule selection and message handling
- Initially, the idea was to explicitly **approve the final schedule and cost** by all agents.
- However, this approach caused message handling mismatches:
  - Some agents had already updated to a new global best.
  - Others were still waiting for approval messages related to the previous global best.
- This led to inconsistent states and synchronization issues.

**Mitigation:**
- A `best_schedule_found` **future** was introduced:
  - In each iteration, if no better global schedule is found among all received candidates, the future is set to `True`.
  - As soon as a message with a better schedule (lower cost) arrives, the future is reset and a new iteration begins.
- This mechanism allows the system to **terminate** once a stable global best has been reached, without 
requiring a separate explicit approval phase. 



## 3. Drawbacks and limitations

### 3.1 Solution quality and hyperparameters
- In its current form, the PSO implementation and chosen hyperparameters **do not reliably find an optimal schedule** or even a schedule without deviation from the target.

- As a metaheuristic, PSO is sensitive to:
  - Initialization of the particles (initial schedules).
  - Choice of hyperparameters ( \(c_1, c_2, w\)).
- With the present settings, the search can remain in suboptimal regions.

**Potential Improvements:**
- Improve the **initial schedule generation** to start the PSO closer to promising regions of the search space.
- Perform a more systematic **hyperparameter tuning** to balance exploration and exploitation.
- Extend the process by:
  - Not immediately terminating when the first "best schedule found" condition is met.
  - Allowing a **limited number of additional iterations** where the system re-evaluates the current best solution.
- This would increase the probability of escaping local minima and approaching a global optimum.

### 3.2 Unsuitability for real-time redispatch
- In its current form, the distributed PSO is **too slow** to be directly applied to real-time redispatch.
- The iterative nature of schedule exchanges and local PSO runs leads to relatively long computation times, especially as the number of agents grows.

**Potential Improvements:**
- Introduce a **time limit** for PSO:
  - Accept suboptimal solutions in exchange for **bounded computation time**.
- This time limiting is not yet implemented, so is the redispatch not yet implemented. 

### 3.3 Convergence and termination for larger scenarios
- For scenarios with a **larger number of agents**, the solver sometimes does not converge and the system does not terminate, even after the final observer notification.
- A likely cause is **message backlog and delayed delivery**:
  - Some agents may still send or receive messages related to previous iterations.
  - Some coroutine is awaited which blocks the termination. 
  
**Potential Improvements:**
- Only set the `best_schedule_found` future to `True` when **all agents** explicitly confirm that they did **not** find an improved schedule in the current iteration.
- This would require an additional message type, e.g., a "no improvement" notification from each agent.
- Once all such messages are collected, the future can be set, ensuring that:
  - No optimization process is still running.
  - No relevant messages remain in the backlog.
- This mechanism is not yet implemented in the current version.


## 4. Learnings and future directions

### 4.1 Alternative schedule combination strategies
- In the current implementation, the one global best schedule is used as the reference for all agents. In this schedule only one agent's schedule is optimized. 
- An alternative approach would be to **combine the individual personal best schedules** of all agents defined by the PSO into a new global schedule. (which needed additional cost calculation)
  - This would ensure that each device contributes directly to the updated schedule.
  - It may also increase the diversity of the search and explore a **broader search space**.

### 4.2 Improved hyperparameter tuning
- A more thorough analysis and tuning of PSO hyperparameters could significantly improve:
  - Convergence speed.
  - Solution quality.
  - Robustness across different scenarios and numbers of agents.
- Due to time constraints, such a detailed hyperparameter study was not feasible in the current project but is identified as an important next step.

### 4.3 Improved message handling and logging
- While in ring topologies messages coming from the other direction take a while to arrive, it is possible that a agent already started a new PSO with a new global schedule. 
- When the message then arrives, a possible backsetting towards the old schedule is possible. 
- This could be handled by an improved message handling and logging, ensuring that older, already handled messages from earlier iterations are not handled anymore. 
---

**Summary:**

The implemented distributed PSO in a multi-agent system successfully demonstrates a working concept for decentralized Economic Dispatch. 
However, the current version shows limitations in terms of solution optimality, computational speed, and robustness for larger scenarios. 
With further refinement in synchronization, schedule combination strategies, and PSO configuration, the approach has the potential to 
become a more practical tool for distributed scheduling and, with additional enhancements, real-time redispatch.

