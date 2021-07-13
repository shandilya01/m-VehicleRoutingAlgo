# m-Vehicle Routing Problem with added constraints for Jobs Carrying Capacity, Charge Availability(for EV's) and timing deadlines.
C++ implementation of a Multiple-Vehicle Routing algorithm among certain pickup and drop locations.
and deadlines for each vehicle to reach a certain drop location.

## Algorithm comprises of 2 sub-algorithms: 

* ### Allocation Algo(Local Search) :
Allot each new incoming job to a certain vehicle.

Perform swaps and relocations on jobs among different Vehicles till we end up on a local minima.
* ### Scheduler Algo(Modified Held-Karp) :
A top-down Dynamic Programming approcah for Held-Karp algo.

Finding the most optimal schedule for each vehicle with minimisation of tardiness cost for deadlines and the arrival time at drop locations.

