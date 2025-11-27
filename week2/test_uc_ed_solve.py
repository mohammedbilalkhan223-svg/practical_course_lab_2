from pyomo_solver_old import ED_solve, UC_solve
from scenarios.test_scenarios import get_test_scenarios

import sys
from operator import add

def main():
    for idx in range(9):
        print(f" --------------------Scenario {idx}----------------------------")

        if len(sys.argv) > 1:
            idx = int(sys.argv[1])

        p = get_test_scenarios()[idx]
        commited_units = UC_solve(p.devices, p.target, p.c_dev)
        #commited_units = [True, True, False]
        selection_cost = sum(p.devices[d].commitment_cost for d in range(len(p.devices)) if commited_units[d])
        print(f"Selection Cost: {selection_cost}")

        res = ED_solve(p.devices, commited_units, p.target, p.c_dev)

        print(f"Total cost: {res[1]}")

        print("Schedules:")
        for schedule in res[0]:
            print(schedule)

        print("Target:")
        print(p.target)

        print("Sum Schedule:")
        sum_schedule = [sum(x) for x in zip(*res[0])]
        print(sum_schedule)

        print("Diff:")
        diff_schedule = list(map(add, p.target, [-x for x in sum_schedule]))
        print(diff_schedule)

main()