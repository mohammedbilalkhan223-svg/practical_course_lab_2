
"""run for specific scenario specified in idx without result plots"""

from pyomo_solver import ED_solve
from scenarios.test_scenarios import get_test_scenarios

import sys
from operator import add

def main():
    idx = 7
    if len(sys.argv) > 1:
        idx = int(sys.argv[1])

    p = get_test_scenarios()[idx]
    res = ED_solve(p.devices, [True for _ in range(len(p.devices))], p.target, p.c_dev)

    print(f"Result Cost: {res[1]}")

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