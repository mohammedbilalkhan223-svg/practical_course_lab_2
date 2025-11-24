
"""run for iterating through all scenarios and saving the plots"""

import os
import time
from copy import deepcopy
import matplotlib.pyplot as plt

from pyomo_solver import ED_solve
from scenarios.test_scenarios import get_test_scenarios
from src.sim_environment.devices.ideal import IdealBatteryState, IdealFuelCellState


# Runtime measurement

def measure_runtime_once(devices, target, c_dev):
    committed = [True] * len(devices)

    t0 = time.perf_counter() #start timer
    ED_solve(devices, committed, target, c_dev)
    t1 = time.perf_counter() #end timer

    return t1 - t0 #return difference in sec


def benchmark_baseline_scenarios():
    print("=== Baseline scenarios (0–8) ===")
    scenarios = get_test_scenarios()

    for idx, p in enumerate(scenarios):
        devices = deepcopy(p.devices)
        target = p.target
        c_dev = p.c_dev

        runs = 1 #possible to adjust and calculate the average time over multiple runs
        times = [measure_runtime_once(devices, target, c_dev) for _ in range(runs)]
        avg_time = sum(times) / runs

        print(
            f"Scenario {idx}: n_dev={len(devices):2d}, "
            f"n_steps={len(target):3d}, "
            f"avg runtime over {runs} run(s) = {avg_time*1000:.3f} ms"
        )

# Plotting and saving (not shown, are saved to folder plots)

def plot_and_save_all_scenarios(output_dir="plots"):

    os.makedirs(output_dir, exist_ok=True)
    scenarios = get_test_scenarios()

    for idx, p in enumerate(scenarios):
        print(f"Generating plot for scenario {idx} ...")

        devices = p.devices
        committed = [True] * len(devices)
        target = p.target
        c_dev = p.c_dev

        power_schedules, cost = ED_solve(devices, committed, target, c_dev)

        n_steps = len(target)
        sum_schedule = [sum(x) for x in zip(*power_schedules)]

        battery_soc = {}
        fuel_level = {}

        # Rebuild SOC and fuel from schedules
        for i, dev in enumerate(devices):
            state = dev.state

            if isinstance(state, IdealBatteryState):
                E = [state.size * state.soc]
                for Pt in power_schedules[i]:
                    E.append(E[-1] - Pt)
                SOC = [e / state.size for e in E]
                battery_soc[i] = SOC

            elif isinstance(state, IdealFuelCellState):
                F = [state.fuel_amount]
                for Pt in power_schedules[i]:
                    F.append(F[-1] - Pt)
                fuel_level[i] = F

        fig, axs = plt.subplots(2, 2, figsize=(10, 6))
        fig.suptitle(f"Scenario {idx}  (Cost = {cost:.1f})")

        # Target vs Sum
        ax = axs[0, 0]
        ax.plot(range(n_steps), target, label="Target", linewidth=2)
        ax.plot(range(n_steps), sum_schedule, "--", label="Sum Schedule")
        ax.set_xlabel("Time step")
        ax.set_ylabel("Power")
        ax.set_title("Target vs Sum")
        ax.grid(True)
        ax.legend()

        # power schedule per device
        ax = axs[0, 1]
        for i, sched in enumerate(power_schedules):
            ax.plot(range(n_steps), sched, label=f"Device {i}")
        ax.set_xlabel("Time step")
        ax.set_ylabel("Power")
        ax.set_title("Device Power Schedules")
        ax.grid(True)
        ax.legend(fontsize=8)

        # Battery SOC
        ax = axs[1, 0]
        if battery_soc:
            for i, soc in battery_soc.items():
                ax.plot(range(len(soc)), soc, label=f"Battery {i}")
            ax.set_ylim(0, 1.05)
            ax.set_ylabel("SOC")
            ax.set_title("Battery SOC")
            ax.legend(fontsize=8)
        else:
            ax.text(0.5, 0.5, "No batteries", ha="center", va="center")
            ax.set_title("Battery SOC")
        ax.set_xlabel("Time step")
        ax.grid(True)

        # Fuel levels
        ax = axs[1, 1]
        if fuel_level:
            for i, F in fuel_level.items():
                ax.plot(range(len(F)), F, label=f"Fuel cell {i}")
            ax.set_ylabel("Fuel amount")
            ax.set_title("Fuel Levels")
            ax.legend(fontsize=8)
        else:
            ax.text(0.5, 0.5, "No fuel cells", ha="center", va="center")
            ax.set_title("Fuel Levels")
        ax.set_xlabel("Time step")
        ax.grid(True)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

        out_path = os.path.join(output_dir, f"scenario_{idx}.png")
        fig.savefig(out_path)
        plt.close(fig)

    print(f"All plots saved to folder: {output_dir}")


if __name__ == "__main__":
    benchmark_baseline_scenarios()
    plot_and_save_all_scenarios("plots")
