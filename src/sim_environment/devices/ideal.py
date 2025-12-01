from src.sim_environment.devices.abstract import AbstractDevice, AbstractState
# Imports the abstract base classes from the previous file.
import logging
# Imports logging to print warnings when illegal states occur.

# -----------------------------------
# States
# -----------------------------------
class IdealBatteryState(AbstractState):
# Defines a concrete implementation of AbstractState for a simple battery model.
    def __init__(self, size, soc, p_min, p_max, final_soc=None):
# Constructor takes battery capacity (size), current SOC, min/max power limits, and an optional final SOC requirement.
        if p_max < 0:
            raise ValueError("Battery max power has to be >= 0.")
# Ensures charging power limit is non-negative.
        if p_min > 0:
            raise ValueError("Battery min power has to be <= 0")
# Ensures discharging power limit is negative or zero.
        if soc < 0 or soc > 1:
            raise ValueError("SOC has to be between 0 and 1.")
# SOC must be a fraction between 0 and 1.
        self.size = size
        self.soc = soc
        self.p_max = p_max
        self.p_min = p_min
        self.power = 0
# Stores arguments and initializes device power to zero.
        if final_soc is None:
            final_soc = soc
        self.final_soc = final_soc
# If no final SOC is specified, the current SOC is used.
    def __asdict__(self):
# Converts the state to a dictionary — useful for serialization/saving.
        d = {
            "size": self.size,
            "soc": self.soc,
            "p_min": self.p_min,
            "p_max": self.p_max,
            "final_soc": self.final_soc
        }
        return d
# Creates a dictionary representation of battery attributes.
    @classmethod
    def __fromdict__(cls, attrs):
        return cls(**attrs)
# Re-creates a battery state from a dictionary using keyword arguments.
    @classmethod
    def __serializer__(cls):
        return cls, cls.__asdict__, cls.__fromdict__
# Returns all the necessary serializer functions for external tools.
    def __repr__(self):
# Defines a readable printed representation of the object.
        output = (
            f"size: {self.size}\n"
            f"soc: {self.soc}\n"
            f"p_max: {self.p_max}\n"
            f"p_min: {self.p_min}\n"
            f"power: {self.power}\n"
        )
        return output

    def set_power(self, power):
# Implements required method to set battery power.
        if power < self.p_min or power > self.p_max:
            raise ValueError("Trying to set battery power out of bounds.")
# Prevents setting a power outside allowable charge/discharge limits.
        self.power = power
# Stores the desired power.
    def predict_state(self, power, n_steps):
# Predicts future SOC after applying a given power for n_steps.
        ok = True
        if power < self.p_min or power > self.p_max:
            ok = False
            logging.warning("Predicting state with impossible power. - Battery")
# Checks whether the requested power is physically possible.
        energy = power * n_steps
        new_soc = ((self.soc * self.size) - energy) / self.size
# Computes total energy change and converts it to a new SOC.
        if new_soc < 0 or new_soc > 1:
            ok = False
            logging.warning("Battery can not sustain this power level.")
# SOC outside bounds means the action is impossible.
        if new_soc < 0:
            new_soc = 0

        if new_soc > 1:
            new_soc = 1
# Clamps SOC within bounds.
        return ok, IdealBatteryState(self.size, new_soc, self.p_min, self.p_max)
# Returns a tuple: (feasible?, new_state).
    # return the maximum amount of power the battery can sustain for <n_steps>
    def max_power(self, n_steps):
        energy = self.size * self.soc
        power = energy / n_steps
# Ensure power does not exceed hardware limit.
        if power > self.p_max:
            return self.p_max

        return power
# Ensure power does not exceed hardware limit.
    # return the minimum amount of power the battery can sustain for <n_steps>
    def min_power(self, n_steps):
        neg_energy = self.size * (self.soc - 1)
        power = neg_energy / n_steps
# Computes the most negative (charging) power possible before exceeding full SOC.
        if power < self.p_min:
            return self.p_min

        return power
# Respect minimum power limit.
    # step the device and return the energy that was used
    def step(self):
        if self.power == 0:
            return 0
# If no power chosen, nothing happens.
        # bounds checks
        p_max = self.max_power(1)
        p_min = self.min_power(1)
# Compute one-step feasible power bounds.
        if self.power > p_max:
            logging.warning(f"Battery can not sustain power level: {self.power}. -- Setting to {p_max}")
            self.power = p_max
            self.soc = 0
            return self.power
# If requested power exceeds capability, adjust and set SOC to zero.
        elif self.power < p_min:
            logging.warning(f"Battery can not sustain power level: {self.power}. -- Setting to {p_min}")
            self.power = p_min
            self.soc = 1
            return self.power
# Same for exceeding charging limit (SOC becomes full).
        ok, new_state = self.predict_state(self.power, 1)
# Predict what happens if the power is applied.
        if ok:
            self.soc = new_state.soc
            return self.power
# If feasible, update SOC and return energy used.
class IdealLoadState(AbstractState):
    def __init__(self, p_min):
        if p_min > 0:
            raise ValueError("Load min_power has to be negative.")

        self.p_min = p_min
        self.p_max = 0
        self.power = 0

    def __asdict__(self):
        d = {
            "p_min": self.p_min
        }
        return d

    @classmethod
    def __fromdict__(cls, attrs):
        return cls(**attrs)

    @classmethod
    def __serializer__(cls):
        return cls, cls.__asdict__, cls.__fromdict__

    def __repr__(self):
        output = f"p_min: {self.p_min}\n" f"power: {self.power}\n"
        return output

    def set_power(self, power):
        if power < self.p_min or power > 0:
            raise ValueError("Trying to set load power out of bounds.")

        self.power = power

    def predict_state(self, power, n_steps):
        ok = True
        if power < self.p_min or power > 0:
            ok = False
            logging.warning("Predicting state with impossible power. - Load")

        return ok, self

    def min_power(self, n_steps):
        return self.p_min

    def max_power(self, n_steps):
        return 0

    def step(self):
        return self.power


"""
Works essentially like a one-way battery with a consumption rate of 1 fuel unit per power.
"""
class IdealFuelCellState(AbstractState):
    def __init__(self, fuel_amount, p_max, change_max=1000, p_prev=None):
        if p_max < 0:
            raise ValueError("Fuel cell must have positive max_power.")

        if fuel_amount < 0:
            raise ValueError("Fuel amount is negative.")

        if p_prev is None:
            p_prev =0

        self.fuel_amount = fuel_amount
        self.p_min = 0
        self.p_max = p_max
        self.power = self.p_prev = p_prev
        self.change_max = change_max

    def __asdict__(self):
        d = {
            "fuel_amount": self.fuel_amount,
            "p_max": self.p_max,
            "change_max": self.change_max,
            "p_prev": self.p_prev
        }
        return d

    @classmethod
    def __fromdict__(cls, attrs):
        return cls(**attrs)

    @classmethod
    def __serializer__(cls):
        return cls, cls.__asdict__, cls.__fromdict__

    def __repr__(self):
        output = (
            f"fuel_amount: {self.fuel_amount}\n"
            f"p_max: {self.p_max}\n"
            f"power: {self.power}\n"
            f"change_max: {self.change_max}\n"
            f"p_prev: {self.p_prev}\n"
        )
        return output

    def set_power(self, power):
        if power > self.p_max or power < 0:
            raise ValueError("Trying to set fuel cell power out of bounds.")

        # NOTE: You can set whatever power value you want in this function within the hard bounds.
        # If the set power deviates too much from the previous power value, the problem will be
        # noticed and corrected in the <step> function, not here!

        self.power = power

    def predict_state(self, power):
        ok = True

        if power < 0 or power > self.p_max:
            ok = False
            logging.warning("Predicting state with impossible power. - Fuel Cell")

        remaining_fuel = self.fuel_amount - power

        if remaining_fuel < 0:
            ok = False
            logging.warning("Fuel cell can not sustain this power level.")
            remaining_fuel = 0

        return ok, IdealFuelCellState(remaining_fuel, self.p_max, self.change_max, power)

    def max_power(self):
        power = self.fuel_amount

        if power > self.p_max:
            return self.p_max

        return power

    def min_power(self):
        return 0

    def step(self):
        # check rate of change constraint
        if self.power - self.p_prev > self.change_max:
            logging.warning(f"Step: Fuel Cell upper change violation: {self.p_prev} to {self.power} -- Setting {self.p_prev + self.change_max} instead.")
            self.power = self.p_prev + self.change_max
        if self.power - self.p_prev < -self.change_max:
            logging.warning(f"Step: Fuel Cell lower change violation: {self.p_prev} to {self.power} -- Setting {self.p_prev - self.change_max} instead.")
            self.power = self.p_prev - self.change_max

        ok, new_state = self.predict_state(self.power)

        if ok:
            self.fuel_amount = new_state.fuel_amount
            self.p_prev = new_state.p_prev
            return self.power

        # could not run the set power
        # can only have failed on fuel constraint
        # ignore the rate of change in this case
        self.power = self.max_power()
        self.fuel_amount = 0
        return self.power


# -----------------------------------
# Devices
# Cost functions are linear functions of the power output.
# Dynamics are entirely set by the state object.
# -----------------------------------
class IdealDevice(AbstractDevice):
# Represents a device with a state and cost.
    def __init__(self, state, c_op, commitment_cost=40):
# Takes a state (battery, fuel cell, etc.), an operational cost, and a one-time startup cost.
        self.cumulative_cost = 0
        self.state = state
        self.c_op = c_op
        self.stepped_powers = []
        self.commitment_cost = commitment_cost
# Stores device info; stepped_powers logs energy usage each timestep.
    def set_output_power(self, power):
        self.state.set_power(power)
# Forwards to the state object.
    def cost(self, energy):
        return abs(energy) * self.c_op
# Cost is linear in absolute energy.
    def step(self):
        used_energy = self.state.step()
        self.cumulative_cost += self.cost(used_energy)
        self.stepped_powers.append(used_energy)
# Applies power, updates cumulative cost, and logs power.