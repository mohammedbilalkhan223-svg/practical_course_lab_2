from week2_group1.src.sim_environment.devices.abstract import AbstractState


# -----------------------------------
# States
# -----------------------------------
class IdealBatteryState(AbstractState):
    def __init__(self, size, soc, p_min, p_max, final_soc=None):
        if p_max < 0:
            raise ValueError(f"Battery max power has to be >= 0 and is: {p_max}")

        if p_min > 0:
            raise ValueError(f"Battery min power has to be <= 0 and is {p_min}")

        if soc < 0 or soc > 1:
            raise ValueError(f"SOC has to be between 0 and 1 and is: {soc}")

        self.size = size
        self.soc = soc
        self.p_max = p_max
        self.p_min = p_min
        self.power = 0
        if final_soc is None:
            final_soc = soc
        self.final_soc = final_soc

    def __asdict__(self):
        d = {
            "size": self.size,
            "soc": self.soc,
            "p_min": self.p_min,
            "p_max": self.p_max,
            "final_soc": self.final_soc
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
            f"size: {self.size}\n"
            f"soc: {self.soc}\n"
            f"p_max: {self.p_max}\n"
            f"p_min: {self.p_min}\n"
            f"power: {self.power}\n"
        )
        return output

    def set_power(self, power):
        if power < self.p_min or power > self.p_max:
            raise ValueError(f"Trying to set battery power out of bounds. min: {self.p_min}, max: {self.p_max}, attempt: {power}")

        self.power = power

    # return the maximum amount of power the battery can sustain for <n_steps>
    def max_power(self, n_steps):
        energy = self.size * self.soc
        power = energy / n_steps

        if power > self.p_max:
            return self.p_max

        return power

    # return the minimum amount of power the battery can sustain for <n_steps>
    def min_power(self, n_steps):
        neg_energy = self.size * (self.soc - 1)
        power = neg_energy / n_steps

        if power < self.p_min:
            return self.p_min

        return power

    # step the device and return the energy that was used
    def step(self):
        return self.power

class IdealLoadState(AbstractState):
    def __init__(self, p_min):
        if p_min > 0:
            raise ValueError(f"Load min_power has to be negative and is: {p_min}")

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
            raise ValueError(f"Trying to set load power out of bounds. min: {self.p_min}, attempt: {power}")

        self.power = power

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
    def __init__(self, fuel_amount, p_max):
        if p_max < 0:
            raise ValueError(f"Fuel cell must have positive max_power. attempt: {p_max}")

        if fuel_amount < 0:
            raise ValueError(f"Fuel amount is negative: {fuel_amount}")

        self.fuel_amount = fuel_amount
        self.p_min = 0
        self.p_max = p_max
        self.power = 0

    def __asdict__(self):
        d = {
            "fuel_amount": self.fuel_amount,
            "p_max": self.p_max
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
        )
        return output

    def set_power(self, power):
        if power > self.p_max or power < 0:
            raise ValueError(f"Trying to set fuel cell power out of bounds. p_max: {self.p_max}, attempt: {power}")

        self.power = power

    def max_power(self, n_steps):
        power = self.fuel_amount / n_steps

        if power > self.p_max:
            return self.p_max

        return power

    def min_power(self, n_steps):
        return 0

    def step(self):
        return self.power