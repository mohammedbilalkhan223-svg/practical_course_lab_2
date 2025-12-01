from abc import ABC, abstractmethod
# import python's built-in package for creating abstract base classes (classes that cannot be instantiated directly)

class AbstractState(ABC):
# Define an abstract base class named AbstractState. Any class inheriting from this must implement the abstract methods
    @abstractmethod
    def set_power(self, p):
        pass
# Declares an abstract method set_power, which must be implemented by subclasses. Sets the device's power to p.
    @abstractmethod
    def max_power(self, duration):
        pass
# Abstract method that must return the maximum allowable power over given duration.
    @abstractmethod
    def min_power(self, duration):
        pass
# Abstract method for returning the minimum allowable power.
    @abstractmethod
    def step(self):
        pass
# Abstract method that must update the device’s state for one timestep.
class AbstractDevice(ABC):
# Defines another abstract base class for all device objects (e.g., battery, fuel cell, load)
    @abstractmethod
    def set_output_power(self, power):
        pass
# Must be implemented to set the power output for the device.
    @abstractmethod
    def cost(self, energy):
        pass
# Must compute and return the cost for using energy over one timestep.
    @abstractmethod
    def step(self):
        pass
# Must advance the device by one timestep and return the energy actually used.