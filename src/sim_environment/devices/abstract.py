from abc import ABC, abstractmethod


class AbstractState(ABC):
    @abstractmethod
    def set_power(self, p):
        pass

    @abstractmethod
    def max_power(self, duration):
        pass

    @abstractmethod
    def min_power(self, duration):
        pass

    @abstractmethod
    def step(self):
        pass

class AbstractDevice(ABC):
    @abstractmethod
    def set_output_power(self, power):
        pass

    @abstractmethod
    def cost(self, energy):
        pass

    @abstractmethod
    def step(self):
        pass