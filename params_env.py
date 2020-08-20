"""Class definition for environment parameters
"""
import json
from typing import List


class ParamsEnv:
    """Parameters loaded from params_env.json
    """

    # given parameters
    description: str
    radius: float  # r
    speed: float  # v
    c_density: float  # \delta
    service_time: float  # \mu
    service_time_unit: str
    vehicle_types: List[str]  # Q = {0, ..., q}
    dummy_type: str
    use_params_crowd: bool
    params_crowd_filename: str
    max_ring_count: int
    gurobi_output: bool

    def __init__(self, filename: str, encoding: str):
        with open(filename, encoding=encoding) as f_data:
            _dict = json.load(f_data)
            for key, value in _dict.items():
                self.__dict__[key] = value

    def amend_time_unit(self):
        """Set service_time value according to service_time_unit
        """
        if self.service_time_unit == "hours":
            pass
        elif self.service_time_unit == "minutes":
            self.service_time = self.service_time / 60.0
        elif self.service_time_unit == "seconds":
            self.service_time = self.service_time / 3600.0

    def print_info(self):
        """terminal print of info
        """
        print(f"{self.__class__.__name__} instance contents")
        for key, value in self.__dict__.items():
            print(f"  {key}:\t{value}")


def main():
    """parameter load test
    """
    json_filename = "params_env.json"
    encoding = "utf-8"
    params_env = ParamsEnv(json_filename, encoding)
    params_env.print_info()


if __name__ == "__main__":
    main()
