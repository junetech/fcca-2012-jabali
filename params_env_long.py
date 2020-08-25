"""Class definition for long_term environment parameters
"""
import json
from typing import List, Iterator

from params_fcca import ParamsFCCA


class ParamsEnvLong:
    """Parameters loaded from params_env_long*.json
    """

    # given parameters
    description: str
    radius: float  # r
    speed: float  # v
    c_density_list: List[float]  # \delta
    service_time: float  # \mu
    service_time_unit: str
    private_cost_per_day: float
    dummy_type: str
    private_veh_types: List[str]
    params_crowd_filename: str
    max_ring_count: int
    abs_tol: float

    def fill_from_json(self, filename: str, encoding: str):
        with open(filename, encoding=encoding) as f_data:
            _dict = json.load(f_data)
            for key, value in _dict.items():
                self.__dict__[key] = value
        self.vehicle_types = [self.dummy_type]
        self.vehicle_types.extend(self.private_veh_types)
        self.days: int = len(self.c_density_list)

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

    def generate_params_fcca(self) -> Iterator[ParamsFCCA]:
        for idx, c_density in enumerate(self.c_density_list):
            params = ParamsFCCA()
            params.description = self.description + "_" + str(idx + 1)
            params.radius = self.radius
            params.speed = self.speed
            params.service_time = self.service_time
            params.service_time_unit = self.service_time_unit
            params.dummy_type = self.dummy_type
            params.private_veh_types = self.private_veh_types.copy()
            params.use_params_crowd = True
            params.params_crowd_filename = self.params_crowd_filename
            params.max_ring_count = self.max_ring_count
            params.gurobi_output = False
            params.abs_tol = self.abs_tol
            params.vehicle_types = self.vehicle_types.copy()
            params.c_density = c_density
            yield params


def main():
    """parameter load test
    """
    json_filename = "params_env.json"
    encoding = "utf-8"
    params_env = ParamsEnvLong(json_filename, encoding)
    params_env.print_info()


if __name__ == "__main__":
    main()
