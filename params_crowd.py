"""Class definition for crowdsourced driver pool parameters
"""
import json
from typing import List, Iterator
from copy import deepcopy
import math  # math.inf is used

from params_veh import ParamsVeh


class ParamsCrowd:
    """Parameters loaded from params_crowd.json
    """

    name_prefix: str
    suffix_idx_start: int
    price_diff: bool
    capacity: int  # c_w
    fixed_cost: float  # f_w
    route_duration_limit: float  # t_w: hours
    var_cost_base: float  # g_w0
    availability_base: int  # z_w0
    var_cost_increment: List[float]
    availability_increment: List[int]

    def __init__(self, filename: str, encoding: str):
        with open(filename, encoding=encoding) as f_data:
            _dict = json.load(f_data)
            for key, value in _dict.items():
                self.__dict__[key] = value
        self.validity_test()

    def validity_test(self):
        if len(self.var_cost_increment) != 1 + len(
            self.availability_increment
        ):
            _str = "The last availability increment is defined as infinite:\n"
            _str += " len(var_cost_increment) == 1 + len(availability_gain)"
            _str += " should hold"
            raise ValueError(_str)
        for cost_inc in self.var_cost_increment:
            if cost_inc <= 0.0:
                _str = "All values in var_cost_increment "
                _str += f"{self.var_cost_increment} should be positive"
                raise ValueError(_str)
        for av_inc in self.availability_increment:
            if av_inc <= 0:
                _str = "All values in availability_increment "
                _str += f"{self.availability_increment} should be an positive integer"
                raise ValueError(_str)

    def print_info(self):
        """terminal print of info
        """
        print(f"{self.__class__.__name__} instance contents")
        for key, value in self.__dict__.items():
            print(f"  {key}:\t{value}")

    def generate_veh_names(self) -> Iterator[str]:
        yield self.name_prefix + str(self.suffix_idx_start)
        name_idx = self.suffix_idx_start
        for _ in self.var_cost_increment:
            name_idx += 1
            yield self.name_prefix + str(name_idx)

    def generate_params_veh(self) -> Iterator[ParamsVeh]:
        veh_name = self.generate_veh_names()
        # the base crowdsourced vehicle
        base_veh = ParamsVeh()
        base_veh.name = next(veh_name)
        base_veh.capacity = self.capacity
        base_veh.fixed_cost = self.fixed_cost
        base_veh.route_duration_limit = self.route_duration_limit
        base_veh.var_cost = self.var_cost_base
        base_veh.availability = self.availability_base
        yield base_veh

        # incremental vehicles
        var_add = 0.0
        if not self.price_diff:
            av_add = 0
        for var_inc, av_inc in zip(
            self.var_cost_increment, self.availability_increment
        ):
            var_add += var_inc
            inc_veh = deepcopy(base_veh)
            inc_veh.name = next(veh_name)
            inc_veh.var_cost += var_add
            if self.price_diff:
                inc_veh.availability = av_inc
            else:
                av_add += av_inc
                inc_veh.availability += av_add
            yield inc_veh

        # last vehicles of infinite availability
        var_add += self.var_cost_increment[-1]
        inf_veh = deepcopy(base_veh)
        inf_veh.name = next(veh_name)
        inf_veh.var_cost += var_add
        inf_veh.availability = math.inf
        yield inf_veh


def main():
    """parameter load test
    """
    json_filename = "params_crowd.json"
    encoding = "utf-8"

    params_crowd = ParamsCrowd(json_filename, encoding)
    params_crowd.print_info()
    for veh in params_crowd.generate_params_veh():
        veh.print_info()


if __name__ == "__main__":
    main()
