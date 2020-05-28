"""Class definition for parameters used in FCCA MIP
"""
import json
import math
from typing import Dict

from params_env import ParamsEnv
from params_veh import ParamsVeh


class ParamsFCCA(ParamsEnv):
    """Parameters and parameter-calculating methods for FCAA MIP
    """
    w_star: float    # w^*: optimal width of the circular trapezoid
    l_star: float    # l^*: optimal width of the approximated rectangle
    ring_count: int  # k:   potential number of rings

    def __init__(self, env_filename: str,
                 veh_file_postfix: str,
                 veh_file_ext: str,
                 encoding: str):
        with open(env_filename, encoding=encoding) as e_data:
            _dict = json.load(e_data)
            for key, value in _dict.items():
                self.__dict__[key] = value
        self.amend_service_time()

        self.veh_dict: Dict[str, ParamsVeh] = dict()
        for v_type in self.vehicle_types:
            veh_filename = veh_file_postfix + v_type + veh_file_ext
            self.veh_dict[v_type] = ParamsVeh(veh_filename, encoding)

        self.calc_w_star()
        self.calc_l_star()
        self.ring_count = 5  # TODO: fixed temporarily for base case

    def print_info(self):
        """terminal print of info
        """
        print(f"{self.__class__.__name__} instance contents")
        print_keys = [key for key in self.__dict__ if key != "veh_dict"]
        for key in print_keys:
            value = self.__dict__[key]
            print(f"  {key}:\t{value}")
        print("  veh_dict contents below")
        for v_type in self.vehicle_types:
            v_ins = self.veh_dict[v_type]
            v_ins.print_info()

    def calc_w_star(self):
        """Calculate and set value of w_star member
        """
        self.w_star = math.sqrt(3/(2*self.c_density))

    def calc_l_star(self):
        """Calculate and set value of l_star member
        """
        _min_cap = min([v_ins.capacity for v_ins in self.veh_dict.values()])
        self.l_star = _min_cap / (math.sqrt(6*self.c_density))


def main():
    """parameter load test
    """
    env_json_filename = "params_env.json"
    veh_file_postfix = "params_veh_"
    veh_file_ext = ".json"
    encoding = "utf-8"

    fcca_params = ParamsFCCA(env_json_filename,
                             veh_file_postfix,
                             veh_file_ext,
                             encoding)
    fcca_params.print_info()


if __name__ == "__main__":
    main()
