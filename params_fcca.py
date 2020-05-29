"""Class definition for parameters used in FCCA MIP
"""
import json
import math
from typing import Dict, List

from params_env import ParamsEnv
from params_veh import ParamsVeh


class ParamsFCCA(ParamsEnv):
    """Parameters and parameter-calculating methods for FCAA MIP
    """
    veh_type_idx_list: List[int]
    veh_type_idx_dict: Dict[int, str]  # index -> v_type
    ring_idx_list: List[int]

    w_star: float    # w^*: optimal width of the circular trapezoid
    l_star: float    # l^*: optimal width of the approximated rectangle
    ring_count: int  # k:   potential number of rings
    gamma: float     # gamma: Newell and Daganzo(1986)'s first ring approx.
    total_customer: int     # e

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

        self.make_veh_type_idx_list()
        self.calc_w_star()
        self.calc_l_star()
        temp_ring_count = 5  # TODO: fixed temporarily for base case
        self.apply_ring_count(temp_ring_count)
        self.calc_gamma()
        self.calc_total_customer()

    def make_veh_type_idx_list(self):
        """make list of index & dictionary for v_type
        """
        self.veh_type_idx_list = list()
        self.veh_type_idx_dict = dict()
        for idx, v_type in enumerate(self.vehicle_types):
            self.veh_type_idx_list.append(idx)
            self.veh_type_idx_dict[idx] = v_type

    def make_idx_list_without_dummy(self) -> List[int]:
        """
        Returns:
            List[int] -- v_type index without dummy
        """
        return_list: List[int] = list()
        for idx in self.veh_type_idx_list:
            v_type = self.veh_type_idx_dict[idx]
            if v_type != self.dummy_type:
                return_list.append(idx)
        return return_list

    def calc_w_star(self):
        """Calculate and set value of w_star member
        """
        self.w_star = math.sqrt(3/(2*self.c_density))

    def calc_l_star(self):
        """Calculate and set value of l_star member
        """
        _min_cap = min([v_ins.capacity for v_ins in self.veh_dict.values()])
        self.l_star = _min_cap / (math.sqrt(6*self.c_density))

    def apply_ring_count(self, ring_count: int):
        self.ring_count = ring_count
        self.ring_idx_list = [i for i in range(self.ring_count)]

    def calc_gamma(self):
        """Calculate and set value of gamma member
        """
        self.gamma = 0.95 * math.sqrt(27/(8*self.c_density))
        # the paper seems wrong: it used math.sqrt(3/(2*c_density))

    def calc_total_customer(self):
        """Calculate and set value of total_customer member
        """
        self.total_customer = \
            math.ceil(math.pi * self.radius * self.radius * self.c_density)

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

    def make_veh_capacity_dict(self) -> Dict[int, int]:
        """
        Returns:
            Dict[int, int] -- v_type_idx -> capacity
        """
        return {idx: self.veh_dict[v_type].capacity
                for idx, v_type in enumerate(self.vehicle_types)}

    def make_veh_fixed_cost_dict(self) -> Dict[int, float]:
        """
        Returns:
            Dict[int, int] -- v_type_idx -> fixed_cost
        """
        return {idx: self.veh_dict[v_type].fixed_cost
                for idx, v_type in enumerate(self.vehicle_types)}

    def make_veh_var_cost_dict(self) -> Dict[int, float]:
        """
        Returns:
            Dict[int, int] -- v_type_idx -> var_cost
        """
        return {idx: self.veh_dict[v_type].var_cost
                for idx, v_type in enumerate(self.vehicle_types)}


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
