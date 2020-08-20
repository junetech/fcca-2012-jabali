"""Class definition for vehicle parameters
"""
import json


class ParamsVeh:
    """Parameters loaded from params_veh_{v_type}.json
    """

    name: str
    capacity: int  # c_i
    fixed_cost: float  # f_i
    var_cost: float  # d_i: $/hour
    route_duration_limit: float  # t_i: hours

    # optional member
    availability: int  # z_i

    def init_from_json(self, filename: str, encoding: str):
        with open(filename, encoding=encoding) as f_data:
            _dict = json.load(f_data)
            for key, value in _dict.items():
                self.__dict__[key] = value

    def print_info(self):
        """terminal print of info
        """
        print(f"{self.__class__.__name__} instance contents")
        for key, value in self.__dict__.items():
            print(f"  {key}:\t{value}")


def main():
    """parameter load test
    """
    json_filename_list = [
        "params_veh_dummy.json",
        "params_veh_private.json",
        "params_veh_crowd.json",
    ]
    encoding = "utf-8"
    for json_filename in json_filename_list:
        params_veh = ParamsVeh()
        params_veh.init_from_json(json_filename, encoding)
        params_veh.print_info()


if __name__ == "__main__":
    main()
