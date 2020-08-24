import math
import csv
from typing import List, Dict, Any

from params_fcca import ParamsFCCA


class ResultFCCA:
    veh_type_list: List[str]
    actual_veh_type_list: List[str]
    ring_id_list: List[int]
    outer_ring_id_list: List[int]
    irid: int  # inner ring id
    ir_type: str
    private_type_id: str
    crowd_type_id: str

    ol_coeff: float
    ot_coeff: float
    il_coeff: float
    it_coeff: float
    prod_coeff: float
    l_sq_coeff: float

    solver_obj: float
    cost_total: float

    def __init__(self, params: ParamsFCCA, model_str: str):
        """
        Args:
            params (ParamsFCCA)
            model_str (str): one among MasterMetadata.model_selection
        """
        v_t = params.vehicle_types
        # variable value dictionary
        # v_type -> value
        self.u_dict: Dict[str, int] = dict()
        # v_type -> ring_id -> value
        self.x_dict: Dict[str, Dict[int, int]] = {v: {} for v in v_t}
        self.n_dict: Dict[str, Dict[int, int]] = {v: {} for v in v_t}
        self.l_dict: Dict[str, Dict[int, float]] = {v: {} for v in v_t}
        self.y_dict: Dict[str, Dict[int, float]] = {v: {} for v in v_t}
        # cost dictionary: v_type -> ring_id -> value
        self.f_cost_dict: Dict[str, Dict[int, float]] = {v: {} for v in v_t}
        self.li_cost_dict: Dict[str, Dict[int, float]] = {v: {} for v in v_t}
        self.lo_cost_dict: Dict[str, Dict[int, float]] = {v: {} for v in v_t}

        self.define_coeffs(params, model_str)

    def define_coeffs(self, params: ParamsFCCA, model_str: str):
        self.il_coeff = params.il_coeff
        self.it_coeff = params.it_coeff
        if model_str == "U1":
            self.ol_coeff = params.ol_coeff
            self.ot_coeff = params.ot_coeff
        if model_str == "L1":
            self.o_coeff = params.l1_o_coeff
            self.prod_coeff = 1 - (params.chi_p / params.alpha) / 2
            self.l_sq_coeff = 1 + params.chi_p * (4 / 3)

    def fill_from_model(
        self, obj_val: float, v_dict: Dict[str, Any], params: ParamsFCCA
    ):
        """
        Args:
            obj_val (float): model.objVal
            v_dict (Dict[str, Any]): from Model instance after optimize()
        """
        self.solver_obj = obj_val
        for i in params.vehicle_types:
            u_name = params.u_name_dict[i]
            self.u_dict[i] = round(v_dict[u_name])
            for j in params.ring_id_list:
                x_name = params.x_name_dict[i][j]
                n_name = params.n_name_dict[i][j]
                y_name = params.y_name_dict[i][j]
                l_name = params.l_name_dict[i][j]
                self.x_dict[i][j] = round(v_dict[x_name])
                self.n_dict[i][j] = round(v_dict[n_name])
                self.y_dict[i][j] = v_dict[y_name]
                self.l_dict[i][j] = v_dict[l_name]

        self.init_info_dict(params)

    def init_info_dict(self, params: ParamsFCCA):
        # ring_id -> value
        # vehicle type of a ring <- define_active_ring
        self.v_type_ring: Dict[int, str] = dict()
        # inner radius of a ring <- define_active_ring
        self.inner_rad_ring: Dict[int, float] = dict()
        # length of a ring <- define_active_ring
        self.length_ring: Dict[int, float] = dict()
        # number of used vehicles in a ring <- define_active_ring
        self.veh_count_ring: Dict[int, int] = dict()
        # customers of a partition in ring <- calc_customer_count
        self.c_count_part: Dict[int, float] = dict()
        # customers per vehicle in ring <- calc_customer_count
        self.c_per_v_part: Dict[int, float] = dict()
        # line-haul time of a partition in ring <- calc_li_costs
        self.li_time_part: Dict[int, float] = dict()
        # local time of a partition in ring <- calc_lo_costs
        self.lo_time_part: Dict[int, float] = dict()
        # travel time of a partition in ring
        self.total_time_part: Dict[int, float] = dict()

        self.define_active_ring(params)

        # actual_veh_type -> value
        # number of used vehicles of type <- calc_customer_count
        self.veh_count_veh: Dict[str, int] = {i: 0 for i in self.veh_type_list}
        # total customers of type <- calc_customer_count
        self.c_count_veh: Dict[str, float] = {
            i: 0.0 for i in self.veh_type_list
        }
        # customers per vehicle of type <- calc_customer_count
        self.c_per_v_veh: Dict[str, float] = dict()
        # total line-haul time of type <- calc_li_costs
        self.li_time_veh: Dict[str, float] = {
            i: 0.0 for i in self.veh_type_list
        }
        # total local travel time of type <- calc_lo_costs
        self.lo_time_veh: Dict[str, float] = {
            i: 0.0 for i in self.veh_type_list
        }
        # total travel time of type <- calc_total_veh_time
        self.total_time_veh: Dict[str, float] = {
            i: 0.0 for i in self.veh_type_list
        }
        # total variable cost of type <- calc_total_veh_time
        self.var_cost_veh: Dict[str, float] = {
            i: 0.0 for i in self.veh_type_list
        }
        # total fixed cost of type <- calc_f_costs
        self.fixed_cost_veh: Dict[str, float] = {
            i: 0.0 for i in self.veh_type_list
        }

    def define_active_ring(self, params: ParamsFCCA):
        """define active ring id list
        should be called after self.fill_from_model method
        Args:
            params (ParamsFCCA)
        """
        self.veh_type_list = params.actual_veh_type_list
        self.irid = params.ring_id_list[0]
        self.outer_ring_id_list = list()
        for i in self.veh_type_list:
            if self.x_dict[i][self.irid] == 1:
                self.ir_type = i
                self.v_type_ring[self.irid] = i
                self.inner_rad_ring[self.irid] = self.l_dict[i][self.irid]
                self.length_ring[self.irid] = 0.0
                self.veh_count_ring[self.irid] = self.n_dict[i][self.irid]
                if i in params.private_veh_types:
                    self.private_type_id = i
                else:
                    self.crowd_type_id = params.crowd_type_id
                    self.crowd_type_id += str(params.g_dict[i])
                break

        for outer_ring_id in params.ring_id_list[1:]:
            for i in self.veh_type_list:
                if self.x_dict[i][outer_ring_id] == 1:
                    self.outer_ring_id_list.append(outer_ring_id)
                    self.v_type_ring[outer_ring_id] = i
                    self.inner_rad_ring[outer_ring_id] = self.y_dict[i][
                        outer_ring_id
                    ]
                    self.length_ring[outer_ring_id] = self.l_dict[i][
                        outer_ring_id
                    ]
                    self.veh_count_ring[outer_ring_id] = self.n_dict[i][
                        outer_ring_id
                    ]
                    if i in params.private_veh_types:
                        self.private_type_id = i
                    else:
                        self.crowd_type_id = params.crowd_type_id
                        self.crowd_type_id += str(params.g_dict[i])
                    break
        self.ring_id_list = [self.irid]
        self.ring_id_list.extend(self.outer_ring_id_list)

    def calc_f_costs(self, f_dict: Dict[str, float]):
        """fill in f_cost_dict

        Args:
            f_dict (Dict[str, float]): fixed cost dictionary
        """
        for i in self.veh_type_list:
            for j in self.ring_id_list:
                self.f_cost_dict[i][j] = f_dict[i] * self.n_dict[i][j]
                self.fixed_cost_veh[i] += self.f_cost_dict[i][j]

    def calc_li_costs(self, g_dict: Dict[str, float], model_str: str):
        if model_str == "U1":
            # outer rings
            for i in self.veh_type_list:
                for j in self.outer_ring_id_list:
                    if self.x_dict[i][j] == 0:
                        continue
                    _li_time = self.y_dict[i][j] * self.n_dict[i][j]
                    _li_time += self.l_dict[i][j] * self.n_dict[i][j] / 2
                    _li_time = _li_time * self.ol_coeff
                    self.li_cost_dict[i][j] = _li_time * g_dict[i]
                    self.li_time_part[j] = _li_time
                    self.li_time_veh[i] += _li_time
            # inner circle
            i = self.ir_type
            _li_time = self.l_dict[i][self.irid] * self.n_dict[i][self.irid]
            _li_time = _li_time * self.il_coeff
            self.li_cost_dict[i][self.irid] = _li_time * g_dict[i]
            self.li_time_part[self.irid] = _li_time
            self.li_time_veh[i] += _li_time
        elif model_str == "L1":  # TODO: 나중에
            for i in self.veh_type_list:
                for j in self.outer_ring_id_list:
                    if self.x_dict[i][j] == 0:
                        continue
                    self.li_cost_dict[i][j] = (
                        self.y_dict[i][j] * self.y_dict[i][j]
                    )
                    self.li_cost_dict[i][j] += (
                        self.prod_coeff * self.y_dict[i][j] * self.l_dict[i][j]
                    )
                    self.li_cost_dict[i][j] += (
                        self.l_sq_coeff
                        * self.l_dict[i][j]
                        * self.l_dict[i][j]
                        / 4
                    )
                    self.li_cost_dict[i][j] = (
                        self.li_cost_dict[i][j] * self.o_coeff * g_dict[i]
                    )
            for i in self.veh_type_list:
                if self.x_dict[i][self.irid] == 0:
                    continue
                self.li_cost_dict[i][self.irid] = (
                    self.l_dict[i][self.irid] * self.l_dict[i][self.irid]
                )
                self.li_cost_dict[i][self.irid] = (
                    self.li_cost_dict[i][self.irid] * self.il_coeff * g_dict[i]
                )

    def calc_lo_costs(self, g_dict: Dict[str, float], model_str: str):
        if model_str == "U1":
            for i in self.veh_type_list:
                for j in self.outer_ring_id_list:
                    if self.x_dict[i][j] == 0:
                        continue
                    _lo_time = self.l_dict[i][j] * self.n_dict[i][j]
                    _lo_time = _lo_time * self.ot_coeff
                    self.lo_cost_dict[i][j] = _lo_time * g_dict[i]
                    self.lo_time_part[j] = _lo_time
                    self.lo_time_veh[i] += _lo_time
            i = self.ir_type
            _lo_time = self.l_dict[i][self.irid] * self.n_dict[i][self.irid]
            _lo_time = _lo_time * self.it_coeff
            self.lo_cost_dict[i][self.irid] = _lo_time * g_dict[i]
            self.lo_time_part[self.irid] = _lo_time
            self.lo_time_veh[i] += _lo_time
        elif model_str == "L1":  # TODO: 나중에
            for i in self.veh_type_list:
                for j in self.outer_ring_id_list:
                    if self.x_dict[i][j] == 0:
                        continue
                    self.lo_cost_dict[i][j] = (
                        self.prod_coeff * self.y_dict[i][j] * self.l_dict[i][j]
                    )
                    self.lo_cost_dict[i][j] += (
                        self.l_sq_coeff
                        * self.l_dict[i][j]
                        * self.l_dict[i][j]
                        / 2
                    )
                    self.lo_cost_dict[i][j] = (
                        self.lo_cost_dict[i][j] * self.o_coeff * g_dict[i]
                    )
            for i in self.veh_type_list:
                if self.x_dict[i][self.irid] == 0:
                    continue
                self.lo_cost_dict[i][self.irid] = (
                    self.l_dict[i][self.irid] * self.l_dict[i][self.irid]
                )
                self.lo_cost_dict[i][self.irid] = (
                    self.lo_cost_dict[i][self.irid] * self.it_coeff * g_dict[i]
                )

    def calc_customer_count(self, c_density: float):
        for i in self.veh_type_list:
            for j in self.ring_id_list:
                if self.l_dict[i][j] == 0.0:
                    continue
                c_count = self.l_dict[i][j] + 2 * self.y_dict[i][j]
                c_count = c_count * self.l_dict[i][j] * math.pi * c_density
                self.c_count_part[j] = c_count
                self.c_per_v_part[j] = c_count / self.n_dict[i][j]
                self.veh_count_veh[i] += self.n_dict[i][j]
                self.c_count_veh[i] += c_count
            if self.veh_count_veh[i] == 0:
                continue
            self.c_per_v_veh[i] = self.c_count_veh[i] / self.veh_count_veh[i]

    def calc_total_ring_time(self):
        for j in self.ring_id_list:
            self.total_time_part[j] = (
                self.li_time_part[j] + self.lo_time_part[j]
            )

    def calc_total_veh_time(self, g_dict: Dict[str, float]):
        for i in self.veh_type_list:
            self.total_time_veh[i] = self.li_time_veh[i] + self.lo_time_veh[i]
            self.var_cost_veh[i] = self.total_time_veh[i] * g_dict[i]

    def calc_all_values(self, params: ParamsFCCA, model_str: str):
        self.calc_li_costs(params.g_dict, model_str)
        self.calc_lo_costs(params.g_dict, model_str)
        self.calc_f_costs(params.f_dict)
        self.cost_total = self.f_total() + self.li_total() + self.lo_total()
        self.calc_customer_count(params.c_density)
        self.calc_total_ring_time()
        self.calc_total_veh_time(params.g_dict)

    def f_total(self) -> float:
        _total = 0.0
        for ring_dict in self.f_cost_dict.values():
            for f_cost in ring_dict.values():
                _total += f_cost
        return _total

    def li_total(self) -> float:
        _total = 0.0
        for ring_dict in self.li_cost_dict.values():
            for li_cost in ring_dict.values():
                _total += li_cost
        return _total

    def lo_total(self) -> float:
        _total = 0.0
        for ring_dict in self.lo_cost_dict.values():
            for lo_cost in ring_dict.values():
                _total += lo_cost
        return _total

    def print_variable_info(self):
        for i in self.veh_type_list:
            for j in self.ring_id_list:
                if self.x_dict[i][j] == 1:
                    _str = f"x[{i}][{j}]: {self.x_dict[i][j]},"
                    _str += f" n[{i}][{j}]: {self.n_dict[i][j]},"
                    _str += f" y[{i}][{j}]: {self.y_dict[i][j]},"
                    _str += f" l[{i}][{j}]: {self.l_dict[i][j]}"
                    print(_str)

    def print_cost_info(self):
        from pprint import pprint

        print(f"Obj. value by solver = {self.solver_obj}")
        print("Fixed costs")
        _f_dict = dict()
        for v_type, ring_dict in self.f_cost_dict.items():
            v_dict = dict()
            for ring_id, f_cost in ring_dict.items():
                if f_cost > 0.0:
                    v_dict[ring_id] = f_cost
            if v_dict:
                _f_dict[v_type] = v_dict
        pprint(_f_dict)
        print("Variable costs for line-haul distance")
        _l_dict = {
            key: value for key, value in self.li_cost_dict.items() if value
        }
        pprint(_l_dict)
        print("Variable costs for transverse distance")
        _t_dict = {
            key: value for key, value in self.lo_cost_dict.items() if value
        }
        pprint(_t_dict)
        f_share = round(self.f_total() / self.cost_total * 100.0, 1)
        l_share = round(self.li_total() / self.cost_total * 100.0, 1)
        t_share = round(self.lo_total() / self.cost_total * 100.0, 1)
        print_str = f"Total cost = {self.cost_total}; "
        print_str += "fixed : line-haul : transverse ="
        print_str += f" {f_share} : {l_share} : {t_share}"
        print(print_str)

    def variable_to_csv(self, csv_path: str):
        with open(csv_path, "w", newline="") as _file:
            fieldname_list = ["V"]
            fieldname_list.extend(self.veh_type_list)
            _writer = csv.DictWriter(_file, fieldnames=fieldname_list)
            _writer.writeheader()
            _row: Dict[str, Any] = {"V": "u"}
            for i in self.veh_type_list:
                _row[i] = self.u_dict[i]
            _writer.writerow(_row)
            for j in self.ring_id_list:
                _row = {"V": f"x_*{j}"}
                for i in self.veh_type_list:
                    _row[i] = self.x_dict[i][j]
                _writer.writerow(_row)
            for j in self.ring_id_list:
                _row = {"V": f"n_*{j}"}
                for i in self.veh_type_list:
                    _row[i] = self.n_dict[i][j]
                _writer.writerow(_row)
            for j in self.ring_id_list:
                _row = {"V": f"y_*{j}"}
                for i in self.veh_type_list:
                    _row[i] = self.y_dict[i][j]
                _writer.writerow(_row)
            for j in self.ring_id_list:
                _row = {"V": f"l_*{j}"}
                for i in self.veh_type_list:
                    _row[i] = self.l_dict[i][j]
                _writer.writerow(_row)

    def vehicle_to_csv(self, params: ParamsFCCA, csv_path: str):
        keystone = "ringID"
        ring_id_idx_dict: Dict[int, int] = {
            ring_id: idx + 1 for idx, ring_id in enumerate(self.ring_id_list)
        }
        with open(csv_path, "w", newline="") as _file:
            fieldname_list: List[Any] = [keystone]
            for ring_id in self.ring_id_list:
                fieldname_list.append(ring_id_idx_dict[ring_id])
            _writer = csv.DictWriter(_file, fieldnames=fieldname_list)
            _writer.writeheader()
            _row1: Dict[Any, Any] = {
                keystone: f"{self.private_type_id}_c_per_v"
            }
            _row2: Dict[Any, Any] = {
                keystone: f"{self.private_type_id}_travel_time"
            }
            _row3: Dict[Any, Any] = {
                keystone: f"{self.private_type_id}_var_cost"
            }
            _row4: Dict[Any, Any] = {keystone: f"{self.crowd_type_id}_c_per_v"}
            _row5: Dict[Any, Any] = {
                keystone: f"{self.crowd_type_id}_travel_time"
            }
            _row6: Dict[Any, Any] = {
                keystone: f"{self.crowd_type_id}_var_cost"
            }
            for j in self.ring_id_list:
                dict_key = ring_id_idx_dict[j]
                v_type = self.v_type_ring[j]
                if v_type in params.private_veh_types:
                    _row1[dict_key] = self.c_per_v_part[j]
                    _row2[dict_key] = self.total_time_part[j]
                    _row3[dict_key] = (
                        self.total_time_part[j] * params.g_dict[v_type]
                    )
                    _row4[dict_key] = 0.0
                    _row5[dict_key] = 0.0
                    _row6[dict_key] = 0.0
                else:
                    _row1[dict_key] = 0.0
                    _row2[dict_key] = 0.0
                    _row3[dict_key] = 0.0
                    _row4[dict_key] = self.c_per_v_part[j]
                    _row5[dict_key] = self.total_time_part[j]
                    _row6[dict_key] = (
                        self.total_time_part[j] * params.g_dict[v_type]
                    )
            _writer.writerow(_row1)
            _writer.writerow(_row2)
            _writer.writerow(_row3)
            _writer.writerow(_row4)
            _writer.writerow(_row5)
            _writer.writerow(_row6)

    def ring_to_csv(self, params: ParamsFCCA, csv_path: str):
        keystone = "ringID"
        last_colhead = "VehTotal"
        ring_id_idx_dict: Dict[int, int] = {
            ring_id: idx + 1 for idx, ring_id in enumerate(self.ring_id_list)
        }
        with open(csv_path, "w", newline="") as _file:
            fieldname_list: List[Any] = [keystone]
            for ring_id in self.ring_id_list:
                fieldname_list.append(ring_id_idx_dict[ring_id])
            fieldname_list.append(last_colhead)
            _writer = csv.DictWriter(_file, fieldnames=fieldname_list)
            _writer.writeheader()
            _row1: Dict[Any, Any] = {
                keystone: f"{self.private_type_id}_t_customers"
            }
            _row2: Dict[Any, Any] = {
                keystone: f"{self.private_type_id}_t_veh_count"
            }
            _row3: Dict[Any, Any] = {
                keystone: f"{self.private_type_id}_t_travel_time"
            }
            _row4: Dict[Any, Any] = {
                keystone: f"{self.private_type_id}_t_var_cost"
            }
            _row5: Dict[Any, Any] = {
                keystone: f"{self.crowd_type_id}_t_customers"
            }
            _row6: Dict[Any, Any] = {
                keystone: f"{self.crowd_type_id}_t_veh_count"
            }
            _row7: Dict[Any, Any] = {
                keystone: f"{self.crowd_type_id}_t_travel_time"
            }
            _row8: Dict[Any, Any] = {
                keystone: f"{self.crowd_type_id}_t_var_cost"
            }
            for j in self.ring_id_list:
                dict_key = ring_id_idx_dict[j]
                v_type = self.v_type_ring[j]
                if v_type in params.private_veh_types:
                    _row1[dict_key] = self.c_count_part[j]
                    _row2[dict_key] = self.veh_count_ring[j]
                    _row3[dict_key] = self.total_time_part[j]
                    _row4[dict_key] = (
                        self.total_time_part[j] * params.g_dict[v_type]
                    )
                    _row5[dict_key] = 0.0
                    _row6[dict_key] = 0.0
                    _row7[dict_key] = 0.0
                    _row8[dict_key] = 0.0
                else:
                    _row1[dict_key] = 0.0
                    _row2[dict_key] = 0.0
                    _row3[dict_key] = 0.0
                    _row4[dict_key] = 0.0
                    _row5[dict_key] = self.c_count_part[j]
                    _row6[dict_key] = self.veh_count_ring[j]
                    _row7[dict_key] = self.total_time_part[j]
                    _row8[dict_key] = (
                        self.total_time_part[j] * params.g_dict[v_type]
                    )
            last_col = sum(_row1[j] for j in ring_id_idx_dict.values())
            _row1[last_colhead] = last_col
            _writer.writerow(_row1)
            last_col = sum(_row2[j] for j in ring_id_idx_dict.values())
            _row2[last_colhead] = last_col
            _writer.writerow(_row2)
            last_col = sum(_row3[j] for j in ring_id_idx_dict.values())
            _row3[last_colhead] = last_col
            _writer.writerow(_row3)
            last_col = sum(_row4[j] for j in ring_id_idx_dict.values())
            _row4[last_colhead] = last_col
            _writer.writerow(_row4)
            last_col = sum(_row5[j] for j in ring_id_idx_dict.values())
            _row5[last_colhead] = last_col
            _writer.writerow(_row5)
            last_col = sum(_row6[j] for j in ring_id_idx_dict.values())
            _row6[last_colhead] = last_col
            _writer.writerow(_row6)
            last_col = sum(_row7[j] for j in ring_id_idx_dict.values())
            _row7[last_colhead] = last_col
            _writer.writerow(_row7)
            last_col = sum(_row8[j] for j in ring_id_idx_dict.values())
            _row8[last_colhead] = last_col
            _writer.writerow(_row8)
