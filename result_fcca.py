import math
from typing import List, Dict, Any

from params_fcca import ParamsFCCA


class ResultFCCA:
    veh_type_list: List[str]
    actual_veh_type_list: List[str]
    ring_id_list: List[int]
    outer_ring_id_list: List[int]
    irid: int  # inner ring id

    ol_coeff: float
    ot_coeff: float
    il_coeff: float
    it_coeff: float
    prod_coeff: float
    l_sq_coeff: float

    solver_obj: float
    f_total: float
    l_total: float
    t_total: float
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
        self.l_cost_dict: Dict[str, Dict[int, float]] = {v: {} for v in v_t}
        self.t_cost_dict: Dict[str, Dict[int, float]] = {v: {} for v in v_t}

        self.define_lists(params)
        self.define_coeffs(params, model_str)

    def define_lists(self, params: ParamsFCCA):
        self.veh_type_list = params.vehicle_types
        self.actual_veh_type_list = params.actual_veh_type_list
        self.ring_id_list = [i for i in params.ring_id_list]
        self.outer_ring_id_list = params.ring_id_list[1:]
        self.irid = params.ring_id_list[0]

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

    def calc_f_costs(self, params: ParamsFCCA):
        """fill in f_cost_dict

        Args:
            params (ParamsFCCA)
        """
        for i in self.actual_veh_type_list:
            for j in self.outer_ring_id_list:
                self.f_cost_dict[i][j] = params.f_dict[i] * self.n_dict[i][j]
        for i in self.actual_veh_type_list:
            self.f_cost_dict[i][self.irid] = (
                params.f_dict[i] * self.n_dict[i][self.irid]
            )
        self.f_total = 0.0
        for ring_dict in self.f_cost_dict.values():
            for f_cost in ring_dict.values():
                self.f_total += f_cost

    def calc_l_costs(self, params: ParamsFCCA, model_str: str):
        if model_str == "U1":
            for i in self.actual_veh_type_list:
                for j in self.outer_ring_id_list:
                    if self.x_dict[i][j] == 0:
                        continue
                    self.l_cost_dict[i][j] = (
                        self.y_dict[i][j] * self.n_dict[i][j]
                    )
                    self.l_cost_dict[i][j] += (
                        self.l_dict[i][j] * self.n_dict[i][j] / 2
                    )
                    self.l_cost_dict[i][j] = (
                        self.l_cost_dict[i][j]
                        * self.ol_coeff
                        * params.g_dict[i]
                    )
            for i in self.actual_veh_type_list:
                if self.x_dict[i][self.irid] == 0:
                    continue
                self.l_cost_dict[i][self.irid] = (
                    self.l_dict[i][self.irid] * self.n_dict[i][self.irid]
                )
                self.l_cost_dict[i][self.irid] = (
                    self.l_cost_dict[i][self.irid]
                    * self.il_coeff
                    * params.g_dict[i]
                )
        elif model_str == "L1":
            for i in self.actual_veh_type_list:
                for j in self.outer_ring_id_list:
                    if self.x_dict[i][j] == 0:
                        continue
                    self.l_cost_dict[i][j] = (
                        self.y_dict[i][j] * self.y_dict[i][j]
                    )
                    self.l_cost_dict[i][j] += (
                        self.prod_coeff * self.y_dict[i][j] * self.l_dict[i][j]
                    )
                    self.l_cost_dict[i][j] += (
                        self.l_sq_coeff
                        * self.l_dict[i][j]
                        * self.l_dict[i][j]
                        / 4
                    )
                    self.l_cost_dict[i][j] = (
                        self.l_cost_dict[i][j]
                        * self.o_coeff
                        * params.g_dict[i]
                    )
            for i in self.actual_veh_type_list:
                if self.x_dict[i][self.irid] == 0:
                    continue
                self.l_cost_dict[i][self.irid] = (
                    self.l_dict[i][self.irid] * self.l_dict[i][self.irid]
                )
                self.l_cost_dict[i][self.irid] = (
                    self.l_cost_dict[i][self.irid]
                    * self.il_coeff
                    * params.g_dict[i]
                )
        self.l_total = 0.0
        for ring_dict in self.l_cost_dict.values():
            for l_cost in ring_dict.values():
                self.l_total += l_cost

    def calc_t_costs(self, params: ParamsFCCA, model_str: str):
        if model_str == "U1":
            for i in self.actual_veh_type_list:
                for j in self.outer_ring_id_list:
                    if self.x_dict[i][j] == 0:
                        continue
                    self.t_cost_dict[i][j] = (
                        self.l_dict[i][j] * self.n_dict[i][j]
                    )
                    self.t_cost_dict[i][j] = (
                        self.t_cost_dict[i][j]
                        * self.ot_coeff
                        * params.g_dict[i]
                    )
            for i in self.actual_veh_type_list:
                if self.x_dict[i][self.irid] == 0:
                    continue
                self.t_cost_dict[i][self.irid] = (
                    self.l_dict[i][self.irid] * self.n_dict[i][self.irid]
                )
                self.t_cost_dict[i][self.irid] = (
                    self.t_cost_dict[i][self.irid]
                    * self.it_coeff
                    * params.g_dict[i]
                )
        elif model_str == "L1":
            for i in self.actual_veh_type_list:
                for j in self.outer_ring_id_list:
                    if self.x_dict[i][j] == 0:
                        continue
                    self.t_cost_dict[i][j] = (
                        self.prod_coeff * self.y_dict[i][j] * self.l_dict[i][j]
                    )
                    self.t_cost_dict[i][j] += (
                        self.l_sq_coeff
                        * self.l_dict[i][j]
                        * self.l_dict[i][j]
                        / 2
                    )
                    self.t_cost_dict[i][j] = (
                        self.t_cost_dict[i][j]
                        * self.o_coeff
                        * params.g_dict[i]
                    )
            for i in self.actual_veh_type_list:
                if self.x_dict[i][self.irid] == 0:
                    continue
                self.t_cost_dict[i][self.irid] = (
                    self.l_dict[i][self.irid] * self.l_dict[i][self.irid]
                )
                self.t_cost_dict[i][self.irid] = (
                    self.t_cost_dict[i][self.irid]
                    * self.it_coeff
                    * params.g_dict[i]
                )
        self.t_total = 0.0
        for ring_dict in self.t_cost_dict.values():
            for t_cost in ring_dict.values():
                self.t_total += t_cost

    def calc_all_costs(self, params: ParamsFCCA, model_str: str):
        self.calc_f_costs(params)
        self.calc_l_costs(params, model_str)
        self.calc_t_costs(params, model_str)
        self.cost_total = self.f_total + self.l_total + self.t_total

    def print_variable_info(self):
        for i in self.veh_type_list:
            if self.u_dict[i] == 0:
                continue
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
            key: value for key, value in self.l_cost_dict.items() if value
        }
        pprint(_l_dict)
        print("Variable costs for transverse distance")
        _t_dict = {
            key: value for key, value in self.t_cost_dict.items() if value
        }
        pprint(_t_dict)
        f_share = round(self.f_total / self.cost_total * 100.0, 1)
        l_share = round(self.l_total / self.cost_total * 100.0, 1)
        t_share = round(self.t_total / self.cost_total * 100.0, 1)
        print_str = f"Total cost = {self.cost_total}; "
        print_str += "fixed : line-haul : transverse ="
        print_str += f" {f_share} : {l_share} : {t_share}"
        print(print_str)

    def variable_to_csv(self, csv_path: str):
        import csv

        with open(csv_path, "w", newline="") as _file:
            fieldname_list = ["V"]
            fieldname_list.extend(self.veh_type_list)
            file_writer = csv.DictWriter(_file, fieldnames=fieldname_list)
            file_writer.writeheader()
            _row: Dict[str, Any] = {"V": "u"}
            _row.update(self.u_dict)
            file_writer.writerow(_row)
            for j in self.ring_id_list:
                _row = {"V": f"x_*{j}"}
                for i in self.veh_type_list:
                    _row[i] = self.x_dict[i][j]
                file_writer.writerow(_row)
            for j in self.ring_id_list:
                _row = {"V": f"n_*{j}"}
                for i in self.veh_type_list:
                    _row[i] = self.n_dict[i][j]
                file_writer.writerow(_row)
            for j in self.ring_id_list:
                _row = {"V": f"y_*{j}"}
                for i in self.veh_type_list:
                    _row[i] = self.y_dict[i][j]
                file_writer.writerow(_row)
            for j in self.ring_id_list:
                _row = {"V": f"l_*{j}"}
                for i in self.veh_type_list:
                    _row[i] = self.l_dict[i][j]
                file_writer.writerow(_row)


def test_leq(lhs, rhs, abs_tol, err_str):
    print(f"{lhs} <= {rhs}", err_str)
    assert lhs <= rhs + abs_tol, err_str


def test_eq(lhs, rhs, abs_tol, err_str):
    print(f"{lhs} == {rhs}", err_str)
    assert math.isclose(lhs, rhs, rel_tol=0.0, abs_tol=abs_tol), err_str


def test_constraints(params: ParamsFCCA, result: ResultFCCA):
    """Test objective & constraints

    Args:
        params (ParamsFCCA)
        result (ResultFCCA): after calc_all_costs
    """
    print("Starting validating a solution of MIP model")
    print("Objective function value")
    result.cost_total = result.cost_total
    _str = f"Objective value equality"
    test_eq(result.solver_obj, result.cost_total, params.abs_tol, _str)

    # local aliases
    veh_type_list: List[str] = params.vehicle_types
    actual_veh_type_list: List[str] = params.actual_veh_type_list
    outer_ring_id_list: List[int] = params.outer_ring_id_list
    inner_ring_id = params.inner_ring_id
    c_dict = params.c_dict
    f_dict = params.f_dict
    g_dict = params.g_dict
    t_dict = params.t_dict
    delta = params.c_density
    total_c = params.total_customer
    u = result.u_dict
    x = result.x_dict
    n = result.n_dict
    y = result.y_dict
    l = result.l_dict

    lhs: Any
    rhs: Any
    # Constraints
    print("(3) minimum number of vehicles constrants")
    for i in veh_type_list:
        for j in outer_ring_id_list:
            constr_n = f"MinNumberVtype_{i}_OuterRing_{j}"
            lhs = (math.pi / params.w_star) * (y[i][j] + l[i][j] / 2)
            rhs = n[i][j]
            test_leq(lhs, rhs, params.abs_tol, constr_n)

    print("(4) each ring serviced by a single vehicle type")
    for j in params.ring_id_list:
        constr_n = f"OnlyOneVtypeOuterRing_{j}"
        lhs = sum(x[i][j] for i in veh_type_list)
        rhs = 1
        test_eq(lhs, rhs, params.abs_tol, constr_n)

    print("(5), (6), (7) big-M constraints")
    big_M_veh_count = total_c
    big_M_radius = params.radius
    for i in veh_type_list:
        for j in params.ring_id_list:
            lhs = n[i][j]
            rhs = big_M_veh_count * x[i][j]
            test_leq(lhs, rhs, params.abs_tol, f"bigM_n_{i}_{j}")
            lhs = y[i][j]
            rhs = math.ceil(big_M_radius * x[i][j])
            test_leq(lhs, rhs, params.abs_tol, f"bigM_y_{i}_{j}")
            lhs = l[i][j]
            rhs = math.ceil(big_M_radius * x[i][j])
            test_leq(lhs, rhs, params.abs_tol, f"bigM_l_{i}_{j}")

    print("(8) minimum distance(the depot -> the inner edge of ring j)")
    for i in veh_type_list:
        for j in outer_ring_id_list:
            constr_n = f"MinDistanceVtype_{i}_OuterRing_{j}"
            lhs = sum(y[m][j - 1] + l[m][j - 1] for m in veh_type_list)
            lhs += big_M_radius * (x[i][j] - 1)
            rhs = y[i][j]
            test_leq(lhs, rhs, params.abs_tol, constr_n)

    print("(9) outer ring capacity constraints")
    for i in veh_type_list:
        for j in outer_ring_id_list:
            constr_n = f"CapacityVtype_{i}_OuterRing_{j}"
            lhs = 2 * delta * params.w_star * l[i][j]
            rhs = c_dict[i]
            test_leq(lhs, rhs, params.abs_tol, constr_n)

    print("(10) duration limit constraints of outer rings")
    for j in outer_ring_id_list:
        for i in actual_veh_type_list:
            constr_n = f"DurationLimitVtype_{i}_OuterRing_{j}"
            rhs = t_dict[i] * params.speed / 2
            lhs = y[i][j] + l[i][j] / 2
            lhs += delta * params.w_star * l[i][j] * math.sqrt(2 / (3 * delta))
            lhs += (
                delta
                * params.w_star
                * l[i][j]
                * params.service_time
                * params.speed
            )
            test_leq(lhs, rhs, params.abs_tol, constr_n)

    print("(12) inner ring not serviced by a dummy vehicle")
    constr_n = "NoDummyVtypeInnerRing"
    lhs = x[params.dummy_type][inner_ring_id]
    rhs = 0
    test_eq(lhs, rhs, params.abs_tol, constr_n)

    print("(13) minimum number of vehicles constrants")
    for i in actual_veh_type_list:
        constr_n = f"MinNumberVtype_{i}_InnerRing"
        lhs = math.pi * l[i][inner_ring_id] / params.gamma
        rhs = n[i][inner_ring_id]
        test_leq(lhs, rhs, params.abs_tol, constr_n)

    print("(14) inner ring capacity constraints for each zone")
    for i in actual_veh_type_list:
        constr_n = f"CapacityVtype_{i}_InnerRing"
        lhs = delta * params.gamma * l[i][inner_ring_id]
        rhs = c_dict[i] * x[i][inner_ring_id]
        test_leq(lhs, rhs, params.abs_tol, constr_n)

    print("(16) the entire region is serviced")
    constr_n = "EntireRegionServed"
    lhs = sum(sum(l[i][j] for j in params.ring_id_list) for i in veh_type_list)
    rhs = params.radius
    test_eq(lhs, rhs, params.abs_tol, constr_n)

    print("(17) total capacity is enough for all customers")
    constr_n = "TotalCapacityEnough"
    lhs = total_c
    rhs = sum(
        sum(c_dict[i] * n[i][j] for j in params.ring_id_list)
        for i in veh_type_list
    )
    test_leq(lhs, rhs, params.abs_tol, constr_n)

    # TODO: print("number of available private vehicles")

    print("number of available crowdsourced vehicles")
    for i in params.crowd_veh_types:
        z_s = params.veh_dict[i].availability
        constr_n = f"VehicleAvailability{i}"
        if z_s is math.inf:
            continue
        lhs = sum(n[i][j] for j in params.ring_id_list)
        test_leq(lhs, z_s, params.abs_tol, constr_n)

    print("u[i] definition")
    for i in params.actual_veh_type_list:
        constr_n = f"{i}Used"
        lhs = sum(l[i][j] for j in params.ring_id_list) / params.radius
        rhs = u[i]
        test_leq(lhs, rhs, params.abs_tol, constr_n)

    if not params.price_diff:
        print("Only one type of crowdsourced vehicles are available")
        constr_n = "OnlyOneCrowdTypeUsed"
        lhs = sum(u[i] for i in params.crowd_veh_types)
        rhs = 1
        test_leq(lhs, rhs, params.abs_tol, constr_n)
