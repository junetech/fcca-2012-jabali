from typing import List, Dict

from params_fcca import ParamsFCCA


class ResultFCCA:
    veh_type_list: List[str]
    actual_veh_type_list: List[str]
    ring_id_list: List[str]
    outer_ring_id_list: List[int]
    irid: str  # inner ring id

    o_coeff: float
    il_coeff: float
    it_coeff: float
    prod_coeff: float
    l_sq_coeff: float

    f_total: float
    l_total: float
    t_total: float
    cost_total: float

    def __init__(self, params: ParamsFCCA,
                 model_str: str):
        """
        Arguments:
            params {ParamsFCCA}
            model_str {str} -- one among MasterMetadata.model_selection
        """
        v_t = params.vehicle_types
        # variable value dictionary: v_type -> ring_id -> value
        self.n_dict: Dict[str, Dict[int, int]] = {v: {} for v in v_t}
        self.x_dict: Dict[str, Dict[int, int]] = {v: {} for v in v_t}
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
        self.ring_id_list = params.ring_id_list
        self.outer_ring_id_list = self.ring_id_list[1:]
        self.irid = self.ring_id_list[0]

    def define_coeffs(self, params: ParamsFCCA,
                      model_str: str):
        self.il_coeff = params.il_coeff
        self.it_coeff = params.it_coeff
        if model_str == "U1":
            self.o_coeff = params.ol_coeff
        if model_str == "L1":
            self.o_coeff = params.l1_o_coeff
            self.prod_coeff = 1 - (params.chi_p / params.alpha)/2
            self.l_sq_coeff = 1 + params.chi_p *(4/3)

    def calc_f_costs(self, params: ParamsFCCA):
        """fill in f_cost_dict

        Arguments:
            params {ParamsFCCA}
        """
        for i in self.veh_type_list:
            for j in self.outer_ring_id_list:
                self.f_cost_dict[i][j] = params.f_dict[i] * self.n_dict[i][j]
        for i in self.actual_veh_type_list:
            self.f_cost_dict[i][self.irid] = \
                params.f_dict[i] * self.n_dict[i][self.irid]
        self.f_total = 0.0
        for ring_dict in self.f_cost_dict.values():
            for f_cost in ring_dict.values():
                self.f_total += f_cost

    def calc_l_costs(self, params: ParamsFCCA,
                     model_str: str):
        if model_str == "U1":
            for i in self.veh_type_list:
                for j in self.outer_ring_id_list:
                    if self.x_dict[i][j] == 0:
                        continue
                    self.l_cost_dict[i][j] = self.y_dict[i][j] * self.y_dict[i][j]
                    self.l_cost_dict[i][j] += self.y_dict[i][j] * self.l_dict[i][j]
                    self.l_cost_dict[i][j] += self.l_dict[i][j] * self.l_dict[i][j] / 4
                    self.l_cost_dict[i][j] = self.l_cost_dict[i][j] * self.o_coeff * params.d_dict[i]
            for i in self.actual_veh_type_list:
                if self.x_dict[i][self.irid] == 0:
                    continue
                self.l_cost_dict[i][self.irid] = self.l_dict[i][self.irid] * self.l_dict[i][self.irid]
                self.l_cost_dict[i][self.irid] = self.l_cost_dict[i][self.irid] * self.il_coeff * params.d_dict[i]
        elif model_str == "L1":
            for i in self.veh_type_list:
                for j in self.outer_ring_id_list:
                    if self.x_dict[i][j] == 0:
                        continue
                    self.l_cost_dict[i][j] = self.y_dict[i][j] * self.y_dict[i][j]
                    self.l_cost_dict[i][j] += self.prod_coeff * self.y_dict[i][j] * self.l_dict[i][j]
                    self.l_cost_dict[i][j] += self.l_sq_coeff * self.l_dict[i][j] * self.l_dict[i][j] / 4
                    self.l_cost_dict[i][j] = self.l_cost_dict[i][j] * self.o_coeff * params.d_dict[i]
            for i in self.actual_veh_type_list:
                if self.x_dict[i][self.irid] == 0:
                    continue
                self.l_cost_dict[i][self.irid] = self.l_dict[i][self.irid] * self.l_dict[i][self.irid]
                self.l_cost_dict[i][self.irid] = self.l_cost_dict[i][self.irid] * self.il_coeff * params.d_dict[i]
        self.l_total = 0.0
        for ring_dict in self.l_cost_dict.values():
            for l_cost in ring_dict.values():
                self.l_total += l_cost

    def calc_t_costs(self, params: ParamsFCCA,
                     model_str: str):
        if model_str == "U1":
            for i in self.veh_type_list:
                for j in self.outer_ring_id_list:
                    if self.x_dict[i][j] == 0:
                        continue
                    self.t_cost_dict[i][j] = self.y_dict[i][j] * self.l_dict[i][j]
                    self.t_cost_dict[i][j] += self.l_dict[i][j] * self.l_dict[i][j] / 2
                    self.t_cost_dict[i][j] = self.t_cost_dict[i][j] * self.o_coeff * params.d_dict[i]
            for i in self.actual_veh_type_list:
                if self.x_dict[i][self.irid] == 0:
                    continue
                self.t_cost_dict[i][self.irid] = self.l_dict[i][self.irid] * self.l_dict[i][self.irid]
                self.t_cost_dict[i][self.irid] = self.t_cost_dict[i][self.irid] * self.it_coeff * params.d_dict[i]
        elif model_str == "L1":
            for i in self.veh_type_list:
                for j in self.outer_ring_id_list:
                    if self.x_dict[i][j] == 0:
                        continue
                    self.t_cost_dict[i][j] = self.prod_coeff * self.y_dict[i][j] * self.l_dict[i][j]
                    self.t_cost_dict[i][j] += self.l_sq_coeff * self.l_dict[i][j] * self.l_dict[i][j] / 2
                    self.t_cost_dict[i][j] = self.t_cost_dict[i][j] * self.o_coeff * params.d_dict[i]
            for i in self.actual_veh_type_list:
                if self.x_dict[i][self.irid] == 0:
                    continue
                self.t_cost_dict[i][self.irid] = self.l_dict[i][self.irid] * self.l_dict[i][self.irid]
                self.t_cost_dict[i][self.irid] = self.t_cost_dict[i][self.irid] * self.it_coeff * params.d_dict[i]
        self.t_total = 0.0
        for ring_dict in self.t_cost_dict.values():
            for t_cost in ring_dict.values():
                self.t_total += t_cost

    def calc_all_costs(self, params: ParamsFCCA,
                       model_str: str):
        self.calc_f_costs(params)
        self.calc_l_costs(params, model_str)
        self.calc_t_costs(params, model_str)

    def print_cost_info(self):
        from pprint import pprint
        pprint(self.f_cost_dict)
        pprint(self.l_cost_dict)
        pprint(self.t_cost_dict)
        self.cost_total = self.f_total + self.l_total + self.t_total
        f_share = round(self.f_total / self.cost_total * 100.0, 1)
        l_share = round(self.l_total / self.cost_total * 100.0, 1)
        t_share = round(self.t_total / self.cost_total * 100.0, 1)
        print_str = f"Total cost = {self.cost_total}; "
        print_str += "fixed : line-haul : transverse ="
        print_str += f" {f_share} : {l_share} : {t_share}"
        print(print_str)
