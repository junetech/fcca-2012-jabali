from typing import List, Dict


class ResultFCCA:
    def __init__(self, v_types: List[str]):
        """
        Arguments:
            v_types {List[str]} -- list of vehicle types
        """
        # variable value dictionary: v_type -> ring_id -> value
        self.n_dict: Dict[str, Dict[int, int]] = {v: {} for v in v_types}
        self.x_dict: Dict[str, Dict[int, int]] = {v: {} for v in v_types}
        self.l_dict: Dict[str, Dict[int, float]] = {v: {} for v in v_types}
        self.y_dict: Dict[str, Dict[int, float]] = {v: {} for v in v_types}
        # cost dictionary: v_type -> ring_id -> value
        self.f_cost_dict: Dict[str, Dict[int, float]] = {v: {} for v in v_types}
        self.l_cost_dict: Dict[str, Dict[int, float]] = {v: {} for v in v_types}
        self.t_cost_dict: Dict[str, Dict[int, float]] = {v: {} for v in v_types}
