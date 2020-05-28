"""Define, optimize, and obtain results from FCCA MIP model
"""
import math
from typing import List
from mip import Model, BINARY, INTEGER, CONTINUOUS, xsum

from params_fcca import ParamsFCCA


def make_fcca_mip_model(params: ParamsFCCA) -> Model:
    """Define Model instance for MIP formulation of FCCA

    Arguments:
        params {ParamsFCCA}

    Returns:
        Model -- instance of MIP, ready to solve
    """
    # instance init
    model = Model()

    # local aliases
    veh_type_list: List[str] = params.vehicle_types
    outer_ring_idx_list: List[int] = params.ring_idx_list[1:]

    # Variables
    # number of vehicles of type i assigned to ring j
    n = [[model.add_var(var_type=INTEGER, name=f"n({i},{j})")
          for i in veh_type_list] for j in params.ring_idx_list]
    # 1 if vehicle type i is assigned to ring j
    x = [[model.add_var(var_type=BINARY, name=f"x({i},{j})")
          for i in veh_type_list] for j in params.ring_idx_list]
    # length of a segment in ring j
    l = [[model.add_var(var_type=CONTINUOUS, name=f"l({i},{j})")
          for i in veh_type_list] for j in params.ring_idx_list]
    # distance(the depot -> the inner edge of ring j) traversed by vehicle i
    y = [[model.add_var(var_type=CONTINUOUS, name=f"y({i},{j})")
          for i in veh_type_list] for j in params.ring_idx_list]

    # Constraints
    # (3) minimum number of vehicles constrants
    for i in veh_type_list:
        for j in outer_ring_idx_list:
            model.add_constr((math.pi / params.w_star)*(y[i][j] + l[i][j]/2),
                             f"Vehicle type {i} min. number on ring {j}")

    # (4) each ring serviced by a single vehicle type
    for j in outer_ring_idx_list:
        model.add_constr(xsum(x[i][j] for i in veh_type_list) == 1,
                         f"Only one vehicle type on ring {j}")

    # (5), (6), (7) big-M constraints
    big_M = 2 * params.radius  # TODO: using params.radius temporarily
    for i in veh_type_list:
        for j in params.ring_idx_list:
            model += n[i][j] <= big_M * x[i][j]
            model += y[i][j] <= big_M * x[i][j]
            model += l[i][j] <= big_M * x[i][j]

    # (8) minimum distance(the depot -> the inner edge of ring j)
    for i in veh_type_list:
        for j in outer_ring_idx_list:
            lhs = xsum(y[m][j-1] + l[m][j-1] for m in veh_type_list)
            lhs += -big_M * (1 * x[i][j])
            model.add_constr(lhs <= y[i][j],
                             f"Minimum distance y[{i}][{j}]")
        # y[i][0] == 0 constraint omitted

    # (9) capacity constraints
    for i in veh_type_list:
        for j in outer_ring_idx_list:
            lhs = 2 * params.c_density * params.w_star * l[i][j]
            constr_n = f"Capacity constraint for vehicle {i} in ring {j}"
            model.add_constr(lhs <= params.veh_dict[i].capacity,
                             constr_n)

    # (10) duration limit constraints
    # TODO: 이어서 작성
    return model
