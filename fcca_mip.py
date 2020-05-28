"""Define, optimize, and obtain results from FCCA MIP model
"""
import math
from typing import List
import gurobipy as gp
from gurobipy import GRB

from params_fcca import ParamsFCCA

Model = gp.Model
BINARY = GRB.BINARY
INTEGER = GRB.INTEGER
CONTINUOUS = GRB.CONTINUOUS
quicksum = gp.quicksum
LinExpr = gp.LinExpr
MINIMIZE = GRB.MINIMIZE


def make_fcca_mip_model(params: ParamsFCCA) -> Model:
    """Define Model instance for MIP formulation of FCCA

    Arguments:
        params {ParamsFCCA}

    Returns:
        Model -- instance of MIP, ready to solve
    """
    # instance init
    model = Model(name=params.description)

    # local aliases
    veh_type_list: List[str] = params.veh_type_idx_list
    actual_veh_type_list: List[str] = params.make_idx_list_without_dummy()
    outer_ring_idx_list: List[int] = params.ring_idx_list[1:]
    inner_ring_idx = params.ring_idx_list[0]
    c_dict = params.make_veh_capacity_dict()
    f_dict = params.make_veh_fixed_cost_dict()
    d_dict = params.make_veh_var_cost_dict()

    # Variables
    # number of vehicles of type i assigned to ring j
    n = [[model.addVar(vtype=INTEGER, name=f"n({i},{j})", lb=0)
          for j in params.ring_idx_list] for i in veh_type_list]
    # 1 if vehicle type i is assigned to ring j
    x = [[model.addVar(vtype=BINARY, name=f"x({i},{j})")
          for j in params.ring_idx_list] for i in veh_type_list]
    # length of a segment in ring j
    l = [[model.addVar(vtype=CONTINUOUS, name=f"l({i},{j})", lb=0.0)
          for j in params.ring_idx_list] for i in veh_type_list]
    # distance(the depot -> the inner edge of ring j) traversed by vehicle i
    y = [[model.addVar(vtype=CONTINUOUS, name=f"y({i},{j})", lb=0.0)
          for j in params.ring_idx_list] for i in veh_type_list]

    # Constraints
    # (3) minimum number of vehicles constrants
    for i in veh_type_list:
        for j in outer_ring_idx_list:
            constr_n = f"MinNumberVtype_{i}_OuterRing_{j}"
            model.addConstr((math.pi / params.w_star)*(y[i][j] + l[i][j]/2)
                            <= n[i][j], constr_n)

    # (4) each outer ring serviced by a single vehicle type
    for j in outer_ring_idx_list:
        model.addConstr(quicksum(x[i][j] for i in veh_type_list) == 1,
                        f"OnlyOneVtypeOuterRing_{j}")

    # (5), (6), (7) big-M constraints
    big_M = 2 * params.radius  # TODO: using params.radius temporarily
    for i in veh_type_list:
        for j in params.ring_idx_list:
            model.addConstr(n[i][j] <= big_M * x[i][j], f"bigM_n_{i}_{j}")
            model.addConstr(y[i][j] <= big_M * x[i][j], f"bigM_y_{i}_{j}")
            model.addConstr(l[i][j] <= big_M * x[i][j], f"bigM_l_{i}_{j}")

    # (8) minimum distance(the depot -> the inner edge of ring j)
    for i in veh_type_list:
        for j in outer_ring_idx_list:
            lhs = quicksum(y[m][j-1] + l[m][j-1] for m in veh_type_list)
            lhs += -big_M * (1 * x[i][j])
            model.addConstr(lhs <= y[i][j],
                            f"MinDistanceVtype_{i}_OuterRing_{j}")
        # y[i][0] == 0 constraint omitted

    # (9) capacity constraints
    for i in veh_type_list:
        for j in outer_ring_idx_list:
            lhs = 2 * params.c_density * params.w_star * l[i][j]
            constr_n = f"CapacityVtype_{i}_OuterRing_{j}"
            model.addConstr(lhs <= c_dict[i], constr_n)

    # (10) duration limit constraints
    for j in outer_ring_idx_list:
        rhs = params.route_duration_limit * params.speed / 2
        lhs = quicksum(y[i][j] + l[i][j]/2 for i in veh_type_list)
        lhs += quicksum(params.c_density * params.w_star * l[i][j] *
                        math.sqrt(2/(3*params.c_density))
                        for i in veh_type_list)
        lhs += quicksum(2 * params.c_density * params.w_star * l[i][j] *
                        params.service_time * params.speed
                        for i in veh_type_list)
        model.addConstr(lhs <= rhs, f"DurationLimitOuterRing_{j}")

    # outer ring cost function
    o_cost = LinExpr()
    # fixed fleet cost
    o_cost += quicksum(quicksum(f_dict[i] * n[i][j] for i in veh_type_list)
                       for j in outer_ring_idx_list)
    # total line-haul cost for the outer rings
    ol_coeff = 2 * math.pi / (params.w_star * params.speed)
    o_cost += quicksum(quicksum(ol_coeff * d_dict[i] * (y[i][j] + l[i][j]/2)
                                * (y[i][j] + l[i][j]/2)
                                for j in outer_ring_idx_list)
                       for i in veh_type_list)
    # total traverse cost
    ot_coeff = 2 * params.c_density * math.pi / params.speed * \
        math.sqrt(2/(3*params.c_density))
    o_cost += quicksum(quicksum((d_dict[i] * (y[i][j] + l[i][j]/2)
                                * l[i][j] * ot_coeff)
                                for j in outer_ring_idx_list)
                       for i in veh_type_list)

    # (12) each inner ring serviced by a single vehicle type except type 0
    model.addConstr(quicksum(x[i][inner_ring_idx]
                             for i in actual_veh_type_list)
                    == 1, "OnlyOneVtypeInnerRing")

    # (13) minimum number of vehicles constrants
    for i in actual_veh_type_list:
        constr_n = f"MinNumberVtype_{i}_InnerRing"
        model.addConstr(math.pi * l[i][inner_ring_idx] / params.gamma
                        <= n[i][inner_ring_idx], constr_n)

    # (14) capacity constraints for each zone serviced by vehicle type i
    for i in actual_veh_type_list:
        constr_n = f"CapacityVtype_{i}_InnerRing"
        model.addConstr(params.c_density * params.gamma * l[i][inner_ring_idx]
                        == c_dict[i], constr_n)

    # inner ring cost function
    i_cost = LinExpr()
    # fixed fleet cost
    i_cost += quicksum(f_dict[i] * n[i][inner_ring_idx]
                       for i in actual_veh_type_list)
    # total line-haul cost for the outer rings
    il_coeff = 2 * math.pi / (params.speed * params.gamma)
    i_cost += quicksum((d_dict[i] * l[i][inner_ring_idx]
                        * l[i][inner_ring_idx] * il_coeff)
                       for i in actual_veh_type_list)
    # total traverse cost
    it_coeff = params.c_density * params.gamma * math.pi / (3 * params.speed)
    i_cost += quicksum((d_dict[i] * l[i][inner_ring_idx]
                        * l[i][inner_ring_idx] * it_coeff)
                       for i in actual_veh_type_list)

    # (16) the entire region is serviced
    model.addConstr(quicksum(quicksum(l[i][j] for i in veh_type_list)
                             for j in params.ring_idx_list) == params.radius,
                    "EntireRegionServed")

    # (17) total capacity is enough for all customers
    total_c = params.total_customer
    model.addConstr(quicksum(quicksum(c_dict[i] * n[i][j]
                                      for i in veh_type_list)
                             for j in params.ring_idx_list) >= total_c,
                    "TotalCapacityEnough")

    # Set minimize total cost objective
    model.setObjective(o_cost + i_cost, MINIMIZE)

    return model


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

    fcca_model = make_fcca_mip_model(fcca_params)

    # write to .lp file
    lp_filename = "fcca_mip.lp"
    fcca_model.write(lp_filename)


if __name__ == "__main__":
    main()
