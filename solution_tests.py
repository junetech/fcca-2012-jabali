import math
from typing import List, Any

from params_fcca import ParamsFCCA
from result_fcca import ResultFCCA


def test_leq(lhs, rhs, abs_tol, err_str):
    print(f"{lhs} <= {rhs}", err_str)
    assert lhs <= rhs + abs_tol, err_str


def test_eq(lhs, rhs, abs_tol, err_str):
    print(f"{lhs} == {rhs}", err_str)
    assert math.isclose(lhs, rhs, rel_tol=0.0, abs_tol=abs_tol), err_str


def test_validity(params: ParamsFCCA, result: ResultFCCA):
    """Test objective & constraints

    Args:
        params (ParamsFCCA)
        result (ResultFCCA): after calc_all_costs
    """
    print("Starting validating a solution of MIP model")
    test_obj(result, params.abs_tol)
    test_constraints(params, result)


def test_obj(result: ResultFCCA, abs_tol: float):
    """check whether objective value from solver
    & value calculated from parameters are the same

    Args:
        result (ResultFCCA): : after calc_all_costs
    """
    _str = f"Objective value equality"
    test_eq(result.solver_obj, result.cost_total, abs_tol, _str)


def test_constraints(params: ParamsFCCA, result: ResultFCCA):
    """Test whether the solution in result satisfies constraints by params

    Args:
        params (ParamsFCCA)
        result (ResultFCCA): after calc_all_costs
    """
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
