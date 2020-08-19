"""Define, optimize, and obtain results from FCCA MIP model
"""
import math
from typing import List, Dict, Any
import gurobipy as gp
from gurobipy import GRB

from params_fcca import ParamsFCCA
from result_fcca import ResultFCCA

Model = gp.Model
BINARY = GRB.BINARY
INTEGER = GRB.INTEGER
CONTINUOUS = GRB.CONTINUOUS
quicksum = gp.quicksum
QuadExpr = gp.QuadExpr
LinExpr = gp.LinExpr
MINIMIZE = GRB.MINIMIZE


def make_fcca_mip_model(params: ParamsFCCA, model_str: str) -> Model:
    """Define Model instance for MIP formulation of FCCA

    Args:
        params (ParamsFCCA)
        model_str (str)

    Raises:
        ValueError: when undefied model_str is given

    Returns:
        Model: instance of MIP, ready to solve
    """
    # instance init
    model = Model(name=params.description)
    if not params.gurobi_output:
        model.setParam("OutputFlag", 0)

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

    # Variables
    # number of vehicles of type i assigned to ring j
    n = {
        i: {
            j: model.addVar(vtype=INTEGER, name=params.n_name_dict[i][j], lb=0)
            for j in params.ring_id_list
        }
        for i in veh_type_list
    }
    # 1 if vehicle type i is assigned to ring j
    x = {
        i: {
            j: model.addVar(vtype=BINARY, name=params.x_name_dict[i][j])
            for j in params.ring_id_list
        }
        for i in veh_type_list
    }
    # length of a segment in ring j
    l = {
        i: {
            j: model.addVar(
                vtype=CONTINUOUS, name=params.l_name_dict[i][j], lb=0.0
            )
            for j in params.ring_id_list
        }
        for i in veh_type_list
    }
    # distance(the depot -> the inner edge of ring j) traversed by vehicle i
    y = {
        i: {
            j: model.addVar(
                vtype=CONTINUOUS, name=params.y_name_dict[i][j], lb=0.0
            )
            for j in params.ring_id_list
        }
        for i in veh_type_list
    }
    # # crowdsourced vehicles availability
    # z_s = model.addVar(vtype=INTEGER, lb=0, name="z_s")
    # g_dict = dict()
    # g_dict["dummy"] = params.g_dict["dummy"]
    # g_dict["private"] = params.g_dict["private"]
    # g_dict["crowd"] = 18 + (z_s - 10) / 10
    # model.addConstr(z_s >= 10)

    # Constraints
    # (3) minimum number of vehicles constrants
    for i in veh_type_list:
        for j in outer_ring_id_list:
            constr_n = f"MinNumberVtype_{i}_OuterRing_{j}"
            model.addConstr(
                (math.pi / params.w_star) * (y[i][j] + l[i][j] / 2) <= n[i][j],
                constr_n,
            )

    # (4) each ring serviced by a single vehicle type
    for j in params.ring_id_list:
        model.addConstr(
            quicksum(x[i][j] for i in veh_type_list) == 1,
            f"OnlyOneVtypeOuterRing_{j}",
        )

    # (5), (6), (7) big-M constraints
    big_M_veh_count = total_c
    big_M_radius = params.radius
    for i in veh_type_list:
        for j in params.ring_id_list:
            model.addConstr(
                n[i][j] <= big_M_veh_count * x[i][j], f"bigM_n_{i}_{j}"
            )
            model.addConstr(
                y[i][j] <= big_M_radius * x[i][j], f"bigM_y_{i}_{j}"
            )
            model.addConstr(
                l[i][j] <= big_M_radius * x[i][j], f"bigM_l_{i}_{j}"
            )

    # (8) minimum distance(the depot -> the inner edge of ring j)
    for i in veh_type_list:
        for j in outer_ring_id_list:
            lhs = quicksum(y[m][j - 1] + l[m][j - 1] for m in veh_type_list)
            lhs += big_M_radius * (x[i][j] - 1)
            model.addConstr(
                lhs <= y[i][j], f"MinDistanceVtype_{i}_OuterRing_{j}"
            )

    # (9) outer ring capacity constraints
    for i in veh_type_list:
        for j in outer_ring_id_list:
            lhs = 2 * delta * params.w_star * l[i][j]
            constr_n = f"CapacityVtype_{i}_OuterRing_{j}"
            model.addConstr(lhs <= c_dict[i], constr_n)

    # (10) duration limit constraints of outer rings
    # change from Jabali et al. 2012:
    # calculated only for actual vehicles(excluded dummy type)
    # calculated for each actual vehicle types
    for j in outer_ring_id_list:
        for i in actual_veh_type_list:
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
            model.addConstr(
                lhs <= rhs, f"DurationLimitVtype_{i}_OuterRing_{j}"
            )
    # # duration limit constraints of inner ring
    # # newly added in Jabali et al. 2012
    # for i in actual_veh_type_list:
    #     rhs = t_dict[i] * params.speed
    #     # big_M for vehicles not assigned to inner ring
    #     # TODO: make it tight
    #     rhs += (1 - x[i][inner_ring_id]) * big_M_radius * 4
    #     # inner ring line-haul travel time of a vehicle
    #     lhs = 2 * l[i][inner_ring_id]
    #     # inner ring transverse travel time of a vehicle
    #     lhs += delta * params.gamma * params.gamma * l[i][inner_ring_id] / 3
    #     model.addConstr(
    #         lhs <= rhs, f"DurationLimitVtype_{i}_InnerRing",
    #     )

    # (12) inner ring serviced by a single vehicle type except dummy type
    model.addConstr(
        x[params.dummy_type][inner_ring_id] == 0, "NoDummyVtypeInnerRing",
    )

    # (13) minimum number of vehicles constrants
    for i in actual_veh_type_list:
        constr_n = f"MinNumberVtype_{i}_InnerRing"
        model.addConstr(
            math.pi * l[i][inner_ring_id] / params.gamma
            <= n[i][inner_ring_id],
            constr_n,
        )

    # (14) inner ring capacity constraints for each zone
    for i in actual_veh_type_list:
        constr_n = f"CapacityVtype_{i}_InnerRing"
        model.addConstr(
            delta * params.gamma * l[i][inner_ring_id]
            <= c_dict[i] * x[i][inner_ring_id],
            constr_n,
        )
        # TODO the paper seems wrong: '== c_dict[i]'

    # (16) the entire region is serviced
    model.addConstr(
        quicksum(
            quicksum(l[i][j] for j in params.ring_id_list)
            for i in veh_type_list
        )
        == params.radius,
        "EntireRegionServed",
    )

    # (17) total capacity is enough for all customers
    model.addConstr(
        quicksum(
            quicksum(c_dict[i] * n[i][j] for j in params.ring_id_list)
            for i in veh_type_list
        )
        >= total_c,
        "TotalCapacityEnough",
    )
    # Apply given number of availabie private vehicles
    model.addConstr(
        quicksum(n["private"][j] for j in params.ring_id_list) <= 1000,
        "PrivateVehicleAvailability",
    )
    # # Number of available crowdsourced vehicles
    # model.addConstr(
    #     quicksum(n["crowd"][j] for j in params.ring_id_list) <= z_s,
    #     "CrowdVehicleAvailability",
    # )

    def make_optimal_obj() -> QuadExpr:
        """
        Returns:
            QuadExpr: quadratic objective in optimal MIP
        """
        # preprocess: coefficients calculated
        params.calc_optimal_obj_coeff()
        # outer ring cost function
        o_cost = QuadExpr()
        # fixed fleet cost
        o_cost += quicksum(
            quicksum(f_dict[i] * n[i][j] for i in veh_type_list)
            for j in outer_ring_id_list
        )
        # total line-haul cost for outer rings
        o_cost += (
            quicksum(
                quicksum(
                    g_dict[i] * (y[i][j] + l[i][j] / 2) * n[i][j]
                    for j in outer_ring_id_list
                )
                for i in veh_type_list
            )
            * params.ol_coeff
        )
        # in Jabali et al. 2012:
        # o_cost += (
        #     quicksum(
        #         quicksum(
        #             g_dict[i]
        #             * (y[i][j] + l[i][j] / 2)
        #             * (y[i][j] + l[i][j] / 2)
        #             for j in outer_ring_id_list
        #         )
        #         for i in veh_type_list
        #     )
        #     * params.ol_coeff
        # )

        # total traverse cost for outer rings
        o_cost += (
            quicksum(
                quicksum(
                    (g_dict[i] * l[i][j] * n[i][j]) for j in outer_ring_id_list
                )
                for i in veh_type_list
            )
            * params.ot_coeff
        )
        # in Jabali et al. 2012:
        # o_cost += (
        #     quicksum(
        #         quicksum(
        #             (g_dict[i] * (y[i][j] + l[i][j] / 2) * l[i][j])
        #             for j in outer_ring_id_list
        #         )
        #         for i in veh_type_list
        #     )
        #     * params.ot_coeff
        # )

        # inner ring cost function
        i_cost = QuadExpr()
        # fixed fleet cost
        i_cost += quicksum(
            f_dict[i] * n[i][inner_ring_id] for i in actual_veh_type_list
        )
        # total line-haul cost for inner ring
        i_cost += (
            quicksum(
                (g_dict[i] * l[i][inner_ring_id] * n[i][inner_ring_id])
                # in Jabali et al. 2012:
                # (g_dict[i] * l[i][inner_ring_id] * l[i][inner_ring_id])
                for i in actual_veh_type_list
            )
            * params.il_coeff
        )
        # total traverse cost for inner ring
        i_cost += (
            quicksum(
                (g_dict[i] * l[i][inner_ring_id] * n[i][inner_ring_id])
                # in Jabali et al. 2012:
                # (g_dict[i] * l[i][inner_ring_id] * l[i][inner_ring_id])
                for i in actual_veh_type_list
            )
            * params.it_coeff
        )

        return o_cost + i_cost

    def make_l1_obj() -> QuadExpr:
        """
        Returns:
            QuadExpr: positive semidefinite function for objective of L1
        """
        params.calc_l1_obj_coeff()

        o_cost = QuadExpr()
        o_cost += quicksum(
            quicksum(f_dict[i] * n[i][j] for i in veh_type_list)
            for j in params.ring_id_list
        )
        alpha = params.alpha
        chi_prime = params.chi_p
        o_coeff = params.l1_o_coeff
        o_cost += quicksum(
            g_dict[i]
            * quicksum(
                (
                    y[i][j] * y[i][j]
                    + (2 - chi_prime / alpha) * y[i][j] * l[i][j]
                    + (3 / 4 + chi_prime) * l[i][j] * l[i][j]
                )
                for j in outer_ring_id_list
            )
            * o_coeff
            for i in veh_type_list
        )
        # inner ring cost function
        i_cost = QuadExpr()
        # total line-haul cost for inner ring
        i_cost += (
            quicksum(
                (g_dict[i] * l[i][inner_ring_id] * l[i][inner_ring_id])
                for i in actual_veh_type_list
            )
            * params.il_coeff
        )
        # total traverse cost for inner ring
        i_cost += (
            quicksum(
                (g_dict[i] * l[i][inner_ring_id] * l[i][inner_ring_id])
                for i in actual_veh_type_list
            )
            * params.it_coeff
        )

        return o_cost + i_cost

    if model_str == "U1":
        obj = make_optimal_obj()
        model.setParam("NonConvex", 2)
    elif model_str == "L1":
        obj = make_l1_obj()
    else:
        raise ValueError(f"Wrong model type input({model_str})")
    # Set minimize total cost objective
    model.setObjective(obj, MINIMIZE)

    return model


def show_result(model: Model, params: ParamsFCCA, model_str: str):
    """Summarize results

    Args:
        model (Model): model with at least feasible solution found
        params (ParamsFCCA)
        model_str (str)
    """
    # variable value retrieval
    v_dict: Dict[str, Any] = dict()
    for v in model.getVars():
        if v.x >= 0.0001:
            v_dict[v.varName] = v.x
        else:
            v_dict[v.varName] = 0
    result = ResultFCCA(params, model_str)
    for i in params.vehicle_types:
        if i == params.dummy_type:
            continue
        for j in params.ring_id_list:
            n_name = params.n_name_dict[i][j]
            x_name = params.x_name_dict[i][j]
            y_name = params.y_name_dict[i][j]
            l_name = params.l_name_dict[i][j]
            result.n_dict[i][j] = round(v_dict[n_name])
            result.x_dict[i][j] = round(v_dict[x_name])
            result.y_dict[i][j] = v_dict[y_name]
            result.l_dict[i][j] = v_dict[l_name]

            if result.x_dict[i][j] == 1:
                _str = f"{x_name}: {result.x_dict[i][j]},"
                _str += f" {n_name}: {result.n_dict[i][j]},"
                _str += f" {y_name}: {result.y_dict[i][j]},"
                _str += f" {l_name}: {result.l_dict[i][j]}"
                print(_str)

    # cost calculation
    result.calc_all_costs(params, model_str)
    result.print_cost_info()


def main():
    """parameter load test
    """
    env_json_filename = "params_env.json"
    veh_file_postfix = "params_veh_"
    veh_file_ext = ".json"
    encoding = "utf-8"

    params_fcca = ParamsFCCA(
        env_json_filename, veh_file_postfix, veh_file_ext, encoding
    )
    params_fcca.amend_time_unit()
    params_fcca.make_mip_dicts()
    fcca_model = make_fcca_mip_model(params_fcca)

    # write to .lp file
    lp_filename = "fcca_mip.lp"
    fcca_model.write(lp_filename)


if __name__ == "__main__":
    main()
