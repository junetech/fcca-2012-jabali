"""main for FCCA MIP model solving
"""
import json
import datetime
import math
from time import time
from typing import List, Dict

from master_metadata import MasterMetadata
from params_fcca import ParamsFCCA
from result_fcca import ResultFCCA
from interval import Interval
from fcca_mip import make_fcca_mip_model, make_result_fcca
from solution_tests import test_validity


def show_result(
    result: ResultFCCA, params: ParamsFCCA, metadata: MasterMetadata
):
    """show results to terminal & save as csv

    Args:
        result (ResultFCCA)
        metadata (MasterMetadata)
    """
    result.print_variable_info()
    result.print_cost_info()
    if metadata.write_var_file:
        result.variable_to_csv(metadata.variable_csv_path())
    result.vehicle_to_csv(
        params, metadata.veh_csv_path(mid_str="delta1_zp1000")
    )
    result.ring_to_csv(params, metadata.ring_csv_path(mid_str="delta1_zp1000"))


def result_by_params(
    params: ParamsFCCA, metadata: MasterMetadata, private_size=math.inf
) -> ResultFCCA:
    params.make_mip_dicts()
    fcca_model = make_fcca_mip_model(params, metadata.model, private_size)
    if metadata.write_lp_file:
        fcca_model.write(metadata.lp_file_path())
    fcca_model.optimize()
    return make_result_fcca(fcca_model, params, metadata.model)


def result_using_inner_circle(
    params: ParamsFCCA, metadata: MasterMetadata, private_size=math.inf
) -> ResultFCCA:
    result = result_by_params(params, metadata, private_size)
    # Optimize until (the inner circle radius > 0)
    inner_circle_radius = sum(
        result.l_dict[i][params.inner_ring_id]
        for i in params.actual_veh_type_list
    )
    while inner_circle_radius == 0.0:
        params.max_ring_count -= 1
        if params.max_ring_count == 0:
            raise ValueError("No solution using inner circle")
        print(f"Trying again with max_ring_count = {params.max_ring_count}")
        result = result_by_params(params, metadata)
        inner_circle_radius = sum(
            result.l_dict[i][params.inner_ring_id]
            for i in params.actual_veh_type_list
        )
    return result


def solve_single():
    """read and solve
    """
    master_metadata_filename = "master_metadata.json"
    metadata = MasterMetadata(master_metadata_filename)

    params = ParamsFCCA()
    params.fill_env_info_from_json(
        metadata.env_json_filename, metadata.json_encoding,
    )
    params.fill_veh_info_from_json(
        metadata.veh_file_postfix,
        metadata.veh_file_ext,
        metadata.json_encoding,
    )
    params.print_info()
    params.amend_time_unit()
    # Validity test of the solution
    result = result_using_inner_circle(params, metadata)
    if metadata.validity_test:
        test_validity(params, result)
    print("Optimize method finished")
    show_result(result, params, metadata)


def substitute_test():
    master_metadata_filename = "master_metadata.json"
    metadata = MasterMetadata(master_metadata_filename)

    params = ParamsFCCA()
    params.fill_env_info_from_json(
        metadata.env_json_filename, metadata.json_encoding,
    )
    params.fill_veh_info_from_json(
        metadata.veh_file_postfix,
        metadata.veh_file_ext,
        metadata.json_encoding,
    )
    params.print_info()
    params.amend_time_unit()
    params.make_mip_dicts()
    fcca_model = make_fcca_mip_model(params, metadata.model)
    fcca_model.optimize()
    result = make_result_fcca(fcca_model, params, metadata.model)
    # Optimize until (the inner circle radius > 0)
    inner_circle_radius = sum(
        result.l_dict[i][params.inner_ring_id]
        for i in params.actual_veh_type_list
    )
    while inner_circle_radius == 0.0:
        params.max_ring_count -= 1
        print(f"Trying again with max_ring_count = {params.max_ring_count}")
        params.make_mip_dicts()
        fcca_model = make_fcca_mip_model(params, metadata.model)
        fcca_model.optimize()
        result = make_result_fcca(fcca_model, params, metadata.model)
        inner_circle_radius = sum(
            result.l_dict[i][params.inner_ring_id]
            for i in params.actual_veh_type_list
        )
    params.veh_dict["crowd1"].availability = 100
    params.make_mip_dicts()
    # Validity test of the solution
    if metadata.validity_test:
        test_validity(params, result)
    show_result(result, metadata)


def make_private_veh_intervals(
    params: ParamsFCCA, metadata: MasterMetadata
) -> Dict[Interval, ResultFCCA]:
    return_dict: Dict[Interval, ResultFCCA] = dict()
    params.amend_time_unit()
    private_size = math.inf
    while private_size > -1:
        result = result_using_inner_circle(
            params, metadata, private_size=private_size
        )
        if metadata.validity_test:
            test_validity(params, result, metadata.validity_print)
        if hasattr(result, "private_type_id"):
            interval_start = result.veh_count_veh[result.private_type_id]
        else:
            interval_start = 0
        return_dict[Interval(interval_start, private_size)] = result
        private_size = interval_start - 1

    return return_dict


def solve_long_term():
    from params_env_long import ParamsEnvLong

    master_metadata_filename = "master_metadata.json"
    metadata = MasterMetadata(master_metadata_filename)

    params_long = ParamsEnvLong()
    params_long.fill_from_json(
        metadata.env_long_filename, metadata.json_encoding
    )
    day_interval_result_dict: Dict[int, Dict[Interval, ResultFCCA]] = dict()
    for idx, params_fcca in enumerate(params_long.generate_params_fcca()):
        params_fcca.fill_veh_info_from_json(
            metadata.veh_file_postfix,
            metadata.veh_file_ext,
            metadata.json_encoding,
        )
        result_dict = make_private_veh_intervals(params_fcca, metadata)
        day_interval_result_dict[idx + 1] = result_dict

    private_size_set = set()
    for result_dict in day_interval_result_dict.values():
        for interval in result_dict:
            private_size_set.update({interval.start, interval.end})
    private_size_list = sorted(list(private_size_set))

    keystone = "PrivateSize"
    p_var_cost_col = "PrivateVarCost"
    c_var_cost_col = "CrowdCost"
    p_fixed_cost_col = "PrivateFixedCost"
    t_cost_col = "TotalCost"

if __name__ == "__main__":
    start_time = time()
    solve_long_term()
    elapsed_time = time() - start_time
    elapsed_date = datetime.timedelta(seconds=elapsed_time)
    print(__name__, "program finished, took total", elapsed_date)
