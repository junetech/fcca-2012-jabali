"""main for FCCA MIP model solving
"""
import json
import datetime
from time import time
from typing import List

from master_metadata import MasterMetadata
from params_fcca import ParamsFCCA
from result_fcca import ResultFCCA
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
    params: ParamsFCCA, metadata: MasterMetadata
) -> ResultFCCA:
    params.make_mip_dicts()
    fcca_model = make_fcca_mip_model(params, metadata.model)
    if metadata.write_lp_file:
        fcca_model.write(metadata.lp_file_path())
    fcca_model.optimize()
    return make_result_fcca(fcca_model, params, metadata.model)


def result_using_inner_circle(
    params: ParamsFCCA, metadata: MasterMetadata
) -> ResultFCCA:
    result = result_by_params(params, metadata)
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


def main():
    """read and solve
    """
    master_metadata_filename = "master_metadata.json"
    metadata = MasterMetadata(master_metadata_filename)

    params = ParamsFCCA(
        metadata.env_json_filename,
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

    params_fcca = ParamsFCCA(
        metadata.env_json_filename,
        metadata.veh_file_postfix,
        metadata.veh_file_ext,
        metadata.json_encoding,
    )
    params_fcca.print_info()
    params_fcca.amend_time_unit()
    params_fcca.make_mip_dicts()
    fcca_model = make_fcca_mip_model(params_fcca, metadata.model)
    fcca_model.optimize()
    result = make_result_fcca(fcca_model, params_fcca, metadata.model)
    # Optimize until (the inner circle radius > 0)
    inner_circle_radius = sum(
        result.l_dict[i][params_fcca.inner_ring_id]
        for i in params_fcca.actual_veh_type_list
    )
    while inner_circle_radius == 0.0:
        params_fcca.max_ring_count -= 1
        print(
            f"Trying again with max_ring_count = {params_fcca.max_ring_count}"
        )
        params_fcca.make_mip_dicts()
        fcca_model = make_fcca_mip_model(params_fcca, metadata.model)
        fcca_model.optimize()
        result = make_result_fcca(fcca_model, params_fcca, metadata.model)
        inner_circle_radius = sum(
            result.l_dict[i][params_fcca.inner_ring_id]
            for i in params_fcca.actual_veh_type_list
        )
    params_fcca.veh_dict["crowd1"].availability = 100
    params_fcca.make_mip_dicts()
    # Validity test of the solution
    if metadata.validity_test:
        test_validity(params_fcca, result)
    show_result(result, metadata)


if __name__ == "__main__":
    start_time = time()
    main()
    elapsed_time = time() - start_time
    elapsed_date = datetime.timedelta(seconds=elapsed_time)
    print(__name__, "program finished, took total", elapsed_date)
