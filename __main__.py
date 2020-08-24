"""main for FCCA MIP model solving
"""
import json
import datetime
from time import time
from typing import List

from params_fcca import ParamsFCCA
from result_fcca import ResultFCCA, test_validity
from fcca_mip import make_fcca_mip_model, make_result_fcca


class MasterMetadata:
    """Data of data files
    """

    env_json_filename: str
    veh_file_postfix: str
    veh_file_ext: str
    json_encoding: str
    lp_filename: str
    variable_csv_path: str
    validity_test: bool

    model: str
    model_selection: List[str]

    def __init__(self, filename: str):
        with open(filename) as f_data:
            _dict = json.load(f_data)
            for key, value in _dict.items():
                self.__dict__[key] = value
        if self.model not in self.model_selection:
            last_model = self.model_selection[-1]
            warn_str = f"Model {self.model} not in available model list"
            warn_str += f" {self.model_selection}: fixed to {last_model}"
            print(Warning(warn_str))
            self.model = last_model


def show_result(result: ResultFCCA, metadata: MasterMetadata):
    """show results to terminal & save as csv

    Args:
        result (ResultFCCA)
        metadata (MasterMetadata)
    """
    result.print_variable_info()
    result.print_cost_info()
    result.variable_to_csv(metadata.variable_csv_path)


def main():
    """read and solve
    """
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
    fcca_model.write(metadata.lp_filename)
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
        fcca_model.write(metadata.lp_filename)
        fcca_model.optimize()
        result = make_result_fcca(fcca_model, params_fcca, metadata.model)
        inner_circle_radius = sum(
            result.l_dict[i][params_fcca.inner_ring_id]
            for i in params_fcca.actual_veh_type_list
        )
    # Validity test of the solution
    if metadata.validity_test:
        test_validity(params_fcca, result)
    print("Optimize method finished")
    show_result(result, metadata)


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
    fcca_model.write(metadata.lp_filename)
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
        fcca_model.write(metadata.lp_filename)
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
