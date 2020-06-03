"""main for FCCA MIP model solving
"""
import json
import datetime
from time import time
from typing import List

from params_fcca import ParamsFCCA
from fcca_mip import make_fcca_mip_model, show_result


class MasterMetadata:
    """Data of data files
    """
    env_json_filename: str
    veh_file_postfix: str
    veh_file_ext: str
    json_encoding: str
    lp_filename: str

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


def optimize_and_show_result(fcca_model,
                             params: ParamsFCCA,
                             metadata: MasterMetadata):
    # write to .lp file
    fcca_model.write(metadata.lp_filename)
    fcca_model.optimize()

    # show results
    print("Optimize method finished")
    show_result(fcca_model, params)


def main():
    """read and solve
    """
    master_metadata_filename = "master_metadata.json"
    metadata = MasterMetadata(master_metadata_filename)

    params_fcca = ParamsFCCA(metadata.env_json_filename,
                             metadata.veh_file_postfix,
                             metadata.veh_file_ext,
                             metadata.json_encoding)
    params_fcca.print_info()
    params_fcca.amend_time_unit()
    params_fcca.make_mip_dicts()
    fcca_model = make_fcca_mip_model(params_fcca, metadata.model)
    optimize_and_show_result(fcca_model, params_fcca, metadata)


if __name__ == "__main__":
    start_time = time()
    main()
    elapsed_time = time() - start_time
    elapsed_date = datetime.timedelta(seconds=elapsed_time)
    print(__name__, "program finished, took total", elapsed_date)
