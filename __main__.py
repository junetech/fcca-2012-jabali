"""main for FCCA MIP model solving
"""
import json
import datetime
from time import time

from params_fcca import ParamsFCCA
from fcca_mip import make_fcca_mip_model


class MasterMetadata:
    """Data of data files
    """
    env_json_filename: str
    veh_file_postfix: str
    veh_file_ext: str
    json_encoding: str
    lp_filename: str

    def __init__(self, filename: str):
        with open(filename) as f_data:
            _dict = json.load(f_data)
            for key, value in _dict.items():
                self.__dict__[key] = value


def main():
    """read and solve
    """
    master_metadata_filename = "master_metadata.json"
    metadata = MasterMetadata(master_metadata_filename)

    params_fcca = ParamsFCCA(metadata.env_json_filename,
                             metadata.veh_file_postfix,
                             metadata.veh_file_ext,
                             metadata.json_encoding)

    fcca_model = make_fcca_mip_model(params_fcca)

    # write to .lp file
    fcca_model.write(metadata.lp_filename)
    fcca_model.optimize()


if __name__ == "__main__":
    start_time = time()
    main()
    elapsed_time = time() - start_time
    elapsed_date = datetime.timedelta(seconds=elapsed_time)
    print(__name__, "program finished, took total", elapsed_date)