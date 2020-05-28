"""main for FCCA MIP model solving
"""
import datetime
from time import time

from params_fcca import ParamsFCCA
from fcca_mip import make_fcca_mip_model


def main():
    """read and solve
    """
    env_json_filename = "params_env.json"
    veh_file_postfix = "params_veh_"
    veh_file_ext = ".json"
    encoding = "utf-8"

    params_fcca = ParamsFCCA(env_json_filename,
                             veh_file_postfix,
                             veh_file_ext,
                             encoding)

    fcca_model = make_fcca_mip_model(params_fcca)

    fcca_model.optimize()


if __name__ == "__main__":
    start_time = time()
    main()
    elapsed_time = time() - start_time
    elapsed_date = datetime.timedelta(seconds=elapsed_time)
    print(__name__, "program finished, took total", elapsed_date)
