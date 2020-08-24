import json
import os
from typing import List


class MasterMetadata:
    """Data of data files
    """

    env_json_filename: str
    veh_file_postfix: str
    veh_file_ext: str
    json_encoding: str
    result_folder: str
    csv_ext: str

    write_lp_file: bool
    lp_filename: str
    write_var_file: bool
    var_file_prefix: str
    veh_file_prefix: str
    ring_file_prefix: str
    long_term_file_prefix: str

    model: str
    model_selection: List[str]
    validity_test: bool

    def __init__(self, filename: str):
        with open(filename) as f_data:
            _dict = json.load(f_data)
            for key, value in _dict.items():
                self.__dict__[key] = value
        if self.model not in self.model_selection:
            first_model = self.model_selection[0]
            warn_str = f"Model {self.model} not in available model list"
            warn_str += f" {self.model_selection}: fixed to {first_model}"
            print(Warning(warn_str))
            self.model = first_model
        if not os.path.exists(self.result_folder):
            os.makedirs(self.result_folder)

    def lp_file_path(self) -> str:
        return self.result_folder + self.lp_filename

    def variable_csv_path(self, mid_str="") -> str:
        return (
            self.result_folder + self.var_file_prefix + mid_str + self.csv_ext
        )

    def veh_csv_path(self, mid_str="") -> str:
        filename = self.veh_file_prefix + mid_str + self.csv_ext
        return self.result_folder + filename

    def ring_csv_path(self, mid_str="") -> str:
        filename = self.ring_file_prefix + mid_str + self.csv_ext
        return self.result_folder + filename

    def long_term_csv_path(self, mid_str="") -> str:
        filename = self.long_term_file_prefix + mid_str + self.csv_ext
        return self.result_folder + filename
