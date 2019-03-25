import collections
import logging
import os
import pickle
import shutil
import traceback as tb
from glob import iglob

import msgpack
import numpy as np

def _load_object_legacy(file_path):
    file_path = os.path.expanduser(file_path)
    with open(file_path, "rb") as fh:
        data = pickle.load(fh, encoding="bytes")
    return data


def load_object(file_path, allow_legacy=True):
    import gc

    file_path = os.path.expanduser(file_path)
    with open(file_path, "rb") as fh:
        try:
            gc.disable()  # speeds deserialization up.
            data = msgpack.unpack(fh, raw=False)
        except Exception as e:
            if not allow_legacy:
                raise e
            else:
                logger.info(
                    "{} has a deprecated format: Will be updated on save".format(
                        file_path
                    )
                )
                data = _load_object_legacy(file_path)
        finally:
            gc.enable()
    return data

if __name__ == "__main__":
    filePath = "/home/jacob/pupilLabs/pupil/capture_settings/Pupil_Cam1_ID2.intrinsics"
    data = load_object(filePath)
    print(data)

