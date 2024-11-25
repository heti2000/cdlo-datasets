import pathlib
import numpy as np

def _extract_npz(npz):
    data = {}
    for key in npz:
        data[key] = npz[key]
    return data

class VesselData:

    def __init__(self, data_path):
        self.data_path = pathlib.Path(data_path)

    def get_train_split(self):
        return list(range(0, 100))

    def get_val_split(self):
        return list(range(100, 118))

    def get_test_split(self):
        return list(range(118, 136))
    
    def get_samples_per_set(self):
        return 96

    def load_pcl(self, set_id, sample_id):
        pcl_path = self.data_path / "{:04d}".format(set_id) / "pcl_2048" / "pcl_{:04d}.npy".format(sample_id)
        return np.load(pcl_path)

    def load_seg(self, set_id, sample_id):
        seg_path = self.data_path / "{:04d}".format(set_id) / "seg_2048" / "seg_{:04d}.npy".format(sample_id)
        return np.load(seg_path)
    
    def load_skeleton(self, sample, sample_id):
        graph_path = self.data_path / "{:04d}".format(sample) / "skeletons" / "{:04d}.npz".format(sample_id)
        return _extract_npz(np.load(graph_path))