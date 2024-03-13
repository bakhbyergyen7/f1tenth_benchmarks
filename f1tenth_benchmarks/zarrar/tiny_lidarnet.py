import time
import numpy as np
from numba import njit
from f1tenth_benchmarks.utils.BasePlanner import BasePlanner
import tensorflow as tf

class TinyLidarNet(BasePlanner):
    def __init__(self, test_id, skip_n, model_path):
        super().__init__("TinyLidarNet", test_id)
        self.skip_n = skip_n
        self.model_path = model_path
        self.name = 'TinyLidarNet'
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_index = self.interpreter.get_input_details()[0]["index"]
        self.output_details = self.interpreter.get_output_details()

    def linear_map(self, x, x_min, x_max, y_min, y_max):
        return (x - x_min) / (x_max - x_min) * (y_max - y_min) + y_min

    def render_waypoints(self, *args, **kwargs):
        pass
        
    def plan(self, obs):
        scans = obs['scan']
        noise = np.random.normal(0, 0.5, scans.shape)
        scans = scans + noise
        # chunks = [scans[i:i+4] for i in range(0, len(scans), 4)]
        # scans = [np.max(chunk) for chunk in chunks]
        # scans = np.array(scans)
        scans[scans>10] = 10
        scans = scans[::self.skip_n]
        scans = np.expand_dims(scans, axis=-1).astype(np.float32)
        scans = np.expand_dims(scans, axis=0)
        self.interpreter.set_tensor(self.input_index, scans)
        

        start_time = time.time()
        self.interpreter.invoke()
        inf_time = time.time() - start_time
        inf_time = inf_time*1000
        output = self.interpreter.get_tensor(self.output_details[0]['index'])

        steer = output[0,0]
        speed = output[0,1]
        speed = self.linear_map(speed, 0, 1, 2., 8)
        action = np.array([steer, speed])

        return action