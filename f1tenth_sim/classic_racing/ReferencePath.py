import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
import trajectory_planning_helpers as tph
import csv



class ReferencePath:
    def __init__(self, map_name, w=0.35):
        # self.width = width
        self.map_name = map_name
        self.path = None
        self.el_lengths = None 
        self.psi = None
        self.nvecs = None
        self.track_length = None
        self.init_path()

        self.center_lut_x, self.center_lut_y = None, None
        self.left_lut_x, self.left_lut_y = None, None
        self.right_lut_x, self.right_lut_y = None, None

        self.center_lut_x, self.center_lut_y = self.get_interpolated_path_casadi('lut_center_x', 'lut_center_y', self.path, self.s_track)
        self.angle_lut_t = self.get_interpolated_heading_casadi('lut_angle_t', self.psi, self.s_track)

        left_path = self.path - self.nvecs * (self.track[:, 2][:, None]  - w)
        right_path = self.path + self.nvecs * (self.track[:, 3][:, None] - w)
        self.left_lut_x, self.left_lut_y = self.get_interpolated_path_casadi('lut_left_x', 'lut_left_y', left_path, self.s_track)
        self.right_lut_x, self.right_lut_y = self.get_interpolated_path_casadi('lut_right_x', 'lut_right_y', right_path, self.s_track)

    def init_path(self):
        filename = 'maps/' + self.map_name + '_centerline.csv'
        track = np.loadtxt(filename, delimiter=',')
        self.track = np.row_stack((track, track[1:int(track.shape[0] / 2), :]))

        self.path = track[:, :2]
        self.path = np.row_stack((self.path, self.path[1:int(self.path.shape[0] / 2), :]))

        self.el_lengths = np.linalg.norm(np.diff(self.track[:, :2], axis=0), axis=1)
        self.s_track = np.insert(np.cumsum(self.el_lengths), 0, 0)
        self.psi, self.kappa = tph.calc_head_curv_num.calc_head_curv_num(self.track, self.el_lengths, False)

        angle_diffs = np.diff(self.psi, axis=0)
        for i in range(len(angle_diffs)):
            if angle_diffs[i] > np.pi:
                self.psi[i+1:] -= 2*np.pi
            elif angle_diffs[i] < -np.pi:
                self.psi[i+1:] += 2*np.pi

        self.nvecs = tph.calc_normal_vectors_ahead.calc_normal_vectors_ahead(self.psi-np.pi/2)

        self.track_length = self.s_track[-1]

    def get_interpolated_path_casadi(self, label_x, label_y, pts, arc_lengths_arr):
        u = arc_lengths_arr
        V_X = pts[:, 0]
        V_Y = pts[:, 1]
        lut_x = ca.interpolant(label_x, 'bspline', [u], V_X)
        lut_y = ca.interpolant(label_y, 'bspline', [u], V_Y)
        return lut_x, lut_y
    
    def get_interpolated_heading_casadi(self, label, pts, arc_lengths_arr):
        u = arc_lengths_arr
        V = pts
        lut = ca.interpolant(label, 'bspline', [u], V)
        return lut
    
    def calculate_s(self, point):
        distances = np.linalg.norm(self.path - point, axis=1)
        idx = np.argmin(distances)
        x, h = self.interp_pts(idx, distances)
        s = (self.s_track[idx] + x) 

        return s

    def interp_pts(self, idx, dists):
        d_ss = self.s_track[idx+1] - self.s_track[idx]
        d1, d2 = dists[idx], dists[idx+1]

        if d1 < 0.01: # at the first point
            x = 0   
            h = 0
        elif d2 < 0.01: # at the second point
            x = dists[idx] # the distance to the previous point
            h = 0 # there is no distance
        else:     # if the point is somewhere along the line
            s = (d_ss + d1 + d2)/2
            Area_square = (s*(s-d1)*(s-d2)*(s-d_ss))
            if Area_square < 0:  # negative due to floating point precision
                h = 0
                x = d_ss + d1
            else:
                Area = Area_square**0.5
                h = Area * 2/d_ss
                x = (d1**2 - h**2)**0.5

        return x, h


    def plot_path(self):
        plt.figure(2)
        plt.clf()

        # plt.plot(self.center_lut_x(self.s_track), self.center_lut_y(self.s_track), label="center", color='blue', alpha=0.7)
        # plt.plot(self.left_lut_x(self.s_track), self.left_lut_y(self.s_track), label="left", color='green', alpha=0.7)
        # plt.plot(self.right_lut_x(self.s_track), self.right_lut_y(self.s_track), label="right", color='green', alpha=0.7)

        plt.plot(self.path[:, 0], self.path[:, 1], label="center", color='blue', alpha=0.7)

        # plt.show()


    def plot_angles(self):
        plt.figure(3)
        plt.clf()

        plt.plot(self.s_track, self.angle_lut_t(self.s_track), label="Fixed angles", color='blue')



if __name__ == "__main__":
    path = ReferencePath("aut")
    path.plot_path()
    path.plot_angles()

    print(path.center_lut_x(120))
    plt.show()
