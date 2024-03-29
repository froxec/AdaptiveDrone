#from pyMPC.mpc import MPCController
import scipy.sparse as sparse
from Factories.ModelsFactory.linear_models import LinearizedQuad
from Factories.ToolsFactory.Converters import AngularVelocityToThrust
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import Trajectory
from Factories.ToolsFactory.GeneralTools import euclidean_distance
#from gekko import GEKKO
import numpy as np
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import time
class PositionController():
    def __init__(self, model=None, control_horizon=None, outer_loop_freq=None):
        self.model = model
        self.control_horizon = control_horizon
        self.outer_loop_freq = outer_loop_freq
    def predict(self, y0, u0, setpoint):
        NotImplementedError

class ModelPredictiveController(PositionController):
    def __init__(self, quad_parameters, x0, trajectory, Ts,  angular_velocity_range, linear_model=LinearizedQuad, save_history=False):
        self.Ts = Ts
        self.x0 = x0
        self.x = x0
        self.position_trajectory = trajectory
        if isinstance(trajectory, Trajectory):
            self.current_waypoint_id = 0
            self.xref = np.concatenate([self.position_trajectory.generated_trajectory[self.current_waypoint_id], np.array([0, 0, 0])])
        else:
            self.xref = np.concatenate([self.position_trajectory, np.array([0, 0, 0])])
            self.current_waypoint_id = None
        self.angular_velocity_range = angular_velocity_range
        self.angular_velocity_converter = AngularVelocityToThrust(quad_parameters['Kt'])
        self.linear_model = linear_model(quad_parameters, yaw_ss=0.0, x_ref=self.xref[0], y_ref=self.xref[1], z_ref=self.xref[2])
        self.thrust_ss = quad_parameters['m']*quad_parameters['g']
        self.Ac = sparse.csc_matrix(self.linear_model.A)
        self.Bc = sparse.csc_matrix(self.linear_model.B)
        self.x_num, self.u_num = self.Bc.shape
        self.Ad = sparse.csc_matrix(np.eye(self.x_num) + self.Ac*self.Ts)
        self.Bd = sparse.csc_matrix(self.Bc*Ts)
        self.epsilon = 1e-3
        #reference values
        self.uminus1 = np.array([0.0, 0.0, 0.0])

        #constraints
        self._set_constraints()

        #cost parameters
        self.q_coef = 1.0
        self.Qx = sparse.diags([self.q_coef/(np.abs(self.xref[0] - self.x0[0]) + self.epsilon), self.q_coef/(np.abs(self.xref[1] - self.x0[1]) + self.epsilon), self.q_coef/(np.abs(self.xref[2] - self.x0[2]) + self.epsilon), 0, 0, 0])  # Quadratic cost for states x0, x1, ..., x_N-1
        self.QxN = sparse.diags([self.q_coef/(np.abs(self.xref[0] - self.x0[0]) + self.epsilon), self.q_coef/(np.abs(self.xref[1] - self.x0[1]) + self.epsilon),self.q_coef/(np.abs(self.xref[2] - self.x0[2]) + self.epsilon), 1, 1, 1])  # Quadratic cost for xN
        self.Qu = sparse.diags([1, 1000, 1000])  # Quadratic cost for u0, u1, ...., u_N-1
        self.QDu = sparse.diags([0, 1000, 1000])  # Quadratic cost for Du0, Du1, ...., Du_N-1

        self.u_prev = self.uminus1
        self.Np = 200
        self.Nc = 10
        self.MPC = MPCController(self.Ad, self.Bd, Np=self.Np, Nc =self.Nc, x0=self.x0, xref=self.xref, uminus1=self.uminus1,
                                 Qx=self.Qx, QxN=self.QxN, Qu=self.Qu, QDu=self.QDu,
                                 xmin=self.xmin, xmax=self.xmax, umin=self.umin, umax=self.umax, Dumin=self.Dumin,
                                 Dumax=self.Dumax)
        self.MPC.setup()
        self.history = []
    def predict(self, x0, u0, setpoint):
        self.x = x0
        self.x0 = x0
        wp_reached = self.check_if_reached_waypoint(x0)
        if wp_reached and self.current_waypoint_id is not None:
            self.current_waypoint_id = self.current_waypoint_id + 1 if self.current_waypoint_id < self.position_trajectory.generated_trajectory.shape[0] - 1 else None
            if self.current_waypoint_id is not None:
                self.xref = np.concatenate([self.position_trajectory.generated_trajectory[self.current_waypoint_id], np.array([0, 0, 0])])
                self.set_reference(self.xref)
        self.MPC.update(self.x, self.u_prev, solve=True)
        u = self.MPC.output()
        self.u_prev = u
        self.save_history(u)
        return u
    def set_reference(self, ref):
        self.xref = ref
        self.Qx = sparse.diags([self.q_coef/(np.abs(self.xref[0] - self.x0[0]) + self.epsilon), self.q_coef/(np.abs(self.xref[1] - self.x0[1]) + self.epsilon), self.q_coef/(np.abs(self.xref[2] - self.x0[2]) + self.epsilon), 0, 0, 0])  # Quadratic cost for states x0, x1, ..., x_N-1
        self.QxN = sparse.diags([self.q_coef/(np.abs(self.xref[0] - self.x0[0]) + self.epsilon), self.q_coef/(np.abs(self.xref[1] - self.x0[1]) + self.epsilon),self.q_coef/(np.abs(self.xref[2] - self.x0[2]) + self.epsilon), 1, 1, 1])  # Quadratic cost for xN
        self.MPC = MPCController(self.Ad, self.Bd, Np=self.Np, Nc=self.Nc, x0=self.x0, xref=self.xref,
                                 uminus1=self.uminus1,
                                 Qx=self.Qx, QxN=self.QxN, Qu=self.Qu, QDu=self.QDu,
                                 xmin=self.xmin, xmax=self.xmax, umin=self.umin, umax=self.umax, Dumin=self.Dumin,
                                 Dumax=self.Dumax)
        self.MPC.setup()
    def update_model_parameters(self, parameters):
        self.angular_velocity_converter = AngularVelocityToThrust(parameters['Kt'])
        self.linear_model.update_parameters(parameters)
        self.thrust_ss = parameters['m'] * parameters['g']

        self.Ac = sparse.csc_matrix(self.linear_model.A)
        self.Bc = sparse.csc_matrix(self.linear_model.B)

        self.Ad = sparse.csc_matrix(np.eye(self.x_num) + self.Ac * self.Ts)
        self.Bd = sparse.csc_matrix(self.Bc * self.Ts)

        self._set_constraints()

        self.MPC = MPCController(self.Ad, self.Bd, Np=self.Np, Nc=self.Nc, x0=self.x0, xref=self.xref,
                                 uminus1=self.uminus1,
                                 Qx=self.Qx, QxN=self.QxN, Qu=self.Qu, QDu=self.QDu,
                                 xmin=self.xmin, xmax=self.xmax, umin=self.umin, umax=self.umax, Dumin=self.Dumin,
                                 Dumax=self.Dumax)
        self.MPC.setup()
    def _set_constraints(self):
        thrust_min = self.angular_velocity_converter(self.angular_velocity_range[0])
        thrust_max = self.angular_velocity_converter(self.angular_velocity_range[1])
        delta_thrust_min = thrust_min - self.thrust_ss
        delta_thrust_max = thrust_max - self.thrust_ss
        self.xmin = np.array([-np.inf, -np.inf, -np.inf, -10, -10, -10])
        self.xmax = np.array([np.inf, np.inf, np.inf, 10, 10, 10])

        self.umin = np.array([delta_thrust_min, -np.pi / 6, -np.pi / 6])
        self.umax = np.array([delta_thrust_max, np.pi / 6, np.pi / 6])

        self.Dumin = np.array([-np.inf, -np.pi * self.Ts / 6, -np.pi * self.Ts / 6])
        self.Dumax = np.array([np.inf, np.pi * self.Ts / 6, np.pi * self.Ts / 6])
    def check_if_reached_waypoint(self, x):
        distance = euclidean_distance(self.xref, x)
        if distance < 0.3:
            return True
        else:
            return False
    def save_history(self, u):
        self.history.append(u)

class gekkoMPC(PositionController):
    def __init__(self, model, control_horizon, outer_loop_freq):
        self.model = model
        self.model.discretize_model(1/outer_loop_freq)
        self.x_bounds = {'lower': [None, None, None, -2.5, -2.5, -2.5],
                    'upper': [None,None, None, 2.5, 2.5, 2.5]}
        self.u_bounds = {'lower': [-model.parameters['m']*model.parameters['g'], -np.pi/6, -np.pi/6],
                         'upper': [model.parameters['m']*model.parameters['g'], np.pi/6, np.pi/6]} #poprawić maksymalne ograniczenie na ciąg
        self.setpoint = None
        self.deadbands = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
        self.costs = {'delta_mv_cost': [0, 0, 0],
                      'delta_mv_max': [np.Inf, np.Inf, np.Inf],
                      'cv_cost': [1, 1, 1, 0, 0, 0]}
        self.m = GEKKO(remote=False)
        self.x, self.y, self.u = self.m.state_space(self.model.Ad, self.model.Bd, self.model.C, D=None, discrete=True)
        self.m.time = np.arange(0, control_horizon, 1/outer_loop_freq)
        #Manipulated variable
        # self.u = [self.m.MV(value=self.u0[i], lb=self.u_bounds['lower'][i], ub=self.u_bounds['upper'][i]) for i in range(len(self.u0))]
        # for i, var in enumerate(self.u):
        #     var.STATUS = 1
        #     var.DCOST = self.costs['delta_mv_cost'][i]
        #     var.DMAX = self.costs['delta_mv_max'][i]
        # #Controled variable
        # self.x = [self.m.CV(value=self.x0[i], lb=self.x_bounds['lower'][i], ub=self.x_bounds['upper'][i]) for i in range(len(self.x0))]
        # for i, var in enumerate(self.x):
        #     var.STATUS = 1
        #     var.DCOST = self.costs['delta_mv_cost'][i]
        #     var.DMAX = self.costs['delta_mv_max'][i]
        #     var.SP = self.setpoint[i]

        for i, var in enumerate(self.u):
            var.LOWER = self.u_bounds['lower'][i]
            var.UPPER = self.u_bounds['upper'][i]
            var.DCOST = self.costs['delta_mv_cost'][i]
            var.DMAX = self.costs['delta_mv_max'][i]
            var.STATUS = 1
            var.FSTATUS = 0
        for i, var in enumerate(self.y):
            # var.LOWER = self.x_bounds['lower'][i]
            # var.UPPER = self.x_bounds['upper'][i]
            #var.COST = self.costs['cv_cost'][i]
            var.STATUS = 1
            var.FSTATUS = 1
        self.m.options.CV_TYPE = 1
        self.m.options.IMODE = 6
        self.m.options.MAX_ITER = 1000
        print(self.m.path)
    def predict(self, y0, u0, setpoint):
        self.setpoint = setpoint
        self.y0 = y0
        self.u0 = u0
        for i, var in enumerate(self.y):
            var.SPHI = self.setpoint[i] + self.deadbands[i]
            var.SPLO = self.setpoint[i] - self.deadbands[i]
            var.SP = self.setpoint[i]
            var.MEAS = self.y0[i]
        self.m.solve(disp=False)
        return np.array([self.u[i].NEWVAL for i in range(len(self.u))])
    def plot(self):
        u_fig = make_subplots(rows=3, cols=1)
        u_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.u[0]), name='Thrust [N]', line={'shape': 'hv'}), row=1, col=1)
        u_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.u[1]), name='Roll [rad]', line={'shape': 'hv'}), row=2, col=1)
        u_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.u[2]), name='Pitch [rad]', line={'shape': 'hv'}), row=3, col=1)

        y_fig = make_subplots(rows=3, cols=2)
        y_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.y[0]), name='delta_x [N]'), row=1, col=1)
        y_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.y[1]), name='delta_y [rad]'), row=2, col=1)
        y_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.y[2]), name='delta_z [rad]'), row=3, col=1)
        y_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.y[3]), name='deltaVx [N]'), row=1, col=2)
        y_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.y[4]), name='deltaVy [rad]'), row=2, col=2)
        y_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.y[5]), name='deltaBz [rad]'), row=3, col=2)

        x_fig = make_subplots(rows=3, cols=2)
        x_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.x[0]), name='delta_x [N]'), row=1, col=1)
        x_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.x[1]), name='delta_y [rad]'), row=2, col=1)
        x_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.x[2]), name='delta_z [rad]'), row=3, col=1)
        x_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.x[3]), name='deltaVx [N]'), row=1, col=2)
        x_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.x[4]), name='deltaVy [rad]'), row=2, col=2)
        x_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.x[5]), name='deltaBz [rad]'), row=3, col=2)
        u_fig.show()
        #x_fig.show()
        y_fig.show()

if __name__ == "__main__":
    model = LinearizedQuadNoYaw(Z550_parameters)
    mpc = gekkoMPC(model, 5 , 10)
    u = mpc.predict([0, 0, 50, 0, 0, 0], [0, 0, 0], [0, 10, 60, 0, 0, 0])
    mpc.plot()