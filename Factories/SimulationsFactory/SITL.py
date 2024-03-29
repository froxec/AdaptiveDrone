import numpy as np
from Simulation.model import system
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from Simulation.model import quadcopterModel, loadPendulum, System
from Factories.RLFactory.Agents.BanditEstimatorAgent import BanditEstimatorAgent, BanditEstimatorAcceleration
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import *
from Factories.ToolsFactory.GeneralTools import euclidean_distance
from Factories.ModelsFactory.uncertain_models import *
from Factories.ControllersFactory.position_controllers.position_controller import PositionController
from typing import Type
from copy import deepcopy
import matplotlib.pyplot as plt

class SoftwareInTheLoop:
    def __init__(self, quad: Type[quadcopterModel],
                 trajectory,
                 position_controller: Type[PositionController],
                 attitude_controller, esc,
                 inner_loop_freq: int,
                 outer_loop_freq: int,
                 estimator=None,
                 adaptive_controller=None,
                 acceleration_noise=[0, 0, 0]):
        self.INNER_LOOP_FREQ = inner_loop_freq
        self.OUTER_LOOP_FREQ = outer_loop_freq
        self.MODULO_FACTOR = self.INNER_LOOP_FREQ / self.OUTER_LOOP_FREQ
        self.quad = quad
        self.position_controller = position_controller
        self.attitude_controller = attitude_controller
        self.estimator = estimator
        self.trajectory = trajectory
        self.esc = esc
        self.adaptive_controller = adaptive_controller
        if self.estimator is not None:
            self.system = System(self.quad, return_dstate=True)
        else:
            self.system = System(self.quad)
        if not isinstance(acceleration_noise, np.ndarray):
            acceleration_noise = np.array(acceleration_noise)
        self.acceleration_noise = acceleration_noise
        self.history = {'u_ref': [],
                        'sigma': [],
                        'u_l1': []}

    def run(self, stop_time, deltaT, x0, u0, setpoint):
        import time
        t = np.arange(0, stop_time, deltaT)
        x = np.zeros((t.size, 12))
        x[0] = x0
        z_prev = x0[3:6]
        ref_prev = u0
        u_prev = ref_prev
        u_saturated_prev = u_prev
        self.history['u_ref'].append(ref_prev)
        self.history['sigma'].append(np.zeros(3))
        self.history['u_l1'].append(np.zeros(3))
        self.position_controller.change_trajectory(setpoint)
        for i, t_i in enumerate(t[1:], 0):
            if (i % self.MODULO_FACTOR) == 0:
                ref = self.position_controller(x[i, :6], ref_prev, convert_throttle=False)
                if self.adaptive_controller is None:
                    u_composite = ref
                ref_prev = ref
            if (self.adaptive_controller is not None and
                    isinstance(self.adaptive_controller.predictor.ref_model, QuadTranslationalDynamicsUncertain)):
                z = x[i, 3:6]
                z_prev = x[i - 1, 3:6]
                u = ref
                u_composite = self.adaptive_controller(z, z_prev,
                                                       np.concatenate([u, np.array([0])]),
                                                       np.concatenate([u_prev, np.array([0])]), t_i)
                u_prev = u
            elif (self.adaptive_controller is not None and
                  isinstance(self.adaptive_controller.predictor.ref_model, LinearQuadUncertain)):
                delta_x, delta_u = self.position_controller.input_converter(x[i, :6], ref)
                z = delta_x[3:6]
                u = delta_u
                delta_u_composite = self.adaptive_controller(z, z_prev,
                                                             u, u_prev, t_i)
                u_prev = u
                z_prev = z
                u_composite = self.position_controller.output_converter(delta_u_composite, throttle=False)
            mpc_u = self.position_controller.output_converter.convert_throttle(u_composite)
            attitude_setpoint = np.concatenate([mpc_u[1:], np.array([0.0])])
            throttle = mpc_u[0]
            ESC_PWMs = self.attitude_controller(attitude_setpoint, self.quad.state[6:9], self.quad.state[9:12],
                                                throttle)
            motors = self.esc(ESC_PWMs)
            x[i + 1], dstate = self.system(np.array(motors), deltaT, self.quad)[:12]
            self.history['u_ref'].append(ref)
            if self.adaptive_controller is not None:
                self.history['u_l1'].append(self.adaptive_controller.lp_filter.u_l1)
                self.history['sigma'].append(self.adaptive_controller.adaptive_law.sigma_hat)
            if isinstance(self.estimator, BanditEstimatorAgent):
                self.estimator(x[i + 1, :6], ref_prev)
            elif isinstance(self.estimator, BanditEstimatorAcceleration):
                if self.estimator.process_finished:
                    if self.estimator.testing_mode:
                        print("SITL break at {} estimators step. Estimator in TESTING mode with MAX_STEPS {}".format(
                            self.estimator.current_step, self.estimator.max_steps
                        ))
                        break
                    else:
                        pass
                else:
                    mode = self.estimator.mode.split('_')
                    if mode[1] == 'CONTROL':
                        angles = attitude_setpoint
                    elif mode[1] == 'MEASUREMENT':
                        angles = x[i, 6:9]
                    if mode[0] == 'ACCELERATION':
                        dstate = dstate[3:6] + np.random.normal(loc=np.zeros_like(self.acceleration_noise),
                                                                scale=self.acceleration_noise,
                                                                size=self.acceleration_noise.shape[0])
                        self.estimator(dstate, force_norm=u_composite[0], angles=angles)
                    elif mode[0] == 'VELOCITY':
                        self.estimator(x[i, 3:6], force_norm=u_composite[0], angles=angles, deltaT=1/self.INNER_LOOP_FREQ)
            if isinstance(self.trajectory, type(TrajectoryWithTerminals())):
                terminal_ind = self.check_if_reached_terminal(x[i + 1, :6])
                if terminal_ind is not None:
                    self.quad.mass = self.quad.nominal_mass + self.trajectory.terminals_payload[terminal_ind]
            ##TODO move animation into different module
            # if (i % int(1/(deltaT*FPS))) == 0:
            #     #print(i)
            #     #visualizer(quad.state[0:3], quad.state[6:9], t_i)
            #     # send = plot_pipe.send
            #     data_to_send = np.concatenate((quad.state[0:3], quad.state[6:9], np.array([t_i])))
            # send(data_to_send)
            # time.sleep(deltaT)
            # while(time.time() - start < t_i + deltaT):
            #     time.sleep(PAUSE_INCREMENT)

            # print(prev_stop_time)
            # print(time.time() - t1)
        return t, x

    def check_if_reached_terminal(self, x):
        x = x[None, :]
        terminals = self.trajectory.terminals
        terminals = np.concatenate([terminals, np.zeros((3, 3))], axis=1)
        distances = euclidean_distance(x, terminals, axis=1)
        terminal_ind = None
        if True in [dist < 1 for dist in distances]:
            terminal_ind = np.argmin(distances)
            print(terminal_ind)
        return terminal_ind


class SoftwareInTheLoopLegacy:
    # TODO pass controllers as configuration
    def __init__(self, quad: Type[quadcopterModel],
                 load: Type[loadPendulum],
                 trajectory,
                 position_controller,
                 attitude_controller,
                 mpc_converters, esc,
                 inner_loop_freq: int,
                 outer_loop_freq: int,
                 thrust_compensator=None,
                 estimator=None,
                 adaptive_controller=None,
                 ramp_saturation_slope=np.array([np.Inf, np.Inf, np.Inf])):
        self.INNER_LOOP_FREQ = inner_loop_freq
        self.OUTER_LOOP_FREQ = outer_loop_freq
        self.MODULO_FACTOR = self.INNER_LOOP_FREQ / self.OUTER_LOOP_FREQ
        self.quad = quad
        self.load = load
        self.position_controller = position_controller
        self.attitude_controller = attitude_controller
        self.mpc_input_converter = mpc_converters[0]
        self.mpc_output_converter = mpc_converters[1]
        self.thrust_compensator = thrust_compensator
        self.estimator = estimator
        self.trajectory = trajectory
        self.esc = esc
        self.adaptive_controller = adaptive_controller
        self.ramp_saturation = RampSaturation(slope_max=ramp_saturation_slope, Ts=1/self.OUTER_LOOP_FREQ)
        #needs refactor
        self.system = System(self.quad)
    def run(self, stop_time, deltaT, x0, u0, setpoint):
        import time
        t = np.arange(0, stop_time, deltaT)
        x = np.zeros((t.size, 12))
        x[0] = x0
        z_prev = x0[3:6]
        ref_prev = u0
        u_prev = ref_prev
        u_saturated_prev = u_prev
        for i, t_i in enumerate(t[1:], 0):
            if (i % self.MODULO_FACTOR) == 0:
                delta_x0, delta_u0 = self.mpc_input_converter(x[i, :6], ref_prev)
                ref = self.position_controller.predict(delta_x0)
                ref_converted = self.mpc_output_converter(ref, throttle=False)
                u_saturated = self.ramp_saturation(ref_converted, u_saturated_prev)
                if self.adaptive_controller is not None:
                    u_saturated_prev = u_saturated
                else:
                    u_saturated_converted = self.mpc_output_converter.convert_throttle(u_saturated)
                    mpc_u = u_saturated_converted
            if (self.adaptive_controller is not None and
                    isinstance(self.adaptive_controller.predictor.ref_model, QuadTranslationalDynamicsUncertain)):
                z = x[i, 3:6]
                z_prev = x[i - 1, 3:6]
                u = u_saturated
                u_composite = self.adaptive_controller(z, z_prev,
                                                       np.concatenate([u, np.array([0])]),
                                                       np.concatenate([u_saturated_prev, np.array([0])]), t_i)
                mpc_u = self.mpc_output_converter.convert_throttle(u_composite)
            elif (self.adaptive_controller is not None and
                    isinstance(self.adaptive_controller.predictor.ref_model, LinearQuadUncertain)):
                z = delta_x0[3:6]
                u = ref
                delta_u_composite = self.adaptive_controller(z, z_prev,
                                                            u, u_prev, t_i)
                u_prev = u
                mpc_u = self.mpc_output_converter(delta_u_composite)
                z_prev = z
            attitude_setpoint = np.concatenate([mpc_u[1:], np.array([0.0])])
            throttle = mpc_u[0]
            ESC_PWMs = self.attitude_controller(attitude_setpoint, self.quad.state[6:9], self.quad.state[9:12], throttle)
            motors = self.esc(ESC_PWMs)
            x[i + 1] = self.system(np.array(motors), deltaT, self.quad)[:12]
            if (i % self.MODULO_FACTOR) == 0:
                ref_prev = self.mpc_output_converter(ref, throttle=False)
            if self.estimator is not None:
                self.estimator(x[i + 1, :6], ref_prev)
            if isinstance(self.trajectory, type(TrajectoryWithTerminals())):
                terminal_ind = self.check_if_reached_terminal(x[i+1, :6])
                if terminal_ind is not None:
                    self.quad.mass = self.quad.nominal_mass + self.trajectory.terminals_payload[terminal_ind]
            ##TODO move animation into different module
            # if (i % int(1/(deltaT*FPS))) == 0:
            #     #print(i)
            #     #visualizer(quad.state[0:3], quad.state[6:9], t_i)
            #     # send = plot_pipe.send
            #     data_to_send = np.concatenate((quad.state[0:3], quad.state[6:9], np.array([t_i])))
            # send(data_to_send)
            # time.sleep(deltaT)
            # while(time.time() - start < t_i + deltaT):
            #     time.sleep(PAUSE_INCREMENT)

            # print(prev_stop_time)
            # print(time.time() - t1)
        return t, x

    def check_if_reached_terminal(self, x):
        x = x[None, :]
        terminals = self.trajectory.terminals
        terminals = np.concatenate([terminals, np.zeros((3, 3))], axis=1)
        distances = euclidean_distance(x, terminals, axis=1)
        terminal_ind = None
        if True in [dist < 1 for dist in distances]:
            terminal_ind = np.argmin(distances)
            print(terminal_ind)
        return terminal_ind


class InnerLoopSITL(SoftwareInTheLoop):
    def __init__(self, quad, load, attitude_controler, esc, inner_loop_freq):
        self.INNER_LOOP_FREQ = inner_loop_freq
        self.deltaT = 1/inner_loop_freq
        self.quad = quad
        self.load = load
        self.attitude_controller = attitude_controler
        self.system = System(self.quad)
        self.esc = esc
    def run(self,  attitude_ref, throttle, stop_time, x0, u0):
        t = np.arange(0, stop_time, self.deltaT)
        x = np.zeros((t.size, 12))
        u = np.zeros((t.size, 4))
        x[0] = x0
        u[0] = u0
        for i, t_i in enumerate(t[1:], 1):
            ESC_PWMs = self.attitude_controller(attitude_ref, self.quad.state[6:9], self.quad.state[9:12],
                                                throttle)
            motors = self.esc(ESC_PWMs)
            x[i], _ = self.system(np.array(motors), self.deltaT, self.quad)[:12]
            u[i] = motors
        self.u = u
        self.attitude_ref = attitude_ref
        self.trajectory = x
        self.time = t
    def plot_attitude_trajectory(self, tested_variable):
        from cycler import cycler
        plt.style.use('../../Factories/PlottingFactory/plotstyle.mplstyle')
        fig, ax = plt.subplots(nrows=3, ncols=1, sharex=True, dpi=100)
        colors = ['#196E85', '#e89b26', 'red', '#326024']
        u = self.u.transpose()
        x = self.trajectory.transpose()
        ref = self.attitude_ref[tested_variable]*np.ones_like(self.time)
        if tested_variable == 0:
            ylabels = [r'$$\phi [rad]$$', r'$$\omega_{\phi} [rad / s]$$', r'$$\varpi_{1, 2, 3, 4} [rad / s]$$']
            data_to_plot = [[x[6, :], ref], [x[9, :]], [u[0,:], u[1,:], u[2,:], u[3,:]]]
            legend = [[r'$$\phi [rad] $$', r'$$\phi_{zad} [rad]$$'], [None], [r'$$\varpi_{1} [rad/s]$$', r'$$\varpi_{2} [rad/s]$$',
                                                                     r'$$\varpi_{3} [rad/s]$$', r'$$\varpi_{4} [rad/s]$$']]
            self._plot_traces(ax, ylabels, data_to_plot, colors, legend, rpm_linestyles=['solid', 'dashed', 'solid', 'dashed'])
        elif tested_variable == 1:
            ylabels = [r'$$\theta [rad]$$', r'$$\omega_{\theta} [rad / s]$$', r'$$\varpi_{1, 2, 3, 4} [rad / s]$$']
            data_to_plot = [[x[7, :]], [x[10, :]], [u[0, :], u[1, :], u[2, :], u[3, :]]]
            legend = [[r'$$\theta [rad]$$', r'$$\theta_{zad} [rad]$$'], [None], [r'$$\varpi_{1} [rad/s]$$', r'$$\varpi_{2} [rad/s]$$',
                                                                     r'$$\varpi_{3} [rad/s]$$', r'$$\varpi_{4} [rad/s]$$']]
            self._plot_traces(ax, ylabels, data_to_plot, colors, legend, rpm_linestyles=['solid', 'solid', 'dashed', 'dashed'])
        elif tested_variable == 2:
            ylabels = [r'$$\psi [rad]$$', r'$$\omega_{\psi} [rad / s]$$', r'$$\varpi_{1, 2, 3, 4} [rad / s]$$']
            data_to_plot = [[x[8, :]], [x[11, :]], [u[0, :], u[1, :], u[2, :], u[3, :]]]
            legend = [[r'$$\psi [rad]$$', r'$$\psi_{zad} [rad]$$'], [None], [r'$$\varpi_{1} [rad/s]$$',  r'$$\varpi_{2} [rad/s]$$',
                      r'$$\varpi_{3} [rad/s]$$',  r'$$\varpi_{4} [rad/s]$$']]
            self._plot_traces(ax, ylabels, data_to_plot, colors, legend, rpm_linestyles=['solid', 'solid', 'dashed', 'dashed'])

    def _plot_traces(self,ax, ylabels, data_to_plot, colors, legend, rpm_linestyles=None):
        for i in range(3):
            traces = data_to_plot[i]
            labels = legend[i]
            if i == 0:
                colors_to_apply = [colors[0], 'red']
            else:
                colors_to_apply = colors
            if i == 2:
                linestyles = rpm_linestyles
            else:
                linestyles = ['solid', 'dashed']
            for j, trace in enumerate(traces):
                ax[i].plot(self.time, trace, linestyle=linestyles[j], color=colors_to_apply[j], label=labels[j])
                ax[i].set_ylabel(ylabels[i])
                if labels[j] is not None:
                    ax[i].legend(loc='right')

class VerificationSITL:
    def __init__(self,
                 quadModel,
                 results_save_path,
                 filename):
        self.quadModel = quadModel
        self.system = System(self.quadModel)
        self.savepath = results_save_path
        self.filename = filename
    def run(self, stop_time, deltaT, x0, motors):
        t = np.arange(0, stop_time, deltaT)
        x = np.zeros((t.size, 12))
        u = np.zeros((t.size, 4))
        x[0] = x0
        u[0] = motors
        for i, t_i in enumerate(t[1:], 1):
            x[i], _ = self.system(motors, deltaT, self.quadModel)[:12]
            u[i] = motors
        return t, x, u

    def plot_trajectory(self, t, x, low_value_indices=[]):
        import matplotlib.pyplot as plt
        plt.style.use('../../Factories/PlottingFactory/plotstyle.mplstyle')
        x = x.transpose()
        data_and_labels = [[(r'$$x [m]$$', x[0]), (r'$$y [m]$$', x[1]), (r'$$z [m]$$', x[2])],
                           [(r'$$V_x [m/s]$$', x[3]), (r'$$V_y [m/s]$$', x[4]), (r'$$V_z [m/s]$$', x[5])],
                           [(r'$$\phi [rad]$$', x[6]), (r'$$\theta [rad]$$', x[7]), (r'$$\psi [rad]$$', x[8])],
                           [(r'$$\omega_{\phi} [rad/s]$$', x[9]), (r'$$\omega_{\theta} [rad/s]$$', x[10]), (r'$$\omega_{\psi} [rad/s]$$', x[11])]]
        fig, ax = plt.subplots(nrows=4, ncols=3, sharex=True, dpi=100)
        for i in range(4):
            for j in range(3):
                ax[i][j].plot(t, data_and_labels[i][j][1])
                ax[i][j].set_ylabel(data_and_labels[i][j][0])
                indices = (i, j)
                if indices in low_value_indices:
                    ax[i][j].set_ylim([-1, 1])
                if i == 3:
                    ax[i][j].set_xlabel(r'$$t [s]$$')
        plt.tight_layout()
        plt.savefig(self.savepath + '/' + self.filename + '.png')
