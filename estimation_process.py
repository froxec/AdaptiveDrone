import pickle
import redis
import numpy as np
from Multiprocessing.PARAMS import PREDICTOR_PARAMETERS, CONVERGENCE_EPSILON_NEIGHBOURHOOD, \
    CONVERGENCE_SAMPLES_REQUIRED, X0, SAMPLING_FREQ
from Multiprocessing.process_interfaces import Estimator_Interface
from Factories.DataManagementFactory.data_holders import DataHolder
from Factories.GaussianProcessFactory.gaussian_process import EfficientGaussianProcess
from Factories.GaussianProcessFactory.kernels import RBF_Kernel
from Factories.RLFactory.Agents.Tools.convergenceChecker import ConvergenceChecker
from Factories.RLFactory.Agents.BanditEstimatorAgent import BanditEstimatorAccelerationProcess
from Factories.ModelsFactory.models_for_estimation import NonlinearTranslationalModel
from Factories.ToolsFactory.GeneralTools import LowPassLiveFilter
from oclock import Timer
import time


if __name__ == "__main__":
    FREQ = SAMPLING_FREQ
    DELTA_T = 1 / FREQ

    # init parameters holder
    parameters_holder = DataHolder(PREDICTOR_PARAMETERS)

    # create redis interface
    db_interface = Estimator_Interface()

    # init estimator agent
    prediction_model = NonlinearTranslationalModel(parameters_holder=parameters_holder)
    rbf_kernel = RBF_Kernel(length=0.3)
    gp = EfficientGaussianProcess(X0, rbf_kernel, noise_std=0.08, max_samples=100, overflow_handling_mode='IMPORTANCE')
    convergence_checker = ConvergenceChecker(CONVERGENCE_SAMPLES_REQUIRED, CONVERGENCE_EPSILON_NEIGHBOURHOOD)
    estimator_agent = BanditEstimatorAccelerationProcess(db_interface=db_interface,
                                                         prediction_model=prediction_model,
                                                         gp=gp,
                                                         convergence_checker=convergence_checker,
                                                         deltaT=DELTA_T,
                                                         atomic_traj_samples_num=15,
                                                         mode='VELOCITY_CONTROL',
                                                         save_images=False)
    # velocity filter
    #velocity_filter = LowPassLiveFilter([15, 15, 15], fs=FREQ, signals_num=3, filter_order=1)

    # init Timer
    timer = Timer(interval=DELTA_T)

    # init flag
    saved_to_file = True

    while True:
        t1 = time.time()
        # fetch db
        db_interface.fetch_db()

        # get measurement (velocity)
        x = db_interface.get_drone_state()
        if None not in x:
            velocity = x[3:6]
        else:
            velocity = [None]
        # get control (force_norm and angles)
        u_output = db_interface.get_u_output()

        # run estimator
        if (None not in velocity
            and None not in u_output
            and db_interface.is_estimator_running()):
            f_norm = u_output[0]
            angles = np.concatenate([u_output[1:], np.array([0.0])]) # assumes yaw = 0.0
            estimator_agent(velocity, f_norm, angles, deltaT=DELTA_T)
            saved_to_file = False
        else:
            if not saved_to_file:
                estimator_agent.save_history_to_file()
                saved_to_file = True
            estimator_agent.reset()
        # update db
        if estimator_agent.converged and estimator_agent.parameters_changed:
            db_interface.update_db()

        timer.checkpt()
        #print(time.time()-t1)