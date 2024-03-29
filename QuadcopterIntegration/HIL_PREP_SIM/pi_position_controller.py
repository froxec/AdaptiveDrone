from Factories.ControllersFactory.position_controllers.mpc import ModelPredictiveControl
from Factories.ModelsFactory.linear_models import AugmentedLinearizedQuadNoYaw, LinearizedQuadNoYaw
from Factories.ModelsFactory.model_parameters import Z550_parameters
from Factories.ToolsFactory.Converters import MPC_input_converter, MPC_output_converter
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SinglePoint, SpiralTrajectory
from QuadcopterIntegration.HIL_PREP_SIM.communication_parameters import *
from Factories.ControllersFactory.position_controllers.position_controller import PositionController
from Factories.CommunicationFactory.interfaces import ControllerInterface
from QuadcopterIntegration.HIL_PREP_SIM.simulation_parameters import *
from Factories.ToolsFactory.GeneralTools import time_control
import numpy as np
import time

@time_control
def run_controller(controller):
    controller()


trajectory = SinglePoint([0, 0, 0])

parameters = Z550_parameters
if __name__ == "__main__":
    print("Initialisation started...")
    #prediction_model = AugmentedLinearizedQuadNoYaw(parameters, Ts=1/OUTER_LOOP_FREQ)
    prediction_model = LinearizedQuadNoYaw(parameters, Ts=1/OUTER_LOOP_FREQ)
    mpc = ModelPredictiveControl(prediction_model, OUTER_LOOP_FREQ, pred_horizon=10)
    input_converter = MPC_input_converter(np.zeros(6), np.array([parameters['m']*parameters['g'], 0, 0]))
    output_converter = MPC_output_converter(np.array([parameters['m']*parameters['g'], 0, 0]), parameters['Kt'], ANGULAR_VELOCITY_RANGE, mode='proprietary')
    interface = ControllerInterface(RASP_IP, RASP_PORT, PC_IP, PC_PORT)
    position_controller = PositionController(mpc,
                                             input_converter,
                                             output_converter,
                                             trajectory, 
                                             interface)
    print("Initialisation finished.")
    while True:
        run_controller(position_controller, 1/OUTER_LOOP_FREQ)