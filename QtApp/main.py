import sys
import threading

from ui_interface import *
from PySide6.QtWidgets import QMainWindow
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtUiTools import QUiLoader
from PySide6.QtGui import QShortcut, QKeySequence
from Custom_Widgets.Widgets import *
from PySide6.QtCore import Slot, QTimer
from QuadcopterIntegration.Utilities.comm_definitions import commands
import folium
from folium import plugins
from pglive.sources.data_connector import DataConnector
from pglive.sources.live_plot import LiveLinePlot
from pglive.sources.live_plot_widget import LivePlotWidget
import io
from threading import Thread
from datetime import datetime
import time
from Factories.CommunicationFactory.Telemetry.telemetry_manager import MQTT_TelemetryManagerGCS
from Factories.CommunicationFactory.Telemetry.mappings import AUXILIARY_COMMANDS_MAPPING, FLIGHT_MODES_MAPPING
from Factories.CommunicationFactory.Telemetry.lidia_telemetry_sender import LidiaTelemetrySender
from Factories.DataManagementFactory.data_writer import DataWriterThread
from Factories.DataManagementFactory.DataWriterConfigurations.online_writer_configuration import DATA_TO_WRITE_GCS, FIELDNAMES_TELEMETRY_NAMES_MAPPING
from Multiprocessing.PARAMS import MQTT_HOST, MQTT_PORT
from Factories.CommunicationFactory.Telemetry.subscriptions import GCS_SUBS
class MainWindow(QMainWindow):
    def __init__(self,
                 telemetry_manager,
                 parent=None):
        QMainWindow.__init__(self)

        # memorize serial connection

        # telemetry
        self.telemetry_manager = telemetry_manager
        self.telemetry = telemetry_manager.telemetry

        # readings update timer
        self.readings_timer = QTimer()
        self.readings_timer.setInterval(100)
        self.readings_timer.start()
        self.readings_timer.timeout.connect(self.update_readings)

        #load UI
        loader = QUiLoader()
        loader.registerCustomWidget(QCustomSlideMenu)
        loader.registerCustomWidget(QCustomStackedWidget)
        loader.registerCustomWidget(LivePlotWidget)
        #print(loader.availableWidgets())
        self.window = loader.load("interface.ui", self)
        #connect signals
        #self.window.mainButton.clicked.connect(self.test)


        #read style
        loadJsonStyle(self, self.window)
        # folium map

        #self.nav_icon = folium.DivIcon()
        # self.coordinate = (54.258978, 18.664669)
        # self.heading = 0.0
        # data = self.create_byte_map(zoom=60)
        # self.webView = QWebEngineView()
        # self.webView.setHtml(data.getvalue().decode())
        # self.webView.setParent(self.window.map_frame)

        # Flight instruments
        self.lidia_telemetry = LidiaTelemetrySender()
        # self.webView = QWebEngineView()
        # self.webView.setParent(self.window.flight_instrumentsContainer)
        # self.webView.setUrl(QtCore.QUrl("http://localhost:5555/pfd"))
        # RT plots embedding
        self.live_plot_widgets = [self.window.x_plot, self.window.y_plot, self.window.z_plot,
                             self.window.Vx_plot, self.window.Vy_plot, self.window.Vz_plot,
                             self.window.estimation_F_plot, self.window.estimation_phi_plot, self.window.estimation_theta_plot,
                             self.window.u_ref_F_plot, self.window.u_ref_phi_plot, self.window.u_ref_theta_plot,
                             self.window.output_f_plot, self.window.output_phi_plot, self.window.output_theta_plot,
                             self.window.throttle_plot]
        self.telemetry_to_plot = [('position_local', 0), ('position_local', 1), ('position_local', 2),
                                  ('velocity', 0), ('velocity', 1), ('velocity', 2),
                                  ('u_l1', 0), ('u_l1', 1), ('u_l1', 2),
                                  ('u', 0), ('u', 1), ('u', 2),
                                  ('u_output', 0), ('u_output', 1), ('u_output', 2),
                                  'throttle']
        self.data_connectors = []
        self.telemetry_updated_event = threading.Event()
        for live_plot in self.live_plot_widgets:
            plot_curve = LiveLinePlot()
            live_plot.addItem(plot_curve)
            self.data_connectors.append(DataConnector(plot_curve, max_points=300, update_rate=50))

        # expand menu
        self.window.HomeButton.clicked.connect(self.window.centralMenuContainer.expandMenu)
        self.window.ConnectionButton.clicked.connect(self.window.centralMenuContainer.expandMenu)
        self.window.ControlsButton.clicked.connect(self.window.centralMenuContainer.expandMenu)

        #close menu
        self.window.closeMenuBtn.clicked.connect(self.window.centralMenuContainer.collapseMenu)

        #arm/disarm vehicle
        self.window.armBtn.clicked.connect(lambda: self.arm_disarm_vehicle("arm"))
        self.window.disarmBtn.clicked.connect(lambda: self.arm_disarm_vehicle("disarm"))

        #change mode
        # self.window.stabilizeButton.clicked.connect(lambda: self.change_flight_mode("STABILIZE"))
        # self.window.guidedButton.clicked.connect(lambda: self.change_flight_mode("GUIDED"))
        # self.window.acroButton.clicked.connect(lambda: self.change_flight_mode("ACRO"))

        # change reference value
        self.window.confirmReference.clicked.connect(lambda: self.change_setpoint('single_point'))

        # set square reference
        self.window.setSquareRefButton.clicked.connect(lambda: self.change_setpoint('square'))

        # RETURN TO LAUNCH
        self.window.returnToLaunchBtn.clicked.connect(lambda: self.auxiliary_command('RETURN_TO_LAUNCH'))
        self.window.landButton.clicked.connect(lambda: self.auxiliary_command('LAND'))
        self.window.takeoffBtn.clicked.connect(lambda: self.auxiliary_command('TAKEOFF'))

        # CONTROLLERS ON OFF
        self.window.MPC_ON_OFF_BTN.setCheckable(True)
        self.window.ADA_ON_OFF_BTN.setCheckable(True)
        self.window.ESTIM_ON_OFF_BTN.setCheckable(True)
        self.window.MPC_ON_OFF_BTN.clicked.connect(lambda: self.on_off_controllers('MPC'))
        self.window.ADA_ON_OFF_BTN.clicked.connect(lambda: self.on_off_controllers('ADAPTIVE'))
        self.window.ESTIM_ON_OFF_BTN.clicked.connect(lambda: self.on_off_controllers('ESTIMATOR'))
        self.window.MPC_ON_OFF_BTN.setText("OFF")
        self.window.MPC_ON_OFF_BTN.setStyleSheet("QPushButton {background-color: lightcoral}")
        self.window.ADA_ON_OFF_BTN.setText("OFF")
        self.window.ADA_ON_OFF_BTN.setStyleSheet("QPushButton {background-color: lightcoral}")
        self.window.ESTIM_ON_OFF_BTN.setText("OFF")
        self.window.ESTIM_ON_OFF_BTN.setStyleSheet("QPushButton {background-color: lightcoral}")

        # ESTIMATION ACCEPT BUTTON
        self.window.ESTIM_ACCEPT_BTN.setText("ACCEPT")
        self.window.ESTIM_ACCEPT_BTN.setStyleSheet("QPushButton {background-color: lightgreen}")
        self.window.ESTIM_ACCEPT_BTN.clicked.connect(lambda: self.auxiliary_command('ACCEPT_ESTIMATION'))

        # IDENTIFICATION_PROCEDURE
        self.window.activateIdentificationBtn.setCheckable(True)
        self.window.activateIdentificationBtn.setText("OFF")
        self.window.activateIdentificationBtn.setStyleSheet("QPushButton {background-color: lightcoral}")
        self.window.activateIdentificationBtn.clicked.connect(self.on_off_estimation_procedure)

        # data writer
        self.data_writer = DataWriterThread(DATA_TO_WRITE_GCS, path='../logs/')
        self.window.saveDataBtn.setCheckable(True)
        self.window.saveDataBtn.clicked.connect(lambda: self.serve_data_writer())
        self.window.saveDataBtn.setText("Saving OFF")
        self.window.saveDataBtn.setStyleSheet("QPushButton {background-color: lightcoral}")
        self.window.dataWritingStatus.setText("DATA NO WRITING")
        self.window.dataWritingStatus.setStyleSheet("QLabel {background-color: lightcoral}")
        self.window.remoteDataWritingStatus.setText("DATA NO WRITING")
        self.window.remoteDataWritingStatus.setStyleSheet("QLabel {background-color: lightcoral}")
        self.record_data_shortcut = QShortcut(QKeySequence('Ctrl+5'), self)
        self.stop_data_shortcut = QShortcut(QKeySequence('Ctrl+6'), self)
        self.record_data_shortcut.activated.connect(lambda: self.shortcut_data_write('start'))
        self.stop_data_shortcut.activated.connect(lambda: self.shortcut_data_write('stop'))

        # STATUS
        self.window.batteryPixmap.setStyleSheet("color: white")
        # show
        self.show()


    @Slot()
    def arm_disarm_vehicle(self, mode):
        if mode == "arm":
            self.telemetry_manager.publish('ARM_DISARM', 1)
            print("Arming command sent..")
        elif mode == "disarm":
            self.telemetry_manager.publish('ARM_DISARM', 0)
            print("Disarming command sent..")

    @Slot()
    def readTelemetry(self):
        if self.read_telemetry.telemetry_set_event.is_set():
            if self.read_telemetry.telemetry is None:
                return
            else:
                self.telemetry = self.read_telemetry.telemetry
                self.telemetry_updated_event.set()

    @Slot()
    def update_readings(self):
        if self.telemetry is None:
            return
        if self.telemetry['attitude'] is not None:
            self.window.pitch.setText(str(self.telemetry['attitude'][0]))
            self.window.roll.setText(str(self.telemetry['attitude'][1]))
            self.window.yaw.setText(str(self.telemetry['attitude'][2]))
        else:
            self.window.pitch.setText("No data")
            self.window.roll.setText("No data")
            self.window.yaw.setText("No data")
        # if self.telemetry['position_global'] is not None or \
        #         self.telemetry['heading'] is not None:
        #     if self.telemetry['position_global'] is not None:
        #         (lat, lon) = self.telemetry['position_global']
        #         self.coordinate = (lat, lon)
        #     if self.telemetry['heading'] is not None:
        #         self.heading = self.telemetry['heading']
        #     data = self.create_byte_map(60)
        #     self.webView.setHtml(data.getvalue().decode())
        if self.telemetry['flight_mode'] is not None:
            self.window.flightModeStatus.setText(str(FLIGHT_MODES_MAPPING[self.telemetry['flight_mode']]) + " " + "MODE")
        if self.telemetry['bat_voltage'] is not None:
            self.window.batteryVoltage.setText(
                ("{:.2f}" + " " + "V").format(self.telemetry['bat_voltage']))
        if self.telemetry['bat_current'] is not None:
            self.window.batteryCurrent.setText(
                ("{:.2f}" + " " + "A").format(self.telemetry['bat_current']))
        if self.telemetry['estimated_mass'] is not None:
            self.window.estimated_mass.setText(
                ("{:.2f}" + " " + "kg").format(self.telemetry['estimated_mass']))
        if self.data_writer.writing_event.is_set():
            self.data_writer.data = self.telemetry
            self.data_writer.data_set.set()
        if self.data_writer.writing_ok:
            self.window.dataWritingStatus.setText("DATA WRITING OK")
            self.window.dataWritingStatus.setStyleSheet("QLabel {background-color: lightgreen}")
        else:
            self.window.dataWritingStatus.setText("DATA NO WRITING")
            self.window.dataWritingStatus.setStyleSheet("QLabel {background-color: lightcoral}")
        if self.telemetry['telem_writing_ok']:
            self.window.remoteDataWritingStatus.setText("REMOTE:DATA WRITING OK")
            self.window.remoteDataWritingStatus.setStyleSheet("QLabel {background-color: lightgreen}")
        else:
            self.window.remoteDataWritingStatus.setText("REMOTE:DATA NO WRITING")
            self.window.remoteDataWritingStatus.setStyleSheet("QLabel {background-color: lightcoral}")
        self.lidia_telemetry(self.telemetry)

    @Slot()
    def change_setpoint(self, setpoint_type):
        if setpoint_type == 'single_point':
            self.telemetry_manager.publish('SET_SPIRAL_SETPOINT:X', self.window.x_ref.value())
            self.telemetry_manager.publish('SET_SPIRAL_SETPOINT:Y', self.window.y_ref.value())
            self.telemetry_manager.publish('SET_SPIRAL_SETPOINT:Z', self.window.z_ref.value())
        elif setpoint_type == 'square':
            self.telemetry_manager.publish('SET_SQUARE_SETPOINT', self.window.square_setpoint_length.value())
    @Slot()
    def auxiliary_command(self, comm_name):
        comm_code = AUXILIARY_COMMANDS_MAPPING[comm_name]
        self.telemetry_manager.publish('AUXILIARY_COMMAND', comm_code)

    @Slot()
    def on_off_controllers(self, controller_type):
        if controller_type == "MPC":
            if self.window.MPC_ON_OFF_BTN.isChecked():
                self.window.MPC_ON_OFF_BTN.setText("ON")
                self.window.MPC_ON_OFF_BTN.setStyleSheet("QPushButton {background-color: lightgreen}")
                self.telemetry_manager.publish('POSITION_CONTROLLER_ON_OFF', 1)
            else:
                self.window.MPC_ON_OFF_BTN.setText("OFF")
                self.window.MPC_ON_OFF_BTN.setStyleSheet("QPushButton {background-color: lightcoral}")
                self.telemetry_manager.publish('POSITION_CONTROLLER_ON_OFF', 0)
        elif controller_type == "ADAPTIVE":
            if self.window.ADA_ON_OFF_BTN.isChecked():
                self.window.ADA_ON_OFF_BTN.setText("ON")
                self.window.ADA_ON_OFF_BTN.setStyleSheet("QPushButton {background-color: lightgreen}")
                self.telemetry_manager.publish('ADAPTIVE_CONTROLLER_ON_OFF', 1)
            else:
                self.window.ADA_ON_OFF_BTN.setText("OFF")
                self.window.ADA_ON_OFF_BTN.setStyleSheet("QPushButton {background-color: lightcoral}")
                self.telemetry_manager.publish('ADAPTIVE_CONTROLLER_ON_OFF', 0)
        elif controller_type == "ESTIMATOR":
            if self.window.ESTIM_ON_OFF_BTN.isChecked():
                self.window.ESTIM_ON_OFF_BTN.setText("ON")
                self.window.ESTIM_ON_OFF_BTN.setStyleSheet("QPushButton {background-color: lightgreen}")
                self.telemetry_manager.publish('ESTIMATOR_ON_OFF', 1)
            else:
                self.window.ESTIM_ON_OFF_BTN.setText("OFF")
                self.window.ESTIM_ON_OFF_BTN.setStyleSheet("QPushButton {background-color: lightcoral}")
                self.telemetry_manager.publish('ESTIMATOR_ON_OFF', 0)

    @Slot()
    def on_off_estimation_procedure(self):
        if self.window.activateIdentificationBtn.isChecked():
            self.window.activateIdentificationBtn.setText("ON")
            self.window.activateIdentificationBtn.setStyleSheet("QPushButton {background-color: lightgreen}")
            self.telemetry_manager.publish('IDENTIFICATION_THROTTLE', self.window.identification_throttle_value.value())
        else:
            self.window.activateIdentificationBtn.setText("OFF")
            self.window.activateIdentificationBtn.setStyleSheet("QPushButton {background-color: lightcoral}")
            self.telemetry_manager.publish('IDENTIFICATION_THROTTLE', -1.0)

    @Slot()
    def serve_data_writer(self):
        if self.window.saveDataBtn.isChecked():
            self.window.saveDataBtn.setText("DATA SAVING")
            self.window.saveDataBtn.setStyleSheet("QPushButton {background-color: lightgreen}")
            filename = self.window.dataFilenameTextbox.toPlainText()
            self.data_writer.filename = filename
            self.data_writer.writing_event.set()
            self.telemetry_manager.publish('DATA_WRITE', 'S_{}RPI'.format(filename))
        else:
            self.window.saveDataBtn.setText("SAVING OFF")
            self.window.saveDataBtn.setStyleSheet("QPushButton {background-color: lightcoral}")
            self.telemetry_manager.publish('DATA_WRITE', 'R')
            self.data_writer.writing_event.clear()

    @Slot()
    def shortcut_data_write(self, mode):
        if mode == 'start':
            self.window.saveDataBtn.setText("DATA SAVING")
            self.window.saveDataBtn.setStyleSheet("QPushButton {background-color: lightgreen}")
            filename = self.window.dataFilenameTextbox.toPlainText()
            self.data_writer.filename = filename
            self.data_writer.writing_event.set()
            self.telemetry_manager.publish('DATA_WRITE', 'S_{}RPI'.format(filename))
        elif mode == 'stop':
            self.window.saveDataBtn.setText("SAVING OFF")
            self.window.saveDataBtn.setStyleSheet("QPushButton {background-color: lightcoral}")
            self.telemetry_manager.publish('DATA_WRITE', 'R')
            self.data_writer.writing_event.clear()

    def change_flight_mode(self, mode):
        self.serial_connection.write(commands[mode])
    def create_byte_map(self, zoom):
        map = folium.Map(
            title='Quad Location',
            zoom_start=zoom,
            location=self.coordinate
        )
        folium.Marker(
            location=self.coordinate,
            icon=folium.plugins.BeautifyIcon(icon='location-arrow',
                                      border_color='transparent',
                                      background_color='transparent',
                                      border_width=1,
                                      text_color='#003EFF',
                                      inner_icon_style='margin:0px;font-size:4em;transform: rotate({0}deg);'.format(
                                          self.heading))
        ).add_to(map)
        data = io.BytesIO()
        map.save(data, close_file=False)
        return data

    def update_plots(self, *connectors):
        start = time.time()
        while True:
            x = time.time()
            available_telemetry = self.telemetry.keys()
            if self.telemetry_manager.telemetry_set_event.is_set():
                for i, connector in enumerate(connectors):
                    data = None
                    telem_index = self.telemetry_to_plot[i]
                    if not isinstance(telem_index, tuple):
                        if telem_index not in available_telemetry:
                            continue
                        data = self.telemetry[telem_index]
                    elif len(telem_index) == 2:
                        if telem_index[0] not in available_telemetry:
                            continue
                        data = self.telemetry[telem_index[0]][telem_index[1]]
                    if data is not None:
                        connector.cb_append_data_point(data, x-start)
                self.telemetry_manager.telemetry_set_event.clear()
            time.sleep(0.1)



if __name__ == "__main__":
    tm = MQTT_TelemetryManagerGCS(MQTT_HOST,
                                  MQTT_PORT,
                                  update_freq=1000,
                                  subscribed_comms=GCS_SUBS)
    app = QApplication(sys.argv)
    window = MainWindow(tm)
    Thread(target=window.update_plots, args=window.data_connectors).start()
    sys.exit(app.exec())