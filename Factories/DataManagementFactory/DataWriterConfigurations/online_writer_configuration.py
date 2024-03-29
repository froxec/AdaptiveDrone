from Factories.ToolsFactory.GeneralTools import BidirectionalDict

DATA_TO_WRITE_GCS = ['TIME', 'POSITION_LOCAL:X', 'POSITION_LOCAL:Y', 'POSITION_LOCAL:Z',
                     'VELOCITY:X', 'VELOCITY:Y', 'VELOCITY:Z',
                     'ATTITUDE:X', 'ATTITUDE:Y', 'ATTITUDE:Z',
                     'u_l1:X', 'u_l1:Y', 'u_l1:Z',
                     'sigma_hat:X', 'sigma_hat:Y', 'sigma_hat:Z',
                     'U_OUTPUT:X', 'U_OUTPUT:Y', 'U_OUTPUT:Z',
                     'U_REF:X', 'U_REF:Y', 'U_REF:Z',
                     'HEADING', 'FLIGHT_MODE', 'BATTERY_VOLTAGE', 'BATTERY_CURRENT']

# FIELDNAMES TO TELEMETRY DICT NAMES MAPPING

FIELDNAMES_TELEMETRY_NAMES_MAPPING = BidirectionalDict()
FIELDNAMES_TELEMETRY_NAMES_MAPPING['POSITION_LOCAL:X'] = ('position_local', 0)
FIELDNAMES_TELEMETRY_NAMES_MAPPING['POSITION_LOCAL:Y'] = ('position_local', 1)
FIELDNAMES_TELEMETRY_NAMES_MAPPING['POSITION_LOCAL:Z'] = ('position_local', 2)

# FIELDNAMES_TELEMETRY_NAMES_MAPPING['POSITION_GLOBAL:X'] = ('position_global', 0)
# FIELDNAMES_TELEMETRY_NAMES_MAPPING['POSITION_GLOBAL:Y'] = ('position_global', 1)
# FIELDNAMES_TELEMETRY_NAMES_MAPPING['POSITION_GLOBAL:Z'] = ('position_global', 2)

FIELDNAMES_TELEMETRY_NAMES_MAPPING['VELOCITY:X'] = ('velocity', 0)
FIELDNAMES_TELEMETRY_NAMES_MAPPING['VELOCITY:Y'] = ('velocity', 1)
FIELDNAMES_TELEMETRY_NAMES_MAPPING['VELOCITY:Z'] = ('velocity', 2)

FIELDNAMES_TELEMETRY_NAMES_MAPPING['ATTITUDE:X'] = ('attitude', 0)
FIELDNAMES_TELEMETRY_NAMES_MAPPING['ATTITUDE:Y'] = ('attitude', 1)
FIELDNAMES_TELEMETRY_NAMES_MAPPING['ATTITUDE:Z'] = ('attitude', 2)

FIELDNAMES_TELEMETRY_NAMES_MAPPING['u_l1:X'] = ('u_l1', 0)
FIELDNAMES_TELEMETRY_NAMES_MAPPING['u_l1:Y'] = ('u_l1', 1)
FIELDNAMES_TELEMETRY_NAMES_MAPPING['u_l1:Z'] = ('u_l1', 2)

FIELDNAMES_TELEMETRY_NAMES_MAPPING['sigma_hat:X'] = ('sigma_hat', 0)
FIELDNAMES_TELEMETRY_NAMES_MAPPING['sigma_hat:Y'] = ('sigma_hat', 1)
FIELDNAMES_TELEMETRY_NAMES_MAPPING['sigma_hat:Z'] = ('sigma_hat', 2)

FIELDNAMES_TELEMETRY_NAMES_MAPPING['U_OUTPUT:X'] = ('u_output', 0)
FIELDNAMES_TELEMETRY_NAMES_MAPPING['U_OUTPUT:Y'] = ('u_output', 1)
FIELDNAMES_TELEMETRY_NAMES_MAPPING['U_OUTPUT:Z'] = ('u_output', 2)

FIELDNAMES_TELEMETRY_NAMES_MAPPING['U_REF:X'] = ('u', 0)
FIELDNAMES_TELEMETRY_NAMES_MAPPING['U_REF:Y'] = ('u', 1)
FIELDNAMES_TELEMETRY_NAMES_MAPPING['U_REF:Z'] = ('u', 2)

FIELDNAMES_TELEMETRY_NAMES_MAPPING['HEADING'] = 'heading'
FIELDNAMES_TELEMETRY_NAMES_MAPPING['FLIGHT_MODE'] = 'flight_mode'
FIELDNAMES_TELEMETRY_NAMES_MAPPING['BATTERY_VOLTAGE'] = 'bat_voltage'
FIELDNAMES_TELEMETRY_NAMES_MAPPING['BATTERY_CURRENT'] = 'bat_current'

DATA_TO_WRITE_PI = DATA_TO_WRITE_GCS