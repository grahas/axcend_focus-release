uint8 UNKNOWN=0 # The state of the instrument hasn't been set yet
uint8 INITIALIZING=1 # The instrument is initializing
uint8 CALIBRATING=2 # The instrument is calibrating
uint8 IDLE=3 # The instrument is waiting for something to do
uint8 TEMPERATURE_CONTROL=4 # The oven is heating up or cooling down to a set point
uint8 PUMP_POSITIONING=5 # The instrument is positioning the pumps
uint8 READY_TO_TRIGGER=6 # A method is loaded but not running, waiting for the start signal
uint8 TRIGGERING=7 # The instrument received the start signal and is starting the method
uint8 PRESSURIZE_SYSTEM=8 # The instrument is pressurizing the fluidic system
uint8 EQUILIBRATION=9 # The instrument is equilibrating the system
uint8 INJECTION=10 # The instrument is injecting the sample
uint8 RUNNING=11 # The instrument is running the method (isocratic or gradient steps)
uint8 TEARDOWN=12 # The instrument is depressurizing the fluidic system
uint8 COMPLETED=13 # The method has completed
uint8 ERROR=14 # The instrument is in an error state
uint8 ABORTED=15 # The method was aborted
uint8 BUSY=16 # The instrument is busy and cannot accept new commands
uint8 UPDATING=17 # The instrument is updating its firmware
uint8 VALVE_ROTATING=18 # The instrument is rotating the valve
uint8 SOLVENT_EXCHANGE=19 # The instrument is exchanging the solvent

uint8 state