# This msg defines the interface to communicate command of the FSM.


# Status control command
uint8 command

# Initial state
uint8 INITIAL=0

# Locked state (motors disabled)
uint8 LOCKED=1

# Hold state (motors enabled but holding position)
uint8 HOLD=2

# Motion state
uint8 MOVE=3

# Free-drag state
uint8 FREEDRAG=4


# Error state
uint8 ERROR=9


# Clear error command
uint8 CLEAR_ERROR=10
