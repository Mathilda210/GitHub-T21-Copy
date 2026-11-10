# This msg defines the interface to communicate arm status.

# Robot arm name (e.g. "left_arm")
string arm

# Robot arm status code
uint8 status

# Initial state
uint8 INITIAL=0

# Locked state
uint8 LOCKED=1

# Hold state
uint8 HOLD=2

# Motion state
uint8 MOVE=3

# Free-drag state
uint8 FREEDRAG=4

# Error state
uint8 ERROR=9

# Idle state
uint8 IDLE=10