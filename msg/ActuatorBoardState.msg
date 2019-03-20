# Joint state
float32[8] position
uint8[8] pwm

# Joint status:
uint8[8] status

uint8 STATUS_RESTING = 0
uint8 STATUS_RUNNING = 1
uint8 STATUS_ERROR   = 2
