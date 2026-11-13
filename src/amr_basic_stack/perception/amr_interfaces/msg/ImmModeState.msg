# One posterior component of the common-state CV/CT/STATIC IMM.
uint8 MODEL_CV = 0
uint8 MODEL_CT = 1
uint8 MODEL_STATIC = 2
uint8 model_type
float64 probability

# Common state [px, py, vx, vy, omega] in the parent array's frame.
float64[5] state

# Row-major covariance of the common 5-D state.
float64[25] covariance
