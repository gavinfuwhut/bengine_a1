MotorState[12] motorState
# uint32 tick                   # time reference
# uint8[40] wirelessRemote      # wireless command
uint8 gaitType	                # 0.stand  1.sit  2.trot running  3.climb stair  4.trot obstacle
# Cartesian[4] eeForce          #it's a 1-DOF force infact, but we use 3-DOF here just for visualization 
IMU imu
int16[4] footForce	

Cartesian[4] eeForceRaw
Cartesian[4] eeForce          #it's a 1-DOF force infact, but we use 3-DOF here just for visualization 
Cartesian position            # will delete
Cartesian velocity            # will delete
Cartesian velocity_w            # will delete