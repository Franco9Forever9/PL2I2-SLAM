Header Header # cloud_info header

int32[] startRingIndex
int32[] endRingIndex

int32[] pointColInd # point column index in a range image
float32[] pointRange # point range

int64 imuAvailable
int64 odomAvailable

float32 imuRollInit # init roll orientation
float32 imuPitchInit # init pitch orientation
float32 imuYawInit # init yaw orientation

float32 odomX # odometry X
float32 odomY # odometry Y
float32 odomZ # odometry Z
float32 odomRoll # odometry Roll
float32 odomPitch # odometry Pitch
float32 odomYaw # odometry Yaw

int64 odomResetId

sensor_msgs/PointCloud2 cloud_deskewed
sensor_msgs/PointCloud2 cloud_corner
sensor_msgs/PointCloud2 cloud_surface