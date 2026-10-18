# ArUco pallet / location marker estimate from forklift_perception (PC-04, PC-05).
# Pose frame = header.frame_id (map). x axis points INTO the pallet (insertion direction), z up; position is the
# centre of the marker on the pallet face.
std_msgs/Header header
int32 marker_id
string kind              # pallet (ids 1-20, 30 mm) | location (ids 21-40, 60 mm)
geometry_msgs/Pose pose  # moving average of the last `samples` raw estimates
float32 confidence       # 0..1 (reprojection error, apparent size, viewing angle, sample agreement)
uint8 samples            # number of raw samples in the average (<= 5)
float32 distance_m       # camera → marker distance of the latest sample
