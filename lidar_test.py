from controller import Robot
import math
# SETTINGS

TIME_STEP = 32

# In ra dữ liệu sau mỗi bao nhiêu giây
PRINT_INTERVAL = 1.0

# Số tia muốn in ra trong mỗi lần
NUMBER_OF_SAMPLES_TO_PRINT = 12
# ROBOT

robot = Robot()
# LIDAR

lidar = robot.getDevice("lidar")

if lidar is None:
    print("ERROR: Cannot find lidar device.")
    exit()


# Enable LiDAR
lidar.enable(TIME_STEP)


# Enable point cloud
lidar.enablePointCloud()
# INFORMATION

horizontal_resolution = lidar.getHorizontalResolution()
number_of_layers = lidar.getNumberOfLayers()
fov = lidar.getFov()
min_range = lidar.getMinRange()
max_range = lidar.getMaxRange()


print()
print("=" * 70)
print("              LAB 01 - LIDAR RAW SCAN")
print("=" * 70)

print(f"Horizontal resolution : {horizontal_resolution}")
print(f"Number of layers      : {number_of_layers}")
print(f"Field of view         : {math.degrees(fov):.2f} degrees")
print(f"Minimum range         : {min_range:.2f} m")
print(f"Maximum range         : {max_range:.2f} m")
print(f"Sampling period       : {TIME_STEP} ms")

print("=" * 70)
print()
# TIMING

elapsed_time = 0.0

# MAIN LOOP

while robot.step(TIME_STEP) != -1:

    elapsed_time += TIME_STEP / 1000.0

    if elapsed_time < PRINT_INTERVAL:
        continue

    elapsed_time = 0.0
    # GET RAW LIDAR DATA

    ranges = lidar.getRangeImage()

    if ranges is None:
        print("WARNING: LiDAR data is not available yet.")
        continue

    # BASIC CHECK
    print()
    print("-" * 70)
    print("NEW LIDAR SCAN")
    print("-" * 70)

    print(f"Number of readings: {len(ranges)}")
    # FIND MINIMUM DISTANCE

    valid_ranges = [
        value
        for value in ranges
        if math.isfinite(value)
        and value >= min_range
        and value <= max_range
    ]

    if valid_ranges:
        minimum_distance = min(valid_ranges)

        minimum_index = ranges.index(minimum_distance)

        minimum_angle = (minimum_index / horizontal_resolution) * 360.0

        print(f"Minimum distance : {minimum_distance:.3f} m")

        print(f"Minimum angle    : {minimum_angle:.2f} degrees")
    # PRINT 12 REPRESENTATIVE RAYS

    print()
    print(" Index       Angle        Distance")


    for i in range(NUMBER_OF_SAMPLES_TO_PRINT):

        index = int(i * horizontal_resolution/ NUMBER_OF_SAMPLES_TO_PRINT)

        distance = ranges[index]

        angle_deg = (index / horizontal_resolution) * 360.0

        if math.isfinite(distance):

            print(
                f"{index:5d}   "
                f"{angle_deg:8.2f} deg   "
                f"{distance:8.3f} m"
            )

        else:

            print(
                f"{index:5d}   "
                f"{angle_deg:8.2f} deg   "
                f"      INF"
            )

    # CONVERT SELECTED RAYS TO X/Y

    print()
    print("Cartesian coordinates:")
    print(
        " Angle       Distance          X          Y"
    )

    # Choose 8 directions
    selected_angles = [
        0,
        45,
        90,
        135,
        180,
        225,
        270,
        315
    ]

    for angle_deg in selected_angles:

        index = int(angle_deg / 360.0* horizontal_resolution)

        distance = ranges[index]

        if not math.isfinite(distance):
            continue

        angle_rad = math.radians(angle_deg)

        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)

        print(
            f"{angle_deg:5.0f} deg   "
            f"{distance:8.3f} m   "
            f"{x:8.3f}   "
            f"{y:8.3f}"
        )

    # POINT CLOUD INFORMATION

    try:

        points = lidar.getPointCloud()

        if points:

            print()
            print(f"Point cloud points: {len(points)}")

            # Show first point
            first_point = points[0]

            print("First point:")

            print(f"  X = {first_point.x:.3f} m")

            print(f"  Y = {first_point.y:.3f} m")

            print(f"  Z = {first_point.z:.3f} m")

    except Exception as error:

        print("Point cloud unavailable:",error)