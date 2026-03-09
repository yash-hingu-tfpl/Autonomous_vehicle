import csv

input_file = "/home/developer/ros2_ws/src/golf_cart_nav2/golf_cart_nav2/poses_color.txt"
output_file = "/home/developer/ros2_ws/src/golf_cart_nav2/golf_cart_nav2/recorded_color_path.csv"

with open(input_file, 'r') as fin, open(output_file, 'w', newline='') as fout:
    writer = csv.writer(fout)
    writer.writerow(["x", "y"])

    for line in fin:
        if line.startswith("#") or len(line.strip()) == 0:
            continue

        data = line.split()
        x = float(data[1])
        y = float(data[2])

        writer.writerow([x, y])

print("Path saved to", output_file)
