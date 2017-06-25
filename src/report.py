# NIS per sensor vs its 95% percentile.
# MRSE per variable component (compare against ground truth)
# Current error per variable component (compare against ground truth)
# Find the right input data, especially for ground truth
import matplotlib.pyplot as plt

def plot_NIS(x, y, threshold, title):
    fig, ax = plt.subplots()
    line1, = ax.plot(x, y)
    constant=[threshold]*len(x)  # LIDAR would be 9.488
    line2, = ax.plot(x, constant)
    plt.title(title)


if __name__ == '__main__':
    input_fname='../data/obj_pose-laser-radar-synthetic-input.txt'
    with open(input_fname) as input_file:
        lines=input_file.readlines()

    print('Read', len(lines), 'lines from input file', input_fname)

    radar_columns= ['sensor_type', 'rho_measured', 'phi_measured', 'rhodot_measured', 'timestamp', 'x_groundtruth', 'y_groundtruth', 'vx_groundtruth', 'vy_groundtruth', 'yaw_groundtruth', 'yawrate_groundtruth']
    lidar_columns=['sensor_type', 'x_measured', 'y_measured', 'timestamp', 'x_groundtruth', 'y_groundtruth', 'vx_groundtruth', 'vy_groundtruth', 'yaw_groundtruth', 'yawrate_groundtruth']

    gt_entries=[]
    for line in lines:
        labels = radar_columns if line[0]=='R' else lidar_columns
        line = line[:len(line)-1]
        data = {key: value for key, value in zip(radar_columns, line.split('\t'))}
        gt_entries.append(data)

    measures_fname='../data/out.txt'

    with open(measures_fname) as input_file:
        lines=input_file.readlines()

    print('Read', len(lines), 'lines from input file', measures_fname)

    columns= ['x_measured', 'y_measured', 'v_measured', 'yaw_measured', 'yaw_rate_measured', 'nis']

    meas_entries=[]

    for line in lines:
        line = line[:len(line)-1]
        data = {key: value for key, value in zip(columns, line.split('\t'))}
        meas_entries.append(data)

    # Truncate the longest of the two lists to the length of the shortest
    shortest_len=min(len(gt_entries), len(meas_entries))
    gt_entries = gt_entries[:shortest_len]
    meas_entries = meas_entries[:shortest_len]

    nis_radar, nis_lidar =[], []
    x_nis_radar, x_nis_lidar =[], []
    count =1;
    count_above_radar_nis_threshold=0;
    count_above_lidar_nis_threshold = 0;
    radar_nis_threshold = 7.815
    lidar_nis_threshold = 9.488
    for meas_entry, gt_entry in zip(meas_entries, gt_entries):
        nis_value = float(meas_entry['nis'])
        if gt_entry['sensor_type'] == 'R':
            nis_radar.append(nis_value)
            x_nis_radar.append(count)
            if nis_value > radar_nis_threshold:
                count_above_radar_nis_threshold+=1

        else:
            assert gt_entry['sensor_type'] == 'L'
            nis_lidar.append(nis_value)
            x_nis_lidar.append(count)
            if nis_value > lidar_nis_threshold:
                count_above_lidar_nis_threshold+=1
        count +=1;


    print('RADAR measurements with NIS above threshold: {:.1%}'.format(count_above_radar_nis_threshold/len(x_nis_radar)))
    print('LIDAR measurements with NIS above threshold: {:.1%}'.format(count_above_lidar_nis_threshold/len(x_nis_lidar)))

    plot_NIS(x_nis_radar, nis_radar, 7.815, "NIS for RADAR")
    plot_NIS(x_nis_lidar, nis_lidar, 9.488, "NIS for LIDAR")
    plt.show()
