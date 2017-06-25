# NIS, overall but also per sensor? What would the meaning be?
# MRSE per variable component (compare against ground truth)
# Find the right input data, especially for ground truth
import csv;

input_fname='../data/obj_pose-laser-radar-synthetic-input.txt';
telemetry=[];
with open(input_fname) as csv_file:
    reader = csv.reader(csv_file)
    for line in reader:
        telemetry.append(line)
print('Read', len(telemetry), 'lines from input csv file', input_fname)