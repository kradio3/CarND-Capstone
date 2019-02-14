import glob
import csv


labels = [0, 1, 2, 4]
#labels = [1]
ds_dir = './camera_img'
output_file_name = './camera_img/ds.csv'

with open(output_file_name, "wb") as csv_file:
    writer = csv.writer(csv_file, delimiter=',')
    for label in labels:
        label_dir = "{}/{}".format(ds_dir, label)

        label_files = glob.glob("{}/*".format(label_dir))
        for path in label_files:
            writer.writerow([path, label])

