import struct
import csv

# Define the struct format
data_struct_format = '1l 1L 4f 3f 3f 4f'
gps_struct_format = '2H 1I 3H 1I'

# Calculate the size of the struct in bytes
struct_size = struct.calcsize(data_struct_format)
gps_struct_size = struct.calcsize(gps_struct_format)

# Define a function to decode the binary file and write to CSV
def decode_and_write_to_csv(file_path, output_csv_path):
    with open(file_path, 'rb') as input_file, open(output_csv_path, 'w', newline='') as output_csv:
        # Create a CSV writer
        csv_writer = csv.writer(output_csv)

        # Write header to CSV
        header = [
            'Time', 'Status', 'Pressure', 'Temperature', 'BMP Altitude', 'Max Altitude',
            'Accel_X', 'Accel_Y', 'Accel_Z', 'Rotation_X', 'Rotation_Y', 'Rotation_Z',
            'Latitude', 'Longitude', 'GPS Altitude', 'Horizontal Speed'
        ]
        csv_writer.writerow(header)

        while True:
            # Read a chunk of data from the file
            data_chunk = input_file.read(struct_size)

            # Break if no more data is available
            if not data_chunk:
                break

            # Check if there is enough data to unpack
            if len(data_chunk) >= struct_size:
                # Unpack the data using the struct format
                decoded_entry = struct.unpack(data_struct_format, data_chunk)

                # Write the decoded entry to the CSV file
                csv_writer.writerow(decoded_entry)
            elif len(data_chunk) >= gps_struct_size:
                # Unpack the data using the struct format
                decoded_entry = struct.unpack(gps_struct_format, data_chunk)

                # Write the decoded entry to the CSV file
                csv_writer.writerow(decoded_entry)
            else:
                print("Incomplete data found. Skipping the last incomplete entry.")

# Example usage
input_file_path = 'log6.bin'                # Replace with the actual file path
output_csv_path = 'output_data.csv'         # Replace with the desired output CSV file path
decode_and_write_to_csv(input_file_path, output_csv_path)