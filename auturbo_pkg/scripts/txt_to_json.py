import json

# Function to read data from txt file and convert to a list of dictionaries
def read_txt_to_json(txt_file_path):
    data = []
    with open(txt_file_path, 'r') as file:
        for line in file:
            # Split the line by whitespace and convert the values to floats
            x, y, z = map(float, line.split())
            # Append the dictionary format to the list
            data.append({"x": x, "y": y, "z": z})
    return data

# Function to write the list of dictionaries to a json file
def write_json(json_data, json_file_path):
    with open(json_file_path, 'w') as json_file:
        json.dump(json_data, json_file, indent=4)
    print(f"Data has been written to {json_file_path}")

# Main execution
if __name__ == '__main__':
    # Path to your txt file
    txt_file_path = '/root/catkin_ws/src/auturbo_pkg/paths/3to3.txt'  # Update with your txt file path
    # Path to output JSON file
    json_file_path = '/root/catkin_ws/src/auturbo_pkg/paths/3to3.json'

    # Read the txt file and convert it to JSON
    json_data = read_txt_to_json(txt_file_path)
    # Write the JSON data to a file
    write_json(json_data, json_file_path)
