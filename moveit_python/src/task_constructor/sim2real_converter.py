#!/usr/bin/env python3
# read json file and then remove all names with "sim" save the new json file with name adding "_real"import json
import json
import os

# Step 1: Read the JSON file
with open('your_file.json', 'r') as file:
    data = json.load(file)

# Step 2: Filter the data
filtered_data = [item for item in data if "sim" not in item['name']]

# Step 3: Save the modified data to a new file
with open('your_file_real.json', 'w') as file:
    json.dump(filtered_data, file, indent=4)
