#!/usr/bin/env python3
# read the json file and check it through errors.

import json
import os

# Function to parse and validate JSON
def parse_json_file(file_path):
    try:
        with open(file_path, 'r') as file:
            json.load(file)
        print(f"{file_path} is valid JSON.")
    except ValueError as e:
        print(f"{file_path} is not valid JSON: {e}")

# Directory containing JSON files
directory = '/path/to/your/json/files'

# List to hold invalid JSON files
invalid_files = []

# Iterate over files in the directory
for filename in os.listdir(directory):
    if filename.endswith('.json'):
        file_path = os.path.join(directory, filename)
        parse_json_file(file_path)
