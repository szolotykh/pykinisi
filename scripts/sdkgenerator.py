import argparse
import datetime
import json
import os

# Map of type names to their size in bytes
type_to_size_map = {
    "bool": 1,
    "uint8_t": 1,
    "uint16_t": 2,
    "uint32_t": 4,
    "int8_t": 1,
    "int16_t": 2,
    "int32_t": 4,
    "double": 8,
    "integer": 4,
}

def file_header(version, comment_char="//"):
    return f"""{comment_char} ----------------------------------------------------------------------------
{comment_char} Kinisi motr controller commands.
{comment_char} This file is auto generated by the commands generator from JSON file.
{comment_char} Do not edit this file manually.
{comment_char} Timestamp: { datetime.datetime.now().strftime("%Y-%m-%d %H:%M:00") }
{comment_char} Version: {version}
{comment_char} ----------------------------------------------------------------------------\n\n"""

# Function to validate names for various categories like 'Type', 'Command', etc.
# 'name' is the name to be validated, 'category' specifies the category, and 'context' provides additional information.
def validate_name(name, category, context):
    errors = []
    # Check if the name contains a space
    if " " in name:
        errors.append(f"{category} name '{name}' in {context} contains a space.")
    return errors


# Function to validate a command in the JSON
# 'command' is a dictionary containing the command details
def validate_command(command):
    errors = []
    # Validate the name of the command
    errors.extend(validate_name(command['command'], 'Command', 'commands'))
    # Validate the names and ranges of properties for the command
    for prop in command.get("properties", []):
        errors.extend(validate_name(prop['name'], 'Property', f"command '{command['command']}'"))
        # Check if a range exists and if so, validate it
        if "range" in prop:
            if not isinstance(prop["range"], list) or len(prop["range"]) != 2 or prop["range"][0] > prop["range"][1]:
                errors.append(f"Invalid range '{prop['range']}' in property '{prop['name']}' for command '{command['command']}'.")
    return errors

# Main function to validate the entire JSON
# 'commands_data' is a dictionary containing the JSON data
def validate_json(commands_data):
    errors = []

    # Validate each command in the JSON
    for command in commands_data.get("commands", []):
        errors.extend(validate_command(command))
    return errors

# Generates Python code from the commands JSON file
def generate_python_code(commands_data):

    type_mapping = {
        'uint8_t': 'int',
        'uint16_t': 'int',
        'uint32_t': 'int',
        'double': 'float',
        'int8_t': 'int',
        'int16_t': 'int',
        'int32_t': 'int',
        'bool': 'bool',
    }

    # Initialize code strings
    constant_code = ''
    function_code = ''
    class_code = '''
class KinisiCommands:
    # Write command to serial interface
    def write(self, msg: bytearray):
        """Abstract method to write the byte message to the serial interface."""
        raise NotImplementedError("This method should be overridden by subclass.")
    
    # Read command from serial interface
    def read(self, length: int) -> bytearray:
        """Abstract method to read a specified number of bytes from the serial interface."""
        raise NotImplementedError("This method should be overridden by subclass.")

    '''
    
    # Generate constants for command codes
    for cmd in commands_data['commands']:
        constant_code += f"{cmd['command']} = {cmd['code']}\n"
    constant_code += "\n"


    # Generate function for each command
    for cmd in commands_data['commands']:
        func_body = f"    # {cmd['description']}\n"
        func_args = ", ".join([f"{prop['name']}:{type_mapping[prop['type']]}" for prop in cmd.get('properties', [])])
        func_body += f"    def {cmd['command'].lower()}(self{', ' if len(func_args) > 0 else ''}{func_args}):\n"
        func_body += f"        msg = {cmd['command']}.to_bytes(1, 'little')"
        for prop in cmd.get('properties', []):
            if prop['type'] == 'double':
                func_body += f" + bytearray(struct.pack('d', {prop['name']}))"
            else:
                signed = ", signed=True" if prop['type'].startswith("int") else ""
                func_body += f" + {prop['name']}.to_bytes({type_to_size_map[prop['type']]}, 'little'{signed})"

        func_body += "\n"
        func_body += f"        length = len(msg)\n"
        func_body += f"        msg = length.to_bytes(1, 'little') + msg\n"
        func_body += f"        self.write(msg)\n"
        if 'response' in cmd:
            func_body += f"        result =  self.read({type_to_size_map[cmd['response']['type']]})\n"
            if cmd['response']['type'] == 'double':
                func_body += f"        return struct.unpack('<d', result)[0]\n"
            else:
                func_body += f"        return {type_mapping[cmd['response']['type']]}.from_bytes(result, 'little')\n"
        function_code += func_body
        function_code += "\n"

    class_code += function_code

    result = file_header(commands_data['version'], "#")
    result += "import struct\n\n"
    result += constant_code
    result += class_code

    return result


# Generate python code from the commands from the input JSON file
# input_json_path: Path to the input JSON file containing command definitions.
# output_path: Path to the output file.
def generate(input_json_path, output_path):
    try:
        with open(input_json_path, 'r') as file:
            commands_data = json.load(file)
    except json.JSONDecodeError:
        print("Error: Could not decode the input JSON file.")
        return
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return

    validation_errors = validate_json(commands_data)
    if validation_errors:
        print("Validation errors found:")
        for error in validation_errors:
            print(f"- {error}")
        return


    generated_code = generate_python_code(commands_data)

    try:
        with open(output_path, 'w') as file:
            file.write(generated_code)
    except Exception as e:
        print(f"An error occurred while writing to the output file: {e}")
        return

    print(f"Code has been generated and saved to {output_path}")