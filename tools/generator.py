import datetime
import json

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

type_mapping = {
    'uint8_t': 'int',
    'uint16_t': 'int',
    'uint32_t': 'int',
    'double': 'float',
    'int8_t': 'int',
    'int16_t': 'int',
    'int32_t': 'int',
    'bool': 'bool'
}

# Generates file header
def file_header(version, comment_char="//"):
    return f"""{comment_char} ----------------------------------------------------------------------------
{comment_char} Kinisi motr controller commands.
{comment_char} This file is auto generated by the commands generator from JSON file.
{comment_char} Do not edit this file manually.
{comment_char} Timestamp: { datetime.datetime.now().strftime("%Y-%m-%d %H:%M:00") }
{comment_char} Version: {version}
{comment_char} ----------------------------------------------------------------------------\n\n"""

# Get object name in pascal case
def get_object_name(object_data):
    # Convert string to pascal case Example motor_controller_state -> MotorControllerState
    return ''.join([word.capitalize() for word in object_data['name'].split('_')])

# Generate object
def generate_object(object_data):
    result = ""
    result += f"# {object_data.get('description', '')}\n"
    result += "# Properties:\n"
    for prop in object_data['properties']:
        result += f"#    {prop['name']}: {prop['description']}\n"
    # Generate class definition
    result += f"class {get_object_name(object_data)}:\n"

    # Generate constructor
    result += "    def __init__(self, "
    result += ", ".join([f"{prop['name']}:{type_mapping[prop['type']]}" for prop in object_data['properties']])
    result += "):\n"
    for prop in object_data['properties']:
        result += f"        self.{prop['name']} = {prop['name']}\n" 
    result += "\n"

    # Generate get size method
    result += "    @staticmethod\n"
    result += "    def get_size() -> int:\n"
    result += f"        return {sum([type_to_size_map[prop['type']] for prop in object_data['properties']])}\n"
    result += "\n"

    # Generate encode method
    result += "    def encode(self) -> bytearray:\n"
    result += "        msg = bytearray()\n"
    for prop in object_data['properties']:
        if prop['type'] == 'double':
            result += f"        msg += bytearray(struct.pack('d', self.{prop['name']}))\n"
        else:
            signed = ", signed=True" if prop['type'].startswith("int") else ""
            result += f"        msg += self.{prop['name']}.to_bytes({type_to_size_map[prop['type']]}, 'little'{signed})\n"
    result += "        return msg\n"
    result += "\n"

    # Generate decode method
    result += "    @staticmethod\n"
    result += "    def decode(msg: bytearray) -> object:\n"
    result += f"        result = {get_object_name(object_data)}(\n"
    offset = 0
    for prop in object_data['properties']:
        if prop['type'] == 'double':
            result += f"            {prop['name']}=struct.unpack('<d', msg[{offset}:{offset + type_to_size_map[prop['type']]}])[0]"
        else:
            result += f"            {prop['name']}={type_mapping[prop['type']]}.from_bytes(msg[{offset}:{offset + type_to_size_map[prop['type']]}], 'little')"
        if prop != object_data['properties'][-1]:
            result += ",\n"
        offset += type_to_size_map[prop['type']]
    result += ")\n"
    result += "        return result\n"
    result += "\n"

    return result


# Generates Python code from the commands JSON file
def generate_python_code(commands_data):

    # Generate objects
    objects = ""
    for obj in commands_data.get("objects", []):
        objects += generate_object(obj)
    objects += "\n"

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
        func_body += "    # Parameters:\n"
        for prop in cmd.get('properties', []):
            func_body += f"    #   {prop['name']}: {prop['description']}\n"
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
            # Response length
            if cmd['response']['type'] == 'object':
                func_body += f"        response_length = {get_object_name(cmd['response'])}.get_size()\n"
                func_body += f"        result =  self.read(response_length)\n"
                func_body += f"        return {get_object_name(cmd['response'])}.decode(result)\n"
            else:
                func_body += f"        response_length = {type_to_size_map[cmd['response']['type']]}\n"
                func_body += f"        result =  self.read(response_length)\n"
                if cmd['response']['type'] == 'double':
                    func_body += f"        return struct.unpack('<d', result)[0]\n"
                else:
                    func_body += f"        return {type_mapping[cmd['response']['type']]}.from_bytes(result, 'little')\n"

        function_code += func_body
        function_code += "\n"

    class_code += function_code

    result = file_header(commands_data['version'], "#")
    result += "import struct\n\n"
    result += objects
    result += constant_code
    result += class_code

    return result


# Generate python code from from input JSON file
# path: Path to the input JSON file containing command definitions.
# output_path: Path to the output file.
def generate(path, output_path):
    try:
        with open(path, 'r') as file:
            commands_data = json.load(file)
    except json.JSONDecodeError:
        print("Error: Could not decode the input JSON file.")
        return
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return

    generated_code = generate_python_code(commands_data)

    try:
        with open(output_path, 'w') as file:
            file.write(generated_code)
    except Exception as e:
        print(f"An error occurred while writing to the output file: {e}")
        return

    print(f"Code has been generated and saved to {output_path}")