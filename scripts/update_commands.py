import requests
import os
import shutil
from sdkgenerator import generate

branch = 'main'
files = [f'https://raw.githubusercontent.com/szolotykh/kinisi-motor-controller-firmware/{branch}/commands.json']

def main():
    # Download the input JSON file from the repository
    shutil.rmtree('./tmp', ignore_errors=True)
    os.mkdir('./tmp')
    for url in files:
        response = requests.get(url)
        with open(f"./tmp/{url.split('/')[-1]}", 'wb') as f:
            f.write(response.content)

    # Generate python code from the commands from the input JSON file
    generate("./tmp/commands.json", "./../pykinisi/KinisiCommands.py")

    # Cleanup
    shutil.rmtree('./tmp', ignore_errors=True)

if __name__ == "__main__":
    main()