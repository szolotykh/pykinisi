import requests
import subprocess
import os
import shutil


files = ['https://raw.githubusercontent.com/szolotykh/kinisi-motor-controller-firmware/main/tools/commands_generator/generator.py',
         'https://raw.githubusercontent.com/szolotykh/kinisi-motor-controller-firmware/main/commands.json']

shutil.rmtree('./tmp', ignore_errors=True)
os.mkdir('./tmp')
for url in files:
  response = requests.get(url)
  with open(f"./tmp/{url.split('/')[-1]}", 'wb') as f:
    f.write(response.content)

# python generator.py commands.json kinisi_commands.py --language python
subprocess.call(["python", "./tmp/generator.py", "./tmp/commands.json", "./../pykinisi/KinisiCommands.py", "--language", "python"])
shutil.rmtree('./tmp', ignore_errors=True)