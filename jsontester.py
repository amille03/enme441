import requests
import json

URL = "https://raw.githubusercontent.com/amille03/enme441/refs/heads/main/jsontest.json"

# Download the file over WiFi
response = requests.get(URL)
data = response.json()   # Automatically parses JSON

# Access turret 1 theta
theta1 = data["turrets"]["1"]["theta"]
print("Turret 1 theta:", theta1)

# Access all globes
globes = data["globes"]

for g in globes:
    print(g["theta"], g["z"])
