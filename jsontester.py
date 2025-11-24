import requests
import json

# RAW GitHub JSON URL
url = "http://192.168.1.254:8000/positions.json"

# Retrieve and parse JSON
response = requests.get(url)
data = response.json()

# ----- Extract Turret Data -----
turret_ids = []
turret_r = []
turret_theta = []

for tid, tinfo in data["turrets"].items():
    turret_ids.append(int(tid))
    turret_r.append(tinfo["r"])
    turret_theta.append(tinfo["theta"])

# ----- Extract Globe Data -----
globe_r = []
globe_theta = []
globe_z = []

for g in data["globes"]:
    globe_r.append(g["r"])
    globe_theta.append(g["theta"])
    globe_z.append(g["z"])

# ----- Print to verify -----
print("Turret IDs:", turret_ids)
print("Turret r:", turret_r)
print("Turret theta:", turret_theta)

print("Globe r:", globe_r)
print("Globe theta:", globe_theta)
print("Globe z:", globe_z)

