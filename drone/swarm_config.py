DRONE_GROUPS = {
    "group_1":[
        "drone1",
        "drone2",
        "drone3",

    ]
}

def get_drone_groups(drone_id):
    for group_name, drones in DRONE_GROUPS.items():
        if drone_id in drones:
            return group_name
    return None

LAYER_1_NEIGHBORS = {
    "drone1":["drone2", "drone3",],
    "drone2":["drone1", "drone3"],
    "drone3":["drone1", "drone2"],
}
