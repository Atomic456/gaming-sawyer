object_wight_augmentation = {
    "1": 0.027,
    "2": 0.015,
    "3": 0.008,
    "4": 0.005,
    "5": 0.016,
    "6": 0.012,
    "7": 0.009,
    "8": 0.005,
    "9": 0.011,
}


def game_piece_picker(object_info, center_x=0, center_y=0):
    total_weight = 0
    x_force = 0
    y_force = 0
    for obj in object_info:
        total_weight += object_wight_augmentation[str(obj["class"])]
        d_x, d_y = calc_deferential_distance(obj["x"], obj["y"], center_x, center_y)
        x_force += d_x * object_wight_augmentation[str(obj["class"])]
        y_force += d_y * object_wight_augmentation[str(obj["class"])]
    object_to_pick = {
        "idx": 0,
        "dx": 1000,
        "dy": 1000
    }
    idx = 0
    for obj in object_info:
        d_x, d_y = calc_deferential_distance(obj["x"], obj["y"], center_x, center_y)
        new_x_force = x_force - d_x * object_wight_augmentation[str(obj["class"])]
        new_y_force = y_force - d_y * object_wight_augmentation[str(obj["class"])]
        new_center_of_mass_x = new_x_force / (total_weight - object_wight_augmentation[str(obj["class"])])
        new_center_of_mass_y = new_y_force / (total_weight - object_wight_augmentation[str(obj["class"])])
        com_dx, com_dy = calc_deferential_distance(new_center_of_mass_x, new_center_of_mass_y, center_x, center_y)
        if abs(com_dx + com_dy) < abs(object_to_pick["dx"] + object_to_pick["dy"]):
            object_to_pick = {"idx": idx, "dx": com_dx, "dy": com_dy}
        idx += 1

    return int(object_to_pick["idx"])



def calc_deferential_distance(x, y, center_x=0, center_y=0):
    d_x = 0
    if x <= center_x:
        d_x = x - center_x
    if x > center_x:
        d_x = center_x - x

    d_y = center_y - y

    return d_x, d_y
