import configparser


def read_config():
    config = configparser.ConfigParser()
    config.read("config.txt")
    R = float(config.get("myvars", "R"))
    z = float(config.get("myvars", "z"))
    A = eval(config.get("myvars", "A"))
    B = eval(config.get("myvars", "B"))
    C = eval(config.get("myvars", "C"))
    D = eval(config.get("myvars", "D"))
    left = eval(config.get("myvars", "left"))
    right = eval(config.get("myvars", "right"))
    V = float(config.get("myvars", "V"))
    max_length = float(config.get("myvars", "max_length"))
    com = str(config.get("myvars", "com"))
    return R, z, A, B, C, D, left, right, V, max_length, com


def read_speed():
    config = configparser.ConfigParser()
    config.read("config.txt")
    V = float(config.get("myvars", "V"))
    return V


def update_config(name, value):
    config = configparser.ConfigParser()
    config.read("config.txt")
    config.set("myvars", str(name), str(value))
    with open("config.txt", "w+") as configfile:
        config.write(configfile)
