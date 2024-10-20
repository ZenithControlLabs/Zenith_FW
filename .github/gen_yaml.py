from jinja2 import Environment, FileSystemLoader

environment = Environment(loader=FileSystemLoader("./"))
template = environment.get_template("main.yml.j2")

with open("workflows/main.yml", mode="w") as f:
    f.write(template.render(
        {
            "buildconfigs": [
                { "name": "proto",  "cmake_opts": "-DCMAKE_C_FLAGS=-DHW_PHOBRI_PROTO"},
                { "name": "v1_X_3d",  "cmake_opts": "-DCMAKE_C_FLAGS=-DHW_PHOBRI_V1_X_3D"},
                { "name": "v1_X_analog",  "cmake_opts": "-DCMAKE_C_FLAGS=-DHW_PHOBRI_V1_X_ANALOG"},
            ]
        }
    ))

