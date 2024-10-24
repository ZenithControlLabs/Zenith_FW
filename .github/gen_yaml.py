from jinja2 import Environment, FileSystemLoader

environment = Environment(loader=FileSystemLoader("./"))
template = environment.get_template("main.yml.j2")

with open("workflows/main.yml", mode="w") as f:
    f.write(template.render(
        {
            "buildconfigs": [
                { "name": "v1_0",  "cmake_opts": "-DCMAKE_C_FLAGS=-DHW_PHOBRI_V1_0"},
                { "name": "v1_1_analog",  "cmake_opts": "-DCMAKE_C_FLAGS=-DHW_PHOBRI_V1_1_ANALOG"},
            ]
        }
    ))

