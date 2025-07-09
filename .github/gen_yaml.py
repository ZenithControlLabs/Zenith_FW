from jinja2 import Environment, FileSystemLoader

environment = Environment(loader=FileSystemLoader("./"))
template = environment.get_template("main.yml.j2")

with open("workflows/main.yml", mode="w") as f:
    f.write(template.render(
        {
            "buildconfigs": [
                { "name": "v1_X",  "platform": "phobri64", "cmake_opts": "-DPCB=HW_PHOBRI_V1_1_ANALOG"},
                { "name": "remapper",  "platform": "remapper", "cmake_opts": ""},
            ],
            "name2uf2": {
                "phobri64": "Phobri64",
                "remapper": "N64_Remapper",
            }
        }
    ))

