import yaml

try:
    with open('default.yaml', 'r', encoding='utf-8') as file:
        data = yaml.safe_load(file)
        print(data)
except yaml.YAMLError as exc:
    print(exc)

