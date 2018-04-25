
import jinja2
import os
import json

def get_flat_endpoint_list(json, prefix):
    flat_list = []
    for item in json:
        item = item.copy()
        if 'type' in item:
            if item['type'] in {'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64'}:
                item['type'] += '_t'
                is_property = True
            elif item['type'] in {'bool', 'float'}:
                is_property = True
            else:
                is_property = False
            if is_property:
                item['name'] = prefix + item['name']
                flat_list.append(item)
        if 'members' in item:
            flat_list = flat_list + get_flat_endpoint_list(item['members'], prefix + item['name'] + '.')
    return flat_list

def generate_code(odrv, template_file, output_file):
    json_data = odrv._json_data
    json_crc = odrv._json_crc

    endpoints = get_flat_endpoint_list(json_data, '')

    env = jinja2.Environment(
        #loader = jinja2.FileSystemLoader("/Data/Projects/")
        #trim_blocks=True,
        #lstrip_blocks=True
    )

    # Expose helper functions to jinja template code
    #env.filters["delimit"] = camel_case_to_words

    # Load and render template
    template = env.from_string(template_file.read())
    output = template.render(
        json_crc=json_crc,
        endpoints=endpoints,
        output_name=os.path.basename(output_file.name)
    )

    # Output
    output_file.write(output)
