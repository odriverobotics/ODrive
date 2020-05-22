#!/bin/python3

import yaml
import json
import jinja2
import jsonschema
import re
import argparse
import sys

# This schema describes what we expect interface definition files to look like
validator = jsonschema.Draft7Validator(yaml.safe_load("""
definitions:
  interface:
    type: object
    properties:
      c_is_class: {type: boolean}
      c_name: {type: string}
      functions:
        type: object
        additionalProperties: {"$ref": "#/definitions/function"}
      attributes:
        type: object
        additionalProperties: {"$ref": "#/definitions/attribute"}
      __line__: {type: object}
      __column__: {type: object}
    required: [c_is_class]
    additionalProperties: false

  valuetype:
    type: object
    properties:
      mode: {type: string} # this shouldn't be here
      c_name: {type: string}
      values: {type: object}
      flags: {type: object}
      nullflag: {type: string}
      __line__: {type: object}
      __column__: {type: object}
    additionalProperties: false

  intf_or_val_type:
    anyOf:
      - {"$ref": "#/definitions/interface"}
      - {"$ref": "#/definitions/valuetype"}
      - {"type": "string"}

  attribute:
    anyOf: # this is probably not being used correctly
      - {"$ref": "#/definitions/intf_or_val_type"}
      - type: object
      - type: object
        properties:
          type: {"$ref": "#/definitions/intf_or_val_type"}
          c_name: {"type": string}
          unit: {"type": string}
          doc: {"type": string}
        additionalProperties: false

  function:
    anyOf:
      - type: 'null'
      - type: object
        properties:
          in: {type: object}
          out: {type: object}
          doc: {type: string}
          __line__: {type: object}
          __column__: {type: object}
        additionalProperties: false


type: object
properties:
  ns: {type: string}
  version: {type: string}
  summary: {type: string}
  interfaces:
    type: object
    additionalProperties: { "$ref": "#/definitions/interface" }
  valuetypes:
    type: object
    additionalProperties: { "$ref": "#/definitions/valuetype" }
  __line__: {type: object}
  __column__: {type: object}
additionalProperties: false
"""))

# Source: https://stackoverflow.com/a/53647080/3621512
class SafeLineLoader(yaml.SafeLoader):
    pass
#    def compose_node(self, parent, index):
#        # the line number where the previous token has ended (plus empty lines)
#        line = self.line
#        node = super(SafeLineLoader, self).compose_node(parent, index)
#        node.__line__ = line + 1
#        return node
#
#    def construct_mapping(self, node, deep=False):
#        mapping = super(SafeLineLoader, self).construct_mapping(node, deep=deep)
#        mapping['__line__'] = node.__line__
#        #mapping['__column__'] = node.start_mark.column + 1
#        return mapping



def get_words(string):
    """
    Splits a string in PascalCase into a list of lower case words
    """
    return [w.lower() for w in re.findall('[a-z0-9]+|[A-Z][a-z0-9]*', string)]

def join_name(*names, delimiter: str = '.'):
    """
    Joins two name components.
    e.g. 'io.helloworld' + 'sayhello' => 'io.helloworld.sayhello'
    """
    return delimiter.join(y for x in names for y in x.split(delimiter) if y != '')

def split_name(name, delimiter: str = '.'):
    def replace_delimiter_in_parentheses():
        parenthesis_depth = 0
        for c in name:
            parenthesis_depth += 1 if c == '<' else -1 if c == '>' else 0
            yield c if (parenthesis_depth == 0) or (c != delimiter) else ':'
    return [part.replace(':', '.') for part in ''.join(replace_delimiter_in_parentheses()).split('.')]

def to_pascal_case(s): return ''.join([w.title() for w in get_words(s)])
def to_camel_case(s): return ''.join([(c.lower() if i == 0 else c) for i, c in enumerate(''.join([w.title() for w in get_words(s)]))])
def to_macro_case(s): return '_'.join(get_words(s)).upper()
def to_snake_case(s): return '_'.join(get_words(s)).lower()
def to_kebab_case(s): return '-'.join(get_words(s)).lower()

value_types = {
    'bool': {'builtin': True, 'fullname': 'bool', 'name': 'bool', 'c_type': 'bool'},
    'float32': {'builtin': True, 'fullname': 'float32', 'name': 'float32', 'c_type': 'float'},
    'uint8': {'builtin': True, 'fullname': 'uint8', 'name': 'uint8', 'c_type': 'uint8_t'},
    'uint16': {'builtin': True, 'fullname': 'uint16', 'name': 'uint16', 'c_type': 'uint16_t'},
    'uint32': {'builtin': True, 'fullname': 'uint32', 'name': 'uint32', 'c_type': 'uint32_t'},
    'uint64': {'builtin': True, 'fullname': 'uint64', 'name': 'uint64', 'c_type': 'uint64_t'},
    'int8': {'builtin': True, 'fullname': 'int8', 'name': 'int8', 'c_type': 'int8_t'},
    'int16': {'builtin': True, 'fullname': 'int16', 'name': 'int16', 'c_type': 'int16_t'},
    'int32': {'builtin': True, 'fullname': 'int32', 'name': 'int32', 'c_type': 'int32_t'},
    'int64': {'builtin': True, 'fullname': 'int64', 'name': 'int64', 'c_type': 'int64_t'},
    'endpoint_ref': {'builtin': True, 'fullname': 'endpoint_ref', 'name': 'endpoint_ref', 'c_type': 'endpoint_ref_t'},
}

enums = {}

interfaces = {}

def make_property_type(typeargs):
    value_type = resolve_valuetype('', typeargs['fibre.Property.type'])
    mode = typeargs.get('fibre.Property.mode', 'readwrite')
    name = 'Property<' + value_type['fullname'] + ', ' + mode + '>'
    fullname = join_name('fibre', name)
    if fullname in interfaces:
        return interfaces[fullname]

    c_type = 'Property<' + ('const ' if mode == 'readonly' else '') + value_type['c_type'] + '>'
    prop_type = {
        'name': name,
        'fullname': fullname,
        'purename': 'fibre.Property',
        'c_type': c_type,
        'value_type': value_type, # TODO: should be a metaarg
        'mode': mode, # TODO: should be a metaarg
        'builtin': True,
        'attributes': {},
        'functions': {}
    }
    if mode != 'readonly':
        prop_type['functions']['exchange'] = {
            'name': 'exchange',
            'fullname': join_name(fullname, 'exchange'),
            'in': {'obj': {'name': 'obj', 'type': {'c_type': c_type}}, 'value': {'name': 'value', 'type': value_type, 'optional': True}},
            'out': {'value': {'name': 'value', 'type': value_type}},
            #'implementation': 'fibre_property_exchange<' + value_type['c_type'] + '>'
        }
    else:
        prop_type['functions']['read'] = {
            'name': 'read',
            'fullname': join_name(fullname, 'read'),
            'in': {'obj': {'name': 'obj', 'type': {'c_type': c_type}}},
            'out': {'value': {'name': 'value', 'type': value_type}},
            #'implementation': 'fibre_property_read<' + value_type['c_type'] + '>'
        }

    interfaces[fullname] = prop_type
    return prop_type

generics = {
    'fibre.Property': make_property_type # TODO: improve generic support
}


def make_ref_type(interface):
    name = 'Ref<' + interface['fullname'] + '>'
    fullname = join_name('fibre', name)
    if fullname in interfaces:
        return interfaces[fullname]

    ref_type = {
        'builtin': True,
        'name': name,
        'fullname': fullname,
        'c_type': interface['fullname'].replace('.', 'Intf::') + 'Intf*'
    }
    value_types[fullname] = ref_type

    return ref_type

def get_dict(elem, key):
    return elem.get(key, None) or {}

def regularize_arg(path, name, elem):
    if elem is None:
        elem = {}
    elif isinstance(elem, str):
        elem = {'type': elem}
    elem['name'] = name
    elem['fullname'] = path = join_name(path, name)
    elem['type'] = regularize_valuetype(path, name, elem['type'])
    return elem

def regularize_func(path, name, elem, prepend_args):
    if elem is None:
        elem = {}
    elem['name'] = name
    elem['fullname'] = path = join_name(path, name)
    elem['in'] = {n: regularize_arg(path, n, arg)
                  for n, arg in {**prepend_args, **get_dict(elem, 'in')}.items()}
    elem['out'] = {n: regularize_arg(path, n, arg)
                  for n, arg in get_dict(elem, 'out').items()}
    return elem

def regularize_attribute(path, name, elem, c_is_class):
    if elem is None:
        elem = {}
    if isinstance(elem, str):
        elem = {'type': elem}
    elif not 'type' in elem:
        elem['type'] = {}
        if 'attributes' in elem: elem['type']['attributes'] = elem.pop('attributes')
        if 'functions' in elem: elem['type']['functions'] = elem.pop('functions')
        if 'c_is_class' in elem: elem['type']['c_is_class'] = elem.pop('c_is_class')
        if 'values' in elem: elem['type']['values'] = elem.pop('values')
        if 'flags' in elem: elem['type']['flags'] = elem.pop('flags')
        if 'nullflag' in elem: elem['type']['nullflag'] = elem.pop('nullflag')
    
    elem['name'] = name
    elem['fullname'] = join_name(path, name)
    elem['typeargs'] = elem.get('typeargs', {})
    elem['c_name'] = elem.get('c_name', None) or (elem['name'] + ('_' if c_is_class else ''))
    if ('c_getter' in elem) or ('c_setter' in elem):
        elem['c_getter'] = elem.get('c_getter', elem['c_name'])
        elem['c_setter'] = elem.get('c_setter', elem['c_name'] + ' = ')

    if isinstance(elem['type'], str) and elem['type'].startswith('readonly '):
        elem['typeargs']['fibre.Property.mode'] = 'readonly'
        elem['typeargs']['fibre.Property.type'] = elem['type'][len('readonly '):]
        elem['type'] = 'fibre.Property'
        if elem['typeargs']['fibre.Property.mode'] == 'readonly' and 'c_setter' in elem: elem.pop('c_setter')
    elif ('flags' in elem['type']) or ('values' in elem['type']):
        elem['typeargs']['fibre.Property.mode'] = elem['typeargs'].get('fibre.Property.mode', None) or 'readwrite'
        elem['typeargs']['fibre.Property.type'] = regularize_valuetype(path, to_pascal_case(name), elem['type'])
        elem['type'] = 'fibre.Property'
        if elem['typeargs']['fibre.Property.mode'] == 'readonly' and 'c_setter' in elem: elem.pop('c_setter')
    else:
        elem['type'] = regularize_interface(path, to_pascal_case(name), elem['type'])
    return elem


def regularize_interface(path, name, elem):
    if elem is None:
        elem = {}
    if isinstance(elem, str):
        return elem # will be resolved during type resolution
    #if path is None:
    #    max_anonymous_type = max([int((re.findall('^' + join_name(path, 'AnonymousType') + '([1-9]+)$', x) + ['0'])[0]) for x in interfaces.keys()])
    #    path = 'AnonymousType' + str(max_anonymous_type + 1)
    elem['name'] = split_name(name)[-1]
    elem['fullname'] = path = join_name(path, name)
    elem['c_type'] = elem.get('c_type', elem['fullname'].replace('.', 'Intf::')) + 'Intf'
    interfaces[path] = elem
    elem['functions'] = {name: regularize_func(path, name, func, {'obj': {'type': make_ref_type(elem)}})
                         for name, func in get_dict(elem, 'functions').items()}
    if not 'c_is_class' in elem:
        raise Exception(elem)
    treat_as_class = elem['c_is_class'] # TODO: add command line arg to make this selectively optional
    elem['attributes'] = {name: regularize_attribute(path, name, prop, treat_as_class)
                          for name, prop in get_dict(elem, 'attributes').items()}
    elem['interfaces'] = []
    elem['enums'] = []
    return elem

def regularize_valuetype(path, name, elem):
    if elem is None:
        elem = {}
    if isinstance(elem, str):
        return elem # will be resolved during type resolution
    elem['name'] = split_name(name)[-1]
    elem['fullname'] = path = join_name(path, name)
    elem['c_type'] = elem.get('c_type', elem['fullname'].replace('.', 'Intf::'))
    value_types[path] = elem

    if 'flags' in elem: # treat as flags
        bit = 0
        for k, v in elem['flags'].items():
            elem['flags'][k] = elem['flags'][k] or {}
            current_bit = elem['flags'][k].get('bit', bit)
            elem['flags'][k]['bit'] = current_bit
            elem['flags'][k]['value'] = 0 if current_bit is None else (1 << current_bit)
            bit = bit if current_bit is None else current_bit + 1
        if 'nullflag' in elem:
            elem['flags'] = {elem['nullflag']: {'value': 0, 'bit': None}, **elem['flags']}
        elem['values'] = elem['flags']
        elem['is_flags'] = True
        elem['is_enum'] = True
        enums[path] = elem

    elif 'values' in elem: # treat as enum
        val = 0
        for k, v in elem['values'].items():
            elem['values'][k] = elem['values'][k] or {}
            val = elem['values'][k].get('value', val)
            elem['values'][k]['value'] = val
            val += 1
        enums[path] = elem
        elem['is_enum'] = True
    
    return elem

def resolve_interface(scope, name, typeargs):
    """
    Resolves a type name (i.e. interface name or value type name) given as a
    string to an interface object. The innermost scope is searched first.
    At every scope level, if no matching interface is found, it is checked if a
    matching value type exists. If so, the interface type fibre.Property<value_type>
    is returned.
    """
    if not isinstance(name, str):
        return name
    
    if 'fibre.Property.type' in typeargs:
        typeargs['fibre.Property.type'] = resolve_valuetype(scope, typeargs['fibre.Property.type'])

    scope = scope.split('.')
    for probe_scope in [join_name(*scope[:(len(scope)-i)]) for i in range(len(scope)+1)]:
        probe_name = join_name(probe_scope, name)
        #print('probing ' + probe_name)
        if probe_name in interfaces:
            return interfaces[probe_name]
        elif probe_name in value_types:
            typeargs['fibre.Property.type'] = value_types[probe_name]
            return make_property_type(typeargs)
        elif probe_name in generics:
            return generics[probe_name](typeargs)

    raise Exception(f'could not resolve type {name} in {join_name(*scope)}. Known interfaces are: {list(interfaces.keys())}. Known value types are: {list(value_types.keys())}')

def resolve_valuetype(scope, name):
    """
    Resolves a type name given as a string to the type object.
    The innermost scope is searched first.
    """
    if not isinstance(name, str):
        return name
    
    scope = scope.split('.')
    for probe_scope in [join_name(*scope[:(len(scope)-i)]) for i in range(len(scope)+1)]:
        probe_name = join_name(probe_scope, name)
        if probe_name in value_types:
            return value_types[probe_name]

    raise Exception(f'could not resolve type {name} in {join_name(*scope)}. Known value types are: {list(value_types.keys())}')


def map_to_fibre01_type(t):
    if t.get('is_enum', False):
        return 'int32'
    elif t['fullname'] == 'float32':
        return 'float'
    return t['fullname']

def generate_endpoint_for_property(prop, attr_bindto, idx):
    prop_intf = interfaces[prop['type']['fullname']]

    endpoint = {
        'id': idx,
        'function': prop_intf['functions']['read' if prop['type']['mode'] == 'readonly' else 'exchange'],
        'in_bindings': {'obj': attr_bindto},
        'out_bindings': []
    }
    endpoint_definition = {
        'name': prop['name'],
        'id': idx,
        'type': map_to_fibre01_type(prop['type']['value_type']),
        'access': 'r' if prop['type']['mode'] == 'readonly' else 'rw',
    }
    return endpoint, endpoint_definition

def generate_endpoint_table(intf, bindto, idx):
    """
    Generates a Fibre v0.1 endpoint table for a given interface.
    This will probably be deprecated in the future.
    The object must have no circular property types (i.e. A.b has type B and B.a has type A).
    """
    endpoints = []
    endpoint_definitions = []
    cnt = 0

    for k, prop in intf['attributes'].items():
        property_value_type = re.findall('^fibre\.Property<([^>]*), (readonly|readwrite)>$', prop['type']['fullname'])
        #attr_bindto = join_name(bindto, bindings_map.get(join_name(intf['fullname'], k), k + ('_' if len(intf['functions']) or (intf['fullname'] in treat_as_classes) else '')))
        attr_bindto = intf['c_type'] + '::get_' + prop['name'] + '(' + bindto + ')'
        if len(property_value_type):
            # Special handling for Property<...> attributes: they resolve to one single endpoint
            endpoint, endpoint_definition = generate_endpoint_for_property(prop, attr_bindto, idx + cnt)
            endpoints.append(endpoint)
            endpoint_definitions.append(endpoint_definition)
            cnt += 1
        else:
            inner_endpoints, inner_endpoint_definitions, inner_cnt = generate_endpoint_table(prop['type'], attr_bindto, idx + cnt)
            endpoints += inner_endpoints
            endpoint_definitions.append({
                'name': k,
                'type': 'object',
                'members': inner_endpoint_definitions
            })
            cnt += inner_cnt

    for k, func in intf['functions'].items():
        endpoints.append({
            'id': idx + cnt,
            'function': func,
            'in_bindings': {**{'obj': bindto}, **{k_arg: '(' + bindto + ')->' + func['name'] + '_in_' + k_arg + '_' for k_arg in list(func['in'].keys())[1:]}},
            'out_bindings': {k_arg: '&(' + bindto + ')->' + func['name'] + '_out_' + k_arg + '_' for k_arg in func['out'].keys()},
        })
        in_def = []
        out_def = []
        for i, (k_arg, arg) in enumerate(list(func['in'].items())[1:]):
            endpoint, endpoint_definition = generate_endpoint_for_property({
                'name': arg['name'],
                'type': make_property_type({'fibre.Property.type': arg['type'], 'fibre.Property.mode': 'readwrite'})
            }, intf['c_type'] + '::get_' + func['name'] + '_in_' + k_arg + '_' + '(' + bindto + ')', idx + cnt + 1 + i)
            endpoints.append(endpoint)
            in_def.append(endpoint_definition)
        for i, (k_arg, arg) in enumerate(func['out'].items()):
            endpoint, endpoint_definition = generate_endpoint_for_property({
                'name': arg['name'],
                'type': make_property_type({'fibre.Property.type': arg['type'], 'fibre.Property.mode': 'readonly'})
            }, intf['c_type'] + '::get_' + func['name'] + '_out_' + k_arg + '_' + '(' + bindto + ')', idx + cnt + len(func['in']) + i)
            endpoints.append(endpoint)
            out_def.append(endpoint_definition)

        endpoint_definitions.append({
            'name': k,
            'id': idx + cnt,
            'type': 'function',
            'inputs': in_def,
            'outputs': out_def
        })
        cnt += len(func['in']) + len(func['out'])

    return endpoints, endpoint_definitions, cnt


# Parse arguments

parser = argparse.ArgumentParser(description="Gernerate code from YAML interface definitions")
parser.add_argument("--version", action="store_true",
                    help="print version information")
parser.add_argument("-v", "--verbose", action="store_true",
                    help="print debug information (on stderr)")
parser.add_argument("-d", "--definitions", type=argparse.FileType('r'), nargs='+',
                    help="the YAML interface definition file(s) used to generate the code")
parser.add_argument("-t", "--template", type=argparse.FileType('r'),
                    help="the code template")
parser.add_argument("-o", "--output", type=argparse.FileType('w'), default='-',
                    help="path of the generated output")
args = parser.parse_args()

if args.version:
    print("0.0.1")
    sys.exit(0)


definition_files = args.definitions
template_file = args.template
output_file = args.output


# Load definition files

for definition_file in definition_files:
    try:
        file_content = yaml.load(definition_file, Loader=SafeLineLoader)
    except yaml.scanner.ScannerError as ex:
        print("YAML parsing error: " + str(ex), file=sys.stderr)
        sys.exit(1)
    for err in validator.iter_errors(file_content):
        if '__line__' in err.absolute_path:
            continue
        if '__column__' in err.absolute_path:
            continue
        #instance = err.instance.get(re.findall("([^']*)' (?:was|were) unexpected\)", err.message)[0], err.instance)
        # TODO: print line number
        raise Exception(err.message + '\nat ' + str(list(err.absolute_path)))
    interfaces = {**interfaces, **get_dict(file_content, 'interfaces')}
    value_types = {**value_types, **get_dict(file_content, 'valuetypes')}


# Preprocess definitions

# Regularize everything into a wellknown form
for k, item in list(interfaces.items()):
    regularize_interface('', k, item)
for k, item in list(value_types.items()):
    regularize_valuetype('', k, item)

if args.verbose:
    print('Known interfaces: ' + ''.join([('\n  ' + k) for k in interfaces.keys()]))
    print('Known value types: ' + ''.join([('\n  ' + k) for k in value_types.keys()]))

# Resolve all types into references
for _, item in list(interfaces.items()):
    for _, prop in item['attributes'].items():
        prop['type'] = resolve_interface(item['fullname'], prop['type'], prop['typeargs'])
    for _, func in item['functions'].items():
        for _, arg in func['in'].items():
            arg['type'] = resolve_valuetype(item['fullname'], arg['type'])
        for _, arg in func['out'].items():
            arg['type'] = resolve_valuetype(item['fullname'], arg['type'])

# Attach interfaces to their parents
toplevel_interfaces = []
for k, item in list(interfaces.items()):
    k = split_name(k)
    if len(k) == 1:
        toplevel_interfaces.append(item)
    else:
        if k[:-1] != ['fibre']: # TODO: remove special handling
            parent = interfaces[join_name(*k[:-1])]
            parent['interfaces'].append(item)
            item['parent'] = parent
toplevel_enums = []
for k, item in list(enums.items()):
    k = split_name(k)
    if len(k) == 1:
        toplevel_enums.append(item)
    else:
        if k[:-1] != ['fibre']: # TODO: remove special handling
            parent = interfaces[join_name(*k[:-1])]
            parent['enums'].append(item)
            item['parent'] = parent



endpoints, embedded_endpoint_definitions, _ = generate_endpoint_table(interfaces['Odrive'], '&odrv', 1) # TODO: make user-configurable
embedded_endpoint_definitions = [{'name': '', 'id': 0, 'type': 'json', 'access': 'r'}] + embedded_endpoint_definitions
endpoints = [{'id': 0, 'function': {'fullname': 'endpoint0_handler', 'in': {}, 'out': {}}, 'bindings': {}}] + endpoints


# Render template

env = jinja2.Environment(
    comment_start_string='[#', comment_end_string='#]',
    block_start_string='[%', block_end_string='%]',
    variable_start_string='[[', variable_end_string=']]'
)

env.filters['to_pascal_case'] = to_pascal_case
env.filters['to_camel_case'] = to_camel_case
env.filters['to_macro_case'] = to_macro_case
env.filters['to_snake_case'] = to_snake_case
env.filters['to_kebab_case'] = to_kebab_case
env.filters['first'] = lambda x: next(iter(x))
env.filters['skip_first'] = lambda x: list(x)[1:]
env.filters['to_c_string'] = lambda x: '\n'.join(('"' + line.replace('"', '\\"') + '"') for line in json.dumps(x, separators=(',', ':')).replace('{"name"', '\n{"name"').split('\n'))

template = env.from_string(template_file.read())

output = template.render(
    interfaces = interfaces,
    value_types = value_types,
    toplevel_interfaces = toplevel_interfaces,
    endpoints = endpoints,
    embedded_endpoint_definitions = embedded_endpoint_definitions
)

output_file.write(output)
