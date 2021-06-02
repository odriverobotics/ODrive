#!/bin/python3

import yaml
import json
import jinja2
import jsonschema
import re
import argparse
import sys
from collections import OrderedDict

# This schema describes what we expect interface definition files to look like
validator = jsonschema.Draft4Validator(yaml.safe_load("""
definitions:
  interface:
    type: object
    properties:
      c_is_class: {type: boolean}
      c_name: {type: string}
      brief: {type: string}
      doc: {type: string}
      implements:
        anyOf:
          - {"$ref": "#/definitions/intf_type_ref"}
          - {type: array, items: {"$ref": "#/definitions/intf_type_ref"}}
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

  intf_type_ref:
    anyOf:
      - {"type": "string"}
      - {"$ref": "#/definitions/interface"}

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
          brief: {type: string}
          doc: {type: string}
          __line__: {type: object}
          __column__: {type: object}
        additionalProperties: false


type: object
properties:
  ns: {type: string}
  version: {type: string}
  summary: {type: string}
  dictionary: {type: array, items: {type: string}}
  interfaces:
    type: object
    additionalProperties: { "$ref": "#/definitions/interface" }
  valuetypes:
    type: object
    additionalProperties: { "$ref": "#/definitions/valuetype" }
  userdata:
    type: object
  __line__: {type: object}
  __column__: {type: object}
additionalProperties: false
"""))

# TODO: detect duplicate keys in yaml dictionaries

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

# Ensure that dicts remain ordered, even in Python <3.6
# source: https://stackoverflow.com/a/21912744/3621512
def construct_mapping(loader, node):
    loader.flatten_mapping(node)
    return OrderedDict(loader.construct_pairs(node))
SafeLineLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_mapping)

dictionary = []

def get_words(string):
    """
    Splits a string in PascalCase or MACRO_CASE into a list of lower case words
    """
    if string.isupper():
        return [w.lower() for w in string.split('_')]
    else:
        regex = ''.join((re.escape(w) + '|') for w in dictionary) + '[a-z0-9]+|[A-Z][a-z0-9]*'
        return [(w if w in dictionary else w.lower()) for w in re.findall(regex, string)]

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

def to_pascal_case(s): return ''.join([(w.title() if not w in dictionary else w) for w in get_words(s)])
def to_camel_case(s): return ''.join([(c.lower() if i == 0 else c) for i, c in enumerate(''.join([w.title() for w in get_words(s)]))])
def to_macro_case(s): return '_'.join(get_words(s)).upper()
def to_snake_case(s): return '_'.join(get_words(s)).lower()
def to_kebab_case(s): return '-'.join(get_words(s)).lower()

value_types = OrderedDict({
    'bool': {'builtin': True, 'fullname': 'bool', 'name': 'bool', 'c_name': 'bool', 'py_type': 'bool'},
    'float32': {'builtin': True, 'fullname': 'float32', 'name': 'float32', 'c_name': 'float', 'py_type': 'float'},
    'uint8': {'builtin': True, 'fullname': 'uint8', 'name': 'uint8', 'c_name': 'uint8_t', 'py_type': 'int'},
    'uint16': {'builtin': True, 'fullname': 'uint16', 'name': 'uint16', 'c_name': 'uint16_t', 'py_type': 'int'},
    'uint32': {'builtin': True, 'fullname': 'uint32', 'name': 'uint32', 'c_name': 'uint32_t', 'py_type': 'int'},
    'uint64': {'builtin': True, 'fullname': 'uint64', 'name': 'uint64', 'c_name': 'uint64_t', 'py_type': 'int'},
    'int8': {'builtin': True, 'fullname': 'int8', 'name': 'int8', 'c_name': 'int8_t', 'py_type': 'int'},
    'int16': {'builtin': True, 'fullname': 'int16', 'name': 'int16', 'c_name': 'int16_t', 'py_type': 'int'},
    'int32': {'builtin': True, 'fullname': 'int32', 'name': 'int32', 'c_name': 'int32_t', 'py_type': 'int'},
    'int64': {'builtin': True, 'fullname': 'int64', 'name': 'int64', 'c_name': 'int64_t', 'py_type': 'int'},
    'endpoint_ref': {'builtin': True, 'fullname': 'endpoint_ref', 'name': 'endpoint_ref', 'c_name': 'endpoint_ref_t', 'py_type': '[not implemented]'},
})

enums = OrderedDict()
interfaces = OrderedDict()
userdata = OrderedDict() # Arbitrary data passed from the definition file to the template

def make_ref_type(interface):
    name = 'Ref<' + interface.fullname + '>'
    fullname = join_name('fibre', name)
    if fullname in interfaces:
        return interfaces[fullname]

    ref_type = {
        'builtin': True,
        'name': name,
        'fullname': fullname,
        'c_name': interface.fullname.replace('.', 'Intf::') + 'Intf*'
    }
    value_types[fullname] = ref_type

    return ref_type

def get_dict(elem, key):
    return elem.get(key, None) or OrderedDict()

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
    elem['in'] = OrderedDict((n, regularize_arg(path, n, arg))
                             for n, arg in (*prepend_args.items(), *get_dict(elem, 'in').items()))
    elem['out'] = OrderedDict((n, regularize_arg(path, n, arg))
                              for n, arg in get_dict(elem, 'out').items())
    return elem

def regularize_attribute(parent, name, elem, c_is_class):
    if elem is None:
        elem = {}
    if isinstance(elem, str):
        elem = {'type': elem}
    elif not 'type' in elem:
        elem['type'] = {}
        if 'attributes' in elem: elem['type']['attributes'] = elem.pop('attributes')
        if 'functions' in elem: elem['type']['functions'] = elem.pop('functions')
        if 'implements' in elem: elem['type']['implements'] = elem.pop('implements')
        if 'c_is_class' in elem: elem['type']['c_is_class'] = elem.pop('c_is_class')
        if 'values' in elem: elem['type']['values'] = elem.pop('values')
        if 'flags' in elem: elem['type']['flags'] = elem.pop('flags')
        if 'nullflag' in elem: elem['type']['nullflag'] = elem.pop('nullflag')
    
    elem['name'] = name
    elem['fullname'] = join_name(parent.fullname, name)
    elem['parent'] = parent
    elem['typeargs'] = elem.get('typeargs', {})
    elem['c_name'] = elem.get('c_name', None) or (elem['name'] + ('_' if c_is_class else ''))
    if ('c_getter' in elem) or ('c_setter' in elem):
        elem['c_getter'] = elem.get('c_getter', elem['c_name'])
        elem['c_setter'] = elem.get('c_setter', elem['c_name'] + ' = ')

    if isinstance(elem['type'], str) and elem['type'].startswith('readonly '):
        elem['typeargs']['fibre.Property.mode'] = 'readonly'
        elem['typeargs']['fibre.Property.type'] = elem['type'][len('readonly '):]
        elem['type'] = InterfaceRefElement(parent.fullname, None, 'fibre.Property', elem['typeargs'])
        if elem['typeargs']['fibre.Property.mode'] == 'readonly' and 'c_setter' in elem: elem.pop('c_setter')
    elif ('flags' in elem['type']) or ('values' in elem['type']):
        elem['typeargs']['fibre.Property.mode'] = elem['typeargs'].get('fibre.Property.mode', None) or 'readwrite'
        elem['typeargs']['fibre.Property.type'] = regularize_valuetype(parent.fullname, to_pascal_case(name), elem['type'])
        elem['type'] = InterfaceRefElement(parent.fullname, None, 'fibre.Property', elem['typeargs'])
        if elem['typeargs']['fibre.Property.mode'] == 'readonly' and 'c_setter' in elem: elem.pop('c_setter')
    else:
        elem['type'] = InterfaceRefElement(parent.fullname, to_pascal_case(name), elem['type'], elem['typeargs'])
    return elem

class InterfaceRefElement():
    def __init__(self, scope, name, elem, typeargs):
        if isinstance(elem, str):
            self._intf = None
            self._scope = scope
            self._name = elem
        else:
            self._intf = InterfaceElement(scope, name, elem)
            self._scope = None
            self._name = None
        self._typeargs = typeargs

    def resolve(self):
        """
        Resolves this interface reference to an actual InterfaceElement instance.
        The innermost scope is searched first.
        At every scope level, if no matching interface is found, it is checked if a
        matching value type exists. If so, the interface type fibre.Property<value_type>
        is returned.
        """
        if not self._intf is None:
            return self._intf

        typeargs = self._typeargs
        if 'fibre.Property.type' in typeargs:
            typeargs['fibre.Property.type'] = resolve_valuetype(self._scope, typeargs['fibre.Property.type'])

        scope = self._scope.split('.')
        for probe_scope in [join_name(*scope[:(len(scope)-i)]) for i in range(len(scope)+1)]:
            probe_name = join_name(probe_scope, self._name)
            #print('probing ' + probe_name)
            if probe_name in interfaces:
                return interfaces[probe_name]
            elif probe_name in value_types:
                typeargs['fibre.Property.type'] = value_types[probe_name]
                return make_property_type(typeargs)
            elif probe_name in generics:
                return generics[probe_name](typeargs)

        raise Exception('could not resolve type {} in {}. Known interfaces are: {}. Known value types are: {}'.format(self._name, self._scope, list(interfaces.keys()), list(value_types.keys())))

class InterfaceElement():
    def __init__(self, path, name, elem):
        if elem is None:
            elem = {}
        assert(isinstance(elem, dict))
        
        path = join_name(path, name)
        interfaces[path] = self

        self.name = split_name(name)[-1]
        self.fullname = path
        self.c_name = elem.get('c_name', self.fullname.replace('.', 'Intf::')) + 'Intf'
        
        if not 'implements' in elem:
            elem['implements'] = []
        elif isinstance(elem['implements'], str):
            elem['implements'] = [elem['implements']]
        self.implements = [InterfaceRefElement(path, None, elem, {}) for elem in elem['implements']]
        self.functions = OrderedDict((name, regularize_func(path, name, func, {'obj': {'type': make_ref_type(self)}}))
                                        for name, func in get_dict(elem, 'functions').items())
        if not 'c_is_class' in elem:
            raise Exception(elem)
        treat_as_class = elem['c_is_class'] # TODO: add command line arg to make this selectively optional
        self.attributes = OrderedDict((name, regularize_attribute(self, name, prop, treat_as_class))
                                        for name, prop in get_dict(elem, 'attributes').items())
        self.interfaces = []
        self.enums = []

    def get_all_attributes(self, stack=[]):
        result = OrderedDict()
        for intf in self.implements:
            assert(not self in stack)
            result.update(intf.get_all_attributes(stack + [self]))
        result.update(self.attributes)
        return result

    def get_all_functions(self, stack=[]):
        result = OrderedDict()
        for intf in self.implements:
            assert(not self in stack)
            result.update(intf.get_all_functions(stack + [self]))
        result.update(self.functions)
        return result

class PropertyInterfaceElement(InterfaceElement):
    def __init__(self, name, fullname, mode, value_type):
        self.name = name
        self.fullname = fullname
        self.purename = 'fibre.Property'
        self.c_name = 'Property<' + ('const ' if mode == 'readonly' else '') + value_type['c_name'] + '>'
        self.value_type = value_type # TODO: should be a metaarg
        self.mode = mode # TODO: should be a metaarg
        self.builtin = True
        self.attributes = OrderedDict()
        self.functions = OrderedDict()
        if mode != 'readonly':
            self.functions['exchange'] = {
                'name': 'exchange',
                'fullname': join_name(fullname, 'exchange'),
                'in': OrderedDict([('obj', {'name': 'obj', 'type': {'c_name': self.c_name}}), ('value', {'name': 'value', 'type': value_type, 'optional': True})]),
                'out': OrderedDict([('value', {'name': 'value', 'type': value_type})]),
                #'implementation': 'fibre_property_exchange<' + value_type['c_name'] + '>'
            }
        else:
            self.functions['read'] = {
                'name': 'read',
                'fullname': join_name(fullname, 'read'),
                'in': OrderedDict([('obj', {'name': 'obj', 'type': {'c_name': self.c_name}})]),
                'out': OrderedDict([('value', {'name': 'value', 'type': value_type})]),
                #'implementation': 'fibre_property_read<' + value_type['c_name'] + '>'
            }

        interfaces[fullname] = self # TODO: not good to write to a global here

def make_property_type(typeargs):
    value_type = resolve_valuetype('', typeargs['fibre.Property.type'])
    mode = typeargs.get('fibre.Property.mode', 'readwrite')
    name = 'Property<' + value_type['fullname'] + ', ' + mode + '>'
    fullname = join_name('fibre', name)
    if fullname in interfaces:
        return interfaces[fullname]
    else:
        return PropertyInterfaceElement(name, fullname, mode, value_type)

generics = {
    'fibre.Property': make_property_type # TODO: improve generic support
}

def regularize_valuetype(path, name, elem):
    if elem is None:
        elem = {}
    if isinstance(elem, str):
        return elem # will be resolved during type resolution
    elem['name'] = split_name(name)[-1]
    elem['fullname'] = path = join_name(path, name)
    elem['c_name'] = elem.get('c_name', elem['fullname'].replace('.', 'Intf::'))
    value_types[path] = elem

    if 'flags' in elem: # treat as flags
        bit = 0
        for k, v in elem['flags'].items():
            elem['flags'][k] = elem['flags'][k] or OrderedDict()
            elem['flags'][k]['name'] = k
            current_bit = elem['flags'][k].get('bit', bit)
            elem['flags'][k]['bit'] = current_bit
            elem['flags'][k]['value'] = 0 if current_bit is None else (1 << current_bit)
            bit = bit if current_bit is None else current_bit + 1
        if 'nullflag' in elem:
            elem['flags'] = OrderedDict([(elem['nullflag'], OrderedDict({'name': elem['nullflag'], 'value': 0, 'bit': None})), *elem['flags'].items()])
        elem['values'] = elem['flags']
        elem['is_flags'] = True
        elem['is_enum'] = True
        enums[path] = elem

    elif 'values' in elem: # treat as enum
        val = 0
        for k, v in elem['values'].items():
            elem['values'][k] = elem['values'][k] or OrderedDict()
            elem['values'][k]['name'] = k
            val = elem['values'][k].get('value', val)
            elem['values'][k]['value'] = val
            val += 1
        enums[path] = elem
        elem['is_enum'] = True
    
    return elem

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

    raise Exception('could not resolve type {} in {}. Known value types are: {}'.format(name, join_name(*scope), list(value_types.keys())))


def map_to_fibre01_type(t):
    if t.get('is_enum', False):
        max_val = max(v['value'] for v in t['values'].values())
        if max_val <= 0xff:
            return 'uint8'
        elif max_val <= 0xffff:
            return 'uint16'
        elif max_val <= 0xffffffff:
            return 'uint32'
        elif max_val <= 0xffffffffffffffff:
            return 'uint64'
        else:
            raise Exception("enum with a maximum value of " + str(max_val) + " not supported")
    elif t['fullname'] == 'float32':
        return 'float'
    return t['fullname']

def generate_endpoint_for_property(prop, attr_bindto, idx):
    prop_intf = interfaces[prop['type'].fullname]

    endpoint = {
        'id': idx,
        'function': prop_intf.functions['read' if prop['type'].mode == 'readonly' else 'exchange'],
        'in_bindings': OrderedDict([('obj', attr_bindto)]),
        'out_bindings': OrderedDict()
    }
    endpoint_definition = {
        'name': prop['name'],
        'id': idx,
        'type': map_to_fibre01_type(prop['type'].value_type),
        'access': 'r' if prop['type'].mode == 'readonly' else 'rw',
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

    for k, prop in intf.get_all_attributes().items():
        property_value_type = re.findall('^fibre\.Property<([^>]*), (readonly|readwrite)>$', prop['type'].fullname)
        #attr_bindto = join_name(bindto, bindings_map.get(join_name(intf['fullname'], k), k + ('_' if len(intf['functions']) or (intf['fullname'] in treat_as_classes) else '')))
        attr_bindto = intf.c_name + '::get_' + prop['name'] + '(' + bindto + ')'
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

    for k, func in intf.get_all_functions().items():
        endpoints.append({
            'id': idx + cnt,
            'function': func,
            'in_bindings': OrderedDict([('obj', bindto), *[(k_arg, '(' + bindto + ')->' + func['name'] + '_in_' + k_arg + '_') for k_arg in list(func['in'].keys())[1:]]]),
            'out_bindings': OrderedDict((k_arg, '&(' + bindto + ')->' + func['name'] + '_out_' + k_arg + '_') for k_arg in func['out'].keys()),
        })
        in_def = []
        out_def = []
        for i, (k_arg, arg) in enumerate(list(func['in'].items())[1:]):
            endpoint, endpoint_definition = generate_endpoint_for_property({
                'name': arg['name'],
                'type': make_property_type({'fibre.Property.type': arg['type'], 'fibre.Property.mode': 'readwrite'})
            }, intf.c_name + '::get_' + func['name'] + '_in_' + k_arg + '_' + '(' + bindto + ')', idx + cnt + 1 + i)
            endpoints.append(endpoint)
            in_def.append(endpoint_definition)
        for i, (k_arg, arg) in enumerate(func['out'].items()):
            endpoint, endpoint_definition = generate_endpoint_for_property({
                'name': arg['name'],
                'type': make_property_type({'fibre.Property.type': arg['type'], 'fibre.Property.mode': 'readonly'})
            }, intf.c_name + '::get_' + func['name'] + '_out_' + k_arg + '_' + '(' + bindto + ')', idx + cnt + len(func['in']) + i)
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
parser.add_argument("-d", "--definitions", type=argparse.FileType('r', encoding='utf-8'), nargs='+',
                    help="the YAML interface definition file(s) used to generate the code")
parser.add_argument("-t", "--template", type=argparse.FileType('r', encoding='utf-8'),
                    help="the code template")
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("-o", "--output", type=argparse.FileType('w', encoding='utf-8'),
                    help="path of the generated output")
group.add_argument("--outputs", type=str,
                    help="path pattern for the generated outputs. One output is generated for each interface. Use # as placeholder for the interface name.")
parser.add_argument("--generate-endpoints", type=str, nargs='?',
                    help="if specified, an endpoint table will be generated and passed to the template for the specified interface")
args = parser.parse_args()

if args.version:
    print("0.0.1")
    sys.exit(0)


definition_files = args.definitions
template_file = args.template


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
    interfaces.update(get_dict(file_content, 'interfaces'))
    value_types.update(get_dict(file_content, 'valuetypes'))
    userdata.update(get_dict(file_content, 'userdata'))
    dictionary += file_content.get('dictionary', None) or []


# Preprocess definitions

# Regularize everything into a wellknown form
for k, item in list(interfaces.items()):
    InterfaceElement('', k, item)
for k, item in list(value_types.items()):
    regularize_valuetype('', k, item)

if args.verbose:
    print('Known interfaces: ' + ''.join([('\n  ' + k) for k in interfaces.keys()]))
    print('Known value types: ' + ''.join([('\n  ' + k) for k in value_types.keys()]))

clashing_names = list(set(value_types.keys()).intersection(set(interfaces.keys())))
if len(clashing_names):
    print("**Error**: Found both an interface and a value type with the name {}. This is not allowed, interfaces and value types (such as enums) share the same namespace.".format(clashing_names[0]), file=sys.stderr)
    sys.exit(1)

# Resolve all types to references
for _, item in list(interfaces.items()):
    item.implements = [ref.resolve() for ref in item.implements]
    for _, prop in item.attributes.items():
        prop['type'] = prop['type'].resolve()
    for _, func in item.functions.items():
        for _, arg in func['in'].items():
            arg['type'] = resolve_valuetype(item.fullname, arg['type'])
        for _, arg in func['out'].items():
            arg['type'] = resolve_valuetype(item.fullname, arg['type'])

# Attach interfaces to their parents
toplevel_interfaces = []
for k, item in list(interfaces.items()):
    k = split_name(k)
    item.parent = None
    if len(k) == 1:
        toplevel_interfaces.append(item)
    else:
        if k[:-1] != ['fibre']: # TODO: remove special handling
            parent = interfaces[join_name(*k[:-1])]
            parent.interfaces.append(item)
            item.parent = parent
toplevel_enums = []
for k, item in list(enums.items()):
    k = split_name(k)
    if len(k) == 1:
        toplevel_enums.append(item)
    else:
        if k[:-1] != ['fibre']: # TODO: remove special handling
            parent = interfaces[join_name(*k[:-1])]
            parent.enums.append(item)
            item['parent'] = parent


if args.generate_endpoints:
    endpoints, embedded_endpoint_definitions, _ = generate_endpoint_table(interfaces[args.generate_endpoints], '&ep_root', 1) # TODO: make user-configurable
    embedded_endpoint_definitions = [{'name': '', 'id': 0, 'type': 'json', 'access': 'r'}] + embedded_endpoint_definitions
    endpoints = [{'id': 0, 'function': {'fullname': 'endpoint0_handler', 'in': {}, 'out': {}}, 'bindings': {}}] + endpoints
else:
    embedded_endpoint_definitions = None
    endpoints = None


# Render template

env = jinja2.Environment(
    comment_start_string='[#', comment_end_string='#]',
    block_start_string='[%', block_end_string='%]',
    variable_start_string='[[', variable_end_string=']]'
)

def tokenize(text, interface, interface_transform, value_type_transform, attribute_transform):
    """
    Looks for referencable tokens (interface names, value type names or
    attribute names) in a documentation text and runs them through the provided
    processing functions.
    Tokens are detected by enclosing back-ticks (`).

    interface: The interface type object that defines the scope in which the
               tokens should be detected.
    interface_transform: A function that takes an interface object as an argument
                         and returns a string.
    value_type_transform: A function that takes a value type object as an argument
                          and returns a string.
    attribute_transform: A function that takes the token string, an interface object
                         and an attribute object as arguments and returns a string.
    """
    if text is None or isinstance(text, jinja2.runtime.Undefined):
        return text

    def token_transform(token):
        token = token.groups()[0]

        if ':' in token:
            intf_name, _, attr_name = token.partition(':')
            intf = InterfaceRefElement(interface.fullname, None, intf_name, []).resolve()
            token = attr_name
        else:
            intf = interface

        token_list = split_name(token)

        # Check if this is an attribute reference
        scope = intf
        attr = None

        for name in token_list:
            if scope.fullname == 'ODrive': # TODO: this is a temporary hack
                scope = interfaces['ODrive3']
            if (not name.endswith('()')) and name in scope.get_all_attributes():
                attr = scope.get_all_attributes()[name]
                scope = attr['type']
            elif name.endswith('()') and name[:-2] in scope.get_all_functions():
                attr = scope.get_all_functions()[name[:-2]]
                scope = None # TODO
            else:
                print('Warning: cannot resolve "{}" in {}'.format(token, intf.fullname))
                return "`" + token + "`"
        
        return attribute_transform(token, interface, intf, attr)


    return re.sub(r'`([A-Za-z0-9\.:_]+)`', token_transform, text)

def html_escape(text):
    import html
    return html.escape(str(text))

env.filters['to_pascal_case'] = to_pascal_case
env.filters['to_camel_case'] = to_camel_case
env.filters['to_macro_case'] = to_macro_case
env.filters['to_snake_case'] = to_snake_case
env.filters['to_kebab_case'] = to_kebab_case
env.filters['first'] = lambda x: next(iter(x))
env.filters['skip_first'] = lambda x: list(x)[1:]
env.filters['to_c_string'] = lambda x: '\n'.join(('"' + line.replace('"', '\\"') + '"') for line in json.dumps(x, separators=(',', ':')).replace('{"name"', '\n{"name"').split('\n'))
env.filters['tokenize'] = tokenize
env.filters['html_escape'] = html_escape
env.filters['diagonalize'] = lambda lst: [lst[:i + 1] for i in range(len(lst))]
env.filters['debug'] = lambda x: print(x)

template = env.from_string(template_file.read())

template_args = {
    'interfaces': interfaces,
    'value_types': value_types,
    'toplevel_interfaces': toplevel_interfaces,
    'userdata': userdata,
    'endpoints': endpoints,
    'embedded_endpoint_definitions': embedded_endpoint_definitions
}

if not args.output is None:
    output = template.render(**template_args)
    args.output.write(output)
else:
    assert('#' in args.outputs)

    for k, intf in interfaces.items():
        if split_name(k)[0] == 'fibre':
            continue # TODO: remove special case
        output = template.render(interface = intf, **template_args)
        with open(args.outputs.replace('#', k.lower()), 'w', encoding='utf-8') as output_file:
            output_file.write(output)

    for k, enum in value_types.items():
        if enum.get('builtin', False) or not enum.get('is_enum', False):
            continue
        output = template.render(enum = enum, **template_args)
        with open(args.outputs.replace('#', k.lower()), 'w', encoding='utf-8') as output_file:
            output_file.write(output)
