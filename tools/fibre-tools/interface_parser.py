
from abc import ABC, abstractmethod
from collections import OrderedDict
import json
import jsonschema
import os
import re
from typing import List, NamedTuple
import yaml

from type_info import TypeInfo, BitfieldInfo, FlagInfo, EnumInfo, EnumeratorInfo, ClassInfo, FunctionInfo, ArgInfo, AttributeInfo, ResolvedTypeRef
from type_registry import TypeRegistry, TypeNameRef

class SourceInfo(NamedTuple):
    file: str
    line: int

# Source: https://stackoverflow.com/a/53647080/3621512
class SafeLineLoader(yaml.SafeLoader):
    def __init__(self, document_name, stream):
        super(SafeLineLoader, self).__init__(stream)
        self.document_name = document_name
        self.linenos = {}

    def compose_node(self, parent, index):
        # the line number where the previous token has ended (plus empty lines)
        line = self.line
        node = super(SafeLineLoader, self).compose_node(parent, index)
        node.__line__ = line + 1
        if parent is None:
            node.path = []
        return node

    def construct_mapping(self, node, deep=False):
        self.flatten_mapping(node)
        for key_node, value_node in node.value:
            value_node.path = node.path + [key_node.value]
        pairs = self.construct_pairs(node)
        if len(set([k for k, v in pairs])) != len(pairs):
                raise Exception("duplicate keys")
        #import ipdb; ipdb.set_trace()
        self.linenos['.'.join(node.path)] = SourceInfo(self.document_name, node.__line__)
        return OrderedDict(pairs)


def get_dict(elem, key):
    return elem.get(key, None) or OrderedDict()

dictionary = [] # TODO

def get_words(string):
    """
    Splits a string in PascalCase or MACRO_CASE into a list of lower case words
    """
    if string.isupper():
        return [w.lower() for w in string.split('_')]
    else:
        regex = ''.join((re.escape(w) + '|') for w in dictionary) + '[a-z0-9]+|[A-Z][a-z0-9]*'
        return [(w if w in dictionary else w.lower()) for w in re.findall(regex, string)]

def to_pascal_case(s): return ''.join([(w.title() if not w in dictionary else w) for w in get_words(s)])


class Loader():
    def __init__(self, registry):
        script_path = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(script_path, 'interface-schema.json')) as fp:
            schema = json.load(fp)
        self._validator = jsonschema.Draft4Validator(schema)
        self._registry = registry

        ns = registry.global_namespace.ns_from_path(('io', 'fibreframework'), construct_if_missing=True)
        ns.add_type(OpaqueTypeInfo('int8'), py_ref_type=('Int8Property',), py_val_type=('int',))
        ns.add_type(OpaqueTypeInfo('uint8'), py_ref_type=('Uint8Property',), py_val_type=('int',))
        ns.add_type(OpaqueTypeInfo('int16'), py_ref_type=('Int16Property',), py_val_type=('int',))
        ns.add_type(OpaqueTypeInfo('uint16'), py_ref_type=('Uint16Property',), py_val_type=('int',))
        ns.add_type(OpaqueTypeInfo('int32'), py_ref_type=('Int32Property',), py_val_type=('int',))
        ns.add_type(OpaqueTypeInfo('uint32'), py_ref_type=('Uint32Property',), py_val_type=('int',))
        ns.add_type(OpaqueTypeInfo('int64'), py_ref_type=('Int64Property',), py_val_type=('int',))
        ns.add_type(OpaqueTypeInfo('uint64'), py_ref_type=('Uint64Property',), py_val_type=('int',))
        ns.add_type(OpaqueTypeInfo('float32'), py_ref_type=('Float32Property',), py_val_type=('float',))
        ns.add_type(OpaqueTypeInfo('bool'), py_ref_type=('BoolProperty',), py_val_type=('bool',))
        ns.add_type(OpaqueTypeInfo('endpoint_ref'), py_ref_type=('EndpointRefProperty',), py_val_type=('RemoteObject',))

    def load_enum_from_fata(self, enum, enum_data):
        pass # TODO

    def get_property_interface(self, codec):
        return None # TODO

    def load_args_from_data(self, ns_path, arg_data, using):
        for k, arg_data in arg_data.items():
            if isinstance(arg_data, str) or (not 'type' in arg_data):
                arg_data = {'type': arg_data}

            if isinstance(arg_data['type'], str):
                type_ref = TypeNameRef(self._registry, [ns_path, *using], arg_data['type'])
            else:
                type = self.load_type_from_data(ns_path, to_pascal_case(k), arg_data['type'], using)
                type_ref = ResolvedTypeRef(type)

            yield ArgInfo(k, type_ref, arg_data.get('doc', None))

    def load_type_from_data(self, ns_path, type_name, type_data, using: List[List['NamespaceInfo']]):
        ns = self._registry.global_namespace.ns_from_path(ns_path, construct_if_missing=True)

        if ('flags' in type_data):
            bitfield = ns.get_type(type_name, kind=BitfieldInfo, construct_if_missing=True)

            for k, flag_data in type_data['flags'].items():
                if flag_data is None:
                    flag_data = {}
                bitfield.flags.append(FlagInfo(k,
                    flag_data.get('bit', bitfield.get_next_bit()),
                    flag_data.get('brief', None),
                    flag_data.get('doc', None)
                ))
                
            if 'nullflag' in type_data:
                bitfield.nullflag = type_data['nullflag']

            return bitfield

        elif ('values' in type_data):
            enum = ns.get_type(type_name, kind=EnumInfo, construct_if_missing=True)

            for k, flag_data in type_data['values'].items():
                if flag_data is None:
                    flag_data = {}
                enum.enumerators.append(EnumeratorInfo(k,
                    flag_data.get('value', enum.get_next_value()),
                    flag_data.get('brief', None),
                    flag_data.get('doc', None)
                ))

            return enum

        else:
            # class type

            cls = ns.get_type(type_name, kind=ClassInfo, construct_if_missing=True)

            for k, attr_data in get_dict(type_data, 'attributes').items():
                if isinstance(attr_data, str):
                    attr_data = {'type': attr_data}
                elif not 'type' in attr_data:
                    attr_data['type'] = {}
                    if 'attributes' in attr_data: attr_data['type']['attributes'] = attr_data.pop('attributes')
                    if 'functions' in attr_data: attr_data['type']['functions'] = attr_data.pop('functions')
                    if 'implements' in attr_data: attr_data['type']['implements'] = attr_data.pop('implements')
                    if 'c_is_class' in attr_data: attr_data['type']['c_is_class'] = attr_data.pop('c_is_class')
                    if 'values' in attr_data: attr_data['type']['values'] = attr_data.pop('values')
                    if 'flags' in attr_data: attr_data['type']['flags'] = attr_data.pop('flags')
                    if 'nullflag' in attr_data: attr_data['type']['nullflag'] = attr_data.pop('nullflag')

                if isinstance(attr_data['type'], str):
                    if attr_data['type'].startswith('readonly '):
                        attr_data['type'] = attr_data['type'][len('readonly '):]
                        attr_data['readonly'] = True
                    type_ref = TypeNameRef(self._registry, [(*ns_path, cls.name), *using], attr_data['type'])
                else:
                    type = self.load_type_from_data((*ns_path, cls.name), to_pascal_case(k), attr_data['type'], using)
                    type_ref = ResolvedTypeRef(type)

                cls.attributes.append(AttributeInfo(k,
                    type_ref,
                    attr_data.get('readonly', False),
                    attr_data.get('brief', None),
                    attr_data.get('doc', None)
                ))

            for k, func_data in get_dict(type_data, 'functions').items():
                if func_data is None:
                    func_data = {}
                in_args = list(self.load_args_from_data((*ns_path, cls.name), get_dict(func_data or {}, 'in'), using))
                out_args = list(self.load_args_from_data((*ns_path, cls.name), get_dict(func_data or {}, 'out'), using))
                cls.functions.append(FunctionInfo(k, in_args, out_args, func_data.get('brief', None), func_data.get('doc', None)))

            return cls


    def load_from_data(self, data):
        using = [tuple(u.split('.')) for u in (data.get('using', None) or [])]
        ns = tuple(data['ns'].split('.'))

        all_types = [
            *get_dict(data, 'interfaces').items(), # TODO: deprecate
            *get_dict(data, 'valuetypes').items(), # TODO: deprecate
            *get_dict(data, 'types').items(),
        ]

        for k, type_data in all_types:
            if k.startswith(':'):
                current_ns = ()
                k = k[1:]
            else:
                current_ns = ns
            long_type_name = k.split('.')
            self.load_type_from_data(current_ns + tuple(long_type_name[:-1]), long_type_name[-1], type_data, using)

    def load_from_yaml_file(self, file: str):
        with open(file) as fp:
            loader = SafeLineLoader(file, fp)
            file_content = loader.get_single_data()

        for err in self._validator.iter_errors(file_content):
            # TODO: print line number
            raise Exception(err.message + '\nat ' + str(list(err.absolute_path)))   

        file_content['using'] = ['io.fibreframework'] # TODO: remove
        self.load_from_data(file_content)


class OpaqueTypeInfo(TypeInfo):
    def __init__(self, name):
        self.name = name


if __name__ == "__main__":
    registry = TypeRegistry()


    loader = Loader(registry)
    loader.load_from_yaml_file('../../Firmware/odrive-interface.yaml')

    registry.resolve_all()
    
    intf1 = registry.get_class('com.odriverobotics.ODrive')
    intf2 = registry.get_class('com.odriverobotics.ODrive.Axis')
    intf3 = registry.get_class('com.odriverobotics.ODrive.Axis.Config')
    intf4 = registry.get_class('com.odriverobotics.ODrive.Motor')

    print(registry.get_py_ref_type_name(('com', 'odriverobotics', 'ODrive'), intf3))
    print(registry.get_py_ref_type_name(('com', 'odriverobotics', 'ODrive'), intf4))

    import ipdb; ipdb.set_trace()

