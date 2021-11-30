
from typing import List, Tuple

from type_info import TypeInfo, TypeRef, ClassInfo, BitfieldInfo, EnumInfo


class NamespaceInfo():
    def __init__(self, registry: 'TypeRegistry', parent: 'NamespaceInfo'):
        self.registry = registry
        self.parent = parent
        self.namespaces = {}
        self.types = {}

    def get_path(self):
        return self.registry._ns_paths[self]

    def ns_from_path(self, path: Tuple[str], construct_if_missing: bool):
        ns = self
        ns_path = self.registry._ns_paths[self]
        for path_elem in path:
            ns_path += (path_elem,)
            sub_ns = ns.namespaces.get(path_elem, None)
            if sub_ns is None:
                if construct_if_missing:
                    sub_ns = NamespaceInfo(self.registry, ns)
                    ns.namespaces[path_elem] = sub_ns
                    self.registry._ns_paths[sub_ns] = ns_path
                else:
                    return None
            ns = sub_ns
            path = path[1:]
        return ns

    def add_type(self, type: 'TypeInfo', py_val_type: Tuple[str] = None, py_ref_type: Tuple[str] = None):
        self.types[type.name] = type
        self.registry._type_parents[type] = self
        if not py_val_type is None:
            self.registry._py_val_type_names[type] = py_val_type
        if not py_ref_type is None:
            self.registry._py_ref_type_names[type] = py_ref_type

    def get_type(self, name: str, kind, construct_if_missing: bool):
        """
        Returns the specified direct subtype or None if the subtype is unknown.
        """
        type = self.types.get(name, None)

        if type is None:
            if construct_if_missing:
                type = kind.make_empty(name)
                self.add_type(type)
        elif (not kind is None) and (not isinstance(type, kind)):
            if construct_if_missing:
                raise Exception("{} cannot be redefined as a {} because it is already defined as a {}.".format(name, kind.label, type.label))
            else:
                raise Exception("{} is not a {}.".format(name, kind.label))

        return type
        


def path_to_name(path):
    return '.'.join(path)
    #return ''.join(ns.name + ('.' if isinstance(ns, ClassInfo) else ':') for ns in path).rstrip('.:')

def split_path(name: str) -> Tuple[bool, Tuple[str]]:
    """
    Splits a type or namespace path of the form "io.fibreframework:Path.To.Type"
    into its components ('io', 'fibreframework', 'Path', 'To', 'Type').
    Returns a Tuple (global, path).
    """
    ns_name, colon, long_type_name = name.rpartition(':')
    path = (
        *((ns_name,) if colon else ()),
        *(long_type_name.split('.') if long_type_name else ())
    )
    return bool(colon), path




class TypeNameRef(TypeRef):
    def __init__(self, registry, scope: List['NamespaceInfo'], name: str):
        self.registry = registry
        self.scope = scope
        self.name = name

    def resolve(self):
        return self.registry.type_from_name(self.name, None, scope=self.scope)

class NotFoundException(Exception):
    def __init__(self, registry, name: Tuple[str], scope: List[Tuple[str]]):
        message = "No type \"{}\" found in {{{}}}.".format(
            path_to_name(name),
            ', '.join(path_to_name(path) for path in scope)
        )
        Exception.__init__(self, message)

class TypeRegistry():
    def __init__(self):
        self.global_namespace = NamespaceInfo(self, None)
        self._type_parents = {}
        self._ns_paths = {self.global_namespace: ()}
        self._py_ref_type_names = {} # maps TypeInfo objects to Python type paths
        self._py_val_type_names = {} # maps TypeInfo objects to Python type paths

    def ns_from_name(self, ns_name: str, scope=[()]):
        if ns_name.startswith(':'):
            ns_name = ns_name[1:]
            scope = [()]
        return self.ns_from_path(tuple(ns_name.split('.')), scope)

    def ns_from_path(self, ns_path: Tuple[str], scope=[()]):
        for path in scope:
            for i in range(len(path) + 1):
                partial_path = path[:(len(path)-i)]
                ns = self.global_namespace.ns_from_path(partial_path + ns_path, construct_if_missing=False)
                if not ns is None:
                    return ns

        raise NotFoundException(self, ns_path, scope)

    def type_from_name(self, type_name: str, kind, scope=[()]):
        if type_name.startswith(':'):
            type_name = type_name[1:]
            scope = [()]
        return self.type_from_path(tuple(type_name.split('.')), kind, scope)

    def type_from_path(self, type_path: Tuple[str], kind, scope=[()]):
        ns_path = type_path[:-1]

        ns_candidates = set()
        for path in scope:
            for i in range(len(path) + 1):
                partial_path = path[:(len(path)-i)]
                ns = self.global_namespace.ns_from_path(partial_path + ns_path, construct_if_missing=False)
                if not ns is None:
                    ns_candidates.add(ns)

        for ns_candidate in ns_candidates:
            type = ns_candidate.get_type(type_path[-1], kind, construct_if_missing=False)
            if not type is None:
                return type

        raise NotFoundException(self, type_path, scope)

    def get_class(self, name: str, scope=[()]) -> ClassInfo:
        """
        Returns the specified class.
        Raises an exception if the interface cannot be found.
        """
        return self.type_from_name(name, ClassInfo, scope)

    def resolve_all(self):
        """
        Resolves type references.
        This should be called after loading all files.
        """
        def resolve_ns(ns):
            for sub_ns in ns.namespaces.values():
                resolve_ns(sub_ns)
            for cls in [t for t in ns.types.values() if isinstance(t, ClassInfo)]:
                #cls.implements = [ref.resolve() for ref in cls.implements]
                for attr in cls.attributes:
                    attr.type = attr.type.resolve()
                for func in cls.functions:
                    for arg in func.input_args:
                        arg.type = arg.type.resolve()
                    for arg in func.output_args:
                        arg.type = arg.type.resolve()
        resolve_ns(self.global_namespace)

    def get_py_ref_type_name(self, decl_ns_path: Tuple[str], type):
        # TODO: combine with get_py_val_type_name
        type_ns_path = self.get_containing_ns(type).get_path()

        def get_py_full_ns_name(ns_path):
            return (ns_path[0].replace('.', '_').lower(),) + ns_path[1:]

        py_decl_path = get_py_full_ns_name(decl_ns_path)
        py_type_path = self._py_ref_type_names.get(type, get_py_full_ns_name(type_ns_path) + (type.name,))

        while len(py_decl_path) > 0 and len(py_type_path) > 1 and py_decl_path[0] == py_type_path[0]:
            py_decl_path = py_decl_path[1:]
            py_type_path = py_type_path[1:]

        p = '.'.join(py_type_path)
        if isinstance(type, BitfieldInfo) or isinstance(type, EnumInfo):
            p = 'Property[' + p + ']'
        return p

    def get_py_val_type_name(self, decl_ns_path: Tuple[str], type):
        # TODO: combine with get_py_ref_type_name
        type_ns_path = self.get_containing_ns(type).get_path()

        def get_py_full_ns_name(ns_path):
            return (ns_path[0].replace('.', '_').lower(),) + ns_path[1:]

        py_decl_path = get_py_full_ns_name(decl_ns_path)
        py_type_path = self._py_val_type_names.get(type, get_py_full_ns_name(type_ns_path) + (type.name,))

        while len(py_decl_path) > 0 and len(py_type_path) > 1 and py_decl_path[0] == py_type_path[0]:
            py_decl_path = py_decl_path[1:]
            py_type_path = py_type_path[1:]

        return '.'.join(py_type_path)

    def get_containing_ns(self, type):
        return self._type_parents[type]

