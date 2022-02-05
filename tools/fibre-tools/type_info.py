from abc import ABC, abstractmethod
from typing import List

class TypeInfo(ABC):
    pass

class TypeRef(ABC):
    @abstractmethod
    def resolve(self) -> 'TypeInfo':
        pass

class ResolvedTypeRef(TypeRef):
    def __init__(self, type: 'TypeInfo'):
        assert isinstance(type, TypeInfo)
        self.type = type
    def resolve(self):
        return self.type

class ArgInfo():
    def __init__(self, name: str, type: TypeRef, doc: str):
        self.name = name
        self.type = type
        self.doc = doc

class FunctionInfo():
    def __init__(self, name: str, input_args: List[ArgInfo], output_args: List[ArgInfo], brief: str, doc: str):
        self.name = name
        self.input_args = input_args
        self.output_args = output_args
        self.brief = brief
        self.doc = doc

class AttributeInfo():
    def __init__(self, name: str, type: TypeRef, readonly: bool, brief: str, doc: str):
        self.name = name
        self.type = type
        self.readonly = readonly
        self.brief = brief
        self.doc = doc

class FlagInfo():
    def __init__(self, name, bit, brief, doc):
        self.name = name
        self.brief = brief
        self.doc = doc
        self.bit = bit

class EnumeratorInfo():
    def __init__(self, name, value, brief, doc):
        self.name = name
        self.brief = brief
        self.doc = doc
        self.value = value

class ClassInfo(TypeInfo):
    label = 'class'

    @staticmethod
    def make_empty(name: str):
        return ClassInfo(name, [], [])

    def __init__(self, name: str, functions: List[FunctionInfo], attributes: List[AttributeInfo]):
        self.name = name
        self.functions = functions
        self.attributes = attributes
        self.py_ref_type = name # TODO

class BitfieldInfo(TypeInfo):
    label = 'bitfield'

    @staticmethod
    def make_empty(name: str):
        return BitfieldInfo(name, [])

    def __init__(self, name: str, flags: List[FlagInfo]):
        self.name = name
        self.py_ref_type = 'Property[' + name + ']' # TODO
        self.flags = flags

    def get_next_bit(self):
        return 0 if len(self.flags) == 0 else (self.flags[-1].bit + 1)

class EnumInfo(TypeInfo):
    label = 'enum'

    @staticmethod
    def make_empty(name: str):
        return EnumInfo(name, [])

    def __init__(self, name: str, enumerators: List[EnumeratorInfo]):
        self.name = name
        self.py_ref_type = 'Property[' + name + ']' # TODO
        self.enumerators = enumerators

    def get_next_value(self):
        return 0 if len(self.enumerators) == 0 else (self.enumerators[-1].value + 1)
