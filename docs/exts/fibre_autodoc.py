
from docutils import nodes
from docutils.nodes import Element, Node
from docutils.parsers.rst import Directive, directives
from docutils.statemachine import StringList
import os
import re
from sphinx import addnodes
from sphinx.application import Sphinx
from sphinx.util.docutils import SphinxDirective, switch_source_input

# from sphinx.util.typing import OptionSpec     # not present in windows/pip3 sphinx install
from typing import Dict, Callable, Any, Tuple   # needed to recreate OptionSpec
OptionSpec = Dict[str, Callable[[str], Any]]    # sphinx.util.typing.OptionSpec from GitHub

import sys
from typing import List

sys.path.append(os.path.abspath('../tools/fibre-tools'))
import interface_parser
import type_registry


def load_file(name, state):
    state.document.settings.record_dependencies.add(name)

def add_indent(lines: List[str], indent_depth=1):
    return [('   ' * indent_depth + l) for l in lines]

def format_docstring(obj, indent_depth=1):
    return [
        *(['', *add_indent(obj.brief.split('\n'), indent_depth)] if obj.brief else []),
        *(['', *add_indent(obj.doc.split('\n'), indent_depth)] if obj.doc else []),
    ]

class Documenter():
    pass

class MethodDocumenter(Documenter):
    objtype = 'method'
    
    #def load_object(self, registry, name: str):
    #    return registry.get_method(name)

    @staticmethod
    def generate(registry, decl_ns_path, method):
        in_str = ', '.join(arg.name for arg in method.input_args)
        if len(method.output_args) == 0:
            out_str = ''
        elif len(method.output_args) == 1:
            out_str = ' -> ' + registry.get_py_val_type_name(decl_ns_path, method.output_args[0].type)
        else:
            out_str = ' -> tuple[' + ', '.join(registry.get_py_val_type_name(decl_ns_path, arg.type) for arg in method.output_args) + ']'

        return [
            '',
            '.. py:method:: ' + method.name + '(' + in_str + ')' + out_str,
            *(format_docstring(method, indent_depth=1)),
            '',
            *(('   :param ' + registry.get_py_val_type_name(decl_ns_path, arg.type) + ' ' + arg.name + ':' + (' ' + arg.doc if arg.doc else '')) for arg in method.input_args),
            '',
        ]

class AttributeDocumenter(Documenter):
    objtype = 'attribute'

    @staticmethod
    def generate(registry, decl_ns_path, attr):
        return [
            '',
            '.. py:attribute:: ' + attr.name,
            '   :type: ' + registry.get_py_ref_type_name(decl_ns_path, attr.type),
            *(format_docstring(attr, indent_depth=1)),
            ''
        ]

class EnumDocumenter(Documenter):
    objtype = 'enum'
    
    @staticmethod
    def generate(registry, decl_ns_path, enum, options):
        lines = [
            '',
            '.. py:class:: ' + registry.get_py_val_type_name(decl_ns_path, enum),
            ''
        ]

        for enumerator in enum.enumerators:
            lines += [
                '',
                '   .. py:attribute:: ' + enumerator.name,
                '      :value: {} (0x{:X})'.format(enumerator.value, enumerator.value),
                *(format_docstring(enumerator, indent_depth=2)),
                ''
            ]

        return lines

class BitfieldDocumenter(Documenter):
    objtype = 'bitfield'
    
    @staticmethod
    def generate(registry, decl_ns_path, bitfield, options):
        lines = [
            '',
            '.. py:class:: ' + registry.get_py_val_type_name(decl_ns_path, bitfield),
            ''
        ]

        for flag in bitfield.flags:
            lines += [
                '',
                '   .. py:attribute:: ' + flag.name,
                '      :value: {} (0x{:X})'.format(1 << flag.bit, 1 << flag.bit),
                *(format_docstring(flag, indent_depth=2)),
                ''
            ]

        return lines

class ClassDocumenter(Documenter):
    objtype = 'class'
    
    @staticmethod
    def load_object(registry, name: str):
        cls = registry.get_class(name)
        return registry.get_containing_ns(cls), cls

    @staticmethod
    def generate(registry, decl_ns_path, cls, options):
        lines = [
            '',
            '.. py:class:: ' + registry.get_py_ref_type_name(decl_ns_path, cls),
            ''
        ]

        sub_decl_ns_path = registry.get_containing_ns(cls).get_path()[:2]

        for method in cls.functions:
            lines += add_indent(MethodDocumenter().generate(registry, sub_decl_ns_path, method))

        for attr in cls.attributes:
            lines += add_indent(AttributeDocumenter().generate(registry, sub_decl_ns_path, attr))

        return lines

class NamespaceDocumenter(Documenter):
    objtype = 'namespace'
    
    @staticmethod
    def load_object(registry, name: str):
        ns = registry.ns_from_name(name)
        return ns, ns

    @staticmethod
    def generate(registry, decl_ns_path, ns, options):
        lines = []

        for subtype in ns.types.values():
            if isinstance(subtype, interface_parser.EnumInfo) and ('enums' in options):
                lines += EnumDocumenter().generate(registry, decl_ns_path, subtype, options)
            elif isinstance(subtype, interface_parser.BitfieldInfo) and ('bitfields' in options):
                lines += BitfieldDocumenter().generate(registry, decl_ns_path, subtype, options)
            elif isinstance(subtype, interface_parser.ClassInfo) and ('classes' in options):
                lines += ClassDocumenter().generate(registry, decl_ns_path, subtype, options)
            else:
                raise Exception("Don't know how to document {} type".format(type(subtype)))

        if 'namespaces' in options:
            for sub_ns in ns.namespaces.values():
                lines += NamespaceDocumenter().generate(registry, decl_ns_path, sub_ns, options)

        return lines




documenter_list = [
    BitfieldDocumenter,
    EnumDocumenter,
    ClassDocumenter,
    NamespaceDocumenter,
]
documenters = {d.objtype: d for d in documenter_list}


class FibredocDirective(SphinxDirective):
    """
    Analogous to Sphinx autodoc class `AutodocDirective`.
    """
    required_arguments = 1
    optional_arguments = 0
    option_spec: OptionSpec = {
        'bitfields': directives.flag,
        'enums': directives.flag,
        'classes': directives.flag,
        'namespaces': directives.flag,
    }

    def run(self) -> List[Node]:
        # look up target Documenter
        objtype = self.name[5:]  # strip prefix (fibre-).
        documenter = documenters[objtype]()

        registry = self.env.app.fibre_registry
        
        for file in self.config.fibre_interface_files:
            self.env.note_dependency(file)

        decl_ns, obj = documenter.load_object(registry, self.arguments[0])
        lines = documenter.generate(registry, decl_ns.get_path()[:2], obj, self.options)

        result_rest = StringList()
        for line in lines:
            result_rest.append(line, 'fibre autogen output', 0)

        #print("reST output: ", result_rest)
        
        # Parse nested reST
        with switch_source_input(self.state, result_rest):
            node = nodes.paragraph()
            node.document = self.state.document
            self.state.nested_parse(result_rest, 0, node)
            return node.children

class fibresummary_toc(nodes.comment):
    pass

def autosummary_toc_visit_html(self: nodes.NodeVisitor, node: fibresummary_toc) -> None:
    """Hide autosummary toctree list in HTML output."""
    raise nodes.SkipNode

def autosummary_noop(self: nodes.NodeVisitor, node: Node) -> None:
    pass

class FibresummaryDirective(SphinxDirective):
    required_arguments = 1
    optional_arguments = 0
    option_spec: OptionSpec = {
        'caption': directives.unchanged_required,
    }

    def run(self) -> List[Node]:
        nodes = [] # TODO: generate table

        docnames = ['fibre_types/' + self.arguments[0].replace('.', '_')]

        tocnode = addnodes.toctree()
        tocnode['includefiles'] = docnames
        tocnode['entries'] = [(None, docn) for docn in docnames]
        tocnode['maxdepth'] = -1
        tocnode['glob'] = None
        tocnode['caption'] = self.options.get('caption')

        nodes.append(fibresummary_toc('', '', tocnode))
        return nodes

def load_yaml_files(app, config):
    registry = type_registry.TypeRegistry()
    loader = interface_parser.Loader(registry)

    for file in config.fibre_interface_files:
        loader.load_from_yaml_file(file)

    registry.resolve_all()

    app.fibre_registry = registry

def generate_stub_file(app: Sphinx, ns_path: Tuple[str], filename: str, deep: bool):
    title = '.'.join(ns_path[2:]) + " Reference"

    lines = [
        title,
        "=" * len(title),
    ]

    parent_ns = app.fibre_registry.global_namespace.ns_from_path(ns_path[:-1], construct_if_missing=False)
    if not parent_ns is None:
        type = parent_ns.get_type(ns_path[-1], kind=None, construct_if_missing=False)
        if not type is None:
            lines.extend([
                "",
                ".. fibreclass:: " + '.'.join(ns_path),
                "",
            ])

    #import ipdb; ipdb.set_trace()
    ns = app.fibre_registry.global_namespace.ns_from_path(ns_path, construct_if_missing=False)
    if not ns is None:
        lines.extend([
            "",
            ".. fibrenamespace:: " + '.'.join(ns_path),
            "   :bitfields:",
            "   :enums:",
            "   :classes:",
            "   :namespaces:",
            "",
        ])


    content = '\n'.join(lines)

    if os.path.isfile(filename):
        with open(filename) as fp:
            old_content = fp.read()
        if content == old_content:
            return False

    with open(filename, 'w') as fp:
        fp.write(content)
    return True
        

def find_autosummary_in_lines(lines, filename):
    """
    Inspired by find_autosummary_in_lines in the autosummary extension
    """
    autosummary_re = re.compile(r'^(\s*)\.\.\s+fibreautosummary::\s*([A-Za-z0-9_.:]+)\s*$')

    documented = []
    for line in lines:
        m = autosummary_re.match(line)
        if m:
            indent, name = m.groups()
            path = os.path.join(os.path.dirname(filename), 'fibre_types')
            documented.append((path, tuple(name.split('.'))))

    return documented

def generate_stub_files(app: Sphinx):
    """
    Inspired by process_generate_options in the autosummary extension.
    """
    env = app.builder.env
    genfiles = [env.doc2path(x, base=None) for x in env.found_docs
                if os.path.isfile(env.doc2path(x))]

    # read
    documented = []
    for filename in genfiles:
        with open(filename, encoding='utf-8', errors='ignore') as f:
            lines = f.read().splitlines()
            documented.extend(find_autosummary_in_lines(lines, filename=filename))

    # write
    for out_dir, ns_name in documented:
        os.makedirs(out_dir, exist_ok=True)
        out_file = os.path.join(out_dir, '_'.join(ns_name) + '.rst')
        generate_stub_file(app, ns_name, out_file, True)


def setup(app):
    app.add_node(fibresummary_toc,
                 html=(autosummary_toc_visit_html, autosummary_noop),
                 latex=(autosummary_noop, autosummary_noop),
                 text=(autosummary_noop, autosummary_noop),
                 man=(autosummary_noop, autosummary_noop),
                 texinfo=(autosummary_noop, autosummary_noop))

    app.add_config_value('fibre_interface_files', [], 'html')
    
    for d in documenter_list:
        app.add_directive('fibre' + d.objtype, FibredocDirective)
    app.add_directive('fibreautosummary', FibresummaryDirective)
    
    app.connect('config-inited', load_yaml_files)
    app.connect('builder-inited', generate_stub_files)

    return {
        'version': '0.1',
        'parallel_read_safe': False, # global state: loaded interfaces
        'parallel_write_safe': True,
    }
