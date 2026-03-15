"""Extract Python documentation using the ast module.

Usage: python3 -c "$(cat this_file)" <source_file>

Outputs JSON to stdout matching the ModuleDoc schema.
"""
import ast
import json
import re
import sys
import os


class Extractor(ast.NodeVisitor):
    def __init__(self, source, filepath):
        self.symbols = []
        self.source = source
        self.filepath = filepath
        self.module_doc = None
        self.imports = []
        self.all_list = None
        self._source_lines = source.splitlines()

    def visit_Module(self, node):
        self.module_doc = ast.get_docstring(node)
        # Check for __all__
        for child in ast.iter_child_nodes(node):
            if isinstance(child, ast.Assign):
                for target in child.targets:
                    if isinstance(target, ast.Name) and target.id == '__all__':
                        if isinstance(child.value, (ast.List, ast.Tuple)):
                            self.all_list = [
                                elt.value for elt in child.value.elts
                                if isinstance(elt, ast.Constant) and isinstance(elt.value, str)
                            ]
        self.generic_visit(node)

    def visit_Import(self, node):
        for alias in node.names:
            self.imports.append(alias.name)

    def visit_ImportFrom(self, node):
        module = node.module or ''
        for alias in node.names:
            self.imports.append(f"{module}.{alias.name}")

    def visit_FunctionDef(self, node):
        self._extract_function(node, is_async=False)

    def visit_AsyncFunctionDef(self, node):
        self._extract_function(node, is_async=True)

    def visit_ClassDef(self, node):
        doc = ast.get_docstring(node)
        bases = [_name_str(b) for b in node.bases]
        decorators = [_name_str(d) for d in node.decorator_list]
        is_dataclass = any('dataclass' in d for d in decorators)

        methods = []
        fields = []

        for item in node.body:
            if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                method = self._build_function(item, isinstance(item, ast.AsyncFunctionDef))
                methods.append(method)
            elif isinstance(item, ast.AnnAssign) and is_dataclass:
                # @dataclass field
                name = _name_str(item.target) if item.target else ''
                type_str = _annotation_str(item.annotation) if item.annotation else None
                fields.append({
                    'name': name,
                    'type_str': type_str,
                    'doc': None,
                })

        # Extract __init__ params as pseudo-fields if not a dataclass
        if not is_dataclass:
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == '__init__':
                    for arg in item.args.args[1:]:  # skip self
                        name = arg.arg
                        type_str = _annotation_str(arg.annotation) if arg.annotation else None
                        fields.append({
                            'name': name,
                            'type_str': type_str,
                            'doc': None,
                        })
                    break

        deprecated = None
        for d in decorators:
            if 'deprecated' in d.lower():
                deprecated = d

        self.symbols.append({
            'kind': 'struct',
            'name': node.name,
            'visibility': 'public',
            'location': {'file': self.filepath, 'line': node.lineno, 'end_line': node.end_lineno},
            'doc': doc,
            'deprecated': deprecated,
            'generic_params': [],
            'fields': fields,
            'methods': methods,
            'trait_impls': bases,
            'derives': decorators,
            'examples': _extract_examples(doc) if doc else [],
        })

    def visit_Assign(self, node):
        # Module-level CONSTANT = value
        if not isinstance(node, ast.Assign):
            return
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id.isupper():
                value = _value_str(node.value)
                self.symbols.append({
                    'kind': 'constant',
                    'name': target.id,
                    'visibility': 'public',
                    'location': {'file': self.filepath, 'line': node.lineno, 'end_line': None},
                    'doc': None,
                    'deprecated': None,
                    'type_str': type(node.value).__name__ if hasattr(node.value, '__class__') else '?',
                    'value': value,
                })

    def _extract_function(self, node, is_async):
        func = self._build_function(node, is_async)
        self.symbols.append(func)

    def _build_function(self, node, is_async):
        doc = ast.get_docstring(node)
        decorators = [_name_str(d) for d in node.decorator_list]
        params = []
        defaults_offset = len(node.args.args) - len(node.args.defaults)

        for i, arg in enumerate(node.args.args):
            if arg.arg == 'self' or arg.arg == 'cls':
                continue
            type_str = _annotation_str(arg.annotation) if arg.annotation else None
            default_idx = i - defaults_offset
            default_value = None
            if default_idx >= 0 and default_idx < len(node.args.defaults):
                default_value = _value_str(node.args.defaults[default_idx])
            params.append({
                'name': arg.arg,
                'type_str': type_str,
                'default_value': default_value,
                'doc': None,
            })

        return_type = _annotation_str(node.returns) if node.returns else None

        # Build signature string
        async_prefix = 'async ' if is_async else ''
        param_strs = []
        for p in params:
            s = p['name']
            if p['type_str']:
                s += f": {p['type_str']}"
            if p['default_value']:
                s += f" = {p['default_value']}"
            param_strs.append(s)
        ret_str = f" -> {return_type}" if return_type else ''
        signature = f"{async_prefix}def {node.name}({', '.join(param_strs)}){ret_str}"

        deprecated = None
        for d in decorators:
            if 'deprecated' in d.lower():
                deprecated = d

        return {
            'kind': 'function',
            'name': node.name,
            'visibility': 'public' if not node.name.startswith('_') else 'private',
            'location': {'file': self.filepath, 'line': node.lineno, 'end_line': node.end_lineno},
            'signature': signature,
            'doc': doc,
            'deprecated': deprecated,
            'params': params,
            'returns': return_type,
            'is_async': is_async,
            'generic_params': [],
            'examples': _extract_examples(doc) if doc else [],
        }


def _name_str(node):
    if isinstance(node, ast.Name):
        return node.id
    elif isinstance(node, ast.Attribute):
        return f"{_name_str(node.value)}.{node.attr}"
    elif isinstance(node, ast.Call):
        return _name_str(node.func)
    elif isinstance(node, ast.Constant):
        return str(node.value)
    elif isinstance(node, ast.Subscript):
        return f"{_name_str(node.value)}[{_name_str(node.slice)}]"
    elif isinstance(node, ast.Tuple):
        return ', '.join(_name_str(e) for e in node.elts)
    return str(node) if node else '?'


def _annotation_str(node):
    if node is None:
        return None
    return _name_str(node)


def _value_str(node):
    if isinstance(node, ast.Constant):
        return repr(node.value)
    elif isinstance(node, ast.Name):
        return node.id
    elif isinstance(node, ast.List):
        return '[...]'
    elif isinstance(node, ast.Dict):
        return '{...}'
    elif isinstance(node, ast.Call):
        return f"{_name_str(node.func)}(...)"
    return '...'


def _extract_examples(doc):
    if not doc:
        return []
    examples = []
    in_example = False
    current = []
    for line in doc.splitlines():
        stripped = line.strip()
        if stripped.startswith('>>>'):
            in_example = True
            current.append(stripped)
        elif in_example and (stripped.startswith('...') or stripped == ''):
            if stripped:
                current.append(stripped)
            else:
                if current:
                    examples.append('\n'.join(current))
                    current = []
                in_example = False
        elif in_example:
            if current:
                examples.append('\n'.join(current))
                current = []
            in_example = False
    if current:
        examples.append('\n'.join(current))
    return examples


def scan_todos(source, filepath):
    todos = []
    pattern = re.compile(r'#\s*(TODO|FIXME|HACK|SAFETY)\s*[:(]?\s*(.*)')
    for i, line in enumerate(source.splitlines(), 1):
        m = pattern.search(line)
        if m:
            kind_map = {'TODO': 'todo', 'FIXME': 'fixme', 'HACK': 'hack', 'SAFETY': 'safety'}
            todos.append({
                'kind': kind_map.get(m.group(1), 'todo'),
                'text': m.group(2).strip(),
                'location': {'file': filepath, 'line': i, 'end_line': None},
            })
    return todos


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 extractor.py <file.py>', file=sys.stderr)
        sys.exit(1)

    filepath = sys.argv[1]
    source = open(filepath, 'r').read()

    try:
        tree = ast.parse(source)
    except SyntaxError as e:
        json.dump({
            'path': filepath,
            'language': 'python',
            'module_doc': None,
            'imports': [],
            'symbols': [],
            'todos': [],
            'error': str(e),
        }, sys.stdout)
        return

    extractor = Extractor(source, filepath)
    extractor.visit(tree)

    # Filter by __all__ if present
    symbols = extractor.symbols
    if extractor.all_list is not None:
        symbols = [s for s in symbols if s['name'] in extractor.all_list]

    todos = scan_todos(source, filepath)

    result = {
        'path': filepath,
        'language': 'python',
        'module_doc': extractor.module_doc,
        'imports': extractor.imports,
        'symbols': symbols,
        'todos': todos,
    }

    json.dump(result, sys.stdout)


if __name__ == '__main__':
    main()
