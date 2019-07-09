import os
import subprocess
import pytest

def pytest_collect_file(parent, path):
    if path.ext == ".yml" and path.basename.startswith("test_") and os.name == 'posix':
        return YamlFile(path, parent)
    if path.ext == '.ipynb':
        return JupyterNotebook(path, parent)

def parse_test_params(item, spec):
    if 'skip' in spec and spec['skip']:
        item.add_marker(pytest.mark.skip)
    if 'xfail' in spec and spec['xfail']:
        item.add_marker(pytest.mark.xfail)

class YamlFile(pytest.File):
    def collect(self):
        import yaml # we need a yaml parser, e.g. PyYAML
        raw = yaml.safe_load(self.fspath.open())

        if not raw:
            return

        for name, spec in sorted(raw.items()):
            yield YamlItem(name, self, spec)

class YamlItem(pytest.Item):
    def __init__(self, name, parent, spec):
        super(YamlItem, self).__init__(name, parent)

        if 'args' in spec:
            self.args = spec['args']
        else:
            self.args = [ ]

        if 'shell' in spec:
            self.shell = spec['shell']
        else:
            self.shell = False

        if 'check' in spec:
            self.check = spec['check']
        else:
            self.check = True

        if 'timeout' in spec:
            self.timeout = spec['timeout']
        else:
            self.timeout = 60

        if 'cwd' in spec:
            self.cwd = spec['cwd']
        else:
            self.cwd = os.path.realpath(os.path.dirname(__file__) + '/..')

        parse_test_params(self, spec)

        if 'cmd' in spec:
            self.cmd = spec['cmd']
        else:
            raise AttributeError('Test is missing mandatory "cmd" attribute')

    def runtest(self):
        cp = subprocess.run([self.cmd] + self.args,
            cwd = self.cwd,
            shell = self.shell,
            timeout = self.timeout,
            check = True
        )

    def repr_failure(self, excinfo):
        return self._repr_failure_py(excinfo, style="short")

    def reportinfo(self):
        return self.fspath, 0, "yaml: %s" % self.name


class JupyterNotebook(pytest.File):
    def collect(self):
        """
        See https://nbconvert.readthedocs.io/en/latest/nbconvert_library.html
        """
        import nbformat

        nb = nbformat.read(self.fspath, as_version=nbformat.NO_CONVERT)

        spec = {}
        for cell in nb.cells:
            if 'test' in cell.metadata:
                spec = { **spec, **cell.metadata['test'] }

        yield JupyterNotebookExport(nb, self, spec)

class JupyterNotebookExport(pytest.Item):

    def __init__(self, nb, parent, spec):
        self.base = os.path.basename(parent.name)
        self.name = os.path.splitext(self.base)[0]
        self.builddir = os.path.splitext(parent.name)[0]
        super(JupyterNotebookExport, self).__init__(self.name, parent)

        self.nb = nb

        parse_test_params(self, spec)

    def runtest(self):
        from traitlets.config import Config
        from nbconvert import HTMLExporter
        from nbconvert.writers import FilesWriter

        c = Config()
        c.FilesWriter.build_directory = 'outputs/' + self.builddir
        c.HTMLExporter.preprocessors = [
            'nbconvert.preprocessors.ExecutePreprocessor',
            'nbconvert.preprocessors.ExtractOutputPreprocessor'
        ]
        exporter = HTMLExporter(config=c)

        os.makedirs(c.FilesWriter.build_directory, exist_ok=True)

        (body, resources) = exporter.from_notebook_node(self.nb)

        writer = FilesWriter(config=c)
        writer.write(body, resources, notebook_name=self.name)

    def repr_failure(self, excinfo):
        from nbconvert.preprocessors import CellExecutionError

        if isinstance(excinfo.value, CellExecutionError):
            return self._repr_failure_py(excinfo, style="no")

        return self._repr_failure_py(excinfo, style="short")

    def reportinfo(self):
        return self.fspath, 0, "nbconvert: %s" % self.name
