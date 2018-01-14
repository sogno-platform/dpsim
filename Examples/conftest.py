import os
import subprocess
import pytest

def pytest_collect_file(parent, path):
    if path.ext == ".yml" and path.basename.startswith("test_"):
        return YamlFile(path, parent)

class YamlFile(pytest.File):
    def collect(self):
        import yaml # we need a yaml parser, e.g. PyYAML
        raw = yaml.safe_load(self.fspath.open())
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
            self.shell = spec['check']
        else:
            self.shell = True

        if 'timeout' in spec:
            self.timeout = spec['timeout']
        else:
            self.timeout = 60

        if 'cwd' in spec:
            self.cwd = spec['cwd']
        else:
            self.cwd = os.path.realpath(os.path.dirname(__file__) + '/..')

        if 'skip' in spec and spec['skip']:
            self.add_marker(pytest.mark.skip)
        if 'xfail' in spec and spec['xfail']:
            self.add_marker(pytest.mark.xfail)

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
