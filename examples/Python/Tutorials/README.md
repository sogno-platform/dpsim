# Tutorial scripts

The scripts behind the [Python tutorials](../../../docs/hugo/content/en/docs/Tutorials/Python), one per
rung, numbered in the order they are meant to be worked through.

Each script is the code the corresponding page quotes, and the numbers printed are the numbers the
page states. A page and its script are expected to agree; if a change makes them diverge, the page
is wrong until it is updated.

Run one with the built extension module on the path:

```sh
PYTHONPATH=build python3 examples/Python/Tutorials/01_first_simulation.py
```

Results are written to `logs/` in the working directory, and reading them back uses the
`villas.dataprocessing` package that the example notebooks also use.

The numbering follows the tutorial order. `07` is reserved for a converter rung that is not
written yet, so the sequence skips it.
