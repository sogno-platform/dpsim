{mkShell, dpsim, python3, pre-commit, clang-tools }: mkShell {
  inputsFrom = [ dpsim ];

  packages = [
    (python3.withPackages (ps: with ps; [
      numpy
    ]))

    pre-commit
    clang-tools
  ];
}