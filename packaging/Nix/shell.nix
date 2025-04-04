{
  mkShell,
  dpsim,
  python3,
  pre-commit,
  clang-tools,
  ruby
}:
mkShell {
  inputsFrom = [ dpsim ];

  packages = [
    (python3.withPackages (ps: with ps; [ numpy ]))

    pre-commit
    ruby # Required for pre-commit
    clang-tools
  ];
}
