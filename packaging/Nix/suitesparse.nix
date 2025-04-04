{ fetchFromGitHub, suitesparse }:
suitesparse.overrideAttrs (oldAttrs: {
  version = "5.10.6-dpsim";
  src = fetchFromGitHub {
    owner = "dpsim-simulator";
    repo = "SuiteSparse";
    rev = "release-v5.10.6";
    hash = "sha256-KUUfy8eT+xj/GFAsGOvkTfQevNyUwH1rJcDOW5hO9mw";
  };

  postPatch = ''
    substituteInPlace ./KLU/Source/klu_print.c --replace 'KLU_dumpKLU' '// KLU_dumpKLU'
  '';
})
