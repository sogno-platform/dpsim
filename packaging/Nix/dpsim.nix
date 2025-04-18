{
  lib,
  stdenv,

  makeFontsConf,

  cimpp,
  cmake,
  doxygen,
  eigen,
  fmt,
  graphviz,
  gsl,
  nlohmann_json,
  pkg-config,
  python312,
  python312Packages,
  readerwriterqueue,
  spdlog,
  sphinx,
  sundials321,
  suitesparse-dpsim,
  villas-node,
  freefont_ttf,

  cudatoolkit,
  magma,

  # Options
  withExamples ? false,
  withAllExtras ? true,
  withOpenMP ? withAllExtras,
  withCIMpp ? withAllExtras,
  withDocumentation ? withAllExtras,
  withVILLAS ? withAllExtras,
  withGSL ? withAllExtras,
  withGraphviz ? withAllExtras,
  withPybind ? withAllExtras,
  withSundials ? withAllExtras,
  withSuiteSparse ? withAllExtras,
  withMNASolverPlugin ? withAllExtras,
}:
stdenv.mkDerivation {
  name = "dpsim";
  src = ../..;

  nativeBuildInputs =
    [
      cmake
      pkg-config
    ]
    ++ lib.optionals withDocumentation [
      doxygen
      sphinx
      python312Packages.sphinx-rtd-theme
    ];

  buildInputs =
    [
      eigen
      fmt
      spdlog
      nlohmann_json
      readerwriterqueue

      # TODO: Add these dependencies
      # cudatoolkit
      # magma
    ]
    ++ lib.optional withCIMpp cimpp
    ++ lib.optional withVILLAS villas-node
    ++ lib.optional withGSL gsl
    ++ lib.optional withGraphviz graphviz
    ++ lib.optional withSundials sundials321
    ++ lib.optional withSuiteSparse suitesparse-dpsim
    ++ lib.optionals withPybind [
      python312
      python312Packages.pybind11
    ];

  enableParallelBuilding = true;

  cmakeFlags = [
    # LTO is currently broken in Nixpkgs (https://github.com/NixOS/nixpkgs/issues/384599)
    "-DWITH_LTO=OFF"

    "-DBUILD_SHARED_LIBS=ON"

    # Feature flags
    "-DWITH_OPENMP=${if withOpenMP then "ON" else "OFF"}"
    "-DWITH_CIM=${if withCIMpp then "ON" else "OFF"}"
    "-DWITH_VILLAS=${if withVILLAS then "ON" else "OFF"}"
    "-DWITH_GSL=${if withGSL then "ON" else "OFF"}"
    "-DWITH_MNASOLVER_PLUGIN=${if withMNASolverPlugin then "ON" else "OFF"}"

    "-DDPSIM_BUILD_EXAMPLES=${if withExamples then "ON" else "OFF"}"
    "-DDPSIM_BUILD_DOC=${if withDocumentation then "ON" else "OFF"}"

    # We can not fetch external code within the Nix sandbox
    # Instead we packaged those dependencies as separate Nix derivations.
    "-DFETCH_EIGEN=OFF"
    "-DFETCH_SUITESPARSE=OFF"
    "-DFETCH_SPDLOG=OFF"
    "-DFETCH_CIMPP=OFF"
    "-DFETCH_PYBIND=OFF"
    "-DFETCH_GRID_DATA=OFF"
    "-DFETCH_FILESYSTEM=OFF"
    "-DFETCH_JSON=OFF"
    "-DFETCH_READERWRITERQUEUE=OFF"
  ];

  FONTCONFIG_FILE = makeFontsConf { fontDirectories = [ freefont_ttf ]; };

  # For building docs diagrams
  preBuild = ''
    export XDG_CACHE_HOME="$(mktemp -d)"
  '';
}
