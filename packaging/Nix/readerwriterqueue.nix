{
  fetchFromGitHub,
  stdenv,
  cmake,
}:
stdenv.mkDerivation {
  name = "readerwriterqueue";
  src = fetchFromGitHub {
    owner = "cameron314";
    repo = "readerwriterqueue";
    rev = "16b48ae1148284e7b40abf72167206a4390a4592";
    hash = "sha256-m4cUIXiDFxTguDZ7d0svjlOSkUNYY0bbUp3t7adBwOo";
  };

  enableParallelBuilding = true;

  nativeBuildInputs = [ cmake ];

  cmakeFlags = [ "-DCMAKE_INSTALL_LIBDIR=lib" ];

  # postInstall = ''
  #   cmake --install . --prefix "''${!outputDev}" --component Devel
  # '';
}
