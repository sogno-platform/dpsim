{
  fetchFromGitHub,
  stdenv,
  lib,
  cmake,
  libxml2,
  fetchurl,

  # Options
  cimVersion ? "CGMES_2.4.15_16FEB2016",
}:
let
  libxml2_2914 =
    (libxml2.overrideAttrs (
      finalAttrs: prevAttrs: {
        version = "2.9.14";
        src = fetchurl {
          url = "mirror://gnome/sources/libxml2/${lib.versions.majorMinor finalAttrs.version}/libxml2-${finalAttrs.version}.tar.xz";
          hash = "sha256-YNdKJX0czsBHXnScui8hVZ5IE577pv8oIkNXx8eY3+4";
        };
        patches = [ ];
      }
    )).override
      { pythonSupport = false; };
in
stdenv.mkDerivation {
  name = "libcimpp";
  src = fetchFromGitHub {
    owner = "sogno-platform";
    repo = "libcimpp";
    # rev = "release/v2.2.0";
    rev = "1b11d5c17bedf0ae042628b42ecb4e49df70b2f6";
    hash = "sha256-RBcV7HlgrKML03E/J9IGIkbKAK23KAXuFJOSXTFZ/i4=";
    fetchSubmodules = true;
  };

  nativeBuildInputs = [ cmake ];

  cmakeFlags = [
    "-DUSE_CIM_VERSION=${cimVersion}"
    "-DBUILD_SHARED_LIBS=ON"
  ];

  enableParallelBuilding = true;

  buildInputs = [ libxml2_2914 ];
}
