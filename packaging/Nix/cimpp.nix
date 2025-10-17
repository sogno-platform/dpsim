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
    rev = "051ee4c311572fe92b30120b897d22deb253e162";
    hash = "sha256-jyeiySIOPceH1/ZHRYsp1LczTlXuIapkMtCqlq2lZM0=";
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
