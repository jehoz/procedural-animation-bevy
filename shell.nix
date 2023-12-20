{ pkgs ? import <nixpkgs> {} }:
with pkgs;
pkgs.mkShell rec {
  nativeBuildInputs = [
    pkg-config
    cargo rustc
  ];
  buildInputs = [
    udev alsa-lib vulkan-loader
    xorg.libX11 xorg.libXcursor xorg.libXi xorg.libXrandr # x11 deps
    libxkbcommon wayland # wayland deps

    rust-analyzer rustfmt clippy
  ];

  LD_LIBRARY_PATH = lib.makeLibraryPath buildInputs;
}
