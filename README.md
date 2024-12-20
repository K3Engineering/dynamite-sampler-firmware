# dynamite-sampler-firmware

Dynamite sampler board firmware

## Development

Use the "Add .vscode subdirectory files" to generate a json that will allow vscode to
figure out where things are defined.

### Configurations & `sdkconfig`

`esp_idf_project_configuration.json` is used to configure either a debug or release build.

Documentation here: https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/additionalfeatures/multiple-projects.html

for saving the config: `idf.py save-defconfig`

### Debuger / breakpoints

launching openocd `openocd -f board/esp32s3-builtin.cfg `
`-d4` for highests debug level

`usblogview` and `USBDriverTool` were quite helpful
