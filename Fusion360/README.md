# Fusion360 URDF Converter Script

This repository contains a Fusion360 URDF converter script that converts Fusion360 models into URDF files. The script is written in Python and uses the Fusion360 API to extract the necessary information from the CAD model and generate the URDF files.

Trees (ie. multiple child components of the same parent) are not currently supported. The code also needs to be factored alongside this as it's quite messy currently.

## Getting Started
Before using the script, make sure your Fusion360 design is set up correctly. The script requires the following:
1. Each link should be a component in the Fusion360 design.
2. At least one link should be designated as the base link. The base link should only have a fixed joint connected to it.
3. The joints should be created using the Fusion360 joint tool. The script currently supports the following joint types:
   - Revolute
   - Fixed
   - Slider
   - The above, but with limits and as-built joints
4. When making a joint, the parent link should be selected first, followed by the child link.

In a Fusion360 environment, you can run the script by following these steps:
1. Open the Fusion360 application.
2. Use `Shift + S` to open the Scripts and Add-ins dialog.
3. Click the green `+` icon to open the Fusion360 script folder.
4. Copy the `Fusion360` folder from inside this repository into the Fusion360 script folder.
5. Run the `FusionURDF` script by clicking the `Run` button.

## Roadmap
1. Write unit tests for the script to ensure code quality and maintainability.
2. Test the script with various Fusion360 models to identify and fix any issues. 
   Edge cases like closed chains may not be handled correctly yet.
3. Add additional features such as materials, rigid groups, and more joint types.
4. Add support for subassemblies and nested components.
5. Ensure suppressed and hidden components are not included in the URDF file.

## Dependencies
The script requires the following dependencies:

Python 3.6+

Fusion360 API
