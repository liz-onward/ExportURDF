# OnShape URDF Exporter
OnShapeURDF.py is a script to export URDF files from OnShape assemblies

## Requirements
- Git
- Python3

## Setup
1. Run `git clone git@github.com:david-dorf/ExportURDF.git` in your terminal
2. If you don't already have one, create a new OnShape API key [here](https://cad.onshape.com/appstore/dev-portal/apiKeys)
3. Add the key to your shell config. If you are using bash, add this to your ~/.bashrc file:
```bash
export ONSHAPE_API=https://cad.onshape.com
export ONSHAPE_ACCESS_KEY=<INSERT YOUR ACCESS KEY HERE>
export ONSHAPE_SECRET_KEY=<INSERT YOUR SECRET KEY HERE>
```
4. Re-open your terminal to apply the changes 
5. Enter this directory in your terminal and run `python3 OnShapeURDF.py`
6. OnShapeURDF.py will ask you to input your design's URL and a name for your robot design, then output a folder with that name containing the URDF and meshes from your design

## Known Issues
1. Subassemblies are not yet supported
2. Parts MUST be located in Part Studios your assembly's OnShape document. Otherwise, those parts cannot be exported manually or automatically by this script.
