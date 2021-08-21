<p align="center">
<img src="./doc/figures/pyri_logo_web.svg" height="200"/>
</p>

# PyRI Open Source Teach Pendant Tesseract Planner Package

This package is part of the PyRI project. See https://github.com/pyri-project/pyri-core#documentation for documentation. This package is **not** included in the `pyri-robotics-superpack` Conda package. It can be installed using the following command:

```
conda install -n pyri -c conda-forge -c robotraconteur -c pyri-project -c tesseract-robotics pyri-tesseract-planner
```

The `pyri-tesseract-planner` package contains plugins and services to utilize the Tesseract robot motion planner. See https://github.com/ros-industrial-consortium/tesseract for more information on Tesseract.

## Blocks and Sandbox Functions

This package contains Blockly blocks and PyRI sandbox functions to plan motions using the Tesseract planner. See [tesseract_blocks_functions.md](doc/tesseract_blocks_functions.md) for more information.

## Service

The `pyri-tesseract-planner-service` runs the Tesseract planner.

This service is **not** started automatically by `pyri-core`. It needs to be started by the user or by an outside launcher. It requires URDF and SRDF files to be specified at launch, along with information to locate additional resources.

Standalone service command line example:

```
pyri-tesseract-planner-service
```

Command line options:

| Option | Type | Required | Description |
| ---    | ---  | ---      | ---         |
| `--urdf-file=` | File | Yes | The URDF file to load for the planning scene |
| `--srdf-file=` | File | Yes | The SRDF file to load for the planning scene |
| `--viewer-http-port=` | uint16 | No | The HTTP port to listen for Tesseract viewer connections. Defaults to 8001 | 
| `--device-info-file=` | File | No | Robot Raconteur `DeviceInfo` YAML file. Defaults to contents of `pyri_tesseract_planner_service_default_info.yml` |

This service may use any standard `--robotraconteur-*` service node options.

## URDF and SRDF format

Tesseract requires a URDF and a SRDF file to load the planning scene. These file formats are common to ROS, and documentation can be found in the ROS and MoveIt! wiki pages.

* http://wiki.ros.org/urdf/XML/model
* http://wiki.ros.org/srdf

The planning scene must match precisely the real world workspace. Otherwise, the planner will not be able to accurately avoid collisions. This package does not currently have the ability to modify the planning scene in real time. This functionality will be added in the future.

The SRDF file identifies robots as "move groups". Tesseract will attempt to plan for a single move group at a time. Move group names must match exactly the "local name" of the robot to which they correspond. They must be defined as "chain" groups in the SRDF, defined by the base and tip link.

## Acknowledgment

This work was supported in part by Subaward No. ARM-TEC-19-01-F-24 from the Advanced Robotics for Manufacturing ("ARM") Institute under Agreement Number W911NF-17-3-0004 sponsored by the Office of the Secretary of Defense. ARM Project Management was provided by Christopher Adams. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of either ARM or the Office of the Secretary of Defense of the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes, notwithstanding any copyright notation herein.

This work was supported in part by the New York State Empire State Development Division of Science, Technology and Innovation (NYSTAR) under contract C160142. 

![](doc/figures/arm_logo.jpg) ![](doc/figures/nys_logo.jpg)

PyRI is developed by Rensselaer Polytechnic Institute, Wason Technology, LLC, and contributors.