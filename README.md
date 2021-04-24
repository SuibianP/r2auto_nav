# EG2310 Group 4
![](https://img.shields.io/badge/EG2310-survived-brightgreen) ![](https://img.shields.io/badge/navigation-wrecked-red) ![](https://img.shields.io/badge/projection-perfect-green)

This repository contains the software source code for EG2310 project, the goal of which is to navigate through a maze and project balls towards an IR target.

## Usage
Please refer to the [project documentation](https://github.com/SuibianP/r2auto_nav/blob/project/Hardware%20Design%20Document%20v2.pdf) for detailed instructions on how to run the code.


## Structure and description
| File | Functionality |
| --- | --- |
| `nav.py` | Main code for navigation |
| `timeout.py` | Timeout function wrapper |
| `dj.py` | Implementation of Dijkstra's algorithm |
| `test.sh` | Testbench for the code in Gazebo |

## Miscellaneous
Approaches for reducing onboard power consumption and any further erratas can be found in the [repository wiki](https://github.com/SuibianP/r2auto_nav/wiki/Power-Saving)

The initial design presentation for software blocks can be found [here](https://suibianp.github.io/r2auto_nav/software.html)

## Team
Hu Jialun

Jareth Tan Eu Quan

Justin Foo Guang En

Liew Yung Jun

Lim Rou Yi
