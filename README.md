# Basic Modules Utilities

The purpose of this repository is to provide a standard format for the development skills (of the Markovito service robot) which we refer to as **Basic Modules**.
To encapsulate a collection of related skills into a **Basic Module** offers the following benefits:
- Eases the integration of multiple skills (that are not related to each other) in a single system
- Allows developing different sets of skills simultaneously
- Basic Modules can be used in a plug-and-play fashion, ready to be invoked by the decision making system (e.g., policy, state machine, teleoperator)
- By developing skills with the same interface, skill documentation is easier to read

## Documentation and template files
Template files can be found in the ```src``` and ```include/basicmodutil_pkg``` directories, which can be used to develop a **Basic Module**, whether it is in Python or C++.
On the other hand, the ```utils``` directory contains the following files:
- ```Basic_Module_Handbook.pdf```: A pdf file conatining the documentation for the usage of the template files of this repository. The document details the motivation behind encapsulating skills into **Basic Modules**, as well as several examples.
- ```writeBM.py```: A script that eases the generation of json files, which are necessary fot the **Basic Modules** documentation.
- ```DEMO_BM.json``` y ```PYTHON_BM.json```: Examples of json documentation files.

