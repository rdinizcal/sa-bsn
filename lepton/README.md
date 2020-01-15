# Lepton Library

Lepton ("lightweight expression parser") is a small C++ library for parsing, evaluating, differentiating, and analyzing mathematical expressions. For more information refer to https://simtk.org/projects/lepton.

The source code author is Peter Eastman from Stanford University. Here we only provide CMake scripts in order to make the compilation straightforward.

The compilation step-by-step is taken as follows: 

1. Clone the repository cointaining Lepton and its dependencies

```
git clone https://github.com/rdinizcal/lepton
``` 

2. Then, create and enter build folder
```
cd lepton
``` 
``` 
mkdir build && cd build
``` 

3. Execute cmake from the build folder
``` 
cmake ..
``` 

4. Finally, compile and install lepton library
``` 
sudo make install
``` 
