# BSN Library

A body sensor network (BSN) library that enables the implementation of sensors, data fusers and emergency detection. Refer to https://arxiv.org/pdf/1804.00994.pdf and https://arxiv.org/pdf/1905.02228.pdf for more information. 

Dependency:
https://github.com/rdinizcal/lepton

The compilation step-by-step is taken as follows: 

1. Clone the repository cointaining Lepton and its dependencies

```
git clone https://github.com/rdinizcal/libbsn
``` 

2. Then, create and enter build folder
```
cd libbsn
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

## Authors

* **Ricardo D. Caldas** - https://github.com/rdinizcal
* **Gabriel Levi** - https://github.com/gabrielevi10
* **Léo Moraes** - https://github.com/leooleo 
* **Samuel Couto** - https://github.com/SCouto97 

Adviser: **Dr. Genaína Nunes Rodrigues** - https://cic.unb.br/~genaina/
