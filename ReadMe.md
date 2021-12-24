# rb-api

## Resources
Homepage: https://www.rainbow-robotics.com

## Supported OS
* Windows 10 (C++, Python)
* Ubuntu 18.04, 20.04 (Python)

## Supported programming language
Currently, C++ is **only supported on Windows**. Ubuntu will also be supported in the future.

* C++ **(at least c++ 11 version)**
* Python **(at least python 3.7 version)**
* ~~C#~~ (TBD)
* ~~Java~~ (TBD)

## Usage
### C++
https://github.com/messy-snail/rb-api-example

### Python
Install Python modules using PyPI
```
pip install rb-api-python
```

```
from rainbow import cobot

# This IP address can be found in the control box.
cobot.ToCB('10.0.2.1') 

# Initialize (activate) the cobot.
cobot.CobotInit()

# In real mode, you need to be careful as the cobot is actually moving.
cobot.SetProgramMode(cobot.PG_MODE.REAL)
```
