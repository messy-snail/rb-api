# rb-api-python
## Project description  
rb-api-python is an open-source library that supports rapid development of software that deals with cobot control.   

## Supported Cobot
* RB5-850
* RB3-1200
* RB10-1300


## Supported platforms
The package has been tested on:

* Ubuntu 18.04 and 20.04
* Windows 10 64-bit

With Python versions:

* 3.7, 3.8

## Examples 
```
from rainbow import cobot

# This IP address can be found in the control box.
cobot.ToCB('10.0.2.1') 

# Initialize (activate) the cobot.
cobot.CobotInit()

# In real mode, you need to be careful as the cobot is actually moving.
cobot.SetProgramMode(cobot.PG_MODE.REAL)


```
