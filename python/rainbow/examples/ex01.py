from rainbow import cobot

# This IP address can be found in the control box.
cobot.ToCB('10.0.2.1') 

# Initialize (activate) the cobot.
cobot.CobotInit()

# In real mode, you need to be careful as the cobot is actually moving.
cobot.SetProgramMode(cobot.PG_MODE.REAL)