@echo off
SET arg1=%1
IF "%arg1%"=="" SET arg1=position

call python agx_tutorials\launch\panda_agx_simulation.py agx_tutorial_resources\agx_tutorial_resources_panda_description\urdf\panda.urdf agx_tutorial_resources --command-interface %arg1%