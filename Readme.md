Usage
=====

install cmake and python invoke (http://www.pyinvoke.org/) ...
change the path in invoke.py and build_settings.cmake note not every path is used till now
then you can use 

```python
inv configure # configure the build using cmake 
inv all # build the project
```

You can find the output in the Debug folder.

```python
inv -L # list the other build targets 
inv clean # clean the project
```
Some more information is coming soon.

It is also possible to test the canpie device file from microcontrol with this project. For this
test add a folder canpie_stm32f4xx_canfd in the source directory. Then change the settings in the
cp_platform.h use the settings from this driver also for the microcontrol driver. Also change the
CMakeLists.txt file and run cmake again.
