@echo off
REM Run this file to build the project outside of the IDE.
REM WARNING: if using a different machine, copy the .rsp files together with this script.
echo OsmosMain.cpp
"C:\Program Files (x86)\AtmelStudio\7.0\toolchain\avr8\avr-gcc-12.1.0-x64-windows\bin\avr-g++.exe" @"VisualGDB/Release/OsmosMain.gcc.rsp" || exit 1
