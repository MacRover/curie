@echo off

echo Creating venv environment...
python -m venv venv
call venv/Scripts/activate.bat

echo Updating pip...
python -m pip install --upgrade pip --quiet

echo Installing matplotlib...
python -m pip install matplotlib --quiet

echo Installing keyboard...
python -m pip install keyboard --quiet

echo Installing pySerial
python -m pip install pyserial --quiet

echo Launching application...
python science_testbench.py
