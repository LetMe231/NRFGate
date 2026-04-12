@echo off
:: Windows wrapper for graphify.
:: Add the tools\ directory to PATH, or call this script directly:
::   tools\graphify.bat <command> [options]
python "%~dp0graphify.py" %*
