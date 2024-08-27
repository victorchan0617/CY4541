@REM This script allows a 3rd party IDE to use CyElfTool to perform
@REM the pre processing that is necessary to extract the bootloader
@REM data from the *.elf file for use when building the bootloadable
@REM application.
@REM NOTE: This script is auto generated. Do not modify.

@REM This is the elf path used by creator, elf path might change depending on IDE.
@REM NOTE: this variable may need to be modified if bootloader project is exported.
set ELF_FILE_PATH="..\Bootloader\CYPD4225-40LQXI_i2c_boot_0_0_0_58_0_0_0_nb.elf"
@IF %errorlevel% NEQ 0 EXIT /b %errorlevel% 

chdir /d ..\.\Export
@IF %errorlevel% NEQ 0 EXIT /b %errorlevel% 
CyElfTool.exe -E %ELF_FILE_PATH% --flash_row_size 256 --flash_size 131072 --flash_array_size 131072 
@IF %errorlevel% NEQ 0 EXIT /b %errorlevel% 
move /y cybootloader.c ..\Generated_Source\PSoC4\cybootloader.c
@IF %errorlevel% NEQ 0 EXIT /b %errorlevel% 
move /y cybootloader.icf ..\Generated_Source\PSoC4\cybootloader.icf
@IF %errorlevel% NEQ 0 EXIT /b %errorlevel% 
