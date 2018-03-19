@echo off
set xv_path=C:\\Xilinx\\Vivado\\2015.2\\bin
call %xv_path%/xsim Instruction_cache_tb_behav -key {Behavioral:sim_1:Functional:Instruction_cache_tb} -tclbatch Instruction_cache_tb.tcl -view C:/Users/vamsi/Desktop/DMA_Controller_Project/DMA_Controller/Instruction_cache_tb_behav.wcfg -log simulate.log
if "%errorlevel%"=="0" goto SUCCESS
if "%errorlevel%"=="1" goto END
:END
exit 1
:SUCCESS
exit 0
