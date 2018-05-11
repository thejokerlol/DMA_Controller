@echo off
set xv_path=C:\\Xilinx\\Vivado\\2015.2\\bin
call %xv_path%/xsim DMA_Controller_behav -key {Behavioral:sim_1:Functional:DMA_Controller} -tclbatch DMA_Controller.tcl -view C:/Users/vamsi/Desktop/DMA_Controller_Project/DMA_Controller/Instruction_cache_tb_behav.wcfg -view C:/Users/vamsi/Desktop/DMA_Controller_Project/DMA_Controller/Instruction_Buffer_tb_behav.wcfg -log simulate.log
if "%errorlevel%"=="0" goto SUCCESS
if "%errorlevel%"=="1" goto END
:END
exit 1
:SUCCESS
exit 0
