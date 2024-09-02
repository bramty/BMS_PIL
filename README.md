# Processor in the loop implementation for Passive Balancing Application on Master-Slave Battery Management Systems

## Overview
Due to the frequent occurrence of cell imbalance within battery packs, a balancing mechanism is essential to extend battery life and optimize energy utilization. This project implements a processor-in-the-loop setup to verify a passive balancing algorithm within a Battery Management System (BMS) using a Master-Slave topology. Three battery cells are modeled in Simulink and interfaced with Slave BMS Modules through USART communication via a Serial block. The Slave BMS Modules transmit the battery cell data to the Master BMS Module via the CAN bus, where it processes the data and determines which cells require passive balancing. The Slave BMS Modules that receive the command from the master then sends the commands back to the Simulink battery cell model for executing balancing mechanism.

![PIL_BMS](https://github.com/user-attachments/assets/b672d43b-1f95-426b-b400-61b369f47f7b)

## Project Structure
* **POC:** containing codes to test communication between the model and the slave modules and between the slave modules and master module
* **Battery Model:** implementation of the battery cell modeld and their passive balancing circuits
* **BMS Master:** code for BMS Master Module
* **BMS Slave:** code for BMS Slave Module
* **Report:** full explanation of Simulink models and Arduino codes for this project
