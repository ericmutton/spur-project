# spur-project

Musings in preparation for the 2025 [Summer Platform for Undergraduate Research](https://spur.utdallas.edu/).

# Getting Started

This repository contains the project files created during my Fall 2024 and Spring 2025 semesters.

## Hardware

The EDA schematic captures and prototype PC boards are maintained in separate KiCAD projects in the [Hardware](/Hardware) directory.
- Single-Cell PMIC and Switch-Mode PSU testbench
- NiceRF SA868 walkie-talkie module breakout board 

## Software

The source code utilizing either the Espressif Systems [ESP-IDF Arduino Core](https://github.com/espressif/arduino-esp32) (using FreeRTOS) for experimentation
or the Zephyr Standalone Application (using ZephyrRTOS) constituting the base radio platform firmware are both maintained in the [Software](/Software) directory.

# Scope

The scope of this research project is to build a two-way radio communications platform that remains extensible in arbitrary 'all-mode' FM modulation in both firmware and hardware.

This means adopting modern real-time operating system ecosystems, particularly the two RTOS under consideration are:
- [Zephyr Project](https://zephyrproject.org) [ZephyrRTOS](https://github.com/zephyrproject-rtos) (a Linux Foundation Project)
- [FreeRTOS](https://freertos.org/) (under Amazon AWS stewardship as of 2017)

For the hardware, the platform is to allow off-the-shelf (and few bespoke) integration of:
 - Wireless communications modules
   - [ ] GNSS/GPS receivers 
   - [ ] Sub-Ghz transceivers
   - [ ] LoRa transceivers
 - Peripheral Human-Machine Interface Devices (HIDs)
   - [ ] Potentiometers (Volume, Frequency), Keypad (DTMF, Navigation), Display
   - [ ] Mono CW Keyer, Keyboard, and USB devices
   - [ ] External audio source/sink, external modem/vocoder

## Groundwork

- [x] Develop a working 'two-way' radio prototype. 
  - [x] Phase out SA868 Demo Board by building a minimal breakout board
  - [x] Make prototype mobile through a rechargable cell and JEITA charger
  - [x] Amplify audio output of SA868 with anything other than an LM386 reference design
  - [x] Allow end-user to enter new RX and TX frequencies
  - [ ] All settings available to the end-user
  - [ ] Persistence of settings between power cycles

## Phase 1

For the purposes here, the SA868 is limited to arbitrary FM modes:
 - Analog FM Voice 
 - Amateur Packet Reporting System (APRS) 
 - Premodulated FM Waveforms
In Analog FM Voice, a parrot test or signal strength report (Readability-Strength-Tone) would be 
## Phase 2
After verifying what can be done with the SA868, the next transceiver is the Texas Instruments CC1200. 

## Approach

A first-principles approach in designing this platform is to be taken on:
1. inter-device communications interfacing
   - [x] sensor device register-level configuration
   - [x] factory firmware flashing (see DMO module)
   - [x] microcontroller (a)synchronous peripheral utilization
2. utilizing vendor hardware abstractions
   - [x] device abstraction and tooling
     - [TI SimpleLink SDK](https://www.ti.com/tool/SIMPLELINK-LOWPOWER-SDK) >= v3.3
     - [SmartRF Studio](https://www.ti.com/tool/SMARTRFTM-STUDIO) == v7 (see TI CC1200)
   - [ ] subsystem abstraction
3. utilizing operating system abstractions
   - [ ] target hardware-application decoupling (see ZephyrRTOS)
   - [ ] communications stack normalization (see USB, IEEE 802.15.4, Bluetooth SIG)
   - [ ] tracing & debugging meta-tooling (see [openocd](https://openocd.org/))
4.  extensibility
