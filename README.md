Air Taxi Tilt-Rotor Design
This repository contains the preliminary aerodynamic and structural design of a conceptual tilt-rotor Air Taxi, aimed at enabling efficient vertical takeoff, transition, and forward flight. The design workflow integrates aerodynamic optimization, structural analysis, and CFD validation using various open-source and cloud-based engineering tools.

📌 Overview
The project focuses on the end-to-end design of a tilt-rotor aircraft for urban air mobility (UAM) applications. It involves:

Preliminary CAD modeling using OpenVSP

Airfoil optimization via XFLR5

Rotor modeling using Actuator Disk Theory

Drag analysis using the Unsteady Vortex Lattice Method (UVLM)

CFD-based aerodynamic validation in SimScale

Structural analysis (shear force and bending moments) in MATLAB

🧰 Tools and Technologies

Tool	Purpose
OpenVSP	Geometric modeling of aircraft
XFLR5	Batch airfoil analysis and optimization
MATLAB	Structural load analysis
SimScale	Cloud-based CFD for force/moment estimation
UVLM	Induced and parasite drag estimation
🔍 Key Features
High-Lift Airfoil Optimization: Performed using batch analysis in XFLR5 to select optimal sections for efficient low-speed and cruise performance.

Rotor Modeling: Eight rotors modeled as actuator disks to simulate realistic thrust loading.

Aerodynamic Interaction Analysis: Tandem-wing and wing-fuselage interactions studied in cruise configuration.

Drag Breakdown: UVLM used to estimate induced and parasite drag components.

Structural Integrity Checks: Bending moments and shear force distributions computed in MATLAB to identify high-stress regions.

📂 Project Structure
graphql
Copy
Edit
/AirTaxi-TiltRotor/
│
├── OpenVSP_Models/           # .vsp3 files of the aircraft geometry
├── XFLR5_Analysis/           # Airfoil performance and polars
├── MATLAB_Structural/        # Load calculation scripts and plots
├── SimScale_Exports/         # Aerodynamic CFD reports and meshes
├── UVLM_Calculations/        # Drag estimation scripts
└── README.md                 # Project overview
📊 Results
Identified optimized airfoil for high lift and low drag.

Validated aerodynamic interactions in cruise.

Highlighted structural stress concentrations for design refinement.

🚀 Future Work
Transition flight analysis (hover ↔ cruise)

Multibody dynamics integration

Control surface optimization

Weight estimation and sizing for payload capacity
