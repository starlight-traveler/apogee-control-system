#constants.py
#Author: Alex Kult
#Description: Data on the universal constants and unit conversion constants
#Date: 6-9-2025
#Copyright Alpha Kappa Sigma

# --- Constants ---
# Universal Constants
g = 9.8067 #Acceleration due to gravity [m/s^2]
gamma = 1.4 #Ratio of specific heats for air
R = 287.05 #Specific gas constant for air [J/(kg·K)]

# Conversions
m2ft = 3.28083989501 #meters to feet
mph2ms = 0.44704 #mph to m/s

def F2K(fahrenheit): #fahrenheit to kelvin
    kelvin = (fahrenheit - 32)/1.8 + 273.15
    return kelvin