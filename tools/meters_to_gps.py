#!/usr/bin/env python

################################
# File: meters_to_gps.py
# Purpose: convert a delta in meters to an approximate delta in latitude/longitude.
#          based on the data from https://www.usna.edu/Users/oceano/pguth/md_help/html/approx_equivalents.htm
# Date Created: 20 Jan 2023
################################

import math

DEGREES_PER_METER = 0.000001/0.11
MINUTES_PER_DEGREE = 60
SECONDS_PER_DEGREE = MINUTES_PER_DEGREE*60

meters = (input("Enter delta in meters: "))
degrees = math.trunc(float(meters)*DEGREES_PER_METER)
minutes = (float(meters)*DEGREES_PER_METER - math.trunc(float(meters)*DEGREES_PER_METER))*MINUTES_PER_DEGREE
print(degrees, ", ", minutes, ", ", degrees+minutes/100)