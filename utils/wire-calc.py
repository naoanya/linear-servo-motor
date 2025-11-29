import math

######################################################

# Coil shaft inner dimensions (mm)
inner_x = 3.0
inner_y = 10.0
# Coil Thickness (mm)
coil_thickness = 5
# Wire diameter (mm)
wire_diameter = 0.4
# Wire Length (m)
wire_length_m = 17

######################################################

# Coil outer dimensions (mm)
outer_x = inner_x + (coil_thickness * 2)
outer_y = inner_y + (coil_thickness * 2)

# Convert line length to mm
wire_length = wire_length_m * 1000

# Average circumference of outer and inner circumferences
inner_perimeter = 2 * (inner_x + inner_y)
outer_perimeter = 2 * (outer_x + outer_y)
avg_turn_length = (inner_perimeter + outer_perimeter) / 2

# Number of turns per layer
turns_per_layer = coil_thickness / wire_diameter

# Wire length per layer
length_per_layer = turns_per_layer * avg_turn_length

# Calculate the number of layers
layers = wire_length / length_per_layer
outer_z = layers * wire_diameter

total_turns = layers * turns_per_layer

######################################################

# Copper Resistivity (ohm·m at 20 °C)
resistivity = 1.68e-8

# Calculate the resistance of the wire
radius_m = (wire_diameter / 1000) / 2
area_m2 = math.pi * radius_m * radius_m # cross-sectional area [m²]
resistance = resistivity * (wire_length_m / area_m2)

ampair_at_5v = 5 / resistance

######################################################

print("inner_x = ", inner_x)
print("inner_y = ", inner_y)
print("coil_thickness = ", coil_thickness)
print("wire_diameter = ", wire_diameter)
print("wire_length_m = ", wire_length_m)
print("wire_length = ", wire_length)

print("outer_x = ", outer_x)
print("outer_y = ", outer_y)

print("inner_perimeter = ", inner_perimeter)
print("outer_perimeter = ", outer_perimeter)
print("avg_turn_length = ", avg_turn_length)
print("turns_per_layer = ", turns_per_layer)
print("length_per_layer = ", length_per_layer)
print("layers = ", layers)
print("total_turns = ", total_turns)
print("outer_z = ", outer_z)
print("radius_m = ", radius_m)
print("area_m2 = ", area_m2)
print("resistance = ", resistance)
print("ampair_at_5v = ", ampair_at_5v)
