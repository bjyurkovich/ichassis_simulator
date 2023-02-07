from IChassis import IChassis

ic = IChassis()

# Sample drive profile in m/s
v_in = [0, 0, 0, 0.2, 0.3, 0.4, 0.5, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# Simulate each 1 second step of the iChassis
# Note: It's a full EV model, so you will have to respect physics!

for v in v_in:
    print(ic.drive(v))
    
    
# Get the full drive history since clearing or reinstantiating
v_out = ic.get_velocity_history()
# print(v_out)

# Clear the drive history (reinstanitating the object will also do it!)
ic.clear_drive_history()