# Cenntro iChassis Simulator
This repo models a representative iChassis from Cenntro.  Forked from https://github.com/bjyurkovich/vehicle-model-python

# Running
Clone the repo:
```bash
git clone https://github.com/bjyurkovich/ichassis_simulator
```
Create your virtual environment:
```bash
python3 -m venv venv
```

Activate your virtual env:
```bash
source venv/bin/activate
```

Load the dependencies:
```bash
pip install -r requirements.txt
```

### Run the example
Run the example:
```bash
python example.py
```

Remember - stuff is in scientific units (`m/s`), so you will need to do a few conversions if you want to think about it in `kph` or `mph`.

> I leave this to your with no warranty, no promise of help, but it _should_ get you to where you need to go!

## What is in the example
It's a simple step-wise simulator (1 second steps) that takes a commanded vehicle velocity and computes the real (model-based) velocity of the chassis.  There is a driver model, but you will need to make sure you are achieving what you command!

Imagine this is like an API to the CAN interface that your autonomous system and navigation unit will hook into (just a lot easier!)

```
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
```