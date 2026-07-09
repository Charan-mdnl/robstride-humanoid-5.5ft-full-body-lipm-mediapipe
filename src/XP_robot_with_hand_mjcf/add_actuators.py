import xml.etree.ElementTree as ET

# -------- SETTINGS --------
XML_FILE = "XP_robot.xml"
OUTPUT_FILE = "XP_robot_with_actuators.xml"
# --------------------------

def suggest_gear(joint_name):
    name = joint_name.lower()

    if "hip" in name:
        return 200
    elif "thigh" in name or "knee" in name:
        return 180
    elif "shank" in name:
        return 120
    elif "foot" in name or "ankle" in name or "uj" in name:
        return 80
    else:
        return 100  # fallback


tree = ET.parse(XML_FILE)
root = tree.getroot()

print("\n--- Actuator Generator ---\n")

# Remove existing actuator block if exists
for actuator in root.findall("actuator"):
    root.remove(actuator)

# Create new actuator block
actuator_block = ET.Element("actuator")

for joint in root.iter("joint"):
    name = joint.get("name")
    jtype = joint.get("type")

    if jtype != "hinge":
        continue

    print(f"\nJoint: {name}")

    suggested = suggest_gear(name)
    print(f"  Suggested gear: {suggested}")

    user_input = input("  Enter gear (press Enter to accept): ")

    if user_input.strip() == "":
        gear = suggested
    else:
        try:
            gear = float(user_input)
        except:
            print("  Invalid input, using suggested value.")
            gear = suggested

    motor = ET.Element("motor")
    motor.set("joint", name)
    motor.set("ctrlrange", "-1 1")
    motor.set("gear", str(gear))

    actuator_block.append(motor)

    print(f"  ✔ Added actuator with gear {gear}")

# Append actuator block to root
root.append(actuator_block)

# Save file
tree.write(OUTPUT_FILE)

print(f"\n✅ Actuator XML saved as: {OUTPUT_FILE}")