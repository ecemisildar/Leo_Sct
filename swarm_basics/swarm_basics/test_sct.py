from sct import SCT

# -------------------------------
# 1️⃣ Initialize SCT
# -------------------------------
sct = SCT("supervisor.yaml")

# Dummy callbacks just for simulation
def dummy_callback(data):
    pass

for ev_index in sct.EV.values():
    sct.add_callback(ev_index, dummy_callback, lambda d: False, None)

# Reset supervisors to initial states
sct.sup_current_state = sct.sup_init_state.copy()

# Map FBD names for readability
fbd_names = ["ForwardFBD", "CWFBD", "CCWFBD", "ObstacleFBD"]

# -------------------------------
# 2️⃣ Dynamic Unit Test Function
# -------------------------------
def test_sct_supervisors(sct):
    print("\n=== Running Dynamic SCT Unit Tests ===")
    for fbd_index, fbd_name in enumerate(fbd_names):
        print(f"\nTesting {fbd_name} (initial state = {sct.sup_current_state[fbd_index]})")

        for event_name, ev_index in sct.EV.items():
            prev_state = sct.sup_current_state[fbd_index]
            sct.make_transition(ev_index)
            next_state = sct.sup_current_state[fbd_index]

            if next_state != prev_state:
                print(f"  Event '{event_name}': {prev_state} -> {next_state}")
            else:
                print(f"  Event '{event_name}': {prev_state} (no change)")

# -------------------------------
# 3️⃣ Optional: Rule Checks
# -------------------------------
def check_forward_obstacle_rule(sct):
    fwd_index = 0  # ForwardFBD
    fwd_state_before = sct.sup_current_state[fwd_index]

    # Simulate obstacle
    ev_obstacle = sct.EV["EV_obstacleFront"]
    sct.make_transition(ev_obstacle)
    fwd_state_after = sct.sup_current_state[fwd_index]

    if fwd_state_after == fwd_state_before:
        print(f"✅ Forward correctly blocked by obstacle: state stayed {fwd_state_after}")
    else:
        print(f"❌ Forward moved despite obstacle: {fwd_state_after}")

# -------------------------------
# 4️⃣ Run Tests
# -------------------------------
if __name__ == "__main__":
    test_sct_supervisors(sct)
    check_forward_obstacle_rule(sct)
