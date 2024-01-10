import simulator

def main_loop():
    # Call simulator function
    simulator.set_own_velo(np.array([1, 0, 0]))
    simulator.operate()
