from swarm_segregation.sct import SCT
import os
from ament_index_python.packages import get_package_share_directory

class SCTWrapper:
    def __init__(self):

        yaml_path = os.path.join(
            get_package_share_directory('swarm_basics'),
            'config',
            'supervisor.yaml')
        self.sct = SCT(yaml_path)

        for event_name, idx in self.sct.EV.items():
            self.sct.add_callback(
                event=idx,
                clbk=lambda data: None,  
                ci=lambda data: True,    
                sup_data=None
            )

        self.sct.input_buffer = []
    
    def add_event(self, ev_name):
        if ev_name in self.sct.EV:
            self.sct.input_buffer.append(self.sct.EV[ev_name])

    def run(self):
        self.sct.run_step()
        return self.sct.get_enabled_events() 

          