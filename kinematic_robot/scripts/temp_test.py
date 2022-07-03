class Flow:
    def __init__(self):

        self.all_states = [ 's0_start',                 # Exit when PM is ready -> useful if next states should be skipped
                            's1_wait_for_cv',           # Exit when CV nodes ready -> Ready flags set
                            's2_camera_calibration',    # Exit when CC finished -> camera calibrated, result file on disk
                            's3_handeye_calibration',   # Exit when HC finished -> handeye calibrated, result file on disk
                            's4_model_registration',    # Exit when MR finished -> target acquired, target pose on disk
                            's5_pre_insertion',         # Exit when robot at pre insertion pose
                            's6_insertion',             # Exit when robot at insertion pose
                            's7_post_insertion',        # Exit when robot at post insertion pose (reverse to pre insertion pose)
                            's8_end',                   # Never exit unless user command
                        ]

        self.curr_state = 's0_start'
    

    def go_to_next_state(self):
        ''' Reads curr_state and sets next_state according to all_states 
            Then waits for user input to confirm next state or overrule
        '''
        
        # Check if state is in all_states
        curr_state  = self.curr_state
        assert curr_state in self.all_states, 'curr_state not found in all_states'

        state_idx   = self.all_states.index(curr_state)

        # Set next state. Set to none, if last state has been reached
        if state_idx == len(self.all_states) - 1:
            print("[PM] Reached end state, no next state specified")
            next_state = None
        
        else:
            next_state  = self.all_states[state_idx + 1]

        print(f"[PM] curr_state = {self.curr_state} --> next_state = {next_state}")

        # Wait for user confirmation or user specified state
        
        # Assign curr_state to next_state and continue PM
        self.curr_state = next_state


    def main_process(self):

        self.curr_state = 's0_start'
        
        while True:
            self.go_to_next_state()    

            a = 1





if __name__ == '__main__':

    flow = Flow()
    flow.main_process()

    all_states = [ 's0_start',                 # Exit when PM is ready -> useful if next states should be skipped
                's1_wait_for_cv',           # Exit when CV nodes ready -> Ready flags set
                's2_camera_calibration',    # Exit when CC finished -> camera calibrated, result file on disk
                's3_handeye_calibration',   # Exit when HC finished -> handeye calibrated, result file on disk
                's4_model_registration',    # Exit when MR finished -> target acquired, target pose on disk
                's5_pre_insertion',         # Exit when robot at pre insertion pose
                's6_insertion',             # Exit when robot at insertion pose
                's7_post_insertion',        # Exit when robot at post insertion pose (reverse to pre insertion pose)
                's8_end',                   # Never exit
                ]

'''


'''