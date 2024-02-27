import rospy
import smach
import smach_ros

class StateMachine:
    def __init__(self):
        self.outcomes_s_1 = ['s3_ext_mech']
        self.outcomes_s_2 = ['s1_cleaning']

        self.transitions = {
            'state_1': {'s3_ext_mech': 'state_2'},
            'state_2': {'s1_cleaning': 'state_1'}
        }

        class State1(smach.State):
            def __init__(self):
                smach.State.__init__(self, outcomes=self.outcomes_s_1, input_keys=['state_1_input'])

            def execute(self, userdata):
                rospy.loginfo('Executing state 1')
                if userdata.state_1_input == 1:
                    return self.outcomes_s_1[0]

        class State2(smach.State):
            def __init__(self):
                smach.State.__init__(self, outcomes=self.outcomes_s_2, input_keys=['state_2_input'])

            def execute(self, userdata):
                rospy.loginfo('Executing state 2')
                if userdata.state_2_input == 1:
                    return self.outcomes_s_2[0]

        self.state_1 = State1
        self.state_2 = State2

    def main(self):
        rospy.init_node('xarm_state_machine')

        # Create the top level SMACH state machine
        state_A = smach.StateMachine(outcomes=['stateB'])
        state_A.userdata.flag_A = 1    # Setting Flag = 1 for State A
        
        # Open the container
        with state_A:

            # Add States to State A
            smach.StateMachine.add('state_1', self.state_1(),
                                    transitions=self.transitions['state_1'],
                                    remapping={'state_1_input': 'flag_A'})
            
            smach.StateMachine.add('state_2', self.state_2(),
                                    transitions=self.transitions['state_2'],
                                    remapping={'state_2_input': 'flag_A'})

        # Execute SMACH plan
        outcome = state_A.execute()

if __name__ == '__main__':
    sm = StateMachine()
    sm.main()
