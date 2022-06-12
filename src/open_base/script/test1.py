
import numpy as np
def find_state (number_of_states,current_or, goal_or):


    states = np.empty(number_of_states)
    for i in range(0,number_of_states):
            state = ((goal_or+(180/number_of_states))+((360/number_of_states)*i))
            if state > 359:
                state = state-360
            states[i] = int(state)
    print(states)


    for i in range (0,number_of_states):
        if states[-1] < states[0]:
            if current_or in list(range(states[-1].astype(int),(states[0].astype(int)))):
                relative_or = 1
                break
        elif states[-1] > states[0]:

            if current_or in list(range(states[-1].astype(int), 360)) or current_or in list(range(0, (states[0].astype(int)))):
                relative_or = 1
                break

        if i+1>number_of_states:
            relative_or = number_of_states
            break
        elif (states[i].astype(int)) > (states[i+1].astype(int)):
            if current_or in list(range(states[i].astype(int),360)) or current_or in list(range(0,(states[i+1].astype(int)))):
                relative_or = i+2
                break
        elif current_or in list(range(states[i].astype(int),(states[i+1].astype(int)))):
            relative_or = i+2
            break
    return relative_or


    # print(relative_or)
        # if i+2> number_of_states+2:
        #     print(states[i.])

    # print(np.max(states))
    # print(np.min(states))
    # print(states)
    # if delta > 0 and delta < 180:

    # elif delta < 0 and abs(delta) <180:
    #     pass

state = find_state(12,224,210)
print(state)
 # assert(find_state(10, 45, )==...)



