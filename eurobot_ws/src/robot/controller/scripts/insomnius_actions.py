from msgs.msg import JointActionPoint

def handle_action(order, action):
    """
    Returns a JointActionPoint with the corresponding action.

    Args:
        order (Integer): Order number.
        action (Integer): Order action number.

    Returns:
        JointActionPoint: The corresponding action data.
    """

    ''' --- POSSIBLE VALUES ---
        *
        * The servos_names / motors_names array will be empty if no servo/motor has to move
        *
        * - position:
        *   - 0: closed position
        *   - 1: open position
        *  
        * - velocity:
        *   - >0: forward
        *   - <0: backward
        *   
        '''

    action_msg = JointActionPoint()

    if order == 1:
        ...
    
    elif order == 2:

        if action == 1:
            action_msg.servos_names = ["S01", "S02", "S03", "S04", "S05"]
            action_msg.motors_names = ["M01", "M02"]

            action_msg.position = [0, 0, 0, 1, 1]
            action_msg.velocity = [150.0, -10.0]
            action_msg.activate = 0
        
        elif action == 2:
            action_msg.servos_names = ["S01"]

            action_msg.position = [1]
            action_msg.activate = 0

    elif order == 3:

        if action == 1:
            action_msg.motors_names = ["M02"]

            action_msg.velocity = [-10.0]
            action_msg.activate = 0

        elif action == 2:
            action_msg.servos_names = ["S01"]

            action_msg.position = [0]
            action_msg.activate = 1

        elif action == 3:
            action_msg.motors_names  = ["M02"]

            action_msg.velocity = [150.0]
            action_msg.activate = 1

    elif order == 4:
        
        if action == 1:
            action_msg.motors_names = ["M02"]

            action_msg.velocity = [-10.0]
            action_msg.activate = 1

        elif action == 2:
            action_msg.motors_names = ["M01"]

            action_msg.velocity = [-150.0]
            action_msg.activate = 1

        elif action == 3:
            action_msg.motors_names = ["M02"]

            action_msg.velocity = [150.0]
            action_msg.activate = 1

        elif action == 4:
            action_msg.motors_names = ["M01"]

            action_msg.velocity = [150.0]
            action_msg.activate = 1

        elif action == 5:
            action_msg.motors_names = ["M02"]

            action_msg.velocity = [-10.0]
            action_msg.activate = 1

        elif action == 6:
            action_msg.servos_names = ["S01"]

            action_msg.position = [1]
            action_msg.activate = 0

    return action_msg