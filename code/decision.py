import numpy as np
import time


def decision_step(Rover):

    # Vision data check
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check navigable terrain
            if Rover.vel < 0.2 and Rover.throttle != 0:
                # If velocity is still 0 with throttle rover is stuck
                if time.time() - Rover.stuck_time > Rover.max_stuck:
                    # Set stuck mode
                    Rover.mode = 'stuck'
                    return Rover
            else:
                # Reset stuck timer
                Rover.stuck_time = time.time()
            if Rover.sample_seen:
                if Rover.picking_up != 0:
                    print('Sample picked up')
                    Rover.sample_seen = False
                    Rover.sample_timer = time.time()
                    return Rover
                if time.time() - Rover.sample_timer > Rover.sample_max_search:
                    print('Cannot find sample')
                    Rover.sample_seen = False
                    Rover.sample_timer = time.time()
                    return Rover
                avg_rock_angle = np.mean(Rover.rock_angle * 180/np.pi)
                if -15 < avg_rock_angle < 15:
                    if max(Rover.rock_dist) < 20:
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = avg_rock_angle
                    else:
                        Rover.throttle = Rover.throttle_set
                        Rover.steer = avg_rock_angle
                elif -50 < avg_rock_angle < 50:
                    print('Turning toward sample')
                    if Rover.vel > 0 and max(Rover.rock_dist) < 50:
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                    else:
                        Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = avg_rock_angle/6
                else:
                    print('Lost sample on camera')
                    Rover.sample_seen = False 
            elif len(Rover.nav_angles) > 50:  
                if Rover.vel < Rover.max_vel:
                    Rover.throttle = Rover.throttle_set
                else: 
                    Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi),
                                              -15, 15)
            else:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'

        elif Rover.mode == 'stop':
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            elif Rover.vel <= 0.2:
                if len(Rover.nav_angles) < 100:
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = -15
                else:
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi),
                                          -15, 15)
                    Rover.mode = 'forward'
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # Rover is stuck 
    if Rover.mode == 'stuck':
        print('Stuck')
        if time.time() - Rover.stuck_time > (Rover.max_stuck + 1):
            Rover.mode = 'forward'
            Rover.stuck_time = time.time()
        else:
            # Turn to get unstuck
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = -15
        return Rover

    # Send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.sample_seen = False
    return Rover
