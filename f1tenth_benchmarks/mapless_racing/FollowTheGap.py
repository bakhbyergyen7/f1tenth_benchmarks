import numpy as np
from f1tenth_benchmarks.utils.BasePlanner import BasePlanner

class FollowTheGap(BasePlanner):
    def __init__(self, test_id):
        super().__init__("FollowTheGap", test_id)
        self.name = 'FollowTheGap'

    def plan(self, obs):
        # car_width = 0.31
        car_width = .296
        tolerance = .5
        scan = obs['scan']#[:-1]

        #process where the farthest point is, and then the farthest safe point
        #dead center is 540, each degree has 4 scans -- assume all measurements are in m
        target = self.safe_point(scan)
        unchecked_angle = self.point_to_steering(target)
        print(f'target :  {target}')
        print(f'unchecked_angle :  {unchecked_angle}')
        print(f'120:220 {min(scan[120:220])} ? {car_width+tolerance}')
        print(f'760:840 {min(scan[760:840])} ? {car_width+tolerance}')
        #check the angle to make sure the car is physically able to turn that far
        if -30 <= unchecked_angle <= 30:
            steering_angle = unchecked_angle
        elif -30 > unchecked_angle:
            steering_angle = -30
        else:
            steering_angle = 30
        # #safety check! if car is within .5 meters of the wall, steer away slightly to provent side collision
        if min(scan[120:220]) > car_width+tolerance and min(scan[760:840]) > car_width+tolerance:
            pass
        elif min(scan[120:220]) <= car_width+tolerance and steering_angle < 0:            
            steering_angle = 0.75
        elif min(scan[760:840]) <= car_width+tolerance and steering_angle > 0:
            steering_angle = -0.75
        '''SPEED CALCULATION'''

        if scan[540]/2 >= 8:
            speed = 8
        else:
            speed = scan[540]/2

        
        print(f'steering_angle: {steering_angle}')
        print(f'speed: {speed}')
        

        action = np.array([self.deg2rad(steering_angle), speed])

        return action

#-----------------------------------------
    
    def safe_point(self, scan):
        # car_width = 0.31
        car_width = .296
        tolerance = 0.5 
        laser_arr = scan
        goal_dist = np.max(laser_arr)
        goal_point = np.argmax(laser_arr)
        # print(goal_point)
        Lsafe_point = self.safe_point_to_left(laser_arr, goal_point, car_width/2 + tolerance)
        Rsafe_point = self.safe_point_to_right(laser_arr, goal_point, car_width/2 + tolerance)
        print(f'Lsafe_point: {Lsafe_point}')
        print(f'Rsafe_point: {Rsafe_point}')
        # try:
        #     if (Lsafe_point != 0) and (Rsafe_point != 0) and laser_arr[goal_point + Lsafe_point] >= laser_arr[goal_point+Rsafe_point]:
        #         target = Lsafe_point
        #         self.get_logger().info(f'num scans: {target}')
        #         return goal_point+Lsafe_point
        #     elif Rsafe_point != 0:
        #         target = Rsafe_point
        #         self.get_logger().info(f'num scans: {target}')
        #         return goal_point+Rsafe_point
        #     else:
        #         return 540
        # except:
        #     return 540
        if (Lsafe_point != 0) and (Rsafe_point != 0):
            left_index = goal_point + Lsafe_point
            right_index = goal_point + Rsafe_point

            # Ensure indices are within bounds
            if left_index < len(laser_arr) and right_index < len(laser_arr):
                if laser_arr[left_index] >= laser_arr[right_index]:
                    target = Lsafe_point
                    return left_index
                else:
                    target = Rsafe_point
                    return right_index
            elif left_index < len(laser_arr):
                target = Lsafe_point
                return left_index
            elif right_index < len(laser_arr):
                target = Rsafe_point
                return right_index
        else:
            return 540
        
    def safe_point_to_right(self, ranges, index_of_furthest, width):
        act_distance = 0
        steps_away = 1

        while act_distance < width:
            # Check if index is within bounds
            if index_of_furthest + steps_away >= len(ranges):
                return 0  # Out of bounds, return 0
            
            act_distance += self.dbs(ranges[index_of_furthest + steps_away])
            steps_away += 1

        return steps_away


    def safe_point_to_left(self, ranges, index_of_furthest, width):
        act_distance = 0
        steps_away = -1

        while act_distance < width:
            # Check if index is within bounds
            if index_of_furthest + steps_away < 0:
                return 0  # Out of bounds, return 0
            
            act_distance += self.dbs(ranges[index_of_furthest + steps_away])
            steps_away -= 1

        return steps_away
    
    def deg2rad(self, deg):
        return (deg*np.pi)/180    
    
    def point_to_steering(self, target):
        #self.get_logger().info(f'Target: {target}')
        if target == 540:
            return 0
        else:
            return (target-540)/4


    def dbs(self, dist):
        #calc distance between scans
        return (np.pi*dist)/540
        
def main():
    pass

if __name__ == '__main__':
    main()