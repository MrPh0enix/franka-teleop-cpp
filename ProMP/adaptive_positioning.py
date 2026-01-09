import numpy as np
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
from Full_ProMP import ProMp
from Full_ProMP import Learner
import Full_ProMP

# Define distance metric for joint trajectories
def euclidean_distance(x, y):
    return np.linalg.norm(x - y)

# Parameters
basis = 8
dof= 7
Nd = 60
trajectoriesList = []
timeList = []

'''     Import Data from demonstrations    '''
for demo in range(1, Nd+1):
    # Load Franka data for the demonstration
    joints_raw, times_raw= Full_ProMP.Franka_data2('NEW_DEMOS/', demo)
    # Reduce data to 100 samples
    indices = np.linspace(0, len(joints_raw)-1, 200, dtype = int)
    joints_raw = np.asarray([joints_raw[i] for i in indices])
    times_raw = np.asarray([times_raw[i] for i in indices])
    # Append data to lists
    trajectoriesList.append(joints_raw)
    timeList.append(times_raw)


# Get the number of data points
n_data = len(joints_raw)
# Create a time array
Time = np.linspace(0, 1, n_data)

    
'''     ProMP    '''
# Create ProMP object
ProMP_ = ProMp(basis, dof, n_data)

# Create a ProMP object for learning
training_model = ProMp(basis, dof, n_data)
# Create a learner object and learn from data
learner = Learner(training_model)
learner.LearningFromData(trajectoriesList, timeList)

# Create a ProMP object for smoothing
ProMP_trained = ProMp(basis, dof, n_data)
ProMP_trained.mu = training_model.mu
ProMP_trained.cov = training_model.cov
# Generate smoothed trajectories
promp_trajectory = ProMP_trained.trajectory_samples(Time, 1)
# Compute mean and covariance of the smoothed trajectories
meanTraj, covTraj = ProMP_trained.trajectory_mean_cov(Time)
# Get mean and standard deviation of the smoothed trajectory
meanTraj, stdTraj = ProMP_trained.trajectory_mean_std(Time)


# condition_pos = [0.8632710863534817, 0.2635637967515765, -0.9615780671185145, -1.937763639918545, 0.24814595905939738, 2.2066593839592374, 0.586055133572883]
# condition_var = [0.17643179, 0.25017093, 0.28268008, 0.04898373, 0.35696326, 0.1773813, 0.18155298]
# ProMP_conditioned = ProMP_trained.jointSpaceConditioning(2, condition_pos, condition_var)
# cond_meanTraj, cond_stdTraj = ProMP_conditioned.trajectory_mean_std(Time)




def DtW(real_time_joint_angles):
    
    corresponding_iterations = []
    
    for measurement in real_time_joint_angles:
        min_distance = float('inf')  # Initialize the minimum distance to a large value
        corresponding_iteration = None
        
        for i, joint_angles in enumerate(meanTraj):
            distance, path = fastdtw(joint_angles, measurement, dist = euclidean_distance)  # Calculate DTW distance
            
            if distance < min_distance:
                min_distance = distance
                corresponding_iteration = i
        
        corresponding_iterations.append(corresponding_iteration)

    
    # Fetch the desired trajectory at corresponding iterations
    desired_joint_positions = meanTraj[corresponding_iterations]
    current_stdDev = stdTraj[corresponding_iteration]
    desired_joint_positions = np.squeeze(desired_joint_positions)
    
    return desired_joint_positions, current_stdDev, corresponding_iteration


# def euclidean_dist_pos(real_time_joint_angles):

#     corresponding_iterations = []
    
#     for measurement in real_time_joint_angles:
#         min_distance = float('inf')  # Initialize the minimum distance to a large value
#         corresponding_iteration = None
        
#         for i, joint_angles in enumerate(meanTraj):
#             distance = np.sqrt(np.sum((np.array(joint_angles) - np.array(measurement)) ** 2))
#             if distance < min_distance:
#                 min_distance = distance
#                 corresponding_iteration = i
        
#         corresponding_iterations.append(corresponding_iteration)

    
#     # Fetch the desired trajectory at corresponding iterations
#     desired_joint_positions = meanTraj[corresponding_iterations]
#     current_stdDev = stdTraj[corresponding_iteration]
#     desired_joint_positions = np.squeeze(desired_joint_positions)
    
#     return desired_joint_positions, current_stdDev, corresponding_iteration


def euclidean_dist_pos(real_time_joint_angles):

    corresponding_iterations = []
    
    min_distance = float('inf')  # Initialize the minimum distance to a large value
    corresponding_iteration = None
    
    for i, joint_angles in enumerate(meanTraj):
        distance = np.sum((np.array(joint_angles) - np.array(real_time_joint_angles)) ** 2)
        if distance < min_distance:
            min_distance = distance
            corresponding_iteration = i
    
    corresponding_iterations.append(corresponding_iteration)

    
    # Fetch the desired trajectory at corresponding iterations
    desired_joint_positions = meanTraj[corresponding_iterations]
    current_stdDev = stdTraj[corresponding_iteration]
    desired_joint_positions = np.squeeze(desired_joint_positions)
    
    return desired_joint_positions, current_stdDev, corresponding_iteration



def euclidean_dist_pos2(real_time_joint_angles):

    corresponding_iterations = []
    
    # for measurement in real_time_joint_angles:
    min_distance = float('inf')  # Initialize the minimum distance to a large value
    corresponding_iteration = None
    joint_weights = np.array([7,6,5,4,3,2,1]) # weights to prioritize the joints
    
    for i, joint_angles in enumerate(meanTraj):
        distance = np.sum(joint_weights * (np.array(joint_angles) - np.array(real_time_joint_angles)) ** 2)
        if distance < min_distance:
            min_distance = distance
            corresponding_iteration = i
    
    corresponding_iterations.append(corresponding_iteration)

    
    # Fetch the desired trajectory at corresponding iterations
    desired_joint_positions = meanTraj[corresponding_iterations]
    current_stdDev = stdTraj[corresponding_iteration]
    desired_joint_positions = np.squeeze(desired_joint_positions)
    
    return desired_joint_positions, current_stdDev, corresponding_iteration




def get_init_pos():
     
    return meanTraj[0]
