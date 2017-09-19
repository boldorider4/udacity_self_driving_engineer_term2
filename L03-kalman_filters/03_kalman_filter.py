# Write a program that will iteratively update and
# predict based on the location measurements 
# and inferred motions shown below. 

def update(mean1, var1, mean2, var2):
    new_mean = float(var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1./(1./var1 + 1./var2)
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]

measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]
measurement_sig = 4.
motion_sig = 2.
mu = 0.
sig = 10000.

mean_mot, var_mot = mu, sig

for measurement, mot in zip(measurements, motion):
    mean_up, var_up = update(mean_mot, var_mot, measurement, measurement_sig)
    mean_mot, var_mot = predict(mean_up, var_up, mot, motion_sig)
    print('update  ', [mean_up, var_up])
    print('predict ', [mean_mot, var_mot])
