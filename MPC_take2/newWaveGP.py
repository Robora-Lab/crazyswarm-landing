# Imports
import math
import numpy as np
import pandas as pd
from random import random

import torch
from scipy.stats import qmc
from itertools import product
from matplotlib import pyplot as plt
from sklearn.gaussian_process import GaussianProcessRegressor

# from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C, RationalQuadratic as RQ, WhiteKernel, \
    # ExpSineSquared as Exp, DotProduct as Lin

#import MPC.oldWaveGP


#import waveFunction
import casadi as ca
import gpytorch
import tensorflow as tf


# *Comments based on https://docs.gpytorch.ai/en/v1.6.0/examples/01_Exact_GPs/Simple_GP_Regression.html
class GPYModel(gpytorch.models.ExactGP):

    # *An __init__ method that takes the training data and a likelihood, and constructs whatever objects are necessary for the model’s forward method. 
    # *This will most commonly include things like a mean module and a kernel module.
    def __init__(self, train_x, train_y, likelihood):
        super(GPYModel, self).__init__(train_x, train_y, likelihood)
        self.K_plus_noise_inv = None
        self.K_plus_noise = None

        self.mean_module = gpytorch.means.ConstantMean()                                 # *This defines the prior mean of the GP.(If you don’t know which mean to use, a gpytorch.means.ConstantMean() is a good place to start.)
        
        # Define some unique kernels
        Ker_RBF =  gpytorch.kernels.RBFKernel()
        Scaled_Ker_RBF = gpytorch.kernels.ScaleKernel(Ker_RBF)

        Ker_Per = gpytorch.kernels.PeriodicKernel()
        additive_kernel = gpytorch.kernels.AdditiveKernel(Scaled_Ker_RBF+Ker_Per)
        
        self.covar_module = Scaled_Ker_RBF                                               # *This defines the prior covariance of the GP.(If you don’t know which kernel to use, a gpytorch.kernels.ScaleKernel(gpytorch.kernels.RBFKernel()) is a good place to start).
        
        self.likelihood=likelihood                                                       # *This is the most common likelihood used for GP regression (gpytorch.likelihoods.GaussianLikelihood)
        self.model_type=gpytorch.models.ExactGP                                          # *This handles most of the inference.
        # *Documentation also suggests: A MultivariateNormal Distribution (gpytorch.distributions.MultivariateNormal) - This is the object used to represent multivariate normal distributions.

        self.init_K(train_x)

    def init_K(self,train_x):
        K_lazy = self.covar_module(train_x.double())
        K_lazy_plus_noise = K_lazy.add_diag(self.likelihood.noise)

        n_samples = train_x.shape[0]
        self.K_plus_noise = K_lazy_plus_noise.matmul(torch.eye(n_samples).double())

        self.K_plus_noise_inv = torch.inverse(self.K_plus_noise)

    # *A forward method that takes in some n×d data x and returns a MultivariateNormal with the prior mean and covariance evaluated at x. 
    # *In other words, we return the vector μ(x) and the n×n matrix Kxx representing the prior mean and covariance matrix of the GP.
    def forward(self, x):
        mean_x = self.mean_module(x)
        covar_x = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)


class ModelAttempt:
    def __init__(self):
        self.z_training = None
        self.xt_training = None
        self.model_attempt=None
        self.casadi_predict=None

        self.prepare_train_inputs()


    def prepare_train_inputs(self):
        #
        print("Prepare Train Inputs...")
        # 
        
        ##### Hypercubic Sampling #####
        npara = int(2)                  # Two input parameters (x and t)        
        nsamp = int(200)                # Draw a number of sample points
        # l_bound = [0, 5]                # Bounds for sample space
        # u_bound = [20, 20]
        l_bound = [4, 4]
        u_bound = [12, 12]

        sampler = qmc.LatinHypercube(d=npara) 
        sample = sampler.random(n=nsamp)
        self.xt_training = qmc.scale(sample, l_bound, u_bound) # These are the observations/training inputs

        z = [oldWaveGP.theta_function(self.xt_training[:, 0], self.xt_training[:, 1])]  # Lowercase z holds the sampled values
        self.z_training = np.array([item for sublist in z for item in sublist])  # Convert NumPy arrays to Python lists
    
    def start_gpy_gp(self):
        likelihood = gpytorch.likelihoods.GaussianLikelihood()       # idk how likelihood stuff works...
        xt_tens=torch.tensor(self.xt_training, dtype=torch.float32)  # Place data into tensors
        z_tens = torch.tensor(self.z_training, dtype=torch.float32)
        self.model_attempt = GPYModel(xt_tens, z_tens, likelihood)   # Instantiate ExactGP model
        self.model_attempt.train()                                   # Find optimal model hyperparameters
        likelihood.train()
        optimizer = torch.optim.Adam(self.model_attempt.parameters(), lr=0.1) 
    
        # Train the model
        num_iter = 2
        for i in range(num_iter):
            optimizer.zero_grad()                                       # *Zero gradients from previous iteration
            output = self.model_attempt(xt_tens)                        # *Output from model
            loss = (-likelihood(output, z_tens).log_prob(z_tens)).sum() # *Calc loss and backdrop gradients
            loss.backward()

            # Display training progress
            print('Iter %d/%d - Loss: %.3f  noise: %.3f' % (
                i + 1, num_iter, loss.item(),
                self.model_attempt.likelihood.noise.item()
            ))

            ### For Additive Kernel ###
            # print('##### kernels named parameter #####')
            # for param_name, param in self.model_attempt.covar_module.kernels.named_parameters():
            #     print(f'Parameter name: {param_name:42} value = {param.item()}')
            
            # print("RBF Part")
            # print(self.model_attempt.covar_module.kernels.get_submodule("0.kernels.0"))
            # print("RBF lenghtscale")
            # print(self.model_attempt.covar_module.kernels.get_submodule("0.kernels.0").base_kernel.lengthscale.item())
            # print("Periodic Part")
            # print(self.model_attempt.covar_module.kernels.get_submodule("0.kernels.1"))
            # print("Periodic lengthscale")
            # print(self.model_attempt.covar_module.kernels.get_submodule("0.kernels.1").lengthscale.item())
        
            # print()
            # print()
            

            optimizer.step()

        self.casadi_predict = self.cas_pred()

    # modified function from safe control gym
    def cas_pred(self):
        train_inputs = self.xt_training
        train_targets = self.z_training

        # RBF Kernel
        length_scale = self.model_attempt.covar_module.base_kernel.lengthscale.detach().numpy() 
        output_scale = self.model_attempt.covar_module.outputscale.detach().numpy()
        # Additive Kernel
        # length_scale = self.model_attempt.covar_module.base_kernel.kernels.0.lengthscale.detach().numpy() 
        # output_scale = self.model_attempt.covar_module.outputscale.detach().numpy()       
        
        #Nx = len(self.model_attempt.input_mask)
        #Nx=torch.ones_like(torch.tensor(train_inputs[0]))

        # **I feel like calling this symbol "z" is misleading, it is our input vector [x, t]
        z = ca.SX.sym('z', 2)   # trying 2 idk?    

        # Determine the covariance of the query point and all known observations
        K_z_ztrain = ca.Function('k_z_ztrain',                                                 # Function name -> "covariance matrix zxz training"
                                 [z],                                                          # Input -> Query point
                                 [covSEard(z, train_inputs.T, length_scale.T, output_scale)],  # Some kind of sine exponential covariance vector (compares query point to all observation points). Size is nsamp x 1.
                                 ['z'],                                                        # Input name:
                                 ['K'])                                                        # Output name: K -> for "covariance matrix zxz"
        print('K_zz function: ')
        print(K_z_ztrain)                                               # Size: 1 x nsamp
        print('noise: ')                                           
        print(self.model_attempt.K_plus_noise_inv.detach().numpy())     # Size: nsamp x nsamp
        print('train_targets: ')                                        
        print(train_targets)                                            # Size: 1 x nsamp

        # Use the covariance of query and observations, the measured values of those observations, and noise information to estimate value
        predict = ca.Function('pred',
                              [z],
                              [K_z_ztrain(z=z)['K'] @ self.model_attempt.K_plus_noise_inv.detach().numpy() @ train_targets],
                            #   [K_z_ztrain(z=z)['K'] @ train_targets.T],                                                     # What happens with no noise? -> not really sure
                              ['z'],
                              ['mean'])
        # K_z_ztrain(z=z)['K'] can be interpeted as pass the function K_z_ztrain our query point "z", and return the cov matrix "K(zxz)"

        print('predict function: ')
        print(predict)

        return predict


#function from safe control gym
def covSEard(x,z,ell,sf2):
    # x: Casadi symbol of the query point
    # z: training inputs (observations from hypercubic sampling) # **Again I feel like calling this "z" is misleading...
    # ell: length scale 
    # sf2: output scale

    # print('x (casadi query point): ')
    # print(x)
    # print('z (training inputs): ')
    # print(z)
    # print('length scale: %.3f' % (ell))
    # print('output scale: %.3f' % (sf2))

    dist = ca.sum1((x - z)**2 / ell**2)
    # print('distance?: ')
    # print(dist)

    return sf2 * ca.SX.exp(-.5 * dist)


#z_pred=model_attempt.casadi_predict(z=z[:, i])['mean']


    ########################## RBF Output #################################
    # (uav) nathan@nathan-HP:~/uav-usv-landing$ python3 run_mpcs.py 
    # 2024-04-06 23:45:23.771866: I tensorflow/tsl/cuda/cudart_stub.cc:28] Could not find cuda drivers on your machine, GPU will not be used.
    # 2024-04-06 23:45:23.813781: I tensorflow/core/platform/cpu_feature_guard.cc:182] This TensorFlow binary is optimized to use available CPU instructions in performance-critical operations.
    # To enable the following instructions: AVX2 FMA, in other operations, rebuild TensorFlow with the appropriate compiler flags.
    # 2024-04-06 23:45:24.476706: W tensorflow/compiler/tf2tensorrt/utils/py_utils.cc:38] TF-TRT Warning: Could not find TensorRT
    # go GP
    # Prepare Train Inputs...

    # ## Model ##
    # GPYModel(
    # (likelihood): GaussianLikelihood(
    #     (noise_covar): HomoskedasticNoise(
    #     (raw_noise_constraint): GreaterThan(1.000E-04)
    #     )
    # )
    # (mean_module): ConstantMean()
    # (covar_module): ScaleKernel(
    #     (base_kernel): RBFKernel(
    #     (raw_lengthscale_constraint): Positive()
    #     )
    #     (raw_outputscale_constraint): Positive()
    # )
    # )
    # ### covar_module ###
    # ScaleKernel(
    # (base_kernel): RBFKernel(
    #     (raw_lengthscale_constraint): Positive()
    # )
    # (raw_outputscale_constraint): Positive()
    # )
    # #### base_kernel ####
    # RBFKernel(
    # (raw_lengthscale_constraint): Positive()
    # )
    # ##### lengthscale #####
    # tensor([[0.7444]], grad_fn=<SoftplusBackward0>)
    # ###### item ######
    # 0.7443966865539551