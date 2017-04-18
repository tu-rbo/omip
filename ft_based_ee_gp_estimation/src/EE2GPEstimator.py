#!/usr/bin/python

import rospy

from sensor_msgs.msg import Image
import cv2
from std_msgs.msg import Float64, Float64MultiArray, Int32

from scipy.spatial.distance import mahalanobis
from scipy.stats import norm

import tf
from cv_bridge import CvBridge, CvBridgeError

from omip_msgs.msg import RigidBodyPosesAndVelsMsg
from omip_msgs.msg import RigidBodyPoseAndVelMsg

from omip_msgs.msg import KinematicStructureMsg

from std_msgs.msg import Empty

import pickle

import numpy as np
import argparse
import scipy
import rospy
from scipy.linalg import *

import copy

import matplotlib.pyplot as plt

import yaml
import argparse
import pickle
import matplotlib.pyplot as plt
import tf.transformations as tlib
import numpy as np
import seaborn as sns
from sklearn.neural_network import MLPRegressor
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler

import tf
import rospkg

PERFECT_GRASP_JOINT = 0
UNCONSTRAINED_RY_GRASP_JOINT =1
UNCONSTRAINED_RY_TY_GRASP_JOINT = 2
DISCONNECTED_JOINT = 3


np.set_printoptions(precision=3, suppress=True)

def twist2homogeneousTransform(pose_twist):
    pose_twistm = np.array([[0., -pose_twist[5], pose_twist[4], pose_twist[0]],
                            [pose_twist[5], 0., -pose_twist[3], pose_twist[1]],
                            [-pose_twist[4], pose_twist[3], 0., pose_twist[2]],
                            [0., 0., 0., 0.0]])
    pose_ht = scipy.linalg.expm(pose_twistm)
    return pose_ht

def twistMsg2twistNumpy(twist_msg):
    twist_np = np.array([twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z,
                         twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z])
    return twist_np

class EE2GPEstimator:
    def __init__(self):

        rospy.loginfo("Creating EE2GPEstimator")        

        self.gt = PERFECT_GRASP_JOINT #Initial guess
        self.ft_current = None    
        self.ee2gp_pose = None    
        
        rospack = rospkg.RosPack()

        self.mm_models = dict()
        self.mm_models['inverse'] = dict()
        self.mm_models['inverse']['translation'] = pickle.load(
            open(rospack.get_path('ft_based_ee_gp_estimation') + '/nn_models/predictor_inv_t.pkl', 'rb'))
        self.mm_models['inverse']['translation_uncertainty'] = pickle.load(
            open(rospack.get_path('ft_based_ee_gp_estimation') + '/nn_models/predictor_inv_unc_t.pkl', 'rb'))
        self.mm_models['inverse']['rotation'] = pickle.load(
            open(rospack.get_path('ft_based_ee_gp_estimation') + '/nn_models/predictor_inv_r.pkl', 'rb'))
        self.mm_models['inverse']['rotation_uncertainty'] = pickle.load(
            open(rospack.get_path('ft_based_ee_gp_estimation') + '/nn_models/predictor_inv_unc_r.pkl', 'rb'))

        self.mm_models['forward'] = dict()
        self.mm_models['forward']['force'] = pickle.load(
            open(rospack.get_path('ft_based_ee_gp_estimation') + '/nn_models/predictor_fwd_f.pkl', 'rb'))
        self.mm_models['forward']['force_uncertainty'] = pickle.load(
            open(rospack.get_path('ft_based_ee_gp_estimation') + '/nn_models/predictor_fwd_unc_f.pkl', 'rb'))
        self.mm_models['forward']['torque'] = pickle.load(
            open(rospack.get_path('ft_based_ee_gp_estimation') + '/nn_models/predictor_fwd_tau.pkl', 'rb'))
        self.mm_models['forward']['torque_uncertainty'] = pickle.load(
            open(rospack.get_path('ft_based_ee_gp_estimation') + '/nn_models/predictor_fwd_unc_tau.pkl', 'rb'))
        
        self.br = tf.TransformBroadcaster()
        
        
        self.use_pressures = False
        
        #self.state = 

        self.acc_Y_pred = []
        
        self.acc_ft = []
        
        self.transformer_ros = tf.TransformerROS()
        
        self.pred_ft = None
        self.pred_ft_unc = None
        self.meas_ft = None
        self.ft_diff = None
        self.force_md_th = 2  #Standard deviations
        self.torque_md_th = 2  #Standard deviations
        self.pred_relpose = None
        self.pred_relpose_unc = None
        
        self.force_y_th = 10 #N
        self.torque_y_th = 1 #Nm
        self.pressures = None
        
        # init subscribers
        rospy.Subscriber("/ft_notool", Float64MultiArray, self.ft_cb)
        rospy.Subscriber("/joint_tracker/grasping_type", Float64MultiArray, self.gt_cb)        
        rospy.Subscriber("/ee2cp/predicted_state", Float64MultiArray, self.ee2gp_cb)        
        rospy.Subscriber("/plot_signal", Empty, self.plot_cb)
        
        if self.use_pressures:
          rospy.Subscriber("/pneumaticbox/pressures", Float64MultiArray, self.p_cb)
        
        
        #init publishers        
        self.rel_pose_meas_pub = rospy.Publisher("/ee2cp/measurement", Float64MultiArray, queue_size = 10)        
        self.pred_ft_pub = rospy.Publisher("/ft_pred", Float64MultiArray, queue_size = 10)
        
        self.slippage_detected_pub = rospy.Publisher("/ee2cp/slippage_detected", Int32, queue_size = 10)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        rospy.loginfo("EE2GPEstimator: Exiting")
        
    def p_cb(self, pressures):
        self.pressures = pressures

    #Inverse measurement model (from measurement to state)
    #Inverse modeling: use of the actual results of some measurements of the observable
    #parameters to infer the actual values of the model parameters
    def ft_cb(self, ft_signal):
        #rospy.loginfo("EE2GPEstimator: Received FT signal")
        self.ft_current = ft_signal
                
        X = [ft_signal.data]
        
        Xr = []
        
        if self.use_pressures and self.pressures != None:
            Xr = [ft_signal.data + self.pressures.data]
        else:
            Xr = [ft_signal.data]
        
        self.meas_ft = X[0]
        
        self.compare_meas_ft_and_predicted_ft()
        
        print 'Received ft \t\t\t', ft_signal.data
        
        #print X
        
        Y_pred = [[]]
        Y_pred_unc = [[]]
        
        #Easy first: estimate the relative pose with each received ft signal
        #Right now we only have one type of inv and fwd models for all joint types
        if self.gt == PERFECT_GRASP_JOINT or self.gt == UNCONSTRAINED_RY_GRASP_JOINT or self.gt == UNCONSTRAINED_RY_TY_GRASP_JOINT:
            Y_pred_t = self.mm_models['inverse']['translation'].predict(X)
            Y_pred_t_unc = self.mm_models['inverse']['translation_uncertainty'].predict(X)
            
            X_r = [list(Xr[0]) + Y_pred_t.tolist()[0][:]]
            
            Y_pred_r = self.mm_models['inverse']['rotation'].predict(X_r)
            
            #Change me: only because it stopped before training the unc network -> we use one without pressure
            #X_r2 = [list(ft_signal.data) + Y_pred_t.tolist()[0]]
            Y_pred_r_unc = self.mm_models['inverse']['rotation_uncertainty'].predict(X_r)
            
            Y_pred = [np.concatenate([Y_pred_t[0], Y_pred_r[0]])]
            Y_pred_unc = [np.concatenate([Y_pred_t_unc[0], Y_pred_r_unc[0]])]
            
        # elif self.gt == UNCONSTRAINED_RY_GRASP_JOINT:
        #     Y_pred = self.cylindrical_ry_grasp_inv_mm.predict(X)
        #     Y_pred_unc = self.cylindrical_ry_grasp_inv_mm_unc.predict(X)
        # elif self.gt == UNCONSTRAINED_RY_TY_GRASP_JOINT:
        #     Y_pred = self.cylindrical_ryty_grasp_inv_mm.predict(X)
        #     Y_pred_unc = self.cylindrical_ryty_grasp_inv_mm_unc.predict(X)
        else:
            Y_pred = [[0, 0, 0, 0, 0, 0]]
            Y_pred_unc = [[0, 0, 0, 0, 0, 0]]
        
        ee_2_gp_translation_relative_ht = tlib.translation_matrix(Y_pred[0][0:3])
        ee_2_gp_rotation_relative_ht = tlib.euler_matrix(*Y_pred[0][3:6])
        
        ee_2_gp_pose_relative_ht = np.dot(ee_2_gp_translation_relative_ht, ee_2_gp_rotation_relative_ht)        
        
        #self.br.sendTransform(Y_pred[0,0:3],
                     #tf.transformations.quaternion_from_euler(*Y_pred[0,3:6]),
                     #rospy.Time.now(),
                     #"gp_point",
                     #"ee_initial")                     
                     
        self.acc_Y_pred.append(Y_pred[0])

        self.pred_relpose = Y_pred[0]

        print "Predicted rel pose: \t\t", self.pred_relpose

        #The regressor can give uncertainties negative (interpolation)
        self.pred_relpose_unc = np.absolute(Y_pred_unc[0])

        print "Predicted UNCERTAINTY rel pose: \t", self.pred_relpose_unc
        
        #ee_2_gp_pose_relative_ht2 = self.transformer_ros.fromTranslationRotation(Y_pred[0,0:3],
                     #tf.transformations.quaternion_from_euler(*Y_pred[0,3:6]))
        #gp_2_ee_pose_relative_ht = np.linalg.inv(ee_2_gp_pose_relative_ht2)
        #self.br.sendTransform(tlib.translation_from_matrix(gp_2_ee_pose_relative_ht),
                     #tf.transformations.quaternion_from_euler(*tlib.euler_from_matrix(gp_2_ee_pose_relative_ht)),
                     #rospy.Time.now(),
                     #"gp_point2",
                     #"ee")   
                    
        self.rel_pose_meas_pub.publish(data=np.concatenate([self.pred_relpose, self.pred_relpose_unc]).tolist())
        
    def compare_meas_ft_and_predicted_ft(self):
        if self.meas_ft != None and self.pred_ft != None:
            print "Measured FT: ", self.meas_ft
            print "Predicted FT: ", np.array(self.pred_ft)
            self.ft_diff = self.meas_ft - self.pred_ft
        else:
            return

        force_meas = np.array([self.meas_ft[0], self.meas_ft[1], self.meas_ft[2]])
        force_pred = np.array([self.pred_ft[0], self.pred_ft[1], self.pred_ft[2]])
        
        force_meas_y = self.meas_ft[1]
        force_pred_y = self.pred_ft[1]

        torque_meas = np.array([self.meas_ft[3], self.meas_ft[4], self.meas_ft[5]])
        torque_pred = np.array([self.pred_ft[3], self.pred_ft[4], self.pred_ft[5]])
        
        torque_meas_y = self.meas_ft[4]
        torque_pred_y = self.pred_ft[4]

        force_pred_inv_sd = np.zeros((3,3))
        force_pred_inv_sd[0, 0] = 1.0 / self.pred_ft_unc[0]
        force_pred_inv_sd[1, 1] = 1.0 / self.pred_ft_unc[1]
        force_pred_inv_sd[2, 2] = 1.0 / self.pred_ft_unc[2]

        torque_pred_inv_sd = np.zeros((3, 3))
        torque_pred_inv_sd[0, 0] = 1.0 / self.pred_ft_unc[3]
        torque_pred_inv_sd[1, 1] = 1.0 / self.pred_ft_unc[4]
        torque_pred_inv_sd[2, 2] = 1.0 / self.pred_ft_unc[5]

        #Compute the Mahalanobis distance (see https://en.wikipedia.org/wiki/Mahalanobis_distance#Normal_distributions)
        md_force = mahalanobis(force_meas, force_pred, force_pred_inv_sd)
        md_torque = mahalanobis(torque_meas, torque_pred, torque_pred_inv_sd)
        
        if np.linalg.norm(md_force) > self.force_md_th:
            rospy.logerr("Large difference between predicted and measured force! Did you lose grasp?")
            print "predicted: ", force_pred
            print "measured: ", force_meas
            self.slippage_detected_pub.publish(data=1)
        elif np.linalg.norm(md_torque) > self.torque_md_th:
            rospy.logerr("Large difference between predicted and measured torque! Did you lose grasp?")
            self.slippage_detected_pub.publish(data=2)
            print "predicted: ", torque_pred
            print "measured: ", torque_meas
        elif abs(force_meas_y - force_pred_y) > self.force_y_th:
            rospy.logerr("Large difference between predicted and measured Force in Y! Slippage!")
            self.slippage_detected_pub.publish(data=3)
            print "predicted: ", force_pred_y
            print "measured: ", force_meas_y
        elif abs(torque_meas_y - torque_pred_y) > self.torque_y_th:
            rospy.logerr("Large difference between predicted and measured Torque in Y! Slippage!")
            self.slippage_detected_pub.publish(data=4)
            print "predicted: ", torque_meas_y
            print "measured: ", torque_pred_y
        else:
            self.slippage_detected_pub.publish(data=0)
          
    #Forward measurement model (from state (predicted) to measurement (predicted)) -> Useful for failure detection
    #Forward modeling: discovery of the physical laws allowing us, for given values of
    #the model parameters, to make predictions on the results of measurements on some
    #observable parameters.
    def ee2gp_cb(self, ee2gp_pose):
        #rospy.loginfo("EE2GPEstimator: Received EE2GP signal")
        self.ee2gp_pose = ee2gp_pose
        
        X = [ee2gp_pose.data]
        
        #print X
        
        Y_pred = [[]]

        print 'Received rel pose \t\t', ee2gp_pose.data
        
        #Easy first: estimate the relative pose with each received ft signal
        # Right now we only have one type of inv and fwd models for all joint types
        if self.gt == PERFECT_GRASP_JOINT or self.gt == UNCONSTRAINED_RY_GRASP_JOINT or self.gt == UNCONSTRAINED_RY_TY_GRASP_JOINT:
            Y_pred_f = self.mm_models['forward']['force'].predict(X)
            Y_pred_f_unc = self.mm_models['forward']['force_uncertainty'].predict(X)
            Y_pred_tau = self.mm_models['forward']['torque'].predict(X)
            Y_pred_tau_unc = self.mm_models['forward']['torque_uncertainty'].predict(X)
            Y_pred = [np.concatenate([Y_pred_f[0], Y_pred_tau[0]])]
            Y_pred_unc = [np.concatenate([Y_pred_f_unc[0], Y_pred_tau_unc[0]])]
        # elif self.gt == UNCONSTRAINED_RY_GRASP_JOINT:
        #     Y_pred = self.cylindrical_ry_grasp_fwd_mm.predict(X)
        # elif self.gt == UNCONSTRAINED_RY_TY_GRASP_JOINT:
        #     Y_pred = self.cylindrical_ryty_grasp_fwd_mm.predict(X)
        else:
            Y_pred = [[0, 0, 0, 0, 0, 0]]
            
        self.pred_ft = Y_pred[0]

        #print "Predicted ft: ", self.pred_ft

        self.pred_ft_unc = Y_pred_unc[0]

        #print "Predicted ft unc: ", self.pred_ft_unc
        
        #print 'Predicted ft ', self.pred_ft
        
        self.pred_ft_pub.publish(data=Y_pred[0] + Y_pred_unc[0])
        
        
    def plot_cb(self, empty):

        n_timesteps = len(self.acc_Y_pred)
        
        self.acc_Y_pred = np.array(self.acc_Y_pred)
                
        #Plot combinations of both
        f,axarr = plt.subplots(3)
        axarr[0].plot(range(n_timesteps),self.acc_Y_pred[:,0],label="ptx")
        axarr[0].set_title('PTx')
        axarr[0].legend()

        axarr[1].plot(range(n_timesteps),self.acc_Y_pred[:,1],label="pty")
        axarr[1].set_title('PTy')
        axarr[1].legend()
        
        axarr[2].plot(range(n_timesteps),self.acc_Y_pred[:,2],label="ptz")
        axarr[2].set_title('PTz')
        axarr[2].legend()
        
        f,axarr = plt.subplots(3)
        axarr[0].plot(range(n_timesteps),self.acc_Y_pred[:,3],label="prx")
        axarr[0].set_title('PRx')
        axarr[0].legend()

        axarr[1].plot(range(n_timesteps),self.acc_Y_pred[:,4],label="pry")
        axarr[1].set_title('PRy')
        axarr[1].legend()
        
        axarr[2].plot(range(n_timesteps),self.acc_Y_pred[:,5],label="prz")
        axarr[2].set_title('PRz')
        axarr[2].legend()
        
        plt.show()
        
    def gt_cb(self, gt_probs):
        #rospy.loginfo("EE2GPEstimator: Received grasping type")
        
        gt_probabilities = gt_probs.data
        
        self.gt = gt_probabilities.index(max(gt_probabilities))
        
        


def main():
    rospy.init_node('EE2GPEstimator')

    with EE2GPEstimator() as ee2gp:
        rospy.spin()

if __name__ == "__main__":
    main()
