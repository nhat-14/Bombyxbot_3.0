#!/usr/bin/env python3
from matplotlib import pyplot as plt
import rospy
from std_msgs.msg import Float32

def plot_x(msg):
    global ave_entropy_gain
    ave_entropy_gain.append(msg.data)


if __name__ == '__main__':
    ave_entropy_gain = [0]
    x = []
    y = []
    step = []

    rospy.init_node("plotter")
    rospy.Subscriber("/gsl_ac/entropy_reporter", Float32, plot_x)
    rospy.spin()

    step = range(1, len(ave_entropy_gain)+1, 1)
    plt.plot(step, ave_entropy_gain)
    plt.xlabel('Step')
    plt.ylabel('Entropy gain')
    # axs[1].hlines(y=0.05, xmin = 0, xmax = len(step), linestyles='--',linewidth=1, color='r')
    plt.show()