'''
code is created by LONG-yy98
https://github.com/LONG-yy98/paparazzi-coursework-2023/blob/master-group12/misc/svm/main.py

download zip files from https://github.com/LONG-yy98/paparazzi-coursework-2023/tree/master-group12/misc/svm
and put them under the dataset dir

'''

import cv2
import os
import numpy as np
from sklearn import svm
import random

def read_img(path):
    col = []
    for filename in os.listdir(path):
        img = cv2.cvtColor(cv2.resize(cv2.imread(path+'/'+filename),(6,6)),cv2.COLOR_BGR2GRAY)
        img = img.flatten()
        # print(img.shape)
        col.append(img)
    return col


obstacle = np.random.permutation(read_img("./mav_datasets/cyberzoo_poles/0"))
empty = np.random.permutation(read_img("./mav_datasets/cyberzoo_poles/1"))

# cv2.imshow("aa",obstacle[22])
# cv2.waitKey(0)

obstacle_train = obstacle[0:100,:]
obstacle_test = obstacle[100:,:]
empty_train = empty[0:220,:]
empty_test = empty[220:,:]

obstacle_train_label = np.zeros(len(obstacle_train))
obstacle_test_label = np.zeros(len(obstacle_test))
empty_train_label = np.ones(len(empty_train))
empty_test_label = np.ones(len(empty_test))

train_data = np.vstack((obstacle_train,empty_train))
train_label = np.hstack((obstacle_train_label,empty_train_label))
test_data = np.vstack((obstacle_test,empty_test))
test_label = np.hstack((obstacle_test_label,empty_test_label))

clf = svm.SVC()
clf.fit(train_data, train_label)

train_predic = clf.predict(train_data)
print("train_accuracy: ",np.sum(train_predic==train_label)/len(train_predic))

test_predic = clf.predict(test_data)
print("test_accuracy: ",np.sum(test_predic==test_label)/len(test_label))