#!/usr/bin/env python

import os
import csv
import cv2
import numpy as np
import math
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split

from keras.models import Sequential
from keras.layers import Dense, Flatten, Lambda, Cropping2D, MaxPooling2D, Conv2D, Dropout
from keras.optimizers import Adam

import argparse

ROOT_DIR = './'
DS_CSV = '{}/sim_camera_src/ds.csv'.format(ROOT_DIR)
IDX_TO_SPARSE = {'0':0, '1':1, '2':2, '4':3}


def read_dataset():
    samples = []
    with open(DS_CSV) as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            samples.append(line)

    train_samples, validation_samples = train_test_split(samples, test_size=0.2)
    return train_samples, validation_samples


def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            labels = []
            for batch_sample in batch_samples:
                [name, label_idx] = batch_sample
                image = cv2.imread(name)
                
                #print(image.shape)
                sparse_label = [0] * 4
                sparse_label[IDX_TO_SPARSE[label_idx]] = 1
                images.append(image)
                labels.append(sparse_label)

            X_train = np.array(images)
            y_train = np.array(labels)
            yield shuffle(X_train, y_train)


def create_model(input_shape=(600, 800, 3)):
    model = Sequential()

    # Preprocess incoming data, centered around zero with small standard deviation 
    model.add(Lambda(lambda x: x/127.5 - 1.,
        input_shape=input_shape,
        output_shape=input_shape))
    #
    # Cropping
    #model.add(Cropping2D(cropping=((60,24), (0,0)), input_shape=input_shape))
    
    #Convolutions
    model.add(Conv2D(6, (5,5), strides=(2,2), activation='relu'))
    #print(model.output_shape)
    model.add(Conv2D(12, (5,5), strides=(2,2), activation='relu'))
    #model.add(Conv2D(48, (5,5), strides=(2,2), activation='relu'))
    #model.add(Conv2D(64, (3,3), activation='relu'))
    #model.add(Conv2D(64, (3,3), activation='relu'))
    #model.add(Conv2D(64, (2,2), activation='relu'))
    
    # Densors
    model.add(Flatten())
    model.add(Dense(128, activation='relu'))
#     model.add(Dense(64, activation='relu'))
#     model.add(Dense(16, activation='relu'))
    
    model.add(Dense(4, activation='softmax'))
    return model

# model.add(Conv2D(32, (3, 3), input_shape=input_shape))
# model.add(Activation('relu'))
# model.add(MaxPooling2D(pool_size=(2, 2)))

# model.add(Conv2D(32, (3, 3)))
# model.add(Activation('relu'))
# model.add(MaxPooling2D(pool_size=(2, 2)))

# model.add(Conv2D(64, (3, 3)))
# model.add(Activation('relu'))
# model.add(MaxPooling2D(pool_size=(2, 2)))

# model.add(Flatten())
# model.add(Dense(64))
# model.add(Activation('relu'))
# model.add(Dropout(0.5))
# model.add(Dense(1))
# model.add(Activation('sigmoid'))


def run_experiment(batch_size=32, lr=1e-5, epochs=3):
    train_samples, validation_samples = read_dataset()
    print("***************************")
    print("Epochs={}; \t lr={}; \t Batch={}; \tTrain={}; \tValid={}".format(epochs, lr, batch_size, len(train_samples), len(validation_samples)))
    print("***************************")
    # compile and train the model using the generator function
    train_generator = generator(train_samples, batch_size=batch_size)
    validation_generator = generator(validation_samples, batch_size=batch_size)

    row, col, ch = 600, 800, 3  # Trimmed image format

    model = create_model((row, col, ch))

    opt = Adam(lr=lr)
    model.compile(optimizer=opt, loss='categorical_crossentropy', metrics=['accuracy'])
    model.fit_generator(
            train_generator, 
            steps_per_epoch=math.ceil(len(train_samples)/batch_size),
            validation_steps=math.ceil(len(validation_samples)/batch_size),
            validation_data=validation_generator, 
            epochs=epochs,
            )
 
#    model.save('model_task-01_ds.v001_E-{}_LR-{}.h5'.format(epochs, lr))
#    print("Model saved")

if __name__ =='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
            '--batch',
            type=int,
            help="Batch size",
            default=32,
            )
    parser.add_argument(
            '--lr',
            help="Learning rate",
            type=float,
            default=1e-5,
            )
    parser.add_argument(
            '--epochs',
            type=int,
            help="Epochs number",
            default=3,
            )
    args = vars(parser.parse_args())
    run_experiment(lr=args['lr'], epochs=args['epochs'], batch_size=args['batch'])
