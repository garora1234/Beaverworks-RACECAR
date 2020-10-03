import tensorflow as tf
from tensorflow.keras.datasets import cifar10
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout, Activation, Flatten
from tensorflow.keras.layers import Conv2D, MaxPooling2D
import numpy as np
import pickle


#print("Num GPUs Available: ", len(tf.config.experimental.list_physical_devices('GPU')))

pickle_in = open("X.pickle","rb")
X = pickle.load(pickle_in)

pickle_in = open("y.pickle","rb")
y = pickle.load(pickle_in)

IMG_SIZE = 128

X = np.array(X).reshape(-1, IMG_SIZE, IMG_SIZE, 3)
#print(X.shape)
y = np.array(y)
y = tf.keras.utils.to_categorical(y)
X = X/255.0

model = Sequential()

model.add(Conv2D(128, (3, 3),activation='relu', input_shape=(128,128,3)))
model.add(Conv2D(128, (3, 3),activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.2))

model.add(Conv2D(256, (3, 3), activation='relu'))
model.add(Conv2D(256, (3, 3), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.2))

model.add(Conv2D(512, (3, 3), activation='relu'))
model.add(Conv2D(512, (3, 3), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.2))


model.add(Flatten()) 

model.add(Dense(256, activation='relu'))

model.add(Dense(3, activation='softmax'))

from tensorflow.keras.optimizers import SGD
opt = SGD(lr=0.01,momentum=0.9)
model.compile(loss='categorical_crossentropy',
              optimizer=opt,
              metrics=['accuracy'])

model.fit(X, y, epochs=10,batch_size=32)

model.save('RACECAR.model')