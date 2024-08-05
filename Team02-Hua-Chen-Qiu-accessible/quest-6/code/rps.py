import os
import cv2
import numpy as np

# Path to the directory containing the images
data_dir = ""

# Define the classes
classes = ['rock', 'paper', 'scissors']

# Initialize lists to store images and labels
images = []
labels = []

# Iterate through each class folder
for i, class_name in enumerate(classes):
    class_dir = os.path.join(data_dir, class_name)
    # Iterate through each image in the class folder
    for image_name in os.listdir(class_dir):
        # Read the image
        image_path = os.path.join(class_dir, image_name)
        image = cv2.imread(image_path)
        # Resize the image to 300x200 pixels
        image = cv2.resize(image, (300, 200))
        # Append the image to the images list
        images.append(image)
        # Append the label to the labels list
        labels.append(i)  # Use the index of the class as the label

# Convert the lists to NumPy arrays
images = np.array(images)
labels = np.array(labels)

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense

# Split the data into training and testing sets
from sklearn.model_selection import train_test_split
X_train, X_test, y_train, y_test = train_test_split(images, labels, test_size=0.2, random_state=42)

# Preprocess the data
X_train = X_train.astype('float32') / 255.0
X_test = X_test.astype('float32') / 255.0

# Convert the labels to one-hot encoding
num_classes = len(classes)
y_train = keras.utils.to_categorical(y_train, num_classes)
y_test = keras.utils.to_categorical(y_test, num_classes)

# Define the model architecture
model = Sequential([
    Conv2D(32, (3, 3), activation='relu', input_shape=(200, 300, 3)),
    MaxPooling2D((2, 2)),
    Conv2D(64, (3, 3), activation='relu'),
    MaxPooling2D((2, 2)),
    Flatten(),
    Dense(64, activation='relu'),
    Dense(num_classes, activation='softmax')
])

# Compile the model
model.compile(optimizer='adam',
              loss='categorical_crossentropy',
              metrics=['accuracy'])


# Train the model
model.fit(X_train, y_train, epochs=10, batch_size=32, validation_data=(X_test, y_test))
model.save('rps.h5')