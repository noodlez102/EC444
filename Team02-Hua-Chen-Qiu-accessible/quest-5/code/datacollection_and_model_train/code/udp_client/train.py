import os
import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow.keras.preprocessing.image import load_img, img_to_array
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Conv2D, Dense, Flatten, Dropout, Input, concatenate, MaxPooling2D
from tensorflow.keras.optimizers import Adam

# Constants
# Constants
IMAGE_HEIGHT, IMAGE_WIDTH, CHANNELS = 480, 640, 3  # Adjust based on your image size

def load_and_preprocess_data(image_dir, csv_dir, weight_factor=2.0):
    image_paths = [os.path.join(image_dir, filename) for filename in sorted(os.listdir(image_dir)) if filename.endswith('.jpg')]
    csv_paths = [os.path.join(csv_dir, filename) for filename in sorted(os.listdir(csv_dir)) if filename.endswith('.csv')]
    
    images = []
    sensor_data = []
    directions = []
    
    for image_path, csv_path in zip(image_paths, csv_paths):
        # Load and preprocess image
        image = load_img(image_path, target_size=(IMAGE_HEIGHT, IMAGE_WIDTH))
        image = img_to_array(image)
        image = image / 255.0  # Normalize the image
        
        # Load and preprocess CSV data
        with open(csv_path, 'r') as f:
            lines = f.readlines()  # Read all lines
            for line in lines:
                if line.strip():  # Check if line is not empty
                    try:
                        data = [float(x) for x in line.strip().split(',')]  # Split values by comma and convert to float
                        sensors = [data[i] for i in [0, 1, 2]]
                        # Multiply the value at index 4 by the weight_factor
                        sensors.append(data[4])
                        direction = [data[3]]  # Extract value at index 4 as direction
                        sensors = sensors / np.array([8, 45, 100, 1])
                        sensor_data.append(sensors)
                        directions.append(direction)
                    except ValueError:
                        print("Error parsing line:", line.strip())
                        # If parsing fails, skip this data sample
                        continue
                    except IndexError:
                        print("Error parsing line:", line.strip())
                        # If parsing fails, skip this data sample
                        continue
        # Add the preprocessed image
        images.append(image)

        # Check if the number of sensor data samples matches the number of images
        if len(sensor_data) != len(images):
            print("Mismatch between image and sensor data samples. Skipping this data sample.")
            if images:  # Check if images list is not empty
                images.pop()  # Remove the last image
            continue

    # Debugging: Print lengths of lists
    print("Number of images:", len(images))
    print("Number of sensor data samples:", len(sensor_data))
    print("Number of direction samples:", len(directions))
    
    return np.array(images), np.array(sensor_data), np.array(directions)




def create_and_train_model(image_dir, csv_dir):
    images, sensor_data, directions = load_and_preprocess_data(image_dir, csv_dir)
    model = create_model()
    train_model(model, images, sensor_data, directions)
    model.save('my_toy_car_model.h5')

def create_model():
    # Image input branch
    image_input = Input(shape=(IMAGE_HEIGHT, IMAGE_WIDTH, CHANNELS))
    x = Conv2D(16, (3, 3), activation='relu')(image_input)
    x = MaxPooling2D()(x)
    x = Dropout(0.2)(x)
    x = Flatten()(x)

    # Sensor data input branch
    sensor_input = Input(shape=(4,))
    # y = Dense(16, activation='relu')(sensor_input)
    y = sensor_input

    # Merge branches
    combined = concatenate([x, y])
    
    # Decision layer
    z = Dropout(0.2)(combined)
    output = Dense(1, activation='sigmoid')(z)  # Output shape is (None, 1) for regression task

    model = Model(inputs=[image_input, sensor_input], outputs=output)
    model.compile(optimizer=Adam(learning_rate=0.001), loss='mean_squared_error', metrics=['accuracy'])
    return model



def train_model(model, images, sensor_data, directions):
    model.fit([images, sensor_data], directions, batch_size=32, epochs=10, validation_split=0.2) 
    # batch size is the number of samples per gradient update
    # epochs is the number of iterations over the entire dataset
    # so the minumum number of iterations is batch_size * epochs

create_and_train_model('data_folder', 'data_folder')