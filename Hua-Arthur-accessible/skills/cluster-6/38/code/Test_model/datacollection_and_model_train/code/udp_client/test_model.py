import numpy as np
import tensorflow as tf
import os
IMAGE_HEIGHT, IMAGE_WIDTH, CHANNELS = 64, 64, 3  # Adjust based on your image size
# Load the saved model
model = tf.keras.models.load_model('my_toy_car_model.h5')

# Function to load and preprocess test images
def load_test_images(image_dir):
    test_images = []
    image_files = sorted(os.listdir(image_dir))
    for filename in image_files:
        if filename.endswith('.jpg'):
            image_path = os.path.join(image_dir, filename)
            image = tf.keras.preprocessing.image.load_img(image_path, target_size=(IMAGE_HEIGHT, IMAGE_WIDTH))
            image = tf.keras.preprocessing.image.img_to_array(image)
            image = image / 255.0  # Normalize pixel values
            test_images.append(image)
    return np.array(test_images)

# Function to load test sensor data
def load_test_sensor_data(sensor_dir):
    test_sensor_data = []
    sensor_files = sorted(os.listdir(sensor_dir))
    for filename in sensor_files:
        if filename.endswith('.csv'):
            sensor_path = os.path.join(sensor_dir, filename)
            with open(sensor_path, 'r') as f:
                lines = f.readlines()
                for line in lines:
                    if line.strip():  # Check if line is not empty
                        try:
                            data = [float(x) for x in line.strip().split(',')]  # Split values by comma and convert to float
                            sensors = [data[i] for i in [0, 1, 2, 3]]  # Extract values at indices 1, 2, 3, and 5 as sensor data
                            test_sensor_data.append(sensors)
                        except ValueError:
                            print("Error parsing line:", line.strip())
    return np.array(test_sensor_data)

# Path to test image and sensor data folders
test_image_dir = 'test_image'
test_sensor_dir = 'test_senso'

# Load test images
test_images = load_test_images(test_image_dir)

# Load test sensor data
test_sensor_data = load_test_sensor_data(test_sensor_dir)

# Make predictions
predictions = model.predict([test_images, test_sensor_data])
print(predictions)
