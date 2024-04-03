import cv2
import tensorflow as tf
from tensorflow.keras.models import load_model
import numpy as np
import socket
import time
# Load the trained model
model = load_model('model_150epochs_large')

HOST = '192.168.25.37'
PORT = 65432  
cap = cv2.VideoCapture(0)

def preprocess_frame(img):
    img = cv2.resize(img, (240, 240))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  
    img = img / 255.0
    img = np.expand_dims(img, axis=0)  
    return img

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

while True:
 
    ret, frame = cap.read()

    if not ret:
        break

    processed_frame = preprocess_frame(frame)
    print(processed_frame.shape)

    prediction = model.predict(processed_frame)
    print(prediction)

    float_prediction = "{:.3f}".format(float(prediction[0][0]))

    print(float_prediction)

    cv2.putText(frame, f'Steering: {float_prediction}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    cv2.imshow('Webcam Feed', frame)

    data_to_send = str(float_prediction)

    client_socket.sendall(data_to_send.encode())
    
    time.sleep(0.1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

client_socket.close()
cap.release()
cv2.destroyAllWindows()

