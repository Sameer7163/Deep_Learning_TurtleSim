from ultralytics import YOLO

# Load a model
model = YOLO("yolo11n.yaml")  # build a new model from YAML


# Train the model using mars rock detection dataset and YOLO11 and store the results in folder runs/train
results = model.train(data="data.yaml", epochs=100, imgsz=640,batch=4)

