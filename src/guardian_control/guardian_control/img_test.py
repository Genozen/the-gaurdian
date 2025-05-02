from ultralytics import YOLO

# Load a YOLO11n PyTorch model
# model = YOLO("yolo11n.pt")
# model = YOLO("rtdetr-l.pt")
model = YOLO("fire_epoch5_batch_4_imgsize_640.pt")

# Export the model to TensorRT
# model.export(format="engine")  # creates 'yolo11n.engine'
model.export(format="engine", device="0")   # should now build the .engine

# Load the exported TensorRT model
trt_model = YOLO("fire1.engine")
# trt_model = YOLO("rtdetr-l.engine")

# Run inference
results = trt_model("https://ultralytics.com/images/bus.jpg") 