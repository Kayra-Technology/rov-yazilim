from ultralytics import YOLO

model = YOLO('yolo11m.pt')

sonuc = model.predict(source=0, show=True, save=True)