from ultralytics import YOLO

def main():
    # 1. Load the pre-trained Medium model
    model = YOLO('yolo11m.pt')

    # 2. Train it on your Roboflow dataset
    model.train(
        data=r'C:\Users\z5406189\Thesis_C\Custom_Dataset_Two_Sessions_Combined\data.yaml', 
        epochs=50, 
        imgsz=640,
        batch=16,
        patience=20,
        name='YOLO11m_Custom_Combined_Data_Sessions',
        device=0
    )

if __name__ == '__main__':
    main()