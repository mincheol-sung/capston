#!/usr/bin/env python3
"""YOLO11n -> OpenVINO IR 변환 (NUC에서 1회 실행).

NUC(Core 5 210H, NVIDIA/NPU 없음)에서는 OpenVINO CPU 또는 Intel iGPU로 추론한다.

준비:
  pip install ultralytics openvino
실행:
  python3 export_yolo_openvino.py                 # yolo11n, 640, FP16
  python3 export_yolo_openvino.py --model yolo11s.pt --imgsz 480
  python3 export_yolo_openvino.py --model yolo11n-seg.pt --imgsz 640
결과:
  yolo11n_openvino_model/  (이 폴더를 sensor_fusion 파라미터 yolo_model_path 로 지정)

성능 확인(선택):
  yolo benchmark model=yolo11n.pt format=openvino imgsz=640
"""
import argparse


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='yolo11n.pt', help='기본 가중치(.pt)')
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--half', action='store_true', default=True,
                    help='FP16 export')
    ap.add_argument('--int8', action='store_true',
                    help='INT8 양자화(더 빠름, 약간의 정확도 손실; calib 데이터 필요)')
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.model)
    path = model.export(
        format='openvino',
        nms=False,
        imgsz=args.imgsz,
        half=(args.half and not args.int8),
        int8=args.int8,
    )
    print(f'\nOpenVINO 모델 생성됨: {path}')
    print('sensor_fusion 실행 시:  -p yolo_model_path:=<위 경로>')


if __name__ == '__main__':
    main()
