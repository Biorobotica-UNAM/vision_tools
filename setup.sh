#!/bin/bash

set -e  # detener si hay error

# Definir rutas
VISION_DIR="$HOME/vision"
MODELS_DIR="$VISION_DIR/models"
FACES_DIR="$VISION_DIR/faces_detected"

echo "Creando estructura de carpetas..."

mkdir -p "$MODELS_DIR"
mkdir -p "$FACES_DIR"

echo "Descargando modelos..."

# YuNet (face detection)
curl -L -o "$MODELS_DIR/face_detection_yunet_2023mar.onnx" \
https://raw.githubusercontent.com/opencv/opencv_zoo/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx

# SFace (face recognition)
curl -L -o "$MODELS_DIR/face_recognition_sface_2021dec.onnx" \
https://raw.githubusercontent.com/opencv/opencv_zoo/main/models/face_recognition_sface/face_recognition_sface_2021dec.onnx

echo "Setup completado!"
echo "Estructura creada en: $VISION_DIR"
