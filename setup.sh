
#!/bin/bash

set -e  # detener si hay error

# Definir rutas
VISION_DIR="$HOME/vision"
MODELS_DIR="$VISION_DIR/models"
FACES_DIR="$VISION_DIR/faces_detected"

echo "--- CONFIGURANDO ENTORNO DE VISIÓN PARA TAKESHI ---"

echo "Creando estructura de carpetas..."
mkdir -p "$MODELS_DIR"
mkdir -p "$FACES_DIR"

echo "Descargando modelos en $MODELS_DIR..."

# YuNet (face detection)
curl -L https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx \
-o "$MODELS_DIR/face_detection_yunet_2023mar.onnx"

# SFace (face recognition)
curl -L https://github.com/opencv/opencv_zoo/raw/main/models/face_recognition_sface/face_recognition_sface_2021dec.onnx \
-o "$MODELS_DIR/face_recognition_sface_2021dec.onnx"

echo "--- VERIFICACIÓN ---"
# Listar los archivos y mostrar su tamaño para confirmar que no son 0 KB
ls -lh "$MODELS_DIR"

echo "Proceso finalizado correctamente."
