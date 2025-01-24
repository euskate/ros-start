import ollama
import requests
import json
from PIL import Image
import base64
from io import BytesIO

# 테스트할 이미지 파일 경로
image_path = "./car.avif"  # 이미지 파일 경로를 입력하세요

# 사용자 질문
question = "이 이미지에 대해 한글로 설명해주세요."

def encode_image_to_base64(image_path):
    """이미지 파일을 Base64 문자열로 인코딩합니다."""
    with open(image_path, "rb") as image_file:
        encoded_string = base64.b64encode(image_file.read()).decode('utf-8')
        return encoded_string

try:
    base64_image = encode_image_to_base64(image_path)
    response = ollama.chat(
        model="llava:7b",
        messages=[
            {
                'role': 'user',
                'content': question,
                'images': [base64_image]
            }
        ]
    )
    print(response['message']['content'])

except FileNotFoundError:
    print(f"오류: 이미지 파일을 찾을 수 없습니다: {image_path}")
except Exception as e:
    print(f"예상치 못한 오류 발생: {e}")