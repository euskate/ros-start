import requests
import base64
import json
# Ollama API 엔드포인트 (기본값)
OLLAMA_API_URL = "http://localhost:11434/api/chat"

# 테스트할 이미지 파일 경로
image_path = "./car.png "   # 이미지 파일 경로를 입력하세요

# 모델 이름
model_name = "llava:7b"

# 사용자 프롬프트
prompt = "이 이미지에 대해 한국어로 설명하세요"

def encode_image_to_base64(image_path):
    """이미지 파일을 Base64 문자열로 인코딩합니다."""
    with open(image_path, "rb") as image_file:
        encoded_string = base64.b64encode(image_file.read()).decode('utf-8')
        return encoded_string

def get_ollama_response(model, prompt, images=None):
    """Ollama API에 요청을 보내고 응답을 받습니다."""
    headers = {'Content-Type': 'application/json'}
    data = {
        "model": model,
        "messages": [
            {
                "role": "user",
                "content": prompt,
                "images": images if images else []
            }
        ],
        "stream": False  # 스트리밍 응답을 받지 않도록 설정
    }
    response = requests.post(OLLAMA_API_URL, headers=headers, data=json.dumps(data))
    if response.status_code == 200:
        return response.json()['message']['content']
    else:
        print(f"오류 발생: {response.status_code}")
        print(response.text)
        return None

if __name__ == "__main__":
    try:
        # 이미지 Base64 인코딩
        base64_image = encode_image_to_base64(image_path)

        # Ollama API에 요청
        response = get_ollama_response(model_name, prompt, images=[base64_image])

        if response:
            print("Ollama 응답:")
            print(response)

    except FileNotFoundError:
        print(f"오류: 이미지 파일을 찾을 수 없습니다: {image_path}")
    except requests.exceptions.ConnectionError:
        print(f"오류: Ollama API 서버에 연결할 수 없습니다. 서버가 실행 중인지 확인하세요 ({OLLAMA_API_URL}).")
    except Exception as e:
        print(f"예상치 못한 오류 발생: {e}")